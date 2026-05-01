#include "FlightDataCal.h"
#include "Imu.h"
#include "Drv_icm20602.h"
#include "MagProcess.h"
#include "Drv_spl06.h"
#include "Drv_ak8975.h"
#include "MotionCal.h"
#include "Sensor_Basic.h"
#include "FlightCtrl.h"
#include "Drv_led.h"
#include "OF.h"
#include "Drv_Laser.h"
/*
 * 文件名称: FlightDataCal.c
 * 所属模块: Algorithm / Estimation
 *
 * 功能描述:
 *   传感器数据调度与融合中枢，串联硬件驱动和控制算法：
 *   1) Fc_Sensor_Get()       —— 1ms 周期读取 IMU，20ms 读取磁力计和气压计
 *   2) IMU_Update_Task()     —— 调用 Imu.c 执行姿态解算
 *   3) Mag_Update_Task()     —— 磁力计数据处理
 *   4) WCZ_Acc_Get_Task()    —— 提取世界坐标系垂直加速度
 *   5) WCZ_Fus_Task()        —— 高度融合：气压计 + ToF + 加速度积分
 *
 * 高度融合流程:
 *   ┌──────────┐     ┌──────────┐     ┌──────────────┐
 *   │ 气压计   │────▶│ 参考状态 │────▶│ 积分修正滤波 │──▶ wcz_hei_fus.out (融合高度)
 *   │ baro_h   │     │ 机 (0→1→2)│    │ 器           │    wcz_spe_fus.out (融合速度)
 *   └──────────┘     └──────────┘     └──────────────┘
 *                         ▲                  ▲
 *                    ToF 高度切换         加速度积分
 *
 *   气压计参考状态机：
 *     状态 0：等待传感器就绪
 *     状态 1：采集参考气压值（地面零点标定）
 *     状态 2：正常融合输出
 *
 * 架构位置:
 *   Fc_Sensor_Get() 由 1ms 硬件定时器中断间接调用。
 *   其余函数由 Scheduler 按各自周期调用（5ms/10ms/20ms）。
 *   输出 wcz_hei_fus / wcz_spe_fus 供 AltCtrl 使用。
 *
 * 教学提示:
 *   - 高度融合使用互补滤波思路：长期信任气压计/ToF，短期信任加速度积分
 *   - ToF ↔ 气压计切换时通过 offset 保持输出连续，避免高度跳变
 *   - BARO_FIX 当前值为 0，温度补偿修正通路实际未启用
 */

/* ══════════════════════════════════════════════════════
 *  传感器读取 (1ms 周期)
 * ══════════════════════════════════════════════════════ */

/* 每 1ms 读取 IMU (加速度计+陀螺仪)，每 20ms 读取磁力计和气压计。 */
void Fc_Sensor_Get(void)
{
	static u8 cnt;
	if(flag.start_ok)
	{
		Drv_Icm20602_Read();

		cnt ++;
		cnt %= 20;
		if(cnt==0)
		{
			Drv_AK8975_Read();
			baro_height = (s32)Drv_Spl0601_Read();
		}
	}
}

/* ══════════════════════════════════════════════════════
 *  姿态更新
 * ══════════════════════════════════════════════════════ */

static u8 s_imu_reset_armed;

/*
 * 管理 IMU 对准状态，设置融合增益，调用姿态解算。
 * 锁定状态 → G_reset=1, 触发陀螺仪校准；解锁后清除复位标志。
 */
void IMU_Update_Task(u8 dT_ms)
{
	if(flag.unlock_sta )
	{
		imu_state.G_reset = imu_state.M_reset = 0;
		s_imu_reset_armed = 0;
	}
	else
	{
		if(s_imu_reset_armed == 0)
		{
			imu_state.G_reset = 1;
			sensor.gyr_CALIBRATE = 2;
			s_imu_reset_armed = 1;
		}
	}

	imu_state.gkp = 0.2f;
	imu_state.gki = 0.01f;
	imu_state.mkp = 0.1f;

	imu_state.M_fix_en = sens_hd_check.mag_ok;

	IMU_update(dT_ms *1e-3f, &imu_state, sensor.Gyro_rad, sensor.Acc_cmss, mag.val, &imu_data);
}

/* ══════════════════════════════════════════════════════
 *  磁力计处理
 * ══════════════════════════════════════════════════════ */

static s16 mag_val[3];

void Mag_Update_Task(u8 dT_ms)
{
	Mag_Get(mag_val);
	Mag_Data_Deal_Task(dT_ms,mag_val,imu_data.z_vec[Z],sensor.Gyro_deg[X],sensor.Gyro_deg[Z]);
}

/* ══════════════════════════════════════════════════════
 *  高度融合
 *
 *  气压计参考状态机 (s_baro_ref_state):
 *    0 → 等待 IMU 就绪，持续记录气压计零点 (baro_h_offset)
 *    1 → IMU 就绪，开始输出相对高度 ref_height_get_1
 *    2 → 正在起飞 / 飞行中
 *    降落后 2→0，重新锁定零点。
 *
 *  ToF / 光流切换 (s_tof_ref_ready):
 *    ToF 有效时记录气压-ToF 偏移并切换到 ToF 参考；
 *    ToF 无效时回退到气压计参考。
 *    通过 baro2tof_offset / tof2baro_offset 实现无跳变切换。
 * ══════════════════════════════════════════════════════ */

/* 外部可见 (DT.c 遥测使用) */
s32 baro_height;
u16 ref_tof_height;

/* 模块内部 */
static s32 baro_h_offset;           /* 气压计零点偏移 (地面时锁定) */
static s32 ref_height_get_1;        /* 气压计相对高度 */
static s32 ref_height_get_2;        /* ToF 相对高度 (含切换偏移) */
static s32 ref_height_used;         /* 最终参考高度 (送入 WCZ 融合) */
static s32 baro2tof_offset;         /* 气压计→ToF 切换时的高度差 */
static s32 tof2baro_offset;         /* ToF→气压计 切换时的高度差 */
static float baro_fix1,baro_fix2,baro_fix; /* 气压计修正量 (BARO_FIX 当前为 0, 均为零) */
static u8 wcz_f_pause;
static float wcz_acc_use;           /* 世界 Z 轴加速度低通滤波值 */

/* 最小周期调用：对世界 Z 轴加速度做一阶低通滤波 */
void WCZ_Acc_Get_Task()
{
	wcz_acc_use += 0.03f *(imu_data.w_acc[Z] - wcz_acc_use);
}

static u8 s_baro_ref_state, s_tof_ref_ready;

void WCZ_Fus_Task(u8 dT_ms)
{
	/* ── 气压计参考状态机 ── */
	if(flag.taking_off)
	{
		s_baro_ref_state = 2;
	}
	else
	{
		if(s_baro_ref_state == 2)
		{
			s_baro_ref_state = 0;
		}
		tof2baro_offset = 0;
	}

	if(s_baro_ref_state >= 1)
	{
		ref_height_get_1 = baro_height - baro_h_offset + baro_fix  + tof2baro_offset;
	}
	else
	{
		if(s_baro_ref_state == 0 )
		{
			baro_h_offset = baro_height;
			if(flag.sensor_imu_ok)
			{
				s_baro_ref_state = 1;
			}
		}
	}

	/* ── 气压计修正量 (BARO_FIX 当前为 0, 此段实际无效果) ── */
	if((flag.flying == 0) && flag.auto_take_off_land == AUTO_TAKE_OFF	)
	{
		wcz_f_pause = 1;

		baro_fix = 0;
	}
	else
	{
		wcz_f_pause = 0;

		if(flag.taking_off == 0)
		{
			baro_fix1 = 0;
			baro_fix2 = 0;

		}
		baro_fix2 = -BARO_FIX;

		baro_fix = baro_fix1 + baro_fix2 - BARO_FIX;
	}

	/* ── ToF / 光流高度切换 ── */
	if((sens_hd_check.of_df_ok || sens_hd_check.of_ok) && s_baro_ref_state)
	{
		if(switchs.tof_on || switchs.of_tof_on)
		{
			if(switchs.of_tof_on)
			{
				ref_tof_height = jsdata.valid_of_alt_cm ;
			}

			if(s_tof_ref_ready == 0)
			{
				baro2tof_offset = ref_height_get_1 - ref_tof_height ;
				s_tof_ref_ready = 1;
			}

			ref_height_get_2 = ref_tof_height + baro2tof_offset;
			ref_height_used = ref_height_get_2;

			tof2baro_offset += 0.5f *((ref_height_get_2 - ref_height_get_1) - tof2baro_offset);
		}
		else
		{
			s_tof_ref_ready = 0;
			ref_height_used = ref_height_get_1 ;
		}
	}
	else
	{
		ref_height_used = ref_height_get_1;
	}

	/* ── 世界 Z 高度融合 ── */
	WCZ_Data_Calc(dT_ms,wcz_f_pause,(s32)wcz_acc_use,(s32)(ref_height_used));
}
