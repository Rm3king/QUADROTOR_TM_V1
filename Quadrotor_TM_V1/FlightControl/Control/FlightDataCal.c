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
 * 模块：飞行数据计算
 * 职责：调度传感器读取、姿态更新和高度相关融合计算。
 * 约束：保持 1ms 任务调用顺序与传感器参与条件不变。
 */
/* 1ms 周期读取 IMU 等传感器原始数据。 */
void Fc_Sensor_Get(void)
{
	static u8 cnt;
	if(flag.start_ok)
	{
		/*读取陀螺仪加速度计数据*/
		Drv_Icm20602_Read();
		
		cnt ++;
		cnt %= 20;
		if(cnt==0)
		{
			/*读取电子罗盘磁力计数据*/
			Drv_AK8975_Read();
			/*读取气压计数据*/
			baro_height = (s32)Drv_Spl0601_Read();
		}
	}	
}
static u8 s_imu_reset_armed;
void IMU_Update_Task(u8 dT_ms)
{
	
			/*如果准备飞行，复位重力复位标记和磁力计复位标记*/
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
				
				imu_state.M_fix_en = sens_hd_check.mag_ok;		//磁力计修正使能
	
				
				/*姿态计算，更新，融合*/
				IMU_update(dT_ms *1e-3f, &imu_state, sensor.Gyro_rad, sensor.Acc_cmss, mag.val, &imu_data);
}
static s16 mag_val[3];
void Mag_Update_Task(u8 dT_ms)
{
	Mag_Get(mag_val);
	
	Mag_Data_Deal_Task(dT_ms,mag_val,imu_data.z_vec[Z],sensor.Gyro_deg[X],sensor.Gyro_deg[Z]);
	
}
s32 baro_height,baro_h_offset,ref_height_get_1,ref_height_get_2,ref_height_used;
s32 baro2tof_offset,tof2baro_offset;
float baro_fix1,baro_fix2,baro_fix;
static u8 wcz_f_pause;
float wcz_acc_use;			
void WCZ_Acc_Get_Task()//最小周期
{
	wcz_acc_use += 0.03f *(imu_data.w_acc[Z] - wcz_acc_use);
}
u16 ref_tof_height;
static u8 s_baro_ref_state, s_tof_ref_ready;
void WCZ_Fus_Task(u8 dT_ms)
{
	
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
	
	if(s_baro_ref_state >= 1)//(flag.taking_off)
	{
		ref_height_get_1 = baro_height - baro_h_offset + baro_fix  + tof2baro_offset;//气压计相对高度，切换点跟随TOF
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
	
	if((sens_hd_check.of_df_ok || sens_hd_check.of_ok) && s_baro_ref_state) //TOF或者OF硬件正常，且气压计记录相对值以后
	{
		if(switchs.tof_on || switchs.of_tof_on) //TOF数据有效
		{
			if(switchs.of_tof_on) //光流带TOF，光流优先
			{
				ref_tof_height = jsdata.valid_of_alt_cm ;
			}
						
			
			//
			if(s_tof_ref_ready == 0)
			{
				baro2tof_offset = ref_height_get_1 - ref_tof_height ; //记录TOF切换点		
				s_tof_ref_ready = 1;
			}
			//
			ref_height_get_2 = ref_tof_height + baro2tof_offset;//TOF参考高度，切换点跟随气压计				
			ref_height_used = ref_height_get_2;
			
			tof2baro_offset += 0.5f *((ref_height_get_2 - ref_height_get_1) - tof2baro_offset);//记录气压计切换点，气压计波动大，稍微滤波一下
			
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
	
	//世界z方向高度信息融合
	WCZ_Data_Calc(dT_ms,wcz_f_pause,(s32)wcz_acc_use,(s32)(ref_height_used));
}
