/*
 * 文件名称: RC.c
 * 所属模块: Application / Service
 *
 * 功能描述:
 *   遥控器输入处理模块，完成从原始通道值到飞控指令的全部转换：
 *   1) RC_duty_task()        -- 主任务：通道归一化 + 解锁判定 + 摇杆功能检测
 *   2) fail_safe_check()     -- 失控保护：检测遥控信号丢失并触发自动降落
 *   3) RC_GetChannel()       -- 公开接口：读取指定通道归一化值
 *   4) RC_GetAllChannels()   -- 公开接口：批量读取所有通道
 *   5) RC_GetChannelEnableMask() -- 公开接口：通道有效位图
 *
 * 解锁流程:
 *   油门最低 + 偏航右满舵 → 持续 1 秒 → 解锁 (flag.unlock_sta = 1)
 *   油门最低 + 偏航左满舵 → 持续 1 秒 → 上锁
 *
 * 通道映射:
 *   CH1=Roll, CH2=Pitch, CH3=Throttle, CH4=Yaw, CH5-8=AUX1-4
 *   归一化范围：±500（中位为 0）
 *
 * 架构位置:
 *   由 Scheduler 按 6ms 周期调用。
 *   输出通过 RC_GetChannel() 接口被 FlightCtrl、AttCtrl 等模块读取。
 *
 * 教学提示:
 *   - 摇杆死区 (UN_xxx_VALUE=300) 防止中位漂移误触发功能
 *   - 失控保护是飞行安全的最后防线，必须在 RC 层实现
 *   - CH_N 数组已封装为 static，外部统一通过 getter 访问
 */
#include "sysconfig.h"
#include "Parameter.h"
#include "RC.h"
#include "Math.h"
#include "Drv_icm20602.h"
#include "MagProcess.h"
#include "Drv_led.h"
#include "Drv_RcIn.h"
#include "DT.h"
#include "Sensor_Basic.h"
#include "LED.h"
/* 摇杆触发阈值。摇杆范围约为 +/-500，超过 300 视为有效触发。 */
#define UN_YAW_VALUE  300
#define UN_THR_VALUE  300
#define UN_PIT_VALUE  300
#define UN_ROL_VALUE  300
/* 当前接收机输入模式。 */
static u8 s_rc_input_mode;
/* 遥控输入初始化 */
void Remote_Control_Init()
{
	s_rc_input_mode = g_fc_param.set.pwmInMode;
	if(s_rc_input_mode == SBUS)
	{
		Drv_RcSbus_Init();
	}
	else
	{
		Drv_RcPpm_Init();
	}
}
/* 遥控通道看门狗计数。 */
static u16 s_channel_watchdog_cnt[10];
static u8 chn_en_bit = 0;
/* 喂通道看门狗 */
void ch_watch_dog_feed(u8 ch_n)
{
	ch_n = LIMIT(ch_n,0,7);
	s_channel_watchdog_cnt[ch_n] = 0;
}
static void RC_ChannelWatchdogTask(u8 dT_ms)
{
	for(u8 i = 0;i<8;i++)
	{
		if(s_channel_watchdog_cnt[i]<500)
		{
			s_channel_watchdog_cnt[i] += dT_ms;
			chn_en_bit |= 0x01<<i;
		}
		else
		{
			chn_en_bit &= ~(0x01<<i);
		}
	}
}
static s16 CH_N[CH_NUM] = {0,0,0,0};
/* 文件内部的解锁与摇杆功能状态。 */
static _stick_f_lp_st s_unlock_hold_cnt;
static u8 s_unlock_gesture_active;
static u16 s_unlock_hold_time_ms = 200;
static _stick_f_lp_st s_cali_gyro_hold_cnt;
static _stick_f_lp_st s_cali_acc_hold_cnt;
static _stick_f_c_st s_cali_mag_state;
static u8 s_stick_fun_gyro_cali;
static u8 s_stick_fun_acc_cali;
static u8 s_stick_fun_mag_cali;
static void RC_ChannelWatchdogTask(u8 dT_ms);
static void RC_StickFunctionCheck(u8 dT_ms,_stick_f_c_st *sv,u8 times_n,u16 reset_time_ms,u8 en,u8 trig_val,u8 *trig);
static void RC_StickFunctionCheckLongPress(u8 dT_ms,u16 *time_cnt,u16 longpress_time_ms,u8 en,u8 trig_val,u8 *trig);
static void RC_StickFunctionTask(u8 dT_ms);
static void RC_UnlockTask(u8 dT_ms);
static void RC_UpdateUnlockErrorState(void)
{
	if( flag.power_state <=2 && g_param_state.save_trig == 0)
	{
		if(sens_hd_check.acc_ok && sens_hd_check.gyro_ok)
		{
			if(sens_hd_check.baro_ok)
			{
				if(flag.sensor_imu_ok  )
				{
					flag.unlock_err = 0;
				}
				else
				{
					flag.unlock_err = 1;
				}
			}
			else
			{
				LED_STA.errBaro = 1;
				flag.unlock_err = 2;
			}
		}
		else
		{
			LED_STA.errMpu = 1;
			flag.unlock_err = 3;
		}
	}
	else
	{
		flag.unlock_err = 4;
	}
}
static void RC_SyncUnlockCommand(void)
{
	if(flag.unlock_sta == 0)
	{
		if(flag.unlock_cmd != 0)
		{
			if(flag.unlock_err == 0)
			{
				flag.unlock_sta = flag.unlock_cmd;
				ANO_DT_SendString("Unlock OK!");
			}
			else
			{
				flag.unlock_cmd = 0;
				if(flag.unlock_err == 4)
				{
					ANO_DT_SendString("Power Low,Unlock Fail!");
				}
				else
				{
					ANO_DT_SendString("Unlock Fail!");
				}
			}
		}
	}
	else
	{
		if(flag.unlock_cmd == 0)
		{
			ANO_DT_SendString(" FC Output Locked! ");
		}
		flag.unlock_sta = flag.unlock_cmd;
	}
}
static void RC_UpdateLockGesture(u8 dT_ms)
{
	if(CH_N[CH_THR] < -UN_THR_VALUE  )
	{
		if(ABS(CH_N[CH_YAW])>0.1f*UN_YAW_VALUE && CH_N[CH_PIT]< -0.1f*UN_PIT_VALUE)
		{
			if(flag.locking == 0)
			{
				flag.locking = 1;
			}
		}
		else
		{
			flag.locking = 0;
		}
		if(CH_N[CH_PIT]<-UN_PIT_VALUE && CH_N[CH_ROL]>UN_ROL_VALUE && CH_N[CH_YAW]<-UN_YAW_VALUE)
		{
			s_unlock_gesture_active = 1;
			flag.locking = 2;
		}
		else if(CH_N[CH_PIT]<-UN_PIT_VALUE && CH_N[CH_ROL]<-UN_ROL_VALUE && CH_N[CH_YAW]>UN_YAW_VALUE)
		{
			s_unlock_gesture_active = 1;
			flag.locking = 2;
		}
		else
		{
			s_unlock_gesture_active = 0;
		}

		u8 unlock_cmd_target = 0;
		if(flag.unlock_sta)
		{
			unlock_cmd_target = 0;
			s_unlock_hold_time_ms = 1000;
		}
		else
		{
			unlock_cmd_target = 2;
			s_unlock_hold_time_ms = 200;
		}
		RC_StickFunctionCheckLongPress(dT_ms,&s_unlock_hold_cnt,s_unlock_hold_time_ms,s_unlock_gesture_active,unlock_cmd_target,&flag.unlock_cmd);
	}
	else
	{
		flag.locking = 0;
		if(flag.unlock_cmd == 2)
		{
			flag.unlock_cmd = 1;
		}
	}
}
static void RC_UpdateThrottleLowState(void)
{
	if(CH_N[CH_THR]>-350)
	{
		flag.thr_low = 0;
	}
	else
	{
		flag.thr_low = 1;
	}
}
/* 解锁与上锁状态更新 */
static void RC_UnlockTask(u8 dT_ms)
{
	RC_UpdateUnlockErrorState();
	RC_SyncUnlockCommand();
	RC_UpdateLockGesture(dT_ms);
	RC_UpdateThrottleLowState();
}
void RC_duty_task(u8 dT_ms)
{
	if(flag.start_ok)
	{
		/* PPM/PWM 通道值读取 */
		if(s_rc_input_mode == PPM || s_rc_input_mode == PWM)
		{
			for(u8 i=0;i<CH_NUM;i++)
			{
				if(chn_en_bit & (1<<i))
				{
					CH_N[i] = ((s16)RC_PPM.Captures[i] - 1500);
				}
				else
				{
					CH_N[i] = 0;
				}
				CH_N[i] = LIMIT(CH_N[i],-500,500);
			}
		}
		else
		{
			for(u8 i=0;i<CH_NUM;i++)
			{
				if(chn_en_bit & (1<<i))
				{
					CH_N[i] = 0.65f *((s16)Rc_Sbus_In[i] - 1024);
				}
				else
				{
					CH_N[i] = 0;
				}
				CH_N[i] = LIMIT(CH_N[i],-500,500);
			}
		}
		RC_UnlockTask(dT_ms);
		RC_StickFunctionTask(dT_ms);
		RC_ChannelWatchdogTask(dT_ms);
		fail_safe_check(dT_ms);
	}
}
/* 执行失控保护输出覆盖 */
static void RC_FailSafeApply(void)
{
	for(u8 i = 0;i<4;i++)
	{
		CH_N[i] = 0;
	}
	if(CH_N[CH_THR]>0)
	{
		CH_N[CH_THR] = 0;
	}
	CH_N[CH_ROL] = 0;
	CH_N[CH_PIT] = 0;
	CH_N[CH_YAW] = 0;

	//切记不能给 CH_N[AUX1]赋值，否则可能导致死循环。（根据AUX1特殊值判断接收机failsafe信号）

	if(flag.unlock_sta)
	{
		if(switchs.gps_on ==0)
		{
			flag.auto_take_off_land = AUTO_LAND;
		}
		else
		{
			flag.rc_loss_back_home = 1;
		}

	}
}
void fail_safe_check(u8 dT_ms)
{
	static u16 cnt;
	static s8 cnt2;

	cnt += dT_ms;
	if(cnt >= 500)
	{
		cnt=0;
		if((chn_en_bit & 0x0F) != 0x0F || flag.chn_failsafe )
		{
			cnt2 ++;
		}
		else
		{
			cnt2 --;
		}

		if(cnt2>=2)
		{
			cnt2 = 0;

			flag.rc_loss = 1;

			LED_STA.noRc = 1;

			RC_FailSafeApply();

		}
		else if(cnt2<=-2)
		{
			cnt2 = 0;

			if(flag.rc_loss)
			{
				flag.rc_loss = 0;
				LED_STA.noRc = 0;

					if(flag.taking_off)
					flag.auto_take_off_land = AUTO_TAKE_OFF_FINISH;
			}

		}
	}


}
/* 摇杆组合触发判定 */
static void RC_StickFunctionCheck(u8 dT_ms,_stick_f_c_st *sv,u8 times_n,u16 reset_time_ms,u8 en,u8 trig_val,u8 *trig)
{
	if(en)
	{
		sv->s_cnt = 0;
		if(sv->s_state==0)
		{
			if(sv->s_now_times!=0)
			{
				sv->s_now_times++;
			}
			sv->s_state = 1;
		}
	}
	else
	{
		sv->s_state = 0;
		sv->s_cnt += dT_ms;
		if(sv->s_cnt>reset_time_ms)
		{
			sv->s_now_times = 1;
		}
	}
	if(sv->s_now_times> times_n)
	{
		*trig = trig_val;
		sv->s_now_times = 0;
	}
}
/* 摇杆长按触发判定 */
static void RC_StickFunctionCheckLongPress(u8 dT_ms,u16 *time_cnt,u16 longpress_time_ms,u8 en,u8 trig_val,u8 *trig)
{
	if(en)
	{
		if(*time_cnt!=0)
		{
			*time_cnt+=dT_ms;
		}
	}
	else
	{
		*time_cnt=1;
	}
	if(*time_cnt>=longpress_time_ms)
	{
		*trig = trig_val;
		*time_cnt = 0;
	}
}
/* 摇杆组合功能处理 */
static void RC_StickFunctionTask(u8 dT_ms)
{
	if(flag.unlock_sta == 0)
	{
		if(flag.thr_low)
		{
			if(CH_N[CH_PIT]<-350 && CH_N[CH_ROL]>350 && CH_N[CH_THR]<-350 && CH_N[CH_YAW]>350)
			{
				s_stick_fun_gyro_cali = s_stick_fun_acc_cali = 1;
			}
			else
			{
				s_stick_fun_gyro_cali = s_stick_fun_acc_cali = 0;
			}

			if(CH_N[CH_PIT]>350)
			{
				s_stick_fun_mag_cali = 1;
			}
			else if(CH_N[CH_PIT]<50)
			{
				s_stick_fun_mag_cali = 0;
			}
		}

		RC_StickFunctionCheckLongPress(dT_ms,&s_cali_gyro_hold_cnt,1000,s_stick_fun_gyro_cali,1,&sensor.gyr_CALIBRATE);
		RC_StickFunctionCheckLongPress(dT_ms,&s_cali_acc_hold_cnt,1000,s_stick_fun_acc_cali,1,&sensor.acc_CALIBRATE);

		RC_StickFunctionCheck(dT_ms,&s_cali_mag_state,5,1000,s_stick_fun_mag_cali,1,&mag.mag_CALIBRATE);

	}
}

/* ══════════════════════════════════════════════════════
 *  公开 getter 接口
 *  CH_N 和 chn_en_bit 为 static，外部模块通过以下函数只读访问。
 * ══════════════════════════════════════════════════════ */

s16 RC_GetChannel(u8 ch)
{
	if(ch >= CH_NUM) return 0;
	return CH_N[ch];
}

void RC_GetAllChannels(s16 *out, u8 count)
{
	u8 n = (count > CH_NUM) ? CH_NUM : count;
	for(u8 i = 0; i < n; i++)
	{
		out[i] = CH_N[i];
	}
}

u8 RC_GetChannelEnableMask(void)
{
	return chn_en_bit;
}
