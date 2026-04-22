/*
 * 模块：遥控输入处理
 * 职责：完成遥控通道解码、解锁判定、失控保护和摇杆功能触发
 * 说明：保持通道映射、解锁条件和失控保护流程不变。
 */
#include "sysconfig.h"
#include "Ano_Parameter.h"
#include "Ano_RC.h"
#include "Ano_Math.h"
#include "Drv_icm20602.h"
#include "Ano_MagProcess.h"
#include "Drv_led.h"
#include "Drv_RcIn.h"
#include "Ano_DT.h"
#include "Ano_Sensor_Basic.h"
#include "Ano_LED.h"

//摇杆触发值，摇杆值范围为+-500，超过300属于触发范围
#define UN_YAW_VALUE  300
#define UN_THR_VALUE  300
#define UN_PIT_VALUE  300
#define UN_ROL_VALUE  300

/* 当前接收机输入模式。 */
static u8 s_rc_input_mode;
/* 遥控输入初始化 */
void Remote_Control_Init()
{
	//
	s_rc_input_mode = Ano_Parame.set.pwmInMode;
	//
	if(s_rc_input_mode == SBUS)
	{
		Drv_RcSbus_Init();
	}
	else
	{
		Drv_RcPpm_Init();
//		PWM_IN_Init(RC_IN_MODE);
	}
}

/* 遥控通道看门狗计数。 */
static u16 s_channel_watchdog_cnt[10];

/* 对外共享的遥控输入状态。 */
u8 chn_en_bit = 0;
/* 喂通道看门狗 */
void ch_watch_dog_feed(u8 ch_n)
{
	ch_n = LIMIT(ch_n,0,7);
	s_channel_watchdog_cnt[ch_n] = 0;
}

static void RC_ChannelWatchdogTask(u8 dT_ms) // 如果是 PPM/SBUS 模式，也只检测前 8 通道
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
//			Rc_Pwm_In[i] = 0;  //把捕获值复位
//			Rc_Ppm_In[i] = 0;
//			Rc_Sbus_In[i] = 0;
		}
	}
}

u16 signal_intensity;
s16 CH_N[CH_NUM] = {0,0,0,0};

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
	if( flag.power_state <=2 && para_sta.save_trig == 0)//只有电池电压非最低并且没有操作flash时，才允许进行解锁
	{
		if(sens_hd_check.acc_ok && sens_hd_check.gyro_ok)
		{
			if(sens_hd_check.baro_ok)
			{
				if(flag.sensor_imu_ok  )//imu传感器正常时，才允许解锁
				{
					flag.unlock_err = 0;	//允许解锁标志位

				}
				else
				{
					flag.unlock_err = 1;//imu异常，不允许解锁

				}
			}
			else
			{
				LED_STA.errBaro = 1;
				flag.unlock_err = 2;//气压计异常，不允许解锁。
			}
		}
		else
		{
			LED_STA.errMpu = 1;
			flag.unlock_err = 3;//惯性传感器异常，不允许解锁。
		}
	}
	else
	{
		flag.unlock_err = 4;//电池电压异常，不允许解锁
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
		flag.thr_low = 0;//油门非低
	}
	else
	{
		flag.thr_low = 1;//油门拉低
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

void RC_duty_task(u8 dT_ms) //建议2ms调用一次
{
	if(flag.start_ok)	
	{
		/////////////获得通道数据////////////////////////
//		if(RC_IN_MODE == PWM)
//		{
//			for(u8 i=0;i<CH_NUM;i++)
//			{
//				if(chn_en_bit & (1<<i))//(Rc_Pwm_In[i]!=0)//该通道有值，==0说明该通道未插线（PWM）
//				{
//					//CH_N[]+1500为上位机显示通道值
//					CH_N[i] = 1.25f *((s16)Rc_Pwm_In[i] - 1500); //1100 -- 1900us,处理成大约+-500摇杆量

//				}
//				else
//				{
//					CH_N[i] = 0;
//				}
//				CH_N[i] = LIMIT(CH_N[i],-500,500);//限制到+—500
//			}
//		}
//		else if(RC_IN_MODE == PPM)
		if(s_rc_input_mode == PPM || s_rc_input_mode == PWM)
		{
			for(u8 i=0;i<CH_NUM;i++)
			{
				if(chn_en_bit & (1<<i))//(Rc_Ppm_In[i]!=0)//该通道有值
				{
					//CH_N[]+1500为上位机显示通道值
					CH_N[i] = ((s16)RC_PPM.Captures[i] - 1500); //1000 -- 2000us,处理成大约+-500摇杆量
				}
				else
				{
					CH_N[i] = 0;
				}
				CH_N[i] = LIMIT(CH_N[i],-500,500);//限制到+—500
			}		
		}
		else//sbus
		{
			for(u8 i=0;i<CH_NUM;i++)
			{
				if(chn_en_bit & (1<<i))//该通道有值
				{
					//CH_N[]+1500为上位机显示通道值
					CH_N[i] = 0.65f *((s16)Rc_Sbus_In[i] - 1024); //248 --1024 --1800,处理成大约+-500摇杆量
				}
				else
				{
					CH_N[i] = 0;
				}
				CH_N[i] = LIMIT(CH_N[i],-500,500);//限制到+—500
			}					
		}

		///////////////////////////////////////////////
		//解锁监测	
		RC_UnlockTask(dT_ms);
		//摇杆触发功能监测
		RC_StickFunctionTask(dT_ms);	
		//通道看门狗
		RC_ChannelWatchdogTask(dT_ms);

		//失控保护检查
		fail_safe_check(dT_ms);//3ms


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
			flag.auto_take_off_land = AUTO_LAND; //如果解锁，自动降落标记置位
		}
		else
		{
			flag.rc_loss_back_home = 1;
		}
		
	}
}

void fail_safe_check(u8 dT_ms) //dT秒调用一次
{
	static u16 cnt;
	static s8 cnt2;
	
	cnt += dT_ms;
	if(cnt >= 500) //500*dT 秒
	{
		cnt=0;
		if((chn_en_bit & 0x0F) != 0x0F || flag.chn_failsafe ) //前4通道有任意一通道无信号或者受到接收机失控保护信号
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
			
			flag.rc_loss = 1; //认为丢失遥控信号
			
			LED_STA.noRc = 1;
			
			RC_FailSafeApply();


				
		}
		else if(cnt2<=-2) //认为信号正常
		{
			cnt2 = 0;
			
			if(flag.rc_loss)
			{
				flag.rc_loss = 0;
				LED_STA.noRc = 0;
				
					if(flag.taking_off)
					flag.auto_take_off_land = AUTO_TAKE_OFF_FINISH; //解除下降
			}
			
		}
		
		signal_intensity=0; //累计接收次数
	}
	
	
}

/* 摇杆组合触发判定 */
static void RC_StickFunctionCheck(u8 dT_ms,_stick_f_c_st *sv,u8 times_n,u16 reset_time_ms,u8 en,u8 trig_val,u8 *trig)
{
	if(en)
	{
		sv->s_cnt = 0; //清除计时
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
		/////
		sv->s_cnt += dT_ms;
		if(sv->s_cnt>reset_time_ms)
		{
			sv->s_now_times = 1; //清除记录次数
		}
	}

	if(sv->s_now_times> times_n)
	{
		*trig = trig_val;            //触发功能标记
		sv->s_now_times = 0;
	}

}
/* 摇杆长按触发判定 */
static void RC_StickFunctionCheckLongPress(u8 dT_ms,u16 *time_cnt,u16 longpress_time_ms,u8 en,u8 trig_val,u8 *trig)
{
	//dT_ms：调用间隔时间
	//time_cnt：积分时间
	//longpress_time_ms：阈值时间，超过这个时间则为满足条件
	//en：摇杆状态是否满足
	//trig_val：满足后的触发值
	//trig：指向需要触发的寄存器
	if(en)//如果满足摇杆条件，则进行时间积分
	{
		if(*time_cnt!=0)
		{
			*time_cnt+=dT_ms;
		}
	}
	else//不满足条件，积分恢复1
	{
		*time_cnt=1;
	}
	//时间积分满足时间阈值，则触发标记
	if(*time_cnt>=longpress_time_ms)
	{
		*trig = trig_val;            //触发功能标记
		*time_cnt = 0;
	}

}

/* 摇杆组合功能处理 */
static void RC_StickFunctionTask(u8 dT_ms)
{
	//////////////状态监测
	//未解锁才允许检测摇杆功能
	if(flag.unlock_sta == 0)
	{
		//油门低，则继续
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
		
			///////////////
		//触发陀螺仪校准
		RC_StickFunctionCheckLongPress(dT_ms,&s_cali_gyro_hold_cnt,1000,s_stick_fun_gyro_cali,1,&sensor.gyr_CALIBRATE);
		//触发加速度计校准
		RC_StickFunctionCheckLongPress(dT_ms,&s_cali_acc_hold_cnt,1000,s_stick_fun_acc_cali,1,&sensor.acc_CALIBRATE);
		
		//触发罗盘校准
		RC_StickFunctionCheck(dT_ms,&s_cali_mag_state,5,1000,s_stick_fun_mag_cali,1,&mag.mag_CALIBRATE);

		
	}

	//////////////
}










