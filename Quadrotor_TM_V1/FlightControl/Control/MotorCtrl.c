#include "MotorCtrl.h"
#include "Math.h"
#include "Parameter.h"
#include "Drv_icm20602.h"
#include "Drv_spl06.h"
#include "Imu.h"
#include "Drv_PwmOut.h"

#include "MotionCal.h"
#include "Filter.h"
#include "Navigate.h"
#include "RC.h"

/*
 * 模块：电机控制
 * 职责：执行解锁预转、混控合成与 PWM 输出前限幅
 * 说明：保持原有混控关系和输出顺序，仅整理文件说明。
 */

/*
四轴：
      机头
   m2     m1
     \   /
      \ /
      / \
     /   \
   m3     m4
      屁股
*/
s16 motor[MOTORSNUM];
s16 motor_step[MOTORSNUM];
//float motor_lpf[MOTORSNUM];

static u16 motor_prepara_cnt;
_mc_st mc;
u16 IDLING;//10*g_fc_param.set.idle_speed_pwm  //200
/* 电机控制任务 */
void Motor_Ctrl_Task(u8 dT_ms)
{
	u8 i;
	
//	if(flag.taking_off)
//	{
//		flag.motor_preparation = 1;
//		motor_prepara_cnt = 0;			
//	}
	
	if(flag.unlock_sta)
	{		
		IDLING = 10*LIMIT(g_fc_param.set.idle_speed_pwm,0,30);
		
		if(flag.motor_preparation == 0)
		{
			motor_prepara_cnt += dT_ms;
			
			if(flag.motor_preparation == 0)
			{			
				if(motor_prepara_cnt<300)
				{
					motor[m1] = IDLING;
				}
				else if(motor_prepara_cnt<600)
				{
					motor[m2] = IDLING;
				}
				else if(motor_prepara_cnt<900)
				{
					motor[m3] = IDLING;
				}	
				else if(motor_prepara_cnt<1200)
				{	
					motor[m4] = IDLING;
				}
				else
				{
					flag.motor_preparation = 1;
					motor_prepara_cnt = 0;
				}
			}
			
		}	
	}
	else
	{
		flag.motor_preparation = 0;
	}
	

			
	if(flag.motor_preparation == 1)
	{	
		motor_step[m1] = mc.ct_val_thr  +mc.ct_val_yaw -mc.ct_val_rol +mc.ct_val_pit;
		motor_step[m2] = mc.ct_val_thr  -mc.ct_val_yaw +mc.ct_val_rol +mc.ct_val_pit;
		motor_step[m3] = mc.ct_val_thr  +mc.ct_val_yaw +mc.ct_val_rol -mc.ct_val_pit;
		motor_step[m4] = mc.ct_val_thr  -mc.ct_val_yaw -mc.ct_val_rol -mc.ct_val_pit;
		
	
		for(i=0;i<MOTORSNUM;i++)
		{	
			motor_step[i] = LIMIT(motor_step[i],IDLING,1000);
//			motor_lpf[i] += 0.5f *(motor_step[i] - motor_lpf[i]) ;		
			
		}
		


	}
	
	for(i=0;i<MOTORSNUM;i++)
	{
		if(flag.unlock_sta)
		{
			if(flag.motor_preparation == 1)
			{
				motor[i] = LIMIT(motor_step[i],IDLING,999);
			}
	
		}
		else
		{		
			motor[i] = 0;
		}	

	}

	//配置输出
	for(u8 i =0;i<4;i++)
	{
		Drv_MotorPWMSet(i,motor[i]);
	}

//#define Cali_Set_ESC
#ifdef Cali_Set_ESC
	//配置输出
	for(u8 i =0;i<4;i++)
	{
		motor[i] = CH_N[CH_THR]+500;
		Drv_MotorPWMSet(i,motor[i]);
	}
	
#endif
	//test
//	Drv_MotorPWMSet(4,200);

}



