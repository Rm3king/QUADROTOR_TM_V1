/*
 * 模块名称：Ano_Parameter
 * 模块职责：维护默认参数、参数镜像同步和延时保存流程。
 * 使用约束：本文件直接关联参数含义与存储时序，重构时不允许改变默认值语义和写入触发逻辑。
 */

//#include "Drv_w25qxx.h"
#include "Ano_Parameter.h"
#include "Drv_Paramter.h"
#include "Ano_FlightCtrl.h"
#include "Drv_led.h"
#include "Drv_icm20602.h"
#include "Ano_Sensor_Basic.h"
#include "Ano_LED.h"
#include "Ano_DT.h"


union Parameter Ano_Parame;
_parameter_state_st para_sta;

/* 注意：这里定义的是参数区默认值。修改代码后若未触发写入，存储区中的旧值不会自动更新。 */
/* 恢复默认 PID 参数。 */
void PID_Rest()
{
/* 姿态控制角速度环 PID 参数。 */
	Ano_Parame.set.pid_att_1level[ROL][KP] = 4.0f;
	Ano_Parame.set.pid_att_1level[ROL][KI] = 3.0f;
	Ano_Parame.set.pid_att_1level[ROL][KD] = 0.15f;
	
	Ano_Parame.set.pid_att_1level[PIT][KP] = 4.0f;
	Ano_Parame.set.pid_att_1level[PIT][KI] = 3.0f;
	Ano_Parame.set.pid_att_1level[PIT][KD] = 0.15f;
	
	Ano_Parame.set.pid_att_1level[YAW][KP] = 6.0f;
	Ano_Parame.set.pid_att_1level[YAW][KI] = 0.5f;
	Ano_Parame.set.pid_att_1level[YAW][KD] = 0.0f;
/* 姿态控制角度环 PID 参数。 */
	Ano_Parame.set.pid_att_2level[ROL][KP] = 7.0f;
	Ano_Parame.set.pid_att_2level[ROL][KI] = 0.0f;
	Ano_Parame.set.pid_att_2level[ROL][KD] = 0.00f;
	
	Ano_Parame.set.pid_att_2level[PIT][KP] = 7.0f;
	Ano_Parame.set.pid_att_2level[PIT][KI] = 0.0f;
	Ano_Parame.set.pid_att_2level[PIT][KD] = 0.00f;
	
	Ano_Parame.set.pid_att_2level[YAW][KP] = 5.0f;
	Ano_Parame.set.pid_att_2level[YAW][KI] = 0.0f;
	Ano_Parame.set.pid_att_2level[YAW][KD] = 0.5;
/* 高度控制速度环 PID 参数。 */
	Ano_Parame.set.pid_alt_1level[KP] = 2.0f;
	Ano_Parame.set.pid_alt_1level[KI] = 1.0f;
	Ano_Parame.set.pid_alt_1level[KD] = 0.05f;
/* 高度控制高度环 PID 参数。 */
	Ano_Parame.set.pid_alt_2level[KP] = 1.0f;
	Ano_Parame.set.pid_alt_2level[KI] = 0;
	Ano_Parame.set.pid_alt_2level[KD] = 0;
/* 位置控制速度环 PID 参数。 */
	Ano_Parame.set.pid_loc_1level[KP] = 0.15f;
	Ano_Parame.set.pid_loc_1level[KI] = 0.10f;
	Ano_Parame.set.pid_loc_1level[KD] = 0.00f;
/* 位置控制位置环 PID 参数。 */
	Ano_Parame.set.pid_loc_2level[KP] = 0;
	Ano_Parame.set.pid_loc_2level[KI] = 0;
	Ano_Parame.set.pid_loc_2level[KD] = 0;
/* GPS 位置控制速度环 PID 参数。 */
	Ano_Parame.set.pid_gps_loc_1level[KP] = 0.15f;
	Ano_Parame.set.pid_gps_loc_1level[KI] = 0.10f;
	Ano_Parame.set.pid_gps_loc_1level[KD] = 0.00f;
/* GPS 位置控制位置环 PID 参数。 */
	Ano_Parame.set.pid_gps_loc_2level[KP] = 0.3f;
	Ano_Parame.set.pid_gps_loc_2level[KI] = 0;
	Ano_Parame.set.pid_gps_loc_2level[KD] = 0;
	
	ANO_DT_SendString("PID reset!");
}


/* 将参数区中的校准数据同步到飞控运行时结构。 */
static void Parame_Copy_Para2fc()
{
	for(u8 i = 0;i<3;i++)
	{	
		save.acc_offset[i]		=	Ano_Parame.set.acc_offset[i];
		save.gyro_offset[i]		=	Ano_Parame.set.gyro_offset[i];
		save.mag_offset[i]		=	Ano_Parame.set.mag_offset[i];  
		save.mag_gain[i]		=	Ano_Parame.set.mag_gain[i];  
		
		Center_Pos_Set();
			
	}
}

/* 将飞控运行时校准数据回填到参数区镜像。 */
static void Parame_Copy_Fc2para()
{

	for(u8 i = 0;i<3;i++)
	{	
		Ano_Parame.set.acc_offset[i]	=	save.acc_offset[i];
		Ano_Parame.set.gyro_offset[i]	=	save.gyro_offset[i];
		Ano_Parame.set.mag_offset[i]	=	save.mag_offset[i];  
		Ano_Parame.set.mag_gain[i]		=	save.mag_gain[i];   
			
		
	}
}
/* 恢复默认飞控参数。 */
void Parame_Reset(void)
{
	Ano_Parame.set.pwmInMode = SBUS;
	Ano_Parame.set.heatSwitch = 0;
	Ano_Parame.set.warn_power_voltage = 3.50f *3;
	Ano_Parame.set.return_home_power_voltage = 3.7f *3;
	Ano_Parame.set.lowest_power_voltage = 3.4f *3;
	
	Ano_Parame.set.auto_take_off_height = 60;	/* cm */
	Ano_Parame.set.auto_take_off_speed = 80;
	Ano_Parame.set.auto_landing_speed = 60;
	
	Ano_Parame.set.idle_speed_pwm = 20;	/* 20% */
	
	for(u8 i = 0;i<3;i++)
	{
		Ano_Parame.set.acc_offset[i] = 0;
		Ano_Parame.set.gyro_offset[i] = 0;
		Ano_Parame.set.mag_offset[i] = 0;  
		Ano_Parame.set.mag_gain[i] = 1;    
		
		Ano_Parame.set.center_pos_cm[i] = 0;
	}
	
	Parame_Copy_Para2fc();
		
	ANO_DT_SendString("parameter reset!");
}



/* 将当前参数镜像写入存储区。 */
static void Ano_Parame_Write(void)
{
	All_PID_Init();	/* 存储 PID 参数后重新初始化控制器。 */
	Ano_Parame.set.frist_init = SOFT_VER;

	Parame_Copy_Fc2para();

	Dvr_ParamterSave();
}

/* 读取参数区，必要时执行默认初始化并写回。 */
void Ano_Parame_Read(void)
{
	Dvr_ParamterRead();
	
	if(Ano_Parame.set.frist_init != SOFT_VER)
	{		
		Parame_Reset();
		PID_Rest();
		Ano_Parame_Write();
	}
	
	Parame_Copy_Para2fc();
	
	
}


/*
 * 参数延时保存任务，避免飞行中立即写入 Flash。
 * 说明：仅在允许保存时进入延时计时，飞行中不直接写入参数区。
 */
void Ano_Parame_Write_task(u16 dT_ms)
{
	if(para_sta.save_en )
	{
		if(para_sta.save_trig == 1)
		{
			/* 收到保存请求后，先进入延时等待。 */
			LED_STA.saving = 1;
			
			para_sta.time_delay = 0;
			para_sta.save_trig = 2;
		}
		
		if(para_sta.save_trig == 2)
		{
			if(para_sta.time_delay<3000)
			{
				para_sta.time_delay += dT_ms;
			}
			else
			{
				/* 延时到达后再真正写入参数区。 */
				para_sta.save_trig = 0;
				Ano_Parame_Write();
				ANO_DT_SendString("Set save OK!");
				LED_STA.saving = 0;
			}
		}
		else
		{
			para_sta.time_delay = 0;
		}
		
	}
	else
	{
		para_sta.time_delay = 0;
		para_sta.save_trig = 0;
	}
}


