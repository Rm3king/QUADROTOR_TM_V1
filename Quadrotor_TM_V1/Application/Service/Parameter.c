/*
 * 文件名称: Parameter.c
 * 所属模块: Application / Service
 *
 * 功能描述:
 *   参数管理模块，维护飞控所有可调参数的默认值、运行时镜像和持久化存储：
 *   1) Default_Para_Init()   -- 设置出厂默认参数（PID 增益、限幅、传感器偏移等）
 *   2) Para_Wr_Check_Task()  -- 延时保存任务：收到写入请求后延迟一段时间再写 Flash
 *   3) g_fc_param             -- 参数联合体，byte[] 与结构体字段共享内存
 *
 * 参数存储结构:
 *   union Parameter {
 *       u8 byte[PARA_SIZE];      // Flash 读写时按字节访问
 *       struct { ... } set;      // 运行时按字段名访问（PID、限幅、校准值等）
 *   };
 *
 * 持久化流程:
 *   上位机写参数 → DT.c 设置保存标志 → Para_Wr_Check_Task() 延时写入 EEPROM/Flash
 *   → 写入完成后 LED 绿灯常亮提示
 *
 * 架构位置:
 *   由 Scheduler 按 20ms 周期调用保存检查任务。
 *   被 DT.c（参数读写）和各控制模块（PID 初始化）引用。
 *
 * 教学提示:
 *   - union 技巧让同一块内存既可按字节操作（Flash I/O）又可按字段访问
 *   - 延时保存避免频繁写 Flash（Flash 寿命有限，约 10 万次擦写）
 *   - 修改参数结构时需同步更新 DT.c 的参数映射表
 */

//#include "Drv_w25qxx.h"
#include "Parameter.h"
#include "Drv_Paramter.h"
#include "FlightCtrl.h"
#include "Drv_led.h"
#include "Drv_icm20602.h"
#include "Sensor_Basic.h"
#include "LED.h"
#include "DT.h"


/* 参数镜像与保存状态实例。 */
union Parameter g_fc_param;
param_state_t g_param_state;

/* 注意：这里定义的是参数默认值。若未触发保存，存储区中的旧值不会自动更新。 */
/* 恢复默认 PID 参数，不改变参数项语义。 */
void FC_Param_ResetPid(void)
{
/* 姿态控制角速度环 PID 参数。 */
	g_fc_param.set.pid_att_1level[ROL][KP] = 4.0f;
	g_fc_param.set.pid_att_1level[ROL][KI] = 3.0f;
	g_fc_param.set.pid_att_1level[ROL][KD] = 0.15f;
	
	g_fc_param.set.pid_att_1level[PIT][KP] = 4.0f;
	g_fc_param.set.pid_att_1level[PIT][KI] = 3.0f;
	g_fc_param.set.pid_att_1level[PIT][KD] = 0.15f;
	
	g_fc_param.set.pid_att_1level[YAW][KP] = 6.0f;
	g_fc_param.set.pid_att_1level[YAW][KI] = 0.5f;
	g_fc_param.set.pid_att_1level[YAW][KD] = 0.0f;
/* 姿态控制角度环 PID 参数。 */
	g_fc_param.set.pid_att_2level[ROL][KP] = 7.0f;
	g_fc_param.set.pid_att_2level[ROL][KI] = 0.0f;
	g_fc_param.set.pid_att_2level[ROL][KD] = 0.00f;
	
	g_fc_param.set.pid_att_2level[PIT][KP] = 7.0f;
	g_fc_param.set.pid_att_2level[PIT][KI] = 0.0f;
	g_fc_param.set.pid_att_2level[PIT][KD] = 0.00f;
	
	g_fc_param.set.pid_att_2level[YAW][KP] = 5.0f;
	g_fc_param.set.pid_att_2level[YAW][KI] = 0.0f;
	g_fc_param.set.pid_att_2level[YAW][KD] = 0.5;
/* 高度控制速度环 PID 参数。 */
	g_fc_param.set.pid_alt_1level[KP] = 2.0f;
	g_fc_param.set.pid_alt_1level[KI] = 1.0f;
	g_fc_param.set.pid_alt_1level[KD] = 0.05f;
/* 高度控制高度环 PID 参数。 */
	g_fc_param.set.pid_alt_2level[KP] = 1.0f;
	g_fc_param.set.pid_alt_2level[KI] = 0;
	g_fc_param.set.pid_alt_2level[KD] = 0;
/* 位置控制速度环 PID 参数。 */
	g_fc_param.set.pid_loc_1level[KP] = 0.15f;
	g_fc_param.set.pid_loc_1level[KI] = 0.10f;
	g_fc_param.set.pid_loc_1level[KD] = 0.00f;
/* 位置控制位置环 PID 参数。 */
	g_fc_param.set.pid_loc_2level[KP] = 0;
	g_fc_param.set.pid_loc_2level[KI] = 0;
	g_fc_param.set.pid_loc_2level[KD] = 0;
/* GPS 位置控制速度环 PID 参数。 */
	g_fc_param.set.pid_gps_loc_1level[KP] = 0.15f;
	g_fc_param.set.pid_gps_loc_1level[KI] = 0.10f;
	g_fc_param.set.pid_gps_loc_1level[KD] = 0.00f;
/* GPS 位置控制位置环 PID 参数。 */
	g_fc_param.set.pid_gps_loc_2level[KP] = 0.3f;
	g_fc_param.set.pid_gps_loc_2level[KI] = 0;
	g_fc_param.set.pid_gps_loc_2level[KD] = 0;
	
	ANO_DT_SendString("PID reset!");
}


/* 将参数镜像中的校准数据同步到飞控运行时状态。 */
static void FC_Param_CopyParamToRuntime(void)
{
	for(u8 i = 0;i<3;i++)
	{	
		save.acc_offset[i]		=	g_fc_param.set.acc_offset[i];
		save.gyro_offset[i]		=	g_fc_param.set.gyro_offset[i];
		save.mag_offset[i]		=	g_fc_param.set.mag_offset[i];  
		save.mag_gain[i]		=	g_fc_param.set.mag_gain[i];  
		
		Center_Pos_Set();
			
	}
}

/* 将飞控运行时校准数据回填到参数镜像。 */
static void FC_Param_CopyRuntimeToParam(void)
{

	for(u8 i = 0;i<3;i++)
	{	
		g_fc_param.set.acc_offset[i]	=	save.acc_offset[i];
		g_fc_param.set.gyro_offset[i]	=	save.gyro_offset[i];
		g_fc_param.set.mag_offset[i]	=	save.mag_offset[i];  
		g_fc_param.set.mag_gain[i]		=	save.mag_gain[i];   
			
		
	}
}
/* 恢复默认飞控参数，不改变外部参数编号和含义。 */
void FC_Param_Reset(void)
{
	g_fc_param.set.pwmInMode = SBUS;
	g_fc_param.set.heatSwitch = 0;
	g_fc_param.set.warn_power_voltage = 3.50f *3;
	g_fc_param.set.return_home_power_voltage = 3.7f *3;
	g_fc_param.set.lowest_power_voltage = 3.4f *3;
	
	g_fc_param.set.auto_take_off_height = 60;	/* cm */
	g_fc_param.set.auto_take_off_speed = 80;
	g_fc_param.set.auto_landing_speed = 60;
	
	g_fc_param.set.idle_speed_pwm = 20;	/* 20% */
	
	for(u8 i = 0;i<3;i++)
	{
		g_fc_param.set.acc_offset[i] = 0;
		g_fc_param.set.gyro_offset[i] = 0;
		g_fc_param.set.mag_offset[i] = 0;  
		g_fc_param.set.mag_gain[i] = 1;    
		
		g_fc_param.set.center_pos_cm[i] = 0;
	}
	
	FC_Param_CopyParamToRuntime();
		
	ANO_DT_SendString("parameter reset!");
}



/* 将当前参数镜像写入存储区。写入后重新装载控制器参数。 */
static void FC_Param_Write(void)
{
	All_PID_Init();	/* 存储 PID 参数后重新初始化控制器。 */
	g_fc_param.set.frist_init = SOFT_VER;

	FC_Param_CopyRuntimeToParam();

	Dvr_ParamterSave();
}

/* 读取参数区。若检测到版本不匹配，则恢复默认值并写回。 */
void FC_Param_Read(void)
{
	Dvr_ParamterRead();
	
	if(g_fc_param.set.frist_init != SOFT_VER)
	{		
		FC_Param_Reset();
		FC_Param_ResetPid();
		FC_Param_Write();
	}
	
	FC_Param_CopyParamToRuntime();
	
	
}


/*
 * 参数延时保存任务，避免飞行中立即写入 Flash。
 * 说明：仅在允许保存时进入延时计时，飞行中不直接写入参数区。
 */
void FC_Param_WriteTask(u16 dT_ms)
{
	if(g_param_state.save_en )
	{
		if(g_param_state.save_trig == 1)
		{
			/* 收到保存请求后，先进入延时等待。 */
			LED_STA.saving = 1;
			
			g_param_state.time_delay = 0;
			g_param_state.save_trig = 2;
		}
		
		if(g_param_state.save_trig == 2)
		{
			if(g_param_state.time_delay<3000)
			{
				g_param_state.time_delay += dT_ms;
			}
			else
			{
				/* 延时到达后再真正写入参数区。 */
				g_param_state.save_trig = 0;
				FC_Param_Write();
				ANO_DT_SendString("Set save OK!");
				LED_STA.saving = 0;
			}
		}
		else
		{
			g_param_state.time_delay = 0;
		}
		
	}
	else
	{
		g_param_state.time_delay = 0;
		g_param_state.save_trig = 0;
	}
}


