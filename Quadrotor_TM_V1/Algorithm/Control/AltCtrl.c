#include "AltCtrl.h"
#include "Imu.h"
#include "Drv_icm20602.h"
#include "MagProcess.h"
#include "Drv_spl06.h"
#include "MotionCal.h"
#include "FlightCtrl.h"
#include "MotorCtrl.h"
#include "AttCtrl.h"
#include "LocCtrl.h"
#include "Parameter.h"
/*
 * 文件名称: AltCtrl.c
 * 所属模块: Algorithm / Control
 *
 * 功能描述:
 *   高度控制模块，实现串级 PID 双环控制和自动起降状态机：
 *   1) Alt_2level_Ctrl()  —— 高度外环：期望高度→期望垂直速度
 *   2) Alt_1level_Ctrl()  —— 高度速度内环：期望速度→油门输出 mc.throttle
 *   3) Auto_Take_Off_Land_Task() —— 自动起飞/降落速度生成
 *
 * 控制结构:
 *       用户油门杆
 *           │
 *           ▼
 *   ┌───────────────┐     ┌───────────────┐     ┌──────────┐
 *   │ Alt_2level    │────▶│ Alt_1level    │────▶│ 油门输出 │
 *   │ (高度→速度)   │     │ (速度→油门)   │     │ mc.throttle │
 *   └───────────────┘     └───────────────┘     └──────────┘
 *         ▲                     ▲
 *    wcz_hei_fus.out       wcz_spe_fus.out
 *    (融合高度)              (融合垂直速度)
 *
 * 架构位置:
 *   由 Scheduler 按 10ms/20ms 周期调用，输出写入 mc.throttle 供 MotorCtrl 混控。
 *   依赖 FlightDataCal 提供的融合高度和速度估计。
 *
 * 教学提示:
 *   - 内环使用微分前移技巧：D 项通过 w_acc_z_lpf 提前补偿，PID 内部 kd 设为 0
 *   - 自动起飞通过 P 控制逼近目标高度，超时或用户推杆即退出
 *   - err_i_comp 是积分补偿量，让起飞后油门从零平滑过渡到悬停值
 */
#define AUTO_TAKE_OFF_KP        2.0f
#define ALT_STEP_LIMIT_CM      200
#define ALT_VEL_OUT_LIMIT      150
#define TAKEOFF_TIMEOUT_MS     5000
#define THR_CHECK_DELAY_MS     2000
#define THR_ACTIVE_THRESHOLD   0.1f
#define ALT_PID_INTE_LIM       100
#define ALT_HOLD_DEADZONE_CM   20
#define ALT_SPEED_FF_GAIN      0.6f

static s16 s_auto_takeoff_speed_cmps;
/* 自动起飞/降落流程管理 */
void Auto_Take_Off_Land_Task(u8 dT_ms)
{
	static u16 take_off_ok_cnt;
	
	one_key_take_off_task(dT_ms);
	
	if(flag.unlock_sta)
	{
		if(flag.taking_off)
		{	
			if(flag.auto_take_off_land == AUTO_TAKE_OFF_NULL)
			{
				flag.auto_take_off_land = AUTO_TAKE_OFF;		
			}
		}
	}
	else
	{
		s_auto_takeoff_speed_cmps = 0;	
		flag.auto_take_off_land = AUTO_TAKE_OFF_NULL;	
	}
	if(flag.auto_take_off_land ==AUTO_TAKE_OFF)
	{
		//限制最大起飞速度
		s16 max_take_off_vel = LIMIT(g_fc_param.set.auto_take_off_speed,20,200);
		//
		take_off_ok_cnt += dT_ms;
		s_auto_takeoff_speed_cmps = AUTO_TAKE_OFF_KP *(g_fc_param.set.auto_take_off_height - wcz_hei_fus.out);
		//计算起飞速度
		s_auto_takeoff_speed_cmps = LIMIT(s_auto_takeoff_speed_cmps,0,max_take_off_vel);
		
		//退出起飞条件1：超过高度环达标或时间满5000毫秒。
		if(take_off_ok_cnt>=TAKEOFF_TIMEOUT_MS || (g_fc_param.set.auto_take_off_height - loc_ctrl_2.exp[Z] <2))
		{
			flag.auto_take_off_land = AUTO_TAKE_OFF_FINISH;
			
			
		}
		//退出起飞条件2：2000毫秒后判断用户正在控制油门。
		if(take_off_ok_cnt >THR_CHECK_DELAY_MS && ABS(fs.speed_set_h_norm[Z])>THR_ACTIVE_THRESHOLD)
		{
			flag.auto_take_off_land = AUTO_TAKE_OFF_FINISH;
		}
	
	}
	else 
	{
		take_off_ok_cnt = 0;
		
		if(flag.auto_take_off_land ==AUTO_TAKE_OFF_FINISH)
		{
			s_auto_takeoff_speed_cmps = 0;
			
		}
		
	}
	if(flag.auto_take_off_land == AUTO_LAND)
	{
		//计算自动降落速度
		s_auto_takeoff_speed_cmps = -(s16)LIMIT(g_fc_param.set.auto_landing_speed,20,200);
	}
}
static _PID_arg_st alt_arg_2;
static _PID_val_st alt_val_2;
/*高度环PID参数初始化*/
void Alt_2level_PID_Init()
{
	alt_arg_2.kp = g_fc_param.set.pid_alt_2level[KP];
	alt_arg_2.ki = g_fc_param.set.pid_alt_2level[KI];
	alt_arg_2.kd_ex = 0.00f;
	alt_arg_2.kd_fb = g_fc_param.set.pid_alt_2level[KD];
	alt_arg_2.k_ff = 0.0f;
}
/* 高度外环控制任务 */
void Alt_2level_Ctrl(float dT_s)
{
	Auto_Take_Off_Land_Task(1000*dT_s);
	
	fs.alt_ctrl_speed_set = fs.speed_set_h[Z] + s_auto_takeoff_speed_cmps;
	//
	loc_ctrl_2.exp[Z] += fs.alt_ctrl_speed_set *dT_s;
	loc_ctrl_2.exp[Z] = LIMIT(loc_ctrl_2.exp[Z],loc_ctrl_2.fb[Z]-ALT_STEP_LIMIT_CM,loc_ctrl_2.fb[Z]+ALT_STEP_LIMIT_CM);
	//
	loc_ctrl_2.fb[Z] = (s32)wcz_hei_fus.out;
	if(fs.alt_ctrl_speed_set != 0)
	{
		flag.ct_alt_hold = 0;
	}
	else
	{
		if(ABS(loc_ctrl_1.exp[Z] - loc_ctrl_1.fb[Z])<ALT_HOLD_DEADZONE_CM)
		{
			flag.ct_alt_hold = 1;
		}
	}
	if(flag.taking_off == 1)
	{
		PID_calculate( dT_s,
						0,
						loc_ctrl_2.exp[Z],
						loc_ctrl_2.fb[Z],
						&alt_arg_2,
						&alt_val_2,
						ALT_PID_INTE_LIM,
						0
						 );
	}
	else
	{
		loc_ctrl_2.exp[Z] = loc_ctrl_2.fb[Z];
		alt_val_2.out = 0;
		
	}
	
	alt_val_2.out  = LIMIT(alt_val_2.out,-ALT_VEL_OUT_LIMIT,ALT_VEL_OUT_LIMIT);
}
static _PID_arg_st alt_arg_1;
static _PID_val_st alt_val_1;
/*高度速度环PID参数初始化*/
void Alt_1level_PID_Init()
{
	alt_arg_1.kp = g_fc_param.set.pid_alt_1level[KP];
	alt_arg_1.ki = g_fc_param.set.pid_alt_1level[KI];
	alt_arg_1.kd_ex = 0.00f;
	alt_arg_1.kd_fb = 0;
	alt_arg_1.k_ff = 0.0f;
}
static float err_i_comp;
static float w_acc_z_lpf;
/* 高度速度内环控制任务 */
void Alt_1level_Ctrl(float dT_s)
{
	u8 out_en;
	out_en = (flag.taking_off != 0) ? 1 : 0;
	
	flag.thr_mode = THR_AUTO;
	
	loc_ctrl_1.exp[Z] = ALT_SPEED_FF_GAIN *fs.alt_ctrl_speed_set + alt_val_2.out;
	
	w_acc_z_lpf += 0.2f *(imu_data.w_acc[Z] - w_acc_z_lpf); //低通滤波
	loc_ctrl_1.fb[Z] = wcz_spe_fus.out + g_fc_param.set.pid_alt_1level[KD] *w_acc_z_lpf;//微分前移，下边PID的微分系数为0
	
	
	PID_calculate( dT_s,
					0,
					loc_ctrl_1.exp[Z],
					loc_ctrl_1.fb[Z] ,
					&alt_arg_1,
					&alt_val_1,
					ALT_PID_INTE_LIM,
					(THR_INTE_LIM *10 - err_i_comp )*out_en
					 );
	
	if(flag.taking_off == 1)
	{
		LPF_1_(1.0f, dT_s, THR_START * 10, err_i_comp);
	}
	else
	{
		err_i_comp = 0;
	}
	
	
	loc_ctrl_1.out[Z] = out_en *(alt_val_1.out + err_i_comp);
	
	loc_ctrl_1.out[Z] = LIMIT(loc_ctrl_1.out[Z],0,MAX_THR_SET *10);	
	
	mc.throttle = loc_ctrl_1.out[Z];
}
