#include "AttCtrl.h"
#include "Imu.h"
#include "Drv_icm20602.h"
#include "MagProcess.h"
#include "Drv_spl06.h"
#include "MotionCal.h"
#include "FlightCtrl.h"
#include "LocCtrl.h"
#include "MotorCtrl.h"
#include "FlyCtrl.h"
#include "RC.h"
#include "Parameter.h"
#include "Sensor_Basic.h"
#include "ProgramCtrl_User.h"
/*
 * 文件名称: AttCtrl.c
 * 所属模块: Algorithm / Control
 *
 * 功能描述:
 *   姿态控制模块，实现串级 PID 双环控制：
 *   1) Att_2level_Ctrl()  —— 姿态角外环：期望角度→期望角速度
 *   2) Att_1level_Ctrl()  —— 角速度内环：期望角速度→混控输出 mc.roll/pitch/yaw
 *
 * 控制结构:
 *   ┌────────────┐     ┌──────────────┐     ┌──────────────┐
 *   │ LocCtrl    │────▶│ Att_2level   │────▶│ Att_1level   │──▶ mc.roll/pitch/yaw
 *   │ (位置→角度)│     │ (角度→角速度)│     │ (角速度→力矩)│
 *   └────────────┘     └──────────────┘     └──────────────┘
 *                           ▲                     ▲
 *                      imu_data.rol/pit      sensor.Gyro_deg
 *                      (欧拉角反馈)          (陀螺仪反馈)
 *
 * YAW 控制:
 *   - 外环：摇杆量转换为期望偏航角速度 → 积分为期望偏航角 → PID 误差
 *   - 偏航误差限制在 ±90° 防止过大累积
 *   - 增量限幅（±30°/s per step）保证平滑转向
 *
 * 架构位置:
 *   由 Scheduler 按 5ms 周期调用。输出写入 mc 结构体供 MotorCtrl 混控。
 *   Roll/Pitch 期望来自 LocCtrl 的位置环输出。
 *
 * 教学提示:
 *   - 双环结构是多旋翼姿态控制的标准方案：外环慢（角度），内环快（角速度）
 *   - FINAL_P 和 X_PROPORTION_X_Y 是最终比例因子，用于补偿机型差异
 *   - Set_Att_1level_Ki() / Set_Att_2level_Ki() 用于起飞前禁止积分累积
 */
#define ATT_PID_INTE_ERR_LIM   5
#define RATE_PID_INTE_ERR_LIM  200
#define YAW_SPEED_MEDIUM       220
#define YAW_SPEED_LOW          200
#define MC_ROLL_PITCH_LIMIT    1000
#define MC_YAW_LIMIT           400
//角度环控制参数
_PID_arg_st arg_2[VEC_RPY] ; 
//角速度环控制参数
_PID_arg_st arg_1[VEC_RPY] ;
//角度环控制数据
_PID_val_st val_2[VEC_RPY];
//角速度环控制数据
_PID_val_st val_1[VEC_RPY];
/* 姿态角外环 PID 参数初始化 */
void Att_2level_PID_Init()
{
	arg_2[ROL].kp = g_fc_param.set.pid_att_2level[ROL][KP];
	arg_2[ROL].ki = g_fc_param.set.pid_att_2level[ROL][KI];
	arg_2[ROL].kd_ex = g_fc_param.set.pid_att_2level[ROL][KD];
	arg_2[ROL].kd_fb = g_fc_param.set.pid_att_2level[ROL][KD];
	arg_2[ROL].k_ff = 0.0f;
	
	arg_2[PIT].kp = g_fc_param.set.pid_att_2level[PIT][KP];
	arg_2[PIT].ki = g_fc_param.set.pid_att_2level[PIT][KI];
	arg_2[PIT].kd_ex = g_fc_param.set.pid_att_2level[PIT][KD];
	arg_2[PIT].kd_fb = g_fc_param.set.pid_att_2level[PIT][KD];
	arg_2[PIT].k_ff = 0.0f;
	arg_2[YAW].kp = g_fc_param.set.pid_att_2level[YAW][KP];
	arg_2[YAW].ki = g_fc_param.set.pid_att_2level[YAW][KI];
	arg_2[YAW].kd_ex = g_fc_param.set.pid_att_2level[YAW][KD];
	arg_2[YAW].kd_fb = g_fc_param.set.pid_att_2level[YAW][KD];
	arg_2[YAW].k_ff = 0.0f;		
}
/*
姿态角速率部分控制参数
arg_1_kp：调整角速度响应速度，不震荡的前提下，尽量越高越好。
震荡试，可以降低arg_1_kp，增大arg_1_kd。
若增大arg_1_kd已经不能抑制震荡，需要将kp和kd同时减小。
*/
#define CTRL_1_KI_START 0.f
/* 角速度内环 PID 参数初始化 */
void Att_1level_PID_Init()
{
	arg_1[ROL].kp = g_fc_param.set.pid_att_1level[ROL][KP];
	arg_1[ROL].ki = g_fc_param.set.pid_att_1level[ROL][KI];
	arg_1[ROL].kd_ex = 0;
	arg_1[ROL].kd_fb = g_fc_param.set.pid_att_1level[ROL][KD];
	arg_1[ROL].k_ff = 0.0f;
	
	arg_1[PIT].kp = g_fc_param.set.pid_att_1level[PIT][KP];
	arg_1[PIT].ki = g_fc_param.set.pid_att_1level[PIT][KI];
	arg_1[PIT].kd_ex = 0;
	arg_1[PIT].kd_fb = g_fc_param.set.pid_att_1level[PIT][KD];
	arg_1[PIT].k_ff = 0.0f;
	arg_1[YAW].kp = g_fc_param.set.pid_att_1level[YAW][KP];
	arg_1[YAW].ki = g_fc_param.set.pid_att_1level[YAW][KI];
	arg_1[YAW].kd_ex = 0;
	arg_1[YAW].kd_fb = g_fc_param.set.pid_att_1level[YAW][KD];
	arg_1[YAW].k_ff = 0.00f;	
	
#if (MOTOR_ESC_TYPE == 2)
	#define DIFF_GAIN 0.3f
	arg_1[ROL].kd_fb = arg_1[ROL].kd_fb *DIFF_GAIN;
	arg_1[PIT].kd_fb = arg_1[PIT].kd_fb *DIFF_GAIN;
#elif (MOTOR_ESC_TYPE == 1)
	#define DIFF_GAIN 1.0f
	arg_1[ROL].kd_fb = arg_1[ROL].kd_fb *DIFF_GAIN;
	arg_1[PIT].kd_fb = arg_1[PIT].kd_fb *DIFF_GAIN;
#endif
}
/* 按模式设置角速度内环积分项 */
void Set_Att_1level_Ki(u8 mode)
{
	if(mode == 0)
	{
		arg_1[ROL].ki = arg_1[PIT].ki = 0;
	}
	else if(mode == 1)
	{
		arg_1[ROL].ki = g_fc_param.set.pid_att_1level[ROL][KI];
		arg_1[PIT].ki = g_fc_param.set.pid_att_1level[PIT][KI];
	}
	else 
	{
		arg_1[ROL].ki = arg_1[PIT].ki = CTRL_1_KI_START;
	}
}
/* 按模式设置姿态角外环积分项 */
void Set_Att_2level_Ki(u8 mode)
{
	if(mode == 0)
	{
		arg_2[ROL].ki = arg_2[PIT].ki = 0;
	}
	else
	{
		arg_2[ROL].ki = g_fc_param.set.pid_att_2level[ROL][KI];
		arg_2[PIT].ki = g_fc_param.set.pid_att_2level[PIT][KI];
	}
}
_att_2l_ct_st att_2l_ct;
static s32 max_yaw_speed,set_yaw_av_tmp;
static float exp_rol_tmp,exp_pit_tmp;
	
/* 姿态角外环控制任务 */
void Att_2level_Ctrl(float dT_s)
{
	/*积分微调*/
	exp_rol_tmp = - loc_ctrl_1.out[Y];
	exp_pit_tmp = - loc_ctrl_1.out[X];
	
	if(flag.flight_mode == ATT_STAB)
	{
		if(ABS(exp_rol_tmp + att_2l_ct.exp_rol_adj) < 5)
		{
			att_2l_ct.exp_rol_adj += 0.2f *exp_rol_tmp *dT_s;
			att_2l_ct.exp_rol_adj = LIMIT(att_2l_ct.exp_rol_adj,-1,1);
		}
		
		if(ABS(exp_pit_tmp + att_2l_ct.exp_pit_adj) < 5)
		{
			att_2l_ct.exp_pit_adj += 0.2f *exp_pit_tmp *dT_s;
			att_2l_ct.exp_pit_adj = LIMIT(att_2l_ct.exp_pit_adj,-1,1);
		}
	}
	else
	{
		att_2l_ct.exp_rol_adj = 
		att_2l_ct.exp_pit_adj = 0;
	}
	
	/*正负参考ANO坐标参考方向*/
	att_2l_ct.exp_rol = exp_rol_tmp + att_2l_ct.exp_rol_adj;
	att_2l_ct.exp_pit = exp_pit_tmp + att_2l_ct.exp_pit_adj;
	
	/*期望角度限幅*/
	att_2l_ct.exp_rol = LIMIT(att_2l_ct.exp_rol,-MAX_ANGLE,MAX_ANGLE);
	att_2l_ct.exp_pit = LIMIT(att_2l_ct.exp_pit,-MAX_ANGLE,MAX_ANGLE);
	
		if(flag.speed_mode == 3)
	{
		max_yaw_speed = MAX_SPEED_YAW;
	}
	else if(flag.speed_mode == 2 )
	{
		max_yaw_speed = YAW_SPEED_MEDIUM;
	}
	else
	{
		max_yaw_speed = YAW_SPEED_LOW;
	}
	//
	fc_stv.yaw_pal_limit = max_yaw_speed;
	/*摇杆量转换为YAW期望角速度 + 程控期望角速度*/
	set_yaw_av_tmp = (s32)(0.0023f *my_deadzone(RC_GetChannel(CH_YAW),0,65) *max_yaw_speed) + (-program_ctrl.yaw_pal_dps) + pc_user.pal_dps_set;
	/*最大YAW角速度限幅*/
	set_yaw_av_tmp = LIMIT(set_yaw_av_tmp ,-max_yaw_speed,max_yaw_speed);
	
	/*没有起飞，复位*/
	if(flag.taking_off == 0)
	{
		att_2l_ct.exp_rol = att_2l_ct.exp_pit = set_yaw_av_tmp = 0;
		att_2l_ct.exp_yaw = att_2l_ct.fb_yaw;
	}
	/*限制误差增大*/
	if(att_2l_ct.yaw_err>90)
	{
		if(set_yaw_av_tmp>0)
		{
			set_yaw_av_tmp = 0;
		}
	}
	else if(att_2l_ct.yaw_err<-90)
	{
		if(set_yaw_av_tmp<0)
		{
			set_yaw_av_tmp = 0;
		}
	}	
	//增量限幅
	att_1l_ct.set_yaw_speed += LIMIT((set_yaw_av_tmp - att_1l_ct.set_yaw_speed),-30,30);
	/*设置期望YAW角度*/
	att_2l_ct.exp_yaw += att_1l_ct.set_yaw_speed *dT_s;
	/*限制为+-180度*/
	if(att_2l_ct.exp_yaw<-180) att_2l_ct.exp_yaw += 360;
	else if(att_2l_ct.exp_yaw>180) att_2l_ct.exp_yaw -= 360;	
	
	/*计算YAW角度误差*/
	att_2l_ct.yaw_err = (att_2l_ct.exp_yaw - att_2l_ct.fb_yaw);
	/*限制为+-180度*/
	if(att_2l_ct.yaw_err<-180) att_2l_ct.yaw_err += 360;
	else if(att_2l_ct.yaw_err>180) att_2l_ct.yaw_err -= 360;
	
		/*赋值反馈角度值*/
		att_2l_ct.fb_yaw = imu_data.yaw ;
			
		att_2l_ct.fb_rol = (imu_data.rol ) ;
		att_2l_ct.fb_pit = (imu_data.pit ) ;
			
	
	PID_calculate( dT_s,
										0 ,
										att_2l_ct.exp_rol ,
										att_2l_ct.fb_rol ,
										&arg_2[ROL],
										&val_2[ROL],
	                  ATT_PID_INTE_ERR_LIM,
										ATT_PID_INTE_ERR_LIM *flag.taking_off
										 )	;
										
	PID_calculate( dT_s,
										0 ,
										att_2l_ct.exp_pit ,
										att_2l_ct.fb_pit ,
										&arg_2[PIT],
										&val_2[PIT],
	                  ATT_PID_INTE_ERR_LIM,
										ATT_PID_INTE_ERR_LIM *flag.taking_off
										 )	;
	
	PID_calculate( dT_s,
										0 ,
										att_2l_ct.yaw_err ,
										0 ,
										&arg_2[YAW],
										&val_2[YAW],
	                  ATT_PID_INTE_ERR_LIM,
										ATT_PID_INTE_ERR_LIM *flag.taking_off
										 )	;
}
_att_1l_ct_st att_1l_ct;
static float ct_val[4];
/*角速度环控制*/
/* 角速度内环控制任务 */
void Att_1level_Ctrl(float dT_s)
{
	/* 根据飞行状态切换控制参数 */
	ctrl_parameter_change_task();
	
		/*目标角速度赋值*/
		 for(u8 i = 0;i<3;i++)
		{
			att_1l_ct.exp_angular_velocity[i] = val_2[i].out;// val_2[i].out;//
		}
	
		/*目标角速度限幅*/
		att_1l_ct.exp_angular_velocity[ROL] = LIMIT(att_1l_ct.exp_angular_velocity[ROL],-MAX_ROLLING_SPEED,MAX_ROLLING_SPEED);
		att_1l_ct.exp_angular_velocity[PIT] = LIMIT(att_1l_ct.exp_angular_velocity[PIT],-MAX_ROLLING_SPEED,MAX_ROLLING_SPEED);
	/*反馈角速度赋值*/
	att_1l_ct.fb_angular_velocity[ROL] = ( sensor.Gyro_deg[X] );
	att_1l_ct.fb_angular_velocity[PIT] = (-sensor.Gyro_deg[Y] );
	att_1l_ct.fb_angular_velocity[YAW] = (-sensor.Gyro_deg[Z] );
	
	/*PID计算*/									 
 for(u8 i = 0;i<3;i++)
 {
		PID_calculate( dT_s,
										0,
										att_1l_ct.exp_angular_velocity[i],
										att_1l_ct.fb_angular_velocity[i],
										&arg_1[i],
										&val_1[i],
                    RATE_PID_INTE_ERR_LIM,
										CTRL_1_INTE_LIM *flag.taking_off
										 )	; 
 
	
	 ct_val[i] = (val_1[i].out);
 }
										 
	/*赋值，最终比例调节*/
	mc.roll =                   FINAL_P *ct_val[ROL];
	mc.pitch = X_PROPORTION_X_Y *FINAL_P *ct_val[PIT];
	mc.yaw =                   FINAL_P *ct_val[YAW];
	/*输出量限幅*/
	mc.roll = LIMIT(mc.roll,-MC_ROLL_PITCH_LIMIT,MC_ROLL_PITCH_LIMIT);
	mc.pitch = LIMIT(mc.pitch,-MC_ROLL_PITCH_LIMIT,MC_ROLL_PITCH_LIMIT);
	mc.yaw = LIMIT(mc.yaw,-MC_YAW_LIMIT,MC_YAW_LIMIT);	
}
_rolling_flag_st rolling_flag;
