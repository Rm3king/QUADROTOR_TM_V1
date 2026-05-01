#include "LocCtrl.h"
#include "Drv_Gps.h"
#include "Imu.h"
#include "FlightCtrl.h"
#include "OF.h"
#include "OF_DecoFusion.h"
#include "Parameter.h"
#include "UWB.h"
/*
 * 文件名称: LocCtrl.c
 * 所属模块: Algorithm / Control
 *
 * 功能描述:
 *   水平位置/速度控制模块，根据当前传感器可用性自动切换观测源：
 *   1) 仅光流模式 (s_loc_mode=1)  —— 光流速度反馈 + 加速度超前补偿
 *   2) GPS 模式 (s_loc_mode=2)    —— GPS 速度/位置反馈 + 位置保持
 *   3) 仅 UWB 模式 (s_loc_mode=3) —— 直接角度映射（预留）
 *   4) UWB + 光流 (s_loc_mode=4)  —— 光流速度 + UWB 积分修正
 *   5) 姿态模式 (s_loc_mode=255)  —— 无定位，摇杆直接映射为倾斜角
 *
 * PID 结构:
 *   采用 PD + I 分离结构：
 *   - loc_arg_1 / loc_val_1 ：速度 PD 控制（响应快、不累积）
 *   - loc_arg_1_fix / loc_val_1_fix ：纯 I 控制（消除稳态偏差）
 *   两者输出叠加后经坐标变换输出 loc_ctrl_1.out → AttCtrl 姿态期望
 *
 * 坐标系:
 *   - h (Heading)：机头方向坐标系，X 向前，Y 向右
 *   - w (World/NWU)：世界坐标系，X 向北，Y 向西，Z 向上
 *   - h2w/w2h_2d_trans()：航向↔世界 2D 坐标变换
 *
 * 架构位置:
 *   由 Scheduler 按 10ms/20ms 周期调用。
 *   输出 loc_ctrl_1.out[X/Y] → AttCtrl 作为 Roll/Pitch 期望角度。
 *
 * 教学提示:
 *   - GPS 模式下使用梯形速度规划（加速→匀速→减速→定点保持）
 *   - 加速度超前补偿 (ACC_LEAD_GAIN) 改善速度环相位裕度
 *   - 模式切换时自动重新初始化 PID 参数，避免参数混用
 */
#define LOC_PID_INTE_ERR_LIM   50
#define LOC_PID_OUT_SCALE      10
#define ACC_LEAD_GAIN          0.03f
#define GPS_DECEL_RATE         5
#define GPS_POS_HOLD_DELAY     50
//位置速度环控制参数
_PID_arg_st loc_arg_1[2] ; 
//位置速度环控制数据
_PID_val_st loc_val_1[2] ; 
static _PID_arg_st loc_arg_1_fix[2] ;
static _PID_val_st loc_val_1_fix[2] ;
static u8 s_loc_mode[2];
/* 水平位置/速度环 PID 参数初始化 */
void Loc_1level_PID_Init()
{
	if(s_loc_mode[1] == 2)
	{
		loc_arg_1[X].kp = g_fc_param.set.pid_gps_loc_1level[KP];//0.22f  ;
		loc_arg_1[X].ki = 0  ;
		loc_arg_1[X].kd_ex = 0.00f ;
		loc_arg_1[X].kd_fb = g_fc_param.set.pid_gps_loc_1level[KD];
		loc_arg_1[X].k_ff = 0.02f;
		
		loc_arg_1[Y] = loc_arg_1[X];
		loc_arg_1_fix[X].kp = 0.0f  ;
		loc_arg_1_fix[X].ki = g_fc_param.set.pid_gps_loc_1level[KI] ;
		loc_arg_1_fix[X].kd_ex = 0.00f;
		loc_arg_1_fix[X].kd_fb = 0.00f;
		loc_arg_1_fix[X].k_ff = 0.0f;
		
		loc_arg_1_fix[Y] = loc_arg_1_fix[X];	
	}
	else if(s_loc_mode[1] == 1)
	{
		loc_arg_1[X].kp = g_fc_param.set.pid_loc_1level[KP];//0.22f  ;
		loc_arg_1[X].ki = 0.0f  ;
		loc_arg_1[X].kd_ex = 0.00f ;
		loc_arg_1[X].kd_fb = g_fc_param.set.pid_loc_1level[KD];
		loc_arg_1[X].k_ff = 0.02f;
		
		loc_arg_1[Y] = loc_arg_1[X];
		loc_arg_1_fix[X].kp = 0.0f  ;
		loc_arg_1_fix[X].ki = g_fc_param.set.pid_loc_1level[KI] ;
		loc_arg_1_fix[X].kd_ex = 0.00f;
		loc_arg_1_fix[X].kd_fb = 0.00f;
		loc_arg_1_fix[X].k_ff = 0.0f;
		
		loc_arg_1_fix[Y] = loc_arg_1_fix[X];	
	}
	//UWB 、UWB AND OF
	else if(s_loc_mode[1] == 3 || s_loc_mode[1] == 4)
	{
		loc_arg_1[X].kp = g_fc_param.set.pid_loc_1level[KP];//0.22f  ;
		loc_arg_1[X].ki = 0.0f  ;
		loc_arg_1[X].kd_ex = 0.00f ;
		loc_arg_1[X].kd_fb = g_fc_param.set.pid_loc_1level[KD];
		loc_arg_1[X].k_ff = 0.02f;
		
		loc_arg_1[Y] = loc_arg_1[X];
		loc_arg_1_fix[X].kp = 0.0f  ;
		loc_arg_1_fix[X].ki = g_fc_param.set.pid_loc_1level[KI] ;
		loc_arg_1_fix[X].kd_ex = 0.00f;
		loc_arg_1_fix[X].kd_fb = 0.00f;
		loc_arg_1_fix[X].k_ff = 0.0f;
		
		loc_arg_1_fix[Y] = loc_arg_1_fix[X];			
	}
	//
	else
	{
	}
	
}
_loc_ctrl_st loc_ctrl_1;
static float fb_speed_fix[2];
static float vel_fb_d_lpf[2];
static float vel_fb_h[2], vel_fb_w[2];
/* 水平位置/速度控制任务 */
void Loc_1level_Ctrl(u16 dT_ms)
{
	static float loc_hand_exp_vel[2]={0};
	static unsigned short waite_gps_loc_cnt = 0;
	float ne_pos_control[2];
	unsigned char vel_diff = GPS_DECEL_RATE;
	float pos_ctrl_h_out[2];
	float pos_ctrl_w_out[2];
	
	//仅有UWB(暂无)
	if(switchs.uwb_on && (!switchs.of_flow_on) && (!switchs.gps_on))
	{
		s_loc_mode[1] = 3;
		if(s_loc_mode[1] != s_loc_mode[0])
		{
			Loc_1level_PID_Init();
			s_loc_mode[0] = s_loc_mode[1];
		}	
		loc_ctrl_1.out[X] = (float)MAX_ANGLE/MAX_SPEED *fs.speed_set_h[X] ;
		loc_ctrl_1.out[Y] = (float)MAX_ANGLE/MAX_SPEED *fs.speed_set_h[Y] ;
			
	}
	//仅有光流和UWB
	else if(switchs.uwb_on && switchs.of_flow_on && (!switchs.gps_on))
	{
		s_loc_mode[1] = 4;
		if(s_loc_mode[1] != s_loc_mode[0])
		{
			Loc_1level_PID_Init();
			s_loc_mode[0] = s_loc_mode[1];
		}	
		//期望赋值
		h2w_2d_trans(fs.speed_set_h,imu_data.hx_vec,loc_ctrl_1.exp);
		//低通滤波
		LPF_1_(5.0f,dT_ms*1e-3f,imu_data.w_acc[X],vel_fb_d_lpf[X]);
		LPF_1_(5.0f,dT_ms*1e-3f,imu_data.w_acc[Y],vel_fb_d_lpf[Y]);		
		//反馈赋值HXYZ（水平航向坐标）
		if(sens_hd_check.of_ok)
		{
			vel_fb_h[0] = OF_DX2;
			vel_fb_h[1] = OF_DY2;
		}
		else//sens_hd_check.of_df_ok
		{
			vel_fb_h[0] = of_rdf.gnd_vel_est_h[X];
			vel_fb_h[1] = of_rdf.gnd_vel_est_h[Y];	
		}
		//转换NWU（北西天）坐标
		h2w_2d_trans(vel_fb_h,imu_data.hx_vec,vel_fb_w);
		//反馈赋值+加速度超前
		loc_ctrl_1.fb[X] = vel_fb_w[0] + ACC_LEAD_GAIN *vel_fb_d_lpf[X];
		loc_ctrl_1.fb[Y] = vel_fb_w[1] + ACC_LEAD_GAIN *vel_fb_d_lpf[Y];
		//速度修正值赋值，用于积分
		fb_speed_fix[0] = uwb_data.w_vel_cmps[0];
		fb_speed_fix[1] = uwb_data.w_vel_cmps[1];
		
		for(u8 i =0;i<2;i++)
		{
			PID_calculate( dT_ms*1e-3f,            //周期（单位：秒）
										loc_ctrl_1.exp[i] ,				//前馈值
										loc_ctrl_1.exp[i] ,				//期望值（设定值）
										loc_ctrl_1.fb[i] ,			//反馈值（）
										&loc_arg_1[i], //PID参数结构体
										&loc_val_1[i],	//PID数据结构体
										LOC_PID_INTE_ERR_LIM,
										LOC_PID_OUT_SCALE *flag.taking_off
										 )	;	
			
				PID_calculate( dT_ms*1e-3f,            //周期（单位：秒）
										loc_ctrl_1.exp[i] ,				//前馈值
										loc_ctrl_1.exp[i] ,				//期望值（设定值）
										fb_speed_fix[i] ,			//反馈值（）
										&loc_arg_1_fix[i], //PID参数结构体
										&loc_val_1_fix[i],	//PID数据结构体
										LOC_PID_INTE_ERR_LIM,
										LOC_PID_OUT_SCALE *flag.taking_off
										 )	;	
			
			pos_ctrl_w_out[i] = loc_val_1[i].out + loc_val_1_fix[i].out;	//(PD)+(I)	
		}	
		//NWU转HXYZ水平航向坐标
		w2h_2d_trans(pos_ctrl_w_out,imu_data.hx_vec,pos_ctrl_h_out); 
		//输出赋值
		loc_ctrl_1.out[0] = pos_ctrl_h_out[0];
		loc_ctrl_1.out[1] = pos_ctrl_h_out[1];	
	}
	//仅有光流
	else if(switchs.of_flow_on && (!switchs.gps_on))
	{
		s_loc_mode[1] = 1;
		if(s_loc_mode[1] != s_loc_mode[0])
		{
			Loc_1level_PID_Init();
			s_loc_mode[0] = s_loc_mode[1];
		}
		loc_ctrl_1.exp[X] = fs.speed_set_h[X];
		loc_ctrl_1.exp[Y] = fs.speed_set_h[Y];
		//
		LPF_1_(5.0f,dT_ms*1e-3f,imu_data.h_acc[X],vel_fb_d_lpf[X]);
		LPF_1_(5.0f,dT_ms*1e-3f,imu_data.h_acc[Y],vel_fb_d_lpf[Y]);		
		
		if(sens_hd_check.of_ok)
		{
			loc_ctrl_1.fb[X] = OF_DX2 + ACC_LEAD_GAIN *vel_fb_d_lpf[X];
			loc_ctrl_1.fb[Y] = OF_DY2 + ACC_LEAD_GAIN *vel_fb_d_lpf[Y];
			
			fb_speed_fix[0] = OF_DX2FIX;
			fb_speed_fix[1] = OF_DY2FIX;
		}
		else//sens_hd_check.of_df_ok
		{
			loc_ctrl_1.fb[X] = of_rdf.gnd_vel_est_h[X] + ACC_LEAD_GAIN *vel_fb_d_lpf[X];
			loc_ctrl_1.fb[Y] = of_rdf.gnd_vel_est_h[Y] + ACC_LEAD_GAIN *vel_fb_d_lpf[Y];
			
			fb_speed_fix[0] = of_rdf.gnd_vel_est_h[X];
			fb_speed_fix[1] = of_rdf.gnd_vel_est_h[Y];		
		}
		
		for(u8 i =0;i<2;i++)
		{
			PID_calculate( dT_ms*1e-3f,            //周期（单位：秒）
										loc_ctrl_1.exp[i] ,				//前馈值
										loc_ctrl_1.exp[i] ,				//期望值（设定值）
										loc_ctrl_1.fb[i] ,			//反馈值（）
										&loc_arg_1[i], //PID参数结构体
										&loc_val_1[i],	//PID数据结构体
										LOC_PID_INTE_ERR_LIM,
										LOC_PID_OUT_SCALE *flag.taking_off
										 )	;	
			
				PID_calculate( dT_ms*1e-3f,            //周期（单位：秒）
										loc_ctrl_1.exp[i] ,				//前馈值
										loc_ctrl_1.exp[i] ,				//期望值（设定值）
										fb_speed_fix[i] ,			//反馈值（）
										&loc_arg_1_fix[i], //PID参数结构体
										&loc_val_1_fix[i],	//PID数据结构体
										LOC_PID_INTE_ERR_LIM,
										LOC_PID_OUT_SCALE *flag.taking_off
										 )	;	
			
			loc_ctrl_1.out[i] = loc_val_1[i].out + loc_val_1_fix[i].out;	//(PD)+(I)	
		}		
	}
	//仅有GPS
	else if (switchs.gps_on)
	{
		s_loc_mode[1] = 2;
		if(s_loc_mode[1] != s_loc_mode[0])
		{
			Loc_1level_PID_Init();
			s_loc_mode[0] = s_loc_mode[1];
		}
		for(u8 j = 0; j < 2; j++)
		{
			if (fs.speed_set_h[j] != 0)				//判读是否动控制摇杆
			{
				if (ABS(loc_hand_exp_vel[j]) < ABS(fs.speed_set_h[j]))	//判断速度是否达到期望速度
				{
					if (loc_hand_exp_vel[j]*fs.speed_set_h[j] < 0)	//判断摇杆方向和期望方向是否相同
					{
						if (fs.speed_set_h[j] > 0)					//判断摇杆方向
						{
							fs.speed_set_h[j] = MAX_SPEED;			//限制最大速度
						}
						else 
						{
							fs.speed_set_h[j] = -MAX_SPEED;
						}
					}
					loc_hand_exp_vel[j] += 0.5f*dT_ms*fs.speed_set_h[j]/MAX_SPEED;		//计算期望速度
				}
				else												
				{
					if (loc_hand_exp_vel[j] > 0)					//回杆响应按最大加速度响应
					{
						loc_hand_exp_vel[j] -= vel_diff;
					}
					else
					{
						loc_hand_exp_vel[j] += vel_diff;
					}
				}
			}
			else
			{
				if (loc_hand_exp_vel[j] > vel_diff)					//回杆响应按最大加速度响应
				{
					loc_hand_exp_vel[j] -= vel_diff;
				}
				else if (loc_hand_exp_vel[j] <= -vel_diff)
				{
					loc_hand_exp_vel[j] += vel_diff;
				}
				else
				{
					loc_hand_exp_vel[j] = 0;
				}
			}
		}
		if (loc_hand_exp_vel[X] || loc_hand_exp_vel[Y])				//判断是否有手动期望速度
		{
			ne_pos_control[0] = 0;									//位置控制量清零
			ne_pos_control[1] = 0;
			waite_gps_loc_cnt = GPS_POS_HOLD_DELAY;
		}
		else
		{
			if (waite_gps_loc_cnt > 0)
			{
				ne_pos_control[0] = 0;					//位置控制量清零
				ne_pos_control[1] = 0;
				waite_gps_loc_cnt--;
				if (waite_gps_loc_cnt == 0)				//估计位置已经稳定 记录期望位置以及对位置进行控制
				{
					Gps_information.hope_latitude = Gps_information.latitude_offset;		//期望位置等于当前位置
					Gps_information.hope_longitude = Gps_information.longitude_offset;
				}
			}
			else
			{
				Gps_information.hope_latitude_err = Gps_information.hope_latitude - Gps_information.latitude_offset;		//纬度误差
				Gps_information.hope_longitude_err = Gps_information.hope_longitude - Gps_information.longitude_offset;		//经度误差
				length_limit(&(Gps_information.hope_latitude_err), &(Gps_information.hope_longitude_err), MAX_SPEED*1.2f, ne_pos_control);	//控制模长限制
			}
		}
		
		
		loc_ctrl_1.exp[X] =  ne_pos_control[0]*g_fc_param.set.pid_gps_loc_2level[KP] + loc_hand_exp_vel[X]*imu_data.hx_vec[0] - loc_hand_exp_vel[Y]*imu_data.hx_vec[1];		//期望速度（航向坐标转换到世界坐标NED）
		loc_ctrl_1.exp[Y] = -ne_pos_control[1]*g_fc_param.set.pid_gps_loc_2level[KP] + loc_hand_exp_vel[X]*imu_data.hx_vec[1] + loc_hand_exp_vel[Y]*imu_data.hx_vec[0];		
		loc_ctrl_1.fb[X] =  (Gps_information.last_N_vel) + (wcx_acc_use*0.2f);			//速度反馈+加速度提前
		loc_ctrl_1.fb[Y] = -(Gps_information.last_E_vel) + (wcy_acc_use*0.2f);
		
		fb_speed_fix[X] =  (Gps_information.last_N_vel);
		fb_speed_fix[Y] = -(Gps_information.last_E_vel);
		
		for(u8 i =0;i<2;i++)
		{
			PID_calculate( dT_ms*1e-3f,            //周期（单位：秒）
										loc_ctrl_1.exp[i] ,				//前馈值
										loc_ctrl_1.exp[i] ,				//期望值（设定值）
										loc_ctrl_1.fb[i] ,			//反馈值（）
										&loc_arg_1[i], //PID参数结构体
										&loc_val_1[i],	//PID数据结构体
										LOC_PID_INTE_ERR_LIM,
										LOC_PID_OUT_SCALE *flag.taking_off
										 )	;		
			
				PID_calculate( dT_ms*1e-3f,            //周期（单位：秒）
										loc_ctrl_1.exp[i] ,				//前馈值
										loc_ctrl_1.exp[i] ,				//期望值（设定值）
										fb_speed_fix[i] ,			//反馈值（）
										&loc_arg_1_fix[i], //PID参数结构体
										&loc_val_1_fix[i],	//PID数据结构体
										LOC_PID_INTE_ERR_LIM,
										LOC_PID_OUT_SCALE *flag.taking_off
										 )	;	
			
			if (!flag.taking_off)
			{
				loc_val_1_fix[i].err_i = 0;
			}				
		}
		//
		pos_ctrl_w_out[0] = loc_val_1[0].out + loc_val_1_fix[0].out;//(PD)+(I)
		pos_ctrl_w_out[1] = loc_val_1[1].out + loc_val_1_fix[1].out;//(PD)+(I)
		w2h_2d_trans(pos_ctrl_w_out, imu_data.hx_vec, pos_ctrl_h_out);	//世界坐标（NWU）控制结果转换到航向坐标下
		loc_ctrl_1.out[X] = pos_ctrl_h_out[0];
		loc_ctrl_1.out[Y] = pos_ctrl_h_out[1];
	}
	//姿态模式，直接用期望速度转为角度（期望角度）
	else
	{
		s_loc_mode[1] = 255;
		if(s_loc_mode[1] != s_loc_mode[0])
		{
			Loc_1level_PID_Init();
			s_loc_mode[0] = s_loc_mode[1];
		}
		loc_ctrl_1.out[X] = (float)MAX_ANGLE/MAX_SPEED *fs.speed_set_h[X] ;
		loc_ctrl_1.out[Y] = (float)MAX_ANGLE/MAX_SPEED *fs.speed_set_h[Y] ;
	}
}
_loc_ctrl_st loc_ctrl_2;
