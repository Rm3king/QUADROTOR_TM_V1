#include "FlightCtrl.h"
#include "Imu.h"
#include "Drv_icm20602.h"
#include "MagProcess.h"
#include "Drv_spl06.h"
#include "MotionCal.h"
#include "AttCtrl.h"
#include "LocCtrl.h"
#include "AltCtrl.h"
#include "MotorCtrl.h"
#include "Drv_led.h"
#include "RC.h"
#include "Drv_laser.h"
#include "OF.h"
#include "OF_DecoFusion.h"
#include "FlyCtrl.h"
#include "UWB.h"
#include "Sensor_Basic.h"
#include "DT.h"
#include "LED.h"
#include "ProgramCtrl_User.h"
#include "Drv_OpenMV.h"
/*
 * 文件名称: FlightCtrl.c
 * 所属模块: Algorithm / Control
 *
 * 功能描述:
 *   飞行控制核心状态管理模块，负责以下职责：
 *   1) All_PID_Init()         —— 全局 PID 参数初始化入口
 *   2) Flight_State_Task()    —— 飞行状态机：起飞确认、飞行中、倾斜保护、落地判定
 *   3) Flight_Mode_Set()      —— 根据 AUX 遥控通道切换飞行模式（姿态/定高/定点/GPS）
 *   4) Swtich_State_Task()    —— 传感器在线状态判定（光流质量/高度有效性/GPS 星数）
 *   5) one_key_take_off/land  —— 一键起飞/降落逻辑
 *   6) ctrl_parameter_change  —— 运行时根据模式切换 PID 增益
 *
 * 架构位置:
 *   本模块是控制层的顶层调度中心，由 Scheduler 按 10ms 周期调用。
 *   向下调用 AttCtrl（姿态）、AltCtrl（高度）、LocCtrl（位置）完成闭环。
 *   向上通过 flag 全局结构体向各子模块广播飞行状态。
 *
 * 数据流:
 *   RC 通道 → Flight_Mode_Set() → flag.flight_mode
 *   传感器状态 → Swtich_State_Task() → switchs.of_flow_on / gps_on
 *   flag.taking_off / flag.flying → 各控制子模块条件分支
 *
 * 教学提示:
 *   - flag 结构体是全局飞行状态广播机制，理解它是读懂控制流的关键
 *   - 落地判定 LandDiscriminate() 使用油门+持续时间双条件避免误判
 *   - 倾斜保护通过 DCM z 轴分量（cos 值）检测姿态异常
 */
#define TAKEOFF_DELAY_MS       1400
#define LAND_DETECT_DELAY_MS   200
#define LAND_THR_THRESHOLD     250
#define LAND_DURATION_MS       1500
#define FLY_CONFIRM_MS         1000
#define TILT_PROTECT_COS       0.25f

#define OF_QUALITY_THRESHOLD   50
#define OF_QUALITY_DELAY_MS    500
#define OF_MAX_ALT_CM          600
#define OF_ALT_DELAY_MS        1000
/* 所有控制环 PID 参数初始化 */
void All_PID_Init(void)
{
	/*姿态控制：角速度PID初始化*/
	Att_1level_PID_Init();
	
	/*姿态控制：角度PID初始化*/
	Att_2level_PID_Init();
	
	/*高度控制：高度速度PID初始化*/
	Alt_1level_PID_Init();	
	
	/*高度控制：高度PID初始化*/
	Alt_2level_PID_Init();
	
	
	/*位置速度控制PID初始化*/
	Loc_1level_PID_Init();
	
}
/* 根据飞行状态切换控制参数 */
void ctrl_parameter_change_task()
{
		if(flag.auto_take_off_land ==AUTO_TAKE_OFF)
		{
			Set_Att_1level_Ki(2);
		}
		else
		{
			Set_Att_1level_Ki(1);
		}
		
		Set_Att_2level_Ki(1);
}
/* 一键翻滚触发入口 */
void one_key_roll()
{
			if(flag.flying && flag.auto_take_off_land == AUTO_TAKE_OFF_FINISH)
			{	
				if(rolling_flag.roll_mode==0)
				{
					rolling_flag.roll_mode = 1;
					
				}
			}
}
static u16 s_one_key_takeoff_delay_ms;
/* 一键起飞延时任务 */
void one_key_take_off_task(u16 dt_ms)
{
	if(s_one_key_takeoff_delay_ms != 0)
	{
		s_one_key_takeoff_delay_ms += dt_ms;
		
		
		if(s_one_key_takeoff_delay_ms > TAKEOFF_DELAY_MS && flag.motor_preparation == 1)
		{
			s_one_key_takeoff_delay_ms = 0;
				if(flag.auto_take_off_land == AUTO_TAKE_OFF_NULL)
				{
					flag.auto_take_off_land = AUTO_TAKE_OFF;
					//进入起飞状态
					flag.taking_off = 1;
				}
			
		}
	}
	if(flag.unlock_sta == 0)
	{
		s_one_key_takeoff_delay_ms = 0;
	}
}
/* 一键起飞触发 */
void one_key_take_off()
{
	if(flag.unlock_err == 0)
	{	
		if(flag.auto_take_off_land == AUTO_TAKE_OFF_NULL && s_one_key_takeoff_delay_ms == 0)
		{
			s_one_key_takeoff_delay_ms = 1;
			flag.unlock_cmd = 1;
		}
	}
}
/* 一键降落触发 */
void one_key_land()
{
	flag.auto_take_off_land = AUTO_LAND;
}
/* 急停/终止任务 */
void Sudden_Stop_Task(void)
{
    flag.unlock_cmd = 0;
    Program_Ctrl_User_Set_HXYcmps(0, 0);
    Program_Ctrl_User_Set_YAWdps(0);
    FlyCtrlReset();
}
_flight_state_st fs;
static s16 flying_cnt, landing_cnt;
static s16 s_land_detect_delay_ms ;
/* 落地状态判定 */
static void LandDiscriminate(s16 dT_ms)
{
	
	
	/*油门归一值<0.1 或处于自动降落*/  
	if((fs.speed_set_h_norm[Z] < 0.1f) || flag.auto_take_off_land == AUTO_LAND)
	{
		if(s_land_detect_delay_ms>0)
		{
			s_land_detect_delay_ms -= dT_ms;
		}
	}
	else
	{
		s_land_detect_delay_ms = LAND_DETECT_DELAY_MS;
	}
	
	/*延时200ms后，若油门低或自动降落则开始落地判定*/	
	if(s_land_detect_delay_ms <= 0 && (flag.thr_low || flag.auto_take_off_land == AUTO_LAND) )
	{
		/*油门输出<250且已解锁且非锁定，持续1.5秒判定落地*/
		if(mc.throttle<LAND_THR_THRESHOLD && flag.unlock_sta == 1 && flag.locking != 2)
		{
			if(landing_cnt<LAND_DURATION_MS)
			{
				landing_cnt += dT_ms;
			}
			else
			{
				flying_cnt = 0;
				flag.taking_off = 0;
					landing_cnt =0;	
					flag.unlock_cmd =0;				
				flag.flying = 0;
			}
		}
		else
		{
			landing_cnt = 0;
		}
			
		
	}
	else
	{
		landing_cnt  = 0;
	}
}
/* 飞行状态管理 */
void Flight_State_Task(u8 dT_ms)
{
	s16 thr_deadzone;
	static float max_speed_lim,vel_z_tmp[2];
	/*获取油门遥杆量*/
	thr_deadzone = (flag.wifi_ch_en != 0) ? 0 : 50;
	fs.speed_set_h_norm[Z] = my_deadzone(RC_GetChannel(CH_THR),0,thr_deadzone) *0.0023f;
	fs.speed_set_h_norm_lpf[Z] += 0.5f *(fs.speed_set_h_norm[Z] - fs.speed_set_h_norm_lpf[Z]);
	
	/*起飞检测*/
	if(flag.unlock_sta)
	{	
		if(fs.speed_set_h_norm[Z]>0.01f && flag.motor_preparation == 1) // 0-1
		{
			flag.taking_off = 1;
		}	
	}
	fc_stv.vel_limit_z_p = MAX_Z_SPEED_UP;
	fc_stv.vel_limit_z_n = -MAX_Z_SPEED_DW;
	if( flag.taking_off )
	{
			
		if(flying_cnt<FLY_CONFIRM_MS)
		{
			flying_cnt += dT_ms;
		}
		else
		{
			/*起飞后1秒，认为已经在飞行*/
			flag.flying = 1;  
		}
		
		if(fs.speed_set_h_norm[Z]>0)
		{
			/*获取上升速度*/
			vel_z_tmp[0] = (fs.speed_set_h_norm_lpf[Z] *MAX_Z_SPEED_UP);
		}
		else
		{
			/*获取下降速度*/
			vel_z_tmp[0] = (fs.speed_set_h_norm_lpf[Z] *MAX_Z_SPEED_DW);
		}
		//飞控系统Z速度目标量综合设定
		vel_z_tmp[1] = vel_z_tmp[0] + program_ctrl.vel_cmps_h[Z] + pc_user.vel_cmps_set_z;
		//
		vel_z_tmp[1] = LIMIT(vel_z_tmp[1],fc_stv.vel_limit_z_n,fc_stv.vel_limit_z_p);
		//
		fs.speed_set_h[Z] += LIMIT((vel_z_tmp[1] - fs.speed_set_h[Z]),-0.8f,0.8f);//缓慢趋近目标速度
	}
	else
	{
		fs.speed_set_h[Z] = 0 ;
	}
	float speed_set_tmp[2];
	/*速度设定，参考ANO坐标系*/
	fs.speed_set_h_norm[X] = (my_deadzone(+RC_GetChannel(CH_PIT),0,50) *0.0022f);
	fs.speed_set_h_norm[Y] = (my_deadzone(-RC_GetChannel(CH_ROL),0,50) *0.0022f);
		
	LPF_1_(3.0f,dT_ms*1e-3f,fs.speed_set_h_norm[X],fs.speed_set_h_norm_lpf[X]);
	LPF_1_(3.0f,dT_ms*1e-3f,fs.speed_set_h_norm[Y],fs.speed_set_h_norm_lpf[Y]);
	
	max_speed_lim = MAX_SPEED;
	
	if(switchs.of_flow_on && !switchs.gps_on )
	{
		max_speed_lim = 1.5f *wcz_hei_fus.out;
		max_speed_lim = LIMIT(max_speed_lim,50,150);
	}	
	
	fc_stv.vel_limit_xy = max_speed_lim;
	
	//飞控系统XY速度目标量综合设定
	speed_set_tmp[X] = fc_stv.vel_limit_xy *fs.speed_set_h_norm_lpf[X] + program_ctrl.vel_cmps_h[X] + pc_user.vel_cmps_set_h[X];
	speed_set_tmp[Y] = fc_stv.vel_limit_xy *fs.speed_set_h_norm_lpf[Y] + program_ctrl.vel_cmps_h[Y] + pc_user.vel_cmps_set_h[Y];
	
	length_limit(&speed_set_tmp[X],&speed_set_tmp[Y],fc_stv.vel_limit_xy,fs.speed_set_h_cms);
	fs.speed_set_h[X] = fs.speed_set_h_cms[X];
	fs.speed_set_h[Y] = fs.speed_set_h_cms[Y];	
	
	/*调用落地判定函数*/
	LandDiscriminate(dT_ms);
	
	/*倾斜过大保护*/
	if(rolling_flag.rolling_step == ROLL_END)
	{
		if(imu_data.z_vec[Z] < TILT_PROTECT_COS) /* 倾斜超过约75度则紧急锁定 */
		{
			//
			if(mag.mag_CALIBRATE==0)
			{
				imu_state.G_reset = 1;
			}
			flag.unlock_cmd = 0;
		}
	}	
	/*校准中，复位姿态解算*/
	if(sensor.gyr_CALIBRATE != 0 || sensor.acc_CALIBRATE != 0 ||sensor.acc_z_auto_CALIBRATE)
	{
		imu_state.G_reset = 1;
	}
	
	/*复位期间认为传感器失效*/
	if(imu_state.G_reset == 1)
	{
		flag.sensor_imu_ok = 0;
		LED_STA.rst_imu = 1;
		WCZ_Data_Reset(); //复位高度数据融合
	}
	else if(imu_state.G_reset == 0)
	{	
		if(flag.sensor_imu_ok == 0)
		{
			flag.sensor_imu_ok = 1;
			LED_STA.rst_imu = 0;
			ANO_DT_SendString("IMU OK!");
		}
	}
	
	/*飞行状态复位*/
	if(flag.unlock_sta == 0)
	{
		flag.flying = 0;
		landing_cnt = 0;
		flag.taking_off = 0;
		flying_cnt = 0;
		
		
		flag.rc_loss_back_home = 0;
		
		if(flag.taking_off == 0)
		{
		}
	}
	
}
//
static u8 of_quality_ok;
static u16 of_quality_delay;
//
static u8 of_alt_ok;
static s16 of_alt_delay;
//
static u8 of_tof_on_tmp;
//
_judge_sync_data_st jsdata;
/* 状态切换判断任务 */
void Swtich_State_Task(u8 dT_ms)
{
	switchs.baro_on = 1;
	//光流模块
	if(sens_hd_check.of_ok || sens_hd_check.of_df_ok)
	{
		//
		if(sens_hd_check.of_ok)
		{
			jsdata.of_qua = OF_QUALITY;
			jsdata.of_alt = (u16)OF_ALT;
		}
		else if(sens_hd_check.of_df_ok)
		{
			jsdata.of_qua = of_rdf.quality;
			jsdata.of_alt = Laser_height_cm;
		}
		
		//
		if(jsdata.of_qua>OF_QUALITY_THRESHOLD )//光流质量>阈值，延时后标记可用
		{
			if(of_quality_delay<OF_QUALITY_DELAY_MS)
			{
				of_quality_delay += dT_ms;
			}
			else
			{
				of_quality_ok = 1;
			}
		}
		else
		{
			of_quality_delay =0;
			of_quality_ok = 0;
		}
		
		if(jsdata.of_alt<OF_MAX_ALT_CM)
		{
			//		
			jsdata.valid_of_alt_cm = jsdata.of_alt;
			//延时判断高度数据是否有效
			if(of_alt_delay<OF_ALT_DELAY_MS)
			{
				of_alt_delay += dT_ms;			
			}
			else
			{
				//判断高度有效
				of_alt_ok = 1;
				of_tof_on_tmp = 1;
			}
		}
		else
		{
			//
			if(of_alt_delay>0)
			{
				of_alt_delay -= dT_ms;	
			}
			else
			{
				//判断高度无效
				of_alt_ok = 0;
				of_tof_on_tmp = 0;
			}				
		}
		//
		
		
		//
		if(flag.flight_mode == LOC_HOLD)
		{		
			if(of_alt_ok && of_quality_ok)
			{
				switchs.of_flow_on = 1;
			}
			else
			{
				switchs.of_flow_on = 0;
			}
		}
		else
		{
			of_tof_on_tmp = 0;
			switchs.of_flow_on = 0;
		}	
		//
		switchs.of_tof_on = of_tof_on_tmp;
	}
	else
	{
		switchs.of_flow_on = switchs.of_tof_on = 0;
	}
	
	//激光模块
	switchs.tof_on = 0;
	
	//GPS	
	
	
	//UWB
	if(uwb_data.online && flag.flight_mode == LOC_HOLD)
	{
		switchs.uwb_on = 1;
	}
	else
	{
		switchs.uwb_on = 0;
	}
	
	
	//OPMV
	if(opmv.offline==0 && flag.flight_mode == LOC_HOLD)
	{
		switchs.opmv_on = 1;
	}
	else
	{
		switchs.opmv_on = 0;
	}
}
static u8 speed_mode_old = 255;
static u8 flight_mode_old = 255;
/* 飞行模式参数设置 */
void Flight_Mode_Set(u8 dT_ms)
{
	if(speed_mode_old != flag.speed_mode) //速度模式状态改变
	{
		speed_mode_old = flag.speed_mode;
	}
	/* AUX1 通道用于飞行模式切换 */
	//CH_N[]+1500为单位表示通道值
	s16 aux1 = RC_GetChannel(AUX1);
	if(aux1 <-100 && aux1>-200)//接收机失联值，需要手工设置遥控器
	{
		//遥控器设置的接收机通道失联关闭安全信号。
		flag.chn_failsafe = 1;
	}
	else
	{
		flag.chn_failsafe = 0;
		if(aux1<-300)
		{
			flag.flight_mode = ATT_STAB;
		}
		else if(aux1<200)
		{
			flag.flight_mode = LOC_HOLD;
		}
		else
		{
			flag.flight_mode = SUDDEN_STOP;
		}
	}
	//
	if(flight_mode_old != flag.flight_mode) //摇杆对应模式状态改变
	{
		flight_mode_old = flag.flight_mode;
		
		flag.rc_loss_back_home = 0;
	}
	//
	if(flag.rc_loss ==0)//接收机有信号
	{
		//CH_N[]+1500为单位表示通道值
		s16 aux2 = RC_GetChannel(AUX2);
		if(aux2<-300)
		{
			flag.flight_mode2 = 0;
		}
		else if(aux2<200)
		{
			flag.flight_mode2 = 1;
		}
		else//>=1700
		{
			flag.flight_mode2 = 2;
		}
	}
	else
	{
		flag.flight_mode2 = 0;
	}
		
}
