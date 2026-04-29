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
 * ????????
 * ?????????????????/??????????
 * ???????????????????????????
 */
/* ���п��ƻ� PID ������ʼ�� */
void All_PID_Init(void)
{
	/*��̬���ƣ����ٶ�PID��ʼ��*/
	Att_1level_PID_Init();
	
	/*��̬���ƣ��Ƕ�PID��ʼ��*/
	Att_2level_PID_Init();
	
	/*�߶ȿ��ƣ��߶��ٶ�PID��ʼ��*/
	Alt_1level_PID_Init();	
	
	/*�߶ȿ��ƣ��߶�PID��ʼ��*/
	Alt_2level_PID_Init();
	
	
	/*λ���ٶȿ���PID��ʼ��*/
	Loc_1level_PID_Init();
	
}
/* ���ݷ���״̬�л����Ʋ��� */
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
/* һ������������� */
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
/* һ�������ʱ���� */
void one_key_take_off_task(u16 dt_ms)
{
	if(s_one_key_takeoff_delay_ms != 0)
	{
		s_one_key_takeoff_delay_ms += dt_ms;
		
		
		if(s_one_key_takeoff_delay_ms > 1400 && flag.motor_preparation == 1)
		{
			s_one_key_takeoff_delay_ms = 0;
				if(flag.auto_take_off_land == AUTO_TAKE_OFF_NULL)
				{
					flag.auto_take_off_land = AUTO_TAKE_OFF;
					//���������
					flag.taking_off = 1;
				}
			
		}
	}
	if(flag.unlock_sta == 0)
	{
		s_one_key_takeoff_delay_ms = 0;
	}
}
/* һ����ɴ��� */
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
/* һ�����䴥�� */
void one_key_land()
{
	flag.auto_take_off_land = AUTO_LAND;
}
/* ��ͣ��ֹ���� */
void Sudden_Stop_Task(void)
{
    flag.unlock_cmd = 0;
    Program_Ctrl_User_Set_HXYcmps(0, 0);
    Program_Ctrl_User_Set_YAWdps(0);
    FlyCtrlReset();
}
_flight_state_st fs;
s16 flying_cnt,landing_cnt;
float stop_baro_hpf;
static s16 s_land_detect_delay_ms ;
/* ����״̬�ж� */
static void LandDiscriminate(s16 dT_ms)
{
	
	
	/*���Ź�һֵС��0.1  ���������Զ�����*/
	if((fs.speed_set_h_norm[Z] < 0.1f) || flag.auto_take_off_land == AUTO_LAND)
	{
		if(s_land_detect_delay_ms>0)
		{
			s_land_detect_delay_ms -= dT_ms;
		}
	}
	else
	{
		s_land_detect_delay_ms = 200;
	}
	
	/*�����ǣ���������������ţ�����Ҫ�ȴ�ֱ������ٶ�С��200cm/s2 ����200ms�ſ�ʼ���*/	
	if(s_land_detect_delay_ms <= 0 && (flag.thr_low || flag.auto_take_off_land == AUTO_LAND) )
	{
		/*�������������С��250����û�����ֶ��������������У�����1�룬��Ϊ��½��Ȼ������*/
		if(mc.throttle<250 && flag.unlock_sta == 1 && flag.locking != 2)
		{
			if(landing_cnt<1500)
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
/* ����״̬���� */
void Flight_State_Task(u8 dT_ms,s16 *CH_N)
{
	s16 thr_deadzone;
	static float max_speed_lim,vel_z_tmp[2];
	/*��������ҡ����*/
	thr_deadzone = (flag.wifi_ch_en != 0) ? 0 : 50;
	fs.speed_set_h_norm[Z] = my_deadzone(CH_N[CH_THR],0,thr_deadzone) *0.0023f;
	fs.speed_set_h_norm_lpf[Z] += 0.5f *(fs.speed_set_h_norm[Z] - fs.speed_set_h_norm_lpf[Z]);
	
	/*���������*/
	if(flag.unlock_sta)
	{	
		if(fs.speed_set_h_norm[Z]>0.01f && flag.motor_preparation == 1) // 0-1
		{
			flag.taking_off = 1;
		}	
	}		
	//
	fc_stv.vel_limit_z_p = MAX_Z_SPEED_UP;
	fc_stv.vel_limit_z_n = -MAX_Z_SPEED_DW;	
	//
	if( flag.taking_off )
	{
			
		if(flying_cnt<1000)//800ms
		{
			flying_cnt += dT_ms;
		}
		else
		{
			/*��ɺ�1�룬��Ϊ�Ѿ��ڷ���*/
			flag.flying = 1;  
		}
		
		if(fs.speed_set_h_norm[Z]>0)
		{
			/*���������ٶ�*/
			vel_z_tmp[0] = (fs.speed_set_h_norm_lpf[Z] *MAX_Z_SPEED_UP);
		}
		else
		{
			/*�����½��ٶ�*/
			vel_z_tmp[0] = (fs.speed_set_h_norm_lpf[Z] *MAX_Z_SPEED_DW);
		}
		//�ɿ�ϵͳZ�ٶ�Ŀ�����ۺ��趨
		vel_z_tmp[1] = vel_z_tmp[0] + program_ctrl.vel_cmps_h[Z] + pc_user.vel_cmps_set_z;
		//
		vel_z_tmp[1] = LIMIT(vel_z_tmp[1],fc_stv.vel_limit_z_n,fc_stv.vel_limit_z_p);
		//
		fs.speed_set_h[Z] += LIMIT((vel_z_tmp[1] - fs.speed_set_h[Z]),-0.8f,0.8f);//������������
	}
	else
	{
		fs.speed_set_h[Z] = 0 ;
	}
	float speed_set_tmp[2];
	/*�ٶ��趨���������ο�ANO����ο�����*/
	fs.speed_set_h_norm[X] = (my_deadzone(+CH_N[CH_PIT],0,50) *0.0022f);
	fs.speed_set_h_norm[Y] = (my_deadzone(-CH_N[CH_ROL],0,50) *0.0022f);
		
	LPF_1_(3.0f,dT_ms*1e-3f,fs.speed_set_h_norm[X],fs.speed_set_h_norm_lpf[X]);
	LPF_1_(3.0f,dT_ms*1e-3f,fs.speed_set_h_norm[Y],fs.speed_set_h_norm_lpf[Y]);
	
	max_speed_lim = MAX_SPEED;
	
	if(switchs.of_flow_on && !switchs.gps_on )
	{
		max_speed_lim = 1.5f *wcz_hei_fus.out;
		max_speed_lim = LIMIT(max_speed_lim,50,150);
	}	
	
	fc_stv.vel_limit_xy = max_speed_lim;
	
	//�ɿ�ϵͳXY�ٶ�Ŀ�����ۺ��趨
	speed_set_tmp[X] = fc_stv.vel_limit_xy *fs.speed_set_h_norm_lpf[X] + program_ctrl.vel_cmps_h[X] + pc_user.vel_cmps_set_h[X];
	speed_set_tmp[Y] = fc_stv.vel_limit_xy *fs.speed_set_h_norm_lpf[Y] + program_ctrl.vel_cmps_h[Y] + pc_user.vel_cmps_set_h[Y];
	
	length_limit(&speed_set_tmp[X],&speed_set_tmp[Y],fc_stv.vel_limit_xy,fs.speed_set_h_cms);
	fs.speed_set_h[X] = fs.speed_set_h_cms[X];
	fs.speed_set_h[Y] = fs.speed_set_h_cms[Y];	
	
	/*���ü����½�ĺ���*/
	LandDiscriminate(dT_ms);
	
	/*��б��������*/
	if(rolling_flag.rolling_step == ROLL_END)
	{
		if(imu_data.z_vec[Z] < 0.25f) /* ? 75 ???????????? */
		{
			//
			if(mag.mag_CALIBRATE==0)
			{
				imu_state.G_reset = 1;
			}
			flag.unlock_cmd = 0;
		}
	}	
	/*У׼�У���λ��������*/
	if(sensor.gyr_CALIBRATE != 0 || sensor.acc_CALIBRATE != 0 ||sensor.acc_z_auto_CALIBRATE)
	{
		imu_state.G_reset = 1;
	}
	
	/*��λ��������ʱ����Ϊ������ʧЧ*/
	if(imu_state.G_reset == 1)
	{
		flag.sensor_imu_ok = 0;
		LED_STA.rst_imu = 1;
		WCZ_Data_Reset(); //��λ�߶������ں�
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
	
	/*����״̬��λ*/
	if(flag.unlock_sta == 0)
	{
		flag.flying = 0;
		landing_cnt = 0;
		flag.taking_off = 0;
		flying_cnt = 0;
		
		
		flag.rc_loss_back_home = 0;
		
		//��λ�ں�
		if(flag.taking_off == 0)
		{
//			wxyz_fusion_reset();
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
/* ״̬�л��ж����� */
void Swtich_State_Task(u8 dT_ms)
{
	switchs.baro_on = 1;
	//����ģ��
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
		if(jsdata.of_qua>50 )//|| flag.flying == 0) //������������50 /*�����ڷ���֮ǰ*/����Ϊ�������ã��ж������ӳ�ʱ��Ϊ1��
		{
			if(of_quality_delay<500)
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
		
		//�����߶�600cm����Ч
		if(jsdata.of_alt<600)
		{
			//		
			jsdata.valid_of_alt_cm = jsdata.of_alt;
			//��ʱ1.5���жϼ���߶��Ƿ���Ч
			if(of_alt_delay<1000)
			{
				of_alt_delay += dT_ms;			
			}
			else
			{
				//�ж��߶���Ч
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
				//�ж��߶���Ч
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
	
	//����ģ��
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
static void Speed_Mode_Switch()
{
}
u8 speed_mode_old = 255;
u8 flight_mode_old = 255;
/* ����ģʽ�������� */
void Flight_Mode_Set(u8 dT_ms)
{
	Speed_Mode_Switch();
	
	if(speed_mode_old != flag.speed_mode) //״̬�ı�
	{
		speed_mode_old = flag.speed_mode;
	}
	/* AUX1 ?????????????????????? */
	//CH_N[]+1500Ϊ��λ����ʾͨ��ֵ
	if(CH_N[AUX1] <-100 && CH_N[AUX1]>-200)//���ջ�ʧ��ֵ����Ҫ�ֹ�����ң����
	{
		//ң�����õĽ��ջ������ʧ�ر������źš�
		flag.chn_failsafe = 1;
	}
	else
	{
		flag.chn_failsafe = 0;
		if(CH_N[AUX1]<-300)
		{
			flag.flight_mode = ATT_STAB;
		}
		else if(CH_N[AUX1]<200)
		{
			flag.flight_mode = LOC_HOLD;
		}
		else
		{
			flag.flight_mode = SUDDEN_STOP;
		}
	}
	//
	if(flight_mode_old != flag.flight_mode) //ҡ�˶�Ӧģʽ״̬�ı�
	{
		flight_mode_old = flag.flight_mode;
		
		flag.rc_loss_back_home = 0;
	}
	//
	if(flag.rc_loss ==0)//���ջ����ź�
	{
		//CH_N[]+1500Ϊ��λ����ʾͨ��ֵ
		if(CH_N[AUX2]<-300)//<1200
		{
			flag.flight_mode2 = 0;
		}
		else if(CH_N[AUX2]<200)//1200-1700
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
