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
 * ģ�����ƣ�AltCtrl
 * ģ��ְ��ִ���Զ��𽵡��߶��⻷�͸߶��ٶ��ڻ����ơ�
 * ʹ��Լ�����Զ���״̬����PID �ṹ�Ϳ���������屣�ֲ��䡣
 */
static s16 s_auto_takeoff_speed_cmps;
#define AUTO_TAKE_OFF_KP 2.0f
/* �Զ����/�������̹��� */
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
		//�����������ٶ�
		s16 max_take_off_vel = LIMIT(g_fc_param.set.auto_take_off_speed,20,200);
		//
		take_off_ok_cnt += dT_ms;
		s_auto_takeoff_speed_cmps = AUTO_TAKE_OFF_KP *(g_fc_param.set.auto_take_off_height - wcz_hei_fus.out);
		//��������ٶ�
		s_auto_takeoff_speed_cmps = LIMIT(s_auto_takeoff_speed_cmps,0,max_take_off_vel);
		
		//�˳������������1������߶Ȼ�������ʱ�����5000���롣
		if(take_off_ok_cnt>=5000 || (g_fc_param.set.auto_take_off_height - loc_ctrl_2.exp[Z] <2))//(auto_ref_height>AUTO_TAKE_OFF_HEIGHT)
		{
			flag.auto_take_off_land = AUTO_TAKE_OFF_FINISH;
			
			
		}
		//�˳������������2��2000������ж��û����ڿ������š�
		if(take_off_ok_cnt >2000 && ABS(fs.speed_set_h_norm[Z])>0.1f)// һ���Ѿ�taking_off,��������Ƹˣ��˳��������
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
		//�����Զ��½��ٶ�
		s_auto_takeoff_speed_cmps = -(s16)LIMIT(g_fc_param.set.auto_landing_speed,20,200);
	}
}
_PID_arg_st alt_arg_2;
_PID_val_st alt_val_2;
/*�߶Ȼ�PID������ʼ��*/
void Alt_2level_PID_Init()
{
	alt_arg_2.kp = g_fc_param.set.pid_alt_2level[KP];
	alt_arg_2.ki = g_fc_param.set.pid_alt_2level[KI];
	alt_arg_2.kd_ex = 0.00f;
	alt_arg_2.kd_fb = g_fc_param.set.pid_alt_2level[KD];
	alt_arg_2.k_ff = 0.0f;
}
/* �߶��⻷�������� */
void Alt_2level_Ctrl(float dT_s)
{
	Auto_Take_Off_Land_Task(1000*dT_s);
	
	fs.alt_ctrl_speed_set = fs.speed_set_h[Z] + s_auto_takeoff_speed_cmps;
	//
	loc_ctrl_2.exp[Z] += fs.alt_ctrl_speed_set *dT_s;
	loc_ctrl_2.exp[Z] = LIMIT(loc_ctrl_2.exp[Z],loc_ctrl_2.fb[Z]-200,loc_ctrl_2.fb[Z]+200);
	//
	loc_ctrl_2.fb[Z] = (s32)wcz_hei_fus.out;
	if(fs.alt_ctrl_speed_set != 0)
	{
		flag.ct_alt_hold = 0;
	}
	else
	{
		if(ABS(loc_ctrl_1.exp[Z] - loc_ctrl_1.fb[Z])<20)
		{
			flag.ct_alt_hold = 1;
		}
	}
	if(flag.taking_off == 1)
	{
		PID_calculate( dT_s,            //���ڣ���λ���룩
						0,				//ǰ��ֵ
						loc_ctrl_2.exp[Z],				//����ֵ���趨ֵ��
						loc_ctrl_2.fb[Z],			//����ֵ����
						&alt_arg_2, //PID�����ṹ��
						&alt_val_2,	//PID���ݽṹ��
						100,//��������޷�
						0
						 );
	}
	else
	{
		loc_ctrl_2.exp[Z] = loc_ctrl_2.fb[Z];
		alt_val_2.out = 0;
		
	}
	
	alt_val_2.out  = LIMIT(alt_val_2.out,-150,150);
}
_PID_arg_st alt_arg_1;
_PID_val_st alt_val_1;
/*�߶��ٶȻ�PID������ʼ��*/
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
/* �߶��ٶ��ڻ��������� */
void Alt_1level_Ctrl(float dT_s)
{
	u8 out_en;
	out_en = (flag.taking_off != 0) ? 1 : 0;
	
	flag.thr_mode = THR_AUTO;
	
	loc_ctrl_1.exp[Z] = 0.6f *fs.alt_ctrl_speed_set + alt_val_2.out;//�ٶ�ǰ��0.6f��ֱ�Ӹ��ٶ�
	
	w_acc_z_lpf += 0.2f *(imu_data.w_acc[Z] - w_acc_z_lpf); //��ͨ�˲�
	loc_ctrl_1.fb[Z] = wcz_spe_fus.out + g_fc_param.set.pid_alt_1level[KD] *w_acc_z_lpf;//΢�����У��±�PID����΢��ϵ��Ϊ0
	
	
	PID_calculate( dT_s,            //���ڣ���λ���룩
					0,				//ǰ��ֵ
					loc_ctrl_1.exp[Z],				//����ֵ���趨ֵ��
					loc_ctrl_1.fb[Z] ,			//����ֵ����
					&alt_arg_1, //PID�����ṹ��
					&alt_val_1,	//PID���ݽṹ��
					100,//��������޷�
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
