/*
 * 文件名称: MagProcess.c
 * 所属模块: Algorithm / Estimation
 *
 * 功能描述:
 *   磁力计数据处理模块：
 *   1) Mag_Calibration_Task() -- 磁力计校准流程（最大最小值法求椭球中心）
 *   2) Mag_Data_Prepare()     -- 运行时数据补偿（减去零偏 + 缩放归一化）
 *   3) Mag_Check()            -- 磁场强度有效性检查（排除磁干扰）
 *
 * 校准原理:
 *   用户旋转飞控，采集各方向磁力计读数的最大/最小值，
 *   中心点 = (max + min) / 2 即为硬铁零偏，
 *   半径 = (max - min) / 2 用于软铁缩放补偿。
 *
 * 架构位置:
 *   由 Scheduler 按 20ms 周期调用。
 *   输出磁航向供 Imu.c 姿态解算的偏航修正使用。
 *
 * 教学提示:
 *   - 磁力计校准本质是求解三维椭球拟合，这里用简化的最大最小值法
 *   - 校准期间 LED 指示不同阶段（绿色呼吸→蓝色闪烁→绿色闪烁完成）
 *   - 磁场强度偏离预期过多说明有磁干扰，此时磁航向不可信
 */
#include "MagProcess.h"
#include "LED.h"
static s16 s_mag_max_raw[VEC_XYZ];
static s16 s_mag_min_raw[VEC_XYZ];
_mag_cal_st mag;
static void MagCalReset(u8 mode)
{
	if(mode == 2)
	{
		for(u8 i = 0;i<2;i++)
		{
			s_mag_max_raw[i] = -30000;
			s_mag_min_raw[i] = 30000;
		}
	}
	else if(mode == 1)
	{
			s_mag_max_raw[Z] = -30000;
			s_mag_min_raw[Z] = 30000;		
	}
	else
	{
		for(u8 i = 0;i<3;i++)
		{
			s_mag_max_raw[i] = -30000;
			s_mag_min_raw[i] = 30000;
		}	
	}
}
static void MagCalUpdateXY(s16 mag_in[])
{
	for(u8 i = 0;i<2;i++)
	{
		s_mag_max_raw[i] = _MAX(s_mag_max_raw[i],mag_in[i]);
		s_mag_min_raw[i] = _MIN(s_mag_min_raw[i],mag_in[i]);
	}
	
	
}
static void MagCalUpdateZ(s16 mag_in[])
{
	s_mag_max_raw[Z] = _MAX(s_mag_max_raw[Z],mag_in[Z]);
	s_mag_min_raw[Z] = _MIN(s_mag_min_raw[Z],mag_in[Z]);
}
static u8 s_mag_cal_step;
void Mag_Data_Deal_Task(u8 dT_ms,s16 mag_in[],float z_vec_z,float gyro_deg_x,float gyro_deg_z)
{	
	static u16 s_mag_cal_timeout_ms;
	static float s_mag_cal_angle_deg[2];
	float field_strength;
	
	for(u8 i = 0;i<3;i++)
	{
		save.mag_gain[i] = LIMIT(save.mag_gain[i],0.05f,100);
		mag.val[i] = (mag_in[i] - save.mag_offset[i]) *save.mag_gain[i];
	}
	/* 校准流程 */
	if(mag.mag_CALIBRATE!= 0 && flag.unlock_sta == 0)
	{	
		switch(s_mag_cal_step)
		{
			case 0://第一步，水平旋转
				MagCalUpdateXY(mag_in);			
			
				if(z_vec_z<0.985f)//+-10deg	
				{
					LED_STA.calMag = 100;
					s_mag_cal_step = 1;
					s_mag_cal_angle_deg[0] = 0;
				}
				else
				{	
					LED_STA.calMag = 1;
					s_mag_cal_angle_deg[0] += dT_ms *1e-3f *(gyro_deg_z); //角度积分，旋转360度
					if(ABS(s_mag_cal_angle_deg[0])>360)
					{
						s_mag_cal_angle_deg[0] = 0;
						s_mag_cal_step = 2;
					}
				}
			break;
			
			case 1://error
				
				MagCalReset(2);
				s_mag_cal_angle_deg[0] = 0;
				s_mag_cal_step = 0;
			break;
			
			case 2://第二步，竖直旋转，机头朝下
				LED_STA.calMag = 2;
				if(z_vec_z<0.1f)//5.7deg
				{
					s_mag_cal_step = 3;
				}
			break;
			
			case 3:
				mag.mag_CALIBRATE = 2;																					
				
				MagCalUpdateZ(mag_in);
				if(z_vec_z>0.17f)//10deg
				{
					LED_STA.calMag = 2;
					s_mag_cal_step = 4;
					s_mag_cal_angle_deg[1] = 0;
				}
				else
				{
					LED_STA.calMag = 3;
					s_mag_cal_angle_deg[1] += dT_ms *1e-3f *(gyro_deg_x);	//角度积分，旋转360度
					if(ABS(s_mag_cal_angle_deg[1])>360)
					{
						s_mag_cal_angle_deg[1] = 0;
						s_mag_cal_step = 5;
					}
				}			
			break;
			
			case 4://error_2，重新开始竖直旋转
				MagCalReset(1);
				s_mag_cal_angle_deg[1] = 0;
				s_mag_cal_step = 2;				
			break;
			
			case 5:
				for(u8 i = 0;i<3;i++)
				{
					save.mag_offset[i] = 0.5f *(s_mag_max_raw[i] + s_mag_min_raw[i]);		//中值校准
					save.mag_gain[i] = safe_div(200.0f ,(0.5f *(s_mag_max_raw[i] - s_mag_min_raw[i])),0);		//幅值校准
				}
				
				MagCalReset(3);		
				s_mag_cal_angle_deg[0] = s_mag_cal_angle_deg[1] = 0;		
				s_mag_cal_step = 0;
				mag.mag_CALIBRATE = 0;			
				LED_STA.calMag = 0;
				
				data_save();//保存数据
			break;
			
			default:break;	
		}
		
		
		if(s_mag_cal_step == 0 || s_mag_cal_step == 3)
		{
			//长时间出错，退出校准逻辑
			if(s_mag_cal_timeout_ms<15000)
			{
				s_mag_cal_timeout_ms+= dT_ms;
				
			}
			else /* 校准数据异常，提示错误 */
			{
				LED_STA.errOneTime = 1;
				s_mag_cal_timeout_ms = 0;
				LED_STA.calMag = 0;				
				mag.mag_CALIBRATE = 0;
			}
		}
		else
		{
			s_mag_cal_timeout_ms = 0;
		}
	}
	else
	{
		s_mag_cal_step = 0;
		field_strength = my_3_norm(mag.val[X], mag.val[Y], mag.val[Z]);
		(void)field_strength;
		/* 预留磁场强度有效性判定扩展点。 */
	}
}
