#ifndef _ANO_UWB_H_
#define _ANO_UWB_H_

/*
 * 模块名称：Ano_UWB
 * 模块职责：定义 UWB 定位数据、融合状态和对外处理接口。
 * 使用约束：仅整理接口说明，不改变定位数据含义和融合流程。
 */

#include "Filter.h"
#include "Math.h"
#include "Imu.h"
#include "FcData.h"
//==定义
typedef struct
{
	u8 init_ok;
	u8 online;
	
	float ref_dir[2];
	float raw_data_loc[3];
	float raw_data_vel[3];	
	float w_dis_cm[3];
	float w_vel_cmps[3];

}_uwb_data_st;


//==数据声明
extern _uwb_data_st uwb_data;


//==函数声明
void UWB_GetByte(u8 data);
void UWB_GetDataTask(u8 dT_ms);
void UWB_DataCalcTask(u8 dT_ms);





#endif





