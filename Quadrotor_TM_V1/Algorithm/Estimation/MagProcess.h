#ifndef __MAG_PROCESS_H__
#define __MAG_PROCESS_H__

/*
 * 模块名称：MagProcess
 * 模块职责：处理磁力计校准、磁场补偿和运行时磁场数据输出。
 * 使用约束：校准步骤、偏置计算和增益计算流程保持不变。
 */
#include "FcData.h"
#include "Filter.h"
#include "Math.h"

typedef struct
{
	u8 mag_CALIBRATE;
	s16 val[VEC_XYZ];

} _mag_cal_st;
extern _mag_cal_st mag;

void Mag_Data_Deal_Task(u8 dT_ms, s16 mag_in[], float z_vec_z, float gyro_deg_x, float gyro_deg_z);
#endif
