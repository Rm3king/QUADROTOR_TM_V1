#ifndef __POWER_H__
#define __POWER_H__

/*
 * 模块名称：Power
 * 模块职责：处理电压采样、低压状态判定和电源相关状态输出。
 * 使用约束：采样触发顺序、滤波逻辑和电压阈值含义保持不变。
 */

#include "Drv_adc.h"

extern float Plane_Votage;

void Power_UpdateTask(u8 dT_ms);

#endif
