/*
 * 模块名称：LocCtrl
 * 模块职责：声明水平位置控制的数据结构与控制接口。
 * 使用约束：坐标含义和控制输出语义保持不变。
 */
#ifndef __LOC_CTRL_H__
#define __LOC_CTRL_H__
#include "FcData.h"
#include "Filter.h"
#include "Math.h"
#include "Pid.h"

typedef struct
{
	float exp[VEC_XYZ];
	float fb[VEC_XYZ];
	float out[VEC_XYZ];
} _loc_ctrl_st;
extern _loc_ctrl_st loc_ctrl_1;
extern _loc_ctrl_st loc_ctrl_2;

extern _PID_arg_st loc_arg_1[];
extern _PID_val_st loc_val_1[];

void Loc_1level_PID_Init(void);
void Loc_1level_Ctrl(u16 dT_ms, s16 *CH_N);

#endif
