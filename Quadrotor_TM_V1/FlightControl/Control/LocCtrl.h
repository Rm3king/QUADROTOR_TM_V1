#ifndef __WXY_CTRL_H
#define __WXY_CTRL_H
#include "FcData.h"
#include "Filter.h"
#include "Math.h"
#include "Pid.h"

/* 水平位置/速度控制量 */
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
