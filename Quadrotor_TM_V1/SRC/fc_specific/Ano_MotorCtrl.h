#ifndef __MOTOR_CTRL_H
#define __MOTOR_CTRL_H
#include "Ano_FcData.h"
#include "Ano_Pid.h"

/* 电机混控输出量 */
typedef struct
{
    s32 ct_val_rol;
    s32 ct_val_pit;
    s32 ct_val_yaw;
    s32 ct_val_thr;
} _mc_st;
extern _mc_st mc;

extern s16 motor[MOTORSNUM];

void Motor_Ctrl_Task(u8 dT_ms);

#endif
