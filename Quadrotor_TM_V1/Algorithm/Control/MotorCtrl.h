#ifndef __MOTOR_CTRL_H
#define __MOTOR_CTRL_H
#include "FcData.h"
#include "Pid.h"

/* 电机混控输入量：姿态控制环输出 + 油门 */
typedef struct
{
    s32 roll;
    s32 pitch;
    s32 yaw;
    s32 throttle;
} _motor_ctrl_st;
extern _motor_ctrl_st mc;

extern s16 motor[MOTORSNUM];

void Motor_Ctrl_Task(u8 dT_ms);

#endif
