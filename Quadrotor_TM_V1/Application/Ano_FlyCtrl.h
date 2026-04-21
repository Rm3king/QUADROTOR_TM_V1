#ifndef _FLY_CTRL_H
#define _FLY_CTRL_H
#include "sysconfig.h"
/*
 * 模块说明。
 * 程控飞行命令执行状态。
 * 用于保存上位机动作指令、执行时长反馈和速度参考值，不改变原有命令语义。
 */
typedef struct
{
    /* 当前指令是否允许执行。 */
    u8 state_ok;
    /* 命令当前值与上一拍值，用于检测命令切换。 */
    u8 cmd_state[2];
    /* 已执行时间反馈，单位 ms。 */
    u32 fb_process_t_ms[4];
    /* 期望执行时间，单位 ms。 */
    u32 exp_process_t_ms[4];
    /* 参考方向向量。 */
    float ref_dir[2];
    /* 地理坐标系速度指令。 */
    float vel_cmps_ref[3];
    /* 世界坐标系速度指令。 */
    float vel_cmps_w[3];
    /* 机体系速度指令。 */
    float vel_cmps_h[3];
    /* 航向角速度指令，单位 dps。 */
    s16 yaw_pal_dps;
} _fly_ct_st;

extern _fly_ct_st program_ctrl;

/* 功能：复位程控命令状态。 */
void FlyCtrlReset(void);
/* 功能：解析程控命令数据帧。 */
void FlyCtrlDataAnl(u8 *data);
/* 功能：20ms 周期执行程控动作。 */
void FlyCtrl_Task(u8 dT_ms);

#endif
