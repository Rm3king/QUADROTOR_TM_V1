#ifndef __ANO_PROGRAMCTRL_USER_H
#define __ANO_PROGRAMCTRL_USER_H
#include "sysconfig.h"
#include "Ano_FcData.h"
/*
 * 模块说明。
 * 用户程控速度设定接口。
 * 用于接收上层控制或 OpenMV 任务输出的速度目标，并保持统一限幅入口。
 */
typedef struct
{
    /* 机体系水平速度设定。 */
    float vel_cmps_set_h[2];
    /* 世界系水平速度设定。 */
    float vel_cmps_set_w[2];
    /* 参考系水平速度设定。 */
    float vel_cmps_set_ref[2];
    /* 垂直速度设定。 */
    float vel_cmps_set_z;
    /* 航向角速度设定。 */
    float pal_dps_set;
} _pc_user_st;

extern _pc_user_st pc_user;

/* 功能：设置机体系水平速度，单位 cm/s。 */
void Program_Ctrl_User_Set_HXYcmps(float hx_vel_cmps, float hy_vel_cmps);
/* 功能：设置垂直速度，单位 cm/s。 */
void Program_Ctrl_User_Set_Zcmps(float z_vel_cmps);
/* 功能：设置航向角速度，单位 dps。 */
void Program_Ctrl_User_Set_YAWdps(float yaw_pal_dps);

#endif
