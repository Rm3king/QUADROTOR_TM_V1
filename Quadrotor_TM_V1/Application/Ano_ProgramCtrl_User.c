#include "Ano_ProgramCtrl_User.h"
#include "Ano_Math.h"

#define MAX_PC_XYVEL_CMPS  200
#define MAX_PC_ZVEL_CMPS   150
#define MAX_PC_PAL_DPS     100

/* 用户程控设定缓存。 */
_pc_user_st pc_user;

/*
 * 功能：设置机体系水平速度。
 * 说明：X 对应前后速度，Y 对应左右速度，并统一执行二维限幅。
 */
void Program_Ctrl_User_Set_HXYcmps(float hx_vel_cmps, float hy_vel_cmps)
{
    pc_user.vel_cmps_set_h[0] = hx_vel_cmps;
    pc_user.vel_cmps_set_h[1] = hy_vel_cmps;
    length_limit(&pc_user.vel_cmps_set_h[0], &pc_user.vel_cmps_set_h[1], MAX_PC_XYVEL_CMPS, pc_user.vel_cmps_set_h);
}

/*
 * 功能：设置垂直速度。
 * 说明：写入后立即执行对称限幅。
 */
void Program_Ctrl_User_Set_Zcmps(float z_vel_cmps)
{
    pc_user.vel_cmps_set_z = z_vel_cmps;
    pc_user.vel_cmps_set_z = LIMIT(pc_user.vel_cmps_set_z, -MAX_PC_ZVEL_CMPS, MAX_PC_ZVEL_CMPS);
}

/*
 * 功能：设置航向角速度。
 * 说明：写入后立即执行对称限幅。
 */
void Program_Ctrl_User_Set_YAWdps(float yaw_pal_dps)
{
    pc_user.pal_dps_set = yaw_pal_dps;
    pc_user.pal_dps_set = LIMIT(pc_user.pal_dps_set, -MAX_PC_PAL_DPS, MAX_PC_PAL_DPS);
}
