#ifndef __OPMV_CBTRACKING_CTRL_H__
#define __OPMV_CBTRACKING_CTRL_H__
#include "sysconfig.h"
#include "FcData.h"
/*
 * 模块说明。
 * OpenMV 色块跟踪控制状态。
 * 保存视觉解耦、地面误差和输出速度等中间量，供上层控制任务读取。
 */
typedef struct
{
    /* 目标丢失标志。 */
    u8 target_loss;
    /* OpenMV 原始目标位置。 */
    s16 opmv_pos[2];
    /* 姿态补偿对应的像素偏移。 */
    s16 rp2pixel_val[2];
    /* 解耦后的像素位置。 */
    float decou_pos_pixel[2];
    /* 地面位置误差，单位 cm。 */
    float ground_pos_err_h_cm[2];
    /* 地面位置误差微分，单位 cm/s。 */
    float ground_pos_err_d_h_cmps[2];
    /* 目标地面速度估计，单位 cm/s。 */
    float target_gnd_velocity_cmps[2];
    /* 输出的期望水平速度，单位 cm/s。 */
    float exp_velocity_h_cmps[2];
} _ano_opmv_cbt_ctrl_st;

extern _ano_opmv_cbt_ctrl_st ano_opmv_cbt_ctrl;

void ANO_CBTracking_Task(u8 dT_ms);
void ANO_CBTracking_Ctrl(u8 *dT_ms, u8 en);

#endif
