#ifndef __OPMV_LINETRACKING_CTRL_H__
#define __OPMV_LINETRACKING_CTRL_H__
#include "sysconfig.h"
#include "FcData.h"
/*
 * 模块说明。
 * OpenMV 寻线控制状态。
 * 保存视觉解耦后的偏差、速度输出和转向输出，供上层程控任务使用。
 */
typedef struct
{
    /* 目标丢失标志。 */
    u8 target_loss;
    /* OpenMV 原始线偏差。 */
    s16 opmv_pos;
    /* 姿态补偿对应的像素偏移。 */
    s16 r2pixel_val;
    /* 解耦后的像素偏差。 */
    float decou_pos_pixel;
    /* 地面位置误差，单位 cm。 */
    float ground_pos_err_h_cm;
    /* 地面位置误差微分，单位 cm/s。 */
    float ground_pos_err_d_h_cmps;
    /* 输出的期望水平速度。 */
    float exp_velocity_h_cmps[2];
    /* 输出的期望航向角速度。 */
    float exp_yaw_pal_dps;
} _ano_opmv_lt_ctrl_st;

extern _ano_opmv_lt_ctrl_st ano_opmv_lt_ctrl;

void ANO_LTracking_Task(u8 dT_ms);
void ANO_LTracking_Ctrl(u8 *dT_ms, u8 en);

#endif
