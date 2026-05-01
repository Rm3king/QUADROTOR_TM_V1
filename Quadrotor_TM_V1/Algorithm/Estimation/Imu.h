#ifndef __IMU_H__
#define __IMU_H__
#include "FcData.h"

/* 姿态解算结果与中间量 */
typedef struct
{
    float w;                    /* 四元数实部 */
    float x;                    /* 四元数 i 分量 */
    float y;                    /* 四元数 j 分量 */
    float z;                    /* 四元数 k 分量 */

    float x_vec[VEC_XYZ];      /* 载体 x 轴在世界坐标的方向 */
    float y_vec[VEC_XYZ];      /* 载体 y 轴在世界坐标的方向 */
    float z_vec[VEC_XYZ];      /* 载体 z 轴在世界坐标的方向 (近似重力方向) */
    float hx_vec[VEC_XYZ];     /* x_vec 投影到水平面的单位向量 (航向) */

    float a_acc[VEC_XYZ];      /* 载体坐标运动加速度 (cm/s^2) */
    float w_acc[VEC_XYZ];      /* 世界坐标运动加速度 */
    float h_acc[VEC_XYZ];      /* 航向坐标运动加速度 */
    float w_mag[VEC_XYZ];      /* 世界坐标磁场向量 */
    float gacc_deadzone[VEC_XYZ]; /* 重力加速度误差死区 */
    float obs_acc_w[VEC_XYZ];  /* 观测运动加速度 (世界坐标, 由导航模块写入) */
    float obs_acc_a[VEC_XYZ];  /* 观测运动加速度 (载体坐标) */
    float gra_acc[VEC_XYZ];    /* 扣除运动分量后的重力加速度 */
    float est_acc_a[VEC_XYZ];  /* 估计加速度 (载体) */
    float est_acc_h[VEC_XYZ];  /* 估计加速度 (航向) */
    float est_acc_w[VEC_XYZ];  /* 估计加速度 (世界) */
    float est_speed_h[VEC_XYZ]; /* 估计速度 (航向) */
    float est_speed_w[VEC_XYZ]; /* 估计速度 (世界) */

    float rol;                  /* 横滚角 (度) */
    float pit;                  /* 俯仰角 (度) */
    float yaw;                  /* 偏航角 (度) */
} _imu_st;
extern _imu_st imu_data;

/* 姿态解算使能与增益配置 */
typedef struct
{
    float gkp;     /* 重力修正比例增益 */
    float gki;     /* 重力修正积分增益 */
    float mkp;     /* 磁力计修正比例增益 */
    float drag_p;  /* 阻力补偿增益 */
    u8 G_reset;    /* 重力对准复位标志 (1=正在快速对准) */
    u8 M_reset;    /* 磁力计对准复位标志 */
    u8 G_fix_en;   /* 重力方向修正使能 */
    u8 M_fix_en;   /* 磁力计航向修正使能 */
    u8 obs_en;     /* 运动加速度观测补偿使能 */
} _imu_state_st;
extern _imu_state_st imu_state;

void IMU_update(float dT, _imu_state_st *state, float gyr[VEC_XYZ], s32 acc[VEC_XYZ], s16 mag_val[VEC_XYZ], _imu_st *imu);
void calculate_RPY(void);
void w2h_2d_trans(float w[VEC_XYZ], float ref_ax[VEC_XYZ], float h[VEC_XYZ]);
void h2w_2d_trans(float h[VEC_XYZ], float ref_ax[VEC_XYZ], float w[VEC_XYZ]);

#endif
