#ifndef __CONFIG_H
#define __CONFIG_H
/*
 * ģ�飺��������
 * ְ�𣺼��ж��������س����͹��̿��ء�
 */
#include "sysconfig.h"

/* 角度与弧度互转 */
#define ANGLE_TO_RADIAN 0.01745329f
#define RAD_TO_DEG      57.2957795f

/* 物理常量 */
#define GRAVITY_CMSS    981         /* 重力加速度 (cm/s^2)，传感器原始单位 */

/* IMU 互补滤波：加速度模值有效范围，超出则不参与姿态修正 */
#define ACC_NORM_MAX    1060        /* ~1.08g，上限 */
#define ACC_NORM_MIN    900         /* ~0.92g，下限 */
#define ANO_DT_USE_NRF24l01
#define SP_EST_DRAG 1.0f
#define BARO_WIND_COMP 0.10f
/* �����˲�����̬���ƾ�������� */
#define GYR_ACC_FILTER 0.25f
#define FINAL_P        0.35f
#define MOTOR_ESC_TYPE 1
#define MOTORSNUM 4
#define BAT_LOW_VOTAGE 3250
#define FLOAW_MAX_HEIGHT 450
#define FLOW_ROLL_CONDITION 8
#define APP_ROLL_CH CH_PIT
#define MAX_ANGLE 25.0f
#define MAX_SPEED_ROL 200
#define MAX_SPEED_PIT 200
#define MAX_SPEED_YAW 250
#define MAX_ROLLING_SPEED 1600
#define MAX_SPEED 500
#define MAX_Z_SPEED_UP 350
#define MAX_Z_SPEED_DW 250
#define MAX_EXP_XY_ACC 500
#define MAX_EXP_Z_ACC 600
#define CTRL_1_INTE_LIM 250
#define ANGULAR_VELOCITY_PID_INTE_D_LIM 300/FINAL_P
#define X_PROPORTION_X_Y 1.0f
#define ROLL_ANGLE_KP 10.0f
#define MAX_THR_SET 85
#define THR_INTE_LIM_SET 70
#define THR_INTE_LIM THR_INTE_LIM_SET/FINAL_P
#define THR_START 35
#define LAND_ACC 500
#define LAND_ACC_DELTA 300
#define BARO_FIX -0
#endif
