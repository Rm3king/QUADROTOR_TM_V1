#ifndef __OF_H__
#define __OF_H__

#include "sysconfig.h"

/*
 * 模块：光流
 * 职责：声明匿名光流模块数据变量。
 * 使用约束：仅修改说明，不改变数据结构和接口。
 */

extern uint8_t OF_STATE, OF_QUALITY;
/* 8位位移量（光流原始输出） */
extern int8_t OF_DX, OF_DY;
/* 16位位移量（含修正值） */
extern int16_t OF_DX2, OF_DY2, OF_DX2FIX, OF_DY2FIX;
/* 光流测距高度 */
extern uint16_t OF_ALT, OF_ALT2;
/* 陀螺仪数据 */
extern int16_t OF_GYR_X, OF_GYR_Y, OF_GYR_Z;
/* 陀螺仪数据2 */
extern int16_t OF_GYR_X2, OF_GYR_Y2, OF_GYR_Z2;
/* 加速度数据 */
extern int16_t OF_ACC_X, OF_ACC_Y, OF_ACC_Z;
/* 加速度数据2 */
extern int16_t OF_ACC_X2, OF_ACC_Y2, OF_ACC_Z2;
/* 姿态角（欧拉角） */
extern float OF_ATT_ROL, OF_ATT_PIT, OF_ATT_YAW;
/* 姿态四元数 */
extern float OF_ATT_S1, OF_ATT_S2, OF_ATT_S3, OF_ATT_S4;

/* 逐字节解析光流串口数据，每收 1 字节调用 1 次 */
void AnoOF_GetOneByte(uint8_t data);
void AnoOF_DataAnl_Task(u8 dT_ms);
void AnoOF_Check(u8 dT_ms);

#endif
