#ifndef __OF_H__
#define __OF_H__

#include "sysconfig.h"

/*
 * 模块：光流
 * 职责：声明匿名光流模块数据变量。
 * 使用约束：仅修改说明，不改变数据结构和接口。
 */

extern uint8_t OF_QUALITY;
/* 16位位移量（含修正值） */
extern int16_t OF_DX2, OF_DY2, OF_DX2FIX, OF_DY2FIX;
/* 光流测距高度 */
extern uint16_t OF_ALT;

/* 逐字节解析光流串口数据，每收 1 字节调用 1 次 */
void AnoOF_GetOneByte(uint8_t data);
void AnoOF_DataAnl_Task(u8 dT_ms);
void AnoOF_Check(u8 dT_ms);

#endif
