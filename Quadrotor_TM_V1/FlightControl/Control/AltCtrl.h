/*
 * 模块名称：AltCtrl
 * 模块职责：声明高度控制和自动起降相关接口。
 * 使用约束：对外接口保持稳定，不在头文件中暴露实现细节。
 */
#ifndef __ALT_CTRL_H__
#define __ALT_CTRL_H__
#include "FcData.h"
#include "Filter.h"
#include "Math.h"
#include "Pid.h"

void Alt_1level_Ctrl(float dT_s);
void Alt_1level_PID_Init(void);
void Alt_2level_PID_Init(void);
void Alt_2level_Ctrl(float dT_s);
void Auto_Take_Off_Land_Task(u8 dT_ms);

#endif
