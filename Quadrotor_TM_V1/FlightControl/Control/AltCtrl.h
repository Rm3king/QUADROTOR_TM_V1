#ifndef __WZ_CTRL_H
#define __WZ_CTRL_H
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
