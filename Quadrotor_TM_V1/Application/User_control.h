#ifndef _USR_CONTROL_H_
#define _USR_CONTROL_H_

#include "sysconfig.h"

/*
 * 模块名称：User_control
 * 模块职责：暴露任务检查、启动状态读取和任务复位接口。
 */

void InspectionTask(u8 dT_ms);
uint8_t Get_start_flag(void);
void reset_mission(void);

#endif
