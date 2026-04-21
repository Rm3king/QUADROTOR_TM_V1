#ifndef _DRV_LED_H_
#define _DRV_LED_H_

#include "sysconfig.h"

/*
 * 模块名称：Drv_Led
 * 模块职责：提供底板 LED 初始化与开关控制接口。
 */

void Dvr_LedInit(void);
void Drv_LedOnOff(u8 led, u8 onoff);

#endif
