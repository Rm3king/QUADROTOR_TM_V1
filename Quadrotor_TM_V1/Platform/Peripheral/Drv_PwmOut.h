#ifndef _DRV_PWMOUT_H_
#define _DRV_PWMOUT_H_

#include "sysconfig.h"

/*
 * 模块名称：Drv_PwmOut
 * 模块职责：提供电机与加热 PWM 输出接口。
 * 使用约束：不修改 PWM 通道映射和输出时序。
 */

void Drv_PwmOutInit(void);
void Drv_MotorPWMSet(uint8_t Motor,uint16_t PwmValue);
void Drv_HeatSet(u16 val);
	
#endif
