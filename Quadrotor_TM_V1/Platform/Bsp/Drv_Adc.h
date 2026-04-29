#ifndef DRV_ADC_H
#define DRV_ADC_H
#include "sysconfig.h"

/*
 * 模块名称：Drv_Adc
 * 模块职责：提供电压采样相关的 ADC 初始化与触发接口。
 */

extern float Voltage;

void Drv_AdcInit(void); 
void Drv_Adc0Trigger(void);

#endif

