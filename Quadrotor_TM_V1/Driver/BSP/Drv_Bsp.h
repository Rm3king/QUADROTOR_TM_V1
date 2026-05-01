#ifndef _DRV_BSP_H_
#define _DRV_BSP_H_

#include "sysconfig.h"

/*
 * 模块名称：Drv_Bsp
 * 模块职责：提供板级初始化、时基延时和系统运行时间接口。
 * 使用约束：不改动时基实现，仅补齐接口说明。
 */

void Drv_BspInit(void);
void MyDelayMs(u32 time);
void SysTick_Init(void );
uint32_t GetSysRunTimeMs(void);
uint32_t GetSysRunTimeUs(void);
extern u8 of_init_type;
#endif
