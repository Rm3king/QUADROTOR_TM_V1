#ifndef __DRV_UART_H__
#define __DRV_UART_H__

#include "sysconfig.h"

/*
 * UART 驱动模块
 *
 * 提供 5 路串口的初始化与发送接口，按外设语义命名。
 * 内部使用统一的环形缓冲结构，发送和中断检查逻辑不再重复。
 *
 * 硬件映射：
 *   GPS    -> TM4C UART0 (PA0/PA1)
 *   数传   -> TM4C UART4 (PC4/PC5)
 *   OpenMV -> TM4C UART2 (PD6/PD7)
 *   光流   -> TM4C UART7 (PE0/PE1)
 *   激光   -> TM4C UART5 (PE4/PE5)
 */

/* GPS 串口 */
void Drv_UartGps_Init(uint32_t baudrate);
void Drv_UartGps_SendBuf(u8 *data, u8 len);

/* 数传串口（ANO 地面站） */
void Drv_UartDt_Init(uint32_t baudrate);
void Drv_UartDt_SendBuf(u8 *data, u8 len);

/* OpenMV 串口 */
void Drv_UartOpenMv_Init(uint32_t baudrate);
void Drv_UartOpenMv_SendBuf(u8 *data, u8 len);

/* 光流串口 */
void Drv_UartOpticalFlow_Init(uint32_t baudrate);
void Drv_UartOpticalFlow_SendBuf(u8 *data, u8 len);

/* 激光测距串口 */
void Drv_UartLaser_Init(uint32_t baudrate);
void Drv_UartLaser_SendBuf(u8 *data, u8 len);

#endif
