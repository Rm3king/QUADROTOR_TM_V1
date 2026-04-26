#ifndef _DRV_UART_H_
#define _DRV_UART_H_

#include "sysconfig.h"

/*
 * 模块名称：Drv_Uart
 * 模块职责：提供 GPS、数传、OpenMV、光流和激光串口的初始化与发送接口。
 * 命名约定：优先使用语义化接口名，底板串口编号接口仅作为兼容层保留。
 * 使用约束：本轮仅整理接口命名与注释，不修改寄存器配置、中断流程和收发行为。
 */

/* GPS 串口：底板串口 1，对应 TM4C UART0。 */
void Drv_UartGps_Init(uint32_t baudrate);
void Drv_UartGps_SendBuf(u8 *data, u8 len);
void Drv_UartGps_TxCheck(void);

/* 数传串口：底板串口 2，对应 TM4C UART4。 */
void Drv_UartDt_Init(uint32_t baudrate);
void Drv_UartDt_SendBuf(u8 *data, u8 len);
void Drv_UartDt_TxCheck(void);

/* OpenMV 串口：底板串口 3，对应 TM4C UART2。 */
void Drv_UartOpenMv_Init(uint32_t baudrate);
void Drv_UartOpenMv_SendBuf(u8 *data, u8 len);
void Drv_UartOpenMv_TxCheck(void);

/* 光流串口：底板串口 4，对应 TM4C UART7。 */
void Drv_UartOpticalFlow_Init(uint32_t baudrate);
void Drv_UartOpticalFlow_SendBuf(u8 *data, u8 len);
void Drv_UartOpticalFlow_TxCheck(void);

/* 激光串口：底板串口 5，对应 TM4C UART5。 */
void Drv_UartLaser_Init(uint32_t baudrate);
void Drv_UartLaser_SendBuf(u8 *data, u8 len);
void Drv_UartLaser_TxCheck(void);

/*
 * 历史兼容接口：
 * Drv_Uart1~5* 沿用底板串口编号命名，保留它们仅用于兼容旧调用点。
 * 新代码请优先使用上面的语义化接口。
 */
void Drv_Uart1Init(uint32_t baudrate);
void Drv_Uart1SendBuf(u8 *data, u8 len);
void Drv_Uart1TxCheck(void);
void Drv_Uart2Init(uint32_t baudrate);
void Drv_Uart2SendBuf(u8 *data, u8 len);
void Drv_Uart2TxCheck(void);
void Drv_Uart3Init(uint32_t baudrate);
void Drv_Uart3SendBuf(u8 *data, u8 len);
void Drv_Uart3TxCheck(void);
void Drv_Uart4Init(uint32_t baudrate);
void Drv_Uart4SendBuf(u8 *data, u8 len);
void Drv_Uart4TxCheck(void);
void Drv_Uart5Init(uint32_t baudrate);
void Drv_Uart5SendBuf(u8 *data, u8 len);
void Drv_Uart5TxCheck(void);

#endif
