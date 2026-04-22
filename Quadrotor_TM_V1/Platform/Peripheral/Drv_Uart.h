#ifndef _DRV_UART_H_
#define _DRV_UART_H_
#include "sysconfig.h"
/*
 * 模块名称：Drv_Uart
 * 模块职责：提供底板串口 1~5 的初始化、发送与发送检查接口。
 * 命名说明：Drv_Uart1~5 对应底板串口编号，不等同于 TM4C 片上 UART 编号。
 * 使用约束：本轮仅澄清接口语义，不修改寄存器配置、中断流程和收发逻辑。
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

/* 语义化兼容别名：用于逐步替代仅按底板编号命名的旧接口。 */
#define Drv_UartGps_Init				Drv_Uart1Init
#define Drv_UartGps_SendBuf				Drv_Uart1SendBuf
#define Drv_UartDt_Init					Drv_Uart2Init
#define Drv_UartDt_SendBuf				Drv_Uart2SendBuf
#define Drv_UartOpenMv_Init				Drv_Uart3Init
#define Drv_UartOpenMv_SendBuf			Drv_Uart3SendBuf
#define Drv_UartOpticalFlow_Init		Drv_Uart4Init
#define Drv_UartOpticalFlow_SendBuf		Drv_Uart4SendBuf
#define Drv_UartLaser_Init				Drv_Uart5Init
#define Drv_UartLaser_SendBuf			Drv_Uart5SendBuf

#endif
