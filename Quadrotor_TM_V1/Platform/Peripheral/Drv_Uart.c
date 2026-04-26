#include "Drv_Uart.h"
#include "uart.h"
#include "hw_ints.h"
#include "hw_types.h"
#include "hw_gpio.h"
#include "DT.h"
#include "Drv_OpenMV.h"
#include "Drv_laser.h"
#include "Drv_gps.h"
#include "OF.h"
#include "Drv_UP_Flow.h"

/*
 * 模块名称：Drv_Uart
 * 模块职责：提供各外设串口的初始化、发送缓存和中断接收入口。
 * 命名说明：函数名按外设语义命名，底板串口编号接口仅作为兼容包装保留。
 * 维护约束：本文件不调整 UART 基址、GPIO 复用、中断优先级和收发处理流程。
 */

#define UART_TX_BUF_LEN 256

static u8 s_gps_tx_buf[UART_TX_BUF_LEN];
static u8 s_gps_tx_write_idx = 0;
static u8 s_gps_tx_read_idx = 0;

static u8 s_dt_tx_buf[UART_TX_BUF_LEN];
static u8 s_dt_tx_write_idx = 0;
static u8 s_dt_tx_read_idx = 0;

static u8 s_openmv_tx_buf[UART_TX_BUF_LEN];
static u8 s_openmv_tx_write_idx = 0;
static u8 s_openmv_tx_read_idx = 0;

static u8 s_optical_flow_tx_buf[UART_TX_BUF_LEN];
static u8 s_optical_flow_tx_write_idx = 0;
static u8 s_optical_flow_tx_read_idx = 0;

static u8 s_laser_tx_buf[UART_TX_BUF_LEN];
static u8 s_laser_tx_write_idx = 0;
static u8 s_laser_tx_read_idx = 0;

/* 底板串口 1 中断服务：接收 GPS 数据。 */
void UART1_IRQHandler(void)
{
	uint8_t com_data;
	uint32_t flag = ROM_UARTIntStatus(UART0_BASE, 1);

	ROM_UARTIntClear(UART0_BASE, flag);

	while (ROM_UARTCharsAvail(UART0_BASE))
	{
		com_data = ROM_UARTCharGet(UART0_BASE);
		Drv_GpsGetOneByte(com_data);
	}

	if (flag & UART_INT_TX)
	{
		Drv_UartGps_TxCheck();
	}
}

/* 初始化 GPS 串口，保持原 UART0 / GPIOA 配置不变。 */
void Drv_UartGps_Init(uint32_t baudrate)
{
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

	ROM_GPIOPinConfigure(UART0_RX);
	ROM_GPIOPinConfigure(UART0_TX);
	ROM_GPIOPinTypeUART(UART0_PORT, UART0_PIN_TX | UART0_PIN_RX);

	ROM_UARTConfigSetExpClk(
		UART0_BASE,
		ROM_SysCtlClockGet(),
		baudrate,
		(UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE));

	ROM_UARTFIFOLevelSet(UART0_BASE, UART_FIFO_TX7_8, UART_FIFO_RX7_8);
	ROM_UARTFIFOEnable(UART0_BASE);
	ROM_UARTEnable(UART0_BASE);

	UARTIntRegister(UART0_BASE, UART1_IRQHandler);
	ROM_IntPrioritySet(INT_UART0, USER_INT2);
	ROM_UARTTxIntModeSet(UART0_BASE, UART_TXINT_MODE_EOT);
	ROM_UARTIntEnable(UART0_BASE, UART_INT_RX | UART_INT_RT | UART_INT_TX);
}

/* 发送 GPS 数据，沿用原环形发送缓存行为。 */
void Drv_UartGps_SendBuf(u8 *data, u8 len)
{
	for (u8 i = 0; i < len; i++)
	{
		s_gps_tx_buf[s_gps_tx_write_idx++] = *(data + i);
	}

	Drv_UartGps_TxCheck();
}

/* 检查 GPS 串口发送缓存并尝试继续发送。 */
void Drv_UartGps_TxCheck(void)
{
	while ((s_gps_tx_read_idx != s_gps_tx_write_idx) &&
		   ROM_UARTCharPutNonBlocking(UART0_BASE, s_gps_tx_buf[s_gps_tx_read_idx]))
	{
		s_gps_tx_read_idx++;
	}
}

/* 底板串口 2 中断服务：接收数传数据。 */
void UART2_IRQHandler(void)
{
	uint8_t com_data;
	uint32_t flag = ROM_UARTIntStatus(UART4_BASE, 1);

	ROM_UARTIntClear(UART4_BASE, flag);

	while (ROM_UARTCharsAvail(UART4_BASE))
	{
		com_data = ROM_UARTCharGet(UART4_BASE);
		ANO_DT_Data_Receive_Prepare(com_data);
	}

	if (flag & UART_INT_TX)
	{
		Drv_UartDt_TxCheck();
	}
}

/* 初始化数传串口，保持原 UART4 / GPIOC 配置不变。 */
void Drv_UartDt_Init(uint32_t baudrate)
{
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART4);
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);

	ROM_GPIOPinConfigure(UART4_RX);
	ROM_GPIOPinConfigure(UART4_TX);
	ROM_GPIOPinTypeUART(UART4_PORT, UART4_PIN_TX | UART4_PIN_RX);

	ROM_UARTConfigSetExpClk(
		UART4_BASE,
		ROM_SysCtlClockGet(),
		baudrate,
		(UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE));

	ROM_UARTFIFOLevelSet(UART4_BASE, UART_FIFO_TX7_8, UART_FIFO_RX7_8);
	ROM_UARTFIFOEnable(UART4_BASE);
	ROM_UARTEnable(UART4_BASE);

	UARTIntRegister(UART4_BASE, UART2_IRQHandler);
	ROM_IntPrioritySet(INT_UART4, USER_INT2);
	ROM_UARTTxIntModeSet(UART4_BASE, UART_TXINT_MODE_EOT);
	ROM_UARTIntEnable(UART4_BASE, UART_INT_RX | UART_INT_RT | UART_INT_TX);
}

/* 发送数传数据，沿用原发送缓存行为。 */
void Drv_UartDt_SendBuf(u8 *data, u8 len)
{
	for (u8 i = 0; i < len; i++)
	{
		s_dt_tx_buf[s_dt_tx_write_idx++] = *(data + i);
	}

	Drv_UartDt_TxCheck();
}

/* 检查数传串口发送缓存并尝试继续发送。 */
void Drv_UartDt_TxCheck(void)
{
	while ((s_dt_tx_read_idx != s_dt_tx_write_idx) &&
		   ROM_UARTCharPutNonBlocking(UART4_BASE, s_dt_tx_buf[s_dt_tx_read_idx]))
	{
		s_dt_tx_read_idx++;
	}
}

/* 底板串口 3 中断服务：接收 OpenMV 数据。 */
void UART3_IRQHandler(void)
{
	uint8_t com_data;
	uint32_t flag = ROM_UARTIntStatus(UART2_BASE, 1);

	ROM_UARTIntClear(UART2_BASE, flag);

	while (ROM_UARTCharsAvail(UART2_BASE))
	{
		com_data = ROM_UARTCharGet(UART2_BASE);
		OpenMV_Byte_Get(com_data);
	}

	if (flag & UART_INT_TX)
	{
		Drv_UartOpenMv_TxCheck();
	}
}

/* 初始化 OpenMV 串口，保持原 UART2 / GPIOD 配置不变。 */
void Drv_UartOpenMv_Init(uint32_t baudrate)
{
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART2);
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);

	ROM_GPIOPinConfigure(UART2_RX);
	ROM_GPIOPinConfigure(UART2_TX);
	ROM_GPIOPinTypeUART(UART2_PORT, UART2_PIN_TX | UART2_PIN_RX);

	ROM_UARTConfigSetExpClk(
		UART2_BASE,
		ROM_SysCtlClockGet(),
		baudrate,
		(UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE));

	ROM_UARTFIFOLevelSet(UART2_BASE, UART_FIFO_TX7_8, UART_FIFO_RX7_8);
	ROM_UARTFIFOEnable(UART2_BASE);
	ROM_UARTEnable(UART2_BASE);

	UARTIntRegister(UART2_BASE, UART3_IRQHandler);
	ROM_IntPrioritySet(INT_UART2, USER_INT2);
	ROM_UARTTxIntModeSet(UART2_BASE, UART_TXINT_MODE_EOT);
	ROM_UARTIntEnable(UART2_BASE, UART_INT_RX | UART_INT_RT | UART_INT_TX);
}

/* 发送 OpenMV 数据，沿用原发送缓存行为。 */
void Drv_UartOpenMv_SendBuf(u8 *data, u8 len)
{
	for (u8 i = 0; i < len; i++)
	{
		s_openmv_tx_buf[s_openmv_tx_write_idx++] = *(data + i);
	}

	Drv_UartOpenMv_TxCheck();
}

/* 检查 OpenMV 串口发送缓存并尝试继续发送。 */
void Drv_UartOpenMv_TxCheck(void)
{
	while ((s_openmv_tx_read_idx != s_openmv_tx_write_idx) &&
		   ROM_UARTCharPutNonBlocking(UART2_BASE, s_openmv_tx_buf[s_openmv_tx_read_idx]))
	{
		s_openmv_tx_read_idx++;
	}
}

/* 底板串口 4 中断服务：接收光流数据。 */
void UART4_IRQHandler(void)
{
	uint8_t com_data;
	uint32_t flag = ROM_UARTIntStatus(UART7_BASE, 1);

	ROM_UARTIntClear(UART7_BASE, flag);

	while (ROM_UARTCharsAvail(UART7_BASE))
	{
		com_data = ROM_UARTCharGet(UART7_BASE);
		OFGetByte(com_data);
	}

	if (flag & UART_INT_TX)
	{
		Drv_UartOpticalFlow_TxCheck();
	}
}

/* 初始化光流串口，保持原 UART7 / GPIOE 配置不变。 */
void Drv_UartOpticalFlow_Init(uint32_t baudrate)
{
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART7);
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);

	ROM_GPIOPinConfigure(UART7_RX);
	ROM_GPIOPinConfigure(UART7_TX);
	ROM_GPIOPinTypeUART(UART7_PORT, UART7_PIN_TX | UART7_PIN_RX);

	ROM_UARTConfigSetExpClk(
		UART7_BASE,
		ROM_SysCtlClockGet(),
		baudrate,
		(UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE));

	ROM_UARTFIFOLevelSet(UART7_BASE, UART_FIFO_TX7_8, UART_FIFO_RX7_8);
	ROM_UARTFIFOEnable(UART7_BASE);
	ROM_UARTEnable(UART7_BASE);

	UARTIntRegister(UART7_BASE, UART4_IRQHandler);
	ROM_IntPrioritySet(INT_UART7, USER_INT2);
	ROM_UARTTxIntModeSet(UART7_BASE, UART_TXINT_MODE_EOT);
	ROM_UARTIntEnable(UART7_BASE, UART_INT_RX | UART_INT_RT | UART_INT_TX);
}

/* 发送光流模块数据，沿用原发送缓存行为。 */
void Drv_UartOpticalFlow_SendBuf(u8 *data, u8 len)
{
	for (u8 i = 0; i < len; i++)
	{
		s_optical_flow_tx_buf[s_optical_flow_tx_write_idx++] = *(data + i);
	}

	Drv_UartOpticalFlow_TxCheck();
}

/* 检查光流串口发送缓存并尝试继续发送。 */
void Drv_UartOpticalFlow_TxCheck(void)
{
	while ((s_optical_flow_tx_read_idx != s_optical_flow_tx_write_idx) &&
		   ROM_UARTCharPutNonBlocking(UART7_BASE, s_optical_flow_tx_buf[s_optical_flow_tx_read_idx]))
	{
		s_optical_flow_tx_read_idx++;
	}
}

/* 底板串口 5 中断服务：接收激光测距数据。 */
void UART5_IRQHandler(void)
{
	uint8_t com_data;
	uint32_t flag = ROM_UARTIntStatus(UART5_BASE, 1);

	ROM_UARTIntClear(UART5_BASE, flag);

	while (ROM_UARTCharsAvail(UART5_BASE))
	{
		com_data = ROM_UARTCharGet(UART5_BASE);
		Drv_Laser_GetOneByte(com_data);
	}

	if (flag & UART_INT_TX)
	{
		Drv_UartLaser_TxCheck();
	}
}

/*
 * 初始化激光串口，保持原 UART5 / GPIOE 配置不变。
 * 注意：PD7 解锁相关写法沿用原工程配置，不在本轮调整。
 */
void Drv_UartLaser_Init(uint32_t baudrate)
{
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART5);
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);

	HWREG(UART2_PORT + GPIO_O_LOCK) = GPIO_LOCK_KEY;
	HWREG(UART2_PORT + GPIO_O_CR) = UART5_PIN_TX;
	HWREG(UART2_PORT + GPIO_O_LOCK) = 0x00;

	ROM_GPIOPinConfigure(UART5_RX);
	ROM_GPIOPinConfigure(UART5_TX);
	ROM_GPIOPinTypeUART(UART5_PORT, UART5_PIN_TX | UART5_PIN_RX);

	ROM_UARTConfigSetExpClk(
		UART5_BASE,
		ROM_SysCtlClockGet(),
		baudrate,
		(UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE));

	ROM_UARTFIFOLevelSet(UART5_BASE, UART_FIFO_TX7_8, UART_FIFO_RX7_8);
	ROM_UARTFIFOEnable(UART5_BASE);
	ROM_UARTEnable(UART5_BASE);

	UARTIntRegister(UART5_BASE, UART5_IRQHandler);
	ROM_IntPrioritySet(INT_UART5, USER_INT2);
	ROM_UARTTxIntModeSet(UART5_BASE, UART_TXINT_MODE_EOT);
	ROM_UARTIntEnable(UART5_BASE, UART_INT_RX | UART_INT_RT | UART_INT_TX);
}

/* 发送激光模块数据，沿用原发送缓存行为。 */
void Drv_UartLaser_SendBuf(u8 *data, u8 len)
{
	for (u8 i = 0; i < len; i++)
	{
		s_laser_tx_buf[s_laser_tx_write_idx++] = *(data + i);
	}

	Drv_UartLaser_TxCheck();
}

/* 检查激光串口发送缓存并尝试继续发送。 */
void Drv_UartLaser_TxCheck(void)
{
	while ((s_laser_tx_read_idx != s_laser_tx_write_idx) &&
		   ROM_UARTCharPutNonBlocking(UART5_BASE, s_laser_tx_buf[s_laser_tx_read_idx]))
	{
		s_laser_tx_read_idx++;
	}
}

/* 历史兼容接口：底板串口 1 对应 GPS。 */
void Drv_Uart1Init(uint32_t baudrate) { Drv_UartGps_Init(baudrate); }
void Drv_Uart1SendBuf(u8 *data, u8 len) { Drv_UartGps_SendBuf(data, len); }
void Drv_Uart1TxCheck(void) { Drv_UartGps_TxCheck(); }

/* 历史兼容接口：底板串口 2 对应数传。 */
void Drv_Uart2Init(uint32_t baudrate) { Drv_UartDt_Init(baudrate); }
void Drv_Uart2SendBuf(u8 *data, u8 len) { Drv_UartDt_SendBuf(data, len); }
void Drv_Uart2TxCheck(void) { Drv_UartDt_TxCheck(); }

/* 历史兼容接口：底板串口 3 对应 OpenMV。 */
void Drv_Uart3Init(uint32_t baudrate) { Drv_UartOpenMv_Init(baudrate); }
void Drv_Uart3SendBuf(u8 *data, u8 len) { Drv_UartOpenMv_SendBuf(data, len); }
void Drv_Uart3TxCheck(void) { Drv_UartOpenMv_TxCheck(); }

/* 历史兼容接口：底板串口 4 对应光流。 */
void Drv_Uart4Init(uint32_t baudrate) { Drv_UartOpticalFlow_Init(baudrate); }
void Drv_Uart4SendBuf(u8 *data, u8 len) { Drv_UartOpticalFlow_SendBuf(data, len); }
void Drv_Uart4TxCheck(void) { Drv_UartOpticalFlow_TxCheck(); }

/* 历史兼容接口：底板串口 5 对应激光。 */
void Drv_Uart5Init(uint32_t baudrate) { Drv_UartLaser_Init(baudrate); }
void Drv_Uart5SendBuf(u8 *data, u8 len) { Drv_UartLaser_SendBuf(data, len); }
void Drv_Uart5TxCheck(void) { Drv_UartLaser_TxCheck(); }
