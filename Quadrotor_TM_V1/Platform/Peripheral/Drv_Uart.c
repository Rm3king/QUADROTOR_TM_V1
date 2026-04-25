#include "Drv_Uart.h"
#include "uart.h"
#include "hw_ints.h"
#include "hw_types.h"
#include "hw_gpio.h"
#include "DT.h"
#include "Drv_OpenMV.h"
#include "Drv_laser.h"

/*
 * ???????
 * ??????????????????????????
 * ?????????? TM4C ?? UART ?????????????
 */
#include "Drv_gps.h"

u8 s_uart1_tx_buf[256];

u8 s_uart1_tx_write_idx = 0;

u8 s_uart1_tx_read_idx = 0;

/* 底板串口1中断服务，接收 GPS 数据 */

void UART1_IRQHandler(void)

{

	uint8_t com_data;

	/*获取中断标志 原始中断状态 不屏蔽中断标志*/		

	uint32_t flag = ROM_UARTIntStatus(UART0_BASE,1);

	/*清除中断标志*/	

	ROM_UARTIntClear(UART0_BASE,flag);		

	/*判断FIFO是否还有数据*/		

	while(ROM_UARTCharsAvail(UART0_BASE))		

	{			

		com_data=ROM_UARTCharGet(UART0_BASE);

		Drv_GpsGetOneByte(com_data);

	}

	if(flag & UART_INT_TX)

	{

		Drv_Uart1TxCheck();

	}

}

/* 初始化底板串口1 */

void Drv_Uart1Init(uint32_t baudrate)

{

	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);

	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

	

	/*GPIO的UART模式配置*/

	ROM_GPIOPinConfigure(UART0_RX);

	ROM_GPIOPinConfigure(UART0_TX);

	ROM_GPIOPinTypeUART(UART0_PORT, UART0_PIN_TX | UART0_PIN_RX);

	/*配置串口号波特率和时钟源*/		

	ROM_UARTConfigSetExpClk(UART0_BASE, ROM_SysCtlClockGet(), baudrate,(UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE));

	/*FIFO设置*/

	ROM_UARTFIFOLevelSet(UART0_BASE,UART_FIFO_TX7_8,UART_FIFO_RX7_8);

	ROM_UARTFIFOEnable(UART0_BASE);

	/*使能串口*/

	ROM_UARTEnable( UART0_BASE );

	/*使能UART0接收中断*/			

	UARTIntRegister(UART0_BASE,UART1_IRQHandler);			

	ROM_IntPrioritySet(INT_UART0, USER_INT2);

	ROM_UARTTxIntModeSet(UART0_BASE,UART_TXINT_MODE_EOT);

	ROM_UARTIntEnable(UART0_BASE,UART_INT_RX | UART_INT_RT | UART_INT_TX);

}

void Drv_Uart1SendBuf(u8 *data, u8 len)

{

	for(u8 i=0; i<len; i++)

	{

		s_uart1_tx_buf[s_uart1_tx_write_idx++] = * ( data + i );

	}

	Drv_Uart1TxCheck();

}

void Drv_Uart1TxCheck(void)

{

	while( (s_uart1_tx_read_idx != s_uart1_tx_write_idx) && (ROM_UARTCharPutNonBlocking(UART0_BASE,s_uart1_tx_buf[s_uart1_tx_read_idx])) )

		s_uart1_tx_read_idx++;

}

u8 s_uart2_tx_buf[256];

u8 s_uart2_tx_write_idx = 0;

u8 s_uart2_tx_read_idx = 0;

/* 底板串口2中断服务，接收数传数据 */

void UART2_IRQHandler(void)

{

	uint8_t com_data;

	/*获取中断标志 原始中断状态 不屏蔽中断标志*/		

	uint32_t flag = ROM_UARTIntStatus(UART4_BASE,1);

	/*清除中断标志*/	

	ROM_UARTIntClear(UART4_BASE,flag);		

	/*判断FIFO是否还有数据*/		

	while(ROM_UARTCharsAvail(UART4_BASE))		

	{			

		com_data=ROM_UARTCharGet(UART4_BASE);

		ANO_DT_Data_Receive_Prepare(com_data);

	}

	if(flag & UART_INT_TX)

	{

		Drv_Uart2TxCheck();

	}

}

/* 初始化底板串口2 */

void Drv_Uart2Init(uint32_t baudrate)

{

	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART4);

	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);

	

	/*GPIO的UART模式配置*/

	ROM_GPIOPinConfigure(UART4_RX);

	ROM_GPIOPinConfigure(UART4_TX);

	ROM_GPIOPinTypeUART(UART4_PORT, UART4_PIN_TX | UART4_PIN_RX);

	/*配置串口号波特率和时钟源*/		

	ROM_UARTConfigSetExpClk(UART4_BASE, ROM_SysCtlClockGet(), baudrate,(UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE));

	/*FIFO设置*/

	ROM_UARTFIFOLevelSet(UART4_BASE,UART_FIFO_TX7_8,UART_FIFO_RX7_8);

	ROM_UARTFIFOEnable(UART4_BASE);

	/*使能串口*/

	ROM_UARTEnable( UART4_BASE );

	/*使能UART0接收中断*/			

	UARTIntRegister(UART4_BASE,UART2_IRQHandler);			

	ROM_IntPrioritySet(INT_UART4, USER_INT2);

	ROM_UARTTxIntModeSet(UART4_BASE,UART_TXINT_MODE_EOT);

	ROM_UARTIntEnable(UART4_BASE,UART_INT_RX | UART_INT_RT | UART_INT_TX);

}

void Drv_Uart2SendBuf(u8 *data, u8 len)

{

	for(u8 i=0; i<len; i++)

	{

		s_uart2_tx_buf[s_uart2_tx_write_idx++] = * ( data + i );

	}

	Drv_Uart2TxCheck();

}

void Drv_Uart2TxCheck(void)

{

	while( (s_uart2_tx_read_idx != s_uart2_tx_write_idx) && (ROM_UARTCharPutNonBlocking(UART4_BASE,s_uart2_tx_buf[s_uart2_tx_read_idx])) )

		s_uart2_tx_read_idx++;

}

u8 s_uart3_tx_buf[256];

u8 s_uart3_tx_write_idx = 0;

u8 s_uart3_tx_read_idx = 0;

/* 底板串口3中断服务，接收 OpenMV 数据 */

void UART3_IRQHandler(void)

{

	uint8_t com_data;

	/*获取中断标志 原始中断状态 不屏蔽中断标志*/		

	uint32_t flag = ROM_UARTIntStatus(UART2_BASE,1);

	/*清除中断标志*/	

	ROM_UARTIntClear(UART2_BASE,flag);		

	/*判断FIFO是否还有数据*/		

	while(ROM_UARTCharsAvail(UART2_BASE))		

	{			

		com_data=ROM_UARTCharGet(UART2_BASE);

		OpenMV_Byte_Get(com_data);

	}

	if(flag & UART_INT_TX)

	{

		Drv_Uart3TxCheck();

	}

}

/* 初始化底板串口3 */

void Drv_Uart3Init(uint32_t baudrate)

{

	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART2);

	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);

	

	/*GPIO的UART模式配置*/

	ROM_GPIOPinConfigure(UART2_RX);

	ROM_GPIOPinConfigure(UART2_TX);

	ROM_GPIOPinTypeUART(UART2_PORT, UART2_PIN_TX | UART2_PIN_RX);

	/*配置串口号波特率和时钟源*/		

	ROM_UARTConfigSetExpClk(UART2_BASE, ROM_SysCtlClockGet(), baudrate,(UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE));

	/*FIFO设置*/

	ROM_UARTFIFOLevelSet(UART2_BASE,UART_FIFO_TX7_8,UART_FIFO_RX7_8);

	ROM_UARTFIFOEnable(UART2_BASE);

	/*使能串口*/

	ROM_UARTEnable( UART2_BASE );

	/*使能UART0接收中断*/			

	UARTIntRegister(UART2_BASE,UART3_IRQHandler);			

	ROM_IntPrioritySet(INT_UART2, USER_INT2);

	ROM_UARTTxIntModeSet(UART2_BASE,UART_TXINT_MODE_EOT);

	ROM_UARTIntEnable(UART2_BASE,UART_INT_RX | UART_INT_RT | UART_INT_TX);

}

void Drv_Uart3SendBuf(u8 *data, u8 len)

{

	for(u8 i=0; i<len; i++)

	{

		s_uart3_tx_buf[s_uart3_tx_write_idx++] = * ( data + i );

	}

	Drv_Uart3TxCheck();

}

void Drv_Uart3TxCheck(void)

{

	while( (s_uart3_tx_read_idx != s_uart3_tx_write_idx) && (ROM_UARTCharPutNonBlocking(UART2_BASE,s_uart3_tx_buf[s_uart3_tx_read_idx])) )

		s_uart3_tx_read_idx++;

}

#include "OF.h"

#include "Drv_UP_Flow.h"

u8 s_uart4_tx_buf[256];

u8 s_uart4_tx_write_idx = 0;

u8 s_uart4_tx_read_idx = 0;

/* 底板串口4中断服务，接收光流数据 */

void UART4_IRQHandler(void)

{

	uint8_t com_data;

	/*获取中断标志 原始中断状态 不屏蔽中断标志*/		

	uint32_t flag = ROM_UARTIntStatus(UART7_BASE,1);

	/*清除中断标志*/	

	ROM_UARTIntClear(UART7_BASE,flag);		

	/*判断FIFO是否还有数据*/		

	while(ROM_UARTCharsAvail(UART7_BASE))		

	{			

		com_data=ROM_UARTCharGet(UART7_BASE);

		OFGetByte(com_data);

	}

	if(flag & UART_INT_TX)

	{

		Drv_Uart4TxCheck();

	}

}

/* 初始化底板串口4 */

void Drv_Uart4Init(uint32_t baudrate)

{

	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART7);

	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);

	

	/*GPIO的UART模式配置*/

	ROM_GPIOPinConfigure(UART7_RX);

	ROM_GPIOPinConfigure(UART7_TX);

	ROM_GPIOPinTypeUART(UART7_PORT, UART7_PIN_TX | UART7_PIN_RX);

	/*配置串口号波特率和时钟源*/		

	ROM_UARTConfigSetExpClk(UART7_BASE, ROM_SysCtlClockGet(), baudrate,(UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE));

	/*FIFO设置*/

	ROM_UARTFIFOLevelSet(UART7_BASE,UART_FIFO_TX7_8,UART_FIFO_RX7_8);

	ROM_UARTFIFOEnable(UART7_BASE);

	/*使能串口*/

	ROM_UARTEnable( UART7_BASE );

	/*使能UART0接收中断*/			

	UARTIntRegister(UART7_BASE,UART4_IRQHandler);			

	ROM_IntPrioritySet(INT_UART7, USER_INT2);

	ROM_UARTTxIntModeSet(UART7_BASE,UART_TXINT_MODE_EOT);

	ROM_UARTIntEnable(UART7_BASE,UART_INT_RX | UART_INT_RT | UART_INT_TX);

}

void Drv_Uart4SendBuf(u8 *data, u8 len)

{

	for(u8 i=0; i<len; i++)

	{

		s_uart4_tx_buf[s_uart4_tx_write_idx++] = * ( data + i );

	}

	Drv_Uart4TxCheck();

}

void Drv_Uart4TxCheck(void)

{

	while( (s_uart4_tx_read_idx != s_uart4_tx_write_idx) && (ROM_UARTCharPutNonBlocking(UART7_BASE,s_uart4_tx_buf[s_uart4_tx_read_idx])) )

		s_uart4_tx_read_idx++;

}

u8 s_uart5_tx_buf[256];

u8 s_uart5_tx_write_idx = 0;

u8 s_uart5_tx_read_idx = 0;

/* 底板串口5中断服务，接收激光测距数据 */

void UART5_IRQHandler(void)

{

	uint8_t com_data;

	/*获取中断标志 原始中断状态 不屏蔽中断标志*/		

	uint32_t flag = ROM_UARTIntStatus(UART5_BASE,1);

	/*清除中断标志*/	

	ROM_UARTIntClear(UART5_BASE,flag);		

	/*判断FIFO是否还有数据*/		

	while(ROM_UARTCharsAvail(UART5_BASE))		

	{			

		com_data=ROM_UARTCharGet(UART5_BASE);

		Drv_Laser_GetOneByte(com_data);

	}

	if(flag & UART_INT_TX)

	{

		Drv_Uart5TxCheck();

	}

}

/* 初始化底板串口5 */

void Drv_Uart5Init(uint32_t baudrate)

{

	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART5);

	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);

	

	/*PD7解锁操作*/

	HWREG(UART2_PORT + GPIO_O_LOCK) = GPIO_LOCK_KEY; 

	HWREG(UART2_PORT + GPIO_O_CR) = UART5_PIN_TX;

	HWREG(UART2_PORT + GPIO_O_LOCK) = 0x00;

	/*GPIO的UART模式配置*/

	ROM_GPIOPinConfigure(UART5_RX);

	ROM_GPIOPinConfigure(UART5_TX);

	ROM_GPIOPinTypeUART(UART5_PORT, UART5_PIN_TX | UART5_PIN_RX);

	/*配置串口号波特率和时钟源*/		

	ROM_UARTConfigSetExpClk(UART5_BASE, ROM_SysCtlClockGet(), baudrate,(UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE));

	/*FIFO设置*/

	ROM_UARTFIFOLevelSet(UART5_BASE,UART_FIFO_TX7_8,UART_FIFO_RX7_8);

	ROM_UARTFIFOEnable(UART5_BASE);

	/*使能串口*/

	ROM_UARTEnable( UART5_BASE );

	/*使能UART0接收中断*/			

	UARTIntRegister(UART5_BASE,UART5_IRQHandler);			

	ROM_IntPrioritySet(INT_UART5, USER_INT2);

	ROM_UARTTxIntModeSet(UART5_BASE,UART_TXINT_MODE_EOT);

	ROM_UARTIntEnable(UART5_BASE,UART_INT_RX | UART_INT_RT | UART_INT_TX);

}

void Drv_Uart5SendBuf(u8 *data, u8 len)

{

	for(u8 i=0; i<len; i++)

	{

		s_uart5_tx_buf[s_uart5_tx_write_idx++] = * ( data + i );

	}

	Drv_Uart5TxCheck();

}

void Drv_Uart5TxCheck(void)

{

	while( (s_uart5_tx_read_idx != s_uart5_tx_write_idx) && (ROM_UARTCharPutNonBlocking(UART5_BASE,s_uart5_tx_buf[s_uart5_tx_read_idx])) )

		s_uart5_tx_read_idx++;

}

