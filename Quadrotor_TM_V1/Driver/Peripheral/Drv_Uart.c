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
 * 文件名称: Drv_Uart.c
 * 所属模块: Driver / Peripheral
 *
 * 功能描述:
 *   通用 UART 驱动模块，管理 TM4C123 的 5 路串口通道：
 *   1) Drv_UartXxx_Init()    -- 各通道初始化（波特率、GPIO 复用、中断注册）
 *   2) Drv_UartXxx_SendBuf() -- 各通道发送（写入环形缓冲，中断驱动发送）
 *   3) UartXxx_IRQHandler()  -- 各通道中断处理（接收分发 + 发送推进）
 *
 * 硬件映射:
 *   ┌──────────┬─────────────────┬────────────┬──────────┐
 *   │ 底板编号 │ TM4C UART (GPIO)│ 外设功能   │ 波特率   │
 *   ├──────────┼─────────────────┼────────────┼──────────┤
 *   │ 串口1    │ UART0 (PA0/PA1) │ GPS        │ 115200   │
 *   │ 串口2    │ UART4 (PC4/PC5) │ 数传/上位机│ 500000   │
 *   │ 串口3    │ UART2 (PD6/PD7) │ OpenMV     │ 500000   │
 *   │ 串口4    │ UART7 (PE0/PE1) │ 光流模块   │ 500000   │
 *   │ 串口5    │ UART5 (PE4/PE5) │ 激光测距   │ 115200   │
 *   └──────────┴─────────────────┴────────────┴──────────┘
 *
 * 发送机制:
 *   uart_send_buf() 将数据写入 256 字节环形缓冲 → 启用 TX 中断
 *   → 中断逐字节发送 → 缓冲清空后关闭 TX 中断
 *
 * 架构位置:
 *   最底层硬件驱动，被 DT.c / Drv_gps.c / OF.c 等上层模块调用。
 *
 * 教学提示:
 *   - uart_ch_t 结构体统一封装避免了 5 套重复的发送代码
 *   - 环形缓冲使用读写指针比较判断空/满，是嵌入式经典模式
 *   - ISR 命名使用语义名（UartGps/UartDt）而非硬件编号，提高可读性
 */

#define UART_TX_BUF_LEN 256

/* 串口通道运行时状态，每路串口各一个实例 */
typedef struct {
    uint32_t base;
    u8       tx_buf[UART_TX_BUF_LEN];
    u8       tx_wr;
    u8       tx_rd;
} uart_ch_t;

static uart_ch_t s_ch_gps     = { .base = UART0_BASE };
static uart_ch_t s_ch_dt      = { .base = UART4_BASE };
static uart_ch_t s_ch_openmv  = { .base = UART2_BASE };
static uart_ch_t s_ch_optflow = { .base = UART7_BASE };
static uart_ch_t s_ch_laser   = { .base = UART5_BASE };

/* ======================== 通用收发逻辑 ======================== */

/* 尝试将缓冲区中的待发送数据写入 UART 硬件 FIFO */
static void uart_tx_check(uart_ch_t *ch)
{
    while ((ch->tx_rd != ch->tx_wr) &&
           ROM_UARTCharPutNonBlocking(ch->base, ch->tx_buf[ch->tx_rd]))
    {
        ch->tx_rd++;
    }
}

/* 将数据写入环形发送缓冲，随后触发发送 */
static void uart_send_buf(uart_ch_t *ch, u8 *data, u8 len)
{
    for (u8 i = 0; i < len; i++)
    {
        ch->tx_buf[ch->tx_wr++] = data[i];
    }
    uart_tx_check(ch);
}

/* ======================== GPS（TM4C UART0） ======================== */

static void UartGps_IRQHandler(void)
{
    uint8_t com_data;
    uint32_t status = ROM_UARTIntStatus(UART0_BASE, 1);
    ROM_UARTIntClear(UART0_BASE, status);

    while (ROM_UARTCharsAvail(UART0_BASE))
    {
        com_data = ROM_UARTCharGet(UART0_BASE);
        Drv_GpsGetOneByte(com_data);
    }

    if (status & UART_INT_TX)
    {
        uart_tx_check(&s_ch_gps);
    }
}

void Drv_UartGps_Init(uint32_t baudrate)
{
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

    ROM_GPIOPinConfigure(UART0_RX);
    ROM_GPIOPinConfigure(UART0_TX);
    ROM_GPIOPinTypeUART(UART0_PORT, UART0_PIN_TX | UART0_PIN_RX);

    ROM_UARTConfigSetExpClk(
        UART0_BASE, ROM_SysCtlClockGet(), baudrate,
        UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE);

    ROM_UARTFIFOLevelSet(UART0_BASE, UART_FIFO_TX7_8, UART_FIFO_RX7_8);
    ROM_UARTFIFOEnable(UART0_BASE);
    ROM_UARTEnable(UART0_BASE);

    UARTIntRegister(UART0_BASE, UartGps_IRQHandler);
    ROM_IntPrioritySet(INT_UART0, USER_INT2);
    ROM_UARTTxIntModeSet(UART0_BASE, UART_TXINT_MODE_EOT);
    ROM_UARTIntEnable(UART0_BASE, UART_INT_RX | UART_INT_RT | UART_INT_TX);
}

void Drv_UartGps_SendBuf(u8 *data, u8 len)
{
    uart_send_buf(&s_ch_gps, data, len);
}

/* ======================== 数传（TM4C UART4） ======================== */

static void UartDt_IRQHandler(void)
{
    uint8_t com_data;
    uint32_t status = ROM_UARTIntStatus(UART4_BASE, 1);
    ROM_UARTIntClear(UART4_BASE, status);

    while (ROM_UARTCharsAvail(UART4_BASE))
    {
        com_data = ROM_UARTCharGet(UART4_BASE);
        ANO_DT_Data_Receive_Prepare(com_data);
    }

    if (status & UART_INT_TX)
    {
        uart_tx_check(&s_ch_dt);
    }
}

void Drv_UartDt_Init(uint32_t baudrate)
{
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART4);
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);

    ROM_GPIOPinConfigure(UART4_RX);
    ROM_GPIOPinConfigure(UART4_TX);
    ROM_GPIOPinTypeUART(UART4_PORT, UART4_PIN_TX | UART4_PIN_RX);

    ROM_UARTConfigSetExpClk(
        UART4_BASE, ROM_SysCtlClockGet(), baudrate,
        UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE);

    ROM_UARTFIFOLevelSet(UART4_BASE, UART_FIFO_TX7_8, UART_FIFO_RX7_8);
    ROM_UARTFIFOEnable(UART4_BASE);
    ROM_UARTEnable(UART4_BASE);

    UARTIntRegister(UART4_BASE, UartDt_IRQHandler);
    ROM_IntPrioritySet(INT_UART4, USER_INT2);
    ROM_UARTTxIntModeSet(UART4_BASE, UART_TXINT_MODE_EOT);
    ROM_UARTIntEnable(UART4_BASE, UART_INT_RX | UART_INT_RT | UART_INT_TX);
}

void Drv_UartDt_SendBuf(u8 *data, u8 len)
{
    uart_send_buf(&s_ch_dt, data, len);
}

/* ======================== OpenMV（TM4C UART2） ======================== */

static void UartOpenMv_IRQHandler(void)
{
    uint8_t com_data;
    uint32_t status = ROM_UARTIntStatus(UART2_BASE, 1);
    ROM_UARTIntClear(UART2_BASE, status);

    while (ROM_UARTCharsAvail(UART2_BASE))
    {
        com_data = ROM_UARTCharGet(UART2_BASE);
        OpenMV_Byte_Get(com_data);
    }

    if (status & UART_INT_TX)
    {
        uart_tx_check(&s_ch_openmv);
    }
}

void Drv_UartOpenMv_Init(uint32_t baudrate)
{
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART2);
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);

    ROM_GPIOPinConfigure(UART2_RX);
    ROM_GPIOPinConfigure(UART2_TX);
    ROM_GPIOPinTypeUART(UART2_PORT, UART2_PIN_TX | UART2_PIN_RX);

    ROM_UARTConfigSetExpClk(
        UART2_BASE, ROM_SysCtlClockGet(), baudrate,
        UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE);

    ROM_UARTFIFOLevelSet(UART2_BASE, UART_FIFO_TX7_8, UART_FIFO_RX7_8);
    ROM_UARTFIFOEnable(UART2_BASE);
    ROM_UARTEnable(UART2_BASE);

    UARTIntRegister(UART2_BASE, UartOpenMv_IRQHandler);
    ROM_IntPrioritySet(INT_UART2, USER_INT2);
    ROM_UARTTxIntModeSet(UART2_BASE, UART_TXINT_MODE_EOT);
    ROM_UARTIntEnable(UART2_BASE, UART_INT_RX | UART_INT_RT | UART_INT_TX);
}

void Drv_UartOpenMv_SendBuf(u8 *data, u8 len)
{
    uart_send_buf(&s_ch_openmv, data, len);
}

/* ======================== 光流（TM4C UART7） ======================== */

static void UartOptFlow_IRQHandler(void)
{
    uint8_t com_data;
    uint32_t status = ROM_UARTIntStatus(UART7_BASE, 1);
    ROM_UARTIntClear(UART7_BASE, status);

    while (ROM_UARTCharsAvail(UART7_BASE))
    {
        com_data = ROM_UARTCharGet(UART7_BASE);
        OFGetByte(com_data);
    }

    if (status & UART_INT_TX)
    {
        uart_tx_check(&s_ch_optflow);
    }
}

void Drv_UartOpticalFlow_Init(uint32_t baudrate)
{
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART7);
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);

    ROM_GPIOPinConfigure(UART7_RX);
    ROM_GPIOPinConfigure(UART7_TX);
    ROM_GPIOPinTypeUART(UART7_PORT, UART7_PIN_TX | UART7_PIN_RX);

    ROM_UARTConfigSetExpClk(
        UART7_BASE, ROM_SysCtlClockGet(), baudrate,
        UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE);

    ROM_UARTFIFOLevelSet(UART7_BASE, UART_FIFO_TX7_8, UART_FIFO_RX7_8);
    ROM_UARTFIFOEnable(UART7_BASE);
    ROM_UARTEnable(UART7_BASE);

    UARTIntRegister(UART7_BASE, UartOptFlow_IRQHandler);
    ROM_IntPrioritySet(INT_UART7, USER_INT2);
    ROM_UARTTxIntModeSet(UART7_BASE, UART_TXINT_MODE_EOT);
    ROM_UARTIntEnable(UART7_BASE, UART_INT_RX | UART_INT_RT | UART_INT_TX);
}

void Drv_UartOpticalFlow_SendBuf(u8 *data, u8 len)
{
    uart_send_buf(&s_ch_optflow, data, len);
}

/* ======================== 激光测距（TM4C UART5） ======================== */

static void UartLaser_IRQHandler(void)
{
    uint8_t com_data;
    uint32_t status = ROM_UARTIntStatus(UART5_BASE, 1);
    ROM_UARTIntClear(UART5_BASE, status);

    while (ROM_UARTCharsAvail(UART5_BASE))
    {
        com_data = ROM_UARTCharGet(UART5_BASE);
        Drv_Laser_GetOneByte(com_data);
    }

    if (status & UART_INT_TX)
    {
        uart_tx_check(&s_ch_laser);
    }
}

/*
 * 激光串口初始化。
 * 历史遗留：此处保留了原工程对 GPIOD (PD7) 的解锁操作，
 * 虽然激光串口实际使用 GPIOE (PE4/PE5)，但删除此操作可能
 * 影响 OpenMV 串口 (UART2, PD7) 的初始化顺序依赖，暂不调整。
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
        UART5_BASE, ROM_SysCtlClockGet(), baudrate,
        UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE);

    ROM_UARTFIFOLevelSet(UART5_BASE, UART_FIFO_TX7_8, UART_FIFO_RX7_8);
    ROM_UARTFIFOEnable(UART5_BASE);
    ROM_UARTEnable(UART5_BASE);

    UARTIntRegister(UART5_BASE, UartLaser_IRQHandler);
    ROM_IntPrioritySet(INT_UART5, USER_INT2);
    ROM_UARTTxIntModeSet(UART5_BASE, UART_TXINT_MODE_EOT);
    ROM_UARTIntEnable(UART5_BASE, UART_INT_RX | UART_INT_RT | UART_INT_TX);
}

void Drv_UartLaser_SendBuf(u8 *data, u8 len)
{
    uart_send_buf(&s_ch_laser, data, len);
}
