#include "FcUsbCdc.h"
#include "usb.h"
#include "hw_ints.h"
#include "usblib.h"
#include "usbcdc.h"
#include "usb-ids.h"
#include "usbdevice.h"
#include "usbdcdc.h"
#include "usb_serial_structs.h"
extern void USBIntRegister(uint32_t ui32Base, void (*pfnHandler)(void));
/*
 * 模块说明。
 * USB CDC 设备通信接口。
 * 负责初始化 USB CDC 设备栈，并提供发送、接收与缓冲区查询接口。
 */
static tLineCoding usb_linecoding =
{
    500000,
    USB_CDC_STOP_BITS_1,
    USB_CDC_PARITY_EVEN,
    8,
};

/* USB 枚举完成标志。 */
static volatile bool g_bUSBConfigured = false;

/*
 * 功能：处理 CDC 控制事件。
 * 说明：维护连接状态，并处理串口参数读写请求。
 */
uint32_t ControlHandler(void *pvCBData, uint32_t ui32Event,
                        uint32_t ui32MsgValue, void *pvMsgData)
{
    (void)pvCBData;
    (void)ui32MsgValue;
    switch(ui32Event)
    {
        case USB_EVENT_CONNECTED:
            g_bUSBConfigured = true;
            USBBufferFlush(&g_sTxBuffer);
            USBBufferFlush(&g_sRxBuffer);
        break;

        case USB_EVENT_DISCONNECTED:
            g_bUSBConfigured = false;
        break;

        case USBD_CDC_EVENT_GET_LINE_CODING:
            *((tLineCoding *)pvMsgData) = usb_linecoding;
        break;

        case USBD_CDC_EVENT_SET_LINE_CODING:
            usb_linecoding = *((tLineCoding *)pvMsgData);
        break;

        case USBD_CDC_EVENT_SET_CONTROL_LINE_STATE:
        case USBD_CDC_EVENT_SEND_BREAK:
        case USBD_CDC_EVENT_CLEAR_BREAK:
        case USB_EVENT_SUSPEND:
        case USB_EVENT_RESUME:
        break;

        default:
#ifdef DEBUG
            while(1)
            {
            }
#else
        break;
#endif
    }
    return 0;
}

/*
 * 功能：处理 CDC 发送事件。
 * 说明：当前仅保留事件占位，不改变原始处理流程。
 */
uint32_t TxHandler(void *pvCBData, uint32_t ui32Event, uint32_t ui32MsgValue,
                   void *pvMsgData)
{
    (void)pvCBData;
    (void)ui32MsgValue;
    (void)pvMsgData;
    switch(ui32Event)
    {
        case USB_EVENT_TX_COMPLETE:
        break;

        default:
#ifdef DEBUG
            while(1)
            {
            }
#else
        break;
#endif
    }
    return 0;
}

/*
 * 功能：处理 CDC 接收事件。
 * 说明：当前由 USBBuffer 统一管理数据缓冲，这里保留回调入口。
 */
uint32_t RxHandler(void *pvCBData, uint32_t ui32Event, uint32_t ui32MsgValue,
                   void *pvMsgData)
{
    uint32_t ui32Count = 0;
    (void)pvCBData;
    (void)ui32MsgValue;
    (void)pvMsgData;
    switch(ui32Event)
    {
        case USB_EVENT_RX_AVAILABLE:
        break;

        case USB_EVENT_DATA_REMAINING:
            return ui32Count;

        case USB_EVENT_REQUEST_BUFFER:
            return 0;

        default:
#ifdef DEBUG
            while(1)
            {
            }
#else
        break;
#endif
    }
    return 0;
}

/*
 * 功能：初始化 USB CDC 设备。
 * 说明：完成 USB 引脚、缓冲区和设备栈初始化。
 */
void UsbCdcInit(void)
{
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
    ROM_GPIOPinTypeUSBAnalog(GPIOD_BASE, GPIO_PIN_4);
    ROM_GPIOPinTypeUSBAnalog(GPIOD_BASE, GPIO_PIN_5);
    USBBufferInit(&g_sTxBuffer);
    USBBufferInit(&g_sRxBuffer);
    USBStackModeSet(0, eUSBModeForceDevice, 0);
    (void)USBDCDCInit(0, &g_sCDCDevice);
    USBIntRegister(INT_USB0, USB0DeviceIntHandler);
    ROM_IntPrioritySet(INT_USB0, USER_INT7);
}

/* 功能：发送 USB CDC 数据。 */
void UsbCdcSend(const uint8_t *data, uint16_t length)
{
    if(g_bUSBConfigured)
    {
        USBBufferWrite(&g_sTxBuffer, data, length);
    }
}

/* 功能：读取 USB CDC 数据。 */
uint16_t UsbCdcRead(uint8_t *data, uint16_t length)
{
    if(g_bUSBConfigured)
    {
        return USBBufferRead(&g_sRxBuffer, data, length);
    }
    return 0;
}

/* 功能：查询 USB CDC 接收缓冲区可读字节数。 */
uint16_t UsbCdcDataAvailable(void)
{
    return USBBufferDataAvailable(&g_sRxBuffer);
}
