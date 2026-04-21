#include "Ano_Usb.h"
#include "usb.h"
#include "hw_ints.h"
#include "usblib.h"
#include "usbcdc.h"
#include "usb-ids.h"
#include "usbdevice.h"
#include "usbdcdc.h"
#include "usb_serial_structs.h"
/*
 * ?????
 * USB CDC ?????????
 *
 * ????? USB CDC ????????????????????
 * ?????????????????
 */
static tLineCoding usb_linecoding =
{
    500000,
    USB_CDC_STOP_BITS_1,
    USB_CDC_PARITY_EVEN,
    8,
};
/* USB ??????? */
static volatile bool g_bUSBConfigured = false;
/*
 * ????? CDC ???????
 * ???
 * ????????????????????????
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
 * ????? CDC ???????
 * ???
 * ???? USBBuffer ???????????????????
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
 * ????? CDC ???????
 * ???
 * ????????????????????????????
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
 * ?????? USB CDC ?????
 * ???
 * ?? USB ???????????????????
 */
void AnoUsbCdcInit(void)
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
/* ?? USB CDC ??? */
void AnoUsbCdcSend(const uint8_t *data, uint16_t length)
{
    if(g_bUSBConfigured)
    {
        USBBufferWrite(&g_sTxBuffer, data, length);
    }
}
/* ?? USB CDC ????? */
uint16_t AnoUsbCdcRead(uint8_t *data, uint16_t length)
{
    if(g_bUSBConfigured)
    {
        return USBBufferRead(&g_sRxBuffer, data, length);
    }
    return 0;
}
/* ?? USB CDC ???????????? */
uint16_t AnoUsbCdcDataAvailable(void)
{
    return USBBufferDataAvailable(&g_sRxBuffer);
}