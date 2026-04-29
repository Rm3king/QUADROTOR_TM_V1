#ifndef __FC_USB_CDC_H__
#define __FC_USB_CDC_H__
/*
 * 模块名称：FcUsbCdc
 * 模块职责：封装 USB CDC 的初始化、收发与数据可用量查询接口。
 * 使用约束：本头文件仅暴露对外接口，不包含 USB 事件处理细节。
 */
#include <stdint.h>
#include "sysconfig.h"

void UsbCdcInit(void);
void UsbCdcSend( const uint8_t* data , uint16_t length );
uint16_t UsbCdcRead( uint8_t* data , uint16_t length );
uint16_t UsbCdcDataAvailable(void);

#endif
