#ifndef __ANO_USB_H
#define __ANO_USB_H
/*
 * 模块名称：Ano_Usb
 * 模块职责：封装 USB CDC 的初始化、收发与数据可用量查询接口。
 * 使用约束：本头文件仅暴露对外接口，不包含 USB 事件处理细节。
 */
#include "sysconfig.h"

void AnoUsbCdcInit(void);
void AnoUsbCdcSend( const uint8_t* data , uint16_t length );
uint16_t AnoUsbCdcRead( uint8_t* data , uint16_t length );
uint16_t AnoUsbCdcDataAvailable(void);

#endif
