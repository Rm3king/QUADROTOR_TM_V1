#ifndef __ANO_USB_H
#define __ANO_USB_H
/* USB CDC 通信接口，仅封装初始化、收发与数据可用量查询。 */
#include "sysconfig.h"

void AnoUsbCdcInit(void);
void AnoUsbCdcSend( const uint8_t* data , uint16_t length );
uint16_t AnoUsbCdcRead( uint8_t* data , uint16_t length );
uint16_t AnoUsbCdcDataAvailable(void);

#endif
