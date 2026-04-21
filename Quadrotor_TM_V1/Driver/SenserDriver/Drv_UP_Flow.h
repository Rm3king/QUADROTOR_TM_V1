#ifndef __DRV_UP_FLOW_H
#define __DRV_UP_FLOW_H
#include "sysconfig.h"
#include "Ano_FcData.h"

/* 光流串口原始缓冲刷新计数。 */
extern uint8_t of_buf_update_cnt;
/* 光流模块原始数据缓冲区。 */
extern uint8_t OF_DATA[];

u8 Drv_OFInit(void);
void OFGetByte(uint8_t data);

#endif
