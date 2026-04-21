#ifndef _LASER_H_
#define _LASER_H_
#include "sysconfig.h"

/*
 * 模块名称：Drv_laser
 * 模块职责：提供激光测距模块的初始化与字节接收接口。
 */

extern u8 LASER_LINKOK;
extern u16 Laser_height_cm;

u8 		Drv_Laser_Init(void);
void 	Drv_Laser_GetOneByte(u8 data);

#endif
