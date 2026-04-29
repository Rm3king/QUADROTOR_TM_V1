#ifndef _DRV_RCIN_H_
#define _DRV_RCIN_H_

#include "sysconfig.h"
/*
 * 模块名称：Drv_RcIn
 * 模块职责：提供 PPM 与 SBUS 两类遥控输入的底层初始化接口。
 * 命名说明：保留历史初始化名，同时补充按协议语义命名的兼容别名。
 * 使用约束：本轮不修改中断、解码与通道换算逻辑。
 */

typedef struct
{
    int16_t Roll;
    int16_t Pitch;
    int16_t Throttle;
    int16_t Yaw;
    int16_t Aux1;
    int16_t Aux2;
    int16_t Aux3;
    int16_t Aux4;
    int16_t Aux5;
    int16_t Aux6;
    int16_t Aux7;
    int16_t Aux8;
    int16_t Aux9;
    int16_t Aux10;
    int16_t Aux11;
    int16_t Aux12;
} RCData_t;
union PPM
{
    uint16_t Captures[16];
    RCData_t Data;
} ;


extern union PPM  RC_PPM;
extern u16 Rc_Sbus_In[16];

void Drv_PpmInit(void);
void Drv_SbusInit(void);

/* 语义化兼容别名：用于逐步替代仅按缩写命名的旧接口。 */
#define Drv_RcPpm_Init		Drv_PpmInit
#define Drv_RcSbus_Init		Drv_SbusInit

#endif
