#ifndef __ANO_OPMV_CTRL_H
#define __ANO_OPMV_CTRL_H
#include "sysconfig.h"
#include "Ano_FcData.h"
/*
 * 模块说明。
 * OpenMV 控制状态管理。
 * 用于汇总 OpenMV 任务使能条件，并协调不同视觉控制子模块的启停。
 */
typedef struct
{
    /* OpenMV 控制总使能。 */
    u8 en;
    /* 高度条件是否满足。 */
    u8 height_flag;
    /* 输出复位保护标志。 */
    u8 reset_flag;
} _opmv_ct_sta_st;

extern _opmv_ct_sta_st opmv_ct_sta;

/* 功能：20ms 周期执行 OpenMV 控制选择。 */
void ANO_OPMV_Ctrl_Task(u8 dT_ms);

#endif
