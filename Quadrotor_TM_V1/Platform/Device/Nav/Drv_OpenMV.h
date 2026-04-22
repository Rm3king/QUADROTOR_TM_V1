#ifndef __DRV_OPENMV_H
#define __DRV_OPENMV_H
#include "sysconfig.h"
#include "FcData.h"

/* 色块跟踪结果。 */
typedef struct
{
    u8 color_flag;
    u8 sta;
    s16 pos_x;
    s16 pos_y;
    u8 dT_ms;
} _openmv_color_block_st;

/* 寻线结果。 */
typedef struct
{
    u8 sta;
    s16 angle;
    s16 deviation;
    u8 p_flag;
    s16 pos_x;
    s16 pos_y;
    u8 dT_ms;
} _openmv_line_tracking_st;

/* OpenMV 数据总状态。 */
typedef struct
{
    u8 offline;
    u8 mode_cmd;
    u8 mode_sta;
    _openmv_color_block_st cb;
    _openmv_line_tracking_st lt;
} _openmv_data_st;

extern _openmv_data_st opmv;

void OpenMV_Byte_Get(u8 bytedata);
void OpenMV_Offline_Check(u8 dT_ms);

#endif
