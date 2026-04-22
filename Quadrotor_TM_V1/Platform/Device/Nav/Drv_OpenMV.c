#include "Drv_OpenMV.h"
#include "Ano_DT.h"
/*
 * 模块说明。
 * OpenMV 数据接收与解析驱动。
 * 负责按协议组包 OpenMV 串口数据，解析色块跟踪与寻线结果，并维护离线检测状态。
 */
static void OpenMV_Data_Analysis(u8 *buf_data, u8 len);
static void OpenMV_Check_Reset(void);

#define OPMV_OFFLINE_TIME_MS   1000
#define OPENMV_FRAME_MAX_LEN   20

u16 offline_check_time;
u8 openmv_buf[OPENMV_FRAME_MAX_LEN];
_openmv_data_st opmv;

/*
 * 功能：按字节接收 OpenMV 数据。
 * 说明：根据帧头、功能字和长度逐步组包，校验通过后再进入解析。
 */
void OpenMV_Byte_Get(u8 bytedata)
{
    static u8 len = 0, rec_sta;
    u8 check_val = 0;

    openmv_buf[rec_sta] = bytedata;
    if(rec_sta == 0)
    {
        if(bytedata == 0xaa)
        {
            rec_sta++;
        }
        else
        {
            rec_sta = 0;
        }
    }
    else if(rec_sta == 1)
    {
        if(1)
        {
            rec_sta++;
        }
        else
        {
            rec_sta = 0;
        }
    }
    else if(rec_sta == 2)
    {
        if(bytedata == 0x05)
        {
            rec_sta++;
        }
        else
        {
            rec_sta = 0;
        }
    }
    else if(rec_sta == 3)
    {
        if(bytedata == 0x41 || bytedata == 0x42)
        {
            rec_sta++;
        }
        else
        {
            rec_sta = 0;
        }
    }
    else if(rec_sta == 4)
    {
        len = bytedata;
        if(len < OPENMV_FRAME_MAX_LEN)
        {
            rec_sta++;
        }
        else
        {
            rec_sta = 0;
        }
    }
    else if(rec_sta == (len + 5))
    {
        u8 i;
        for(i = 0; i < len + 5; i++)
        {
            check_val += openmv_buf[i];
        }
        if(check_val == bytedata)
        {
            OpenMV_Data_Analysis(openmv_buf, len + 6);
        }
        rec_sta = 0;
    }
    else
    {
        rec_sta++;
    }
}

/*
 * 功能：解析 OpenMV 数据帧。
 * 说明：支持色块跟踪帧和寻线帧两种类型，并置位对应的发送标记。
 */
static void OpenMV_Data_Analysis(u8 *buf_data, u8 len)
{
    (void)len;
    if(*(buf_data + 3) == 0x41)
    {
        opmv.cb.color_flag = *(buf_data + 5);
        opmv.cb.sta = *(buf_data + 6);
        opmv.cb.pos_x = (s16)((*(buf_data + 7) << 8) | *(buf_data + 8));
        opmv.cb.pos_y = (s16)((*(buf_data + 9) << 8) | *(buf_data + 10));
        opmv.cb.dT_ms = *(buf_data + 11);
        opmv.mode_sta = 1;
        f.send_omv_ct = 1;
    }
    else if(*(buf_data + 3) == 0x42)
    {
        opmv.lt.sta = *(buf_data + 5);
        opmv.lt.angle = (s16)((*(buf_data + 6) << 8) | *(buf_data + 7));
        opmv.lt.deviation = (s16)((*(buf_data + 8) << 8) | *(buf_data + 9));
        opmv.lt.p_flag = *(buf_data + 10);
        opmv.lt.pos_x = (s16)((*(buf_data + 11) << 8) | *(buf_data + 12));
        opmv.lt.pos_y = (s16)((*(buf_data + 13) << 8) | *(buf_data + 14));
        opmv.lt.dT_ms = *(buf_data + 15);
        opmv.mode_sta = 2;
        f.send_omv_lt = 1;
    }
    OpenMV_Check_Reset();
}

/* 功能：OpenMV 离线检测。 */
void OpenMV_Offline_Check(u8 dT_ms)
{
    if(offline_check_time < OPMV_OFFLINE_TIME_MS)
    {
        offline_check_time += dT_ms;
    }
    else
    {
        opmv.offline = 1;
        opmv.mode_sta = 0;
    }
}

/* 功能：刷新 OpenMV 在线状态。 */
static void OpenMV_Check_Reset(void)
{
    offline_check_time = 0;
    opmv.offline = 0;
}
