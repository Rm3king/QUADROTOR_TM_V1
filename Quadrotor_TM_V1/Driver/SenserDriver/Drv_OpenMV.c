#include "Drv_OpenMV.h"
#include "Ano_DT.h"
/*
 * ?????
 * OpenMV ??????????
 *
 * ?????????? OpenMV ????????????????
 * ?????????????
 */
static void OpenMV_Data_Analysis(u8 *buf_data, u8 len);
static void OpenMV_Check_Reset(void);
#define OPMV_OFFLINE_TIME_MS   1000
#define OPENMV_FRAME_MAX_LEN   20
u16 offline_check_time;
u8 openmv_buf[OPENMV_FRAME_MAX_LEN];
_openmv_data_st opmv;
/*
 * ???????? OpenMV ???
 * ???
 * ????????????????????????????
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
 * ????? OpenMV ????
 * ???
 * ?????????????????????????????
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
/* ??????? */
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
/* ?????????????? */
static void OpenMV_Check_Reset(void)
{
    offline_check_time = 0;
    opmv.offline = 0;
}