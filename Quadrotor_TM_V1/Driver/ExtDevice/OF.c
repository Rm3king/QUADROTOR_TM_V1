/*
 * 文件名称: OF.c
 * 所属模块: Driver / ExtDevice
 *
 * 功能描述:
 *   匿名光流模块 V3 串口协议驱动：
 *   1) AnoOF_GetOneByte()   -- 逐字节接收，状态机解析协议帧
 *   2) AnoOF_DataAnl()      -- 完整帧解析：运动数据/测距/IMU 三种消息类型
 *   3) AnoOF_Check()        -- 在线检测：超过 1 秒无数据则判定离线
 *
 * 协议帧格式:
 *   帧头1(0xAA) + 帧头2(0x22) + 设备地址 + 功能码 + 数据长度 + 数据 + 校验和
 *
 * 消息类型:
 *   0x51 (MOTION) -- 光流位移量 + 质量值
 *   0x52 (RANGE)  -- ToF 测距高度
 *   0x53 (IMU)    -- 光流模块内置 IMU 数据（备用）
 *
 * 公开变量（供控制层使用）:
 *   OF_QUALITY  -- 光流质量 (0-255)
 *   OF_DX2/DY2  -- 16位位移速度估计
 *   OF_DX2FIX/DY2FIX -- 修正后的位移速度
 *   OF_ALT      -- ToF 测距高度 (mm)
 *
 * 架构位置:
 *   AnoOF_GetOneByte() 由 UART7 接收中断逐字节调用。
 *   解析结果供 OF_DecoFusion.c 和 LocCtrl.c 使用。
 *
 * 教学提示:
 *   - 状态机解析是嵌入式串口协议的标准方法，逐字节处理无需缓存整帧
 *   - 校验和使用字节累加和，简单高效
 *   - 内部 IMU 数据（OF_GYR/ACC）当前未被控制层使用，保留供扩展
 */
#include "OF.h"
#include "FcData.h"
#include "Drv_Bsp.h"

#define ANO_OF_FRAME_HEAD1       0xAA
#define ANO_OF_FRAME_HEAD2       0x22
#define ANO_OF_MSG_MOTION        0x51
#define ANO_OF_MSG_RANGE         0x52
#define ANO_OF_MSG_IMU           0x53
#define OF_OFFLINE_THRESHOLD_MS  1000
#define OF_CHECK_CNT_MAX         10000
/*
OF_STATE :
0bit: 1-有效，0-无效
1bit: 1-有效，0-无效
2bit: 1-数据有效，0-数据无效
3bit: 1-数据有效，0-数据无效
4:0
7bit: 1
*/
static uint8_t        OF_STATE;
uint8_t               OF_QUALITY;
static int8_t         OF_DX, OF_DY;
int16_t               OF_DX2, OF_DY2, OF_DX2FIX, OF_DY2FIX;
uint16_t              OF_ALT;
static uint16_t       OF_ALT2;
static int16_t        OF_GYR_X, OF_GYR_Y, OF_GYR_Z;
static int16_t        OF_GYR_X2, OF_GYR_Y2, OF_GYR_Z2;
static int16_t        OF_ACC_X, OF_ACC_Y, OF_ACC_Z;
static int16_t        OF_ACC_X2, OF_ACC_Y2, OF_ACC_Z2;
static float          OF_ATT_ROL, OF_ATT_PIT, OF_ATT_YAW;
static float          OF_ATT_S1, OF_ATT_S2, OF_ATT_S3, OF_ATT_S4;

void AnoOF_DataAnl(uint8_t *data_buf, uint8_t num);

static uint8_t _datatemp[50];
static u8 _data_cnt = 0;
static u8 of_check_f[2];
static u16 of_check_cnt[2] = {OF_CHECK_CNT_MAX, OF_CHECK_CNT_MAX};

void AnoOF_DataAnl_Task(u8 dT_ms)
{
    AnoOF_Check(dT_ms);
}

/* 逐字节接收光流串口数据并解析 */
void AnoOF_GetOneByte(uint8_t data)
{
    /* 匿名光流 V3.0 协议解析 */
    static u8 _data_len = 0;
    static u8 state = 0;

    if(state == 0 && data == ANO_OF_FRAME_HEAD1)
    {
        state = 1;
        _datatemp[0] = data;
    }
    else if(state == 1 && data == ANO_OF_FRAME_HEAD2)
    {
        state = 2;
        _datatemp[1] = data;
    }
    else if(state == 2)
    {
        state = 3;
        _datatemp[2] = data;
    }
    else if(state == 3)
    {
        state = 4;
        _datatemp[3] = data;
    }
    else if(state == 4)
    {
        state = 5;
        _datatemp[4] = data;
        _data_len = data;
        _data_cnt = 0;
    }
    else if(state == 5 && _data_len > 0)
    {
        _data_len--;
        _datatemp[5 + _data_cnt++] = data;
        if(_data_len == 0)
        {
            state = 6;
        }
    }
    else if(state == 6)
    {
        state = 0;
        _datatemp[5 + _data_cnt] = data;
        AnoOF_DataAnl(_datatemp, _data_cnt + 6);
    }
    else
    {
        state = 0;
    }
}

void AnoOF_Check(u8 dT_ms)
{
    for(u8 i = 0; i < 2; i++)
    {
        if(of_check_f[i] == 0)
        {
            if(of_check_cnt[i] < OF_CHECK_CNT_MAX)
            {
                of_check_cnt[i] += dT_ms;
            }
        }
        else
        {
            of_check_cnt[i] = 0;
        }

        of_check_f[i] = 0;
    }

    if(of_check_cnt[0] > OF_OFFLINE_THRESHOLD_MS || of_check_cnt[1] > OF_OFFLINE_THRESHOLD_MS)
    {
        sens_hd_check.of_ok = 0;
    }
    else
    {
        sens_hd_check.of_ok = 1;
    }
}

void AnoOF_DataAnl(uint8_t *data_buf, uint8_t num)
{
    u8 sum = 0;

    /* 匿名光流 V3.0 数据帧解析 */
    for(u8 i = 0; i < (num - 1); i++)
    {
        sum += *(data_buf + i);
    }
    if(!(sum == *(data_buf + num - 1)))
    {
        return;
    }

    if(*(data_buf + 3) == ANO_OF_MSG_MOTION)
    {
        if(*(data_buf + 5) == 0)
        {
            OF_STATE = *(data_buf + 6);
            OF_DX = *(data_buf + 7);
            OF_DY = *(data_buf + 8);
            OF_QUALITY = *(data_buf + 9);
        }
        else if(*(data_buf + 5) == 1)
        {
            OF_STATE = *(data_buf + 6);
            OF_DX2 = (int16_t)(*(data_buf + 7) << 8) | *(data_buf + 8);
            OF_DY2 = (int16_t)(*(data_buf + 9) << 8) | *(data_buf + 10);
            OF_DX2FIX = (int16_t)(*(data_buf + 11) << 8) | *(data_buf + 12);
            OF_DY2FIX = (int16_t)(*(data_buf + 13) << 8) | *(data_buf + 14);
            OF_QUALITY = *(data_buf + 19);

            of_check_f[0] = 1;
            of_init_type = 1;
        }
    }

    if(*(data_buf + 3) == ANO_OF_MSG_RANGE)
    {
        if(*(data_buf + 5) == 0)
        {
            OF_ALT = (uint16_t)(*(data_buf + 6) << 8) | *(data_buf + 7);
            of_check_f[1] = 1;
        }
        else if(*(data_buf + 5) == 1)
        {
            OF_ALT2 = (uint16_t)(*(data_buf + 6) << 8) | *(data_buf + 7);
        }
    }

    if(*(data_buf + 3) == ANO_OF_MSG_IMU)
    {
        if(*(data_buf + 5) == 0)
        {
            OF_GYR_X = (int16_t)(*(data_buf + 6) << 8) | *(data_buf + 7);
            OF_GYR_Y = (int16_t)(*(data_buf + 8) << 8) | *(data_buf + 9);
            OF_GYR_Z = (int16_t)(*(data_buf + 10) << 8) | *(data_buf + 11);
            OF_ACC_X = (int16_t)(*(data_buf + 12) << 8) | *(data_buf + 13);
            OF_ACC_Y = (int16_t)(*(data_buf + 14) << 8) | *(data_buf + 15);
            OF_ACC_Z = (int16_t)(*(data_buf + 16) << 8) | *(data_buf + 17);
        }
        else if(*(data_buf + 5) == 1)
        {
            OF_GYR_X2 = (int16_t)(*(data_buf + 6) << 8) | *(data_buf + 7);
            OF_GYR_Y2 = (int16_t)(*(data_buf + 8) << 8) | *(data_buf + 9);
            OF_GYR_Z2 = (int16_t)(*(data_buf + 10) << 8) | *(data_buf + 11);
            OF_ACC_X2 = (int16_t)(*(data_buf + 12) << 8) | *(data_buf + 13);
            OF_ACC_Y2 = (int16_t)(*(data_buf + 14) << 8) | *(data_buf + 15);
            OF_ACC_Z2 = (int16_t)(*(data_buf + 16) << 8) | *(data_buf + 17);
        }
    }
}
