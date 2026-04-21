#ifndef _RC_H_
#define _RC_H_
#include "Ano_FcData.h"

/* 摇杆功能识别状态。 */
typedef struct
{
    u16 s_cnt;
    u8 s_now_times;
    u8 s_state;
} _stick_f_c_st;

/* 摇杆低通滤波状态类型。 */
#define _stick_f_lp_st   u16

/* 遥控通道索引。 */
enum
{
    CH1 = 0,
    CH2,
    CH3,
    CH4,
    CH5,
    CH6,
    CH7,
    CH8
};

/* 当前遥控通道量，范围约为 -500~500。 */
extern s16 CH_N[CH_NUM];
/* 信号强度累计值，由接收中断喂入并周期采样。 */
extern u16 signal_intensity;
/* 当前有效通道位图。 */
extern u8 chn_en_bit;

void fail_safe_check(u8 dT_ms);
void Remote_Control_Init(void);
void RC_duty_task(u8 dT_ms);
void ch_watch_dog_feed(u8 ch_n);

#endif
