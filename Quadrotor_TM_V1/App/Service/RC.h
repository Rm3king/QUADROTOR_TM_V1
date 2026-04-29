#ifndef _RC_H_
#define _RC_H_
#include "FcData.h"

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

/*
 * 第二阶段语义接口：
 * 当前先通过兼容别名暴露更清晰的命名，底层实现与旧接口保持一致。
 * 后续如果继续推进，再逐步迁移实现文件内部符号。
 */
#define g_rc_channels             CH_N
#define g_rc_signal_intensity     signal_intensity
#define g_rc_channel_enable_mask  chn_en_bit

#define FC_Rc_Init                Remote_Control_Init
#define FC_Rc_Task                RC_duty_task
#define FC_Rc_FailSafeCheck       fail_safe_check
#define FC_Rc_ChannelWatchdogFeed ch_watch_dog_feed

void fail_safe_check(u8 dT_ms);
void Remote_Control_Init(void);
void RC_duty_task(u8 dT_ms);
void ch_watch_dog_feed(u8 ch_n);

#endif
