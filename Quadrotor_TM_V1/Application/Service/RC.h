#ifndef __RC_H__
#define __RC_H__
#include "FcData.h"

/* 摇杆功能识别状态。 */
typedef struct
{
    u16 s_cnt;
    u8 s_now_times;
    u8 s_state;
} _stick_f_c_st;

/* 摇杆低通滤波状态类型。 */
typedef u16 _stick_f_lp_st;

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

/* RC 模块公开接口 */
s16  RC_GetChannel(u8 ch);
void RC_GetAllChannels(s16 *out, u8 count);
u8   RC_GetChannelEnableMask(void);

void fail_safe_check(u8 dT_ms);
void Remote_Control_Init(void);
void RC_duty_task(u8 dT_ms);
void ch_watch_dog_feed(u8 ch_n);

#define FC_Rc_Init                Remote_Control_Init
#define FC_Rc_Task                RC_duty_task
#define FC_Rc_FailSafeCheck       fail_safe_check
#define FC_Rc_ChannelWatchdogFeed ch_watch_dog_feed

#endif
