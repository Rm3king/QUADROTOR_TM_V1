#ifndef __PID_H__
#define __PID_H__
/*
 * 模块：PID 控制
 * 职责：提供 PID 计算函数和参数/数据结构。
 */
#include "FcData.h"
typedef struct
{
    u8 fb_d_mode;
    float kp;
    float ki;
    float kd_ex;
    float kd_fb;
    float k_ff;
} _PID_arg_st;
typedef struct
{
    float err;
    float exp_old;
    float feedback_old;
    float fb_d;
    float fb_d_ex;
    float exp_d;
    float err_i;
    float ff;
    float pre_d;
    float out;
} _PID_val_st;
float PID_calculate(float T,
                    float in_ff,
                    float expect,
                    float feedback,
                    _PID_arg_st *pid_arg,
                    _PID_val_st *pid_val,
                    float inte_d_lim,
                    float inte_lim);
#endif
