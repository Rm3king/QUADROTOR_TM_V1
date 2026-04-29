#ifndef _SCHEDULER_H_
#define _SCHEDULER_H_
#include "config.h"
/*
 * 调度任务描述。
 * task_func 为任务入口，interval_ticks 和 last_run 的单位均为 us。
 */
typedef struct
{
    void(*task_func)(u32 dT_us);
    u32 interval_ticks;
    u32 last_run;
} sched_task_t;

void INT_1ms_Task(void);
u8 Main_Task(void);

#endif
