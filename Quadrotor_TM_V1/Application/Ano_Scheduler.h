#ifndef _SCHEDULER_H_
#define _SCHEDULER_H_
#include "config.h"
/*
 * ???????
 * task_func ???????????????? us?
 */
typedef struct
{
    void(*task_func)(u32 dT_us);
    u32 interval_ticks;
    u32 last_run;
} sched_task_t;
u8 Main_Task(void);
void INT_1ms_Task(void);
#endif