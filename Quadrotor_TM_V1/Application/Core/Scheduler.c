#include "Scheduler.h"
#include "Drv_Bsp.h"
#include "Drv_icm20602.h"
#include "LED.h"
#include "FlightDataCal.h"
#include "Sensor_Basic.h"
#include "Drv_gps.h"
#include "DT.h"
#include "RC.h"
#include "Parameter.h"
#include "Drv_led.h"
#include "Drv_ak8975.h"
#include "Drv_spl06.h"
#include "FlightCtrl.h"
#include "AttCtrl.h"
#include "LocCtrl.h"
#include "AltCtrl.h"
#include "MotorCtrl.h"
#include "MagProcess.h"
#include "Power.h"
#include "OF.h"
#include "Drv_heating.h"
#include "FlyCtrl.h"
#include "UWB.h"
#include "Drv_OpenMV.h"
#include "OPMV_CBTracking_Ctrl.h"
#include "OPMV_LineTracking_Ctrl.h"
#include "OPMV_Ctrl.h"
#include "OF_DecoFusion.h"
#include "User_control.h"
/*
 * 文件名称: Scheduler.c
 * 所属模块: Application / Core
 *
 * 功能描述:
 *   协作式任务调度器，基于 1ms 硬件定时器中断驱动所有飞控任务：
 *
 *   INT_1ms_Task()   -- 1ms 中断：累加节拍 + LED PWM 驱动
 *   Main_Task()      -- 主循环轮询入口
 *
 * 任务周期表:
 *   ┌──────────┬────────┬─────────────────────────────────────┐
 *   │ 周期     │ 函数   │ 主要任务                            │
 *   ├──────────┼────────┼─────────────────────────────────────┤
 *   │ 1ms      │ Task_0 │ 传感器读取 (Fc_Sensor_Get)          │
 *   │ 2ms      │ Task_1 │ 姿态解算 + 姿态控制 + 电机输出      │
 *   │ 6ms      │ Task_2 │ 遥控处理 + 高度控制 + 位置控制      │
 *   │ 11ms     │ Task_5 │ LED 状态 + 光流融合                 │
 *   │ 20ms     │ Task_8 │ 数据发送 + GPS + 导航               │
 *   │ 50ms     │ Task_9 │ 电源监测 + 参数保存                 │
 *   └──────────┴────────┴─────────────────────────────────────┘
 *
 * 调度机制:
 *   采用简单的计数轮询（非抢占）：1ms 中断仅累加标志，
 *   主循环依次检查各任务标志并执行。dT_us 参数记录实际间隔用于控制计算。
 *
 * 架构位置:
 *   Main_Task() 由 main.c 的 while(1) 循环调用。
 *   INT_1ms_Task() 由硬件定时器中断调用。
 *
 * 教学提示:
 *   - 这是最简单的协作式调度器，适合教学理解实时任务调度
 *   - 任务执行顺序决定了控制链的时序：传感器→姿态→控制→输出
 *   - dT_us 使用 SysTick 精确计时，比固定周期假设更准确
 */
#define CIRCLE_NUM   20

static u8 lt0_run_flag;
static u8 circle_cnt[2];

static void Loop_Task_0(void);
static void Loop_Task_1(u32 dT_us);
static void Loop_Task_2(u32 dT_us);
static void Loop_Task_5(u32 dT_us);
static void Loop_Task_8(u32 dT_us);
static void Loop_Task_9(u32 dT_us);

/*
 * 功能：1ms 中断任务。
 * 说明：累加调度节拍，同时驱动 LED 软件 PWM。
 */
void INT_1ms_Task(void)
{
    lt0_run_flag++;
    LED_1ms_DRV();
    circle_cnt[0]++;
    circle_cnt[0] %= CIRCLE_NUM;
    if(!circle_cnt[0])
    {
    }
}

/* 1ms 周期任务。 */
static void Loop_Task_0(void)
{
    Fc_Sensor_Get();
    Sensor_Data_Prepare(1);
    IMU_Update_Task(1);
    WCZ_Acc_Get_Task();
    WCXY_Acc_Get_Task();
    Flight_State_Task(1);
    Swtich_State_Task(1);
    ANO_OF_Data_Prepare_Task(0.001f);
    ANO_DT_Data_Exchange();
}

/* 2ms 周期任务。 */
static void Loop_Task_1(u32 dT_us)
{
    (void)dT_us;
    Att_1level_Ctrl(2e-3f);
    Motor_Ctrl_Task(2);
}

/* 6ms 周期任务。 */
static void Loop_Task_2(u32 dT_us)
{
    (void)dT_us;
    calculate_RPY();
    Att_2level_Ctrl(6e-3f);
}

/* 11ms 周期任务。 */
static void Loop_Task_5(u32 dT_us)
{
    (void)dT_us;
    RC_duty_task(11);
    Flight_Mode_Set(11);
    if(flag.flight_mode == SUDDEN_STOP)
    {
        Sudden_Stop_Task();
        return;
    }
    WCZ_Fus_Task(11);
    GPS_Data_Processing_Task(11);
    Alt_1level_Ctrl(11e-3f);
    Alt_2level_Ctrl(11e-3f);
    AnoOF_DataAnl_Task(11);
    LED_Task2(11);
}

/* 20ms 周期任务。 */
static void Loop_Task_8(u32 dT_us)
{
    (void)dT_us;
    Mag_Update_Task(20);
    FlyCtrl_Task(20);
    ANO_OFDF_Task(20);
    UWB_DataCalcTask(20);
    Loc_1level_Ctrl(20);
    OpenMV_Offline_Check(20);
    ANO_CBTracking_Task(20);
    ANO_LTracking_Task(20);
    ANO_OPMV_Ctrl_Task(20);
    InspectionTask(20);
}

/* 50ms 周期任务。 */
static void Loop_Task_9(u32 dT_us)
{
    (void)dT_us;
    Power_UpdateTask(50);
    Thermostatic_Ctrl_Task(50);
    FC_Param_WriteTask(50);
}

/* 调度表。 */
static sched_task_t sched_tasks[] =
{
    {Loop_Task_1,  2000,  0},
    {Loop_Task_2,  6000,  0},
    {Loop_Task_5, 11000,  0},
    {Loop_Task_8, 20000,  0},
    {Loop_Task_9, 50000,  0},
};

#define TASK_NUM   (sizeof(sched_tasks) / sizeof(sched_task_t))

/*
 * 功能：主循环调度入口。
 * 说明：先处理 1ms 快速任务，再按周期轮询其余任务。
 */
u8 Main_Task(void)
{
    uint8_t index;
    uint32_t time_now, delta_time_us;

    if(lt0_run_flag != 0)
    {
        lt0_run_flag--;
        Loop_Task_0();
    }

    for(index = 0; index < TASK_NUM; index++)
    {
        time_now = GetSysRunTimeUs();
        if(time_now - sched_tasks[index].last_run >= sched_tasks[index].interval_ticks)
        {
            delta_time_us = (u32)(time_now - sched_tasks[index].last_run);
            sched_tasks[index].last_run = time_now;
            sched_tasks[index].task_func(delta_time_us);
        }
    }

    return 0;
}
