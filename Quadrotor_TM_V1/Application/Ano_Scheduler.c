#include "Ano_Scheduler.h"
#include "Drv_Bsp.h"
#include "Drv_icm20602.h"
#include "Ano_LED.h"
#include "Ano_FlightDataCal.h"
#include "Ano_Sensor_Basic.h"
#include "Drv_gps.h"
#include "Ano_DT.h"
#include "Ano_RC.h"
#include "Ano_Parameter.h"
#include "Drv_led.h"
#include "Drv_ak8975.h"
#include "Drv_spl06.h"
#include "Ano_FlightCtrl.h"
#include "Ano_AttCtrl.h"
#include "Ano_LocCtrl.h"
#include "Ano_AltCtrl.h"
#include "Ano_MotorCtrl.h"
#include "Ano_MagProcess.h"
#include "Ano_Power.h"
#include "Ano_OF.h"
#include "Drv_heating.h"
#include "Ano_FlyCtrl.h"
#include "Ano_UWB.h"
#include "Drv_OpenMV.h"
#include "Ano_OPMV_CBTracking_Ctrl.h"
#include "Ano_OPMV_LineTracking_Ctrl.h"
#include "Ano_OPMV_Ctrl.h"
#include "Ano_OF_DecoFusion.h"
#include "User_control.h"
/*
 * ?????
 * ???????
 *
 * ??????? 1ms ??????? 2ms?6ms?11ms?20ms?50ms
 * ?????????????????????????
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
 * ???1ms ?????
 * ???
 * ?????????? LED ? PWM???????????
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
/* 1ms ????? */
static void Loop_Task_0(void)
{
    Fc_Sensor_Get();
    Sensor_Data_Prepare(1);
    IMU_Update_Task(1);
    WCZ_Acc_Get_Task();
    WCXY_Acc_Get_Task();
    Flight_State_Task(1, CH_N);
    Swtich_State_Task(1);
    ANO_OF_Data_Prepare_Task(0.001f);
    ANO_DT_Data_Exchange();
}
/* 2ms ????? */
static void Loop_Task_1(u32 dT_us)
{
    (void)dT_us;
    Att_1level_Ctrl(2e-3f);
    Motor_Ctrl_Task(2);
}
/* 6ms ????? */
static void Loop_Task_2(u32 dT_us)
{
    (void)dT_us;
    calculate_RPY();
    Att_2level_Ctrl(6e-3f, CH_N);
}
/* 11ms ????? */
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
/* 20ms ????? */
static void Loop_Task_8(u32 dT_us)
{
    (void)dT_us;
    Mag_Update_Task(20);
    FlyCtrl_Task(20);
    ANO_OFDF_Task(20);
    Ano_UWB_Data_Calcu_Task(20);
    Loc_1level_Ctrl(20, CH_N);
    OpenMV_Offline_Check(20);
    ANO_CBTracking_Task(20);
    ANO_LTracking_Task(20);
    ANO_OPMV_Ctrl_Task(20);
    InspectionTask(20);
}
/* 50ms ????? */
static void Loop_Task_9(u32 dT_us)
{
    (void)dT_us;
    Power_UpdateTask(50);
    Thermostatic_Ctrl_Task(50);
    Ano_Parame_Write_task(50);
}
/* ?????? */
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
 * ???????????
 * ???
 * ??? 1ms ???????????????????????
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