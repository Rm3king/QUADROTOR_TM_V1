//#include "DUT_Data.h"
//#include "OnekeyFlightCtrl.h"
//#include "DUT_RC.h"
//#include "Data_mag.h"
//#include "DUT_IMU.h"
//#include "PC_User.h"
//#include "bsp_T265.h"

//画矩形
#include "Ano_RC.h"
#include "Ano_DT.h"
#include "Ano_ProgramCtrl_User.h"
#include "Ano_FlightCtrl.h"

u32 TimeStamp = 0, time = 0;
static u8 stage = 0, switchflag = 0;
uint8_t start_flag = 0;
uint8_t cmd_take_off_f = 0;

uint8_t Get_start_flag()
{
    return start_flag;
}

static inline void reset_timer()
{
    TimeStamp = 0;
}

void reset_mission()
{
    TimeStamp = 0;
    time = 0;
    stage = 0;
    switchflag = 0;
    start_flag = 0;
    cmd_take_off_f = 0;
    Program_Ctrl_User_Set_HXYcmps(0, 0);
    Program_Ctrl_User_Set_YAWdps(0);
}

void Takeoff()
{
    if(flag.flight_mode == LOC_HOLD && switchs.of_flow_on)
    {
        if(flag.auto_take_off_land == AUTO_TAKE_OFF_NULL)
        {
            if(cmd_take_off_f == 0)
            {
                cmd_take_off_f = 1;
                ANO_DT_SendString("Take off!");
                one_key_take_off();
            }
        }
    }
}

void StageTask0(u32 timestamp)   // 等待触发
{
    if(CH_N[AUX3] < 0)
    {
        switchflag = 1;
    }

    if(switchflag == 1 && CH_N[AUX3] > 0)
    {
        ANO_DT_SendString("Stage1: Takeoff Pending");
        start_flag = 1;
        stage = 1;
        reset_timer();
        time = 2000;
    }
}

void StageTask1(u32 timestamp)   // 5秒后起飞
{
    if(timestamp < time) return;

    Takeoff();
    ANO_DT_SendString("Stage2: Take off");
    stage = 2;
    reset_timer();
    time = 6000;   // 起飞后悬停6秒
}

void StageTask2(u32 timestamp)   // 起飞后悬停
{
    Program_Ctrl_User_Set_HXYcmps(0, 0);
    Program_Ctrl_User_Set_YAWdps(0);

    if(timestamp < time) return;

    ANO_DT_SendString("Stage3: Forward");
    stage = 3;
    reset_timer();
    time = 3000;   // 前进5秒
}

void StageTask3(u32 timestamp)   // 前进
{
    if(timestamp < time)
    {
        Program_Ctrl_User_Set_HXYcmps(25, 0);
        Program_Ctrl_User_Set_YAWdps(0);
        return;
    }

    Program_Ctrl_User_Set_HXYcmps(0, 0);
    ANO_DT_SendString("Stage4: Left");
    stage = 4;
    reset_timer();
    time = 3000;   // 左移5秒
}

void StageTask4(u32 timestamp)   // 左移
{
    if(timestamp < time)
    {
        Program_Ctrl_User_Set_HXYcmps(0, -25);   // 如果方向反了改成 (0, -20)
        Program_Ctrl_User_Set_YAWdps(0);
        return;
    }

    Program_Ctrl_User_Set_HXYcmps(0, 0);
    ANO_DT_SendString("Stage5: Backward");
    stage = 5;
    reset_timer();
    time = 3000;   // 后退5秒
}

void StageTask5(u32 timestamp)   // 后退
{
    if(timestamp < time)
    {
        Program_Ctrl_User_Set_HXYcmps(-25, 0);
        Program_Ctrl_User_Set_YAWdps(0);
        return;
    }

    Program_Ctrl_User_Set_HXYcmps(0, 0);
    ANO_DT_SendString("Stage6: Right");
    stage = 6;
    reset_timer();
    time = 3000;   // 右移5秒
}

void StageTask6(u32 timestamp)   // 右移
{
    if(timestamp < time)
    {
        Program_Ctrl_User_Set_HXYcmps(0, 25);  // 如果方向反了改成 (0, 20)
        Program_Ctrl_User_Set_YAWdps(0);
        return;
    }

    Program_Ctrl_User_Set_HXYcmps(0, 0);
    ANO_DT_SendString("Stage7: Hover");
    stage = 7;
    reset_timer();
    time = 4000;   // 悬停5秒
}

void StageTask7(u32 timestamp)   // 回到起点附近悬停
{
    Program_Ctrl_User_Set_HXYcmps(0, 0);
    Program_Ctrl_User_Set_YAWdps(0);

    if(timestamp < time) return;

    ANO_DT_SendString("Stage8: Land");
    one_key_land();
    stage = 8;
    reset_timer();
    time = 4000;
}

void StageTask8(u32 timestamp)   // 任务结束，复位
{
    Program_Ctrl_User_Set_HXYcmps(0, 0);
    Program_Ctrl_User_Set_YAWdps(0);

    if(timestamp < time) return;

    ANO_DT_SendString("Mission Reset");
    reset_mission();
}

void InspectionTask(u8 dT_ms)
{
    TimeStamp += dT_ms;

    if(1)
    {
        if(stage == 0) StageTask0(TimeStamp);
        else if(stage == 1) StageTask1(TimeStamp);
        else if(stage == 2) StageTask2(TimeStamp);
        else if(stage == 3) StageTask3(TimeStamp);
        else if(stage == 4) StageTask4(TimeStamp);
        else if(stage == 5) StageTask5(TimeStamp);
        else if(stage == 6) StageTask6(TimeStamp);
        else if(stage == 7) StageTask7(TimeStamp);
        else if(stage == 8) StageTask8(TimeStamp);
    }
    else
    {
        Program_Ctrl_User_Set_HXYcmps(0, 0);
        Program_Ctrl_User_Set_YAWdps(0);
        one_key_land();
        reset_mission();
    }

    if(CH_N[AUX4] > 0)
    {
        ANO_DT_SendString("Must Land");
        Program_Ctrl_User_Set_HXYcmps(0, 0);
        Program_Ctrl_User_Set_YAWdps(0);
        one_key_land();
        reset_mission();
    }
}

//一键起飞接入手动
//#include "Ano_RC.h"
//#include "Ano_DT.h"
//#include "Ano_ProgramCtrl_User.h"
//#include "Ano_FlightCtrl.h"

//u32 TimeStamp = 0, time = 0;
//static u8 stage = 0, switchflag = 0;
//uint8_t start_flag = 0;
//uint8_t cmd_take_off_f = 0;

//uint8_t Get_start_flag()
//{
//    return start_flag;
//}

//static inline void reset_timer()
//{
//    TimeStamp = 0;
//}

//void reset_mission()
//{
//    TimeStamp = 0;
//    time = 0;
//    stage = 0;
//    switchflag = 0;
//    start_flag = 0;
//    cmd_take_off_f = 0;
//    Program_Ctrl_User_Set_HXYcmps(0, 0);
//    Program_Ctrl_User_Set_YAWdps(0);
//}

//void Takeoff()
//{
//    if(flag.flight_mode == LOC_HOLD && switchs.of_flow_on)
//    {
//        if(flag.auto_take_off_land == AUTO_TAKE_OFF_NULL)
//        {
//            if(cmd_take_off_f == 0)
//            {
//                cmd_take_off_f = 1;
//                ANO_DT_SendString("Take off!");
//                one_key_take_off();
//            }
//        }
//    }
//}

//void StageTask0(u32 timestamp)   // 等待触发
//{
//    if(CH_N[AUX3] < 0)
//    {
//        switchflag = 1;
//    }

//    if(switchflag == 1 && CH_N[AUX3] > 0)
//    {
//        ANO_DT_SendString("Stage1: Takeoff Pending");
//        start_flag = 1;
//        stage = 1;
//        reset_timer();
//        time = 2000;   // 延时2秒后起飞
//    }
//}

//void StageTask1(u32 timestamp)   // 发起起飞
//{
//    if(timestamp < time) return;

//    Takeoff();
//    ANO_DT_SendString("Stage2: Take off");
//    stage = 2;
//    reset_timer();
//    time = 6000;   // 起飞后悬停6秒，再交还手动
//}

//void StageTask2(u32 timestamp)   // 起飞后悬停，然后交还手动
//{
//    Program_Ctrl_User_Set_HXYcmps(0, 0);
//    Program_Ctrl_User_Set_YAWdps(0);

//    if(timestamp < time) return;

//    ANO_DT_SendString("Takeoff OK: Manual Control");
//    reset_mission();   // 不降落，直接退出自动任务，交还手动
//}

//void InspectionTask(u8 dT_ms)
//{
//    TimeStamp += dT_ms;

//    if(stage == 0) StageTask0(TimeStamp);
//    else if(stage == 1) StageTask1(TimeStamp);
//    else if(stage == 2) StageTask2(TimeStamp);

//    if(CH_N[AUX4] > 0)
//    {
//        ANO_DT_SendString("Must Land");
//        Program_Ctrl_User_Set_HXYcmps(0, 0);
//        Program_Ctrl_User_Set_YAWdps(0);
//        one_key_land();
//        reset_mission();
//    }
//}