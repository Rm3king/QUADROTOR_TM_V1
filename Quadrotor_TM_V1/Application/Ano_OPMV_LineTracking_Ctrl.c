#include "Ano_OPMV_LineTracking_Ctrl.h"
#include "Drv_OpenMV.h"
#include "Ano_OPMV_Ctrl.h"
#include "Ano_Math.h"
#include "Ano_Filter.h"
#include "ANO_IMU.h"
#include "Ano_FlightCtrl.h"
/*
 * 模块说明。
 * OpenMV 寻线控制实现。
 * 负责根据寻线偏差完成姿态解耦、误差估计以及速度和航向输出计算。
 */
static void ANO_LTracking_Decoupling(u8 *dT_ms, float rol_degs, float pit_degs);
static void ANO_LTracking_Calcu(u8 *dT_ms, s32 relative_height_cm);

#define IMU_ROL                 (imu_data.rol)
#define IMU_PIT                 (imu_data.pit)
#define RELATIVE_HEIGHT_CM      (jsdata.valid_of_alt_cm)
#define LT_KP                   (0.80f)
#define LT_KD                   (0.05f)

_ano_opmv_lt_ctrl_st ano_opmv_lt_ctrl;
static u16 line_loss_hold_time;
static float lt_decou_pos_pixel_lpf[2];
static u8 step_pro_sta;

/* 参数标定值。 */
#define LT_PIXELPDEG    2.4f
#define LT_CMPPIXEL     0.01f
#define LLH_TIME        1000
#define CONFIRM_TIMES   10
#define YAW_PAL_DPS     90
#define FORWARD_VEL     50

/*
 * 功能：执行寻线数据预处理。
 * 说明：在 OpenMV 寻线模式下完成姿态解耦和误差估计。
 */
void ANO_LTracking_Task(u8 dT_ms)
{
    if(opmv.mode_sta == 2)
    {
        ANO_LTracking_Decoupling(&dT_ms, IMU_ROL, IMU_PIT);
        ANO_LTracking_Calcu(&dT_ms, (s32)RELATIVE_HEIGHT_CM);
    }
    else
    {
        ano_opmv_lt_ctrl.target_loss = 1;
    }
}

/* 功能：执行寻线姿态解耦。 */
static void ANO_LTracking_Decoupling(u8 *dT_ms,float rol_degs,float pit_degs)
{
    float dT_s = (*dT_ms) * 1e-3f;

    if(opmv.lt.sta != 0)
    {
        ano_opmv_lt_ctrl.target_loss = 0;
        line_loss_hold_time = 0;
    }
    else
    {
        if(line_loss_hold_time < LLH_TIME)
        {
            line_loss_hold_time += *dT_ms;
        }
        else
        {
            ano_opmv_lt_ctrl.target_loss = 1;
        }
    }

    ano_opmv_lt_ctrl.opmv_pos = opmv.lt.deviation;

    if(opmv.lt.sta != 0)
    {
        /* 当前版本保留姿态补偿关闭策略，仅保留限幅框架。 */
        (void)rol_degs;
        (void)pit_degs;
        ano_opmv_lt_ctrl.r2pixel_val = 0;
        ano_opmv_lt_ctrl.r2pixel_val = LIMIT(ano_opmv_lt_ctrl.r2pixel_val, -80, 80);
    }

    if(ano_opmv_lt_ctrl.target_loss == 0)
    {
        lt_decou_pos_pixel_lpf[0] += 0.2f * ((ano_opmv_lt_ctrl.opmv_pos - ano_opmv_lt_ctrl.r2pixel_val) - lt_decou_pos_pixel_lpf[0]);
        lt_decou_pos_pixel_lpf[1] += 0.2f * (lt_decou_pos_pixel_lpf[0] - lt_decou_pos_pixel_lpf[1]);
        ano_opmv_lt_ctrl.decou_pos_pixel = lt_decou_pos_pixel_lpf[1];
    }
    else
    {
        LPF_1_(0.2f, dT_s, 0, ano_opmv_lt_ctrl.decou_pos_pixel);
    }
}

/* 功能：计算寻线地面偏差。 */
static void ANO_LTracking_Calcu(u8 *dT_ms,s32 relative_height_cm)
{
    static float relative_height_cm_valid;
    static float lt_gnd_pos_err_old;

    if(relative_height_cm < 500)
    {
        relative_height_cm_valid = relative_height_cm;
    }

    lt_gnd_pos_err_old = ano_opmv_lt_ctrl.ground_pos_err_h_cm;
    ano_opmv_lt_ctrl.ground_pos_err_h_cm = LT_CMPPIXEL * relative_height_cm_valid * ano_opmv_lt_ctrl.decou_pos_pixel;
    ano_opmv_lt_ctrl.ground_pos_err_d_h_cmps = (ano_opmv_lt_ctrl.ground_pos_err_h_cm - lt_gnd_pos_err_old) * (1000 / (*dT_ms));
}

/*
 * 功能：执行寻线分步流程。
 * 说明：保持原有状态推进、确认次数和持续时间不变。
 */
void ANO_LT_StepProcedure(u8 *dT_ms)
{
    static u8 confirm_cnt;
    static u16 elapsed_time_ms;
    switch(step_pro_sta)
    {
        case 0:
        {
            ano_opmv_lt_ctrl.exp_yaw_pal_dps = 0;
            ano_opmv_lt_ctrl.exp_velocity_h_cmps[0] = 0;
            ano_opmv_lt_ctrl.exp_velocity_h_cmps[1] = 0;
            elapsed_time_ms = 0;
            step_pro_sta = 1;
        }
        break;

        case 1:
        {
            confirm_cnt = 0;
            if(opmv.lt.sta == 1)
            {
                ano_opmv_lt_ctrl.exp_velocity_h_cmps[0] = FORWARD_VEL;
            }
            else if(opmv.lt.sta == 2)
            {
                step_pro_sta = 2;
            }
            else if(opmv.lt.sta == 3)
            {
                step_pro_sta = 3;
            }
            else if(opmv.lt.sta == 0)
            {
                step_pro_sta = 0;
            }
        }
        break;

        case 2:
        {
            if(opmv.lt.sta != 2)
            {
                step_pro_sta = 1;
            }
            else if(confirm_cnt < CONFIRM_TIMES)
            {
                confirm_cnt++;
            }
            else
            {
                confirm_cnt = 0;
                step_pro_sta = 4;
            }
        }
        break;

        case 3:
        {
            if(opmv.lt.sta != 3)
            {
                step_pro_sta = 1;
            }
            else if(confirm_cnt < CONFIRM_TIMES)
            {
                confirm_cnt++;
            }
            else
            {
                confirm_cnt = 0;
                step_pro_sta = 5;
            }
        }
        break;

        case 4:
        {
            ano_opmv_lt_ctrl.exp_velocity_h_cmps[0] = 10;
            ano_opmv_lt_ctrl.exp_velocity_h_cmps[1] = 0;
            if(elapsed_time_ms < 1500)
            {
                elapsed_time_ms += *dT_ms;
            }
            else
            {
                elapsed_time_ms = 0;
                step_pro_sta = 12;
            }
        }
        break;

        case 5:
        {
            ano_opmv_lt_ctrl.exp_velocity_h_cmps[0] = 20;
            ano_opmv_lt_ctrl.exp_velocity_h_cmps[1] = 0;
            if(elapsed_time_ms < 1500)
            {
                elapsed_time_ms += *dT_ms;
            }
            else
            {
                elapsed_time_ms = 0;
                step_pro_sta = 13;
            }
        }
        break;

        case 12:
        {
            ano_opmv_lt_ctrl.exp_velocity_h_cmps[0] = 25;
            ano_opmv_lt_ctrl.exp_velocity_h_cmps[1] = 0;
            ano_opmv_lt_ctrl.exp_yaw_pal_dps = -YAW_PAL_DPS;
            if(elapsed_time_ms < 90000 / YAW_PAL_DPS)
            {
                elapsed_time_ms += *dT_ms;
            }
            else
            {
                ano_opmv_lt_ctrl.exp_yaw_pal_dps = 0;
                elapsed_time_ms = 0;
                step_pro_sta = 20;
            }
        }
        break;

        case 13:
        {
            ano_opmv_lt_ctrl.exp_velocity_h_cmps[0] = 25;
            ano_opmv_lt_ctrl.exp_velocity_h_cmps[1] = 0;
            ano_opmv_lt_ctrl.exp_yaw_pal_dps = YAW_PAL_DPS;
            if(elapsed_time_ms < 90000 / YAW_PAL_DPS)
            {
                elapsed_time_ms += *dT_ms;
            }
            else
            {
                ano_opmv_lt_ctrl.exp_yaw_pal_dps = 0;
                elapsed_time_ms = 0;
                step_pro_sta = 20;
            }
        }
        break;

        case 20:
        {
            elapsed_time_ms += *dT_ms;
            if(elapsed_time_ms < 100)
            {
                elapsed_time_ms += *dT_ms;
            }
            else if(elapsed_time_ms < 600)
            {
                elapsed_time_ms += *dT_ms;
                ano_opmv_lt_ctrl.exp_velocity_h_cmps[0] = FORWARD_VEL;
                ano_opmv_lt_ctrl.exp_velocity_h_cmps[1] = 0;
            }
            else
            {
                elapsed_time_ms = 0;
                step_pro_sta = 1;
            }
        }
        break;

        default:
        {
        }
        break;
    }
}

/*
 * 功能：计算寻线控制输出。
 * 说明：保留原有横向 PD 与航向修正逻辑。
 */
void ANO_LTracking_Ctrl(u8 *dT_ms,u8 en)
{
    if(en)
    {
        ano_opmv_lt_ctrl.exp_velocity_h_cmps[1]\
        = LT_KP * ano_opmv_lt_ctrl.ground_pos_err_h_cm\
        + LT_KD * ano_opmv_lt_ctrl.ground_pos_err_d_h_cmps;

        if(opmv.lt.sta == 1)
        {
            ano_opmv_lt_ctrl.exp_yaw_pal_dps = -opmv.lt.angle * 0.5f;
            ano_opmv_lt_ctrl.exp_yaw_pal_dps = LIMIT(ano_opmv_lt_ctrl.exp_yaw_pal_dps, -30, 30);
        }
        else
        {
            ano_opmv_lt_ctrl.exp_yaw_pal_dps = 0;
        }
        ANO_LT_StepProcedure(dT_ms);
    }
    else
    {
        ano_opmv_lt_ctrl.exp_velocity_h_cmps[0] = 0;
        ano_opmv_lt_ctrl.exp_velocity_h_cmps[1] = 0;
        ano_opmv_lt_ctrl.exp_yaw_pal_dps = 0;
        step_pro_sta = 0;
    }
}
