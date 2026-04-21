#include "Ano_OPMV_CBTracking_Ctrl.h"
#include "Drv_OpenMV.h"
#include "Ano_OPMV_Ctrl.h"
#include "Ano_Math.h"
#include "ANO_IMU.h"
#include "Ano_OF.h"
#include "Ano_OF_DecoFusion.h"
#include "Ano_FlightCtrl.h"
#include "Ano_MotionCal.h"
/*
 * 模块说明。
 * OpenMV 色块跟踪控制实现。
 * 负责根据视觉目标位置完成姿态解耦、地面误差估计和速度控制量计算。
 */
static void ANO_CBTracking_Decoupling(u8 *dT_ms, float rol_degs, float pit_degs);
static void ANO_CBTracking_Calcu(u8 *dT_ms, s32 relative_height_cm);

#define IMU_ROL                 (imu_data.rol)
#define IMU_PIT                 (imu_data.pit)
#define RELATIVE_HEIGHT_CM      (jsdata.valid_of_alt_cm)
#define OF_DATA_SOURCE          ((sens_hd_check.of_ok) ? 1 : 0)
#define VELOCITY_CMPS_X_S1      (OF_DX2FIX)
#define VELOCITY_CMPS_Y_S1      (OF_DY2FIX)
#define VELOCITY_CMPS_X_S2      (of_rdf.gnd_vel_est_h[0])
#define VELOCITY_CMPS_Y_S2      (of_rdf.gnd_vel_est_h[1])
#define CBT_KP                  (0.80f)
#define CBT_KD                  (0.20f)
#define CBT_KF                  (0.20f)

static u16 target_loss_hold_time;
_ano_opmv_cbt_ctrl_st ano_opmv_cbt_ctrl;
static s16 ref_carrier_velocity[2];
static float decou_pos_pixel_lpf[2][2];

/* 参数标定值。 */
#define PIXELPDEG_X    2.4f
#define PIXELPDEG_Y    2.4f
#define CMPPIXEL_X     0.01f
#define CMPPIXEL_Y     0.01f
#define TLH_TIME       1000

/*
 * 功能：执行色块跟踪数据预处理。
 * 说明：在 OpenMV 色块模式下完成姿态解耦与地面误差估计。
 */
void ANO_CBTracking_Task(u8 dT_ms)
{
    if(opmv.mode_sta == 1)
    {
        ANO_CBTracking_Decoupling(&dT_ms, IMU_ROL, IMU_PIT);
        ANO_CBTracking_Calcu(&dT_ms, (s32)RELATIVE_HEIGHT_CM);
    }
    else
    {
        ano_opmv_cbt_ctrl.target_loss = 1;
    }
}

/*
 * 功能：执行色块跟踪姿态解耦。
 * 说明：把 OpenMV 图像坐标转换为飞控坐标下的平移偏差。
 */
static void ANO_CBTracking_Decoupling(u8 *dT_ms,float rol_degs,float pit_degs)
{
    float dT_s = (*dT_ms) * 1e-3f;

    if(opmv.cb.sta != 0)
    {
        ano_opmv_cbt_ctrl.target_loss = 0;
        target_loss_hold_time = 0;
    }
    else
    {
        if(target_loss_hold_time < TLH_TIME)
        {
            target_loss_hold_time += *dT_ms;
        }
        else
        {
            ano_opmv_cbt_ctrl.target_loss = 1;
        }
    }

    ano_opmv_cbt_ctrl.opmv_pos[0] = opmv.cb.pos_y;
    ano_opmv_cbt_ctrl.opmv_pos[1] = -opmv.cb.pos_x;

    if(opmv.cb.sta != 0)
    {
        ano_opmv_cbt_ctrl.rp2pixel_val[0] = -PIXELPDEG_X * pit_degs;
        ano_opmv_cbt_ctrl.rp2pixel_val[1] = -PIXELPDEG_Y * rol_degs;
        ano_opmv_cbt_ctrl.rp2pixel_val[0] = LIMIT(ano_opmv_cbt_ctrl.rp2pixel_val[0], -60, 60);
        ano_opmv_cbt_ctrl.rp2pixel_val[1] = LIMIT(ano_opmv_cbt_ctrl.rp2pixel_val[1], -80, 80);
        if(OF_DATA_SOURCE == 1)
        {
            ref_carrier_velocity[0] = VELOCITY_CMPS_X_S1;
            ref_carrier_velocity[1] = VELOCITY_CMPS_Y_S1;
        }
        else
        {
            ref_carrier_velocity[0] = VELOCITY_CMPS_X_S2;
            ref_carrier_velocity[1] = VELOCITY_CMPS_Y_S2;
        }
    }
    else
    {
        ref_carrier_velocity[0] = 0;
        ref_carrier_velocity[1] = 0;
    }

    if(ano_opmv_cbt_ctrl.target_loss == 0)
    {
        decou_pos_pixel_lpf[0][0] += 0.2f * ((ano_opmv_cbt_ctrl.opmv_pos[0] - ano_opmv_cbt_ctrl.rp2pixel_val[0]) - decou_pos_pixel_lpf[0][0]);
        decou_pos_pixel_lpf[0][1] += 0.2f * ((ano_opmv_cbt_ctrl.opmv_pos[1] - ano_opmv_cbt_ctrl.rp2pixel_val[1]) - decou_pos_pixel_lpf[0][1]);
        decou_pos_pixel_lpf[1][0] += 0.2f * (decou_pos_pixel_lpf[0][0] - decou_pos_pixel_lpf[1][0]);
        decou_pos_pixel_lpf[1][1] += 0.2f * (decou_pos_pixel_lpf[0][1] - decou_pos_pixel_lpf[1][1]);
        ano_opmv_cbt_ctrl.decou_pos_pixel[0] = decou_pos_pixel_lpf[1][0];
        ano_opmv_cbt_ctrl.decou_pos_pixel[1] = decou_pos_pixel_lpf[1][1];
    }
    else
    {
        LPF_1_(0.2f, dT_s, 0, ano_opmv_cbt_ctrl.decou_pos_pixel[0]);
        LPF_1_(0.2f, dT_s, 0, ano_opmv_cbt_ctrl.decou_pos_pixel[1]);
    }
}

/* 功能：根据解耦结果计算地面误差与目标速度。 */
static void ANO_CBTracking_Calcu(u8 *dT_ms,s32 relative_height_cm)
{
    static float relative_height_cm_valid;
    static float g_pos_err_old[2];
    float gped_tmp[2];

    if(relative_height_cm < 500)
    {
        relative_height_cm_valid = relative_height_cm;
    }

    g_pos_err_old[0] = ano_opmv_cbt_ctrl.ground_pos_err_h_cm[0];
    g_pos_err_old[1] = ano_opmv_cbt_ctrl.ground_pos_err_h_cm[1];
    ano_opmv_cbt_ctrl.ground_pos_err_h_cm[0] = CMPPIXEL_X * relative_height_cm_valid * ano_opmv_cbt_ctrl.decou_pos_pixel[0];
    ano_opmv_cbt_ctrl.ground_pos_err_h_cm[1] = CMPPIXEL_Y * relative_height_cm_valid * ano_opmv_cbt_ctrl.decou_pos_pixel[1];

    gped_tmp[0] = (ano_opmv_cbt_ctrl.ground_pos_err_h_cm[0] - g_pos_err_old[0]) * (1000 / (*dT_ms));
    gped_tmp[1] = (ano_opmv_cbt_ctrl.ground_pos_err_h_cm[1] - g_pos_err_old[1]) * (1000 / (*dT_ms));
    ano_opmv_cbt_ctrl.ground_pos_err_d_h_cmps[0] += 0.2f * (gped_tmp[0] - ano_opmv_cbt_ctrl.ground_pos_err_d_h_cmps[0]);
    ano_opmv_cbt_ctrl.ground_pos_err_d_h_cmps[1] += 0.2f * (gped_tmp[1] - ano_opmv_cbt_ctrl.ground_pos_err_d_h_cmps[1]);
    ano_opmv_cbt_ctrl.target_gnd_velocity_cmps[0] = ano_opmv_cbt_ctrl.ground_pos_err_d_h_cmps[0] + ref_carrier_velocity[0];
    ano_opmv_cbt_ctrl.target_gnd_velocity_cmps[1] = ano_opmv_cbt_ctrl.ground_pos_err_d_h_cmps[1] + ref_carrier_velocity[1];
}

/*
 * 功能：计算色块跟踪控制输出。
 * 说明：保持原有 PD 加速度前馈形式不变。
 */
void ANO_CBTracking_Ctrl(u8 *dT_ms,u8 en)
{
    (void)dT_ms;
    if(en)
    {
        ano_opmv_cbt_ctrl.exp_velocity_h_cmps[0]\
        = CBT_KF * ano_opmv_cbt_ctrl.target_gnd_velocity_cmps[0]\
        + CBT_KP * (ano_opmv_cbt_ctrl.ground_pos_err_h_cm[0])\
        + CBT_KD * ano_opmv_cbt_ctrl.ground_pos_err_d_h_cmps[0];

        ano_opmv_cbt_ctrl.exp_velocity_h_cmps[1]\
        = CBT_KF * ano_opmv_cbt_ctrl.target_gnd_velocity_cmps[1]\
        + CBT_KP * (ano_opmv_cbt_ctrl.ground_pos_err_h_cm[1])\
        + CBT_KD * ano_opmv_cbt_ctrl.ground_pos_err_d_h_cmps[1];
    }
    else
    {
        ano_opmv_cbt_ctrl.exp_velocity_h_cmps[0] = 0;
        ano_opmv_cbt_ctrl.exp_velocity_h_cmps[1] = 0;
    }
}
