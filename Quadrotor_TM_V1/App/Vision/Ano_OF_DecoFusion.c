#include "Ano_OF_DecoFusion.h"
#include "Ano_IMU.h"
#include "Ano_Math.h"
#include "Ano_Filter.h"
#include "Drv_UP_Flow.h"
#include "Ano_Sensor_Basic.h"
#include "Drv_laser.h"

/*
 * 模块说明。
 * 优像光流解耦与融合实现。
 * 负责解析光流原始数据、执行姿态旋转补偿，并将光流观测与惯性估计进行互补融合。
 */
static void ANO_OF_Data_Get(float *dT_s, u8 *of_data_buf);
static void OF_INS_Get(float *dT_s, float rad_ps_x, float rad_ps_y, float acc_wx, float acc_wy);
static void ANO_OF_Decouple(u8 *dT_ms);
static void ANO_OF_Fusion(u8 *dT_ms, s32 ref_height_cm);
static void OF_State(void);
static void OF_INS_Reset(void);

#define LASER_ONLINE           (LASER_LINKOK)
#define BUF_UPDATE_CNT         (of_buf_update_cnt)
#define OF_DATA_BUF            (OF_DATA)
#define RADPS_X                (sensor.Gyro_rad[0])
#define RADPS_Y                (sensor.Gyro_rad[1])
#define RELATIVE_HEIGHT_CM     (Laser_height_cm)

u8 of_buf_update_flag;
_of_data_st of_data;
_of_rdf_st of_rdf;
float of_rot_d_degs[2];
float of_fus_err[2],of_fus_err_i[2];

#define UPOF_PIXELPDEG_X       160.0f
#define UPOF_PIXELPDEG_Y       160.0f
#define UPOF_CMPPIXEL_X        0.00012f
#define UPOF_CMPPIXEL_Y        0.00012f
#define FUS_KP                 2.0f
#define FUS_KI                 1.0f
#define UPOF_UP_DW             0
#define OBJREF_HEIGHT_CM       280

/* 功能：准备光流原始数据与惯性估计量。 */
void ANO_OF_Data_Prepare_Task(float dT_s)
{
    ANO_OF_Data_Get(&dT_s, OF_DATA_BUF);
    OF_INS_Get(&dT_s, RADPS_X, RADPS_Y, imu_data.w_acc[0], imu_data.w_acc[1]);
}

/* 功能：执行光流解耦与融合任务。 */
void ANO_OFDF_Task(u8 dT_ms)
{
    OF_State();
    ANO_OF_Decouple(&dT_ms);
    ANO_OF_Fusion(&dT_ms, (s32)RELATIVE_HEIGHT_CM);
}

/* 功能：更新惯性侧估计量。 */
static void OF_INS_Get(float *dT_s,float rad_ps_x,float rad_ps_y,float acc_wx,float acc_wy)
{
    static float rad_ps_lpf[2];

    /* 低通滤波后再参与光流旋转补偿，便于相位对齐。 */
    LPF_1_(5.0f,*dT_s,rad_ps_x,rad_ps_lpf[0]);
    LPF_1_(5.0f,*dT_s,rad_ps_y,rad_ps_lpf[1]);
    of_rot_d_degs[0] = rad_ps_lpf[0] * DEG_PER_RAD;
    of_rot_d_degs[1] = rad_ps_lpf[1] * DEG_PER_RAD;

    LPF_1_(5.0f,*dT_s,acc_wx,of_rdf.gnd_acc_est_w[X]);
    LPF_1_(5.0f,*dT_s,acc_wy,of_rdf.gnd_acc_est_w[Y]);
    for(u8 i = 0;i < 2;i++)
    {
        of_rdf.gnd_vel_est_w[i] += of_rdf.gnd_acc_est_w[i] * (*dT_s);
    }
}

/* 功能：读取并解析光流原始缓冲区。 */
static void ANO_OF_Data_Get(float *dT_s,u8 *of_data_buf)
{
    static float offline_delay_time_s;
    u8 ADD = 0;

    if(of_buf_update_flag != BUF_UPDATE_CNT)
    {
        of_buf_update_flag = BUF_UPDATE_CNT;
        ADD = of_data_buf[0];
        for(u8 i = 1;i < 18;i++)
        {
            ADD += of_data_buf[i];
        }
        if(ADD == of_data_buf[18])
        {
            of_data.updata ++;
            of_data.valid = of_data_buf[16];
            if(of_data.valid != 0xf5)
            {
                of_data.flow_x_integral = 0;
                of_data.flow_y_integral = 0;
            }
            else
            {
                /* 原始协议中 X/Y 轴定义与本地坐标存在交叉，对齐方式保持不变。 */
                of_data.flow_x_integral = (s16)(of_data_buf[12] | (of_data_buf[13] << 8));
                of_data.flow_y_integral = (s16)(of_data_buf[10] | (of_data_buf[11] << 8));
            }
            of_data.it_ms = ((u16)(of_data_buf[14] | (of_data_buf[15] << 8))) / 1000;
            RELATIVE_HEIGHT_CM = (u16)(of_data_buf[6] | (of_data_buf[7] << 8)) / 10;
        }
        offline_delay_time_s = 0;
        of_data.online = 1;
        LASER_ONLINE = 1;
        sens_hd_check.tof_ok = LASER_ONLINE;
    }
    else
    {
        if(offline_delay_time_s < 1.0f)
        {
            offline_delay_time_s += *dT_s;
        }
        else
        {
            of_data.online = 0;
            LASER_ONLINE = 0;
            sens_hd_check.tof_ok = LASER_ONLINE;
        }
    }
}

/* 功能：执行光流去旋转补偿。 */
static void ANO_OF_Decouple(u8 *dT_ms)
{
    if(of_data.valid != 0xf5)
    {
        of_rdf.of_vel[X] = 0;
        of_rdf.of_vel[Y] = 0;
        if(of_rdf.quality >= 5)
        {
            of_rdf.quality -= 5;
        }
    }
    else
    {
        if(UPOF_UP_DW == 0)
        {
            of_rdf.of_vel[X] = (1000 / of_data.it_ms * of_data.flow_x_integral + UPOF_PIXELPDEG_X * of_rot_d_degs[Y]);
            of_rdf.of_vel[Y] = (1000 / of_data.it_ms * of_data.flow_y_integral - UPOF_PIXELPDEG_Y * of_rot_d_degs[X]);
        }
        else
        {
            of_rdf.of_vel[X] = -(1000 / of_data.it_ms * of_data.flow_x_integral + UPOF_PIXELPDEG_X * of_rot_d_degs[Y]);
            of_rdf.of_vel[Y] =  (1000 / of_data.it_ms * of_data.flow_y_integral + UPOF_PIXELPDEG_Y * of_rot_d_degs[X]);
        }
        if(of_rdf.quality <= 250)
        {
            of_rdf.quality += 5;
        }
    }
    (void)dT_ms;
}

/* 功能：融合光流观测与惯性估计。 */
static void ANO_OF_Fusion(u8 *dT_ms,s32 ref_height_cm)
{
    static float F_KP,F_KI;
    float dT_s = (*dT_ms) * 1e-3f;

    if(UPOF_UP_DW == 0)
    {
        of_rdf.of_ref_height = LIMIT(ref_height_cm,20,500);
    }
    else
    {
        of_rdf.of_ref_height = LIMIT((OBJREF_HEIGHT_CM - ref_height_cm),20,500);
    }

    of_rdf.gnd_vel_obs_h[X] = UPOF_CMPPIXEL_X * of_rdf.of_vel[X] * of_rdf.of_ref_height;
    of_rdf.gnd_vel_obs_h[Y] = UPOF_CMPPIXEL_Y * of_rdf.of_vel[Y] * of_rdf.of_ref_height;
    h2w_2d_trans(of_rdf.gnd_vel_obs_h, imu_data.hx_vec, of_rdf.gnd_vel_obs_w);

    switch(of_rdf.state)
    {
        case 0:
        {
            of_rdf.state = 1;
            F_KP = FUS_KP;
            F_KI = FUS_KI;
            OF_INS_Reset();
        }
        break;

        case 1:
        {
            /* 采用 PI 互补融合，保留现有修正顺序与参数。 */
            if(of_data.valid == 0xf5)
            {
                for(u8 i = 0;i < 2;i++)
                {
                    of_fus_err[i] = of_rdf.gnd_vel_obs_w[i] - of_rdf.gnd_vel_est_w[i];
                    of_fus_err_i[i] += F_KI * of_fus_err[i] * dT_s;
                    of_fus_err_i[i] = LIMIT(of_fus_err_i[i], -100, 100);
                    of_rdf.gnd_vel_est_w[i] += (of_fus_err[i] * F_KP + of_fus_err_i[i]) * dT_s;
                }
            }
            w2h_2d_trans(of_rdf.gnd_vel_est_w, imu_data.hx_vec, of_rdf.gnd_vel_est_h);
        }
        break;

        default:
        {
            OF_INS_Reset();
        }
        break;
    }
}

/* 功能：复位惯性估计内部状态。 */
static void OF_INS_Reset()
{
    for(u8 i = 0;i < 2;i++)
    {
        of_rdf.gnd_vel_est_w[i] = 0;
        of_fus_err_i[i] = 0;
    }
}

/* 功能：更新光流融合状态机。 */
static void OF_State()
{
    if(imu_state.G_reset)
    {
        if(of_rdf.state == 1)
        {
            of_rdf.state = 0;
        }
    }
    else
    {
        if(of_rdf.quality < 100)
        {
            of_rdf.state = 0;
        }
        else if(of_rdf.quality > 200)
        {
            of_rdf.state = 1;
        }
    }

    if(of_data.online && LASER_ONLINE)
    {
        sens_hd_check.of_df_ok = 1;
    }
    else
    {
        sens_hd_check.of_df_ok = 0;
    }
}
