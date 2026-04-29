#ifndef __OF_DECOFUSION_H__
#define __OF_DECOFUSION_H__
#include "sysconfig.h"
#include "FcData.h"

/* 光流原始数据。 */
typedef struct
{
    u8 updata;
    u8 online;
    s16 flow_x_integral;
    s16 flow_y_integral;
    u16 it_ms;
    u8 valid;
} _of_data_st;

/* 光流解耦与融合状态。 */
typedef struct
{
    /* 状态机：0 初始化，1 正常，2 保留。 */
    u8 state;
    u8 quality;
    u8 valid;

    /* 参考高度与光流观测。 */
    float of_ref_height;
    float of_vel[2];
    float gnd_vel_obs_h[2];
    float gnd_vel_obs_w[2];

    /* 惯性估计与融合输出。 */
    float gnd_acc_est_w[2];
    float gnd_vel_est_w[2];
    float gnd_vel_est_h[2];
    float gnd_vel_fixout_w[2];
    float gnd_vel_fixout_h[2];
} _of_rdf_st;

extern float of_rot_d_degs[];
extern _of_data_st of_data;
extern _of_rdf_st of_rdf;

static void ANO_OF_Data_Get(float *dT_s, u8 *of_data_buf);
static void OF_INS_Get(float *dT_s, float rad_ps_x, float rad_ps_y, float acc_wx, float acc_wy);
static void ANO_OF_Decouple(u8 *dT_ms);
static void ANO_OF_Fusion(u8 *dT_ms, s32 ref_height_cm);
static void OF_State(void);
static void OF_INS_Reset(void);

void ANO_OF_Data_Prepare_Task(float dT_s);
void ANO_OFDF_Task(u8 dT_ms);

#endif
