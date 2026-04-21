#ifndef __ANO_FILTER_H
#define __ANO_FILTER_H
#include "Ano_FcData.h"

/* ?????????? */
typedef struct
{
    float out;
    float last_out;
    float a;
    float b;
    float a_c2;
    float b_c2;
    float c_c2;
    float k;
} _ano_filter_1_st;

/* ???????? */
typedef struct
{
    float in_est;
    float in_obs;
    float fix_ki;
    float ei_limit;
    float e;
    float ei;
    float out;
} _inte_fix_filter_st;

/* ?????????? */
typedef struct
{
    float in_est_d;
    float in_obs;
    float fix_kp;
    float e_limit;
    float e;
    float out;
} _fix_inte_filter_st;

/* ?????? */
typedef struct
{
    float lpf_1;
    float out;
} _lf_t;

/* ???????? */
typedef struct
{
    float lpf_1;
    float lpf_2;
    float in_old;
    float out;
} _jldf_t;

/* ???????? */
typedef struct
{
    float a;
    float b;
    s16 limit;
    float e_nr;
    float err_v;
    float out;
    float ei;
    float out_f;
} _filter_1_st;

/* ?????????? */
typedef struct
{
    u8 cnt;
    s32 lst_pow_sum;
    s32 now_out;
    s32 lst_out;
    s32 now_velocity_xdt;
} _steepest_st;

typedef struct Filter
{
    float _cutoff_freq;
    float _a1;
    float _a2;
    float _b0;
    float _b1;
    float _b2;
    float _delay_element_1;
    float _delay_element_2;
} filter_s;

void inte_fix_filter(float dT, _inte_fix_filter_st *data);
void fix_inte_filter(float dT, _fix_inte_filter_st *data);
void filter_1(float a, float b, float e_nr, float in, _filter_1_st *data);
void limit_filter(float T, float hz, _lf_t *data, float in);
void limit_filter_2(float T, float hz, _lf_t *data, float in);
void limit_filter_3(float T, float hz, _lf_t *data, float in);
void init_low_pass2_fliter(float sample_freq, float cutoff_freq, filter_s *imu_filter);
float low_pass2_filter(float sample, filter_s *imu_filter);
void steepest_descend(s32 arr[], u8 len, _steepest_st *steepest, u8 step_num, s32 in);
void jyoun_limit_deadzone_filter(float T, float hz1, float hz2, _jldf_t *data, float in);
void jyoun_filter(float dT, float hz, float ref_value, float exp, float fb, float *out);
void Moving_Average(float moavarray[], u16 len, u16 *fil_cnt, float in, float *out);
void step_filter(float step, float in, float *out);
void fir_arrange_filter(float *arr, u16 len, u8 *fil_cnt, float in, float *arr_out);

#define LPF_1_(hz,t,in,out) ((out) += ( 1 / ( 1 + 1 / ( (hz) *6.28f *(t) ) ) ) *( (in) - (out) ))
#define S_LPF_1(a,in,out)   ((out) += (a) *( (in) - (out) ))

void LPF_1_db(float hz, float time, double in, double *out);
void LPF_I(float raw_a, float raw_b, float time, float in, float *out, float *intera);
float my_deadzone_3(float T, float hz, float x, float ref, float zoom, float range_x, float *zoom_adj);
void vec_3dh_transition(float ref[VEC_XYZ], float in[VEC_XYZ], float out[VEC_XYZ]);
void vec_3dh_transition_matrix(float ref[VEC_XYZ], float wh_matrix[VEC_XYZ][VEC_XYZ]);

#endif
