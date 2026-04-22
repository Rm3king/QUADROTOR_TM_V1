#ifndef __FLIGHT_CTRL_H
#define __FLIGHT_CTRL_H
#include "Ano_FcData.h"
#include "Ano_Filter.h"
#include "Ano_Math.h"

/* 飞行动作脚本编号 */
enum
{
    null = 0,
    takeoff,
    landing,
    s_up_down_2,
    s_yaw_pn_2,
    b_yaw_pn_1,
    s_rol_pn_2,
    b_rol_pn_1,
    s_pit_pn_2,
    b_pit_pn_1,
    yaw_n360,
    yaw_p360,
    roll_1,
    pit_jump_pn_2,
    rol_jump_pn_2,
    rol_up_down_2,
    yaw_up_dowm_1,
    pit_rol_pn_2,
};

/* 飞行状态控制量 */
typedef struct
{
    s16 alt_ctrl_speed_set;
    float speed_set_h[VEC_XYZ];
    float speed_set_h_cms[VEC_XYZ];
    float speed_set_h_norm[VEC_XYZ];
    float speed_set_h_norm_lpf[VEC_XYZ];
} _flight_state_st;
extern _flight_state_st fs;

/* 光流与高度同步判定输入 */
typedef struct
{
    u8 of_qua;
    u16 of_alt;
    u16 valid_of_alt_cm;
} _judge_sync_data_st;
extern _judge_sync_data_st jsdata;

extern float wifi_selfie_mode_yaw_vlue;

void user_fun(float dT, u8 action_num);
void All_PID_Init(void);
void one_key_take_off(void);
void one_key_land(void);
void Sudden_Stop_Task(void);
void one_key_roll(void);
void app_one_key_roll(void);
void app_one_key_roll_reset(void);
void one_key_take_off_task(u16 dt_ms);
void ctrl_parameter_change_task(void);
void Flight_State_Task(u8 dT_ms, s16 *CH_N);
void Flight_Mode_Set(u8 dT_ms);
void Swtich_State_Task(u8 dT_ms);

#endif
