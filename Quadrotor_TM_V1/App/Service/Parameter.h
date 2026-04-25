#ifndef _PARAMETER_H
#define	_PARAMETER_H

#include "config.h"
#include "FcData.h"

/*
 * 模块名称：Parameter
 * 模块职责：定义飞控参数存储结构、参数保存状态和参数读写接口。
 * 使用约束：本头文件只描述参数布局与接口，不改变参数含义与存储时序。
 */
/* 持久化参数结构，字段顺序直接决定存储布局。 */
__packed struct Parameter_s
{
	/* 版本与初始化标记。 */
	u16 frist_init;

	/* 输入与外设开关配置。 */
	u8 pwmInMode;
	u8 heatSwitch;

	/* 传感器校准参数。 */
	float acc_offset[VEC_XYZ];
	float gyro_offset[VEC_XYZ];
	float surface_vec[VEC_XYZ];
	float center_pos_cm[VEC_XYZ];
	float mag_offset[VEC_XYZ];
	float mag_gain[VEC_XYZ];

	/* 姿态与高度控制 PID 参数。 */
	float pid_att_1level[VEC_RPY][PID];
	float pid_att_2level[VEC_RPY][PID];
	float pid_alt_1level[PID];
	float pid_alt_2level[PID];

	/* 水平位置控制 PID 参数。 */
	float pid_loc_1level[PID];
	float pid_loc_2level[PID];
	float pid_gps_loc_1level[PID];
	float pid_gps_loc_2level[PID];

	/* 电压保护参数。 */
	float warn_power_voltage;
	float return_home_power_voltage;
	float lowest_power_voltage;

	/* 自动起降与电机准备参数。 */
	float auto_take_off_height;
	float auto_take_off_speed;
	float auto_landing_speed;
	float idle_speed_pwm;
};

/* 参数存储联合体，提供结构化访问和字节级访问。 */
union Parameter
{
	/* 联合体同时提供结构化访问和字节级访问。 */
	struct Parameter_s set;
	u8 byte[2048];
};
extern union Parameter g_fc_param;

/* 参数保存运行状态。 */
typedef struct
{
	u8 save_en;
	u8 save_trig;
	u16 time_delay;
} param_state_t;
extern param_state_t g_param_state;

/* 读取参数区，并在必要时执行默认初始化。 */
void FC_Param_Read(void);
/* 周期性参数保存任务。 */
void FC_Param_WriteTask(u16 dT_ms);
/* 恢复默认 PID 参数。 */
void FC_Param_ResetPid(void);
/* 恢复默认飞控参数。 */
void FC_Param_Reset(void);

#endif 

