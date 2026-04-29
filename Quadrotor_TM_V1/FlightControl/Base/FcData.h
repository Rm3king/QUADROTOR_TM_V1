#ifndef __FCDATA_H__
#define __FCDATA_H__

#include "config.h"

/*
 * 模块名称：FcData
 * 模块职责：集中声明飞控全局状态、参数镜像、传感器索引和公共状态结构。
 * 使用约束：本头文件定义的是跨模块共享状态，重构时只允许补充说明，不允许随意改变字段语义。
 */

#define TRUE 1
#define FALSE 0 

/* 自动起飞与降落状态。 */
enum
{
	AUTO_TAKE_OFF_NULL = 0,
	AUTO_TAKE_OFF = 1,
	AUTO_TAKE_OFF_FINISH,
	AUTO_LAND,
};

/* 遥控输入模式。 */
enum pwminmode_e
{
	PWM = 0,
	PPM,
	SBUS,
};

/* IMU 原始数据索引。 */
enum
{
 A_X = 0,
 A_Y ,
 A_Z ,
 G_X ,
 G_Y ,
 G_Z ,
 TEM ,
 MPU_ITEMS ,
};

	
/* 遥控通道索引。 */
enum
{
 CH_ROL = 0,
 CH_PIT ,
 CH_THR ,
 CH_YAW ,
 AUX1 ,
 AUX2 ,
 AUX3 ,
 AUX4 ,
 CH_NUM,
};

/* 电机索引。 */
enum
{
	m1=0,
	m2,
	m3,
	m4,
	m5,
	m6,
	m7,
	m8,

};

/* IMU 器件实例索引。 */
enum
{
	MPU_6050_0 = 0,
	MPU_6050_1,
	
};

/* 三轴向量索引。 */
enum
{
	X = 0,
	Y = 1,
	Z = 2,
	VEC_XYZ,
};

/* 姿态轴索引。 */
enum
{
	ROL = 0,
	PIT = 1,
	YAW = 2,
	VEC_RPY,
};

/* PID 参数索引。 */
enum
{
	KP = 0,
	KI = 1,
	KD = 2,
	PID,
};

/* 电压报警等级。 */
enum _power_alarm
{

	HIGH_POWER = 0,
	HALF_POWER,
	LOW_POWER ,
	LOWEST_POWER, 
	

};


/* 飞行模式。 */
enum _flight_mode
{
	ATT_STAB = 0,
	LOC_HOLD,
	RETURN_HOME,
	SUDDEN_STOP,
	
};

/* 油门模式。 */
enum
{
  THR_MANUAL = 0,
	THR_AUTO,
	
};

/* 运行时校准参数镜像。 */
typedef struct
{
	u8 first_f;
	float acc_offset[VEC_XYZ];
	float gyro_offset[VEC_XYZ];
	
	float surface_vec[VEC_XYZ];
	
	float mag_offset[VEC_XYZ];
	float mag_gain[VEC_XYZ];

} _save_st ;
extern _save_st save;

/* 飞控运行时标志位，覆盖传感器、控制与飞行状态。 */
typedef struct
{
	/* 基本状态与传感器状态。 */
	u8 start_ok;
	u8 sensor_imu_ok;
	u8 mems_temperature_ok;
	
	u8 motionless;
	u8 power_state;
	u8 wifi_ch_en;
	u8 chn_failsafe;
	u8 rc_loss;	
	u8 rc_loss_back_home;
	u8 gps_ok;	

	
	/* 控制相关状态。 */
	u8 manual_locked;
	u8 unlock_err;
	u8 unlock_cmd;
	u8 unlock_sta;
	u8 thr_low;
	u8 locking;
	u8 taking_off;
	u8 set_yaw;
	u8 ct_loc_hold;
	u8 ct_alt_hold;

	
	/* 飞行过程状态。 */
	u8 flying;
	u8 auto_take_off_land;
	u8 home_location_ok;	
	u8 speed_mode;
	u8 thr_mode;	
	u8 flight_mode;
	u8 flight_mode2;
	u8 gps_mode_en;
	u8 motor_preparation;
	u8 locked_rotor;
	
	
}_flag;
extern _flag flag;

/* 飞控运行时限幅状态变量。 */
typedef struct
{
	float vel_limit_xy;
	float vel_limit_z_p;
	float vel_limit_z_n;
	float yaw_pal_limit;
}_fc_sta_var_st;
extern _fc_sta_var_st fc_stv;

/* 外设开关状态。 */
typedef struct
{
	u8 sonar_on;
	u8 tof_on;
	u8 of_flow_on;
	u8 of_tof_on;
	u8 baro_on;
	u8 gps_on;
	u8 uwb_on;
	u8 opmv_on;
	
}_switch_st;
extern _switch_st switchs;

/* 传感器硬件自检结果。 */
typedef struct
{
	u8 gyro_ok;
	u8 acc_ok;
	u8 mag_ok;
	u8 baro_ok;
	u8 gps_ok;
	u8 sonar_ok;
	u8 tof_ok;
	u8 of_ok;
	u8 of_df_ok;
	
} _sensor_hd_check_st;
extern _sensor_hd_check_st sens_hd_check;

/* 共享状态实例说明：
 * save            参数区与运行时共享的校准镜像
 * flag            飞控核心状态标志
 * fc_stv          控制限幅和约束量
 * switchs         外设使能开关
 * sens_hd_check   硬件自检结果
 */

/* 触发参数延时保存。 */
void data_save(void);
/* 初始化参数读取流程。 */
void Para_Data_Init(void);

/* 第二阶段语义接口：先通过兼容别名暴露更清晰的入口命名。 */
#define FC_Param_RequestSave data_save
#define FC_Param_InitData    Para_Data_Init


#endif
