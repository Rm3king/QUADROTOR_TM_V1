#include "MotorCtrl.h"
#include "Math.h"
#include "Parameter.h"
#include "Drv_icm20602.h"
#include "Drv_spl06.h"
#include "Imu.h"
#include "Drv_PwmOut.h"

#include "MotionCal.h"
#include "Filter.h"
#include "Navigate.h"
#include "RC.h"

/*
 * 电机控制模块
 *
 * 执行解锁预转、混控合成和 PWM 输出前限幅。
 *
 * 混控布局（X 型四旋翼，机头朝前）：
 *      机头
 *   m2     m1
 *     \   /
 *      \ /
 *      / \
 *     /   \
 *   m3     m4
 *      机尾
 */

s16 motor[MOTORSNUM];
s16 motor_step[MOTORSNUM];

static u16 motor_prep_cnt;
motor_ctrl_t mc;
u16 motor_idle_pwm;

#define MOTOR_PWM_MAX   1000
#define MOTOR_PREP_TIME 300     /* 每个电机预转持续时间 (ms) */

void Motor_Ctrl_Task(u8 dT_ms)
{
	u8 i;

	if (flag.unlock_sta)
	{
		motor_idle_pwm = 10 * LIMIT(g_fc_param.set.idle_speed_pwm, 0, 30);

		if (flag.motor_preparation == 0)
		{
			motor_prep_cnt += dT_ms;

			if (motor_prep_cnt < MOTOR_PREP_TIME)
			{
				motor[m1] = motor_idle_pwm;
			}
			else if (motor_prep_cnt < MOTOR_PREP_TIME * 2)
			{
				motor[m2] = motor_idle_pwm;
			}
			else if (motor_prep_cnt < MOTOR_PREP_TIME * 3)
			{
				motor[m3] = motor_idle_pwm;
			}
			else if (motor_prep_cnt < MOTOR_PREP_TIME * 4)
			{
				motor[m4] = motor_idle_pwm;
			}
			else
			{
				flag.motor_preparation = 1;
				motor_prep_cnt = 0;
			}
		}
	}
	else
	{
		flag.motor_preparation = 0;
	}

	/* X 型四旋翼混控矩阵：
	 *   m1(右前) = +thr +yaw -roll +pitch
	 *   m2(左前) = +thr -yaw +roll +pitch
	 *   m3(左后) = +thr +yaw +roll -pitch
	 *   m4(右后) = +thr -yaw -roll -pitch
	 */
	if (flag.motor_preparation == 1)
	{
		motor_step[m1] = mc.throttle + mc.yaw - mc.roll + mc.pitch;
		motor_step[m2] = mc.throttle - mc.yaw + mc.roll + mc.pitch;
		motor_step[m3] = mc.throttle + mc.yaw + mc.roll - mc.pitch;
		motor_step[m4] = mc.throttle - mc.yaw - mc.roll - mc.pitch;

		for (i = 0; i < MOTORSNUM; i++)
		{
			motor_step[i] = LIMIT(motor_step[i], motor_idle_pwm, MOTOR_PWM_MAX);
		}
	}

	for (i = 0; i < MOTORSNUM; i++)
	{
		if (flag.unlock_sta && flag.motor_preparation == 1)
		{
			motor[i] = LIMIT(motor_step[i], motor_idle_pwm, MOTOR_PWM_MAX - 1);
		}
		else
		{
			motor[i] = 0;
		}
	}

	for (u8 i = 0; i < 4; i++)
	{
		Drv_MotorPWMSet(i, motor[i]);
	}

#ifdef Cali_Set_ESC
	for (u8 i = 0; i < 4; i++)
	{
		motor[i] = CH_N[CH_THR] + 500;
		Drv_MotorPWMSet(i, motor[i]);
	}
#endif
}
