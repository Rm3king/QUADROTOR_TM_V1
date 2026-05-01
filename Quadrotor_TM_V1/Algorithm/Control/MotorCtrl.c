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
 * 文件名称: MotorCtrl.c
 * 所属模块: Algorithm / Control
 *
 * 功能描述:
 *   电机控制模块，完成控制链的最后一步：
 *   1) 解锁后逐电机预转（避免同时启动的电流冲击）
 *   2) X 型四旋翼混控：将 roll/pitch/yaw/throttle 合成为 4 路 PWM
 *   3) PWM 限幅并输出到硬件定时器
 *
 * 混控矩阵（X 型四旋翼，机头朝前）:
 *         机头
 *      m2     m1          m1 = +thr +yaw -roll +pitch
 *        \   /            m2 = +thr -yaw +roll +pitch
 *         \ /             m3 = +thr +yaw +roll -pitch
 *         / \             m4 = +thr -yaw -roll -pitch
 *        /   \
 *      m3     m4
 *         机尾
 *
 * 数据流:
 *   AttCtrl → mc.roll/pitch/yaw
 *   AltCtrl → mc.throttle
 *   MotorCtrl → motor[0..3] → Drv_MotorPWMSet() → 硬件 PWM
 *
 * 架构位置:
 *   由 Scheduler 按 2ms 周期调用，是控制链的输出端。
 *
 * 教学提示:
 *   - 预转序列（m1→m2→m3→m4）让用户可以目视确认电机接线顺序
 *   - motor_idle_pwm 是怠速转速，防止低油门时电机停转
 *   - MOTOR_PWM_MAX 为 PWM 上限，实际输出限制为 MAX-1 保留余量
 */

s16 motor[MOTORSNUM];
s16 motor_step[MOTORSNUM];

static u16 motor_prep_cnt;
_motor_ctrl_st mc;
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
		motor[i] = RC_GetChannel(CH_THR) + 500;
		Drv_MotorPWMSet(i, motor[i]);
	}
#endif
}
