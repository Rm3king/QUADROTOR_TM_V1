/*
 * 文件名称: Power.c
 * 所属模块: Algorithm / Control
 *
 * 功能描述:
 *   电源管理模块，通过 ADC 采样电池电压并提供低电压保护：
 *   1) Power_Check_Task() -- 周期读取 ADC → 低通滤波 → 电压估计 → 低压告警
 *
 * 低压保护:
 *   电压低于设定阈值时设置 LED_STA.lowVt 标志，LED 红灯快闪告警。
 *   可配合自动降落使用（需在 FlightCtrl 中启用）。
 *
 * 架构位置:
 *   由 Scheduler 按 50ms 周期调用。输出 Plane_Votage 供 DT.c 遥测。
 *
 * 教学提示:
 *   - ADC 采样值需要经过分压比换算才是真实电池电压
 *   - 低通滤波消除 ADC 采样噪声，避免电压抖动触发误报
 *   - 锂电池过放会永久损坏，低压保护是飞控必备功能
 */
#include "Power.h"
#include "Parameter.h"
#include "Filter.h"
#include "Drv_led.h"
#include "Math.h"
#include "LED.h"

float Plane_Votage = 0;
static float s_voltage_lpf_mv = 30000;
static u8 s_voltage_ready;
void Power_UpdateTask(u8 dT_ms)
{
	static s16 s_voltage_sample_mv;
	float cutoff_hz;
	//触发ADC采样
	Drv_Adc0Trigger();
	//赋值电压数据
	s_voltage_sample_mv = Voltage * 1000;
	
	if(s_voltage_ready == 0)
	{
		cutoff_hz = 2.0f;
		
		if(s_voltage_lpf_mv > 2000 && ABS(s_voltage_sample_mv - s_voltage_lpf_mv) < 200)
		{
			s_voltage_ready = 1;
		}
	}	
	else
	{
		cutoff_hz = 0.02f;
	}
	
	LPF_1_(cutoff_hz, dT_ms * 1e-3f, s_voltage_sample_mv, s_voltage_lpf_mv);
	

	
	Plane_Votage = s_voltage_lpf_mv * 0.001f;


		
	if(Plane_Votage<g_fc_param.set.lowest_power_voltage)
	{
		flag.power_state = 3;//将禁止解锁		
	}
	else
	{
		flag.power_state = 1;
	}

	if(Plane_Votage<g_fc_param.set.warn_power_voltage)
	{
		LED_STA.lowVt = 1;
	}
	else if(Plane_Votage>g_fc_param.set.warn_power_voltage+0.2f)
	{
		LED_STA.lowVt = 0;
	}
		
	if(Plane_Votage<g_fc_param.set.return_home_power_voltage)
	{
		
	
	}
}





