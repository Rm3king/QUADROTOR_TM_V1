/*
 * 模块名称：Power
 * 模块职责：更新电池电压估计，并据此刷新低压与返航相关状态。
 * 使用约束：ADC 触发顺序、电压滤波节奏和阈值判定保持不变。
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





