#ifndef __ANO_LED_H
#define __ANO_LED_H

/*
 * 模块名称：Ano_LED
 * 模块职责：定义 LED 状态指示相关的公共数据结构、位掩码与任务入口。
 * 使用约束：本头文件仅暴露调度器实际使用的公共接口。
 */
#include "sysconfig.h"


/* LED 编号定义。 */
enum
{
	X_led = 0,
	B_led,
	R_led,
	G_led,
	LED_NUM,
};

#define BIT_NULLLED 0x00		
#define BIT_XLED 0x01		//飞控LED
#define BIT_BLED 0x02		//蓝色
#define BIT_RLED 0x04		//红色
#define BIT_GLED 0x08		//绿色
#define BIT_WLED 0x0e		//白色
#define BIT_PLED 0x06		//紫色
#define BIT_YLED 0x0c		//黄色

typedef struct 
{
	u8 allOk;
	u8 lowVt;//低电压
	u8 rst_imu;
	u8 calGyr;
	u8 calAcc;
	u8 calMag;
	u8 calHz;	//校准水平面
	u8 errMpu;
	u8 errMag;
	u8 errBaro;
	u8 errOneTime;	//错误提示
	u8 noRc;	//失控
	u8 staOf;	//光流状态
	u8 staGps;	//GPS状态
	u8 saving;	//参数保存中
}_led_sta;




/* 对外状态数据。 */
extern _led_sta LED_STA;

/* 对外任务接口。 */
void LED_1ms_DRV(void );
void LED_Task2(u8 dT_ms);







#endif
