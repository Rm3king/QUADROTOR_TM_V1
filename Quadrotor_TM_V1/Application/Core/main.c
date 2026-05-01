/*
 * 文件名称: main.c
 * 所属模块: Application / Core
 *
 * 功能描述:
 *   飞控程序入口。完成硬件初始化后进入主循环，
 *   由 Scheduler.c 的 Main_Task() 驱动所有任务。
 *
 * 启动流程:
 *   1) Drv_BspInit()  -- 初始化 GPIO、SPI、UART、定时器、PWM 等硬件外设
 *   2) flag.start_ok = 1 -- 通知各模块硬件就绪
 *   3) Main_Task()    -- 进入协作式调度循环（永不返回）
 *
 * 教学提示:
 *   - 嵌入式系统没有操作系统，main() 的 while(1) 就是"操作系统"
 *   - 所有实时任务通过 Scheduler 的中断标志轮询驱动
 *   - Drv_BspInit() 内部完成全部硬件配置，之后才能使用传感器和通信
 */
#include "sysconfig.h"
#include "Drv_Bsp.h"
#include "Scheduler.h"
#include "FcData.h"

int main(void)
{
	Drv_BspInit();
	flag.start_ok = 1;

	while(1)
	{
		Main_Task();
	}
}
