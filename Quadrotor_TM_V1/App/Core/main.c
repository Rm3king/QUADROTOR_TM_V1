#include "sysconfig.h"
#include "Drv_Bsp.h"
#include "Scheduler.h"
#include "FcData.h"

int main(void)
{
	Drv_BspInit();
	//
	flag.start_ok = 1;
	
	while(1)
	{
		//轮询任务调度器
		Main_Task();
	}
}
