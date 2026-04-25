#include "FcData.h"
#include "Parameter.h"

/*
 * 模块名称：FcData
 * 模块职责：定义飞控公共状态实例，并提供参数保存与初始化入口。
 * 使用约束：本文件只维护共享状态实例和轻量封装，不承载控制算法。
 */
_switch_st switchs;
 _save_st save;
_flag flag;
_fc_sta_var_st fc_stv;
_sensor_hd_check_st sens_hd_check;



/* 根据当前解锁状态触发参数保存请求。 */
void data_save(void)
{
	g_param_state.save_en = !flag.unlock_sta;
	g_param_state.save_trig = 1;
}


/* 初始化参数区读取。 */
void Para_Data_Init()
{

	FC_Param_Read();
}
