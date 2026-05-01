#include "FcData.h"
#include "Parameter.h"

/*
 * 文件名称: FcData.c
 * 所属模块: Algorithm / Common
 *
 * 功能描述:
 *   飞控公共数据定义文件，实例化全局共享的状态结构体：
 *   - switchs  (_switch_st)          -- 传感器/外设开关状态
 *   - flag     (_flag_st)            -- 飞行状态标志（解锁/起飞/模式等）
 *   - fc_stv   (_fc_sta_var_st)      -- 运行时限幅变量
 *   - sens_hd_check (_sensor_hd_check_st) -- 传感器在线状态
 *   - save     (_save_st)            -- 参数保存请求
 *
 * 架构位置:
 *   被全项目几乎所有模块 include，是飞控状态广播的中枢。
 *   对应头文件 FcData.h 定义了所有公共结构体和 extern 声明。
 *
 * 教学提示:
 *   - flag 结构体是理解飞控状态机的入口，建议从这里开始阅读
 *   - switchs 记录哪些外设当前可用，由 Swtich_State_Task() 更新
 */
_switch_st switchs;
 _save_st save;
_flag_st flag;
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
