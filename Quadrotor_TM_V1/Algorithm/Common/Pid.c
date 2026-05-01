/*
 * 文件名称: Pid.c
 * 所属模块: Algorithm / Common
 *
 * 功能描述:
 *   通用 PID 控制器实现，供姿态/高度/位置所有控制环复用。
 *   PID_calculate() 包含以下特性：
 *   - 前馈项 (in_ff * k_ff)
 *   - 期望微分 (kd_ex) 和反馈微分 (kd_fb) 分离
 *   - 积分误差门限 (inte_d_lim)：误差超限时停止积分累积
 *   - 积分输出限幅 (inte_lim)：限制积分项最大贡献
 *
 * PID 公式:
 *   err = expect - feedback
 *   P   = kp * err
 *   I  += ki * err * dT  (当 |err| < inte_d_lim 时)
 *   D   = kd_ex * d(expect)/dT - kd_fb * d(feedback)/dT
 *   FF  = k_ff * in_ff
 *   out = P + I + D + FF
 *
 * 架构位置:
 *   纯计算函数，不依赖硬件。被 AttCtrl、AltCtrl、LocCtrl 调用。
 *
 * 教学提示:
 *   - 微分分离避免期望突变时的 D 项尖峰（经典"微分踢"问题）
 *   - inte_d_lim 防止大偏差时积分饱和，是抗积分饱和的简易方案
 *   - 所有控制环共用此函数，通过不同的 _PID_arg_st 参数实现不同行为
 */
#include "Pid.h"
#include "Math.h"
#include "Filter.h"
float PID_calculate( float dT_s,            //周期（单位：秒）
										float in_ff,				//前馈值
										float expect,				//期望值（设定值）
										float feedback,			//反馈值（）
										_PID_arg_st *pid_arg, //PID参数结构体
										_PID_val_st *pid_val,	//PID数据结构体
										float inte_d_lim,//积分误差限幅
										float inte_lim
										 )	
{
	float differential,hz;
	hz = safe_div(1.0f,dT_s,0);
	
//	pid_arg->k_inc_d_norm = LIMIT(pid_arg->k_inc_d_norm,0,1);
	
	
	pid_val->exp_d = (expect - pid_val->exp_old) *hz;
	
	if(pid_arg->fb_d_mode == 0)
	{
		pid_val->fb_d = (feedback - pid_val->feedback_old) *hz;
	}
	else
	{
		pid_val->fb_d = pid_val->fb_d_ex;
	}	
	differential = (pid_arg->kd_ex *pid_val->exp_d - pid_arg->kd_fb *pid_val->fb_d);
	
	pid_val->err = (expect - feedback);	
	pid_val->err_i += pid_arg->ki *LIMIT((pid_val->err ),-inte_d_lim,inte_d_lim )*dT_s;//)*T;//+ differential/pid_arg->kp
	//pid_val->err_i += pid_arg->ki *(pid_val->err )*T;//)*T;//+ pid_arg->k_pre_d *pid_val->feedback_d
	pid_val->err_i = LIMIT(pid_val->err_i,-inte_lim,inte_lim);
	
	
	
	pid_val->out = pid_arg->k_ff *in_ff 
	    + pid_arg->kp *pid_val->err  
			+	differential
//	    + pid_arg->k_inc_d_norm *pid_val->err_d_lpf + (1.0f-pid_arg->k_inc_d_norm) *differential
    	+ pid_val->err_i;
	
	pid_val->feedback_old = feedback;
	pid_val->exp_old = expect;
	
	return (pid_val->out);
}
