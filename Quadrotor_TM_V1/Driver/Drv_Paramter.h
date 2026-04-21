#ifndef _DRV_PARAMTER_H_
#define _DRV_PARAMTER_H_

#include "sysconfig.h"

/*
 * 模块名称：Drv_Paramter
 * 模块职责：提供参数区初始化、读取与保存接口。
 * 使用约束：仅整理接口说明，不修改参数存储时序。
 */

void Dvr_ParamterInit(void);
void Dvr_ParamterRead(void);
void Dvr_ParamterSave(void);

#endif

