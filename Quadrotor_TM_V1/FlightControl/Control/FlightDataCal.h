#ifndef __FLIGHT_DATA_COMP_H
#define __FLIGHT_DATA_COMP_H
/*
 * ???????????
 * ?????????????????????????
 */
#include "FcData.h"
#include "Filter.h"
#include "Math.h"
extern s32 baro_height;
extern u16 ref_tof_height;
void Fc_Sensor_Get(void);
void IMU_Update_Task(u8 dT_ms);
void Mag_Update_Task(u8 dT_ms);
void WCZ_Acc_Get_Task(void);
void WCZ_Fus_Task(u8 dT_ms);
#endif
