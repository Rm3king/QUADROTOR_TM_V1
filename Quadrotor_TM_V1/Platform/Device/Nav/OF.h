#ifndef __OF_H__
#define __OF_H__

#include "sysconfig.h"
/*
 * 模块名称：Ano_OF
 * 模块职责：定义光流协议解析后的公共状态和接收接口。
 * 使用约束：仅暴露解析结果和字节接收入口，不改变报文字段含义。
 */
/* 以下为光流模块解析后的公共状态。 */
/* 光流信息质量：QUA */
/* 光照强度：LIGHT */
extern uint8_t 	OF_STATE,OF_QUALITY;
//原始光流信息，具体意义见光流模块手册
extern int8_t	OF_DX,OF_DY;
//融合后的光流信息，具体意义见光流模块手册
extern int16_t	OF_DX2,OF_DY2,OF_DX2FIX,OF_DY2FIX;
//原始高度信息和融合后高度信息
extern uint16_t	OF_ALT,OF_ALT2;
//原始陀螺仪数据
extern int16_t	OF_GYR_X,OF_GYR_Y,OF_GYR_Z;
//滤波后陀螺仪数据
extern int16_t	OF_GYR_X2,OF_GYR_Y2,OF_GYR_Z2;
//原始加速度数据
extern int16_t	OF_ACC_X,OF_ACC_Y,OF_ACC_Z;
//滤波后加速度数据
extern int16_t	OF_ACC_X2,OF_ACC_Y2,OF_ACC_Z2;
//欧拉角格式的姿态数据
extern float	OF_ATT_ROL,OF_ATT_PIT,OF_ATT_YAW;
//四元数格式的姿态数据
extern float	OF_ATT_S1,OF_ATT_S2,OF_ATT_S3,OF_ATT_S4;


//本函数是唯一一个需要外部调用的函数，因为光流模块是串口输出数据
//所以本函数需要在串口接收中断中调用，每接收一字节数据，调用本函数一次
void AnoOF_GetOneByte(uint8_t data);
void AnoOF_DataAnl_Task(u8 dT_ms);
void AnoOF_Check(u8 dT_ms);
#endif
