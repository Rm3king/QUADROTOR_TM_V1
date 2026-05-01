/*
 * 文件名称: Navigate.c
 * 所属模块: Algorithm / Estimation
 *
 * 功能描述:
 *   GPS 导航工具函数：
 *   1) dlon_dlat_dx_dy()  -- 经纬度差 → 水平距离(cm) 转换
 *   2) dlon_180()         -- 经度差限制在 ±180° 范围
 *
 * 转换原理:
 *   纬度方向：1° ≈ 111km，直接按比例换算
 *   经度方向：需乘以 cos(latitude) 补偿纬度收缩
 *
 * 教学提示:
 *   - GPS 输出经纬度精度为 10^-7 度，需要大整数运算避免浮点精度丢失
 *   - 在小范围内（几百米）可将地球表面近似为平面
 */
#include "Navigate.h"
#include "Math.h"
#include "Imu.h"

s32 dlon_180(s32 x) //10^-7
{
	return (x>1800000000?(x-3600000000):(x<-1800000000?(x+3600000000):x));

}

void dlon_dlat_dx_dy(s32 lon,s32 lat,s32 lon_ref,s32 lat_ref,s32 *dx,s32 *dy ) //10^-7  ->  cm
{
	s32 dlon_t;
	float lon_cos;
	
	dlon_t = dlon_180(lon - lon_ref);
	lon_cos = my_cos( ABS( (s16)(lat/10000000) ) *ANGLE_TO_RADIAN );
	
	*dx = 1.117f *dlon_t *lon_cos;
	*dy = 1.117f *(lat - lat_ref); //10^-7 *1000 *100 = 0.01f锛?.01f *111.7	

}


