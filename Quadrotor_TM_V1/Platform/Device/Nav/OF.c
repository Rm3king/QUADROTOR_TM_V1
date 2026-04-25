/*
 * 模块：光流数据解析
 * 职责：解析下位机光流协议并更新质量、高度与速度数据
 * 说明：保持原有报文格式和字段含义不变。
 */
#include "OF.h"
#include "FcData.h"
#include "Drv_Bsp.h"
/*
OF_STATE :
0bit: 1-高度有效；0-高度无效
1bit: 1-光流有效；0-光流无效
2bit: 1-高度融合有效；0-高度融合无效
3bit: 1-光流融合有效；0-光流融合无效
4:0
7bit: 1
*/
uint8_t		OF_STATE,OF_QUALITY;
int8_t		OF_DX,OF_DY;
int16_t		OF_DX2,OF_DY2,OF_DX2FIX,OF_DY2FIX;
uint16_t	OF_ALT,OF_ALT2;
int16_t		OF_GYR_X,OF_GYR_Y,OF_GYR_Z;
int16_t		OF_GYR_X2,OF_GYR_Y2,OF_GYR_Z2;
int16_t		OF_ACC_X,OF_ACC_Y,OF_ACC_Z;
int16_t		OF_ACC_X2,OF_ACC_Y2,OF_ACC_Z2;
float		OF_ATT_ROL,OF_ATT_PIT,OF_ATT_YAW;
float		OF_ATT_S1,OF_ATT_S2,OF_ATT_S3,OF_ATT_S4;
void AnoOF_DataAnl(uint8_t *data_buf,uint8_t num);
static uint8_t _datatemp[50];
static u8 _data_cnt = 0;
void AnoOF_DataAnl_Task(u8 dT_ms)
{
	AnoOF_Check(dT_ms);
}
/* 串口逐字节接收光流数据，组帧完成后进入统一解析。 */
void AnoOF_GetOneByte(uint8_t data)
{
	/* 当前工程保持 V3.0 报文格式。 */
	static u8 _data_len = 0;
	static u8 state = 0;
	
	if(state==0&&data==0xAA)
	{
		state=1;
		_datatemp[0]=data;
	}
	else if(state==1&&data==0x22)	//源地址
	{
		state=2;
		_datatemp[1]=data;
	}
	else if(state==2)			//目的地址
	{
		state=3;
		_datatemp[2]=data;
	}
	else if(state==3)			//功能字
	{
		state = 4;
		_datatemp[3]=data;
	}
	else if(state==4)			//长度
	{
		state = 5;
		_datatemp[4]=data;
		_data_len = data;
		_data_cnt = 0;
	}
	else if(state==5&&_data_len>0)
	{
		_data_len--;
		_datatemp[5+_data_cnt++]=data;
		if(_data_len==0)
			state = 6;
	}
	else if(state==6)
	{
		state = 0;
		_datatemp[5+_data_cnt]=data;
		AnoOF_DataAnl(_datatemp,_data_cnt+6);
	}
	else
		state = 0;
}
//AnoOF_DataAnl为光流数据解析函数，可以通过本函数得到光流模块输出的各项数据
//具体数据的意义，请参照匿名光流模块使用手册，有详细的介绍
static u8 of_check_f[2];
static u16 of_check_cnt[2] = { 10000,10000 };
void AnoOF_Check(u8 dT_ms)
{
	for(u8 i=0;i<2;i++)
	{
		if(of_check_f[i] == 0 )
		{
			if(of_check_cnt[i]<10000)
			{
				of_check_cnt[i] += dT_ms;	
			}
		}
		else
		{
			of_check_cnt[i] = 0;
		}
		
		of_check_f[i] = 0;
	}
	
	
	if(of_check_cnt[0] > 1000 || of_check_cnt[1] > 1000)
	{
		sens_hd_check.of_ok = 0;
	}
	else
	{
		sens_hd_check.of_ok = 1;
	}
		
	
}
void AnoOF_DataAnl(uint8_t *data_buf,uint8_t num)
{
	
	/* 当前工程仅保留 V3.0 光流协议解析。 */
