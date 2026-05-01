/*
 * 文件名称: DT.c
 * 所属模块: Application / Service
 *
 * 功能描述:
 *   数据传输（Data Transmission）模块，实现飞控与上位机之间的双向通信：
 *
 *   发送方向（飞控→上位机）：
 *   1) ANO_DT_Send_Status()    -- 飞行状态（解锁/模式/电压）
 *   2) ANO_DT_Send_Sensor()    -- IMU 原始数据
 *   3) ANO_DT_Send_RCData()    -- 遥控通道值
 *   4) ANO_DT_Send_Power()     -- 电池电压电流
 *   5) ANO_DT_Send_Location()  -- GPS 经纬度
 *   6) ANO_DT_Send_User()      -- 自定义调试数据
 *
 *   接收方向（上位机→飞控）：
 *   1) dt_rx_feed()     -- 通用字节接收状态机（UART/USB 共用）
 *   2) ANO_DT_Data_Receive_Anl() -- 命令帧解析（校准/PID读写/飞控指令）
 *
 *   参数映射：
 *   1) para_set_mapping()  -- 上位机参数ID → g_fc_param 字段
 *   2) para_read_mapping() -- g_fc_param 字段 → 上位机回传
 *
 * 协议格式:
 *   帧头(0xAA) + 硬件地址 + 功能码 + 数据长度 + 数据 + 校验和
 *
 * 架构位置:
 *   由 Scheduler 按 2ms 周期调用 ANO_DT_Data_Send_Task()。
 *   通过 Drv_Uart (UART4) 或 USB CDC 与上位机通信。
 *
 * 教学提示:
 *   - 理解帧格式是调试飞控的基础，上位机软件（匿名地面站）使用相同协议
 *   - 接收状态机使用参数化设计：UART 和 USB 共用 dt_rx_feed()
 *   - 参数映射表是上位机调参功能的核心，修改参数结构时需同步更新
 */


#include "DT.h"
#include "Drv_Uart.h"
#include "FcUsbCdc.h"
#include "RC.h"
#include "Sensor_Basic.h"
#include "Drv_gps.h"
#include "Parameter.h"
#include "Imu.h"
#include "Drv_icm20602.h"
#include "MagProcess.h"
#include "MotorCtrl.h"
#include "Power.h"
#include "FlightCtrl.h"
#include "Drv_OpenMV.h"
#include "MotionCal.h"
#include "FlightDataCal.h"

#include "LocCtrl.h"
#include "FlyCtrl.h"
#include "OPMV_CBTracking_Ctrl.h"
#include "OF_DecoFusion.h"

/* ──────────── 多字节拆分宏 ──────────── */
#define BYTE0(dwTemp)       ( *( (char *)(&dwTemp)		) )
#define BYTE1(dwTemp)       ( *( (char *)(&dwTemp) + 1) )
#define BYTE2(dwTemp)       ( *( (char *)(&dwTemp) + 2) )
#define BYTE3(dwTemp)       ( *( (char *)(&dwTemp) + 3) )

/* ──────────── 协议地址与缓冲区 ──────────── */
#define MYHWADDR	0x05
#define SWJADDR		0xAF
#define ANO_DT_PARAM_COUNT		100
#define ANO_DT_TX_BUFFER_SIZE	50
#define ANO_DT_RX_BUFFER_SIZE	100
#define ANO_DT_RX_MAX_PAYLOAD	80
#define ANO_DT_FRAME_HEAD		0xAA
#define ANO_DT_FRAME_LEN_INDEX	4
#define ANO_DT_FRAME_DATA_INDEX	5

/* ──────────── 协议消息 ID ──────────── */
#define ANO_MSG_VER         0x00
#define ANO_MSG_STATUS      0x01
#define ANO_MSG_SENSOR      0x02
#define ANO_MSG_RCDATA      0x03
#define ANO_MSG_LOCATION    0x04
#define ANO_MSG_POWER       0x05
#define ANO_MSG_MOTOR       0x06
#define ANO_MSG_SENSOR2     0x07
#define ANO_MSG_SENSOR_STA  0x08
#define ANO_MSG_SPEED       0x0B
#define ANO_MSG_OMV_CT      0x41
#define ANO_MSG_OMV_LT      0x42
#define ANO_MSG_CMD         0xE0
#define ANO_MSG_PARAM       0xE1
#define ANO_MSG_USER        0xF1
#define ANO_MSG_STRING      0xA0
#define ANO_MSG_STRVAL      0xA1

/* ──────────── 命令帧功能码 (ANO_MSG_CMD 内) ──────────── */
#define ANO_CMD_CALI        0x01
#define ANO_CMD_RESET       0x02
#define ANO_CMD_READ_PARAM  0xE1
#define ANO_CMD_FLYCTRL     0x10

/* ──────────── 校准子码 (ANO_CMD_CALI 内 data[6..7]) ──────────── */
#define ANO_CALI_ACC        0x0001
#define ANO_CALI_GYRO       0x0002
#define ANO_CALI_MAG        0x0004
#define ANO_CALI_READ_VER   0x00B0

/* ──────────── 复位子码 (ANO_CMD_RESET 内 data[6..7]) ──────────── */
#define ANO_RESET_PID       0x00AA
#define ANO_RESET_PARAM     0x00AB
#define ANO_RESET_ALL       0x00AF

/* ──────────── 周期发送间隔 (ms) ──────────── */
#define DT_PERIOD_SENSOR    10      /* 100 Hz */
#define DT_PERIOD_SENSOR2   50      /*  20 Hz */
#define DT_PERIOD_USER      10      /* 100 Hz */
#define DT_PERIOD_STATUS    15      /*  66 Hz */
#define DT_PERIOD_RCDATA    20      /*  50 Hz */
#define DT_PERIOD_MOTOR     20      /*  50 Hz */
#define DT_PERIOD_POWER     50      /*  20 Hz */
#define DT_PERIOD_SPEED     50      /*  20 Hz */
#define DT_PERIOD_SSTA      500     /*   2 Hz */
#define DT_PERIOD_OMV       100     /*  10 Hz */
#define DT_PERIOD_LOCATION  500     /*   2 Hz */

/* ──────────── 其他常量 ──────────── */
#define DT_CNT_WRAP         1000
#define ANO_RC_NEUTRAL      1500
#define PARNUM		ANO_DT_PARAM_COUNT

/* ──────────── 模块变量 ──────────── */
s32 g_dt_param_list[ANO_DT_PARAM_COUNT];
dt_flag_t g_dt_flag;
static u8 data_to_send[ANO_DT_TX_BUFFER_SIZE];

/* ──────────── 接收状态机上下文 ──────────── */
typedef struct {
	u8 buf[ANO_DT_RX_BUFFER_SIZE];
	u8 cnt;
	u8 data_len;
	u8 state;
	u8 ok;
} dt_rx_ctx_t;

static dt_rx_ctx_t s_rx_uart;
static dt_rx_ctx_t s_rx_usb;

/* ──────────── 前向声明 ──────────── */
void ANO_DT_Send_Data(u8 *dataToSend , u8 length);
static u8 ANO_DT_FrameStart(u8 dest, u8 msg_id);
static u8 ANO_DT_AppendChecksum(u8 *buffer, u8 frame_len);
static void ANO_DT_FrameSend(u8 frame_len);
static void ANO_DT_UpdatePeriodFlags(u16 cnt, u8 *flag_send_omv);
static void ANO_DT_Data_Receive_Anl_Task(void);
static void dt_rx_feed(dt_rx_ctx_t *ctx, u8 data);
static void ANO_DT_Data_Receive_Anl(u8 *data_buf, u8 num);
static void ANO_DT_Send_VER(void);
static void ANO_DT_SendParame(u16 num);
static void ANO_DT_GetParame(u16 num,s32 data);
static void ANO_DT_ParListToParUsed(void);
static void ANO_DT_ParUsedToParList(void);

/* ================================================================
 *  组帧与发送
 * ================================================================ */

/* 组帧入口：写入协议头、目标地址和消息编号，并预留长度位。 */
static u8 ANO_DT_FrameStart(u8 dest, u8 msg_id)
{
	u8 frame_len = 0;

	data_to_send[frame_len++] = ANO_DT_FRAME_HEAD;
	data_to_send[frame_len++] = MYHWADDR;
	data_to_send[frame_len++] = dest;
	data_to_send[frame_len++] = msg_id;
	data_to_send[frame_len++] = 0;

	return frame_len;
}

/* 计算并追加单字节校验和。 */
static u8 ANO_DT_AppendChecksum(u8 *buffer, u8 frame_len)
{
	u8 sum = 0;

	for(u8 i = 0; i < frame_len; i++)
	{
		sum += buffer[i];
	}

	buffer[frame_len++] = sum;
	return frame_len;
}

/* 写回有效载荷长度后统一发送。 */
static void ANO_DT_FrameSend(u8 frame_len)
{
	data_to_send[ANO_DT_FRAME_LEN_INDEX] = frame_len - ANO_DT_FRAME_DATA_INDEX;
	frame_len = ANO_DT_AppendChecksum(data_to_send, frame_len);
	ANO_DT_Send_Data(data_to_send, frame_len);
}

/*
 * 功能：发送已经完成组帧的数据。
 * 说明：本函数是协议层统一发送出口，底层可切换为 USB 或串口。
 */
void ANO_DT_Send_Data(u8 *dataToSend , u8 length)
{
#ifdef ANO_DT_USE_USB
    UsbCdcSend( dataToSend , length );
#endif
#ifdef ANO_DT_USE_USART2
	Drv_UartDt_SendBuf(dataToSend, length);
#endif
}

/* ================================================================
 *  周期发送调度
 * ================================================================ */

/*
 * 功能：根据计数器更新各类周期发送标志。
 * 说明：仅负责置位请求，不直接执行发送。
 */
static void ANO_DT_UpdatePeriodFlags(u16 cnt, u8 *flag_send_omv)
{
	if((cnt % DT_PERIOD_SENSOR) == (DT_PERIOD_SENSOR - 1))
		g_dt_flag.send_senser = 1;

	if((cnt % DT_PERIOD_SENSOR2) == (DT_PERIOD_SENSOR2 - 1))
		g_dt_flag.send_senser2 = 1;

	if((cnt % DT_PERIOD_USER) == (DT_PERIOD_USER - 2))
		g_dt_flag.send_user = 1;

	if((cnt % DT_PERIOD_STATUS) == (DT_PERIOD_STATUS - 1))
		g_dt_flag.send_status = 1;

	if((cnt % DT_PERIOD_RCDATA) == (DT_PERIOD_RCDATA - 1))
		g_dt_flag.send_rcdata = 1;

	if((cnt % DT_PERIOD_MOTOR) == (DT_PERIOD_MOTOR - 2))
		g_dt_flag.send_motopwm = 1;

	if((cnt % DT_PERIOD_POWER) == (DT_PERIOD_POWER - 2))
		g_dt_flag.send_power = 1;

	if((cnt % DT_PERIOD_SPEED) == (DT_PERIOD_SPEED - 3))
		g_dt_flag.send_speed = 1;

	if((cnt % DT_PERIOD_SSTA) == (DT_PERIOD_SSTA - 2))
		g_dt_flag.send_sensorsta = 1;

	if((cnt % DT_PERIOD_OMV) == (DT_PERIOD_OMV - 2))
		*flag_send_omv = 1;

	if((cnt % DT_PERIOD_LOCATION) == (DT_PERIOD_LOCATION - 3))
		g_dt_flag.send_location = 1;
}

/*
 * 功能：协议数据轮询发送任务。
 * 调用周期：1ms。
 * 说明：按照既有优先级链逐项发送，发送顺序不可随意调整。
 */
void ANO_DT_Data_Exchange(void)
{
	static u16 cnt = 0;
	static u8	flag_send_omv = 0;

	ANO_DT_UpdatePeriodFlags(cnt, &flag_send_omv);

	if(++cnt > DT_CNT_WRAP)
		cnt = 0;
	if(g_dt_flag.send_version)
	{
		g_dt_flag.send_version = 0;
		ANO_DT_Send_Version(4,300,100,400,0);
	}
	else if(g_dt_flag.paraToSend < 0xffff)
	{
		ANO_DT_SendParame(g_dt_flag.paraToSend);
		g_dt_flag.paraToSend = 0xffff;
	}
	else if(g_dt_flag.send_status)
	{
		g_dt_flag.send_status = 0;
		ANO_DT_Send_Status(imu_data.rol,imu_data.pit,imu_data.yaw,wcz_hei_fus.out,(flag.flight_mode+1),flag.unlock_sta);
	}
	else if(g_dt_flag.send_speed)
	{
		g_dt_flag.send_speed = 0;
		ANO_DT_Send_Speed(loc_ctrl_1.fb[Y],loc_ctrl_1.fb[X],loc_ctrl_1.fb[Z]);
	}
	else if(g_dt_flag.send_user)
	{
		g_dt_flag.send_user = 0;
		ANO_DT_Send_User();
	}
	else if(g_dt_flag.send_senser)
	{
		g_dt_flag.send_senser = 0;
		ANO_DT_Send_Senser(sensor.Acc[X],sensor.Acc[Y],sensor.Acc[Z],sensor.Gyro[X],sensor.Gyro[Y],sensor.Gyro[Z],mag.val[X],mag.val[Y],mag.val[Z]);
	}
	else if(g_dt_flag.send_senser2)
	{
		g_dt_flag.send_senser2 = 0;
        ANO_DT_Send_Senser2(baro_height,ref_tof_height,sensor.Tempreature_C*10);
	}
	else if(flag_send_omv)
	{
		flag_send_omv = 0;
		if(g_dt_flag.send_omv_ct)
		{
			g_dt_flag.send_omv_ct = 0;
			ANO_DT_SendOmvCt(opmv.cb.color_flag,opmv.cb.sta,opmv.cb.pos_x,opmv.cb.pos_y,opmv.cb.dT_ms);
		}
		else if(g_dt_flag.send_omv_lt)
		{
			g_dt_flag.send_omv_lt = 0;
			ANO_DT_SendOmvLt(opmv.lt.sta, opmv.lt.angle, opmv.lt.deviation, opmv.lt.p_flag, opmv.lt.pos_x, opmv.lt.pos_y, opmv.lt.dT_ms);
		}
	}
	else if(g_dt_flag.send_rcdata)
	{
		g_dt_flag.send_rcdata = 0;
		s16 CH_GCS[CH_NUM];
		u8 en_mask = RC_GetChannelEnableMask();
		for(u8 i=0;i<CH_NUM;i++)
		{
            if((en_mask & (1<<i)))
			{
				CH_GCS[i] = RC_GetChannel(i) + ANO_RC_NEUTRAL;
			}
			else
			{
				CH_GCS[i] = 0;
			}
		}
		ANO_DT_Send_RCData(CH_GCS[2],CH_GCS[3],CH_GCS[0],CH_GCS[1],CH_GCS[4],CH_GCS[5],CH_GCS[6],CH_GCS[7],0,0);
	}
	else if(g_dt_flag.send_motopwm)
	{
		g_dt_flag.send_motopwm = 0;
#if MOTORSNUM == 8
		ANO_DT_Send_MotoPWM(motor[0],motor[1],motor[2],motor[3],motor[4],motor[5],motor[6],motor[7]);
#elif MOTORSNUM == 6
		ANO_DT_Send_MotoPWM(motor[0],motor[1],motor[2],motor[3],motor[4],motor[5],0,0);
#elif MOTORSNUM == 4
		ANO_DT_Send_MotoPWM(motor[0],motor[1],motor[2],motor[3],0,0,0,0);
#else

#endif
	}
	else if(g_dt_flag.send_power)
	{
		g_dt_flag.send_power = 0;
		ANO_DT_Send_Power(Plane_Votage*100,0);
	}
	else if(g_dt_flag.send_sensorsta)
	{
		g_dt_flag.send_sensorsta = 0;
		ANO_DT_SendSensorSta(switchs.of_flow_on ,switchs.gps_on,switchs.opmv_on,switchs.uwb_on,switchs.of_tof_on);
	}
	else if(g_dt_flag.send_location)
	{
		g_dt_flag.send_location = 0;
		ANO_DT_Send_Location(switchs.gps_on,Gps_information.satellite_num,(s32)Gps_information.longitude,(s32)Gps_information.latitude,123,456);

	}
	else if(g_dt_flag.send_vef)
	{
		ANO_DT_Send_VER();
		g_dt_flag.send_vef = 0;
	}
	ANO_DT_Data_Receive_Anl_Task();
}

/* ================================================================
 *  协议接收
 * ================================================================ */

/*
 * 功能：通用接收状态机。
 * 说明：UART 和 USB 共用同一逻辑，通过独立的 dt_rx_ctx_t 上下文隔离状态。
 *
 * 帧格式：[HEAD 0xAA] [SRC 0xAF] [DEST] [MSG_ID] [LEN] [DATA...] [CHECKSUM]
 */
static void dt_rx_feed(dt_rx_ctx_t *ctx, u8 data)
{
	if(ctx->state==0 && data==ANO_DT_FRAME_HEAD)
	{
		ctx->state = 1;
		ctx->buf[0] = data;
	}
	else if(ctx->state==1 && data==SWJADDR)
	{
		ctx->state = 2;
		ctx->buf[1] = data;
	}
	else if(ctx->state==2)
	{
		ctx->state = 3;
		ctx->buf[2] = data;
	}
	else if(ctx->state==3)
	{
		ctx->state = 4;
		ctx->buf[3] = data;
	}
	else if(ctx->state==4)
	{
		ctx->state = 5;
		ctx->buf[4] = data;
		ctx->data_len = data;
		ctx->cnt = 0;
	}
	else if(ctx->state==5 && ctx->data_len>0 && ctx->data_len<ANO_DT_RX_MAX_PAYLOAD)
	{
		ctx->data_len--;
		ctx->buf[5 + ctx->cnt++] = data;
		if(ctx->data_len==0)
			ctx->state = 6;
	}
	else if(ctx->state==6)
	{
		ctx->state = 0;
		ctx->buf[5 + ctx->cnt] = data;
		ctx->ok = 1;
	}
	else
		ctx->state = 0;
}

/* 串口接收入口（由 UART ISR 逐字节调用，公开接口不变）。 */
void ANO_DT_Data_Receive_Prepare(u8 data)
{
	dt_rx_feed(&s_rx_uart, data);
}

/*
 * 功能：轮询接收缓冲区并触发协议解析。
 * 说明：串口和 USB 共用同一套解析函数。
 */
static void ANO_DT_Data_Receive_Anl_Task(void)
{
	static u8 usbdatarxbuf[ANO_DT_RX_BUFFER_SIZE];

	if(s_rx_uart.ok)
	{
		ANO_DT_Data_Receive_Anl(s_rx_uart.buf, s_rx_uart.cnt + 6);
		s_rx_uart.ok = 0;
	}
	u8 len = UsbCdcRead(usbdatarxbuf, ANO_DT_RX_BUFFER_SIZE);
	if(len)
	{
		for(u8 i=0; i<len; i++)
			dt_rx_feed(&s_rx_usb, usbdatarxbuf[i]);
	}
	if(s_rx_usb.ok)
	{
		ANO_DT_Data_Receive_Anl(s_rx_usb.buf, s_rx_usb.cnt + 6);
		s_rx_usb.ok = 0;
	}
}

/* ================================================================
 *  协议解析与命令分发
 * ================================================================ */

/*
 * 功能：解析单帧协议数据。
 * 说明：先做帧头和校验检查，再分发命令或参数写入请求。
 */
static void ANO_DT_Data_Receive_Anl(u8 *data_buf,u8 num)
{
	u8 sum = 0;
	for(u8 i=0;i<(num-1);i++)
		sum += *(data_buf+i);
	if(!(sum==*(data_buf+num-1)))		return;
	if(!(*(data_buf)==ANO_DT_FRAME_HEAD && *(data_buf+1)==SWJADDR))		return;

	if(*(data_buf+2)==MYHWADDR)
	{
		if(*(data_buf+3)==ANO_MSG_CMD)
		{
			u16 sub = (u16)(*(data_buf+6)<<8)|*(data_buf+7);
			switch(*(data_buf+5))
			{
				case ANO_CMD_CALI:
					if(sub == ANO_CALI_ACC)
						sensor.acc_CALIBRATE = 1;
					if(sub == ANO_CALI_GYRO)
						sensor.gyr_CALIBRATE = 1;
					if(sub == ANO_CALI_MAG)
						mag.mag_CALIBRATE = 1;
					if(sub == ANO_CALI_READ_VER)
						g_dt_flag.send_version = 1;
					break;
				case ANO_CMD_RESET:
					if(sub == ANO_RESET_PID)
					{
						FC_Param_ResetPid();
						All_PID_Init();
						data_save();
					}
					if(sub == ANO_RESET_PARAM)
					{
						FC_Param_Reset();
						data_save();
					}
					if(sub == ANO_RESET_ALL)
					{
						FC_Param_ResetPid();
						All_PID_Init();
						FC_Param_Reset();
						data_save();
					}
					break;
				case ANO_CMD_READ_PARAM:
					g_dt_flag.paraToSend = (u16)(*(data_buf+6)<<8)|*(data_buf+7);
					break;
				case ANO_CMD_FLYCTRL:
					FlyCtrlDataAnl(data_buf+5);
					break;
				default:
					break;
			}
			ANO_DT_SendCmd(SWJADDR,*(data_buf+5),(u16)(*(data_buf+6)<<8)|*(data_buf+7),(u16)(*(data_buf+8)<<8)|*(data_buf+9),(u16)(*(data_buf+10)<<8)|*(data_buf+11),(u16)(*(data_buf+12)<<8)|*(data_buf+13),(u16)(*(data_buf+14)<<8)|*(data_buf+15));
		}
		else if(*(data_buf+3)==ANO_MSG_PARAM)
		{
			u16 _paraNum = (u16)(*(data_buf+5)<<8)|*(data_buf+6);
			s32 _paraVal = (s32)(((*(data_buf+7))<<24) + ((*(data_buf+8))<<16) + ((*(data_buf+9))<<8) + (*(data_buf+10)));
			ANO_DT_GetParame(_paraNum,_paraVal);
		}
	}
}

/* 发送命令回包，保持上位机与飞控命令交互一致。 */
void ANO_DT_SendCmd(u8 dest, u8 fun, u16 cmd1, u16 cmd2, u16 cmd3, u16 cmd4, u16 cmd5)
{
	u8 _cnt = ANO_DT_FrameStart(dest, ANO_MSG_CMD);

	data_to_send[_cnt++]=fun;
	data_to_send[_cnt++]=BYTE1(cmd1);
	data_to_send[_cnt++]=BYTE0(cmd1);
	data_to_send[_cnt++]=BYTE1(cmd2);
	data_to_send[_cnt++]=BYTE0(cmd2);
	data_to_send[_cnt++]=BYTE1(cmd3);
	data_to_send[_cnt++]=BYTE0(cmd3);
	data_to_send[_cnt++]=BYTE1(cmd4);
	data_to_send[_cnt++]=BYTE0(cmd4);
	data_to_send[_cnt++]=BYTE1(cmd5);
	data_to_send[_cnt++]=BYTE0(cmd5);

	ANO_DT_FrameSend(_cnt);
}

/* ================================================================
 *  参数读写
 * ================================================================ */

/* 按参数编号回传单个参数值。 */
static void ANO_DT_SendParame(u16 num)
{
	u8 _cnt = ANO_DT_FrameStart(SWJADDR, ANO_MSG_PARAM);
	int32_t data;
	if(num > PARNUM)
		return;
	ANO_DT_ParUsedToParList();
	data = g_dt_param_list[num];
	data_to_send[_cnt++]=BYTE1(num);
	data_to_send[_cnt++]=BYTE0(num);
	data_to_send[_cnt++]=BYTE3(data);
	data_to_send[_cnt++]=BYTE2(data);
	data_to_send[_cnt++]=BYTE1(data);
	data_to_send[_cnt++]=BYTE0(data);

	ANO_DT_FrameSend(_cnt);
}
/* 写入单个参数并触发保存流程。 */
static void ANO_DT_GetParame(u16 num,s32 data)
{
	if(num > PARNUM)
		return;
	g_dt_param_list[num] = data;
	ANO_DT_ParListToParUsed();
	g_dt_flag.paraToSend = num;
	data_save();
}
/* 将参数列表同步到实际飞控参数结构。 */
static void ANO_DT_ParListToParUsed(void)
{
	g_fc_param.set.pid_att_1level[ROL][KP] = (float) g_dt_param_list[PAR_PID_1_P] / 1000;
	g_fc_param.set.pid_att_1level[ROL][KI] = (float) g_dt_param_list[PAR_PID_1_I] / 1000;
	g_fc_param.set.pid_att_1level[ROL][KD] = (float) g_dt_param_list[PAR_PID_1_D] / 1000;
	g_fc_param.set.pid_att_1level[PIT][KP] = (float) g_dt_param_list[PAR_PID_2_P] / 1000;
	g_fc_param.set.pid_att_1level[PIT][KI] = (float) g_dt_param_list[PAR_PID_2_I] / 1000;
	g_fc_param.set.pid_att_1level[PIT][KD] = (float) g_dt_param_list[PAR_PID_2_D] / 1000;
	g_fc_param.set.pid_att_1level[YAW][KP] = (float) g_dt_param_list[PAR_PID_3_P] / 1000;
	g_fc_param.set.pid_att_1level[YAW][KI] = (float) g_dt_param_list[PAR_PID_3_I] / 1000;
	g_fc_param.set.pid_att_1level[YAW][KD] = (float) g_dt_param_list[PAR_PID_3_D] / 1000;

	g_fc_param.set.pid_att_2level[ROL][KP] = (float) g_dt_param_list[PAR_PID_4_P] / 1000;
	g_fc_param.set.pid_att_2level[ROL][KI] = (float) g_dt_param_list[PAR_PID_4_I] / 1000;
	g_fc_param.set.pid_att_2level[ROL][KD] = (float) g_dt_param_list[PAR_PID_4_D] / 1000;
	g_fc_param.set.pid_att_2level[PIT][KP] = (float) g_dt_param_list[PAR_PID_5_P] / 1000;
	g_fc_param.set.pid_att_2level[PIT][KI] = (float) g_dt_param_list[PAR_PID_5_I] / 1000;
	g_fc_param.set.pid_att_2level[PIT][KD] = (float) g_dt_param_list[PAR_PID_5_D] / 1000;
	g_fc_param.set.pid_att_2level[YAW][KP] = (float) g_dt_param_list[PAR_PID_6_P] / 1000;
	g_fc_param.set.pid_att_2level[YAW][KI] = (float) g_dt_param_list[PAR_PID_6_I] / 1000;
	g_fc_param.set.pid_att_2level[YAW][KD] = (float) g_dt_param_list[PAR_PID_6_D] / 1000;

	g_fc_param.set.pid_alt_1level[KP] = (float) g_dt_param_list[PAR_PID_7_P] / 1000;
	g_fc_param.set.pid_alt_1level[KI] = (float) g_dt_param_list[PAR_PID_7_I] / 1000;
	g_fc_param.set.pid_alt_1level[KD] = (float) g_dt_param_list[PAR_PID_7_D] / 1000;
	g_fc_param.set.pid_alt_2level[KP] = (float) g_dt_param_list[PAR_PID_8_P] / 1000;
	g_fc_param.set.pid_alt_2level[KI] = (float) g_dt_param_list[PAR_PID_8_I] / 1000;
	g_fc_param.set.pid_alt_2level[KD] = (float) g_dt_param_list[PAR_PID_8_D] / 1000;

	g_fc_param.set.pid_loc_1level[KP] = (float) g_dt_param_list[PAR_PID_9_P] / 1000;
	g_fc_param.set.pid_loc_1level[KI] = (float) g_dt_param_list[PAR_PID_9_I] / 1000;
	g_fc_param.set.pid_loc_1level[KD] = (float) g_dt_param_list[PAR_PID_9_D] / 1000;
	g_fc_param.set.pid_loc_2level[KP] = (float) g_dt_param_list[PAR_PID_10_P] / 1000;
	g_fc_param.set.pid_loc_2level[KI] = (float) g_dt_param_list[PAR_PID_10_I] / 1000;
	g_fc_param.set.pid_loc_2level[KD] = (float) g_dt_param_list[PAR_PID_10_D] / 1000;

	g_fc_param.set.pid_gps_loc_1level[KP] = (float) g_dt_param_list[PAR_PID_11_P] / 1000;
	g_fc_param.set.pid_gps_loc_1level[KI] = (float) g_dt_param_list[PAR_PID_11_I] / 1000;
	g_fc_param.set.pid_gps_loc_1level[KD] = (float) g_dt_param_list[PAR_PID_11_D] / 1000;
	g_fc_param.set.pid_gps_loc_2level[KP] = (float) g_dt_param_list[PAR_PID_12_P] / 1000;
	g_fc_param.set.pid_gps_loc_2level[KI] = (float) g_dt_param_list[PAR_PID_12_I] / 1000;
	g_fc_param.set.pid_gps_loc_2level[KD] = (float) g_dt_param_list[PAR_PID_12_D] / 1000;

	if(g_dt_param_list[PAR_RCINMODE] == 0)
		g_fc_param.set.pwmInMode = PWM;
	else if(g_dt_param_list[PAR_RCINMODE] == 1)
		g_fc_param.set.pwmInMode = PPM;
	else
		g_fc_param.set.pwmInMode = SBUS;

	g_fc_param.set.warn_power_voltage = (float) g_dt_param_list[PAR_LVWARN] / 10;
	g_fc_param.set.return_home_power_voltage = (float) g_dt_param_list[PAR_LVRETN] / 10;
	g_fc_param.set.lowest_power_voltage = (float) g_dt_param_list[PAR_LVDOWN] / 10;

	g_fc_param.set.auto_take_off_height = g_dt_param_list[PAR_TAKEOFFHIGH];
	g_fc_param.set.auto_take_off_speed = g_dt_param_list[PAR_TAKEOFFSPEED];
	g_fc_param.set.auto_landing_speed = g_dt_param_list[PAR_LANDSPEED];
	g_fc_param.set.idle_speed_pwm	 = g_dt_param_list[PAR_UNLOCKPWM];

	if(g_dt_param_list[PAR_HEATSWITCH] == 0)
		g_fc_param.set.heatSwitch = 0;
	else
		g_fc_param.set.heatSwitch = 1;
}
/* 将实际飞控参数结构回填到协议参数列表。 */
static void ANO_DT_ParUsedToParList(void)
{
	g_dt_param_list[PAR_PID_1_P] = g_fc_param.set.pid_att_1level[ROL][KP] * 1000;
	g_dt_param_list[PAR_PID_1_I] = g_fc_param.set.pid_att_1level[ROL][KI] * 1000;
	g_dt_param_list[PAR_PID_1_D] = g_fc_param.set.pid_att_1level[ROL][KD] * 1000;
	g_dt_param_list[PAR_PID_2_P] = g_fc_param.set.pid_att_1level[PIT][KP] * 1000;
	g_dt_param_list[PAR_PID_2_I] = g_fc_param.set.pid_att_1level[PIT][KI] * 1000;
	g_dt_param_list[PAR_PID_2_D] = g_fc_param.set.pid_att_1level[PIT][KD] * 1000;
	g_dt_param_list[PAR_PID_3_P] = g_fc_param.set.pid_att_1level[YAW][KP] * 1000;
	g_dt_param_list[PAR_PID_3_I] = g_fc_param.set.pid_att_1level[YAW][KI] * 1000;
	g_dt_param_list[PAR_PID_3_D] = g_fc_param.set.pid_att_1level[YAW][KD] * 1000;

	g_dt_param_list[PAR_PID_4_P] = g_fc_param.set.pid_att_2level[ROL][KP] * 1000;
	g_dt_param_list[PAR_PID_4_I] = g_fc_param.set.pid_att_2level[ROL][KI] * 1000;
	g_dt_param_list[PAR_PID_4_D] = g_fc_param.set.pid_att_2level[ROL][KD] * 1000;
	g_dt_param_list[PAR_PID_5_P] = g_fc_param.set.pid_att_2level[PIT][KP] * 1000;
	g_dt_param_list[PAR_PID_5_I] = g_fc_param.set.pid_att_2level[PIT][KI] * 1000;
	g_dt_param_list[PAR_PID_5_D] = g_fc_param.set.pid_att_2level[PIT][KD] * 1000;
	g_dt_param_list[PAR_PID_6_P] = g_fc_param.set.pid_att_2level[YAW][KP] * 1000;
	g_dt_param_list[PAR_PID_6_I] = g_fc_param.set.pid_att_2level[YAW][KI] * 1000;
	g_dt_param_list[PAR_PID_6_D] = g_fc_param.set.pid_att_2level[YAW][KD] * 1000;

	g_dt_param_list[PAR_PID_7_P] = g_fc_param.set.pid_alt_1level[KP] * 1000;
	g_dt_param_list[PAR_PID_7_I] = g_fc_param.set.pid_alt_1level[KI] * 1000;
	g_dt_param_list[PAR_PID_7_D] = g_fc_param.set.pid_alt_1level[KD] * 1000;
	g_dt_param_list[PAR_PID_8_P] = g_fc_param.set.pid_alt_2level[KP] * 1000;
	g_dt_param_list[PAR_PID_8_I] = g_fc_param.set.pid_alt_2level[KI] * 1000;
	g_dt_param_list[PAR_PID_8_D] = g_fc_param.set.pid_alt_2level[KD] * 1000;

	g_dt_param_list[PAR_PID_9_P] = g_fc_param.set.pid_loc_1level[KP] * 1000;
	g_dt_param_list[PAR_PID_9_I] = g_fc_param.set.pid_loc_1level[KI] * 1000;
	g_dt_param_list[PAR_PID_9_D] = g_fc_param.set.pid_loc_1level[KD] * 1000;
	g_dt_param_list[PAR_PID_10_P] = g_fc_param.set.pid_loc_2level[KP] * 1000;
	g_dt_param_list[PAR_PID_10_I] = g_fc_param.set.pid_loc_2level[KI] * 1000;
	g_dt_param_list[PAR_PID_10_D] = g_fc_param.set.pid_loc_2level[KD] * 1000;

	g_dt_param_list[PAR_PID_11_P] = g_fc_param.set.pid_gps_loc_1level[KP] * 1000;
	g_dt_param_list[PAR_PID_11_I] = g_fc_param.set.pid_gps_loc_1level[KI] * 1000;
	g_dt_param_list[PAR_PID_11_D] = g_fc_param.set.pid_gps_loc_1level[KD] * 1000;
	g_dt_param_list[PAR_PID_12_P] = g_fc_param.set.pid_gps_loc_2level[KP] * 1000;
	g_dt_param_list[PAR_PID_12_I] = g_fc_param.set.pid_gps_loc_2level[KI] * 1000;
	g_dt_param_list[PAR_PID_12_D] = g_fc_param.set.pid_gps_loc_2level[KD] * 1000;

	if(g_fc_param.set.pwmInMode == PWM)
		g_dt_param_list[PAR_RCINMODE] = 0;
	else if(g_fc_param.set.pwmInMode == PPM)
		g_dt_param_list[PAR_RCINMODE] = 1;
	else
		g_dt_param_list[PAR_RCINMODE] = 2;

	g_dt_param_list[PAR_LVWARN] = g_fc_param.set.warn_power_voltage * 10;
	g_dt_param_list[PAR_LVRETN] = g_fc_param.set.return_home_power_voltage * 10;
	g_dt_param_list[PAR_LVDOWN] = g_fc_param.set.lowest_power_voltage * 10;

	g_dt_param_list[PAR_TAKEOFFHIGH] = g_fc_param.set.auto_take_off_height;
	g_dt_param_list[PAR_TAKEOFFSPEED] = g_fc_param.set.auto_take_off_speed;
	g_dt_param_list[PAR_LANDSPEED] = g_fc_param.set.auto_landing_speed;
	g_dt_param_list[PAR_UNLOCKPWM] = g_fc_param.set.idle_speed_pwm;

	if(g_fc_param.set.heatSwitch == 0)
		g_dt_param_list[PAR_HEATSWITCH] = 0;
	else
		g_dt_param_list[PAR_HEATSWITCH] = 1;
}

/* ================================================================
 *  数据帧发送函数
 * ================================================================ */

/* 保留的旧版版本信息发送接口（帧格式与新协议不同：0xAA 0xAA 0x00）。 */
static void ANO_DT_Send_VER(void)
{
	u8 temp[14];
	temp[0] = ANO_DT_FRAME_HEAD;
	temp[1] = ANO_DT_FRAME_HEAD;
	temp[2] = 0x00;
	temp[3] = 9;
	temp[4] = HW_TYPE;
	temp[5] = HW_VER/256;
	temp[6] = HW_VER%256;
	temp[7] = SOFT_VER/256;
	temp[8] = SOFT_VER%256;
	temp[9] = PT_VER/256;
	temp[10] = PT_VER%256;
	temp[11] = BL_VER/256;
	temp[12] = BL_VER%256;
	ANO_DT_AppendChecksum(temp, 13);

	ANO_DT_Send_Data(temp,14);
}

/* 发送版本信息帧。 */
void ANO_DT_Send_Version(u8 hardware_type, u16 hardware_ver,u16 software_ver,u16 protocol_ver,u16 bootloader_ver)
{
	u8 _cnt = ANO_DT_FrameStart(SWJADDR, ANO_MSG_VER);

	data_to_send[_cnt++]=hardware_type;
	data_to_send[_cnt++]=BYTE1(hardware_ver);
	data_to_send[_cnt++]=BYTE0(hardware_ver);
	data_to_send[_cnt++]=BYTE1(software_ver);
	data_to_send[_cnt++]=BYTE0(software_ver);
	data_to_send[_cnt++]=BYTE1(protocol_ver);
	data_to_send[_cnt++]=BYTE0(protocol_ver);
	data_to_send[_cnt++]=BYTE1(bootloader_ver);
	data_to_send[_cnt++]=BYTE0(bootloader_ver);

	ANO_DT_FrameSend(_cnt);
}

/* 发送速度信息。 */
void ANO_DT_Send_Speed(float x_s,float y_s,float z_s)
{
	u8 _cnt = ANO_DT_FrameStart(SWJADDR, ANO_MSG_SPEED);
	s16 _temp;

	_temp = (int)(0.1f *x_s);
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (int)(0.1f *y_s);
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (int)(0.1f *z_s);
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);

	ANO_DT_FrameSend(_cnt);

}

/* 发送定位状态、经纬度和返航信息。 */
void ANO_DT_Send_Location(u8 state,u8 sat_num,s32 lon,s32 lat,float back_home_angle,float back_home_dist)
{
	u8 _cnt = ANO_DT_FrameStart(SWJADDR, ANO_MSG_LOCATION);
	s16 _temp;
	s32 _temp2;
	u16 _temp3;

	data_to_send[_cnt++]=state;
	data_to_send[_cnt++]=sat_num;

	_temp2 = lon;
	data_to_send[_cnt++]=BYTE3(_temp2);
	data_to_send[_cnt++]=BYTE2(_temp2);
	data_to_send[_cnt++]=BYTE1(_temp2);
	data_to_send[_cnt++]=BYTE0(_temp2);

	_temp2 = lat;
	data_to_send[_cnt++]=BYTE3(_temp2);
	data_to_send[_cnt++]=BYTE2(_temp2);
	data_to_send[_cnt++]=BYTE1(_temp2);
	data_to_send[_cnt++]=BYTE0(_temp2);


	_temp = (s16)(10 *back_home_angle);
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);

	_temp3 = (u16)(back_home_dist);
	data_to_send[_cnt++]=BYTE1(_temp3);
	data_to_send[_cnt++]=BYTE0(_temp3);

	ANO_DT_FrameSend(_cnt);

}


/* 发送姿态、高度、模式和解锁状态。 */
void ANO_DT_Send_Status(float angle_rol, float angle_pit, float angle_yaw, s32 alt, u8 fly_model, u8 armed)
{
	u8 _cnt = ANO_DT_FrameStart(SWJADDR, ANO_MSG_STATUS);
	s16 _temp;
	s32 _temp2 = alt;

	_temp = (int)(angle_rol*100);
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (int)(angle_pit*100);
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (int)(angle_yaw*100);
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);

	data_to_send[_cnt++]=BYTE3(_temp2);
	data_to_send[_cnt++]=BYTE2(_temp2);
	data_to_send[_cnt++]=BYTE1(_temp2);
	data_to_send[_cnt++]=BYTE0(_temp2);

	data_to_send[_cnt++] = fly_model;

	data_to_send[_cnt++] = armed;

	ANO_DT_FrameSend(_cnt);
}
/* 发送加速度、角速度和磁力计原始数据。 */
void ANO_DT_Send_Senser(s16 a_x,s16 a_y,s16 a_z,s16 g_x,s16 g_y,s16 g_z,s16 m_x,s16 m_y,s16 m_z)
{
	u8 _cnt = ANO_DT_FrameStart(SWJADDR, ANO_MSG_SENSOR);
	s16 _temp;

	_temp = a_x;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = a_y;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = a_z;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);

	_temp = g_x;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = g_y;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = g_z;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);

	_temp = m_x;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = m_y;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = m_z;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);

	ANO_DT_FrameSend(_cnt);
}
/* 发送气压、测距和温度等补充传感器数据。 */
void ANO_DT_Send_Senser2(s32 bar_alt,s32 csb_alt, s16 sensertmp)
{
	u8 _cnt = ANO_DT_FrameStart(SWJADDR, ANO_MSG_SENSOR2);

	data_to_send[_cnt++]=BYTE3(bar_alt);
	data_to_send[_cnt++]=BYTE2(bar_alt);
	data_to_send[_cnt++]=BYTE1(bar_alt);
	data_to_send[_cnt++]=BYTE0(bar_alt);

	data_to_send[_cnt++]=BYTE3(csb_alt);
	data_to_send[_cnt++]=BYTE2(csb_alt);
	data_to_send[_cnt++]=BYTE1(csb_alt);
	data_to_send[_cnt++]=BYTE0(csb_alt);

	data_to_send[_cnt++]=BYTE1(sensertmp);
	data_to_send[_cnt++]=BYTE0(sensertmp);

	ANO_DT_FrameSend(_cnt);
}
/* 发送遥控器通道数据。 */
void ANO_DT_Send_RCData(u16 thr,u16 yaw,u16 rol,u16 pit,u16 aux1,u16 aux2,u16 aux3,u16 aux4,u16 aux5,u16 aux6)
{
	u8 _cnt = ANO_DT_FrameStart(SWJADDR, ANO_MSG_RCDATA);

	data_to_send[_cnt++]=BYTE1(thr);
	data_to_send[_cnt++]=BYTE0(thr);
	data_to_send[_cnt++]=BYTE1(yaw);
	data_to_send[_cnt++]=BYTE0(yaw);
	data_to_send[_cnt++]=BYTE1(rol);
	data_to_send[_cnt++]=BYTE0(rol);
	data_to_send[_cnt++]=BYTE1(pit);
	data_to_send[_cnt++]=BYTE0(pit);
	data_to_send[_cnt++]=BYTE1(aux1);
	data_to_send[_cnt++]=BYTE0(aux1);
	data_to_send[_cnt++]=BYTE1(aux2);
	data_to_send[_cnt++]=BYTE0(aux2);
	data_to_send[_cnt++]=BYTE1(aux3);
	data_to_send[_cnt++]=BYTE0(aux3);
	data_to_send[_cnt++]=BYTE1(aux4);
	data_to_send[_cnt++]=BYTE0(aux4);
	data_to_send[_cnt++]=BYTE1(aux5);
	data_to_send[_cnt++]=BYTE0(aux5);
	data_to_send[_cnt++]=BYTE1(aux6);
	data_to_send[_cnt++]=BYTE0(aux6);

	ANO_DT_FrameSend(_cnt);
}
/* 发送电压和电流信息。 */
void ANO_DT_Send_Power(u16 votage, u16 current)
{
	u8 _cnt = ANO_DT_FrameStart(SWJADDR, ANO_MSG_POWER);
	u16 temp;

	temp = votage;
	data_to_send[_cnt++]=BYTE1(temp);
	data_to_send[_cnt++]=BYTE0(temp);
	temp = current;
	data_to_send[_cnt++]=BYTE1(temp);
	data_to_send[_cnt++]=BYTE0(temp);

	ANO_DT_FrameSend(_cnt);
}
/* 发送电机输出占空比。 */
void ANO_DT_Send_MotoPWM(u16 m_1,u16 m_2,u16 m_3,u16 m_4,u16 m_5,u16 m_6,u16 m_7,u16 m_8)
{
	u8 _cnt = ANO_DT_FrameStart(SWJADDR, ANO_MSG_MOTOR);

	data_to_send[_cnt++]=BYTE1(m_1);
	data_to_send[_cnt++]=BYTE0(m_1);
	data_to_send[_cnt++]=BYTE1(m_2);
	data_to_send[_cnt++]=BYTE0(m_2);
	data_to_send[_cnt++]=BYTE1(m_3);
	data_to_send[_cnt++]=BYTE0(m_3);
	data_to_send[_cnt++]=BYTE1(m_4);
	data_to_send[_cnt++]=BYTE0(m_4);
	data_to_send[_cnt++]=BYTE1(m_5);
	data_to_send[_cnt++]=BYTE0(m_5);
	data_to_send[_cnt++]=BYTE1(m_6);
	data_to_send[_cnt++]=BYTE0(m_6);
	data_to_send[_cnt++]=BYTE1(m_7);
	data_to_send[_cnt++]=BYTE0(m_7);
	data_to_send[_cnt++]=BYTE1(m_8);
	data_to_send[_cnt++]=BYTE0(m_8);

	ANO_DT_FrameSend(_cnt);
}

/* 发送纯字符串调试信息。 */
void ANO_DT_SendString(const char *str)
{
	u8 _cnt = ANO_DT_FrameStart(SWJADDR, ANO_MSG_STRING);
	u8 i = 0;
	while(*(str+i) != '\0')
	{
		data_to_send[_cnt++] = *(str+i++);
		if(_cnt >= ANO_DT_TX_BUFFER_SIZE)
			break;
	}

	ANO_DT_FrameSend(_cnt);
}
/* 发送"字符串 + 数值"调试信息。 */
void ANO_DT_SendStrVal(const char *str, s32 val)
{
	u8 _cnt = ANO_DT_FrameStart(SWJADDR, ANO_MSG_STRVAL);
	data_to_send[_cnt++]=BYTE3(val);
	data_to_send[_cnt++]=BYTE2(val);
	data_to_send[_cnt++]=BYTE1(val);
	data_to_send[_cnt++]=BYTE0(val);
	u8 i = 0;
	while(*(str+i) != '\0')
	{
		data_to_send[_cnt++] = *(str+i++);
		if(_cnt >= ANO_DT_TX_BUFFER_SIZE)
			break;
	}

	ANO_DT_FrameSend(_cnt);
}

/* 发送外设工作状态概览。 */
void ANO_DT_SendSensorSta(u8 of_sta,u8 gps_sta,u8 opmv_sta,u8 uwb_sta,u8 altadd_sta)
{
	u8 _cnt = ANO_DT_FrameStart(SWJADDR, ANO_MSG_SENSOR_STA);

	data_to_send[_cnt++]=of_sta;
	data_to_send[_cnt++]=gps_sta;
	data_to_send[_cnt++]=opmv_sta;
	data_to_send[_cnt++]=uwb_sta;
	data_to_send[_cnt++]=altadd_sta;

	ANO_DT_FrameSend(_cnt);
}

/* 发送 OpenMV 色块跟踪信息。 */
void ANO_DT_SendOmvCt(u8 color, u8 sta, s16 x, s16 y, u8 d_tim)
{
	u8 _cnt = ANO_DT_FrameStart(SWJADDR, ANO_MSG_OMV_CT);

	data_to_send[_cnt++]=color;
	data_to_send[_cnt++]=sta;
	data_to_send[_cnt++]=BYTE1(x);
	data_to_send[_cnt++]=BYTE0(x);
	data_to_send[_cnt++]=BYTE1(y);
	data_to_send[_cnt++]=BYTE0(y);
	data_to_send[_cnt++]=d_tim;

	ANO_DT_FrameSend(_cnt);
}

/* 发送 OpenMV 巡线信息。 */
void ANO_DT_SendOmvLt(u8 sta, s16 angle, s16 offset, u8 pflag, s16 x, s16 y, u8 d_tim)
{
	u8 _cnt = ANO_DT_FrameStart(SWJADDR, ANO_MSG_OMV_LT);

	data_to_send[_cnt++]=sta;
	data_to_send[_cnt++]=BYTE1(angle);
	data_to_send[_cnt++]=BYTE0(angle);
	data_to_send[_cnt++]=BYTE1(offset);
	data_to_send[_cnt++]=BYTE0(offset);
	data_to_send[_cnt++]=pflag;
	data_to_send[_cnt++]=BYTE1(x);
	data_to_send[_cnt++]=BYTE0(x);
	data_to_send[_cnt++]=BYTE1(y);
	data_to_send[_cnt++]=BYTE0(y);
	data_to_send[_cnt++]=d_tim;

	ANO_DT_FrameSend(_cnt);
}


/*
 * 功能：发送用户自定义调试数据。
 * 说明：当前内容服务于 OpenMV 相关调试，字段顺序需与上位机显示保持一致。
 */
void ANO_DT_Send_User()
{
	u8 _cnt = ANO_DT_FrameStart(SWJADDR, ANO_MSG_USER);
	s16 _temp;

	_temp = (s16)(ano_opmv_cbt_ctrl.opmv_pos[1] );
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);

	_temp = (s16)(ano_opmv_cbt_ctrl.decou_pos_pixel[1] );
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);

	_temp = (s16)(ano_opmv_cbt_ctrl.ground_pos_err_h_cm[1] );
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);

	_temp = (s16)(ano_opmv_cbt_ctrl.ground_pos_err_d_h_cmps[1] );
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);

	_temp = (s16)(ano_opmv_cbt_ctrl.target_gnd_velocity_cmps[1]);
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (s16)(ano_opmv_cbt_ctrl.exp_velocity_h_cmps[1]);
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);

	ANO_DT_FrameSend(_cnt);
}
