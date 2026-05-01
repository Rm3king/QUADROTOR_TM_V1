/*
 * 文件名称: Drv_spl06.c
 * 所属模块: Driver / Sensor
 *
 * 功能描述:
 *   SPL06-001 气压计 SPI 驱动，提供气压高度测量：
 *   1) Drv_Spl0601Init()       -- 初始化：读取校准系数 + 配置采样率和精度
 *   2) Drv_Spl0601_Read()      -- 读取原始温度和气压 → 补偿计算 → 输出高度(cm)
 *   3) spl0601_get_temperature/pressure() -- 应用出厂校准系数的补偿公式
 *
 * 高度计算:
 *   气压 Pa → 国际气压高度公式 → 相对海平面高度(cm)
 *   h = 44330 * (1 - (P / P0)^0.19029)
 *
 * 数据流:
 *   SPI 读取 → 温度/气压补偿 → baro_pressure → 高度计算
 *   → FlightDataCal.c 高度融合
 *
 * 教学提示:
 *   - 气压计需要温度补偿，SPL06 内部校准系数存储在 0x10-0x21 寄存器
 *   - 气压高度精度约 ±50cm，需要与加速度计互补滤波提高动态精度
 *   - SEA_LEVEL_PRESSURE_PA (101400 Pa) 是标准大气压近似值
 */
#include "drv_spl06.h"
#include "Drv_spi.h"

#define SPL06_REG_PSR_B2     0x00
#define SPL06_REG_TMP_B2     0x03
#define SPL06_REG_PRS_CFG    0x06
#define SPL06_REG_TMP_CFG    0x07
#define SPL06_REG_MEAS_CFG   0x08
#define SPL06_REG_CFG        0x09
#define SPL06_REG_ID         0x0D
#define SPL06_REG_COEF_BASE  0x10

#define SEA_LEVEL_PRESSURE_PA 101400.0f
void Drv_SPL06CSPinInit ( void )
{
    ROM_SysCtlPeripheralEnable(SPL_CSPIN_SYSCTL);
	ROM_GPIOPinTypeGPIOOutput(SPL06_CS_PORT,SPL06_CS_PIN);
	ROM_GPIOPinWrite(SPL06_CS_PORT, SPL06_CS_PIN,SPL06_CS_PIN);
}
static void spl06_enable ( u8 ena )
{
    if(ena)
		ROM_GPIOPinWrite(SPL06_CS_PORT, SPL06_CS_PIN,0);
	else
		ROM_GPIOPinWrite(SPL06_CS_PORT, SPL06_CS_PIN,SPL06_CS_PIN);
}
static struct spl0601_t spl0601;
static struct spl0601_t *p_spl0601;
void spl0601_get_calib_param ( void );
/*****************************************************************************
 函 数 名  : spl0601_write
 功能描述  : I2C 寄存器写入子函数
 输入参数  : uint8 hwadr   硬件地址
             uint8 regadr  寄存器地址
             uint8 val     值
 输出参数  : 无
 返 回 值  :
 调用函数  :
 被调函数  :
 修改历史      :
  1.日    期   : 2015年11月30日
    作    者   : WL
    修改内容   : 新生成函数
*****************************************************************************/
static void spl0601_write ( unsigned char regadr, unsigned char val )
{
    spl06_enable ( 1 );
    Drv_Spi0SingleWirteAndRead ( regadr );
    Drv_Spi0SingleWirteAndRead ( val );
    spl06_enable ( 0 );
}
/*****************************************************************************
 函 数 名  : spl0601_read
 功能描述  : I2C 寄存器读取子函数
 输入参数  : uint8 hwadr   硬件地址
             uint8 regadr  寄存器地址
 输出参数  :
 返 回 值  : uint8 读出值
 调用函数  :
 被调函数  :
 修改历史      :
  1.日    期   : 2015年11月30日
    作    者   : WL
    修改内容   : 新生成函数
*****************************************************************************/
static u8 spl0601_read ( unsigned char regadr )
{
    u8 reg_data;
    spl06_enable ( 1 );
    Drv_Spi0SingleWirteAndRead ( regadr | 0x80 );
    reg_data = Drv_Spi0SingleWirteAndRead ( 0xff );
    spl06_enable ( 0 );
    return reg_data;
}
/*****************************************************************************
 函 数 名  : spl0601_rateset
 功能描述  :  设置温度传感器的每秒采样次数以及过采样率
 输入参数  : uint8 u8OverSmpl  过采样率         Maximal = 128
             uint8 u8SmplRate  每秒采样次数(Hz) Maximal = 128
             uint8 iSensor     0: Pressure; 1: Temperature
 输出参数  : 无
 返 回 值  : 无
 调用函数  :
 被调函数  :
 修改历史      :
  1.日    期   : 2015年11月24日
    作    者   : WL
    修改内容   : 新生成函数
*****************************************************************************/
void spl0601_rateset ( u8 iSensor, u8 u8SmplRate, u8 u8OverSmpl )
{
    u8 reg = 0;
    int32_t i32kPkT = 0;
    switch ( u8SmplRate )
    {
    case 2:
        reg |= ( 1 << 4 );
        break;
    case 4:
        reg |= ( 2 << 4 );
        break;
    case 8:
        reg |= ( 3 << 4 );
        break;
    case 16:
        reg |= ( 4 << 4 );
        break;
    case 32:
        reg |= ( 5 << 4 );
        break;
    case 64:
        reg |= ( 6 << 4 );
        break;
    case 128:
        reg |= ( 7 << 4 );
        break;
    case 1:
    default:
        break;
    }
    switch ( u8OverSmpl )
    {
    case 2:
        reg |= 1;
        i32kPkT = 1572864;
        break;
    case 4:
        reg |= 2;
        i32kPkT = 3670016;
        break;
    case 8:
        reg |= 3;
        i32kPkT = 7864320;
        break;
    case 16:
        i32kPkT = 253952;
        reg |= 4;
        break;
    case 32:
        i32kPkT = 516096;
        reg |= 5;
        break;
    case 64:
        i32kPkT = 1040384;
        reg |= 6;
        break;
    case 128:
        i32kPkT = 2088960;
        reg |= 7;
        break;
    case 1:
    default:
        i32kPkT = 524288;
        break;
    }
    if ( iSensor == 0 )
    {
        p_spl0601->i32kP = i32kPkT;
        spl0601_write ( SPL06_REG_PRS_CFG, reg );
        if ( u8OverSmpl > 8 )
        {
            reg = spl0601_read ( SPL06_REG_CFG );
            spl0601_write ( SPL06_REG_CFG, reg | 0x04 );
        }
    }
    if ( iSensor == 1 )
    {
        p_spl0601->i32kT = i32kPkT;
        spl0601_write ( SPL06_REG_TMP_CFG, reg | 0x80 ); //Using mems temperature
        if ( u8OverSmpl > 8 )
        {
            reg = spl0601_read ( SPL06_REG_CFG );
            spl0601_write ( SPL06_REG_CFG, reg | 0x08 );
        }
    }
}
/*****************************************************************************
 函 数 名  : spl0601_get_calib_param
 功能描述  : 获取校准参数
 输入参数  : void
 输出参数  : 无
 返 回 值  :
 调用函数  :
 被调函数  :
 修改历史      :
  1.日    期   : 2015年11月30日
    作    者   : WL
    修改内容   : 新生成函数
*****************************************************************************/
void spl0601_get_calib_param ( void )
{
    u32 h;
    u32 m;
    u32 l;
    h =  spl0601_read ( SPL06_REG_COEF_BASE + 0 );
    l  =  spl0601_read ( SPL06_REG_COEF_BASE + 1 );
    p_spl0601->calib_param.c0 = ( int16_t ) h << 4 | l >> 4;
    p_spl0601->calib_param.c0 = ( p_spl0601->calib_param.c0 & 0x0800 ) ? ( 0xF000 | p_spl0601->calib_param.c0 ) : p_spl0601->calib_param.c0;
    h =  spl0601_read ( SPL06_REG_COEF_BASE + 1 );
    l  =  spl0601_read ( SPL06_REG_COEF_BASE + 2 );
    p_spl0601->calib_param.c1 = ( int16_t ) ( h & 0x0F ) << 8 | l;
    p_spl0601->calib_param.c1 = ( p_spl0601->calib_param.c1 & 0x0800 ) ? ( 0xF000 | p_spl0601->calib_param.c1 ) : p_spl0601->calib_param.c1;
    h =  spl0601_read ( SPL06_REG_COEF_BASE + 3 );
    m =  spl0601_read ( SPL06_REG_COEF_BASE + 4 );
    l =  spl0601_read ( SPL06_REG_COEF_BASE + 5 );
    p_spl0601->calib_param.c00 = ( int32_t ) h << 12 | ( int32_t ) m << 4 | ( int32_t ) l >> 4;
    p_spl0601->calib_param.c00 = ( p_spl0601->calib_param.c00 & 0x080000 ) ? ( 0xFFF00000 | p_spl0601->calib_param.c00 ) : p_spl0601->calib_param.c00;
    h =  spl0601_read ( SPL06_REG_COEF_BASE + 5 );
    m =  spl0601_read ( SPL06_REG_COEF_BASE + 6 );
    l =  spl0601_read ( SPL06_REG_COEF_BASE + 7 );
    p_spl0601->calib_param.c10 = ( int32_t ) h << 16 | ( int32_t ) m << 8 | l;
    p_spl0601->calib_param.c10 = ( p_spl0601->calib_param.c10 & 0x080000 ) ? ( 0xFFF00000 | p_spl0601->calib_param.c10 ) : p_spl0601->calib_param.c10;
    h =  spl0601_read ( SPL06_REG_COEF_BASE + 8 );
    l  =  spl0601_read ( SPL06_REG_COEF_BASE + 9 );
    p_spl0601->calib_param.c01 = ( int16_t ) h << 8 | l;
    h =  spl0601_read ( SPL06_REG_COEF_BASE + 10 );
    l  =  spl0601_read ( SPL06_REG_COEF_BASE + 11 );
    p_spl0601->calib_param.c11 = ( int16_t ) h << 8 | l;
    h =  spl0601_read ( SPL06_REG_COEF_BASE + 12 );
    l  =  spl0601_read ( SPL06_REG_COEF_BASE + 13 );
    p_spl0601->calib_param.c20 = ( int16_t ) h << 8 | l;
    h =  spl0601_read ( SPL06_REG_COEF_BASE + 14 );
    l  =  spl0601_read ( SPL06_REG_COEF_BASE + 15 );
    p_spl0601->calib_param.c21 = ( int16_t ) h << 8 | l;
    h =  spl0601_read ( SPL06_REG_COEF_BASE + 16 );
    l  =  spl0601_read ( SPL06_REG_COEF_BASE + 17 );
    p_spl0601->calib_param.c30 = ( int16_t ) h << 8 | l;
}
/*****************************************************************************
 函 数 名  : spl0601_start_temperature
 功能描述  : 发起一次温度测量
 输入参数  : void
 输出参数  : 无
 返 回 值  :
 调用函数  :
 被调函数  :
 修改历史      :
  1.日    期   : 2015年11月30日
    作    者   : WL
    修改内容   : 新生成函数
*****************************************************************************/
void spl0601_start_temperature ( void )
{
    spl0601_write ( SPL06_REG_MEAS_CFG, 0x02 );
}
/* 发起一次压力值测量 */
void spl0601_start_pressure ( void )
{
    spl0601_write ( SPL06_REG_MEAS_CFG, 0x01 );
}
/*****************************************************************************
 函 数 名  : spl0601_start_continuous
 功能描述  : Select node for the continuously measurement
 输入参数  : uint8 mode  1: pressure; 2: temperature; 3: pressure and temperature
 输出参数  : 无
 返 回 值  :
 调用函数  :
 被调函数  :
 修改历史      :
  1.日    期   : 2015年11月25日
    作    者   : WL
    修改内容   : 新生成函数
*****************************************************************************/
void spl0601_start_continuous ( u8 mode )
{
    spl0601_write ( SPL06_REG_MEAS_CFG, mode + 4 );
}
/*****************************************************************************
 函 数 名  : spl0601_get_raw_temp
 功能描述  : 获取温度的原始值，并转换成32Bits整数
 输入参数  : void
 输出参数  : 无
 返 回 值  :
 调用函数  :
 被调函数  :
 修改历史      :
  1.日    期   : 2015年11月30日
    作    者   : WL
    修改内容   : 新生成函数
*****************************************************************************/
void spl0601_get_raw_temp ( void )
{
    u8 h[3] = {0};
    h[0] = spl0601_read ( SPL06_REG_TMP_B2 + 0 );
    h[1] = spl0601_read ( SPL06_REG_TMP_B2 + 1 );
    h[2] = spl0601_read ( SPL06_REG_TMP_B2 + 2 );
    p_spl0601->i32rawTemperature = ( int32_t ) h[0] << 16 | ( int32_t ) h[1] << 8 | ( int32_t ) h[2];
    p_spl0601->i32rawTemperature = ( p_spl0601->i32rawTemperature & 0x800000 ) ? ( 0xFF000000 | p_spl0601->i32rawTemperature ) : p_spl0601->i32rawTemperature;
}
/*****************************************************************************
 函 数 名  : spl0601_get_raw_pressure
 功能描述  : 获取压力原始值，并转换成32bits整数
 输入参数  : void
 输出参数  : 无
 返 回 值  :
 调用函数  :
 被调函数  :
 修改历史      :
  1.日    期   : 2015年11月30日
    作    者   : WL
    修改内容   : 新生成函数
*****************************************************************************/
void spl0601_get_raw_pressure ( void )
{
    u8 h[3];
    h[0] = spl0601_read ( SPL06_REG_PSR_B2 + 0 );
    h[1] = spl0601_read ( SPL06_REG_PSR_B2 + 1 );
    h[2] = spl0601_read ( SPL06_REG_PSR_B2 + 2 );
    p_spl0601->i32rawPressure = ( int32_t ) h[0] << 16 | ( int32_t ) h[1] << 8 | ( int32_t ) h[2];
    p_spl0601->i32rawPressure = ( p_spl0601->i32rawPressure & 0x800000 ) ? ( 0xFF000000 | p_spl0601->i32rawPressure ) : p_spl0601->i32rawPressure;
}
/*****************************************************************************
 函 数 名  : spl0601_init
 功能描述  : SPL06-01 初始化函数
 输入参数  : void
 输出参数  : 无
 返 回 值  :
 调用函数  :
 被调函数  :
 修改历史      :
  1.日    期   : 2015年11月30日
    作    者   : WL
    修改内容   : 新生成函数
*****************************************************************************/
u8 Drv_Spl0601Init ( void )
{
    p_spl0601 = &spl0601;
    p_spl0601->i32rawPressure = 0;
    p_spl0601->i32rawTemperature = 0;
    p_spl0601->chip_id = spl0601_read ( SPL06_REG_ID );
	
    spl0601_get_calib_param();
    spl0601_rateset ( PRESSURE_SENSOR, 128, 16 );
    spl0601_rateset ( TEMPERATURE_SENSOR, 8, 8 );
    spl0601_start_continuous ( CONTINUOUS_P_AND_T );
	
	if(p_spl0601->chip_id == 0x10)
	{
		return 1;
	}
	else
	{
		return 0;
	}
}
/*****************************************************************************
 函 数 名  : spl0601_get_temperature
 功能描述  : 在获取原始值的基础上，返回浮点校准后的温度值
 输入参数  : void
 输出参数  : 无
 返 回 值  :
 调用函数  :
 被调函数  :
 修改历史      :
  1.日    期   : 2015年11月30日
    作    者   : WL
    修改内容   : 新生成函数
*****************************************************************************/
float spl0601_get_temperature ( void )
{
    float fTCompensate;
    float fTsc;
    fTsc = p_spl0601->i32rawTemperature / ( float ) p_spl0601->i32kT;
    fTCompensate =  p_spl0601->calib_param.c0 * 0.5 + p_spl0601->calib_param.c1 * fTsc;
    return fTCompensate;
}
/*****************************************************************************
 函 数 名  : spl0601_get_pressure
 功能描述  : 在获取原始值的基础上，返回浮点校准后的压力值
 输入参数  : void
 输出参数  : 无
 返 回 值  :
 调用函数  :
 被调函数  :
 修改历史      :
  1.日    期   : 2015年11月30日
    作    者   : WL
    修改内容   : 新生成函数
*****************************************************************************/
float spl0601_get_pressure ( void )
{
    float fTsc, fPsc;
    float qua2, qua3;
    float fPCompensate;
    fTsc = p_spl0601->i32rawTemperature / ( float ) p_spl0601->i32kT;
    fPsc = p_spl0601->i32rawPressure / ( float ) p_spl0601->i32kP;
    qua2 = p_spl0601->calib_param.c10 + fPsc * ( p_spl0601->calib_param.c20 + fPsc * p_spl0601->calib_param.c30 );
    qua3 = fTsc * fPsc * ( p_spl0601->calib_param.c11 + fPsc * p_spl0601->calib_param.c21 );
    //qua3 = 0.9f *fTsc * fPsc * (p_spl0601->calib_param.c11 + fPsc * p_spl0601->calib_param.c21);
    fPCompensate = p_spl0601->calib_param.c00 + fPsc * qua2 + fTsc * p_spl0601->calib_param.c01 + qua3;
    //fPCompensate = p_spl0601->calib_param.c00 + fPsc * qua2 + 0.9f *fTsc  * p_spl0601->calib_param.c01 + qua3;
    return fPCompensate;
}
static float baro_Offset, alt_3, height;
static float temperature, alt_high;
static float baro_pressure;
float Drv_Spl0601_Read ( void )
{
    spl0601_get_raw_temp();
    temperature = spl0601_get_temperature();
    spl0601_get_raw_pressure();
    baro_pressure = spl0601_get_pressure();
    /* 简化气压高度换算公式（三次多项式近似） */
    alt_3 = ( SEA_LEVEL_PRESSURE_PA - baro_pressure ) / 1000.0f;
    height = 0.82f * alt_3 * alt_3 * alt_3 + 0.09f * ( SEA_LEVEL_PRESSURE_PA - baro_pressure ) * 100.0f ;
    alt_high = ( height - baro_Offset ) ; //cm +
    return alt_high;
}
