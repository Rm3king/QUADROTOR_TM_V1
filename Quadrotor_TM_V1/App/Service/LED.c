#include "LED.h"
#include "Math.h"
#include "Drv_led.h"
#include "FcData.h"
#include "OPMV_CBTracking_Ctrl.h"
#include "OPMV_LineTracking_Ctrl.h"
/*
 * 模块说明。
 * LED 状态显示任务。
 * 负责维护软件 PWM 亮度输出，并根据飞控状态切换不同的指示灯模式。
 */
/* PWM 精度，LED_1ms_DRV() 以该值作为亮度分辨率。 */
u16 led_accuracy = 20;
/* LED 亮度缓存，顺序为 X、B、R、G。 */
float LED_Brightness[4] = {0, 20, 0, 0};
/* LED 状态机。 */
_led_sta LED_STA;

/*
 * 功能：1ms LED 驱动任务。
 * 说明：按照亮度缓存执行软件 PWM，占空比分辨率由 led_accuracy 决定。
 */
void LED_1ms_DRV(void)
{
    static u16 led_cnt[4];
    u8 i;
    for(i = 0; i < 4; i++)
    {
        if(led_cnt[i] < LED_Brightness[i])
        {
            switch(i)
            {
                case 2:
                    Drv_LedOnOff(LED_R, 1);
                break;
                case 3:
                    Drv_LedOnOff(LED_G, 1);
                break;
                case 1:
                    Drv_LedOnOff(LED_B, 1);
                break;
                case 0:
                    Drv_LedOnOff(LED_S, 1);
                break;
            }
        }
        else
        {
            switch(i)
            {
                case 2:
                    Drv_LedOnOff(LED_R, 0);
                break;
                case 3:
                    Drv_LedOnOff(LED_G, 0);
                break;
                case 1:
                    Drv_LedOnOff(LED_B, 0);
                break;
                case 0:
                    Drv_LedOnOff(LED_S, 0);
                break;
            }
        }
        if(++led_cnt[i] >= led_accuracy)
        {
            led_cnt[i] = 0;
        }
    }
}

/*
 * 功能：直接设置 LED 开关状态。
 * 说明：根据位图参数把目标 LED 设为满亮，其余熄灭。
 */
static void ledOnOff(u8 led)
{
    u8 i;
    for(i = 0; i < LED_NUM; i++)
    {
        if(led & (1 << i))
        {
            LED_Brightness[i] = 20;
        }
        else
        {
            LED_Brightness[i] = 0;
        }
    }
}

/*
 * 功能：执行呼吸灯效果。
 * 说明：T 为完整呼吸周期，单位 ms。
 */
static void ledBreath(u8 dT_ms, u8 led, u16 T)
{
    static u8 dir[LED_NUM];
    u8 i;
    for(i = 0; i < LED_NUM; i++)
    {
        if(led & (1 << i))
        {
            switch(dir[i])
            {
                case 0:
                    LED_Brightness[i] += safe_div(led_accuracy, ((float)T / dT_ms), 0);
                    if(LED_Brightness[i] > 20)
                    {
                        dir[i] = 1;
                    }
                break;
                case 1:
                    LED_Brightness[i] -= safe_div(led_accuracy, ((float)T / dT_ms), 0);
                    if(LED_Brightness[i] < 0)
                    {
                        dir[i] = 0;
                    }
                break;
                default:
                    dir[i] = 0;
                break;
            }
        }
        else
        {
            LED_Brightness[i] = 0;
        }
    }
}

/*
 * 功能：执行闪烁效果。
 * 说明：on_ms 为亮灯时间，off_ms 为熄灯时间，单位 ms。
 */
static void ledFlash(u8 dT_ms, u8 led, u16 on_ms, u16 off_ms)
{
    static u16 tim_tmp;
    if(tim_tmp < on_ms)
    {
        ledOnOff(led);
    }
    else
    {
        ledOnOff(0);
    }
    tim_tmp += dT_ms;
    if(tim_tmp >= (on_ms + off_ms))
    {
        tim_tmp = 0;
    }
}

/*
 * 功能：LED 状态机任务。
 * 说明：在 11ms 周期内根据校准、告警、飞行模式和外设状态切换显示效果。
 */
void LED_Task2(u8 dT_ms)
{
    static u16 timtmp = 0;
    if(LED_STA.errOneTime)
    {
        /* 一次性错误提示，持续显示 3 秒。 */
        ledOnOff(BIT_RLED);
        timtmp += dT_ms;
        if(timtmp > 3000)
        {
            timtmp = 0;
            LED_STA.errOneTime = 0;
        }
    }
    else if(LED_STA.errMpu > 0 || LED_STA.errMag > 0 || LED_STA.errBaro > 0)
    {
        u8 flashtims;
        static u8 cnttmp = 0;
        /* 通过闪烁次数区分不同传感器错误。 */
        if(LED_STA.errMpu > 0)
        {
            flashtims = 2;
        }
        else if(LED_STA.errMag > 0)
        {
            flashtims = 3;
        }
        else
        {
            flashtims = 4;
        }
        if(cnttmp < flashtims)
        {
            if(timtmp < 60)
            {
                ledOnOff(BIT_BLED);
            }
            else
            {
                ledOnOff(0);
            }
            timtmp += dT_ms;
            if(timtmp > 200)
            {
                timtmp = 0;
                cnttmp++;
            }
        }
        else
        {
            timtmp += dT_ms;
            if(timtmp > 1000)
            {
                timtmp = 0;
                cnttmp = 0;
            }
        }
    }
    else if(LED_STA.saving)
    {
        /* 参数保存中，常亮绿色。 */
        LED_Brightness[G_led] = 20;
        LED_Brightness[R_led] = 0;
        LED_Brightness[B_led] = 0;
    }
    else if(LED_STA.calAcc || LED_STA.calGyr || LED_STA.rst_imu)
    {
        /* IMU 校准或复位过程中，蓝灯快闪。 */
        ledFlash(dT_ms, BIT_BLED, 40, 40);
    }
    else if(LED_STA.calMag)
    {
        /* 磁力计校准不同阶段使用不同灯效。 */
        if(LED_STA.calMag == 1)
        {
            ledBreath(dT_ms, BIT_GLED, 300);
        }
        else if(LED_STA.calMag == 2)
        {
            ledFlash(dT_ms, BIT_BLED, 40, 40);
        }
        else if(LED_STA.calMag == 100)
        {
            ledFlash(dT_ms, BIT_GLED, 40, 40);
        }
        else
        {
            ledBreath(dT_ms, BIT_BLED, 300);
        }
    }
    else if(LED_STA.noRc)
    {
        /* 遥控丢失时红灯呼吸。 */
        ledBreath(dT_ms, BIT_RLED, 500);
    }
    else if(LED_STA.lowVt)
    {
        /* 低电压告警时红灯快闪。 */
        ledFlash(dT_ms, BIT_RLED, 60, 60);
    }
    else
    {
        static u8 statmp = 0;
        static u8 modtmp = 0;
        /* 正常状态下分阶段轮询显示飞行模式和外设状态。 */
        if(statmp == 0)
        {
            /* 第一阶段：按次数显示飞行模式编号。 */
            if(modtmp <= flag.flight_mode)
            {
                if(timtmp < 60)
                {
                    if(flag.unlock_sta)
                    {
                        ledOnOff(BIT_GLED);
                    }
                    else
                    {
                        ledOnOff(BIT_WLED);
                    }
                }
                else
                {
                    ledOnOff(0);
                }
                timtmp += dT_ms;
                if(timtmp > 200)
                {
                    timtmp = 0;
                    modtmp++;
                }
            }
            else
            {
                modtmp = 0;
                statmp = 1;
            }
        }
        else if(statmp == 1)
        {
            /* 第二阶段：依次显示 GPS、光流和 OpenMV 状态。 */
            if(modtmp == 0)
            {
                if(switchs.gps_on)
                {
                    if(timtmp < 60)
                    {
                        ledOnOff(BIT_BLED);
                    }
                    else
                    {
                        ledOnOff(0);
                    }
                    timtmp += dT_ms;
                    if(timtmp > 200)
                    {
                        timtmp = 0;
                        modtmp++;
                    }
                }
                else
                {
                    modtmp = 1;
                }
            }
            else if(modtmp == 1)
            {
                if(switchs.of_flow_on > 0 && switchs.of_tof_on > 0)
                {
                    if(timtmp < 60)
                    {
                        ledOnOff(BIT_YLED);
                    }
                    else
                    {
                        ledOnOff(0);
                    }
                    timtmp += dT_ms;
                    if(timtmp > 200)
                    {
                        timtmp = 0;
                        modtmp++;
                    }
                }
                else
                {
                    modtmp = 2;
                }
            }
            else if(modtmp == 2)
            {
                if(switchs.opmv_on && (ano_opmv_cbt_ctrl.target_loss == 0 || ano_opmv_lt_ctrl.target_loss == 0))
                {
                    if(timtmp < 60)
                    {
                        ledOnOff(BIT_PLED);
                    }
                    else
                    {
                        ledOnOff(0);
                    }
                    timtmp += dT_ms;
                    if(timtmp > 200)
                    {
                        timtmp = 0;
                        modtmp++;
                    }
                }
                else
                {
                    modtmp = 3;
                }
            }
            if(modtmp == 3)
            {
                statmp = 2;
                modtmp = 0;
            }
        }
        else
        {
            /* 第三阶段：全部熄灭一段时间，再重新开始轮询。 */
            ledOnOff(0);
            timtmp += dT_ms;
            if(timtmp > 1000)
            {
                timtmp = 0;
                statmp = 0;
                modtmp = 0;
            }
        }
    }
}
