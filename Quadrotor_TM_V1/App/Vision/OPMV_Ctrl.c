#include "OPMV_LineTracking_Ctrl.h"
#include "OPMV_CBTracking_Ctrl.h"
#include "Drv_OpenMV.h"
#include "OPMV_Ctrl.h"
#include "ProgramCtrl_User.h"
#include "FlightCtrl.h"

/* OpenMV 控制启用的最低相对高度，单位 cm。 */
#define RELATIVE_HEIGHT_CM   (jsdata.valid_of_alt_cm)

/* OpenMV 控制状态。 */
_opmv_ct_sta_st opmv_ct_sta;

/*
 * 功能：OpenMV 控制总入口。
 * 说明：根据飞行状态选择色块跟踪或寻线控制，并在退出时清零输出。
 */
void ANO_OPMV_Ctrl_Task(u8 dT_ms)
{
    if(RELATIVE_HEIGHT_CM > 40)
    {
        /* 超过 40cm 后认为高度条件满足。 */
        opmv_ct_sta.height_flag = 1;
    }
    if(flag.unlock_sta == 0)
    {
        /* 未解锁时强制复位高度条件。 */
        opmv_ct_sta.height_flag = 0;
    }

    if(switchs.of_flow_on
        && switchs.opmv_on
        && opmv_ct_sta.height_flag != 0
        && flag.flight_mode2 == 2)
    {
        opmv_ct_sta.en = 1;
    }
    else
    {
        opmv_ct_sta.en = 0;
    }

    if(opmv.mode_sta == 1)
    {
        /* 色块跟踪模式。 */
        opmv_ct_sta.reset_flag = 0;
        ANO_CBTracking_Ctrl(&dT_ms, opmv_ct_sta.en);
        Program_Ctrl_User_Set_HXYcmps(ano_opmv_cbt_ctrl.exp_velocity_h_cmps[0], ano_opmv_cbt_ctrl.exp_velocity_h_cmps[1]);
    }
    else if(opmv.mode_sta == 2)
    {
        /* 寻线模式。 */
        opmv_ct_sta.reset_flag = 0;
        ANO_LTracking_Ctrl(&dT_ms, opmv_ct_sta.en);
        Program_Ctrl_User_Set_HXYcmps(ano_opmv_lt_ctrl.exp_velocity_h_cmps[0], ano_opmv_lt_ctrl.exp_velocity_h_cmps[1]);
        Program_Ctrl_User_Set_YAWdps(ano_opmv_lt_ctrl.exp_yaw_pal_dps);
    }
    else
    {
        /* 无有效视觉模式时，仅执行一次输出清零。 */
        if(opmv_ct_sta.reset_flag == 0)
        {
            opmv_ct_sta.reset_flag = 1;
            Program_Ctrl_User_Set_HXYcmps(0, 0);
            Program_Ctrl_User_Set_YAWdps(0);
        }
    }
}
