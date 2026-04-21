#include "Ano_OPMV_LineTracking_Ctrl.h"
#include "Ano_OPMV_CBTracking_Ctrl.h"
#include "Drv_OpenMV.h"
#include "Ano_OPMV_Ctrl.h"
#include "Ano_ProgramCtrl_User.h"
#include "Ano_FlightCtrl.h"
/* OpenMV ?????????????? cm? */
#define RELATIVE_HEIGHT_CM   (jsdata.valid_of_alt_cm)
/* OpenMV ????? */
_opmv_ct_sta_st opmv_ct_sta;
/*
 * ???OpenMV ?????
 * ???
 * 1. ???????????????????????? OpenMV ???????
 * 2. ?? OpenMV ??????????????????????
 * 3. ??????????????????????
 */
void ANO_OPMV_Ctrl_Task(u8 dT_ms)
{
    if(RELATIVE_HEIGHT_CM > 40)
    {
        /* ??????? 40cm?????????? */
        opmv_ct_sta.height_flag = 1;
    }
    if(flag.unlock_sta == 0)
    {
        /* ???????????? */
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
        /* ??????? */
        opmv_ct_sta.reset_flag = 0;
        ANO_CBTracking_Ctrl(&dT_ms, opmv_ct_sta.en);
        Program_Ctrl_User_Set_HXYcmps(ano_opmv_cbt_ctrl.exp_velocity_h_cmps[0], ano_opmv_cbt_ctrl.exp_velocity_h_cmps[1]);
    }
    else if(opmv.mode_sta == 2)
    {
        /* ????? */
        opmv_ct_sta.reset_flag = 0;
        ANO_LTracking_Ctrl(&dT_ms, opmv_ct_sta.en);
        Program_Ctrl_User_Set_HXYcmps(ano_opmv_lt_ctrl.exp_velocity_h_cmps[0], ano_opmv_lt_ctrl.exp_velocity_h_cmps[1]);
        Program_Ctrl_User_Set_YAWdps(ano_opmv_lt_ctrl.exp_yaw_pal_dps);
    }
    else
    {
        /* ????????????????????????? */
        if(opmv_ct_sta.reset_flag == 0)
        {
            opmv_ct_sta.reset_flag = 1;
            Program_Ctrl_User_Set_HXYcmps(0, 0);
            Program_Ctrl_User_Set_YAWdps(0);
        }
    }
}
