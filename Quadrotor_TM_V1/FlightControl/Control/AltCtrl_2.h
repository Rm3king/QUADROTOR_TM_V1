#ifndef __ALTCTRL_2_H__
#define __ALTCTRL_2_H__

/*
 * ???????????
 * ???????????????????????????
 * ?????????????????????
 */

#include "sysconfig.h"
#include "FcData.h"

typedef struct
{
	float target_vel_fixup_cmps;
	float target_acc_fixup_cmpss;

	float expect_hei;
	float expect_vel;
	float expect_acc;

	float feedback_hei;
	float feedback_vel;
	float feedback_acc;

	float e_hei;
	float e_vel;
	float e_acc;
	float ei_acc;

	float adj_b;
	float obs_twr;
	float out_u;

} _ano_alt_ctrl_st;

extern _ano_alt_ctrl_st alt_ctrl;

void ANO_TargetVal_FixUp(u8 *dT_ms, float target_alt_vel_cmps);
void ANO_Alt_Ctrl(u8 *dT_ms);
void ANO_AltCtrl_Task(u8 dT_ms);

#endif
