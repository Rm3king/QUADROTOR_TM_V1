#include "FlyCtrl.h"
#include "DT.h"
#include "FcData.h"
#include "FlightCtrl.h"
#include "Imu.h"

/* 程控飞行状态。 */
_fly_ct_st program_ctrl;
/* 当前命令参数缓存。 */
static u16 val, spd;
/* 起飞命令边沿标志。 */
static u8 cmd_take_off_f;

#define FLYCTRL_AXIS_YAW 3
#define FLYCTRL_CMD_TAKE_OFF         0x01
#define FLYCTRL_CMD_LAND             0x02
#define FLYCTRL_CMD_GO_UP            0x03
#define FLYCTRL_CMD_GO_DOWN          0x04
#define FLYCTRL_CMD_GO_AHEAD         0x05
#define FLYCTRL_CMD_GO_BACK          0x06
#define FLYCTRL_CMD_GO_LEFT          0x07
#define FLYCTRL_CMD_GO_RIGHT         0x08
#define FLYCTRL_CMD_TURN_LEFT        0x09
#define FLYCTRL_CMD_TURN_RIGHT       0x0A
#define FLYCTRL_CMD_EMERGENCY_STOP   0xA0

/*
 * 功能：解析程控命令数据。
 * 说明：data[2] 为命令字，data[3:4] 为位移或角度，data[5:6] 为速度。
 */
void FlyCtrlDataAnl(u8 *data)
{
    val = ((*(data + 3)) << 8) + (*(data + 4));
    spd = ((*(data + 5)) << 8) + (*(data + 6));
    program_ctrl.cmd_state[0] = *(data + 2);
}

/*
 * 功能：执行程控命令。
 * 说明：保持原有命令切换条件、状态检查和执行顺序不变。
 */
void FlyCtrl_Task(u8 dT_ms)
{
    if(program_ctrl.cmd_state[0] != program_ctrl.cmd_state[1])
    {
        /* 命令切换时先清空上一条命令的过程状态。 */
        FlyCtrlReset();
        if(flag.rc_loss == 0 && flag.flight_mode == LOC_HOLD && (switchs.of_flow_on || switchs.gps_on))
        {
            program_ctrl.state_ok = 1;
        }
        else
        {
            program_ctrl.state_ok = 0;
            ANO_DT_SendString("FC State Error!");
            program_ctrl.cmd_state[0] = 0;
        }
    }

    switch(program_ctrl.cmd_state[0])
    {
        case FLYCTRL_CMD_TAKE_OFF:
        {
            if(program_ctrl.state_ok != 0)
            {
                if(flag.auto_take_off_land == AUTO_TAKE_OFF_NULL)
                {
                    if(cmd_take_off_f == 0)
                    {
                        cmd_take_off_f = 1;
                        ANO_DT_SendString("Take off!");
                        one_key_take_off();
                    }
                }
                else if(flag.auto_take_off_land == AUTO_TAKE_OFF_FINISH)
                {
                    ANO_DT_SendString("Take off OK!");
                    program_ctrl.cmd_state[0] = 0;
                }
                else if(flag.auto_take_off_land > AUTO_TAKE_OFF_FINISH)
                {
                    ANO_DT_SendString("CMD Error!");
                    program_ctrl.cmd_state[0] = 0;
                }
            }
        }
        break;

        case FLYCTRL_CMD_LAND:
        {
            if(flag.auto_take_off_land == AUTO_TAKE_OFF_FINISH)
            {
                ANO_DT_SendString("Landing!");
                one_key_land();
            }
            else if(flag.auto_take_off_land == AUTO_TAKE_OFF_NULL)
            {
                ANO_DT_SendString("Landing OK!");
                program_ctrl.cmd_state[0] = 0;
            }
        }
        break;

        case FLYCTRL_CMD_EMERGENCY_STOP:
        {
            if(flag.unlock_sta)
            {
                ANO_DT_SendString("Emergency stop OK!");
                flag.unlock_cmd = 0;
                program_ctrl.cmd_state[0] = 0;
            }
        }
        break;

        case FLYCTRL_CMD_GO_UP:
        {
            program_ctrl.vel_cmps_ref[Z] = spd;
            if(spd != 0)
            {
                program_ctrl.exp_process_t_ms[Z] = val * 1000 / LIMIT(spd, 0, fc_stv.vel_limit_z_p);
            }
            else
            {
                program_ctrl.exp_process_t_ms[Z] = 0;
            }
            if(program_ctrl.fb_process_t_ms[Z] == 0)
            {
                ANO_DT_SendString("Go up!");
            }
            else if(program_ctrl.exp_process_t_ms[Z] < program_ctrl.fb_process_t_ms[Z])
            {
                ANO_DT_SendString("Go up OK!");
                program_ctrl.cmd_state[0] = 0;
            }
            program_ctrl.fb_process_t_ms[Z] += dT_ms;
        }
        break;

        case FLYCTRL_CMD_GO_DOWN:
        {
            program_ctrl.vel_cmps_ref[Z] = -spd;
            if(spd != 0)
            {
                program_ctrl.exp_process_t_ms[Z] = val * 1000 / LIMIT(spd, 0, -fc_stv.vel_limit_z_n);
            }
            else
            {
                program_ctrl.exp_process_t_ms[Z] = 0;
            }
            if(program_ctrl.fb_process_t_ms[Z] == 0)
            {
                ANO_DT_SendString("Go down!");
            }
            else if(program_ctrl.exp_process_t_ms[Z] < program_ctrl.fb_process_t_ms[Z])
            {
                ANO_DT_SendString("Go down OK!");
                program_ctrl.cmd_state[0] = 0;
            }
            program_ctrl.fb_process_t_ms[Z] += dT_ms;
        }
        break;

        case FLYCTRL_CMD_GO_AHEAD:
        {
            program_ctrl.vel_cmps_ref[X] = spd;
            if(spd != 0)
            {
                program_ctrl.exp_process_t_ms[X] = val * 1000 / LIMIT(spd, 0, fc_stv.vel_limit_xy);
            }
            else
            {
                program_ctrl.exp_process_t_ms[X] = 0;
            }
            if(program_ctrl.fb_process_t_ms[X] == 0)
            {
                ANO_DT_SendString("Go ahead!");
            }
            else if(program_ctrl.exp_process_t_ms[X] < program_ctrl.fb_process_t_ms[X])
            {
                ANO_DT_SendString("Go ahead OK!");
                program_ctrl.cmd_state[0] = 0;
            }
            program_ctrl.fb_process_t_ms[X] += dT_ms;
        }
        break;

        case FLYCTRL_CMD_GO_BACK:
        {
            program_ctrl.vel_cmps_ref[X] = -spd;
            if(spd != 0)
            {
                program_ctrl.exp_process_t_ms[X] = val * 1000 / LIMIT(spd, 0, fc_stv.vel_limit_xy);
            }
            else
            {
                program_ctrl.exp_process_t_ms[X] = 0;
            }
            if(program_ctrl.fb_process_t_ms[X] == 0)
            {
                ANO_DT_SendString("Go back!");
            }
            else if(program_ctrl.exp_process_t_ms[X] < program_ctrl.fb_process_t_ms[X])
            {
                ANO_DT_SendString("Go back OK!");
                program_ctrl.cmd_state[0] = 0;
            }
            program_ctrl.fb_process_t_ms[X] += dT_ms;
        }
        break;

        case FLYCTRL_CMD_GO_LEFT:
        {
            program_ctrl.vel_cmps_ref[Y] = spd;
            if(spd != 0)
            {
                program_ctrl.exp_process_t_ms[Y] = val * 1000 / LIMIT(spd, 0, fc_stv.vel_limit_xy);
            }
            else
            {
                program_ctrl.exp_process_t_ms[Y] = 0;
            }
            if(program_ctrl.fb_process_t_ms[Y] == 0)
            {
                ANO_DT_SendString("Go left!");
            }
            else if(program_ctrl.exp_process_t_ms[Y] < program_ctrl.fb_process_t_ms[Y])
            {
                ANO_DT_SendString("Go left OK!");
                program_ctrl.cmd_state[0] = 0;
            }
            program_ctrl.fb_process_t_ms[Y] += dT_ms;
        }
        break;

        case FLYCTRL_CMD_GO_RIGHT:
        {
            program_ctrl.vel_cmps_ref[Y] = -spd;
            if(spd != 0)
            {
                program_ctrl.exp_process_t_ms[Y] = val * 1000 / LIMIT(spd, 0, fc_stv.vel_limit_xy);
            }
            else
            {
                program_ctrl.exp_process_t_ms[Y] = 0;
            }
            if(program_ctrl.fb_process_t_ms[Y] == 0)
            {
                ANO_DT_SendString("Go right!");
            }
            else if(program_ctrl.exp_process_t_ms[Y] < program_ctrl.fb_process_t_ms[Y])
            {
                ANO_DT_SendString("Go right OK!");
                program_ctrl.cmd_state[0] = 0;
            }
            program_ctrl.fb_process_t_ms[Y] += dT_ms;
        }
        break;

        case FLYCTRL_CMD_TURN_LEFT:
        {
            program_ctrl.yaw_pal_dps = spd;
            if(spd != 0)
            {
                program_ctrl.exp_process_t_ms[FLYCTRL_AXIS_YAW] = val * 1000 / LIMIT(spd, 0, fc_stv.yaw_pal_limit);
            }
            else
            {
                program_ctrl.exp_process_t_ms[FLYCTRL_AXIS_YAW] = 0;
            }
            if(program_ctrl.fb_process_t_ms[FLYCTRL_AXIS_YAW] == 0)
            {
                ANO_DT_SendString("Turn left!");
            }
            else if(program_ctrl.exp_process_t_ms[FLYCTRL_AXIS_YAW] < program_ctrl.fb_process_t_ms[FLYCTRL_AXIS_YAW])
            {
                ANO_DT_SendString("Turn left OK!");
                program_ctrl.cmd_state[0] = 0;
            }
            program_ctrl.fb_process_t_ms[FLYCTRL_AXIS_YAW] += dT_ms;
        }
        break;

        case FLYCTRL_CMD_TURN_RIGHT:
        {
            program_ctrl.yaw_pal_dps = -spd;
            if(spd != 0)
            {
                program_ctrl.exp_process_t_ms[FLYCTRL_AXIS_YAW] = val * 1000 / LIMIT(spd, 0, fc_stv.yaw_pal_limit);
            }
            else
            {
                program_ctrl.exp_process_t_ms[FLYCTRL_AXIS_YAW] = 0;
            }
            if(program_ctrl.fb_process_t_ms[FLYCTRL_AXIS_YAW] == 0)
            {
                ANO_DT_SendString("Turn right!");
            }
            else if(program_ctrl.exp_process_t_ms[FLYCTRL_AXIS_YAW] < program_ctrl.fb_process_t_ms[FLYCTRL_AXIS_YAW])
            {
                ANO_DT_SendString("Turn right OK!");
                program_ctrl.cmd_state[0] = 0;
            }
            program_ctrl.fb_process_t_ms[FLYCTRL_AXIS_YAW] += dT_ms;
        }
        break;

        default:
        {
        }
        break;
    }

    /* 命令结束后立即复位过程量。 */
    if(program_ctrl.cmd_state[0] == 0)
    {
        FlyCtrlReset();
    }

    /* 保存上一拍命令状态。 */
    program_ctrl.cmd_state[1] = program_ctrl.cmd_state[0];

    /*
     * 飞行器解锁后再进行坐标系变换。
     * 未解锁时仅刷新参考方向，避免残留旧指令。
     */
    if(flag.unlock_sta != 0)
    {
        h2w_2d_trans(program_ctrl.vel_cmps_ref, program_ctrl.ref_dir, program_ctrl.vel_cmps_w);
        w2h_2d_trans(program_ctrl.vel_cmps_w, imu_data.hx_vec, program_ctrl.vel_cmps_h);
        program_ctrl.vel_cmps_h[Z] = program_ctrl.vel_cmps_w[Z] = program_ctrl.vel_cmps_ref[Z];
    }
    else
    {
        program_ctrl.ref_dir[X] = imu_data.hx_vec[X];
        program_ctrl.ref_dir[Y] = imu_data.hx_vec[Y];
    }
}

/*
 * 功能：复位程控控制量。
 * 说明：清空速度、时间反馈和起飞边沿标志。
 */
void FlyCtrlReset(void)
{
    u8 i;
    cmd_take_off_f = 0;
    for(i = 0; i < 4; i++)
    {
        if(i < 3)
        {
            program_ctrl.vel_cmps_ref[i] = 0;
            program_ctrl.vel_cmps_w[i] = 0;
            program_ctrl.vel_cmps_h[i] = 0;
        }
        else
        {
            program_ctrl.yaw_pal_dps = 0;
        }
        program_ctrl.exp_process_t_ms[i] = 0;
        program_ctrl.fb_process_t_ms[i] = 0;
    }
}
