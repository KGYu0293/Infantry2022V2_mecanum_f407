#ifndef _GIMBAL_BOARD_CMD_H_
#define _GIMBAL_BOARD_CMD_H_

#include "bsp_log.h"
#include "pub_sub.h"
#include "robot_def.h"
#include "stdint.h"
// 外设
#include "BMI088.h"
#include "DT7_DR16.h"
#include "buzzer.h"
#include "can_pc.h"
#include "can_recv.h"
#include "can_send.h"

typedef struct Gimbal_board_cmd_t {
    Robot_mode mode;
    AutoAim_mode autoaim_mode;
    // 标志量
    uint8_t robot_ready;
    // 板间通信
    can_recv *recv;
    can_send *send;
    Gimbal_board_send_data send_data;
    Chassis_board_send_data *recv_data;
    // 外设
    buzzer *internal_buzzer;
    canpc *pc;
    canpc_send pc_send_data;
    dt7Remote *remote;

    Publisher *shoot_cmd_puber;
    Cmd_shoot shoot_control;
    Publisher *gimbal_cmd_puber;
    Cmd_gimbal gimbal_control;
    Subscriber *gimbal_upload_suber;
    Upload_gimbal *gimbal_upload_data;
    
} gimbal_board_cmd;

gimbal_board_cmd *Gimbal_board_CMD_Create(void);
void Gimbal_board_CMD_Update(gimbal_board_cmd *obj);

#endif