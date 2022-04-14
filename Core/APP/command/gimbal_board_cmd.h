#ifndef _GIMBAL_BOARD_CMD_H_
#define _GIMBAL_BOARD_CMD_H_

#include "bsp_log.h"
#include "pub_sub.h"
#include "robot_def.h"
#include "stdint.h"
// 外设
#include "BMI088.h"
#include "buzzer.h"
#include "DT7_DR16.h"
#include "can_pc.h"
#include "can_recv.h"
#include "can_send.h"

typedef struct board_com_t_ {
    can_recv *recv;
    can_send *send;
    board_com_goci_data *goci_data;
    board_com_gico_data *gico_data;
} Board_com_;

typedef struct Gimbal_board_cmd_t {
    Board_com_ board_com;
    Robot_mode mode;
    uint8_t ready;
    // 外设
    buzzer *internal_buzzer;
    canpc *pc;
    dt7Remote *remote;

    Publisher *gimbal_cmd_puber;
    Gimbal_param gimbal_param;
    Publisher *shoot_cmd_puber;
    Shoot_param shoot_param;
    Subscriber *gimbal_upload_suber;
} gimbal_board_cmd;

gimbal_board_cmd *Gimbal_board_CMD_Create(void);
void Gimbal_board_CMD_Update(gimbal_board_cmd *obj);

#endif