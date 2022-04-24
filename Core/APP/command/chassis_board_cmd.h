#ifndef _CHASSIS_BOARD_CMD_H_
#define _CHASSIS_BOARD_CMD_H_

#include "pub_sub.h"
#include "robot_def.h"
#include "stdint.h"
// 调试
#include "bsp_log.h"
// 外设
#include "BMI088.h"
#include "buzzer.h"
#include "can_pc.h"
#include "can_recv.h"
#include "can_send.h"
#include "referee.h"

typedef struct Chassis_board_cmd_t {
    // 机器人状态
    Robot_mode mode;
    // 板间通信
    can_recv *recv;
    can_send *send;
    chassis_board_send_data send_data;
    gimbal_board_send_data *recv_data;
    // 外设
    buzzer *internal_buzzer;
    Referee *referee;

    Publisher *chassis_cmd_puber;
    Cmd_chassis chassis_param;  // 将要pub的变量定义在结构体中以长期保存(pub的是指针，要放在指针不会销毁的地方)
    Subscriber *chassis_upload_sub;

} chassis_board_cmd;

chassis_board_cmd *Chassis_board_CMD_Create(void);
void Chassis_board_CMD_Update(chassis_board_cmd *obj);

#endif