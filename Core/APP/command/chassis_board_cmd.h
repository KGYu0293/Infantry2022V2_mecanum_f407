#ifndef _CHASSIS_BOARD_CMD_H_
#define _CHASSIS_BOARD_CMD_H_

#include "bsp_log.h"
#include "pub_sub.h"
#include "robot_def.h"
#include "stdint.h"
// 外设
#include "BMI088.h"
#include "buzzer.h"
#include "can_pc.h"
#include "can_recv.h"
#include "can_send.h"
#include "referee.h"
typedef struct board_com_t {
    can_recv *recv;
    can_send *send;
    board_com_goci_data *goci_data;
    board_com_gico_data *gico_data;
} Board_com;

typedef struct Chassis_board_cmd_t {
    Robot_mode mode;

    Board_com board_com;
    // 外设
    buzzer *internal_buzzer;
    Referee *referee;

    Publisher *chassis_cmd_puber;
    Chassis_param chassis_param;  // 将要pub的变量定义在结构体中以长期保存(pub的是指针，要放在指针不会销毁的地方)
    Subscriber *chassis_upload_sub;

} chassis_board_cmd;

chassis_board_cmd *Chassis_board_CMD_Create(void);
void Chassis_board_CMD_Update(chassis_board_cmd *obj);

#endif