#ifndef _REFEREE_H_
#define _REFEREE_H_

#include "bsp_uart.h"
#include "circular_queue.h"
#include "cvector.h"
#include "monitor.h"
#include "referee_def.h"
#include "soft_crc.h"
#include "stdint.h"

typedef struct referee_config_t {
    uint8_t bsp_uart_index;
    lost_callback lost_callback;
} referee_config;

// 解析后的裁判系统数据
typedef struct referee_rx_data_t {
    Robot_id id;

    ext_game_state_t game_state;
    ext_game_result_t game_result;
    ext_game_robot_HP_t robot_HP;
    ext_dart_status_t dart_status;

    ext_event_data_t event;
    ext_supply_projectile_action_t supply_projectile_action;
    ext_referee_warning_t referee_warning;
    ext_dart_remaining_time_t dart_remaining_time;

    ext_game_robot_state_t game_robot_state;
    ext_power_heat_data_t power_heat;
    ext_game_robot_pos_t robot_pos;
    ext_buff_t buff;
    aerial_robot_energy_t aerial_robot_energy;
    ext_robot_hurt_t robot_hurt;
    ext_shoot_data_t shoot_data;
    ext_bullet_remaining_t bullet_remaining;
    ext_rfid_status_t rfid_status;
    ext_dart_cilent_cmd_t dart_cilent_cmd;
    ext_robot_command_t robot_command;
    ext_client_map_command_t client_map_command;
    //机器人间通信结构体
    ext_robot_interactive_data interactve_data;
} referee_rx_data;

// 解包辅助结构体
typedef enum referee_unpack_step_e { s_header_sof = 0,
                                     s_header_length,
                                     s_header_seq,
                                     s_header_crc8,
                                     s_cmd_id,
                                     s_data,
                                     s_crc16 } referee_unpack_step;
typedef struct referee_unpack_tool_t {
    referee_rx_pack rx_pack;
    referee_unpack_step step;
    uint16_t next_step_wait_len;
} referee_unpack_tool;

// 裁判系统外设结构体
typedef struct Referee_t {
    circular_queue *primary_data;
    referee_rx_data rx_data;

    referee_config config;
    monitor_item *monitor;
    referee_unpack_tool tool;
    uint8_t robot_status_received; //收到了机器人状态数据
} Referee;

void referee_driver_init(void);
//裁判系统接收Loop函数
void referee_recv_loop();
//发送机器人交互数据，包括自定义数据以及UI信息
void referee_send_ext(Referee *obj, ext_robot_interact_frame *frame);
Referee *referee_Create(referee_config *config);

#endif