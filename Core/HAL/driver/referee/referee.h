#ifndef _REFEREE_H_
#define _REFEREE_H_

#include "cvector.h"
#include "monitor.h"
#include "stdint.h"

#define REFEREE_RX_MIN_SIZE 9
#define REFEREE_RX_MAX_SIZE 128 + REFEREE_RX_MIN_SIZE

typedef enum Robot_id_e {
    RED_HERO = 1,
    RED_ENGINEER = 2,
    RED_STANDARD_1 = 3,
    RED_STANDARD_2 = 4,
    RED_STANDARD_3 = 5,
    RED_AERIAL = 6,
    RED_SENTRY = 7,
    BLUE_HERO = 101,
    BLUE_ENGINEER = 102,
    BLUE_STANDARD_1 = 103,
    BLUE_STANDARD_2 = 104,
    BLUE_STANDARD_3 = 105,
    BLUE_AERIAL = 106,
    BLUE_SENTRY = 107,
} Robot_id;
typedef enum {
    PROGRESS_UNSTART = 0,
    PROGRESS_PREPARE = 1,
    PROGRESS_SELFCHECK = 2,
    PROGRESS_5sCOUNTDOWN = 3,
    PROGRESS_BATTLE = 4,
    PROGRESS_CALCULATING = 5,
} game_progress_e;

typedef struct referee_config_t {
    uint8_t bsp_uart_index;
    lost_callback lost_callback;
} referee_config;

// 成对使用，用以代替特有的typedef __packed struct写法
#pragma pack(1)

//比赛信息部分
// 0x0001
typedef struct {
    uint8_t game_type : 4;
    uint8_t game_progress : 4;
    uint16_t stage_remain_time;

    uint64_t SyncTimeStamp;
} ext_game_state_t;
// 0x0002
typedef struct {
    uint8_t winner;
} ext_game_result_t;
// 0x0003
typedef struct {
    uint16_t red_1_robot_HP;
    uint16_t red_2_robot_HP;
    uint16_t red_3_robot_HP;
    uint16_t red_4_robot_HP;
    uint16_t red_5_robot_HP;
    uint16_t red_7_robot_HP;
    uint16_t red_outpost_HP;
    uint16_t red_base_HP;
    uint16_t blue_1_robot_HP;
    uint16_t blue_2_robot_HP;
    uint16_t blue_3_robot_HP;
    uint16_t blue_4_robot_HP;
    uint16_t blue_5_robot_HP;
    uint16_t blue_7_robot_HP;
    uint16_t blue_outpost_HP;
    uint16_t blue_base_HP;
} ext_game_robot_HP_t;
// 0x0004
typedef struct {
    uint8_t dart_belong;
    uint16_t stage_remaining_time;
} ext_dart_status_t;
// 0x0005

//事件信息部分
// 0x0101
typedef struct {
    uint32_t event_type;
} ext_event_data_t;
// 0x0102
typedef struct {
    uint8_t supply_projectile_id;
    uint8_t supply_robot_id;
    uint8_t supply_projectile_step;
    uint8_t supply_projectile_num;
} ext_supply_projectile_action_t;
// 0x0103

// 0x0104
typedef struct {
    uint8_t level;
    uint8_t foul_robot_id;
} ext_referee_warning_t;
// 0x0105
typedef struct {
    uint8_t dart_remaining_time;
} ext_dart_remaining_time_t;

//机器人信息部分
// 0x0201
typedef struct {
    uint8_t robot_id;
    uint8_t robot_level;
    uint16_t remain_HP;
    uint16_t max_HP;

    uint16_t shooter_id1_17mm_cooling_rate;
    uint16_t shooter_id1_17mm_cooling_limit;
    uint16_t shooter_id1_17mm_speed_limit;
    uint16_t shooter_id2_17mm_cooling_rate;
    uint16_t shooter_id2_17mm_cooling_limit;
    uint16_t shooter_id2_17mm_speed_limit;
    uint16_t shooter_id1_42mm_cooling_rate;
    uint16_t shooter_id1_42mm_cooling_limit;
    uint16_t shooter_id1_42mm_speed_limit;

    uint16_t chassis_power_limit;
    uint8_t mains_power_gimbal_output : 1;
    uint8_t mains_power_chassis_output : 1;
    uint8_t mains_power_shooter_output : 1;
} ext_game_robot_state_t;
// 0x0202
typedef struct {
    uint16_t chassis_volt;
    uint16_t chassis_current;
    float chassis_power;
    uint16_t chassis_power_buffer;
    uint16_t shooter_id1_17mm_cooling_heat;
    uint16_t shooter_id2_17mm_cooling_heat;
    uint16_t shooter_id1_42mm_cooling_heat;
} ext_power_heat_data_t;
// 0x0203
typedef struct {
    float x;
    float y;
    float z;
    float yaw;
} ext_game_robot_pos_t;
// 0x0204
typedef struct {
    uint8_t power_rune_buff;
} ext_buff_t;
// 0x0205
typedef struct {
    uint8_t attack_time;
} aerial_robot_energy_t;
// 0x0206
typedef struct {
    uint8_t armor_type : 4;
    uint8_t hurt_type : 4;
} ext_robot_hurt_t;
// 0x0207
typedef struct {
    uint8_t bullet_type;
    uint8_t shooter_id;
    uint8_t bullet_freq;
    float bullet_speed;
} ext_shoot_data_t;
// 0x0208
typedef struct {
    uint16_t bullet_remaining_num_17mm;
    uint16_t bullet_remaining_num_42mm;
    uint16_t coin_remaining_num;
} ext_bullet_remaining_t;
// 0x0209
typedef struct {
    uint32_t rfid_status;
} ext_rfid_status_t;
// 0x020A
typedef struct {
    uint8_t dart_launch_opening_status;
    uint8_t dart_attack_target;
    uint16_t target_change_time;
    uint8_t first_dart_speed;
    uint8_t second_dart_speed;
    uint8_t third_dart_speed;
    uint8_t fourth_dart_speed;
    uint16_t last_dart_launch_time;
    uint16_t operate_launch_cmd_time;
} ext_dart_cilent_cmd_t;

//交互部分

/*机器人ID：
1.英雄R 2.工程R 3/4/5.步兵R 6.空中R 7.哨兵R 9.雷达站R
101.英雄B 102.工程B 103/104/105.步兵B 106.空中B 107.哨兵B 109.雷达站B
客户端ID：
0x0101.英雄操作手客户端R
0x0102.工程操作手客户端R
0x0103/0x0104/0x0105.步兵操作手客户端R
0x0106.空中操作手客户端R
0x0165.英雄操作手客户端B
0x0166.工程操作手客户端B
0x0167/0x0168/0x0169.步兵操作手客户端步兵B
0x016A.空中操作手客户端B
学生机器人间通信 cmd_id 0x0301，内容ID:0x0200~0x02FF
*/
// 0x0301
typedef struct {
    uint16_t data_cmd_id;
    uint16_t sender_ID;
    uint16_t receiver_ID;
} ext_student_interactive_header_data_t;
//机器人间通信数据 可按照官方协议自定义
typedef struct {
    // uint8_t data[112];
    uint8_t sentry_chassis_state;
    uint8_t sentry_gimbal_state;
    uint8_t sentry_fire_state;
} ext_robot_interactive_data_t;

// 0x0302 自定义控制器
// 0x0303 小地图交互数据
typedef struct {
    float target_position_x;
    float target_position_y;
    float target_position_z;
    uint8_t commd_keyboard;
    uint16_t target_robot_ID;
} ext_robot_command_t;

// 0x0304 通过图传串口发送的键鼠和遥控器数据
// 0x0305
typedef struct {
    uint16_t target_robot_ID;
    float target_position_x;
    float target_position_y;
} ext_client_map_command_t;

// 成对使用，用以代替特有的typedef __packed struct写法
#pragma pack()

// 裁判系统数据
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
    ext_robot_interactive_data_t robot_interactive_data;
    ext_robot_command_t robot_command;
    ext_client_map_command_t client_map_command;
} referee_rx_data;

// 裁判系统外设结构体
typedef struct Referee_t {
    uint8_t primary_data[REFEREE_RX_MAX_SIZE];
    referee_rx_data rx_data;

    referee_config config;
    monitor_item *monitor;
    uint8_t receive_status;
    uint8_t receive_len;
    uint16_t receive_target_len;
} Referee;

void referee_driver_init(void);

Referee *referee_Create(referee_config *config);

#endif