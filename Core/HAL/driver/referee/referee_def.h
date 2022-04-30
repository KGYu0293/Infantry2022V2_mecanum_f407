#ifndef _REFEREE_DEF_H
#define _REFEREE_DEF_H
#include <stdint.h>
// 固定起始帧
#define REFEREE_SOF 0xA5
// 单个接收数据包最小整包长度
#define REFEREE_RX_MIN_SIZE 9
// 单个接收数据包最大整包长度
#define REFEREE_RX_MAX_SIZE 128
// 单个发送数据包最大长度
#define REFEREE_PACK_MAX_SIZE 128
// 单个接收数据包中各部分长度
#define REFEREE_PACK_LEN_HEADER 5
#define REFEREE_PACK_LEN_CMD_ID 2
#define REFEREE_PACK_LEN_TAIL 2
#define REFEREE_PACK_LEN_DATA_MAX REFEREE_RX_MAX_SIZE - REFEREE_PACK_LEN_HEADER - REFEREE_PACK_LEN_CMD_ID - REFEREE_PACK_LEN_TAIL

// 接收缓冲队列最大长度
#define REFEREE_RX_QUENE_MAX_LEN 1024

// cmd_id 命令码ID
#define GAME_STATE_CMD_ID 0x0001
#define GAME_RESULT_CMD_ID 0x0002
#define GAME_ROBOT_HP_CMD_ID 0x0003
#define DART_STATUS_ID 0x0004
#define FIELD_EVENTS_CMD_ID 0x0101
#define SUPPLY_PROJECTILE_ACTION_CMD_ID 0x0102
#define SUPPLY_PROJECTILE_BOOKING_CMD_ID 0x0103
#define REFEREE_WARNING_CMD_ID 0x0104
#define DART_REMAINING_TIME_ID 0x0105
#define ROBOT_STATE_CMD_ID 0x0201
#define POWER_HEAT_DATA_CMD_ID 0x0202
#define ROBOT_POS_CMD_ID 0x0203
#define BUFF_MUSK_CMD_ID 0x0204
#define AERIAL_ROBOT_ENERGY_CMD_ID 0x0205
#define ROBOT_HURT_CMD_ID 0x0206
#define SHOOT_DATA_CMD_ID 0x0207
#define BULLET_REMAINING_CMD_ID 0x0208
#define RFID_STATUS_ID 0x0209
#define DART_CILENT_CMD_T 0x020A
#define ROBOT_INTERACTIVE_DATA_T 0X0302
#define ROBOT_COMMAND_T 0X0303
#define CLIENT_MAP_COMMAND_T 0X0305

//机器人ID
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

//客户端ID
typedef enum Client_id_e {
    CRED_HERO = 0x0101,
    CRED_ENGINEER = 0x0102,
    CRED_STANDARD_1 = 0x0103,
    CRED_STANDARD_2 = 0x0104,
    CRED_STANDARD_3 = 0x0105,
    CRED_AERIAL = 0x0106,
    CBLUE_HERO = 0x0165,
    CBLUE_ENGINEER = 0x0166,
    CBLUE_STANDARD_1 = 0x0167,
    CBLUE_STANDARD_2 = 0x0168,
    CBLUE_STANDARD_3 = 0x0169,
    CBLUE_AERIAL = 0x16A,
} Client_id;

//比赛进程定义
typedef enum {
    PROGRESS_UNSTART = 0,
    PROGRESS_PREPARE = 1,
    PROGRESS_SELFCHECK = 2,
    PROGRESS_5sCOUNTDOWN = 3,
    PROGRESS_BATTLE = 4,
    PROGRESS_CALCULATING = 5,
} game_progress_e;

// 成对使用，用以代替特有的typedef __packed struct写法
#pragma pack(1)

// 裁判系统统一包头
typedef struct Frame_header_t {
    uint8_t SOF;
    uint16_t data_length;
    uint8_t seq;
    uint8_t CRC8;
} frame_header;

//机器人接收：比赛信息部分
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

//机器人接收：事件信息部分
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

//机器人接收：机器人信息部分
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

//裁判系统接收数据包
typedef struct referee_rx_pack_t {
    frame_header header;
    uint16_t cmd_id;
    uint8_t data[REFEREE_RX_MAX_SIZE];
    uint16_t frame_tail;
} referee_rx_pack;

//机器人交互（发送）：机器人交互部分

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
//交互数据统一数据包包头
typedef struct {
    uint16_t data_cmd_id;
    uint16_t sender_ID;
    uint16_t receiver_ID;
} ext_student_interactive_header_data_t;

//机器人间通信数据 可按照官方协议自定义
//以下为示例
/*
typedef struct {
    // uint8_t data[112];
    uint8_t sentry_chassis_state;
    uint8_t sentry_gimbal_state;
    uint8_t sentry_fire_state;
} ext_robot_interactive_data_t;
*/

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

//自定义交互数据发送结构体，机器人间交互以及UI发送都用这个
typedef struct {
    frame_header header;                                //总包头
    ext_student_interactive_header_data_t data_header;  //交互专用数据段包头
    uint8_t* data;                                      //数据内容
    uint16_t frame_tail;                                // CRC16校验包尾
} ext_robot_interact_frame;

// 成对使用，用以代替特有的typedef __packed struct写法
#pragma pack()

#endif