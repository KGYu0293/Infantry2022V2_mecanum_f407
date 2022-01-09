#include "referee.h"

#include "bsp_uart.h"
#include "cvector.h"
// 待测

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
#define STUDENT_INTERACTIVE_DATA_CMD_ID 0x0301
#define ROBOT_INTERACTIVE_DATA_T 0X0302
#define ROBOT_COMMAND_T 0X0303
#define CLIENT_MAP_COMMAND_T 0X0305

// 收到的裁判系统数据包
typedef struct referee_rx_pack_t {
    struct {
        uint8_t SOF;
        uint16_t data_length;
        uint8_t seq;
        uint8_t CRC8;
    } frame_header;
    uint16_t cmd_id;
    uint8_t data[REFEREE_RX_MAX_SIZE];
    uint16_t frame_tail;
} referee_rx_pack;

cvector *referee_instances;

void referee_data_solve(referee *obj, uint32_t len);
void referee_Rx_callback(uint8_t uart_index, uint8_t *data, uint32_t len);

void referee_driver_init() {
    referee_instances = cvector_create(sizeof(referee *));
    BSP_UART_RegisterRxCallback(UART_REFEREE_PORT, referee_Rx_callback);
}

referee *referee_Create(referee_config *config) {
    referee *obj = (referee *)malloc(sizeof(referee));
    obj->config = *config;
    obj->monitor = Monitor_Register(obj->config.lost_callback, 10, obj);
    cvector_pushback(referee_instances, &obj);
    return obj;
}

void referee_Rx_callback(uint8_t uart_index, uint8_t *data, uint32_t len) {
    for (size_t i = 0; i < referee_instances->cv_len; i++) {
        referee *now = *(referee **)cvector_val_at(referee_instances, i);
        if (uart_index == now->config.bsp_uart_index) {
            memcpy(now->primary_data, data, REFEREE_RX_MAX_SIZE);
            now->monitor->reset(now->monitor);
            referee_data_solve(now, len);
        }
    }
}

void referee_data_solve(referee *obj, uint32_t len) {
    referee_rx_pack data_pack;
    memcpy(&data_pack.frame_header, obj->primary_data, 5);
    memcpy(&data_pack.cmd_id, obj->primary_data + 5, 2);
    memcpy(data_pack.data, obj->primary_data + 7, len - 9);  // data本身即为指针
    memcpy(&data_pack.frame_tail, obj->primary_data + len - 2, 2);

    switch (data_pack.cmd_id) {
        case GAME_STATE_CMD_ID:
            memcpy(&obj->rx_data.game_state, data_pack.data, sizeof(obj->rx_data.game_state));
            break;
        case GAME_RESULT_CMD_ID:
            memcpy(&obj->rx_data.game_result, data_pack.data, sizeof(obj->rx_data.game_result));
            break;
        case GAME_ROBOT_HP_CMD_ID:
            memcpy(&obj->rx_data.robot_HP, data_pack.data, sizeof(obj->rx_data.robot_HP));
            break;
        case DART_STATUS_ID:
            memcpy(&obj->rx_data.dart_status, data_pack.data, sizeof(obj->rx_data.dart_status));
            break;
        case FIELD_EVENTS_CMD_ID:
            memcpy(&obj->rx_data.event, data_pack.data, sizeof(obj->rx_data.event));
            break;
        case SUPPLY_PROJECTILE_ACTION_CMD_ID:
            memcpy(&obj->rx_data.supply_projectile_action, data_pack.data, sizeof(obj->rx_data.supply_projectile_action));
            break;
        case SUPPLY_PROJECTILE_BOOKING_CMD_ID:
            // RM对抗赛尚未开放
            break;
        case REFEREE_WARNING_CMD_ID:
            memcpy(&obj->rx_data.referee_warning, data_pack.data, sizeof(obj->rx_data.referee_warning));
            break;
        case DART_REMAINING_TIME_ID:
            memcpy(&obj->rx_data.dart_remaining_time, data_pack.data, sizeof(obj->rx_data.dart_remaining_time));
            break;
        case ROBOT_STATE_CMD_ID:
            memcpy(&obj->rx_data.game_robot_state, data_pack.data, sizeof(obj->rx_data.game_robot_state));
            break;
        case POWER_HEAT_DATA_CMD_ID:
            memcpy(&obj->rx_data.power_heat, data_pack.data, sizeof(obj->rx_data.power_heat));
            break;
        case ROBOT_POS_CMD_ID:
            memcpy(&obj->rx_data.robot_pos, data_pack.data, sizeof(obj->rx_data.robot_pos));
            break;
        case BUFF_MUSK_CMD_ID:
            memcpy(&obj->rx_data.buff, data_pack.data, sizeof(obj->rx_data.buff));
            break;
        case AERIAL_ROBOT_ENERGY_CMD_ID:
            memcpy(&obj->rx_data.aerial_robot_energy, data_pack.data, sizeof(obj->rx_data.aerial_robot_energy));
            break;
        case ROBOT_HURT_CMD_ID:
            memcpy(&obj->rx_data.robot_hurt, data_pack.data, sizeof(obj->rx_data.robot_hurt));
            break;
        case SHOOT_DATA_CMD_ID:
            memcpy(&obj->rx_data.shoot_data, data_pack.data, sizeof(obj->rx_data.shoot_data));
            break;
        case BULLET_REMAINING_CMD_ID:
            memcpy(&obj->rx_data.bullet_remaining, data_pack.data, sizeof(obj->rx_data.bullet_remaining));
            break;
        case RFID_STATUS_ID:
            memcpy(&obj->rx_data.rfid_status, data_pack.data, sizeof(obj->rx_data.rfid_status));
            break;
        case DART_CILENT_CMD_T:
            memcpy(&obj->rx_data.dart_cilent_cmd, data_pack.data, sizeof(obj->rx_data.dart_cilent_cmd));
            break;
        case STUDENT_INTERACTIVE_DATA_CMD_ID:
            memcpy(&obj->rx_data.robot_interactive_data, data_pack.data, sizeof(obj->rx_data.robot_interactive_data));
            break;
        case ROBOT_INTERACTIVE_DATA_T:
            // 自定义控制器 暂无
            break;
        case ROBOT_COMMAND_T:
            memcpy(&obj->rx_data.robot_command, data_pack.data, sizeof(obj->rx_data.robot_command));
            break;
        case CLIENT_MAP_COMMAND_T:
            memcpy(&obj->rx_data.client_map_command, data_pack.data, sizeof(obj->rx_data.client_map_command));
            break;
        default:
            break;
    }
}