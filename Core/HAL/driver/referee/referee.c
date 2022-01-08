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

cvector *referee_instances;

void referee_data_solve(referee *obj, referee_rx_data *data);
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
    if (0 <= len && len <= 100) {
        for (size_t i = 0; i < referee_instances->cv_len; i++) {
            referee *now = *(referee **)cvector_val_at(referee_instances, i);
            if (uart_index == now->config.bsp_uart_index) {
                // memcpy()
                //referee_data_solve(now);
            }
        }
    }
}

void referee_data_solve(referee *obj, referee_rx_data *data) {
    switch (data->cmd_id) {
        case GAME_STATE_CMD_ID:
            break;
        case GAME_RESULT_CMD_ID:
            break;
        case GAME_ROBOT_HP_CMD_ID:
            break;
        case DART_STATUS_ID:
            break;
        case FIELD_EVENTS_CMD_ID:
            break;
        case SUPPLY_PROJECTILE_ACTION_CMD_ID:
            break;
        case SUPPLY_PROJECTILE_BOOKING_CMD_ID:
            break;
        case REFEREE_WARNING_CMD_ID:
            break;
        case DART_REMAINING_TIME_ID:
            break;
        case ROBOT_STATE_CMD_ID:
            break;
        case POWER_HEAT_DATA_CMD_ID:
            break;
        case ROBOT_POS_CMD_ID:
            break;
        case BUFF_MUSK_CMD_ID:
            break;
        case AERIAL_ROBOT_ENERGY_CMD_ID:
            break;
        case ROBOT_HURT_CMD_ID:
            break;
        case SHOOT_DATA_CMD_ID:
            break;
        case BULLET_REMAINING_CMD_ID:
            break;
        case RFID_STATUS_ID:
            break;
        case DART_CILENT_CMD_T:
            break;
        case STUDENT_INTERACTIVE_DATA_CMD_ID:
            break;
        case ROBOT_INTERACTIVE_DATA_T:
            break;
        case ROBOT_COMMAND_T:
            break;
        case CLIENT_MAP_COMMAND_T:
            break;
        default:
            break;
    }
}