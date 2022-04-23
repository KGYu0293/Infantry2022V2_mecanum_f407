#include "referee.h"

#include "cvector.h"
// 待测

// 固定起始帧
#define REFEREE_SOF 0xA5
// 单个接收数据包最小整包长度
#define REFEREE_RX_MIN_SIZE 9
// 单个接收数据包最大整包长度
#define REFEREE_RX_MAX_SIZE 128
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
#define STUDENT_INTERACTIVE_DATA_CMD_ID 0x0301
#define ROBOT_INTERACTIVE_DATA_T 0X0302
#define ROBOT_COMMAND_T 0X0303
#define CLIENT_MAP_COMMAND_T 0X0305

// 收到的裁判系统数据包
#pragma pack(1)
typedef struct Frame_header_t {
    uint8_t SOF;
    uint16_t data_length;
    uint8_t seq;
    uint8_t CRC8;
} frame_header;
typedef struct referee_rx_pack_t {
    frame_header header;
    uint16_t cmd_id;
    uint8_t data[REFEREE_RX_MAX_SIZE];
    uint16_t frame_tail;
} referee_rx_pack;
#pragma pack()
// 解包辅助结构体
typedef enum referee_unpack_step_e { s_header_sof = 0, s_header_length, s_header_seq, s_header_crc8, s_cmd_id, s_data, s_crc16 } referee_unpack_step;
typedef struct referee_unpack_tool_t {
    referee_rx_pack rx_pack;
    referee_unpack_step step;
} referee_unpack_tool;

cvector *referee_instances;

void referee_Rx_callback(uint8_t uart_index, uint8_t *data, uint32_t len);
void referee_solve_pack(Referee *obj, referee_rx_pack *rx_pack);

void referee_driver_init() {
    referee_instances = cvector_create(sizeof(Referee *));
    BSP_UART_RegisterRxCallback(UART_REFEREE_PORT, referee_Rx_callback);
}

Referee *referee_Create(referee_config *config) {
    Referee *obj = (Referee *)malloc(sizeof(Referee));
    memset(obj, 0, sizeof(sizeof(Referee)));
    obj->config = *config;
    obj->monitor = Monitor_Register(obj->config.lost_callback, 10, obj);
    cvector_pushback(referee_instances, &obj);
    // 数据接收队列
    obj->primary_data = create_circular_queue(sizeof(uint8_t), REFEREE_RX_QUENE_MAX_LEN);
    memset(&(obj->rx_data), 0, sizeof(referee_rx_data));
    return obj;
}

void referee_Rx_callback(uint8_t uart_index, uint8_t *data, uint32_t len) {
    for (size_t i = 0; i < referee_instances->cv_len; i++) {
        Referee *now = *(Referee **)cvector_val_at(referee_instances, i);
        if (uart_index == now->config.bsp_uart_index) {
            // 注释原因：刚开局时可能一次性收到大量数据
            // if (len > REFEREE_RX_MAX_SIZE) return;
            for (size_t i = 0; i < len; i++) {
                // 将数据直接塞进缓冲队列 可能一个包触发两次空闲中断/多个包触发一次，所以不做判断
                circular_queue_push(now->primary_data, data[i]);
            }
            now->monitor->reset(now->monitor);
        }
    }
}

void referee_data_solve(Referee *obj) {
    uint16_t byte_now = 0;
    referee_unpack_tool tool;

    // 反复弹出直到缓冲队列长度为0
    while (obj->primary_data->cq_len > 0) {
        byte_now = circular_queue_pop(obj->primary_data);
        switch (tool.step) {
            case s_header_sof:
                // 包错误判断-固定起始字节
                if (byte_now == REFEREE_SOF) {
                    tool.step++;
                } else {
                    tool.step = 0;
                }
                break;
            case s_header_length:
                if (obj->primary_data->cq_len >= 2) {
                    tool.rx_pack.header.data_length = byte_now;
                    byte_now = circular_queue_pop(obj->primary_data);
                    tool.rx_pack.header.data_length |= (byte_now << 8);
                    // 包错误判断-长度超范围
                    if (tool.rx_pack.header.data_length > REFEREE_PACK_LEN_DATA_MAX) {
                        tool.step = 0;
                    } else {
                        tool.step++;
                    }
                } else {
                    tool.step = 0;
                }
                break;
            case s_header_seq:
                // 没看出来这个数有啥意义 暂时不管
                tool.rx_pack.header.seq = byte_now;
                tool.step++;
                break;
            case s_header_crc8: {
                // 包错误判断-CRC8校验,crc_check=0表示通过
                int crc8_result = CRC8_Modbus_calc(&(tool.rx_pack.header.SOF), sizeof(frame_header), crc8_default);
                if (crc8_result) {
                    tool.step = 0;
                } else {
                    tool.step++;
                }
                break;
            }
            case s_cmd_id:
                if (obj->primary_data->cq_len >= 2) {
                    tool.rx_pack.cmd_id = (byte_now << 8);
                    byte_now = circular_queue_pop(obj->primary_data);
                    tool.rx_pack.cmd_id |= byte_now;
                    tool.step++;
                }
                uint8_t first_data_flag = 0;
                break;
            case s_data:
                if (obj->primary_data->cq_len > tool.rx_pack.header.data_length) {
                    for (size_t i = 0; i++; i < tool.rx_pack.header.data_length) {
                        if (first_data_flag != 0) {
                            byte_now = circular_queue_pop(obj->primary_data);
                        }
                        tool.rx_pack.data[i] = byte_now;
                        first_data_flag = 1;
                    }
                    tool.step++;
                } else {
                    tool.step = 0;
                }
                break;
            case s_crc16: {
                // 包错误判断-CRC16校验
                int crc16_result =
                    CRC16_Modbus_calc(&(tool.rx_pack.header.SOF), REFEREE_PACK_LEN_HEADER + REFEREE_PACK_LEN_CMD_ID + REFEREE_PACK_LEN_TAIL + tool.rx_pack.header.data_length, crc16_default);
                if (crc16_result) {
                    tool.step = 0;
                } else {
                    // 包提取完成 拷贝数据
                    referee_solve_pack(obj, &(tool.rx_pack));
                }
                break;
            }
            default:
                tool.step = 0;
                break;
        }
    }
}

void referee_solve_pack(Referee *obj, referee_rx_pack *rx_pack) {
    switch (rx_pack->cmd_id) {
        case GAME_STATE_CMD_ID:
            memcpy(&obj->rx_data.game_state, rx_pack->data, sizeof(obj->rx_data.game_state));
            break;
        case GAME_RESULT_CMD_ID:
            memcpy(&obj->rx_data.game_result, rx_pack->data, sizeof(obj->rx_data.game_result));
            break;
        case GAME_ROBOT_HP_CMD_ID:
            memcpy(&obj->rx_data.robot_HP, rx_pack->data, sizeof(obj->rx_data.robot_HP));
            break;
        case DART_STATUS_ID:
            memcpy(&obj->rx_data.dart_status, rx_pack->data, sizeof(obj->rx_data.dart_status));
            break;
        case FIELD_EVENTS_CMD_ID:
            memcpy(&obj->rx_data.event, rx_pack->data, sizeof(obj->rx_data.event));
            break;
        case SUPPLY_PROJECTILE_ACTION_CMD_ID:
            memcpy(&obj->rx_data.supply_projectile_action, rx_pack->data, sizeof(obj->rx_data.supply_projectile_action));
            break;
        case SUPPLY_PROJECTILE_BOOKING_CMD_ID:
            // RM对抗赛尚未开放
            break;
        case REFEREE_WARNING_CMD_ID:
            memcpy(&obj->rx_data.referee_warning, rx_pack->data, sizeof(obj->rx_data.referee_warning));
            break;
        case DART_REMAINING_TIME_ID:
            memcpy(&obj->rx_data.dart_remaining_time, rx_pack->data, sizeof(obj->rx_data.dart_remaining_time));
            break;
        case ROBOT_STATE_CMD_ID:
            memcpy(&obj->rx_data.game_robot_state, rx_pack->data, sizeof(obj->rx_data.game_robot_state));
            break;
        case POWER_HEAT_DATA_CMD_ID:
            memcpy(&obj->rx_data.power_heat, rx_pack->data, sizeof(obj->rx_data.power_heat));
            break;
        case ROBOT_POS_CMD_ID:
            memcpy(&obj->rx_data.robot_pos, rx_pack->data, sizeof(obj->rx_data.robot_pos));
            break;
        case BUFF_MUSK_CMD_ID:
            memcpy(&obj->rx_data.buff, rx_pack->data, sizeof(obj->rx_data.buff));
            break;
        case AERIAL_ROBOT_ENERGY_CMD_ID:
            memcpy(&obj->rx_data.aerial_robot_energy, rx_pack->data, sizeof(obj->rx_data.aerial_robot_energy));
            break;
        case ROBOT_HURT_CMD_ID:
            memcpy(&obj->rx_data.robot_hurt, rx_pack->data, sizeof(obj->rx_data.robot_hurt));
            break;
        case SHOOT_DATA_CMD_ID:
            memcpy(&obj->rx_data.shoot_data, rx_pack->data, sizeof(obj->rx_data.shoot_data));
            break;
        case BULLET_REMAINING_CMD_ID:
            memcpy(&obj->rx_data.bullet_remaining, rx_pack->data, sizeof(obj->rx_data.bullet_remaining));
            break;
        case RFID_STATUS_ID:
            memcpy(&obj->rx_data.rfid_status, rx_pack->data, sizeof(obj->rx_data.rfid_status));
            break;
        case DART_CILENT_CMD_T:
            memcpy(&obj->rx_data.dart_cilent_cmd, rx_pack->data, sizeof(obj->rx_data.dart_cilent_cmd));
            break;
        case STUDENT_INTERACTIVE_DATA_CMD_ID:
            memcpy(&obj->rx_data.robot_interactive_data, rx_pack->data, sizeof(obj->rx_data.robot_interactive_data));
            break;
        case ROBOT_INTERACTIVE_DATA_T:
            // 自定义控制器 暂无
            break;
        case ROBOT_COMMAND_T:
            memcpy(&obj->rx_data.robot_command, rx_pack->data, sizeof(obj->rx_data.robot_command));
            break;
        case CLIENT_MAP_COMMAND_T:
            memcpy(&obj->rx_data.client_map_command, rx_pack->data, sizeof(obj->rx_data.client_map_command));
            break;
        default:
            break;
    }
}
