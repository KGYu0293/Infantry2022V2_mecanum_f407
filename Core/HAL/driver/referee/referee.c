#include "referee.h"

#include "bsp_log.h"
#include "cvector.h"

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
    memset(&(obj->tool), 0, sizeof(referee_unpack_tool));
    obj->tool.next_step_wait_len = 1;
    obj->robot_status_received = 0;
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
                circular_queue_push(now->primary_data, &data[i]);
            }
            now->monitor->reset(now->monitor);
        }
    }
}

void referee_data_solve(Referee *obj) {
    uint8_t *byte_now_pt = NULL;

    // 反复弹出直到缓冲队列长度为0
    while (obj->primary_data->cq_len >= obj->tool.next_step_wait_len) {
        byte_now_pt = circular_queue_pop(obj->primary_data);
        uint8_t byte_now = *byte_now_pt;
        switch (obj->tool.step) {
            case s_header_sof:
                // 包错误判断-固定起始字节
                if (byte_now == REFEREE_SOF) {
                    obj->tool.step++;
                    obj->tool.rx_pack.header.SOF = byte_now;
                    obj->tool.next_step_wait_len = 2;
                } else {
                    obj->tool.step = 0;
                    obj->tool.next_step_wait_len = 1;
                }
                break;
            case s_header_length:
                obj->tool.rx_pack.header.data_length = byte_now;
                byte_now_pt = circular_queue_pop(obj->primary_data);
                byte_now = *byte_now_pt;
                obj->tool.rx_pack.header.data_length |= ((uint16_t)byte_now << 8);
                // 包错误判断-长度超范围
                if (obj->tool.rx_pack.header.data_length > REFEREE_PACK_LEN_DATA_MAX) {
                    obj->tool.step = 0;
                    obj->tool.next_step_wait_len = 1;
                } else {
                    obj->tool.step++;
                    obj->tool.next_step_wait_len = 1;
                }
                break;
            case s_header_seq:
                // 没看出来这个数有啥意义 暂时不管
                obj->tool.rx_pack.header.seq = byte_now;
                obj->tool.step++;
                obj->tool.next_step_wait_len = 1;
                break;
            case s_header_crc8: {
                // 包错误判断-CRC8校验
                uint8_t crc8_result = CRC8_Modbus_calc(&(obj->tool.rx_pack.header.SOF), sizeof(frame_header) - 1, crc8_default);
                if (crc8_result == byte_now) {
                    obj->tool.step++;
                    obj->tool.next_step_wait_len = 2;
                    obj->tool.rx_pack.header.CRC8 = byte_now;
                    // printf_log("crc8:%d\n",obj->tool.buffer_pt);
                } else {
                    obj->tool.step = 0;
                    obj->tool.next_step_wait_len = 1;
                }
                break;
            }
            case s_cmd_id:
                obj->tool.rx_pack.cmd_id = byte_now;
                byte_now_pt = circular_queue_pop(obj->primary_data);
                byte_now = *byte_now_pt;
                obj->tool.rx_pack.cmd_id |= ((uint16_t)byte_now << 8);
                obj->tool.step++;
                obj->tool.next_step_wait_len = obj->tool.rx_pack.header.data_length;
                break;
            case s_data:
                for (size_t i = 0; i < obj->tool.rx_pack.header.data_length; i++) {
                    if (i > 0) {
                        byte_now_pt = circular_queue_pop(obj->primary_data);
                        byte_now = *byte_now_pt;
                    }
                    obj->tool.rx_pack.data[i] = byte_now;
                }
                obj->tool.step++;
                obj->tool.next_step_wait_len = 2;
                break;
            case s_crc16: {
                uint16_t crc16_recv = byte_now;
                byte_now_pt = circular_queue_pop(obj->primary_data);
                byte_now = *byte_now_pt;
                crc16_recv |= ((uint16_t)byte_now << 8);
                // printf_log("a:%d %d\n",REFEREE_PACK_LEN_HEADER + REFEREE_PACK_LEN_CMD_ID + obj->tool.rx_pack.header.data_length,obj->tool.buffer_pt);
                // 包错误判断-CRC16校验
                uint16_t crc16_result =
                    CRC16_Modbus_calc(&(obj->tool.rx_pack.header.SOF), REFEREE_PACK_LEN_HEADER + REFEREE_PACK_LEN_CMD_ID + obj->tool.rx_pack.header.data_length, crc16_default);

                if (crc16_result != crc16_recv) {
                    obj->tool.step = s_header_sof;
                    obj->tool.next_step_wait_len = 1;
                } else {
                    // 包提取完成 拷贝数据
                    referee_solve_pack(obj, &(obj->tool.rx_pack));
                    obj->tool.step = s_header_sof;
                    obj->tool.next_step_wait_len = 1;
                }
                break;
            }
            default:
                obj->tool.step = 0;
                obj->tool.next_step_wait_len = 1;
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
            //第一次收到时置位为1
            if (!obj->robot_status_received) {
                obj->robot_status_received = 1;
            }

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
        case ROBOT_INTERACT_ID:{
            // 机器人间通信接收
            uint16_t recv_len = rx_pack->header.data_length;
            memcpy(&obj->rx_data.interactve_data, rx_pack->data, recv_len);
            break;
        }
        case ROBOT_EXT_CONTROL_ID:
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

void referee_recv_loop() {
    for (size_t i = 0; i < referee_instances->cv_len; i++) {
        Referee *now = *(Referee **)cvector_val_at(referee_instances, i);
        referee_data_solve(now);
    }
}

void referee_send_ext(Referee *obj, ext_robot_interact_frame *frame) {
    uint16_t send_len = REFEREE_PACK_LEN_HEADER + REFEREE_PACK_LEN_CMD_ID + REFEREE_PACK_LEN_TAIL + frame->header.data_length;
    BSP_UART_Send_DMA(obj->config.bsp_uart_index, (uint8_t *)frame, send_len);
}