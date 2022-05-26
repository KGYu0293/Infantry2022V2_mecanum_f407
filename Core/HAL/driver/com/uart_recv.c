#include "uart_recv.h"

#include "bsp_uart.h"
#include "bsp_def.h"
#include "soft_crc.h"
#include "cvector.h"
#include "stdio.h"

cvector* uart_recv_instances;

void UartRecv_RxCallBack(uint8_t uart_index, uint8_t* data, uint32_t len);

void UartRecv_Driver_Init() {
    uart_recv_instances = cvector_create(sizeof(uart_recv*));
    BSP_UART_RegisterRxCallback(0, UartRecv_RxCallBack);
}

uart_recv* UartRecv_Create(uart_recv_config* config) {
    uart_recv* obj = (uart_recv*)malloc(sizeof(uart_recv));
    memset(obj,0,sizeof(uart_recv));
    obj->config = *config;
    obj->data_rx.len = obj->config.data_len;
    obj->data_rx.data = (uint8_t*)malloc(obj->config.data_len);
    memset(obj->data_rx.data,0,obj->config.data_len);
    //identifier 2位，起始s结束e 2位，crc16 2位，len 1位
    obj->buf_len = obj->config.data_len + 7;
    obj->rxbuf = (uint8_t*) malloc(obj->buf_len);
    obj->recv_len = 0;
    obj->recv_status = 0;
    obj->data_updated = 0;
    cvector_pushback(uart_recv_instances,&obj);
    obj->monitor = Monitor_Register(obj->config.lost_callback, 20, obj);
    return obj;
}

void UartRecv_RxCallBack(uint8_t uart_index, uint8_t* data, uint32_t len){
    for(size_t i = 0;i < uart_recv_instances->cv_len;++i){
        uart_recv* now = *(uart_recv**)cvector_val_at(uart_recv_instances,i);
        if(now->config.bsp_uart_index == uart_index){
            if(data[0] == 's'){
                now->recv_status = 1;
            }
            if(now->recv_status){
                if(now->recv_len + len > now->buf_len){
                    now->recv_status = 0;
                    now->recv_len = 0;
                    continue;
                }
                memcpy(now->rxbuf + now->recv_len,data,len);
                now->recv_len += len;
                if(now->recv_len > 2){
                    uint16_t data_recv_id = (((uint16_t)now->rxbuf[2]) << 8) | now->rxbuf[1];
                    if(data_recv_id != now->config.uart_identifier){
                        now->recv_status = 0;
                        now->recv_len = 0;
                        continue;
                    }
                }
                if(now->recv_len == now->buf_len && now->rxbuf[now->recv_len - 1] == 'e'){
                    if(CheckVaild(now->rxbuf + 3,now->buf_len - 4)){
                        BufferToData(now->rxbuf + 3,&now->data_rx);
                        now->recv_status = 0;
                        now->recv_len = 0;
                        now->data_updated = 1;
                        now->monitor->reset(now->monitor);
                        if(now->config.notify_func != NULL){
                            now->config.notify_func(now);
                        }
                    }
                }
            }
        }
    }
}
