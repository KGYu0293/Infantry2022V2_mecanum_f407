// 外设监控
#include "monitor.h"

#include "cvector.h"

cvector* monitor_lists;

void Monitor_Init() { monitor_lists = cvector_create(sizeof(monitor_item*)); }

void Monitor_Reset(monitor_item* item) { item->count = item->reload_count; }
// 采用注册的方式
// 比如电机注册一个monitor，该函数会返回一个对象指针
// count相当于看门狗计数器值
monitor_item* Monitor_Register(lost_callback callback, int count,void* callback_data) {
    monitor_item* item = malloc(sizeof(monitor_item));
    item->reload_count = count;
    item->count = count;
    item->callback = callback;
    item->reset = Monitor_Reset;
    item->data = callback_data;
    cvector_pushback(monitor_lists,&item);
    return item;
}

void Monitor_Loop() {
    for (int i = 0; i < monitor_lists->cv_len; i++) {
        // void* val = cvector_val_at(monitor_lists, i);
        monitor_item* item = *(monitor_item**) cvector_val_at(monitor_lists,i);
        item->count--;
        if (item->count <= 0) {
            item->count = 0;
            if(item->callback != NULL){
                item->callback(item->data);
            }
        }
    }
}

uint8_t is_Offline(monitor_item* obj){
    return obj->count <= 0;
}