// 外设监控
#include "monitor.h"

#include "cvector.h"

cvector* monitor_lists;

void Monitor_Init() { monitor_lists = cvector_create(sizeof(monitor_item)); }

void Monitor_Reset(monitor_item* item) { item->count = item->reload_count; }
// 采用注册的方式
// 比如电机注册一个monitor，该函数会返回一个对象指针
// count相当于看门狗计数器值
monitor_item* Monitor_Register(lost_callback callback, char count) {
    monitor_item item;
    item.reload_count = count;
    item.count = count;
    item.callback = callback;
    item.reset = Monitor_Reset;
    void* vector = cvector_pushback(monitor_lists, &item);
    return (monitor_item*)vector;
}

void Monitor_Loop() {
    int lists_len = cvector_length(monitor_lists);
    for (int i = 0; i < lists_len; i++) {
        void* val = cvector_val_at(monitor_lists, i);
        monitor_item* item = (monitor_item*)val;
        item->count--;
        if (item->count <= 0) {
            item->count = 0;
            item->callback();
        }
    }
}
