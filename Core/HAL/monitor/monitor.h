#ifndef H_MONITOR_H
#define H_MONITOR_H
#include <stdint.h>
typedef void (*lost_callback)(void* data);

typedef struct monitor_item_t {
    int reload_count;
    int count;
    lost_callback callback;
    void (*reset)(struct monitor_item_t* item);
    void* data;
} monitor_item;

void Monitor_Init();
monitor_item* Monitor_Register(lost_callback callback, int count, void* callback_data);
uint8_t is_Offline(monitor_item* obj);
void Monitor_Loop();

#endif
