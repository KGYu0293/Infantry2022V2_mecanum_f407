#ifndef H_MONITOR_H
#define H_MONITOR_H

typedef void (*lost_callback)(void);

typedef struct monitor_item_t {
    char reload_count;
    char count;
    lost_callback callback;
    void (*reset)(struct monitor_item_t* item);
} monitor_item;

void Monitor_Init();
monitor_item* Monitor_Register(lost_callback callback, char count);
void Monitor_Loop();

#endif
