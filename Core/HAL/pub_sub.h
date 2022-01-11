#ifndef _PUB_SUB_H
#define _PUB_SUB_H

#include "cvector.h"
#include "circular_queue.h"
#include "stdint.h"

typedef struct publish_data_t {
    uint8_t* data;
    int len;
} publish_data;

typedef struct publisher_t {
    const char* pub_topic;
    cvector* subs;
    void (*publish)(struct publisher_t* pub, publish_data data);
} Publisher;

typedef struct subscriber_t{
    const char *sub_topic;
    circular_queue* queue;
    publish_data (*getdata)(struct subscriber_t* sub);
} Subscriber;

void SubPub_Init();
Publisher* register_pub(const char* topic);
Subscriber* register_sub(const char* topic, uint32_t buffer_len);

#endif