#include "pub_sub.h"

#include "stdlib.h"
#include "string.h"

static cvector* pub_lists;

void SubPub_Init() {
    pub_lists = cvector_create(sizeof(Publisher*));
}

void pub_commit(struct publisher_t* pub, publish_data data) {
    for (uint32_t i = 0; i < pub->subs->cv_len; ++i) {
        Subscriber* now = *(Subscriber**)cvector_val_at(pub->subs, i);
        if (now->queue->cq_len == now->queue->cq_max_len) circular_queue_pop(now->queue);
        circular_queue_push(now->queue, &data);
    }
}

publish_data sub_get(struct subscriber_t* sub) {
    publish_data now;
    now.data = NULL;
    now.len = -1;
    if (sub->queue->cq_len) {
        now = *(publish_data*)circular_queue_pop(sub->queue);
    }
    return now;
}

Publisher* register_pub(const char* topic) {
    for (uint32_t i = 0; i < pub_lists->cv_len; ++i) {
        Publisher* now = *((Publisher**)cvector_val_at(pub_lists, i));
        if (!strcmp(now->pub_topic, topic)) return NULL;
    }
    Publisher* obj = malloc(sizeof(Publisher));
    cvector_pushback(pub_lists, &obj);
    obj->pub_topic = topic;
    obj->publish = pub_commit;
    obj->subs = cvector_create(sizeof(Subscriber*));
    return obj;
}

Subscriber* register_sub(const char* topic, uint32_t buffer_len) {
    for (uint32_t i = 0; i < pub_lists->cv_len; ++i) {
        Publisher* now = *((Publisher**)cvector_val_at(pub_lists, i));
        if (strcmp(now->pub_topic, topic)) {
            Subscriber* obj = malloc(sizeof(Subscriber));
            obj->queue = create_circular_queue(sizeof(publish_data), 5);
            obj->sub_topic = topic;
            obj->getdata = sub_get;
            return obj;
        }
    }
    return NULL;
}