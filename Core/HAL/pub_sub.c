#include "pub_sub.h"

#include "stdlib.h"
#include "string.h"

struct internal_topic {
    Publisher* pub;
    cvector* subs;
    const char* topic_str;
};

cvector* topics;

void SubPub_Init() { topics = cvector_create(sizeof(struct internal_topic*)); }

void pub_commit(struct publisher_t* pub, publish_data data) {
    for (uint32_t i = 0; i < pub->topic->subs->cv_len; ++i) {
        Subscriber* now = *(Subscriber**)cvector_val_at(pub->topic->subs, i);
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

struct internal_topic* register_topic(const char* topic) {
    for (uint32_t i = 0; i < topics->cv_len; ++i) {
        struct internal_topic* now = *(struct internal_topic**)cvector_val_at(topics, i);
        if (!strcmp(now->topic_str, topic)) {
            return now;
        }
    }
    struct internal_topic* now = malloc(sizeof(struct internal_topic));
    now->pub = NULL;
    now->subs = cvector_create(sizeof(Subscriber*));
    now->topic_str = topic;
    cvector_pushback(topics,&now);
    return now;
}

Publisher* register_pub(const char* topic) {
    struct internal_topic* now_topic = register_topic(topic);
    if (now_topic->pub != NULL) return now_topic->pub;
    Publisher* obj = malloc(sizeof(Publisher));
    obj->pub_topic = topic;
    obj->topic = now_topic;
    obj->publish = pub_commit;
    now_topic->pub = obj;
    return obj;
}

Subscriber* register_sub(const char* topic, uint32_t buffer_len) {
    struct internal_topic* now_topic = register_topic(topic);
    Subscriber* obj = malloc(sizeof(Subscriber));
    obj->queue = create_circular_queue(sizeof(publish_data), buffer_len);
    cvector_pushback(now_topic->subs, &obj);
    obj->sub_topic = topic;
    obj->getdata = sub_get;
    return obj;
}