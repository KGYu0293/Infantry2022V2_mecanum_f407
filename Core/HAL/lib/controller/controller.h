#ifndef _CONTROLLER_H
#define _CONTROLLER_H
#include <pid.h>

enum controller_type_e { PID_MODEL = 0,
                         MRAC_MODEL };

typedef struct controller_config_t {
    enum controller_type_e control_type;
} controller_config;

#endif