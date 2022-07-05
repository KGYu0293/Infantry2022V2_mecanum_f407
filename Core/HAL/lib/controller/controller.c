#include <controller.h>
#include <stdlib.h>
#include <string.h>
void controller_calc(controller* obj) {
    if (obj->config.control_type == PID_MODEL) {
        if (obj->config.control_depth >= POS_CONTROL) {
            obj->pid_pos_data.fdb = obj->fdb_position;
            obj->pid_pos_data.ref = obj->ref_position;
            PID_Calc(&obj->pid_pos_data);
            obj->ref_speed = obj->pid_pos_data.output;
        }
        if (obj->config.control_depth >= SPEED_CONTROL) {
            obj->pid_speed_data.fdb = obj->fdb_speed;
            obj->pid_speed_data.ref = obj->ref_speed;
            PID_Calc(&obj->pid_speed_data);
            obj->output = obj->pid_speed_data.output;
        }
    } else if (obj->config.control_type == MRAC_MODEL) {
        mrac_2d_calc(&obj->mrac_2d_data, obj->ref_position, obj->fdb_position, obj->fdb_speed, 1);
        obj->output = obj->mrac_2d_data.output;
    } else if (obj->config.control_type == ADRC_MODEL) {
        if (obj->config.control_depth >= POS_CONTROL) {
            obj->adrc_pos_data.prog.fdb = obj->fdb_position;
            obj->adrc_pos_data.prog.ref = obj->ref_position;
            ADRCFunction(&obj->adrc_pos_data);
            obj->ref_speed = obj->adrc_pos_data.prog.output;
        }
        if (obj->config.control_depth >= SPEED_CONTROL) {
            obj->adrc_speed_data.prog.fdb = obj->fdb_speed;
            obj->adrc_speed_data.prog.ref = obj->ref_speed;
            ADRCFunction(&obj->adrc_speed_data);
            obj->output = obj->adrc_speed_data.prog.output;
        }
    }
}

controller* create_controller(controller_config* _config) {
    controller* obj = malloc(sizeof(controller));
    memset(obj, 0, sizeof(controller));
    obj->config = *_config;
    if (obj->config.control_type == MRAC_MODEL) {
        // obj->mrac_2d_data = obj->config.mrac_2d_init;
        mrac_Init(&obj->mrac_2d_data, &obj->config.mrac_config);
    }
    if (obj->config.control_type == PID_MODEL) {
        obj->pid_pos_data.config = obj->config.position_pid_config;
        obj->pid_speed_data.config = obj->config.speed_pid_config;
    }
    if (obj->config.control_type == ADRC_MODEL) {
        obj->adrc_pos_data.adrc_config = obj->config.pos_adrc_config;
        obj->adrc_speed_data.adrc_config = obj->config.speed_adrc_config;
        // memset(&obj->adrc_speed_data, 0, sizeof(obj->adrc_speed_data));
        // memset(&obj->adrc_pos_data, 0, sizeof(obj->adrc_pos_data));
    }
    return obj;
}