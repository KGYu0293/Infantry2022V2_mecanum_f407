#ifndef _EXCEPTIONS_H
#define _EXCEPTIONS_H

void motor_lost(void* motor);
void imu_lost(void* imu);
void referee_lost(void* referee);
void remote_lost(void* remote);

#endif