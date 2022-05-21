#ifndef _IMU_DATA_H_
#define _IMU_DATA_H_

//此处欧拉角定义为 绕固定参考坐标轴旋转X-Y-Z 也就是 roll pitch yaw
typedef struct imu_data_t {
    //XYZ
    float accel[3];  //加速度
    float gyro[3];        // 角速度 弧度每秒
    float gyro_deg[3];    //角速度 度每秒
    //roll pitch yaw
    float euler[3];       // 欧拉角 弧度
    float euler_deg[3];   // 欧拉角 角度
    float euler_8192[3];  // 欧拉角 编码器版 0-8192

    int round;
    float yaw_8192_real;
} imu_data;

#endif