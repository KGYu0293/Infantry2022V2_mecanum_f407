#include "app.h"

#include "arm_math.h"
#include "bsp.h"
#include "bsp_random.h"
#include "hal.h"
#include "stdio.h"
#include "ws2812b.h"


// 定义共享外设
dt7Remote* remote;
BMI088_imu* imu;
buzzer* internal_buzzer;
canpc* pc;
// 电机创建命名规范：motor_位置_编号或用途，例如motor_chaiss_1 或 motor_chaiss_4 或 motor_gimbal_pitch
can_motor* motor_chaiss_1;
// #TODO to add other motors

//此处定义外设的配置文件，也可分开文件配置
dt7_config remote_dt7_config;
BMI088_config internal_imu_config;
buzzer_config internal_buzzer_config;
canpc_config pc_config;
can_motor_config motor_chaiss_1_config;
ws2812_config fanlight_config;
// #TODO to add other motors

void motor_lost_test(void* motor){
    printf_log("motor lost!\n");
    can_motor* now = (can_motor*) motor;
    now->monitor->reset(now->monitor);
}

void APP_Layer_Init() {
    // app层需要的外设配置设置
    // lost_callback设置掉线回调函数
    // remote
    remote_dt7_config.bsp_uart_index = UART_REMOTE_PORT;
    remote_dt7_config.lost_callback = NULL;

    // bmi088
    internal_imu_config.bsp_gpio_accel_index = GPIO_BMI088_ACCEL_NS;
    internal_imu_config.bsp_gpio_gyro_index = GPIO_BMI088_GYRO_NS;
    internal_imu_config.bsp_pwm_heat_index = PWM_BMI088_HEAT_PORT;
    internal_imu_config.bsp_spi_index = SPI_BMI088_PORT;
    internal_imu_config.temp_target = 55.0f;  //设定温度为55度
    internal_imu_config.lost_callback = NULL;

    // buzzer
    uint32_t music_id = 1;
    internal_buzzer_config.music = musics[music_id];
    internal_buzzer_config.len = music_lens[music_id];
    internal_buzzer_config.bsp_pwm_index = PWM_BUZZER_PORT;

    // PC
    pc_config.bsp_can_index = 1;
    pc_config.recv_identifer = 0x201;
    pc_config.send_identifer = 0x202;
    pc_config.lost_callback = NULL;


    // motors
    // example :bsp_can_index：can1填充0,can2填充1
    //          motor_set_id 直接填充电调上通过闪灯次数确定的id，如闪烁次数是1，则id是1，motor_set_id填1
    //          motor_model 填写MODEL_3508/MODEL_2006/MODEL_6020
    //          motor_pid_model 填写SPEED_LOOP/POSITION_LOOP/CURRENT_LOOP: 速度环/位置环/电流环
    //          speed_fdb_model 填写MOTOR_FDB/OTHER_FDB，表示使用电机自身的反馈或者使用其它的反馈模式，如果使用OTHER_FDB，需要填写speed_pid_fdb指针指向你的反馈数据，数据单位为度每秒，数据类型为float
    //          position_fdb_model 填写MOTOR_FDB/OTHER_FDB，表示使用电机自身的反馈或者使用其它的反馈模式，如果使用OTHER_FDB，需要填写position_pid_fdb指针指向你的反馈数据，数据单位为0.0439453(360/8192)度，数据类型为float
    motor_chaiss_1_config.bsp_can_index = 0;
    motor_chaiss_1_config.motor_set_id = 3;
    motor_chaiss_1_config.motor_model = MODEL_6020;
    motor_chaiss_1_config.motor_pid_model = SPEED_LOOP;
    motor_chaiss_1_config.speed_fdb_model = MOTOR_FDB;
    motor_chaiss_1_config.position_fdb_model = MOTOR_FDB;
    motor_chaiss_1_config.lost_callback = motor_lost_test;
    // pid参数初始化
    PID_SetConfig(&motor_chaiss_1_config.config_speed, 1, 0.4, 0, 2000, 5000);
    PID_SetConfig(&motor_chaiss_1_config.config_position, 1, 0, 0, 0, 0);

    //初始化app层需要的外设
    remote = dt7_Create(&remote_dt7_config);
    imu = BMI088_Create(&internal_imu_config);
    internal_buzzer = Buzzer_Create(&internal_buzzer_config);
    pc = CanPC_Create(&pc_config);
    motor_chaiss_1 = Can_Motor_Create(&motor_chaiss_1_config);

    // fan = Fanlight_APP_Init();
}

void APP_Layer_default_loop() {
    if (imu->bias_init_success) {
        Buzzer_Update(internal_buzzer);
    }
    // FanLight_Update(fan);
}

void APP_Log_Loop() {
    if (imu->bias_init_success) {
        // uint8_t buf[10] = "1234567812";
        // CanSend_Send(test_send, buf);
        // BSP_CAN_Send(1, 0x200, buf, 8);

        // static char fbufs[3][10];
        // Float2Str(fbufs[0], imu->data.euler_deg[0]);
        // Float2Str(fbufs[1], imu->data.euler_deg[1]);
        // Float2Str(fbufs[2], imu->data.euler_deg[2]);
        // printf_log("%s %s %s\n", fbufs[0], fbufs[1], fbufs[2]);

        // printf_log("test_log\n");
    }
}

