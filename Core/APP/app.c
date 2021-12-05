#include "app.h"

#include "bsp.h"
#include "bsp_random.h"
#include "hal.h"

BMI088_imu* imu;
buzzer* internal_buzzer;
canpc* pc;

BMI088_config internal_imu_config;
buzzer_config internal_buzzer_config;
canpc_config pc_config;

void APP_Layer_Init() {
    // app层需要的外设配置设置

    // bmi088
    internal_imu_config.bsp_gpio_accel_index = GPIO_BMI088_ACCEL_NS;
    internal_imu_config.bsp_gpio_gyro_index = GPIO_BMI088_GYRO_NS;
    internal_imu_config.bsp_pwm_heat_index = PWM_BMI088_HEAT_PORT;
    internal_imu_config.bsp_spi_index = SPI_BMI088_PORT;
    internal_imu_config.temp_target = 55.0f;  //设定温度为55度

    // buzzer
    uint32_t music_id = GetRand_Int() % 7;
    internal_buzzer_config.music = musics[music_id]; 
    internal_buzzer_config.len = music_lens[music_id];
    internal_buzzer_config.bsp_pwm_index = PWM_BUZZER_PORT;

    //PC
    pc_config.bsp_can_index = 1;
    pc_config.recv_identifer = 0x201;
    pc_config.send_identifer = 0x202;

    //初始化app层需要的外设
    imu = BMI088_Create(&internal_imu_config);
    internal_buzzer = Buzzer_Create(&internal_buzzer_config);
    pc = CanPC_Create(&pc_config);
}

void APP_Layer_default_loop() {
    if (imu->bias_init_success) {
        // Buzzer_Update(internal_buzzer);
    }
}

void APP_Log_Loop() {
    if (imu->bias_init_success) {
        // uint8_t buf[10] = "1234567812";
        // CanSend_Send(test_send, buf);
        // BSP_CAN_Send(1, 0x200, buf, 8);
    }
}