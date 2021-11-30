#include "app.h"
#include "stm32f4xx_hal.h"
#include "gpio.h"
#include "spi.h"

BMI088_imu* imu;
buzzer* internal_buzzer;

BMI088_config internal_imu_config;
buzzer_config internal_buzzer_config;

void APP_Layer_Init(){
    //app层外设设置
    
    //bmi088
    internal_imu_config.ACCEL_NS_BASE = GPIOA;
    internal_imu_config.ACCEL_NS_PIN = GPIO_PIN_4;
    internal_imu_config.GYRO_NS_BASE = GPIOB;
    internal_imu_config.GYRO_NS_PIN = GPIO_PIN_0;
    internal_imu_config.SPI_PORT = &hspi1;
    internal_imu_config.HEAT_PWM_BASE = &htim10;
    internal_imu_config.HAET_PWM_CHANNEL = TIM_CHANNEL_1;
    internal_imu_config.temp_target = 55.0f; //设定温度为55度

    //buzzer
    internal_buzzer_config.music = music2;
    internal_buzzer_config.len = 14;
    
    //初始化app层外设
    imu = BMI088_Create(&internal_imu_config);
    internal_buzzer = Buzzer_Create(&internal_buzzer_config);
}
void APP_Layer_default_loop(){
    if(imu->bias_init_success){
        Buzzer_Update(internal_buzzer);
    }
}

void APP_Log_Loop(){
    if(imu->bias_init_success){
        // Do some send
    }
}