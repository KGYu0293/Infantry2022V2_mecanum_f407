#include "BMI088.h"

#include "BMI088def.h"
#include "BMI088reg.h"
#include "bsp_delay.h"
#include "bsp_gpio.h"
#include "bsp_pwm.h"
#include "bsp_random.h"
#include "bsp_spi.h"
#include "common.h"
#include "cvector.h"
#include "stdio.h"
#include "string.h"
#include "bsp_log.h"

////BMI088全局配置
#define BMI088_ACCEL_SEN BMI088_ACCEL_3G_SEN
#define BMI088_GYRO_SEN BMI088_GYRO_2000_SEN
#define BMI088_BIAS_INIT_DISCARD 1000
#define BMI088_BIAS_INIT_COUNT 2000
//加热PID配置
#define HEAT_PID_KP 1600.0f       // kp of temperature control PID
#define HEAT_PID_KI 0.2f          // ki of temperature control PID
#define HEAT_PID_KD 0.0f          // kd of temperature control PID
#define HEAT_PID_MAX_OUT 1000.0f  // max out of temperature control PID
#define HEAT_PID_MAX_IOUT 900.0f  // max out of temperature control PID
// BMI088加速度计配置数组
static uint8_t BMI088_accel_config[BMI088_WRITE_ACCEL_REG_NUM][3] = {
    {BMI088_ACC_PWR_CTRL, BMI088_ACC_ENABLE_ACC_ON, BMI088_ACC_PWR_CTRL_ERROR},
    {BMI088_ACC_PWR_CONF, BMI088_ACC_PWR_ACTIVE_MODE,
     BMI088_ACC_PWR_CONF_ERROR},
    {BMI088_ACC_CONF,
     BMI088_ACC_NORMAL | BMI088_ACC_800_HZ | BMI088_ACC_CONF_MUST_Set,
     BMI088_ACC_CONF_ERROR},
    {BMI088_ACC_RANGE, BMI088_ACC_RANGE_3G, BMI088_ACC_RANGE_ERROR},
    {BMI088_INT1_IO_CTRL,
     BMI088_ACC_INT1_IO_ENABLE | BMI088_ACC_INT1_GPIO_PP |
         BMI088_ACC_INT1_GPIO_LOW,
     BMI088_INT1_IO_CTRL_ERROR},
    {BMI088_INT_MAP_DATA, BMI088_ACC_INT1_DRDY_INTERRUPT,
     BMI088_INT_MAP_DATA_ERROR}

};
// BMI088陀螺仪配置数组
static uint8_t BMI088_gyro_config[BMI088_WRITE_GYRO_REG_NUM][3] = {
    {BMI088_GYRO_RANGE, BMI088_GYRO_2000, BMI088_GYRO_RANGE_ERROR},
    {BMI088_GYRO_BANDWIDTH,
     BMI088_GYRO_1000_116_HZ | BMI088_GYRO_BANDWIDTH_MUST_Set,
     BMI088_GYRO_BANDWIDTH_ERROR},
    {BMI088_GYRO_LPM1, BMI088_GYRO_NORMAL_MODE, BMI088_GYRO_LPM1_ERROR},
    {BMI088_GYRO_CTRL, BMI088_DRDY_ON, BMI088_GYRO_CTRL_ERROR},
    {BMI088_GYRO_INT3_INT4_IO_CONF,
     BMI088_GYRO_INT3_GPIO_PP | BMI088_GYRO_INT3_GPIO_LOW,
     BMI088_GYRO_INT3_INT4_IO_CONF_ERROR},
    {BMI088_GYRO_INT3_INT4_IO_MAP, BMI088_GYRO_DRDY_IO_INT3,
     BMI088_GYRO_INT3_INT4_IO_MAP_ERROR}

};

cvector *bmi088_instances;

//成员函数
uint8_t BMI088_init(BMI088_imu *obj);
void BMI088_accel_init(BMI088_imu *obj);
void BMI088_gyro_init(BMI088_imu *obj);
void BMI088_heat_init(BMI088_imu *obj);
void BMI088_heat_control(BMI088_imu *obj);

void BMI088_read_raw(BMI088_imu *obj);

// void BMI088_ACCEL_NS_L(BMI088_imu *obj);
// void BMI088_ACCEL_NS_H(BMI088_imu *obj);
// void BMI088_GYRO_NS_L(BMI088_imu *obj);
// void BMI088_GYRO_NS_H(BMI088_imu *obj);

void BMI088_accel_read(BMI088_imu *obj, uint8_t reg, uint8_t *buf, uint8_t len);
void BMI088_accel_read_single(BMI088_imu *obj, uint8_t reg, uint8_t *buf);
void BMI088_accel_write(BMI088_imu *obj, uint8_t reg, uint8_t data);
void BMI088_gyro_read(BMI088_imu *obj, uint8_t reg, uint8_t *buf, uint8_t len);
void BMI088_gyro_read_single(BMI088_imu *obj, uint8_t reg, uint8_t *buf);
void BMI088_gyro_write(BMI088_imu *obj, uint8_t reg, uint8_t data);

//静态成员函数
uint8_t spi_read_write_byte(uint8_t spi_index, uint8_t txdata);
void spi_read_multi(uint8_t spi_index, uint8_t reg, uint8_t *buf, uint8_t len);
void spi_write_byte(uint8_t spi_index, uint8_t reg, uint8_t data);

//驱动初始化
void BMI088_Driver_Init() {
    //指针数组
    bmi088_instances = cvector_create(sizeof(BMI088_imu *));
}

//更新所有的IMU
void BMI088_Update_All() {
    for (size_t i = 0; i < bmi088_instances->cv_len; ++i) {
        BMI088_imu *imu_now =
            *((BMI088_imu **)cvector_val_at(bmi088_instances, i));
        BMI088_Update(imu_now);
    }
}

// BMI088构造函数
BMI088_imu *BMI088_Create(BMI088_config *config) {
    BMI088_imu *obj = (BMI088_imu *)malloc(sizeof(BMI088_imu));
    obj->config = *config;
    while (BMI088_init(obj))
        ;
    cvector_pushback(bmi088_instances, &obj);
    return obj;
}

//初始化BMI088
uint8_t BMI088_init(BMI088_imu *obj) {
    //初始化标志位
    obj->init_error = BMI088_NO_ERROR;
    obj->bias_init_success = 0;
    //分别初始化
    BMI088_accel_init(obj);
    BMI088_gyro_init(obj);
    BMI088_heat_init(obj);
    memset(obj->gyrobias, 0, sizeof(float) * 3);
    MahonyAHRS_init(&obj->mahony_solver, 2 * 1.0, 2 * 0.0001, 500.0f);
    MadgwickAHRS_init(&obj->madgwick_solver, 0.00, 500.0f);
    return obj->init_error;
}
// BMI088更新函数
void BMI088_Update(BMI088_imu *obj) {
    BMI088_read_raw(obj);
    BMI088_heat_control(obj);
    if (obj->temp < 52) {
        obj->bias_init_success = 0;
        return;
    }
    //获取陀螺仪静态误差
    if (!obj->bias_init_success) {
        static uint16_t init_count = 0;
        ++init_count;
        if (init_count < BMI088_BIAS_INIT_DISCARD)
            return;
        else if (init_count <
                 BMI088_BIAS_INIT_COUNT + BMI088_BIAS_INIT_DISCARD) {
            obj->gyrobias[0] += obj->data.gyro[0];
            obj->gyrobias[1] += obj->data.gyro[1];
            obj->gyrobias[2] += obj->data.gyro[2];
        } else if (init_count ==
                   BMI088_BIAS_INIT_COUNT + BMI088_BIAS_INIT_DISCARD) {
            obj->gyrobias[0] /= BMI088_BIAS_INIT_COUNT;
            obj->gyrobias[1] /= BMI088_BIAS_INIT_COUNT;
            obj->gyrobias[2] /= BMI088_BIAS_INIT_COUNT;
            obj->bias_init_success = 1;

        } else {
            init_count = 0;
        }
        return;
    }
    //校准静态误差
    obj->data.gyro[0] -= obj->gyrobias[0];
    obj->data.gyro[1] -= obj->gyrobias[1];
    obj->data.gyro[2] -= obj->gyrobias[2];

    // Mahony算法姿态解算
    MahonyAHRS_update(&obj->mahony_solver, obj->data.gyro[0], obj->data.gyro[1],
                      obj->data.gyro[2], obj->data.accel[0], obj->data.accel[1],
                      obj->data.accel[2]);
    memcpy(obj->data.euler, obj->mahony_solver.euler, sizeof(float) * 3);
    obj->data.euler_deg[0] = obj->data.euler[0] * RAD2DEG;
    obj->data.euler_deg[1] = obj->data.euler[1] * RAD2DEG;
    obj->data.euler_deg[2] = obj->data.euler[2] * RAD2DEG;
    // MadgwickAHRS_update(&obj->madgwick_solver, obj->data.gyro[0],
    // obj->data.gyro[1], obj->data.gyro[2], obj->data.accel[0],
    // obj->data.accel[1], obj->data.accel[2]); memcpy(obj->data.euler,
    // obj->madgwick_solver.euler, sizeof(float) * 3); printf("imu: %.2lf %.2lf
    // %.2lf %.2lf\n", imu.data.euler[0] * RAD2DEG, imu.data.euler[1] * RAD2DEG,
    // imu.data.euler[2] * RAD2DEG, imu.temp);
}

// BMI088加速度计初始化
void BMI088_accel_init(BMI088_imu *obj) {
    uint8_t res = 0;
    // dummy read
    BMI088_accel_read_single(obj, BMI088_ACC_CHIP_ID, &res);
    bsp_delay_us(BMI088_COM_WAIT_SENSOR_TIME);

    // software reset
    BMI088_accel_write(obj, BMI088_ACC_SOFTRESET, BMI088_ACC_SOFTRESET_VALUE);
    bsp_delay_ms(100);
    // check normal

    // dummy read
    BMI088_accel_read_single(obj, BMI088_ACC_CHIP_ID, &res);
    bsp_delay_us(BMI088_COM_WAIT_SENSOR_TIME);
    // get chip ID
    BMI088_accel_read_single(obj, BMI088_ACC_CHIP_ID, &res);
    bsp_delay_us(BMI088_COM_WAIT_SENSOR_TIME);

    if (res != BMI088_ACC_CHIP_ID_VALUE) {
        obj->init_error |= BMI088_NO_SENSOR;
        return;
    }
    for (int i = 0; i < BMI088_WRITE_ACCEL_REG_NUM; ++i) {
        BMI088_accel_write(obj, BMI088_accel_config[i][0],
                           BMI088_accel_config[i][1]);
        bsp_delay_us(BMI088_COM_WAIT_SENSOR_TIME);

        BMI088_accel_read_single(obj, BMI088_accel_config[i][0], &res);
        bsp_delay_us(BMI088_COM_WAIT_SENSOR_TIME);
        if (res != BMI088_accel_config[i][1]) {
            obj->init_error |= BMI088_accel_config[i][2];
            return;
        }
    }
    printf_log("BMI088 accel init success!\n");
}
// BMI088陀螺仪初始化
void BMI088_gyro_init(BMI088_imu *obj) {
    uint8_t res = 0;
    BMI088_gyro_read_single(obj, BMI088_GYRO_CHIP_ID, &res);
    bsp_delay_us(BMI088_COM_WAIT_SENSOR_TIME);

    BMI088_gyro_write(obj, BMI088_GYRO_SOFTRESET, BMI088_GYRO_SOFTRESET_VALUE);
    bsp_delay_ms(100);

    BMI088_gyro_read_single(obj, BMI088_GYRO_CHIP_ID, &res);
    bsp_delay_us(BMI088_COM_WAIT_SENSOR_TIME);

    if (res != BMI088_GYRO_CHIP_ID_VALUE) {
        obj->init_error |= BMI088_NO_SENSOR;
        return;
    }
    for (int i = 0; i < BMI088_WRITE_GYRO_REG_NUM; ++i) {
        BMI088_gyro_write(obj, BMI088_gyro_config[i][0],
                          BMI088_gyro_config[i][1]);
        bsp_delay_us(BMI088_COM_WAIT_SENSOR_TIME);

        BMI088_gyro_read_single(obj, BMI088_gyro_config[i][0], &res);
        bsp_delay_us(BMI088_COM_WAIT_SENSOR_TIME);
        if (res != BMI088_gyro_config[i][1]) {
            obj->init_error |= BMI088_gyro_config[i][2];
            return;
        }
    }
    printf_log("BMI088 gyro init success!\n");
}
// BMI088温控初始化
void BMI088_heat_init(BMI088_imu *obj) {
    // HAL_TIM_PWM_Start(obj->HEAT_PWM_BASE, obj->HAET_PWM_CHANNEL);
    BSP_PWM_Start(obj->config.bsp_pwm_heat_index);
    memset(&obj->heat_pid, 0, sizeof(struct PID_t));
    struct PID_config_t bmi088_config;
    bmi088_config.PID_Mode = PID_POSITION;
    bmi088_config.KP = HEAT_PID_KP;
    bmi088_config.KI = HEAT_PID_KI;
    bmi088_config.KD = HEAT_PID_KD;
    bmi088_config.error_max = 2048;
    bmi088_config.outputMax = HEAT_PID_MAX_OUT;
    bmi088_config.error_max = HEAT_PID_MAX_IOUT;
    PID_Init(&obj->heat_pid,&bmi088_config);
    obj->heat_pid.ref = obj->config.temp_target;
}

void BMI088_heat_control(BMI088_imu *obj) {
    obj->heat_pid.fdb = obj->temp;
    PID_Calc(&obj->heat_pid);
    // __HAL_TIM_SetCompare(obj->HEAT_PWM_BASE, obj->HAET_PWM_CHANNEL,
    //                      (uint16_t)(obj->heat_pid.output));
    BSP_PWM_SetCCR(obj->config.bsp_pwm_heat_index,
                   (uint16_t)(obj->heat_pid.output));
}

// BMI088读取函数
void BMI088_read_raw(BMI088_imu *obj) {
    uint8_t buf[8] = {0};
    int16_t tmp;

    //加速度
    BMI088_accel_read(obj, BMI088_ACCEL_XOUT_L, buf, 6);
    tmp = (int16_t)((buf[1]) << 8) | buf[0];
    obj->data.accel[0] = tmp * BMI088_ACCEL_SEN;
    tmp = (int16_t)((buf[3]) << 8) | buf[2];
    obj->data.accel[1] = tmp * BMI088_ACCEL_SEN;
    tmp = (int16_t)((buf[5]) << 8) | buf[4];
    obj->data.accel[2] = tmp * BMI088_ACCEL_SEN;

    //陀螺仪
    BMI088_gyro_read(obj, BMI088_GYRO_CHIP_ID, buf, 8);
    tmp = (int16_t)((buf[3]) << 8) | buf[2];
    obj->data.gyro[0] = tmp * BMI088_GYRO_SEN;
    tmp = (int16_t)((buf[5]) << 8) | buf[4];
    obj->data.gyro[1] = tmp * BMI088_GYRO_SEN;
    tmp = (int16_t)((buf[7]) << 8) | buf[6];
    obj->data.gyro[2] = tmp * BMI088_GYRO_SEN;

    //温度
    BMI088_accel_read(obj, BMI088_TEMP_M, buf, 2);
    tmp = (int16_t)((buf[0] << 3) | (buf[1] >> 5));
    if (tmp > 1023) tmp -= 2048;
    obj->temp = tmp * BMI088_TEMP_FACTOR + BMI088_TEMP_OFFSET;
}

// //片选信号函数
// void BMI088_ACCEL_NS_L(BMI088_imu *obj) {
//     HAL_GPIO_WritePin(obj->ACCEL_NS_BASE, obj->ACCEL_NS_PIN, GPIO_PIN_RESET);
// }
// void BMI088_ACCEL_NS_H(BMI088_imu *obj) {
//     HAL_GPIO_WritePin(obj->ACCEL_NS_BASE, obj->ACCEL_NS_PIN, GPIO_PIN_SET);
// }
// void BMI088_GYRO_NS_L(BMI088_imu *obj) {
//     HAL_GPIO_WritePin(obj->GYRO_NS_BASE, obj->GYRO_NS_PIN, GPIO_PIN_RESET);
// }
// void BMI088_GYRO_NS_H(BMI088_imu *obj) {
//     HAL_GPIO_WritePin(obj->GYRO_NS_BASE, obj->GYRO_NS_PIN, GPIO_PIN_SET);
// }

//辅助读/写函数
void BMI088_accel_read(BMI088_imu *obj, uint8_t reg, uint8_t *buf,
                       uint8_t len) {
    // BMI088_ACCEL_NS_L(obj);
    BSP_GPIO_Set(obj->config.bsp_gpio_accel_index, 0);
    spi_read_write_byte(obj->config.bsp_spi_index, reg | 0x80);
    spi_read_multi(obj->config.bsp_spi_index, reg, buf, len);
    BSP_GPIO_Set(obj->config.bsp_gpio_accel_index, 1);
    // BMI088_ACCEL_NS_H(obj);
}
void BMI088_accel_read_single(BMI088_imu *obj, uint8_t reg, uint8_t *buf) {
    // BMI088_ACCEL_NS_L(obj);
    BSP_GPIO_Set(obj->config.bsp_gpio_accel_index, 0);
    spi_read_write_byte(obj->config.bsp_spi_index, reg | 0x80);
    spi_read_write_byte(obj->config.bsp_spi_index, 0x55);
    *buf = spi_read_write_byte(obj->config.bsp_spi_index, 0x55);
    BSP_GPIO_Set(obj->config.bsp_gpio_accel_index, 1);
    // BMI088_ACCEL_NS_H(obj);
}
void BMI088_accel_write(BMI088_imu *obj, uint8_t reg, uint8_t data) {
    // BMI088_ACCEL_NS_L(obj);
    BSP_GPIO_Set(obj->config.bsp_gpio_accel_index, 0);
    spi_write_byte(obj->config.bsp_spi_index, reg, data);
    BSP_GPIO_Set(obj->config.bsp_gpio_accel_index, 1);
    // BMI088_ACCEL_NS_H(obj);
}
void BMI088_gyro_read(BMI088_imu *obj, uint8_t reg, uint8_t *buf, uint8_t len) {
    // BMI088_GYRO_NS_L(obj);
    BSP_GPIO_Set(obj->config.bsp_gpio_gyro_index, 0);
    spi_read_multi(obj->config.bsp_spi_index, reg, buf, len);
    BSP_GPIO_Set(obj->config.bsp_gpio_gyro_index, 1);
    // BMI088_GYRO_NS_H(obj);
}
void BMI088_gyro_read_single(BMI088_imu *obj, uint8_t reg, uint8_t *buf) {
    // BMI088_GYRO_NS_L(obj);
    BSP_GPIO_Set(obj->config.bsp_gpio_gyro_index, 0);
    spi_read_write_byte(obj->config.bsp_spi_index, reg | 0x80);
    *buf = spi_read_write_byte(obj->config.bsp_spi_index, 0x55);
    BSP_GPIO_Set(obj->config.bsp_gpio_gyro_index, 1);
    // BMI088_GYRO_NS_H(obj);
}
void BMI088_gyro_write(BMI088_imu *obj, uint8_t reg, uint8_t data) {
    // BMI088_GYRO_NS_L(obj);
    BSP_GPIO_Set(obj->config.bsp_gpio_gyro_index, 0);
    spi_write_byte(obj->config.bsp_spi_index, reg, data);
    BSP_GPIO_Set(obj->config.bsp_gpio_gyro_index, 1);
    // BMI088_GYRO_NS_H(obj);
}

uint8_t spi_read_write_byte(uint8_t spi_index, uint8_t txdata) {
    uint8_t rx_data;
    BSP_SPI_TransmitReceive(spi_index, &txdata, &rx_data, 1, 1000);
    return rx_data;
}
void spi_read_multi(uint8_t spi_index, uint8_t reg, uint8_t *buf, uint8_t len) {
    spi_read_write_byte(spi_index, reg | 0x80);
    while (len != 0) {
        *buf = spi_read_write_byte(spi_index, 0x55);
        ++buf;
        --len;
    }
}
void spi_write_byte(uint8_t spi_index, uint8_t reg, uint8_t data) {
    spi_read_write_byte(spi_index, reg);
    spi_read_write_byte(spi_index, data);
}
