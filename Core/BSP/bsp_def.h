/**
 * @file           : bsp_def.h
 * @brief          : BSP层所有配置的宏定义
 * @Author         : 李鸣航
 * @Date           : 2022-05-01 20:45
 * @LastEditTime   : 2022-05-03 11:04
 * @Note           : 请仔细阅读注释，严禁私自更改，随意更改，否则后果自负
 * @Copyright(c)   : 哈尔滨工业大学（深圳）南工骁鹰机器人队版权所有 Critical HIT copyrighted
 */
#ifndef _BSP_H_
#define _BSP_H_

/**
 * @brief      :黑科技宏函数
 * @attention  :禁止更改
 */
#define LINK(a, b, c) a##_##b##_##c

/*--------------------------------------------------bsp_can--------------------------------------------------*/
/**
 * @brief      :can通信过滤器的最大数目
 * @attention  :此宏定义按照标准帧定义，无需更改
 */
#define FILTER_MAX_CNT (4 * 14)

/**
 * @brief      :主控共使用（拥有）的can总线数量
 * @attention  :如果使用A板或C板，此处值为2
 *              如果使用F7板子，此处值为3，且bsp_can.c中的内容也需要更改，此时应询问电控组长，禁止自行更改
 */
#define DEVICE_CAN_CNT 2

/**
 * @brief      :can线上能用到的最大的ID号
 * @attention  :一般用不到，无需更改
 */
#define ID_MAX 0x07FF

/**
 * @brief      :未使用can通信过滤器时候的过滤器填充值
 * @attention  :固定值，禁止更改
 */
#define ID_NOTSET 0x800

/*--------------------------------------------------bsp_gpio--------------------------------------------------*/
/**
 * @brief      :定义GPIO的输入与输出模式
 * @attention  :固定值，禁止更改
 */
#define GPIO_INPUT_MODE 0
#define GPIO_OUTPUT_MODE 1

/**
 * @brief      :主控用到的GPIO口的数目
 * @attention  :用到几个就填几个，如果更改的话需要在bsp_gpio.c中的函数里面加上额外的gpio配置，并添加如下的宏定义
 */
#define DEVICE_GPIO_CNT 2

/**
 * @brief      :gpio_ports[0]的宏定义配置，分别为引脚定义、输入还是输出模式
 * @attention  :gpio数目一定要和DEVICE_GPIO_CNT一致
 */
#define GPIO_0_BASE GPIOA
#define GPIO_0_PIN GPIO_PIN_4
#define GPIO_0_MODE GPIO_OUTPUT_MODE

/**
 * @brief      :gpio_ports[1]的宏定义配置，分别为引脚定义、输入还是输出模式
 * @attention  :gpio数目一定要和DEVICE_GPIO_CNT一致
 */
#define GPIO_1_BASE GPIOB
#define GPIO_1_PIN GPIO_PIN_0
#define GPIO_1_MODE GPIO_OUTPUT_MODE

/**
 * @brief      :APP层和HAL层会调用的GPIO口的宏定义
 * @attention  :0代表gpio_ports[0]，1代表gpio_ports[1]，以此类推
 */
#define GPIO_BMI088_ACCEL_NS 0
#define GPIO_BMI088_GYRO_NS 1

/*--------------------------------------------------bsp_pwm--------------------------------------------------*/
/**
 * @brief      :主控用到的PWM的数目
 * @attention  :用到几个就填几个，如果更改的话需要在bsp_pwm.c中的函数里面加上额外的pwm配置，并添加如下的宏定义
 */
#define DEVICE_PWM_CNT 3

/**
 * @brief      :用到DMA的PWM输出定时器口
 * @attention  :开哪个就填那个
 */
#define PWM_DMA_1 hdma_tim1_ch1

/**
 * @brief      :pwm_ports[0]的宏定义配置，分别为定时器索引、通道数
 * @attention  :pwm数目一定要和DEVICE_PWM_CNT一致
 *              0以后以此类推
 */
#define PWM_0_BASE &htim10
#define PWM_0_CHANNEL TIM_CHANNEL_1

#define PWM_1_BASE &htim4
#define PWM_1_CHANNEL TIM_CHANNEL_3

#define PWM_2_BASE &htim1
#define PWM_2_CHANNEL TIM_CHANNEL_1
#define PWM_3_BASE &htim1
#define PWM_3_CHANNEL TIM_CHANNEL_2
#define PWM_4_BASE &htim1
#define PWM_4_CHANNEL TIM_CHANNEL_3
#define PWM_5_BASE &htim1
#define PWM_5_CHANNEL TIM_CHANNEL_4
#define PWM_6_BASE &htim8
#define PWM_6_CHANNEL TIM_CHANNEL_1
#define PWM_7_BASE &htim8
#define PWM_7_CHANNEL TIM_CHANNEL_2
#define PWM_8_BASE &htim8
#define PWM_8_CHANNEL TIM_CHANNEL_3
/**
 * @brief      :APP层和HAL层会调用的PWM口的宏定义
 * @attention  :0代表pwm_ports[0]，1代表pwm_ports[1]，以此类推
 */
#define PWM_BMI088_HEAT_PORT 0
#define PWM_BUZZER_PORT 1

#define PWM_SERVO_1_PORT 2  // 从远离DBUS端开始数
#define PWM_SERVO_2_PORT 3
#define PWM_SERVO_3_PORT 4
#define PWM_SERVO_4_PORT 5
#define PWM_SERVO_5_PORT 6
#define PWM_SERVO_6_PORT 7
#define PWM_SERVO_7_PORT 8

/*--------------------------------------------------bsp_reset--------------------------------------------------*/
/**
 * @brief      :暂无需要更改的配置
 * @attention  :\\
 */

/*--------------------------------------------------bsp_spi--------------------------------------------------*/
/**
 * @brief      :主控用到的SPI的数目
 * @attention  :用到几个就填几个
 */
#define DEVICE_SPI_CNT 1

/**
 * @brief      :spi_ports[0]的宏定义配置，只有spi口
 * @attention  :spi数目一定要和DEVICE_PWM_CNT一致
 */
#define SPI_0_PORT &hspi1

/**
 * @brief      :APP层和HAL层会调用的SPI口的宏定义
 * @attention  :0代表spi_ports[0]，以此类推
 */
#define SPI_BMI088_PORT 0

/*--------------------------------------------------bsp_time--------------------------------------------------*/
/**
 * @brief      :暂无需要更改的配置
 * @attention  :\\
 */

/*--------------------------------------------------bsp_uart--------------------------------------------------*/
/**
 * @brief      :主控用到的UART的数目
 * @attention  :用到几个就填几个
 */
#define DEVICE_UART_CNT 2

/**
 * @brief      :串口的DMA缓冲区大小，一次传输数据不应超过该长度
 * @attention  :固定值，禁止修改
 */
#define BSP_UART_DMA_BUFF_SIZE 255

/**
 * @brief      :uart_ports[0]的宏定义配置，为定时器端口
 * @attention  :uart数目一定要和DEVICE_UART_CNT一致
 */
#define UART_0_PORT &huart3

/**
 * @brief      :uart_ports[1]的宏定义配置，为定时器端口
 * @attention  :uart数目一定要和DEVICE_UART_CNT一致
 */
#define UART_1_PORT &huart6

/**
 * @brief      :APP层和HAL层会调用的UART口的宏定义
 * @attention  :0代表uart_ports[0]，以此类推
 */
#define UART_REMOTE_PORT 0
#define UART_REFEREE_PORT 1

/*--------------------------------------------------bsp_delay--------------------------------------------------*/
/**
 * @brief      :自制delay函数用到的定时器
 * @attention  :用到哪个就填哪个
 */
#define DLY_TIM_Handle (&htim13)

/*--------------------------------------------------bsp_log--------------------------------------------------*/
/**
 * @brief      :log日志的记录索引
 * @attention  :TODO:注释待完善
 */
#define BUFFER_INDEX 0

#endif