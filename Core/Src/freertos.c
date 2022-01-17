/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * File Name          : freertos.c
 * Description        : Code for freertos applications
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under Ultimate Liberty license
 * SLA0044, the "License"; You may not use this file except in compliance with
 * the License. You may obtain a copy of the License at:
 *                             www.st.com/SLA0044
 *
 ******************************************************************************
 */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "app.h"
#include "bsp.h"
#include "hal.h"
#include "stdio.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for ImuTask */
osThreadId_t ImuTaskHandle;
const osThreadAttr_t ImuTask_attributes = {
  .name = "ImuTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for LogTask */
osThreadId_t LogTaskHandle;
const osThreadAttr_t LogTask_attributes = {
  .name = "LogTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for MotorTask */
osThreadId_t MotorTaskHandle;
const osThreadAttr_t MotorTask_attributes = {
  .name = "MotorTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for MonitorTask */
osThreadId_t MonitorTaskHandle;
const osThreadAttr_t MonitorTask_attributes = {
  .name = "MonitorTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for RobotCMDTask */
osThreadId_t RobotCMDTaskHandle;
const osThreadAttr_t RobotCMDTask_attributes = {
  .name = "RobotCMDTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for SuperCapTask */
osThreadId_t SuperCapTaskHandle;
const osThreadAttr_t SuperCapTask_attributes = {
  .name = "SuperCapTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void StartImuTask(void *argument);
void StartLogTask(void *argument);
void StartMotorTask(void *argument);
void StartMonitorTask(void *argument);
void StartRobotCMDTask(void *argument);
void StartCapTask(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
    /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
    /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
    /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
    /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of ImuTask */
  ImuTaskHandle = osThreadNew(StartImuTask, NULL, &ImuTask_attributes);

  /* creation of LogTask */
  LogTaskHandle = osThreadNew(StartLogTask, NULL, &LogTask_attributes);

  /* creation of MotorTask */
  MotorTaskHandle = osThreadNew(StartMotorTask, NULL, &MotorTask_attributes);

  /* creation of MonitorTask */
  MonitorTaskHandle = osThreadNew(StartMonitorTask, NULL, &MonitorTask_attributes);

  /* creation of RobotCMDTask */
  RobotCMDTaskHandle = osThreadNew(StartRobotCMDTask, NULL, &RobotCMDTask_attributes);

  /* creation of SuperCapTask */
  SuperCapTaskHandle = osThreadNew(StartCapTask, NULL, &SuperCapTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
    /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
    /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
 * @brief  Function implementing the defaultTask thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
    /* Infinite loop */
    // Buzzer_Init(&internal_buzzer,music2,14);
    portTickType currentTimeDefault;
    currentTimeDefault = xTaskGetTickCount();
    for (;;) {
        APP_Layer_default_loop();
        vTaskDelayUntil(&currentTimeDefault, 100);
    }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_StartImuTask */
/**
 * @brief Function implementing the ImuTask thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartImuTask */
void StartImuTask(void *argument)
{
  /* USER CODE BEGIN StartImuTask */
    /* Infinite loop */
    portTickType currentTimeImu;
    currentTimeImu = xTaskGetTickCount();
    for (;;) {
        HAL_Imu_Loop();
        vTaskDelayUntil(&currentTimeImu, 2);
    }
  /* USER CODE END StartImuTask */
}

/* USER CODE BEGIN Header_StartLogTask */
/**
 * @brief Function implementing the LogTask thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartLogTask */
void StartLogTask(void *argument)
{
  /* USER CODE BEGIN StartLogTask */
    portTickType currentTimeLog;
    currentTimeLog = xTaskGetTickCount();
    for (;;) {
        APP_Log_Loop();
        vTaskDelayUntil(&currentTimeLog, 10);
    }
  /* USER CODE END StartLogTask */
}

/* USER CODE BEGIN Header_StartMotorTask */
/**
* @brief Function implementing the MotorTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartMotorTask */
void StartMotorTask(void *argument)
{
  /* USER CODE BEGIN StartMotorTask */
  portTickType currentTimeMotor;
  currentTimeMotor = xTaskGetTickCount();
  /* Infinite loop */
  for(;;){
    HAL_Motor_Calc_Loop();
    vTaskDelayUntil(&currentTimeMotor, 1);
  }
  /* USER CODE END StartMotorTask */
}

/* USER CODE BEGIN Header_StartMonitorTask */
/**
* @brief Function implementing the MonitorTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartMonitorTask */
void StartMonitorTask(void *argument)
{
  /* USER CODE BEGIN StartMonitorTask */
  /* Infinite loop */
  portTickType currentTimeMonitor;
  currentTimeMonitor = xTaskGetTickCount();
  for(;;)
  {
    Monitor_Loop();
    vTaskDelayUntil(&currentTimeMonitor, 100);
  }
  /* USER CODE END StartMonitorTask */
}

/* USER CODE BEGIN Header_StartRobotCMDTask */
/**
* @brief Function implementing the RobotCMDTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartRobotCMDTask */
void StartRobotCMDTask(void *argument)
{
  /* USER CODE BEGIN StartRobotCMDTask */
  /* Infinite loop */
  portTickType currentTimeRobotCMD;
  currentTimeRobotCMD = xTaskGetTickCount();
  for(;;)
  {
    APP_RobotCmd_Loop();
    vTaskDelayUntil(&currentTimeRobotCMD, 2);
  }
  /* USER CODE END StartRobotCMDTask */
}

/* USER CODE BEGIN Header_StartCapTask */
/**
* @brief Function implementing the SuperCapTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartCapTask */
void StartCapTask(void *argument)
{
  /* USER CODE BEGIN StartCapTask */
  /* Infinite loop */
  portTickType currentTimeSuperCap;
  currentTimeSuperCap = xTaskGetTickCount();
  for(;;)
  {
    HAL_Super_cap_wuli_Loop();
    vTaskDelayUntil(&currentTimeSuperCap, 2);
  }
  /* USER CODE END StartCapTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

