/**
  ******************************************************************************
  * �ļ���        ��supervise.h
	* ����ʱ��      ��2019.11.10
	* ����          ��лʤ��������
	*-----------------------------------------------------------------------------
	* ����޸�ʱ��  ��2019.11.10
	* �޸���        ��������
  ******************************************************************************
	* 1.���������STM32F427IIH6��������̻���λKeil 5������FreeRTOS���п���
	* 2.������ֻ������Robomaster������
	* 3.������δ��������ֹ˽��ת������ֹ˽��ʹ��
	* 4.�����������������ע�ͣ�����ANSI�����ʽ��
	* 5.���������ս���Ȩ���������ҵ��ѧ�����ڣ��Ϲ���ӥս��Critical HIT����
	* 
	* Copyright (c) ��������ҵ��ѧ�����ڣ��Ϲ���ӥս��Critical HIT ��Ȩ����
	******************************************************************************
  */

#ifndef _SUPERVISE_H_
#define _SUPERVISE_H_

#include "gpio.h"

// #define Heat_Test   //����¶ȼ�غ궨��

#define IMU_INDEX 11
#define PC_INDEX 12
#define BOARD_INDEX 13
#define PC_CHASSIS_INDEX 14
#define DETECT_NUM 16u

typedef struct 
{
	short count;
	short FPS;
	int now;
	int last;
}FPS_t;

typedef struct 
{
	FPS_t FPS_other1;
	FPS_t FPS_other2;
	FPS_t FPS_Cap;
	FPS_t FPS_RobotTask;
	FPS_t FPS_IMU;
	FPS_t FPS_controller;
	FPS_t FPS_PC;
	FPS_t FPS_board;
	FPS_t FPS_Judge;
}FrameRate_t;

typedef struct
{
	uint8_t RC_lost;
	uint8_t IMU_lost;
	uint8_t PC_lost;
	uint8_t board_lost;
	uint8_t Chassis_lost;
	uint8_t Gimbal_lost;
	uint8_t Shoot_lost;
	uint8_t Heat_lost;
	uint8_t PC_Chassis_lost;
	FrameRate_t FrameRate; //֡��ͳ��
}Monitor_t;

void SuperVise(void);
void LostCounterFeed(int index);
void LostCounterFeed_Remote(void);
void FrameRateStatistics(FPS_t *FPS);

extern Monitor_t Monitor;

#endif
