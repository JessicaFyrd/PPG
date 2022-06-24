/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

#include "main.h"
#include "Board.h"
#include "DSP.h"
#include "Acquisition.h"

uint8_t a = 0;

extern float32_t data_ir[LENGTH_WHOLE_DATA];			//All the offline raw data stocked in an other .c
extern float32_t data_1s_ir[LENGTH_DATA];
extern uint8_t flag_filter;

int main(void)
{
	STM_INIT();
	SENSOR_2LED_INIT();
	DSP_INIT();

	//Sensor changes
	heartrate10_A_FULL_EN(MAX86916_A_FULL_DIS);			//Disable interruption when watermark level is reached in FIFO
	heartrate10_SMP_RDY_EN(MAX86916_SMP_RDY_DIS);		//Disable interruption when a new sample is in FIFO

	while (1)
	{
		//Divide samples by blocks to simulate the buffer which will stock the 1s online data
		for (a=0;a<LENGTH_DATA;a++)
		{
			data_1s_ir[a]=(float32_t)data_ir[a];
		}

		//HR calculation activation
		flag_filter = 1;

		//Heart rate
		HEART_RATE_CALCULATION();
	}
}
