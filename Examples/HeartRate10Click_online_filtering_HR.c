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
  Author: Jessica Fayard
  */
/* USER CODE END Header */

#include "main.h"
#include "MAX86916_ppg.h"
#include "DSP.h"
#include "Board.h"

int main(void)
{
	STM_INIT();
	SENSOR_2LED_INIT();
	DSP_INIT();

//	//Enable the cycle counter to check duration
//	CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
//	DWT->CYCCNT = 0;
//	DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

	while (1)
	{
		//LEDs
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_RESET);	// Set the Output (LED) Pin (Red) low to see the end of the interruption
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_SET);		// Set the Output (LED) Pin (Blue) high to see the main

		//Heart Rate
		HEART_RATE_CALCULATION();
	}
}
