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

//UART
extern UART_HandleTypeDef hlpuart1;

//1s Flag
extern uint8_t flag_filter;

//Heart Rate calculation PV
float32_t HR = 0;

int main(void)
{
	STM_INIT();
	SENSOR_2LED_INIT();
	DSP_INIT();

	//First, enable the cycle counter once at startup:
	CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
	DWT->CYCCNT = 0;
	DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

	while (1)
	{
		//LEDs
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_RESET);	// Set the Output (LED) Pin (Red) low to see the end of the interruption
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_SET);		// Set the Output (LED) Pin (Blue) high to see the main

		//Filtration that happen when the define number of samples has been reached (=> flag_filter = 1)
		if (flag_filter == 1)
		{
			HR = HEART_RATE_CALCULATION();
//			HAL_UART_Transmit(&hlpuart1, (uint8_t*)&HR, (uint16_t)4, HAL_MAX_DELAY);
			unsigned long t1;
			unsigned long t2 = DWT->CYCCNT;
			unsigned long diff = t2 - t1;
			float time = (float)diff*1000/(float)110000000;
			HAL_UART_Transmit(&hlpuart1, (uint8_t*)&time, (uint16_t)4, HAL_MAX_DELAY);
			t1 = DWT->CYCCNT;
		}
	}
}
