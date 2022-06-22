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

//Includes
#include "main.h"
#include "MAX86916_ppg.h"
#include "Board.h"
#include "Acquisition.h"
#include "DSP.h"


int main(void)
{
  STM_INIT();
  SENSOR_2LED_INIT();
  DSP_INIT();


//  //Enable the cycle counter to check duration
//  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
//  DWT->CYCCNT = 0;
//  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

  while (1)
  {
	  HEART_RATE_CALCULATION();
  }
}
