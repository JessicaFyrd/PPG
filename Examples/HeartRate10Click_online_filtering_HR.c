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
#include "DSP.h"
#include "Board.h"


int main(void)
{
  STM_INIT();
  SENSOR_2LED_INIT();
  DSP_INIT();

  while (1)
  {
	  HEART_RATE_CALCULATION();
  }
}
