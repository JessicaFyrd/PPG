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


//Data acquisition variables
data_2leds_TypeDef data;								//Data which contains 2 uint32 variables, one for each led
uint8_t rd_dat = 0, number_available_samples = 0;
extern uint16_t i;
extern uint8_t j;

//Filter variables
uint32_t block_size = BLOCK_SIZE;
uint32_t numBlocks = LENGTH_DATA/BLOCK_SIZE;			//Number of blocks to have all the samples in data_1s_ir when filtering BLOCK_SIZE samples at a time
extern uint8_t flag_filter;
extern float32_t data_1s_ir[LENGTH_DATA];				//1 second IR buffer
extern float32_t data_1s_red[LENGTH_DATA];				//1 second red buffer
extern float32_t data_1s_ir_filtered[LENGTH_DATA];		//Filter IR data buffer
extern float32_t data_1s_red_filtered[LENGTH_DATA];		//Filter red data buffer
extern float32_t data_10s_ir[LENGTH_DATA_10s];			//10 seconds IR buffer


int main(void)
{
  STM_INIT();
  DSP_INIT();

  while (1)
  {
	  //LEDs
	  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_RESET);
	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_SET);		// Set the Output (LED) Pin (Blue) high to see the main

	  //Heart rate
	  HEART_RATE_CALCULATION();
  }
}


void HAL_GPIO_EXTI_Falling_Callback(uint16_t GPIO_Pin)
{
	if(GPIO_Pin == GPIO_PIN_3) 									// If The INT Source Is EXTI Line3
	{
		//LEDs
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_SET); 	// Set the Output (LED) Pin (Red) high to see the interrupt
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET);

		//Data acquisition
//		number_available_samples=HEARTRATE10_NUMBER_AVAILABLE_SAMPLES();
//		HAL_UART_Transmit(&hlpuart1, (uint8_t*)&number_available_samples, 1, 1000);
		READ(HEARTRATE10_REG_INT_STATUS, &rd_dat);
		for(i=0;i<block_size;i++)
		{
			HEARTRATE10_READ_2LEDS_FIFO_DATA(&data);
//			HAL_UART_Transmit(&hlpuart1, (uint8_t*)&data, 8, 1000);
			data_1s_ir[i+j*block_size]=(float32_t)data.ir*(-1);									//Stock IR data on the IR buffer until the define number of samples has been reached
//			HAL_UART_Transmit(&hlpuart1, (uint8_t*)&data_1s_ir[i+j*block_size], 4, 1000);
			data_1s_red[i+j*block_size]=(float32_t)data.red*(-1);								//Stock red data on the IR buffer until the define number of samples has been reached
//			HAL_UART_Transmit(&hlpuart1, (uint8_t*)&data_1s_red[i+j*block_size], 4, 1000);
		}
//		HAL_UART_Transmit(&hlpuart1, (uint8_t*)&j, 1, 1000);

		//Counter
		j++;
		if (j == numBlocks)			//When LENGTH_DATA data have been acquired
		{
			flag_filter = 1;
		}
	}
}
