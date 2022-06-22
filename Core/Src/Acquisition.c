/*
 * Acquisition.c
 *
 *  Created on: Jun 22, 2022
 *      Author: Jessica Fayard
 */


//Includes ===========================================================================================================================================
#include "Acquisition.h"


//Value verification =================================================================================================================================
extern UART_HandleTypeDef hlpuart1;
unsigned long t1, t2;


//Variables ==========================================================================================================================================
data_2leds_TypeDef data_2leds;							//Data which contains 2 uint32 variables, one for each led
uint8_t rd_dat = 0;
uint8_t number_available_samples = 0;
uint16_t i = 0;
uint8_t j = 0;
uint8_t flag_filter = 0;
uint32_t numBlocks = LENGTH_DATA/BLOCK_SIZE;			//Number of blocks to have all the samples in data_1s_ir when filtering BLOCK_SIZE samples at a time


//Buffers ============================================================================================================================================
float32_t data_1s_ir[LENGTH_DATA] = {0};						//1 second IR data buffer
float32_t data_1s_red[LENGTH_DATA] = {0};						//1 second red data buffer



/*==================================================================================================================================================*/
/*=================================================================Acquisition=======================================================================*/
/*==================================================================================================================================================*/
//Functions ==========================================================================================================================================
acqui_return_value_t ACQUISITION_BY_BLOCKSIZE(void)
{
	//Red LED
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_SET);

	//Data ACQUISITION_BY_BLOCKSIZE
//	number_available_samples=HEARTRATE10_NUMBER_AVAILABLE_SAMPLES();
//	HAL_UART_Transmit(&hlpuart1, (uint8_t*)&number_available_samples, 1, 1000);
	READ(HEARTRATE10_REG_INT_STATUS, &rd_dat);
	for(i=0;i<BLOCK_SIZE;i++)
	{
		HEARTRATE10_READ_2LEDS_FIFO_DATA(&data_2leds);
		data_1s_ir[i+j*BLOCK_SIZE]=(float32_t)data_2leds.ir*(-1);	//Stock IR data on the IR buffer until the define number of samples has been reached
		data_1s_red[i+j*BLOCK_SIZE]=(float32_t)data_2leds.red*(-1);				//Stock red data on the IR buffer until the define number of samples has been reached


/*=======Values verification=======*/
//		HAL_UART_Transmit(&hlpuart1, (uint8_t*)&data_2leds, 8, 1000);
//		HAL_UART_Transmit(&hlpuart1, (uint8_t*)&data_1s_ir[i+j*BLOCK_SIZE], 4, 1000);
//		HAL_UART_Transmit(&hlpuart1, (uint8_t*)&data_1s_red[i+j*BLOCK_SIZE], 4, 1000);
	}

	//Counter
	j++;
	if (j == numBlocks)												//When LENGTH_DATA data have been acquired
	{
		flag_filter = 1;
	}

	//Red LED
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_RESET);


	return ACQUI_OK;
}

void HAL_GPIO_EXTI_Falling_Callback(uint16_t GPIO_Pin)
{
	if(GPIO_Pin == GPIO_PIN_3) 									// If The INT Source Is EXTI Line3
	{
		ACQUISITION_BY_BLOCKSIZE();

/*========Time verification========*/
//		//End time
//		t2 = DWT->CYCCNT;
//
//		//Calculate duration
//		diff = t2 - t1;
//		duration_ms = (float)((float)diff/(float)CPU_frequency)*1000;
//		HAL_UART_Transmit(&hlpuart1, (uint8_t*)&duration_ms, (uint16_t)4, HAL_MAX_DELAY);
//
//		//Beginning time
//		t1 = DWT->CYCCNT;
	}
}
