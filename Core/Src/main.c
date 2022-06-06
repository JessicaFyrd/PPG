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
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "MAX86916_ppg.h"
#include "arm_math.h"
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
//Filter PD
#define NUMBER_COEFS              	5
#define NUMBER_STAGE              	1
#define BLOCK_SIZE            		20				//Must be the same as the watermark level for interruption
#define LENGTH_DATA 				200				//Must be equal to the sample rate/number of samples for average to obtain 1 seconde of data
#define LENGTH_DATA_10s 			10*LENGTH_DATA

//Heart Rate calculation
#define SHIFT 40
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
 I2C_HandleTypeDef hi2c2;

UART_HandleTypeDef hlpuart1;

RTC_HandleTypeDef hrtc;

PCD_HandleTypeDef hpcd_USB_FS;

/* USER CODE BEGIN PV */
//Data acquisition
data_2leds_TypeDef data;														//Data which contains 2 uint32 variables, one for each led
uint8_t rd_dat = 0, number_available_samples = 0, i = 0;
static uint8_t j = 0;

//Filter PV
uint8_t flag_filter = 0;
float32_t block_data_ir[LENGTH_DATA]={0};										//Online data buffer for IR LED
float32_t block_data_red[LENGTH_DATA]={0};										//Online data buffer for red LED
float32_t data_10s_ir[LENGTH_DATA_10s]={0};										//10 seconds IR buffer
float32_t data_10s_red[LENGTH_DATA_10s]={0};									//10 seconds red buffer
data_1s data_10s[LENGTH_DATA_10s]={0};											//10 seconds buffer with IR and red data (printing buffer)
uint32_t block_size = BLOCK_SIZE;
uint32_t numBlocks = LENGTH_DATA/BLOCK_SIZE;									//Number of blocks to have all the samples in block_data_ir when filtering BLOCK_SIZE samples at a time
arm_biquad_cascade_df2T_instance_f32 S_ir, S_red;								//Type that contains the number of stages, a pointer to the buffer with coefficients and a pointer to the state
float32_t  *inputF32_ir,*inputF32_red, *outputF32_ir, *outputF32_red; 			//Pointers to input and output buffers

//Heart Rate calculation PV
uint8_t max_y = 0, max_x = 0;
uint32_t Heart_Rate = 0;
float32_t auto_corr[2*LENGTH_DATA_10s-1]={0};
float32_t auto_corr_extract[LENGTH_DATA_10s-1-SHIFT]={0};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C2_Init(void);
static void MX_LPUART1_UART_Init(void);
static void MX_USB_PCD_Init(void);
static void MX_ICACHE_Init(void);
static void MX_RTC_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
//Filter buffer
static float32_t output_ir[LENGTH_DATA]={0};					//Filter IR data buffer
static float32_t output_red[LENGTH_DATA]={0};					//Filter red data buffer

static float32_t iirStateF32_ir[2*NUMBER_STAGE]={0};			//State IR buffer
static float32_t iirStateF32_red[2*NUMBER_STAGE]={0};			//State red buffer
const float32_t iirCoeffs32[NUMBER_COEFS] =						//Coefficients buffer
{
		0.1245,         0,   -0.1245,    1.7492,    -0.7509 	//b0 b1 b2 a1 a2 -->Matlab coefficient with erasing a0 which is 1 and inverted the a signs
};
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C2_Init();
  MX_LPUART1_UART_Init();
  MX_USB_PCD_Init();
  MX_ICACHE_Init();
  MX_RTC_Init();
  /* USER CODE BEGIN 2 */
  //Filter variables
  inputF32_ir = &block_data_ir[0]; 								//Initialize input IR buffer pointers
  outputF32_ir = &output_ir[0]; 								//Initialize output IR buffer pointers
  inputF32_red = &block_data_red[0]; 							//Initialize input red buffer pointers
  outputF32_red = &output_red[0]; 								//Initialize output red buffer pointers
  arm_biquad_cascade_df2T_init_f32(&S_ir,(uint8_t)NUMBER_STAGE,(const float32_t *)&iirCoeffs32[0], (float32_t *)&iirStateF32_ir[0]); 	//Initialize IR filter
  arm_biquad_cascade_df2T_init_f32(&S_red,(uint8_t)NUMBER_STAGE,(const float32_t *)&iirCoeffs32[0], (float32_t *)&iirStateF32_red[0]);	//Initialize red filter

  //Config
  heartrate10_return_value_t err_t;
  err_t = heartrate10_default_2leds_cfg(hi2c2);					//2 LEDS init
  if (err_t!=0)
  {
	  HAL_UART_Transmit(&hlpuart1, (uint8_t*)&err_t, 1, 1000);
  }
  READ(HEARTRATE10_REG_INT_STATUS, &rd_dat);					//Clean interrupts
  heartrate10_SMP_RDY_EN(MAX86916_SMP_RDY_DIS);					//Disable interruption when a new sample is in FIFO
  heartrate10_A_FULL_EN(MAX86916_A_FULL_EN);					//Enable interruption when a watermark level of samples is in FIFO
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  //LEDs
	  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_RESET);
	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_SET);		// Set the Output (LED) Pin (Blue) high to see the main

	  //Filtration that happen when the define number of samples has been reached (=> flag_filter = 1)
	  if (flag_filter == 1)
	  {
		  //Shift a LENGTH_DATA size block to the left (erasing the 1st second of data)
//		  for (i=0;i<9*LENGTH_DATA;i++)
//		  {
//			  data_10s_ir[i]=data_10s_ir[i+LENGTH_DATA];
//			  data_10s_red[i]=data_10s_red[i+LENGTH_DATA];
//		  }
		  memcpy(&data_10s_ir[0],&data_10s_ir[LENGTH_DATA],9*LENGTH_DATA*4); //Memory copy in the 1st argument from the second with the size in 3rd argument (in bytes)

		  //Filter
		  IIR_filter();

		  //Add a LENGTH_DATA size block on the right (add a new second of data)
//		  for (i=0;i<LENGTH_DATA;i++)
//		  {
//			  data_10s_ir[i+9*LENGTH_DATA]=output_ir[i];
//			  data_10s_red[i+9*LENGTH_DATA]=output_red[i];
//		  }
		  memcpy(&data_10s_ir[9*LENGTH_DATA],output_ir,LENGTH_DATA*4);

		  //Add the non filtered and non filtered data to transmit it via UART
//		  for (i=0;i<LENGTH_DATA_10s;i++)
//		  {
//			  data_10s[i].ir=data_10s_ir[i];
//			  data_10s[i].red=data_10s_red[i];
//		  }
//		  HAL_UART_Transmit(&hlpuart1, (uint8_t*)data_10s, (uint16_t)4*LENGTH_DATA_10s, 1000);

		  HAL_Delay(1);
		  HAL_UART_Transmit(&hlpuart1, (uint8_t*)&data_10s_ir[9*LENGTH_DATA], (uint16_t)4*LENGTH_DATA, HAL_MAX_DELAY);
//		  HAL_UART_Transmit(&hlpuart1, (uint8_t*)data_10s_ir, (uint16_t)4*LENGTH_DATA_10s, HAL_MAX_DELAY);
//		  for(i = 0; i < 10; i++)
//		  {
//			  HAL_UART_Transmit(&hlpuart1, (uint8_t*)&data_10s_ir[i], (uint16_t)4*LENGTH_DATA, HAL_MAX_DELAY);
//		  }

		  //Re-initialize variables
		  j = 0;
		  flag_filter = 0;
		  max_y = 0;
		  max_x = 0;

		  //Heart Rate calculation
//		  arm_correlate_f32	((const float32_t *) data_10s_ir, (uint32_t) LENGTH_DATA_10s, (const float32_t *) data_10s_ir, (uint32_t) LENGTH_DATA_10s, (float32_t *) auto_corr);
//		  memcpy(auto_corr_extract,&auto_corr[LENGTH_DATA_10s+SHIFT],LENGTH_DATA_10s-1-SHIFT);
//		  for(i = 0; i < LENGTH_DATA_10s-1-SHIFT; i++)
//		  {
//			  if (auto_corr_extract[i]>max_y)
//			  {
//				  max_y = auto_corr_extract[i];
//				  max_x = i+SHIFT;
//			  }
//		  }
//		  Heart_Rate = LENGTH_DATA * 60 / max_x;												//Sampling frequency*60/index of max correlation
//		  HAL_UART_Transmit(&hlpuart1, (uint8_t*)&Heart_Rate, (uint16_t)4, HAL_MAX_DELAY);
	  }

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_LSE
                              |RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.LSIDiv = RCC_LSI_DIV1;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = RCC_MSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }

  /** Enable MSI Auto calibration
  */
  HAL_RCCEx_EnableMSIPLLMode();
}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.Timing = 0x00303D5B;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief ICACHE Initialization Function
  * @param None
  * @retval None
  */
static void MX_ICACHE_Init(void)
{

  /* USER CODE BEGIN ICACHE_Init 0 */

  /* USER CODE END ICACHE_Init 0 */

  /* USER CODE BEGIN ICACHE_Init 1 */

  /* USER CODE END ICACHE_Init 1 */

  /** Enable instruction cache in 1-way (direct mapped cache)
  */
  if (HAL_ICACHE_ConfigAssociativityMode(ICACHE_1WAY) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_ICACHE_Enable() != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ICACHE_Init 2 */

  /* USER CODE END ICACHE_Init 2 */

}

/**
  * @brief LPUART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_LPUART1_UART_Init(void)
{

  /* USER CODE BEGIN LPUART1_Init 0 */

  /* USER CODE END LPUART1_Init 0 */

  /* USER CODE BEGIN LPUART1_Init 1 */

  /* USER CODE END LPUART1_Init 1 */
  hlpuart1.Instance = LPUART1;
  hlpuart1.Init.BaudRate = 209700;
  hlpuart1.Init.WordLength = UART_WORDLENGTH_8B;
  hlpuart1.Init.StopBits = UART_STOPBITS_1;
  hlpuart1.Init.Parity = UART_PARITY_NONE;
  hlpuart1.Init.Mode = UART_MODE_TX_RX;
  hlpuart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  hlpuart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  hlpuart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  hlpuart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  hlpuart1.FifoMode = UART_FIFOMODE_DISABLE;
  if (HAL_UART_Init(&hlpuart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&hlpuart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&hlpuart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&hlpuart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN LPUART1_Init 2 */

  /* USER CODE END LPUART1_Init 2 */

}

/**
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  RTC_PrivilegeStateTypeDef privilegeState = {0};

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */

  /** Initialize RTC Only
  */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutRemap = RTC_OUTPUT_REMAP_NONE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  hrtc.Init.OutPutPullUp = RTC_OUTPUT_PULLUP_NONE;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }
  privilegeState.rtcPrivilegeFull = RTC_PRIVILEGE_FULL_NO;
  privilegeState.backupRegisterPrivZone = RTC_PRIVILEGE_BKUP_ZONE_NONE;
  privilegeState.backupRegisterStartZone2 = RTC_BKP_DR0;
  privilegeState.backupRegisterStartZone3 = RTC_BKP_DR0;
  if (HAL_RTCEx_PrivilegeModeSet(&hrtc, &privilegeState) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

}

/**
  * @brief USB Initialization Function
  * @param None
  * @retval None
  */
static void MX_USB_PCD_Init(void)
{

  /* USER CODE BEGIN USB_Init 0 */

  /* USER CODE END USB_Init 0 */

  /* USER CODE BEGIN USB_Init 1 */

  /* USER CODE END USB_Init 1 */
  hpcd_USB_FS.Instance = USB;
  hpcd_USB_FS.Init.dev_endpoints = 8;
  hpcd_USB_FS.Init.speed = PCD_SPEED_FULL;
  hpcd_USB_FS.Init.phy_itface = PCD_PHY_EMBEDDED;
  hpcd_USB_FS.Init.Sof_enable = DISABLE;
  hpcd_USB_FS.Init.low_power_enable = DISABLE;
  hpcd_USB_FS.Init.lpm_enable = DISABLE;
  hpcd_USB_FS.Init.battery_charging_enable = DISABLE;
  if (HAL_PCD_Init(&hpcd_USB_FS) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USB_Init 2 */

  /* USER CODE END USB_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  HAL_PWREx_EnableVddIO2();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pin : INTB_Pin */
  GPIO_InitStruct.Pin = INTB_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(INTB_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PA9 */
  GPIO_InitStruct.Pin = GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PB7 */
  GPIO_InitStruct.Pin = GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);

}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Falling_Callback(uint16_t GPIO_Pin)
{
	if(GPIO_Pin == GPIO_PIN_3) 													// If The INT Source Is EXTI Line3
	{
		//LEDs
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_SET); 					// Set the Output (LED) Pin (Red) high to see the interrupt
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET);

		//Data acquisition
//		number_available_samples=heartrate10_number_available_samples();
		READ(HEARTRATE10_REG_INT_STATUS, &rd_dat);
//		HAL_UART_Transmit(&hlpuart1, (uint8_t*)&number_available_samples, 1, 1000);
		for(i=0;i<block_size;i++)
		{
			heartrate10_read_2leds_fifo_data(&data);
//			HAL_UART_Transmit(&hlpuart1, (uint8_t*)&data, 8, 1000);
			block_data_ir[i+j*block_size]=(float32_t)data.ir;									//Stock IR data on the IR buffer until the define number of samples has been reached
//			HAL_UART_Transmit(&hlpuart1, (uint8_t*)&block_data_ir[i+j*block_size], 4, 1000);
			block_data_red[i+j*block_size]=(float32_t)data.red;									//Stock red data on the IR buffer until the define number of samples has been reached
//			HAL_UART_Transmit(&hlpuart1, (uint8_t*)&block_data_red[i+j*block_size], 4, 1000);
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

void IIR_filter(void)
{
	arm_biquad_cascade_df2T_f32(&S_ir, inputF32_ir, outputF32_ir, LENGTH_DATA);					//Filter IR data by blocks of LENGTH_DATA
	arm_biquad_cascade_df2T_f32(&S_red, inputF32_red, outputF32_red, LENGTH_DATA);				//Filter red data by blocks of LENGTH_DATA
//	HAL_UART_Transmit(&hlpuart1, (uint8_t*)block_data_ir, (uint16_t)4*LENGTH_DATA, 1000);
//	HAL_UART_Transmit(&hlpuart1, (uint8_t*)block_data_red, (uint16_t)4*LENGTH_DATA, 1000);
//	HAL_UART_Transmit(&hlpuart1, (uint8_t*)output_ir, (uint16_t)4*LENGTH_DATA, 1000);
//	HAL_UART_Transmit(&hlpuart1, (uint8_t*)output_red, (uint16_t)4*LENGTH_DATA, 1000);
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
