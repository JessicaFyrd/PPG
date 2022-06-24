//Includes ===========================================================================================================================================
#include "DSP.h"


//Value verification =================================================================================================================================
extern UART_HandleTypeDef hlpuart1;
extern unsigned long t1, t2;


//Extern variables ===================================================================================================================================
extern uint16_t i;
extern uint8_t j;
extern uint8_t flag_filter;
extern float32_t data_1s_ir[LENGTH_DATA];						//1 second IR data buffer
extern float32_t data_1s_red[LENGTH_DATA];						//1 second red data buffer


//Variables ===========================================================================================================================================
arm_biquad_cascade_df2T_instance_f32 S_ir, S_red;				//Type that contains the number of stages, a pointer to the buffer with coefficients and a pointer to the state
uint16_t max_x = 0;												//The max is the number of points, 2000 here so 16 bits is enough
uint64_t max_y = 0; 											//In Matlab, the max is about 4,5.10^9 so 32 bits is too short
float32_t Heart_Rate = 0;


//Buffers ============================================================================================================================================
static float32_t iirStateF32_ir[2*NUMBER_STAGE] = {0};			//State IR buffer
static float32_t iirStateF32_red[2*NUMBER_STAGE] = {0};			//State red buffer
const float32_t iirCoeffs32[NUMBER_COEFS] =						//Coefficients buffer
{
		0.1245,         0,   -0.1245,    1.7492,    -0.7509 	//b0 b1 b2 a1 a2 -->Matlab coefficient with erasing a0 which is 1 and inverted the a signs
};

float32_t data_1s_ir_filtered[LENGTH_DATA] = {0};				//Filter IR data 1s buffer
float32_t data_1s_red_filtered[LENGTH_DATA] = {0};				//Filter red data 1s buffer
float32_t data_10s_ir[LENGTH_DATA_10s] = {0};					//10 seconds IR data buffer
float32_t data_10s_red[LENGTH_DATA_10s] = {0};					//10 seconds red data buffer
float32_t data_10s_ir_filtered[LENGTH_DATA_10s] = {0};			//10 seconds IR filtered buffer
float32_t data_10s_red_filtered[LENGTH_DATA_10s] = {0};			//10 seconds red filtered buffer
float32_t auto_corr[2*LENGTH_DATA_10s-1] = {0};					//Autocorrelation buffer



/*==================================================================================================================================================*/
/*=================================================================Filtration=======================================================================*/
/*==================================================================================================================================================*/
//Initialization =====================================================================================================================================
dsp_return_value_t DSP_INIT(void)
{
	arm_biquad_cascade_df2T_init_f32(&S_ir,(uint8_t)NUMBER_STAGE,(const float32_t *)&iirCoeffs32[0], (float32_t *)&iirStateF32_ir[0]);
	arm_biquad_cascade_df2T_init_f32(&S_red,(uint8_t)NUMBER_STAGE,(const float32_t *)&iirCoeffs32[0], (float32_t *)&iirStateF32_red[0]);

	return DSP_OK;
}


//Functions ==========================================================================================================================================
dsp_return_value_t IIR_FILTER(void)
{
	arm_biquad_cascade_df2T_f32(&S_ir, data_1s_ir, data_1s_ir_filtered, LENGTH_DATA);					//Filter IR data by blocks of LENGTH_DATA
	arm_biquad_cascade_df2T_f32(&S_red, data_1s_red, data_1s_red_filtered, LENGTH_DATA);				//Filter red data by blocks of LENGTH_DATA


/*=======Values verification=======*/
//	HAL_UART_Transmit(&hlpuart1, (uint8_t*)data_1s_ir, (uint16_t)4*LENGTH_DATA, 1000);
//	HAL_UART_Transmit(&hlpuart1, (uint8_t*)data_1s_red, (uint16_t)4*LENGTH_DATA, 1000);
//	HAL_UART_Transmit(&hlpuart1, (uint8_t*)data_1s_ir_filtered, (uint16_t)4*LENGTH_DATA, 1000);
//	HAL_UART_Transmit(&hlpuart1, (uint8_t*)data_1s_red_filtered, (uint16_t)4*LENGTH_DATA, 1000);

	return DSP_OK;
}




/*==================================================================================================================================================*/
/*=============================================================Heart rate Calculation===============================================================*/
/*==================================================================================================================================================*/
//Functions ==========================================================================================================================================
dsp_return_value_t ROLL_BUFFER(float32_t *input_buffer, float32_t *output_buffer)
{
	//Shift a LENGTH_DATA size block to the left (erasing the 1st second of data)
	memcpy(&output_buffer[0],&output_buffer[LENGTH_DATA],(LENGTH_DATA_10s-LENGTH_DATA)*4); //Memory copy in the 1st argument from the second with the size in 3rd argument (in bytes)

	//Add a LENGTH_DATA size block on the right (add a new second of data)
	memcpy(&output_buffer[(LENGTH_DATA_10s-LENGTH_DATA)],input_buffer,LENGTH_DATA*4);


/*=======Values verification=======*/
	//UART Transmission
//	HAL_UART_Transmit(&hlpuart1, (uint8_t*)&output_buffer[(LENGTH_DATA_10s-LENGTH_DATA)], (uint16_t)4*LENGTH_DATA, HAL_MAX_DELAY);		//Last second
//	HAL_UART_Transmit(&hlpuart1, (uint8_t*)output_buffer, (uint16_t)4*LENGTH_DATA_10s, HAL_MAX_DELAY);									//Entire buffer
//	HAL_UART_Transmit(&hlpuart1, (uint8_t*)output_buffer, (uint16_t)4*LENGTH_DATA, HAL_MAX_DELAY);										//First second

	return DSP_OK;
}

dsp_return_value_t AUTO_CORRELATION(void)
{
	arm_correlate_f32((const float32_t *) data_10s_ir_filtered, (uint32_t) LENGTH_DATA_10s, (const float32_t *) data_10s_ir_filtered, (uint32_t) LENGTH_DATA_10s, (float32_t *) auto_corr);


/*=======Values verification=======*/
	//UART Transmission
//	HAL_UART_Transmit(&hlpuart1, (uint8_t*)auto_corr, (uint16_t)4*(2*LENGTH_DATA_10s-1), HAL_MAX_DELAY);
//	HAL_UART_Transmit(&hlpuart1, (uint8_t*)&auto_corr[500], (uint16_t)4*(LENGTH_DATA_10s-500), HAL_MAX_DELAY);
//	HAL_UART_Transmit(&hlpuart1, (uint8_t*)&auto_corr[LENGTH_DATA_10s], (uint16_t)4*(LENGTH_DATA_10s-501), HAL_MAX_DELAY);

	return DSP_OK;
}

float32_t HEART_RATE_CALCULATION(void)
{
	if (flag_filter == 1) //Calculation happen when the define number of samples has been reached (=> flag_filter = 1)
	{
		//Blue LED
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_SET);

		//Delete the older second and add the new one (raw data)
		ROLL_BUFFER(data_1s_ir, data_10s_ir);

		//Filter
		IIR_FILTER();

		//Delete the older second and add the new one (filtered data)
		ROLL_BUFFER(data_1s_ir_filtered, data_10s_ir_filtered);

		//Re-initialize variables
		j = 0;
		flag_filter = 0;
		max_y = 0;
		max_x = 0;

		//Heart Rate calculation
		AUTO_CORRELATION();

		//Maximum search for the second half with a little shift to erase the first pic
		for(i = (LENGTH_DATA_10s+SHIFT); i < (2*LENGTH_DATA_10s-1); i++)
		{
			if (auto_corr[i]>max_y)
			{
			  max_y = auto_corr[i];
			  max_x = i-LENGTH_DATA_10s+1;
			}
		}

		//Heart rate calculation
		Heart_Rate = (float32_t)((LENGTH_DATA * 60) / max_x);

		//Blue LED
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_RESET);


/*=======Values verification=======*/
		//UART Transmission
//		HAL_UART_Transmit(&hlpuart1, (uint8_t*)&data_10s_ir[(LENGTH_DATA_10s-LENGTH_DATA)], (uint16_t)4*LENGTH_DATA, HAL_MAX_DELAY);				//Last second
//		HAL_UART_Transmit(&hlpuart1, (uint8_t*)data_10s_ir, (uint16_t)4*LENGTH_DATA_10s, HAL_MAX_DELAY);											//Entire buffer
//		HAL_UART_Transmit(&hlpuart1, (uint8_t*)data_10s_ir, (uint16_t)4*LENGTH_DATA, HAL_MAX_DELAY);												//First second
//		HAL_UART_Transmit(&hlpuart1, (uint8_t*)&data_10s_ir_filtered[(LENGTH_DATA_10s-LENGTH_DATA)], (uint16_t)4*LENGTH_DATA, HAL_MAX_DELAY);		//Last second
//		HAL_UART_Transmit(&hlpuart1, (uint8_t*)data_10s_ir_filtered, (uint16_t)4*LENGTH_DATA_10s, HAL_MAX_DELAY);									//Entire buffer
//		HAL_UART_Transmit(&hlpuart1, (uint8_t*)data_10s_ir_filtered, (uint16_t)4*LENGTH_DATA, HAL_MAX_DELAY);										//First second
//		HAL_UART_Transmit(&hlpuart1, (uint8_t*)&max_x, (uint16_t)2, HAL_MAX_DELAY);
		HAL_UART_Transmit(&hlpuart1, (uint8_t*)&Heart_Rate, (uint16_t)4, HAL_MAX_DELAY);

/*========Time verification========*/
		//End time
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

	return Heart_Rate;
}

