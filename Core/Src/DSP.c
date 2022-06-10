//Includes ===========================================================================================================================================
#include <DSP.h>
#include "arm_math.h"
#include "string.h"


/*==================================================================================================================================================*/
/*=================================================================Filtration=======================================================================*/
/*==================================================================================================================================================*/


//Buffers ============================================================================================================================================
static float32_t iirStateF32_ir[2*NUMBER_STAGE] = {0};			//State IR buffer
static float32_t iirStateF32_red[2*NUMBER_STAGE] = {0};			//State red buffer
const float32_t iirCoeffs32[NUMBER_COEFS] =						//Coefficients buffer
{
		0.1245,         0,   -0.1245,    1.7492,    -0.7509 	//b0 b1 b2 a1 a2 -->Matlab coefficient with erasing a0 which is 1 and inverted the a signs
};

float32_t data_1s_ir[LENGTH_DATA] = {0};						//1 second IR buffer
float32_t data_1s_red[LENGTH_DATA] = {0};						//1 second red buffer
float32_t data_1s_ir_filtered[LENGTH_DATA] = {0};				//Filter IR data buffer
float32_t data_1s_red_filtered[LENGTH_DATA] = {0};				//Filter red data buffer

arm_biquad_cascade_df2T_instance_f32 S_ir, S_red;				//Type that contains the number of stages, a pointer to the buffer with coefficients and a pointer to the state


//Functions ==========================================================================================================================================
//Initialisation =====================================================================================================================================
dsp_return_value_t DSP_INIT(void)
{
	arm_biquad_cascade_df2T_init_f32(&S_ir,(uint8_t)NUMBER_STAGE,(const float32_t *)&iirCoeffs32[0], (float32_t *)&iirStateF32_ir[0]);
	arm_biquad_cascade_df2T_init_f32(&S_red,(uint8_t)NUMBER_STAGE,(const float32_t *)&iirCoeffs32[0], (float32_t *)&iirStateF32_red[0]);

	return DSP_OK;
}

dsp_return_value_t IIR_filter(void)
{
	arm_biquad_cascade_df2T_f32(&S_ir, data_1s_ir, data_1s_ir_filtered, LENGTH_DATA);					//Filter IR data by blocks of LENGTH_DATA
	arm_biquad_cascade_df2T_f32(&S_red, data_1s_red, data_1s_red_filtered, LENGTH_DATA);				//Filter red data by blocks of LENGTH_DATA

//	HAL_UART_Transmit(&hlpuart1, (uint8_t*)data_1s_ir, (uint16_t)4*LENGTH_DATA, 1000);
//	HAL_UART_Transmit(&hlpuart1, (uint8_t*)data_1s_red, (uint16_t)4*LENGTH_DATA, 1000);
//	HAL_UART_Transmit(&hlpuart1, (uint8_t*)data_1s_ir_filtered, (uint16_t)4*LENGTH_DATA, 1000);
//	HAL_UART_Transmit(&hlpuart1, (uint8_t*)data_1s_red_filtered, (uint16_t)4*LENGTH_DATA, 1000);

	return DSP_OK;
}





/*==================================================================================================================================================*/
/*=============================================================Heart rate Calculation===============================================================*/
/*==================================================================================================================================================*/


//Variables ==========================================================================================================================================
uint16_t max_x = 0;										//The max is the number of points, 2000 here so 16 bits is enough
uint64_t max_y = 0; 									//In Matlab, the max is about 4,5.10^9 so 32 bits is too short
float32_t Heart_Rate = 0;
extern uint16_t i;
extern uint8_t j;
extern uint8_t flag_filter;


//Buffers ============================================================================================================================================
float32_t data_10s_ir[LENGTH_DATA_10s] = {0};					//10 seconds IR buffer
float32_t data_10s_red[LENGTH_DATA_10s] = {0};					//10 seconds red buffer
float32_t auto_corr[2*LENGTH_DATA_10s-1] = {0};					//Autocorrelation buffer


//Functions ==========================================================================================================================================
dsp_return_value_t roll_buffer(void)
{
	//Shift a LENGTH_DATA size block to the left (erasing the 1st second of data)
	memcpy(&data_10s_ir[0],&data_10s_ir[LENGTH_DATA],9*LENGTH_DATA*4); //Memory copy in the 1st argument from the second with the size in 3rd argument (in bytes)

	//Add a LENGTH_DATA size block on the right (add a new second of data)
	memcpy(&data_10s_ir[9*LENGTH_DATA],data_1s_ir_filtered,LENGTH_DATA*4);

	//UART Transmission
//	HAL_UART_Transmit(&hlpuart1, (uint8_t*)&data_10s_ir[9*LENGTH_DATA], (uint16_t)4*LENGTH_DATA, HAL_MAX_DELAY);
//	HAL_UART_Transmit(&hlpuart1, (uint8_t*)data_10s_ir, (uint16_t)4*LENGTH_DATA_10s, HAL_MAX_DELAY);

	return DSP_OK;
}

dsp_return_value_t auto_correlation(void)
{
	arm_correlate_f32((const float32_t *) data_10s_ir, (uint32_t) LENGTH_DATA_10s, (const float32_t *) data_10s_ir, (uint32_t) LENGTH_DATA_10s, (float32_t *) auto_corr);
//	HAL_UART_Transmit(&hlpuart1, (uint8_t*)&auto_corr, (uint16_t)4*(2*LENGTH_DATA_10s-1), HAL_MAX_DELAY);

	return DSP_OK;
}

float32_t heart_rate_calculation(void)
{
	//Filter
	IIR_filter();

	//Delete the older second and add the new one (filtered data)
	roll_buffer();

	//Re-initialize variables
	j = 0;
	flag_filter = 0;
	max_y = 0;
	max_x = 0;

	//Heart Rate calculation
	auto_correlation();

	//Maximum search for the second half with a little shift to erase the first pic
	for(i = (LENGTH_DATA_10s+SHIFT); i < (LENGTH_DATA_10s+SHIFT+300); i++)
	{
		if (auto_corr[i]>max_y)
		{
		  max_y = auto_corr[i];
		  max_x = i+SHIFT-LENGTH_DATA_10s;
		}
	}

	//Heart rate calculation
	Heart_Rate = (float32_t)((LENGTH_DATA * 60) / max_x);

	//UART Transmission
//	HAL_UART_Transmit(&hlpuart1, (uint8_t*)&max_x, (uint16_t)2, HAL_MAX_DELAY);
//	HAL_UART_Transmit(&hlpuart1, (uint8_t*)&Heart_Rate, (uint16_t)4, HAL_MAX_DELAY);

	return Heart_Rate;
}
