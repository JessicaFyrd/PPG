/*
 * DSP.h
 *
 *  Created on: 10 Jun 2022
 *      Author: Jessica Fayard
 */


#ifndef SRC_DSP_H_
#define SRC_DSP_H_



//Includes ============================================================================================================================================
#include "main.h"
#include "arm_math.h"
#include "string.h"
#include "Board.h"


//Enumeration types  ==================================================================================================================================
typedef enum
{
   DSP_OK = 0,
   DSP_ERROR = -1

} dsp_return_value_t;




/*===================================================================================================================================================*/
/*==================================================================Filtration=======================================================================*/
/*===================================================================================================================================================*/


//Variables ===========================================================================================================================================
#define NUMBER_COEFS              	5
#define NUMBER_STAGE              	1
#define BLOCK_SIZE            		20					//Must be the same as the watermark level for interruption
#define LENGTH_DATA 				200					//Must be equal to the sample rate/number of samples for average to obtain 1 seconde of data
#define LENGTH_DATA_10s 			10*LENGTH_DATA


//Functions  ==========================================================================================================================================
dsp_return_value_t DSP_INIT(void);
dsp_return_value_t ACQUISITION_BY_BLOCKSIZE(void);
dsp_return_value_t IIR_FILTER(void);




/*===================================================================================================================================================*/
/*==============================================================Heart rate Calculation===============================================================*/
/*===================================================================================================================================================*/


//Variables ===========================================================================================================================================
#define SHIFT 20


//Functions  ==========================================================================================================================================
dsp_return_value_t ROLL_BUFFER(float32_t *input_buffer, float32_t *output_buffer);
dsp_return_value_t AUTO_CORRELATION(void);
float32_t HEART_RATE_CALCULATION(void);



#endif /* SRC_DSP_H_ */
