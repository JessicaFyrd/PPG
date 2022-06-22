/*
 * MAX86916_ppg.h
 *
 *  Created on: Apr 28, 2022
 *      Author: Jessica Fayard
 */

#ifndef INC_MAX86916_PPG_H_
#define INC_MAX86916_PPG_H_


//Includes ===========================================================================================================================================
#include "main.h"
#include "arm_math.h"


//Registers  =========================================================================================================================================
#define HEARTRATE10_REG_INT_STATUS              0x00
#define HEARTRATE10_REG_INT_ENABLE              0x02
#define HEARTRATE10_REG_FIFO_WR_PTR             0x04
#define HEARTRATE10_REG_FIFO_OVF_CNT            0x05
#define HEARTRATE10_REG_FIFO_RD_PTR             0x06
#define HEARTRATE10_REG_FIFO_DATA               0x07
#define HEARTRATE10_REG_FIFO_CFG                0x08
#define HEARTRATE10_REG_MODE_CFG1               0x09
#define HEARTRATE10_REG_MODE_CFG2               0x0A
#define HEARTRATE10_REG_LED1_PA                 0x0C
#define HEARTRATE10_REG_LED2_PA                 0x0D
#define HEARTRATE10_REG_LED3_PA                 0x0E
#define HEARTRATE10_REG_LED4_PA                 0x0F
#define HEARTRATE10_REG_LED_RANGE               0x11
#define HEARTRATE10_REG_PILOT_PA                0x12
#define HEARTRATE10_REG_LED_SEQ1                0x13
#define HEARTRATE10_REG_LED_SEQ2                0x14
#define HEARTRATE10_REG_DAC1_CROSSTALK_CODE     0x26
#define HEARTRATE10_REG_DAC2_CROSSTALK_CODE     0x27
#define HEARTRATE10_REG_DAC3_CROSSTALK_CODE     0x28
#define HEARTRATE10_REG_DAC4_CROSSTALK_CODE     0x29
#define HEARTRATE10_REG_PROX_INT_THRESHOLD      0x30
#define HEARTRATE10_REG_LED_COMPARATOR_EN       0x31
#define HEARTRATE10_REG_LED_COMPARATOR_STATUS   0x32
#define HEARTRATE10_REG_REV_ID                  0xFE
#define HEARTRATE10_REG_PART_ID                 0xFF


//Sensor ID  =========================================================================================================================================
#define HEARTRATE10_PART_ID                     0x2B


//Sensor addresses  ==================================================================================================================================
#define HEARTRATE10_SET_DEV_ADDR                0x57
#define HEARTRATE10_SET_DEV_ADDR_W              0xAE
#define HEARTRATE10_SET_DEV_ADDR_R              0xAF


//Enumeration types  =================================================================================================================================
	//Heart rate return
typedef enum
{
   HEARTRATE10_OK = 0,
   HEARTRATE10_ERROR = -1

} heartrate10_return_value_t;

	//Interrupt enable 1 (0x02)
typedef enum
{
	MAX86916_A_FULL_DIS = 0b0,
	MAX86916_A_FULL_EN = 0b1,
} MAX86916_A_FULL_EN_TypeDef;

typedef enum
{
	MAX86916_SMP_RDY_DIS = 0b0,
	MAX86916_SMP_RDY_EN = 0b1,
} MAX86916_SMP_RDY_EN_TypeDef;

typedef enum
{
	MAX86916_ALC_OVF_DIS = 0b0,
	MAX86916_ALC_OVF_EN = 0b1,
} MAX86916_ALC_OVF_EN_TypeDef;

typedef enum
{
	MAX86916_PROX_INT_DIS = 0b0,
	MAX86916_PROX_INT_EN = 0b1,
} MAX86916_PROX_INT_EN_TypeDef;

	//FIFO Config (0x08)
typedef enum
{
	MAX86916_FIFO_SAMPLE_AVERAGE_1 = 0b000,
	MAX86916_FIFO_SAMPLE_AVERAGE_2 = 0b001,
	MAX86916_FIFO_SAMPLE_AVERAGE_4 = 0b010,
	MAX86916_FIFO_SAMPLE_AVERAGE_8 = 0b011,
	MAX86916_FIFO_SAMPLE_AVERAGE_16 = 0b100,
	MAX86916_FIFO_SAMPLE_AVERAGE_32 = 0b101,
} MAX86916_FIFO_SAMPLE_AVERAGE_TypeDef;

typedef enum
{
	MAX86916_FIFO_RO_DIS = 0b0,
	MAX86916_FIFO_RO_EN = 0b1,		//FIFO automatically rolls over on full
} MAX86916_FIFO_RO_TypeDef;

typedef enum
{
	MAX86916_FIFO_A_FULL_32 = 0b0000, //32 samples in the FIFO when the interrupt is asserted -> 0 free space at interrupt
	MAX86916_FIFO_A_FULL_31 = 0b0001,
	MAX86916_FIFO_A_FULL_30 = 0b0010,
	MAX86916_FIFO_A_FULL_29 = 0b0011,
	MAX86916_FIFO_A_FULL_28 = 0b0100,
	MAX86916_FIFO_A_FULL_27 = 0b0101,
	MAX86916_FIFO_A_FULL_26 = 0b0110,
	MAX86916_FIFO_A_FULL_25 = 0b0111,
	MAX86916_FIFO_A_FULL_24 = 0b1000,
	MAX86916_FIFO_A_FULL_23 = 0b1001,
	MAX86916_FIFO_A_FULL_22 = 0b1010,
	MAX86916_FIFO_A_FULL_21 = 0b1011,
	MAX86916_FIFO_A_FULL_20 = 0b1100,
	MAX86916_FIFO_A_FULL_19 = 0b1101,
	MAX86916_FIFO_A_FULL_18 = 0b1110,
	MAX86916_FIFO_A_FULL_17 = 0b1111,
} MAX86916_FIFO_A_FULL_TypeDef;

	//Mode configuration 1 (0x09)
typedef enum
{
	MAX86916_MODE_DISABLE = 0b00,
	MAX86916_MODE_LED1 = 0b01,
	MAX86916_MODE_LED1_LED2 = 0b10,
	MAX86916_MODE_FLEX = 0b11,
} MAX86916_MODE_TypeDef;

	//Mode configuration 2 (0x0A)
typedef enum
{
	MAX86916_ADC_RANGE_4096 = 0b00,
	MAX86916_ADC_RANGE_8192 = 0b01,
	MAX86916_ADC_RANGE_16384 = 0b10,
	MAX86916_ADC_RANGE_32768 = 0b11,
} MAX86916_ADC_range_TypeDef;

typedef enum
{
	MAX86916_SR_50 = 0b000,
	MAX86916_SR_100 = 0b001,
	MAX86916_SR_200 = 0b010,
	MAX86916_SR_400 = 0b011,
	MAX86916_SR_800 = 0b100,
	MAX86916_SR_1000 = 0b101,
	MAX86916_SR_1600 = 0b110,
	MAX86916_SR_3200 = 0b111,
} MAX86916_SR_TypeDef;

typedef enum
{
	MAX86916_LED_PW_70 = 0b00,
	MAX86916_LED_PW_120 = 0b01,
	MAX86916_LED_PW_220 = 0b10,
	MAX86916_LED_PW_420 = 0b11,
} MAX86916_LED_PW_TypeDef;

	//LED Range (0x11)
typedef enum
{
	MAX86916_LED_RANGE_50 = 0b00,
	MAX86916_LED_RANGE_100 = 0b01,
	MAX86916_LED_RANGE_150 = 0b10,
	MAX86916_LED_RANGE_200 = 0b11,
} MAX86916_LED_RANGE_TypeDef;

	//LED Sequence (0x13 and 0x14)
typedef enum
{
	MAX86916_LED_SEQ_OFF = 0b0000,
	MAX86916_LED_SEQ_LED1 = 0b0001,
	MAX86916_LED_SEQ_LED2 = 0b0010,
	MAX86916_LED_SEQ_LED3 = 0b0011,
	MAX86916_LED_SEQ_LED4 = 0b0100,
	MAX86916_LED_SEQ_PILOT_LED1 = 0b0101,
	MAX86916_LED_SEQ_PILOT_LED2 = 0b0110,
	MAX86916_LED_SEQ_PILOT_LED3 = 0b0111,
	MAX86916_LED_SEQ_PILOT_LED4 = 0b1000,
} MAX86916_LED_SEQ_PILOT_TypeDef;


//Structures  ========================================================================================================================================
typedef struct
{
	uint32_t ir;
	uint32_t red;
	uint32_t green;
	uint32_t blue;
} data_4leds_TypeDef;

typedef struct
{
	uint32_t filter_ir;
	uint32_t filter_red;
	uint32_t filter_green;
	uint32_t filter_blue;
} filter_data_4leds_TypeDef;

typedef struct
{
	uint32_t ir;
	uint32_t red;
} data_2leds_TypeDef;

typedef struct
{
	uint32_t filter_ir;
	uint32_t filter_red;
} filter_data_2leds_TypeDef;


//Functions  =========================================================================================================================================
heartrate10_return_value_t HEARTRATE10_DEFAULT_4LEDS_CFG(I2C_HandleTypeDef i2c);
heartrate10_return_value_t HEARTRATE10_DEFAULT_2LEDS_CFG(I2C_HandleTypeDef i2c);
heartrate10_return_value_t HEARTRATE10_SHUTDOWN_DEVICE(void);
heartrate10_return_value_t HEARTRATE10_RESET_DEVICE(void);
heartrate10_return_value_t HEARTRATE10_SET_MODE(MAX86916_MODE_TypeDef MODE);
heartrate10_return_value_t HEARTRATE10_SET_LED_SEQUENCE_1(MAX86916_LED_SEQ_PILOT_TypeDef SEQ);
heartrate10_return_value_t HEARTRATE10_SET_LED_SEQUENCE_2(MAX86916_LED_SEQ_PILOT_TypeDef SEQ);
heartrate10_return_value_t HEARTRATE10_SET_LED_SEQUENCE_3(MAX86916_LED_SEQ_PILOT_TypeDef SEQ);
heartrate10_return_value_t HEARTRATE10_SET_LED_SEQUENCE_4(MAX86916_LED_SEQ_PILOT_TypeDef SEQ);
heartrate10_return_value_t HEARTRATE10_ADC_RANGE(MAX86916_ADC_range_TypeDef ADC_RANGE);
heartrate10_return_value_t HEARTRATE10_SR(MAX86916_SR_TypeDef SR);
heartrate10_return_value_t HEARTRATE10_LED_PULSE_WIDTH(MAX86916_LED_PW_TypeDef LED_PW);
heartrate10_return_value_t HEARTRATE10_LED_RANGE_1(MAX86916_LED_RANGE_TypeDef LED_RANGE);
heartrate10_return_value_t HEARTRATE10_LED_RANGE_2(MAX86916_LED_RANGE_TypeDef LED_RANGE);
heartrate10_return_value_t HEARTRATE10_LED_RANGE_3(MAX86916_LED_RANGE_TypeDef LED_RANGE);
heartrate10_return_value_t HEARTRATE10_LED_RANGE_4(MAX86916_LED_RANGE_TypeDef LED_RANGE);
heartrate10_return_value_t HEARTRATE10_LED_POWER_1(uint8_t LED_PA_1);
heartrate10_return_value_t HEARTRATE10_LED_POWER_2(uint8_t LED_PA_2);
heartrate10_return_value_t HEARTRATE10_LED_POWER_3(uint8_t LED_PA_3);
heartrate10_return_value_t HEARTRATE10_LED_POWER_4(uint8_t LED_PA_4);
heartrate10_return_value_t heartrate10_FIFO_SAMPLE_AVERAGE(MAX86916_FIFO_SAMPLE_AVERAGE_TypeDef SMP_AVE);
heartrate10_return_value_t heartrate10_FIFO_RO(MAX86916_FIFO_RO_TypeDef FIFO_RO_EN);
heartrate10_return_value_t heartrate10_FIFO_A_FULL(MAX86916_FIFO_A_FULL_TypeDef FIFO_A_FULL);
heartrate10_return_value_t heartrate10_A_FULL_EN(MAX86916_A_FULL_EN_TypeDef A_FULL_EN);
heartrate10_return_value_t heartrate10_SMP_RDY_EN(MAX86916_SMP_RDY_EN_TypeDef SMP_RDY_EN);
heartrate10_return_value_t heartrate10_ALC_OVF_EN(MAX86916_ALC_OVF_EN_TypeDef ALC_OVF_EN);
heartrate10_return_value_t heartrate10_PROX_INT_EN(MAX86916_PROX_INT_EN_TypeDef PROX_INT_EN);
HAL_StatusTypeDef HEARTRATE10_FIFO_READ(uint8_t *pData, uint8_t RX_LEN);
uint32_t HEARTRATE10_READ_FIFO_SAMPLE(void);
uint8_t HEARTRATE10_NUMBER_AVAILABLE_SAMPLES(void);
HAL_StatusTypeDef HEARTRATE10_READ_COMPLETE_FIFO_DATA(data_4leds_TypeDef *pData);
HAL_StatusTypeDef HEARTRATE10_READ_2LEDS_FIFO_DATA(data_2leds_TypeDef *pData);
HAL_StatusTypeDef READ(uint16_t Address, uint8_t *pData);
HAL_StatusTypeDef SEND(uint16_t Address, uint8_t *pData);

#endif /* INC_MAX86916_PPG_H_ */
