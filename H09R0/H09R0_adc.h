/*
 * H09R0_adc.h
 *
 *  Created on: ١٧‏/٠٣‏/٢٠٢١
 *      Author: shift
 */

#ifndef H09R0_ADC_H_
#define H09R0_ADC_H_



#endif /* H09R0_ADC_H_ */

#include "stm32f0xx_hal.h"

DMA_HandleTypeDef hdma_adc;
ADC_HandleTypeDef hadc;

#define Thermocable_input_Pin GPIO_PIN_7
#define Thermocable_input_GPIO_Port GPIOA

//extern uint32_t Thermo_buffer[1]={0};
