/*
    BitzOS (BOS) V0.2.6 - Copyright (C) 2017-2022 Hexabitz
    All rights reserved

    File Name     : H09R0_adc.h
    Description   : Peripheral ADC setup header file.
*/

#ifndef H09R0_ADC_H_
#define H09R0_ADC_H_



#endif /* H09R0_ADC_H_ */

#include "stm32f0xx_hal.h"

DMA_HandleTypeDef hdma_adc;
ADC_HandleTypeDef hadc;
#define Thermocable_input_Pin GPIO_PIN_7
#define Thermocable_input_GPIO_Port GPIOA

