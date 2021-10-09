/*
    BitzOS (BOS) V0.2.5 - Copyright (C) 2017-2021 Hexabitz
    All rights reserved

    File Name     : H09R0_adc.c
    Description   : Peripheral ADC setup source file.
*/


#include <H09R0_adc.h>


/* ADC init function */

void ADC_Channel_config(void)
{
	GPIO_InitTypeDef GPIO_InitStruct ={0};
	GPIO_InitStruct.Pin = Thermocable_input_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(Thermocable_input_GPIO_Port,&GPIO_InitStruct);

}


