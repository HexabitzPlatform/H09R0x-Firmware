/*
    BitzOS (BOS) V0.2.7 - Copyright (C) 2017-2022 Hexabitz
    All rights reserved
		
    File Name     : H09R0.h
 Description   : Header file for module H09R0
  	  	  	  	 thermocouple temperature sensor module
*/
	
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef H09R0_H
#define H09R0_H

/* Includes ------------------------------------------------------------------*/
#include "BOS.h"
#include "H09R0_MemoryMap.h"
#include "H09R0_uart.h"
#include "H09R0_gpio.h"
#include "H09R0_dma.h"
#include "H09R0_inputs.h"
#include "H09R0_eeprom.h"
	
	
/* Exported definitions -------------------------------------------------------*/

#define	modulePN		_H09R0
#define _IND_LED_PORT		GPIOA
#define _IND_LED_PIN		GPIO_PIN_11

/* module constants */
#define KELVIN_RATIO         273.15
#define FAHRENHEIT_RATIO     1.8
#define FAHRENHEIT_BIAS      32
/* Port-related definitions */
#define	NumOfPorts		5
#define P_PROG 				P2						/* ST factory bootloader UART */

/* Define available ports */
#define _P1 
#define _P2 
#define _P3 
#define _P4 
#define _P5 

/* Define available USARTs */
#define _Usart1 1
#define _Usart2 1
#define _Usart3 1
#define _Usart4 1
#define _Usart6 1

/* Port-UART mapping */
#define P1uart &huart4
#define P2uart &huart2	
#define P3uart &huart6	
#define P4uart &huart3
#define P5uart &huart1

/* Port Definitions */
#define	USART1_TX_PIN		GPIO_PIN_9
#define	USART1_RX_PIN		GPIO_PIN_10
#define	USART1_TX_PORT		GPIOA
#define	USART1_RX_PORT		GPIOA
#define	USART1_AF			GPIO_AF1_USART1

#define	USART2_TX_PIN		GPIO_PIN_2
#define	USART2_RX_PIN		GPIO_PIN_3
#define	USART2_TX_PORT		GPIOA
#define	USART2_RX_PORT		GPIOA
#define	USART2_AF			GPIO_AF1_USART2

#define	USART3_TX_PIN		GPIO_PIN_10
#define	USART3_RX_PIN		GPIO_PIN_11
#define	USART3_TX_PORT		GPIOB
#define	USART3_RX_PORT		GPIOB
#define	USART3_AF			GPIO_AF4_USART3

#define	USART4_TX_PIN		GPIO_PIN_0
#define	USART4_RX_PIN		GPIO_PIN_1
#define	USART4_TX_PORT		GPIOA
#define	USART4_RX_PORT		GPIOA
#define	USART4_AF			GPIO_AF4_USART4

#define	USART5_TX_PIN		GPIO_PIN_3
#define	USART5_RX_PIN		GPIO_PIN_4
#define	USART5_TX_PORT	    GPIOB
#define	USART5_RX_PORT		GPIOB
#define	USART5_AF			GPIO_AF4_USART5

#define	USART6_TX_PIN		GPIO_PIN_4
#define	USART6_RX_PIN		GPIO_PIN_5
#define	USART6_TX_PORT		GPIOA
#define	USART6_RX_PORT		GPIOA
#define	USART6_AF			GPIO_AF5_USART6


#define NUM_MODULE_PARAMS		1

typedef enum  { STATE_OFF, STATE_ON, STATE_PWM } Relay_state_t; 


#define TIMERID_TIMEOUT_MEASUREMENT   0xFF
#define STOP_MEASUREMENT_RANGING      0
#define START_MEASUREMENT_RANGING     1

/* H01R0_Status Type Definition */  
typedef enum 
{
    H09R0_OK = 0,
	H09R0_ERR_UnknownMessage = 1,
	H09R0_ERR_WrongParams = 2,
	H09R0_ERROR = 255
} Module_Status;


/* Indicator LED */


/* Export UART variables */
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;
extern UART_HandleTypeDef huart4;
extern UART_HandleTypeDef huart6;

/* Define ADC_DMA Init prototypes */
extern DMA_HandleTypeDef hdma_adc;
extern ADC_HandleTypeDef hadc;

/* Define UART Init prototypes */
extern void MX_USART1_UART_Init(void);
extern void MX_USART2_UART_Init(void);
extern void MX_USART3_UART_Init(void);
extern void MX_USART4_UART_Init(void);
extern void MX_USART6_UART_Init(void);

/* Define UART Init prototypes */
extern void MX_ADC_Init(void);
extern void ADC_Channel_config(void);

extern uint32_t Thermo_buffer[1];
/* -----------------------------------------------------------------------
	|																APIs	 																 	|
   ----------------------------------------------------------------------- 
*/

extern int SampleC(float * temp);
extern int SampleF(float * temp);
extern int SampleK(float * temp);
extern float CalculationTemp(void);
void remoteBootloaderUpdate(uint8_t src,uint8_t dst,uint8_t inport,uint8_t outport);
void SetupPortForRemoteBootloaderUpdate(uint8_t port);

#endif
/************************ (C) COPYRIGHT HEXABITZ *****END OF FILE****/
