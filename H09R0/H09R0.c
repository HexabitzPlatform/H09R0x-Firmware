/*
 BitzOS (BOS) V0.2.5 - Copyright (C) 2017-2021 Hexabitz
 All rights reserved

 File Name     : H09R0.c
 Description   : Source code for module H09R0
  	  	  	  	 thermocouple temperature sensor module


 Required MCU resources :

 >> USARTs 1,2,3,4,6 for module ports.
 >> Timer 3 (Ch3) for Relay PWM (H0FR6 only).
 >> GPIOB 0 for Relay.

 */

/* Includes ------------------------------------------------------------------*/
#include "BOS.h"
#include "H09R0_adc.h"
/* Define UART variables */
UART_HandleTypeDef huart4;
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;
UART_HandleTypeDef huart6;

/* Module exported parameters ------------------------------------------------*/
module_param_t modParam[NUM_MODULE_PARAMS] ={{.paramPtr = NULL, .paramFormat =FMT_FLOAT, .paramName =""}};

/* Exported variables */
extern FLASH_ProcessTypeDef pFlash;
extern uint8_t numOfRecordedSnippets;
uint32_t Thermo_buffer[1] ={0};
float raw_adc, thermo_volt, tempC;
extern uint8_t GPIO_flag1;

TIM_HandleTypeDef htim3;

TaskHandle_t ThermocoupleHandle = NULL;
TimerHandle_t xTimer = NULL;
uint8_t startMeasurementRanging = STOP_MEASUREMENT_RANGING;

uint32_t temp32;
float thermo_samples[10], temp_C, lps_out_1 =0, lps_out;
float DATA_To_SEND =0.0f;     //float
float DATA_To_SEND1 =0.0f;
float DATA_To_SEND2 __attribute__((section(".mySection")));
/* Private variables ---------------------------------------------------------*/

#define REF 1.24
#define Temp_drift 0.005
#define beta 0.1
#define Celsius                    1
#define Kelvin                     2
#define Fahrenheit                 3

#define IDLE_CASE                0
#define STREAM_CLI_CASE          1
#define STREAM_PORT_CASE         2
#define STREAM_BUFFER_CASE       3
#define STREAM_CLI_VERBOSE_CASE  4
#define SAMPLE_CLI_CASE          6
#define SAMPLE_PORT_CASE         7
#define SAMPLE_BUFFER_CASE       8
#define SAMPLE_CLI_VERBOSE_CASE  9

uint8_t global_port, global_module, global_mode, unit = Celsius;
uint32_t global_period, global_timeout;
float thermo_buffer;
float *ptr_thermo_buffer;

uint8_t H09R0_DATA_FORMAT =FMT_FLOAT;
float H09R0_Temp_C;
float H09R0_Temp_F;
float H09R0_Temp_K;

/* Private function prototypes -----------------------------------------------*/
int
SendResults(float message,uint8_t Mode,uint8_t unit,uint8_t Port,uint8_t Module,float *Buffer);
float
CalculationTemp(void);
//static void HandleTimeout(TimerHandle_t xTimer);
int
SampleC(float *temp);
int
SampleF(float *temp);
int
Samplek(float *temp);
int
StreamCToPort(uint8_t Port,uint8_t Module,uint32_t Period,uint32_t Timeout);

int
StreamFToPort(uint8_t Port,uint8_t Module,uint32_t Period,uint32_t Timeout);

int
StreamKToPort(uint8_t Port,uint8_t Module,uint32_t Period,uint32_t Timeout);

void
ThermocoupleTask(void *argument);
void
TimerTask(void *argument);
static void
CheckForEnterKey(void);
static void
HandleTimeout(TimerHandle_t xTimer);

/* Create CLI commands --------------------------------------------------------*/

static portBASE_TYPE
sampleCommand(int8_t *pcWriteBuffer,size_t xWriteBufferLen,const int8_t *pcCommandString);
static portBASE_TYPE
streamCommand(int8_t *pcWriteBuffer,size_t xWriteBufferLen,const int8_t *pcCommandString);
static portBASE_TYPE
stopCommand(int8_t *pcWriteBuffer,size_t xWriteBufferLen,const int8_t *pcCommandString);
static portBASE_TYPE
unitCommand(int8_t *pcWriteBuffer,size_t xWriteBufferLen,const int8_t *pcCommandString);
static portBASE_TYPE
demoCommand(int8_t *pcWriteBuffer,size_t xWriteBufferLen,const int8_t *pcCommandString);

/*-----------------------------------------------------------*/

/* CLI command structure : demo */
const CLI_Command_Definition_t demoCommandDefinition ={(const int8_t* )"demo", /* The command string to type. */
(const int8_t* )"demo:\r\n Run a demo program to test module functionality\r\n\r\n", demoCommand, /* The function to run. */
0
/* one parameter is expected. */
};

/* CLI command structure : sample */
const CLI_Command_Definition_t sampleCommandDefinition ={(const int8_t* )"sample", /* The command string to type. */
(const int8_t* )"sample:\r\n Take one sample with the determinated unit \r\n\r\n", sampleCommand, /* The function to run. */
0
/* one parameter is expected. */
};

/* CLI command structure : stream */
const CLI_Command_Definition_t streamCommandDefinition ={(const int8_t* )"stream", /* The command string to type. */
(const int8_t* )"stream:\r\n Stream temp to the CLI, buffer or port with period (ms) and total time (ms). \r\n\r\n", streamCommand, /* The function to run. */
-1
/* Multiparameters are expected. */
};

/* CLI command structure : unit */
const CLI_Command_Definition_t unitCommandDefinition ={(const int8_t* )"unit", /* The command string to type. */

(const int8_t* )"unit:\r\n Set the measurement unit (C, F, K )\r\n\r\n", unitCommand, /* The function to run. */
1
/* one parameter is expected. */
};

/* CLI command structure : stop */
const CLI_Command_Definition_t stopCommandDefinition ={(const int8_t* )"stop", /* The command string to type. */
(const int8_t* )"stop:\r\n Stop streaming \r\n\r\n", stopCommand, /* The function to run. */
0
/* No parameters are expected. */
};

/* -----------------------------------------------------------------------
 |												 Private Functions	 														|
 -----------------------------------------------------------------------
 */

void SystemClock_Config(void){
	RCC_OscInitTypeDef RCC_OscInitStruct;
	RCC_ClkInitTypeDef RCC_ClkInitStruct;
	RCC_PeriphCLKInitTypeDef PeriphClkInit;

	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue =16;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
	RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
	HAL_RCC_OscConfig(&RCC_OscInitStruct);

	RCC_ClkInitStruct.ClockType =(RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1);
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
	HAL_RCC_ClockConfig(&RCC_ClkInitStruct,FLASH_LATENCY_1);

	PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1 | RCC_PERIPHCLK_USART2 | RCC_PERIPHCLK_USART3;
	PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK1;
	PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
	PeriphClkInit.Usart3ClockSelection = RCC_USART3CLKSOURCE_PCLK1;
	HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit);

	__HAL_RCC_PWR_CLK_ENABLE();
	HAL_PWR_EnableBkUpAccess();
	PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC;
	PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_HSE_DIV32;
	HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit);

	HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq() / 1000);

	HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

	__SYSCFG_CLK_ENABLE()
	;

	/* SysTick_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(SysTick_IRQn,0,0);

}

/*-----------------------------------------------------------*/
/* --- Save array topology and Command Snippets in Flash RO --- 
 */
uint8_t SaveToRO(void){
	BOS_Status result =BOS_OK;
	HAL_StatusTypeDef FlashStatus =HAL_OK;
	uint16_t add =2, temp =0;
	uint8_t snipBuffer[sizeof(snippet_t) + 1] ={0};

	HAL_FLASH_Unlock();

	/* Erase RO area */
	FLASH_PageErase(RO_START_ADDRESS);
	FlashStatus =FLASH_WaitForLastOperation((uint32_t ) HAL_FLASH_TIMEOUT_VALUE);
	if(FlashStatus != HAL_OK){
		return pFlash.ErrorCode;
	}
	else{
		/* Operation is completed, disable the PER Bit */
		CLEAR_BIT(FLASH->CR,FLASH_CR_PER);
	}

	/* Save number of modules and myID */
	if(myID){
		temp =(uint16_t )(N << 8) + myID;
		HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD,RO_START_ADDRESS,temp);
		FlashStatus =FLASH_WaitForLastOperation((uint32_t ) HAL_FLASH_TIMEOUT_VALUE);
		if(FlashStatus != HAL_OK){
			return pFlash.ErrorCode;
		}
		else{
			/* If the program operation is completed, disable the PG Bit */
			CLEAR_BIT(FLASH->CR,FLASH_CR_PG);
		}

		/* Save topology */
		for(uint8_t i =1; i <= N; i++){
			for(uint8_t j =0; j <= MaxNumOfPorts; j++){
				if(array[i - 1][0]){
					HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD,
					RO_START_ADDRESS + add,array[i - 1][j]);
					add +=2;
					FlashStatus =FLASH_WaitForLastOperation((uint32_t ) HAL_FLASH_TIMEOUT_VALUE);
					if(FlashStatus != HAL_OK){
						return pFlash.ErrorCode;
					}
					else{
						/* If the program operation is completed, disable the PG Bit */
						CLEAR_BIT(FLASH->CR,FLASH_CR_PG);
					}
				}
			}
		}
	}

	// Save Command Snippets
	int currentAdd = RO_MID_ADDRESS;
	for(uint8_t s =0; s < numOfRecordedSnippets; s++){
		if(snippets[s].cond.conditionType){
			snipBuffer[0] =0xFE;		// A marker to separate Snippets
			memcpy((uint8_t* )&snipBuffer[1],(uint8_t* )&snippets[s],sizeof(snippet_t));
			// Copy the snippet struct buffer (20 x numOfRecordedSnippets). Note this is assuming sizeof(snippet_t) is even.
			for(uint8_t j =0; j < (sizeof(snippet_t) / 2); j++){
				HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD,currentAdd,*(uint16_t* )&snipBuffer[j * 2]);
				FlashStatus =FLASH_WaitForLastOperation((uint32_t ) HAL_FLASH_TIMEOUT_VALUE);
				if(FlashStatus != HAL_OK){
					return pFlash.ErrorCode;
				}
				else{
					/* If the program operation is completed, disable the PG Bit */
					CLEAR_BIT(FLASH->CR,FLASH_CR_PG);
					currentAdd +=2;
				}
			}
			// Copy the snippet commands buffer. Always an even number. Note the string termination char might be skipped
			for(uint8_t j =0; j < ((strlen(snippets[s].cmd) + 1) / 2); j++){
				HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD,currentAdd,*(uint16_t* )(snippets[s].cmd + j * 2));
				FlashStatus =FLASH_WaitForLastOperation((uint32_t ) HAL_FLASH_TIMEOUT_VALUE);
				if(FlashStatus != HAL_OK){
					return pFlash.ErrorCode;
				}
				else{
					/* If the program operation is completed, disable the PG Bit */
					CLEAR_BIT(FLASH->CR,FLASH_CR_PG);
					currentAdd +=2;
				}
			}
		}
	}

	HAL_FLASH_Lock();

	return result;
}

/* --- Clear array topology in SRAM and Flash RO --- 
 */
uint8_t ClearROtopology(void){
	// Clear the array 
	memset(array,0,sizeof(array));
	N =1;
	myID =0;

	return SaveToRO();
}
/* --- H0FR6 module initialization --- 
 */
void Module_Peripheral_Init(void){

	/* Array ports */
	MX_USART1_UART_Init();
	MX_USART2_UART_Init();
	MX_USART3_UART_Init();
	MX_USART4_UART_Init();
	MX_USART6_UART_Init();

	xTaskCreate(ThermocoupleTask,(const char* ) "ThermocoupleTask",(2*configMINIMAL_STACK_SIZE),NULL,osPriorityNormal - osPriorityIdle,&ThermocoupleHandle);

	MX_ADC_Init();
}

void initialValue(void)
{
	DATA_To_SEND2=0;
}
/*-----------------------------------------------------------*/

/* --- H0FR6 message processing task. 
 */
Module_Status Module_MessagingTask(uint16_t code,uint8_t port,uint8_t src,uint8_t dst,uint8_t shift){

	Module_Status result =H09R0_OK;
	uint32_t period =0;
	uint32_t timeout =0;

	switch(code){

		case (CODE_H09R0_STREAM_PORT_C):
			period =((uint32_t )cMessage[port - 1][6 + shift] << 24) + ((uint32_t )cMessage[port - 1][5 + shift] << 16) + ((uint32_t )cMessage[port - 1][4 + shift] << 8) + cMessage[port - 1][3 + shift];
			timeout =((uint32_t )cMessage[port - 1][10 + shift] << 24) + ((uint32_t )cMessage[port - 1][9 + shift] << 16) + ((uint32_t )cMessage[port - 1][8 + shift] << 8) + cMessage[port - 1][7 + shift];
			StreamCToPort(cMessage[port - 1][shift],cMessage[port - 1][1 + shift],period,timeout);
			break;

		case (CODE_H09R0_STREAM_PORT_K):
			period =((uint32_t )cMessage[port - 1][6 + shift] << 24) + ((uint32_t )cMessage[port - 1][5 + shift] << 16) + ((uint32_t )cMessage[port - 1][4 + shift] << 8) + cMessage[port - 1][3 + shift];
			timeout =((uint32_t )cMessage[port - 1][10 + shift] << 24) + ((uint32_t )cMessage[port - 1][9 + shift] << 16) + ((uint32_t )cMessage[port - 1][8 + shift] << 8) + cMessage[port - 1][7 + shift];
			StreamKToPort(cMessage[port - 1][shift],cMessage[port - 1][1 + shift],period,timeout);
			break;

		case (CODE_H09R0_STREAM_PORT_F):
			period =((uint32_t )cMessage[port - 1][6 + shift] << 24) + ((uint32_t )cMessage[port - 1][5 + shift] << 16) + ((uint32_t )cMessage[port - 1][4 + shift] << 8) + cMessage[port - 1][3 + shift];
			timeout =((uint32_t )cMessage[port - 1][10 + shift] << 24) + ((uint32_t )cMessage[port - 1][9 + shift] << 16) + ((uint32_t )cMessage[port - 1][8 + shift] << 8) + cMessage[port - 1][7 + shift];
			StreamFToPort(cMessage[port - 1][shift],cMessage[port - 1][1 + shift],period,timeout);
			break;

		case (CODE_H09R0_STOP):
			global_mode = IDLE_CASE;
			xTimerStop(xTimer,portMAX_DELAY);
			break;

		case (CODE_H09R0_SAMPLE_PORT_C):
			SampleC(&H09R0_Temp_C);
			SendResults(DATA_To_SEND,SAMPLE_PORT_CASE,Celsius,cMessage[port - 1][shift],cMessage[port - 1][1 + shift],NULL);
			break;
		case (CODE_H09R0_SAMPLE_PORT_K):
			SampleK(&H09R0_Temp_K);
			SendResults(DATA_To_SEND,SAMPLE_PORT_CASE,Kelvin,cMessage[port - 1][shift],cMessage[port - 1][1 + shift],NULL);

			break;
		case (CODE_H09R0_SAMPLE_PORT_F):
			SampleF(&H09R0_Temp_F);
			SendResults(DATA_To_SEND,SAMPLE_PORT_CASE,Fahrenheit,cMessage[port - 1][shift],cMessage[port - 1][1 + shift],NULL);

			break;

		default:
			result =H09R0_ERR_UnknownMessage;
			break;
	}

	return result;

}

/*-----------------------------------------------------------*/

/* --- Register this module CLI Commands 
 */
void RegisterModuleCLICommands(void){

	FreeRTOS_CLIRegisterCommand(&sampleCommandDefinition);
	FreeRTOS_CLIRegisterCommand(&streamCommandDefinition);
	FreeRTOS_CLIRegisterCommand(&stopCommandDefinition);
	FreeRTOS_CLIRegisterCommand(&unitCommandDefinition);
	FreeRTOS_CLIRegisterCommand(&demoCommandDefinition);

}

/*-----------------------------------------------------------*/

/* --- Get the port for a given UART. 
 */
uint8_t GetPort(UART_HandleTypeDef *huart){
	if(huart->Instance == USART4)
		return P1;
	else if(huart->Instance == USART2)
		return P2;
	else if(huart->Instance == USART6)
		return P3;
	else if(huart->Instance == USART3)
		return P4;
	else if(huart->Instance == USART1)
		return P5;

	return 0;
}

/*-----------------------------------------------------------*/

/* --- send results to x --- */

int SendResults(float message,uint8_t Mode,uint8_t Unit,uint8_t Port,uint8_t Module,float *Buffer){
	float Raw_Msg =0.0f;
	uint32_t RawMsgInt =0;
	int8_t *pcOutputString;
	static const int8_t *pcWeightMsg =(int8_t* )"Temp (%s): %f\r\n";
	static const int8_t *pcWeightVerboseMsg =(int8_t* )"%f\r\n";
	static const int8_t *pcWeightMsgUINT =(int8_t* )"Temp (%s): %d\r\n";
	static const int8_t *pcWeightVerboseMsgUINT =(int8_t* )"%d\r\n";
	char *strUnit;
	static uint8_t temp[4];
	/* specify the unit */

	Raw_Msg =message;

	/* Get CLI output buffer */
	pcOutputString =FreeRTOS_CLIGetOutputBuffer();

	if(Mode != STREAM_CLI_VERBOSE_CASE && Mode != STREAM_PORT_CASE){
		strUnit =malloc(6 * sizeof(char));
		memset(strUnit,0,(6 * sizeof(char)));
		if(unit == Celsius){
			sprintf((char* )strUnit,"Celsius");
		}
		else if(unit == Kelvin){
			sprintf((char* )strUnit,"Kelvin");
		}
		else if(unit == Fahrenheit){
			sprintf((char* )strUnit,"Fahrenheit");
		}
		else{
			sprintf((char* )strUnit,"Celsius");
		}
	}

	// Send the value to appropriate outlet
	switch(Mode){
		case SAMPLE_CLI_CASE:
		case STREAM_CLI_CASE:
			if(H09R0_DATA_FORMAT == FMT_UINT32){
				RawMsgInt =Raw_Msg * 10;
				sprintf((char* )pcOutputString,(char* )pcWeightMsgUINT,strUnit,RawMsgInt);
				writePxMutex(PcPort,(char* )pcOutputString,strlen((char* )pcOutputString),cmd500ms,
				HAL_MAX_DELAY);
				CheckForEnterKey();
			}
			else if(H09R0_DATA_FORMAT == FMT_FLOAT){
				sprintf((char* )pcOutputString,(char* )pcWeightMsg,strUnit,Raw_Msg);
				writePxMutex(PcPort,(char* )pcOutputString,strlen((char* )pcOutputString),cmd500ms,
				HAL_MAX_DELAY);
				CheckForEnterKey();
			}
			break;

		case SAMPLE_PORT_CASE:
		case STREAM_PORT_CASE:
			if(H09R0_DATA_FORMAT == FMT_UINT32){
				RawMsgInt =Raw_Msg * 10;
				if(Module == myID){
					temp[0] =*((__IO uint8_t* )(&RawMsgInt) + 3);
					temp[1] =*((__IO uint8_t* )(&RawMsgInt) + 2);
					temp[2] =*((__IO uint8_t* )(&RawMsgInt) + 1);
					temp[3] =*((__IO uint8_t* )(&RawMsgInt) + 0);
					writePxMutex(Port,(char* )&temp,4 * sizeof(uint8_t),10,10);
				}
				else{
					messageParams[0] =Port;
					messageParams[1] =*((__IO uint8_t* )(&RawMsgInt) + 3);
					messageParams[2] =*((__IO uint8_t* )(&RawMsgInt) + 2);
					messageParams[3] =*((__IO uint8_t* )(&RawMsgInt) + 1);
					messageParams[4] =*((__IO uint8_t* )(&RawMsgInt) + 0);
					SendMessageToModule(Module,CODE_PORT_FORWARD,sizeof(uint32_t) + 1);
				}

			}
			else if(H09R0_DATA_FORMAT == FMT_FLOAT){
				if(Module == myID){
					temp[0] =*((__IO uint8_t* )(&Raw_Msg) + 3);
					temp[1] =*((__IO uint8_t* )(&Raw_Msg) + 2);
					temp[2] =*((__IO uint8_t* )(&Raw_Msg) + 1);
					temp[3] =*((__IO uint8_t* )(&Raw_Msg) + 0);
					writePxMutex(Port,(char* )&temp,4 * sizeof(uint8_t),10,10);
				}
				else{
					messageParams[0] =Port;
					messageParams[1] =*((__IO uint8_t* )(&Raw_Msg) + 3);
					messageParams[2] =*((__IO uint8_t* )(&Raw_Msg) + 2);
					messageParams[3] =*((__IO uint8_t* )(&Raw_Msg) + 1);
					messageParams[4] =*((__IO uint8_t* )(&Raw_Msg) + 0);
					SendMessageToModule(Module,CODE_PORT_FORWARD,sizeof(float) + 1);
				}
			}

			break;

		case SAMPLE_BUFFER_CASE:
		case STREAM_BUFFER_CASE:
			memset(Buffer,0,sizeof(float));
			memcpy(Buffer,&Raw_Msg,sizeof(float));
			break;
	}
	if(Mode != STREAM_CLI_VERBOSE_CASE && Mode != STREAM_PORT_CASE){
		free(strUnit);
	}
	return (H09R0_OK);
}

/*-----------------------------------------------------------*/

/* --- thermocouple stream task
 */
void ThermocoupleTask(void *argument){
	ADC_ChannelConfTypeDef sConfig ={0};
	while(1){

		sConfig.Channel = ADC_CHANNEL_7;
		sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
		sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
		if(HAL_ADC_ConfigChannel(&hadc,&sConfig) != HAL_OK){

		}
		HAL_ADC_Start(&hadc);
		HAL_ADC_PollForConversion(&hadc,100);
		Thermo_buffer[0] = HAL_ADC_GetValue(&hadc);
		HAL_ADC_Stop(&hadc);

		uint32_t t0 = 0;

		sConfig.Channel = ADC_CHANNEL_7;
		sConfig.Rank = ADC_RANK_NONE;
		sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
		if(HAL_ADC_ConfigChannel(&hadc,&sConfig) != HAL_OK){

		}
		switch(unit){
			case Celsius:
				DATA_To_SEND =CalculationTemp();
				break;
			case Fahrenheit:
				DATA_To_SEND =(CalculationTemp() * FAHRENHEIT_RATIO) + FAHRENHEIT_BIAS;
				break;
			case Kelvin:
				DATA_To_SEND =CalculationTemp() + KELVIN_RATIO;
				break;
			default:
				DATA_To_SEND =CalculationTemp();
				break;

		}

		switch(global_mode){
			case STREAM_CLI_CASE:
				t0 =HAL_GetTick();
				SendResults(DATA_To_SEND,global_mode,unit,0,0,NULL);
				while(HAL_GetTick() - t0 < (global_period - 1)){
					taskYIELD();
				}
				break;

			case STREAM_PORT_CASE:
				t0 =HAL_GetTick();
				SendResults(DATA_To_SEND,global_mode,unit,global_port,global_module,NULL);
				while(HAL_GetTick() - t0 < global_period){
					taskYIELD();
				}
				break;

			case STREAM_BUFFER_CASE:
				t0 =HAL_GetTick();
				SendResults(DATA_To_SEND,global_mode,unit,0,0,ptr_thermo_buffer);
				while(HAL_GetTick() - t0 < global_period){
					taskYIELD();
				}
				break;

			default:
				global_mode = IDLE_CASE;
				break;
		}
		taskYIELD();
	}
}
/* -----------------------------------------------------------------------
 |							private function
 -----------------------------------------------------------------------
 */

float CalculationTemp(void){

	raw_adc =Thermo_buffer[0];
	thermo_volt =raw_adc * (3.3 / 4095);
	tempC =(thermo_volt - REF) / Temp_drift;

	lps_out =(1 - beta) * lps_out_1 + (beta * tempC);
	lps_out_1 =lps_out;

	temp_C =lps_out;
	return temp_C;

}
/*-----------------------------------------------------------*/
/* ---  Check for CLI stop key  --- */
static void CheckForEnterKey(void){
	// Look for ENTER key to stop the stream
	for(uint8_t chr =0; chr < MSG_RX_BUF_SIZE; chr++){
		if(UARTRxBuf[PcPort - 1][chr] == '\r'){
			UARTRxBuf[PcPort - 1][chr] =0;
			startMeasurementRanging = STOP_MEASUREMENT_RANGING;
			global_mode = IDLE_CASE;		      // Stop the streaming task
			xTimerStop(xTimer,0);            // Stop the timeout timer
			break;
		}
	}
}

/*-----------------------------------------------------------*/

/* --- software timer callback function
 */
static void HandleTimeout(TimerHandle_t xTimer){
	uint32_t tid =0;

	/* Get Timer ID */
	tid =(uint32_t )pvTimerGetTimerID(xTimer);
	if(TIMERID_TIMEOUT_MEASUREMENT == tid){
		global_mode = IDLE_CASE;		      // Stop the streaming task
		startMeasurementRanging = STOP_MEASUREMENT_RANGING; // stop streaming
	}
}

/* -----------------------------------------------------------------------
 |																APIs	 																 	|
 -----------------------------------------------------------------------
 */
int SampleC(float *temp){

	*temp =temp_C;
	return (H09R0_OK);
}

int SampleF(float *temp){
	*temp =(temp_C * 1.8) + 32;
	return (H09R0_OK);
}

int SampleK(float *temp){
	*temp =temp_C + 273.15;
	return (H09R0_OK);
}

int StreamCToPort(uint8_t Port,uint8_t Module,uint32_t Period,uint32_t Timeout){
	global_port =Port;
	global_module =Module;
	global_period =Period;
	global_timeout =Timeout;
	global_mode = STREAM_PORT_CASE;
	unit = Celsius;
	if((global_timeout > 0) && (global_timeout < 0xFFFFFFFF)){
		/* start software timer which will create event timeout */
		/* Create a timeout timer */
		xTimer =xTimerCreate("Timeout Measurement",pdMS_TO_TICKS(global_timeout),pdFALSE,(void* ) TIMERID_TIMEOUT_MEASUREMENT,HandleTimeout);
		/* Start the timeout timer */
		xTimerStart(xTimer,portMAX_DELAY);
	}
	return (H09R0_OK);
}

int StreamFToPort(uint8_t Port,uint8_t Module,uint32_t Period,uint32_t Timeout){
	global_port =Port;
	global_module =Module;
	global_period =Period;
	global_timeout =Timeout;
	global_mode = STREAM_PORT_CASE;
	unit = Fahrenheit;
	if((global_timeout > 0) && (global_timeout < 0xFFFFFFFF)){
		/* start software timer which will create event timeout */
		/* Create a timeout timer */
		xTimer =xTimerCreate("Timeout Measurement",pdMS_TO_TICKS(global_timeout),pdFALSE,(void* ) TIMERID_TIMEOUT_MEASUREMENT,HandleTimeout);
		/* Start the timeout timer */
		xTimerStart(xTimer,portMAX_DELAY);
	}
	return (H09R0_OK);
}

int StreamKToPort(uint8_t Port,uint8_t Module,uint32_t Period,uint32_t Timeout){
	global_port =Port;
	global_module =Module;
	global_period =Period;
	global_timeout =Timeout;
	global_mode = STREAM_PORT_CASE;
	unit = Kelvin;
	if((global_timeout > 0) && (global_timeout < 0xFFFFFFFF)){
		/* start software timer which will create event timeout */
		/* Create a timeout timer */
		xTimer =xTimerCreate("Timeout Measurement",pdMS_TO_TICKS(global_timeout),pdFALSE,(void* ) TIMERID_TIMEOUT_MEASUREMENT,HandleTimeout);
		/* Start the timeout timer */
		xTimerStart(xTimer,portMAX_DELAY);
	}
	return (H09R0_OK);
}

int StreamCToBuffer(float *Buffer,uint32_t Period,uint32_t Timeout){
	global_period =Period;
	global_timeout =Timeout;
	ptr_thermo_buffer =Buffer;
	global_mode = STREAM_BUFFER_CASE;
	unit = Celsius;
	if((global_timeout > 0) && (global_timeout < 0xFFFFFFFF)){
		/* start software timer which will create event timeout */
		/* Create a timeout timer */
		xTimer =xTimerCreate("Measurement Timeout",pdMS_TO_TICKS(global_timeout),pdFALSE,(void* ) TIMERID_TIMEOUT_MEASUREMENT,HandleTimeout);
		/* Start the timeout timer */
		xTimerStart(xTimer,portMAX_DELAY);
	}

	return (H09R0_OK);
}

int StreamFToBuffer(float *Buffer,uint32_t Period,uint32_t Timeout){
	global_period =Period;
	global_timeout =Timeout;
	ptr_thermo_buffer =Buffer;
	global_mode = STREAM_BUFFER_CASE;
	unit = Fahrenheit;
	if((global_timeout > 0) && (global_timeout < 0xFFFFFFFF)){
		/* start software timer which will create event timeout */
		/* Create a timeout timer */
		xTimer =xTimerCreate("Measurement Timeout",pdMS_TO_TICKS(global_timeout),pdFALSE,(void* ) TIMERID_TIMEOUT_MEASUREMENT,HandleTimeout);
		/* Start the timeout timer */
		xTimerStart(xTimer,portMAX_DELAY);
	}

	return (H09R0_OK);
}

int StreamKToBuffer(float *Buffer,uint32_t Period,uint32_t Timeout){
	global_period =Period;
	global_timeout =Timeout;
	ptr_thermo_buffer =Buffer;
	global_mode = STREAM_BUFFER_CASE;
	unit = Kelvin;
	if((global_timeout > 0) && (global_timeout < 0xFFFFFFFF)){
		/* start software timer which will create event timeout */
		/* Create a timeout timer */
		xTimer =xTimerCreate("Measurement Timeout",pdMS_TO_TICKS(global_timeout),pdFALSE,(void* ) TIMERID_TIMEOUT_MEASUREMENT,HandleTimeout);
		/* Start the timeout timer */
		xTimerStart(xTimer,portMAX_DELAY);
	}

	return (H09R0_OK);
}

/*------------------------------------------------*/

float Average(uint8_t samples){

	float avr =0, sum =0, DATA_To_AVR;
	for(uint8_t i =0; i < samples; i++){

		switch(unit){
			case Celsius:
				DATA_To_AVR = CalculationTemp();
				break;
			case Fahrenheit:
				DATA_To_AVR = (CalculationTemp() * 1.8) + 32;
				break;
			case Kelvin:
				DATA_To_AVR = CalculationTemp() + 273.15;
				break;
			default:
				DATA_To_AVR = CalculationTemp();
				break;
		}
		sum +=DATA_To_AVR;
	}
	avr =sum / samples;
	return avr;
}

/*-----------------------------------------------------------*/

int Stop(void){
	global_mode = IDLE_CASE;

	xTimerStop(xTimer,0);

	return H09R0_OK;
}

/* --- stream weightvalue from channel ch to CLI
 */
int StreamCToCLI(uint32_t Period,uint32_t Timeout){

	global_period =Period;
	global_timeout =Timeout;
	global_mode =STREAM_CLI_CASE;
	unit =Celsius;
	if((global_timeout > 0) && (global_timeout < 0xFFFFFFFF)){
		/* start software timer which will create event timeout */
		/* Create a timeout timer */
		xTimer =xTimerCreate("Measurement Timeout",pdMS_TO_TICKS(global_timeout),pdFALSE,(void* ) TIMERID_TIMEOUT_MEASUREMENT,HandleTimeout);
		/* Start the timeout timer */
		xTimerStart(xTimer,portMAX_DELAY);
	}
	if(global_timeout > 0){
		startMeasurementRanging = START_MEASUREMENT_RANGING;
	}

	return (H09R0_OK);
}

int StreamKToCLI(uint32_t Period,uint32_t Timeout){

	global_period =Period;
	global_timeout =Timeout;
	global_mode =STREAM_CLI_CASE;
	unit =Kelvin;
	if((global_timeout > 0) && (global_timeout < 0xFFFFFFFF)){
		/* start software timer which will create event timeout */
		/* Create a timeout timer */
		xTimer =xTimerCreate("Measurement Timeout",pdMS_TO_TICKS(global_timeout),pdFALSE,(void* ) TIMERID_TIMEOUT_MEASUREMENT,HandleTimeout);
		/* Start the timeout timer */
		xTimerStart(xTimer,portMAX_DELAY);
	}
	if(global_timeout > 0){
		startMeasurementRanging = START_MEASUREMENT_RANGING;
	}

	return (H09R0_OK);
}

int StreamFToCLI(uint32_t Period,uint32_t Timeout){

	global_period =Period;
	global_timeout =Timeout;
	global_mode =STREAM_CLI_CASE;
	unit =Fahrenheit;
	if((global_timeout > 0) && (global_timeout < 0xFFFFFFFF)){
		/* start software timer which will create event timeout */
		/* Create a timeout timer */
		xTimer =xTimerCreate("Measurement Timeout",pdMS_TO_TICKS(global_timeout),pdFALSE,(void* ) TIMERID_TIMEOUT_MEASUREMENT,HandleTimeout);
		/* Start the timeout timer */
		xTimerStart(xTimer,portMAX_DELAY);
	}
	if(global_timeout > 0){
		startMeasurementRanging = START_MEASUREMENT_RANGING;
	}

	return (H09R0_OK);
}

/* -----------------------------------------------------------------------
 |															Commands																 	|
 -----------------------------------------------------------------------
 */

static portBASE_TYPE unitCommand(int8_t *pcWriteBuffer,size_t xWriteBufferLen,const int8_t *pcCommandString){
	Module_Status result =H09R0_OK;
	int8_t *pcParameterString1;
	portBASE_TYPE xParameterStringLength1 =0;
	static const int8_t *pcMessageWrongParam =(int8_t* )"Wrong parameter!\r\n";

	/* Remove compile time warnings about unused parameters, and check the
	 write buffer is not NULL.  NOTE - for simplicity, this example assumes the
	 write buffer length is adequate, so does not check for buffer overflows. */
	(void )xWriteBufferLen;
	configASSERT(pcWriteBuffer);

	/* 1st parameter for naming of uart port: P1 to P6 */
	pcParameterString1 =(int8_t* )FreeRTOS_CLIGetParameter(pcCommandString,1,&xParameterStringLength1);
	if(!strncmp((const char* )pcParameterString1,"c",1)){
		unit = Celsius;
		strcpy((char* )pcWriteBuffer,(char* )"Used measurement unit: Celsius\r\n");
	}
	else if(!strncmp((const char* )pcParameterString1,"k",1)){
		unit = Kelvin;
		strcpy((char* )pcWriteBuffer,(char* )"Used measurement unit: Kelvin\r\n");
	}
	else if(!strncmp((const char* )pcParameterString1,"f",1)){
		unit = Fahrenheit;
		strcpy((char* )pcWriteBuffer,(char* )"Used measurement unit: Fahrenheit\r\n");
	}
	else{
		result =H09R0_ERR_WrongParams;
	}

	/* Respond to the command */
	if(H09R0_ERR_WrongParams == result){
		strcpy((char* )pcWriteBuffer,(char* )pcMessageWrongParam);
	}

	/* There is no more data to return after this single string, so return pdFALSE. */
	return pdFALSE;
}

/*-----------------------------------------------------------*/

static portBASE_TYPE sampleCommand(int8_t *pcWriteBuffer,size_t xWriteBufferLen,const int8_t *pcCommandString){
	static const int8_t *pcMessageError =(int8_t* )"Wrong parameter\r\n";

	Module_Status result =H09R0_OK;

	/* Remove compile time warnings about unused parameters, and check the
	 write buffer is not NULL.  NOTE - for simplicity, this example assumes the
	 write buffer length is adequate, so does not check for buffer overflows. */
	(void )pcCommandString;
	(void )xWriteBufferLen;
	configASSERT(pcWriteBuffer);

	switch(unit){
		case Celsius:
			SampleC(&DATA_To_SEND1);
			break;
		case Fahrenheit:
			SampleF(&DATA_To_SEND1);
			break;
		case Kelvin:
			SampleK(&DATA_To_SEND1);
			break;
		default:
			SampleC(&DATA_To_SEND1);
	}
	DATA_To_SEND2 = DATA_To_SEND1;
	global_mode = SAMPLE_CLI_CASE;
	SendResults(DATA_To_SEND1,global_mode,unit,0,0,NULL);

	if(result != H09R0_OK)
		strcpy((char* )pcWriteBuffer,(char* )pcMessageError);
	/* clean terminal output */
	memset((char* )pcWriteBuffer,0,configCOMMAND_INT_MAX_OUTPUT_SIZE);

	/* There is no more data to return after this single string, so return pdFALSE. */
	return pdFALSE;
}

/*-----------------------------------------------------------*/

static portBASE_TYPE stopCommand(int8_t *pcWriteBuffer,size_t xWriteBufferLen,const int8_t *pcCommandString){

	/* Remove compile time warnings about unused parameters, and check the
	 write buffer is not NULL.  NOTE - for simplicity, this example assumes the
	 write buffer length is adequate, so does not check for buffer overflows. */
	(void )xWriteBufferLen;
	configASSERT(pcWriteBuffer);
	thermo_buffer =0;
	Stop();

	/* There is no more data to return after this single string, so return pdFALSE. */
	return pdFALSE;
}

/*-----------------------------------------------------------*/

static portBASE_TYPE streamCommand(int8_t *pcWriteBuffer,size_t xWriteBufferLen,const int8_t *pcCommandString){
	static const int8_t *pcMessageBuffer =(int8_t* )"Streaming measurements to internal buffer. Access in the CLI using module parameters: thermocouble \n\r";
	static const int8_t *pcMessageModule =(int8_t* )"Streaming measurements to port P%d in module #%d\n\r";
	static const int8_t *pcMessageCLI =(int8_t* )"Streaming measurements to the CLI\n\n\r";
	static const int8_t *pcMessageError =(int8_t* )"Wrong parameter\r\n";
	static const int8_t *pcMessageWrongName =(int8_t* )"Wrong module name\r\n";
	int8_t *pcParameterString1; /* period */
	int8_t *pcParameterString2; /* timeout */
	int8_t *pcParameterString3; /* port or buffer */
	int8_t *pcParameterString4; /* module */
	portBASE_TYPE xParameterStringLength1 =0;
	portBASE_TYPE xParameterStringLength2 =0;
	portBASE_TYPE xParameterStringLength3 =0;
	portBASE_TYPE xParameterStringLength4 =0;
	uint32_t period =0;
	uint32_t timeout =0;
	uint8_t port =0;
	uint8_t module =0;

	Module_Status result =H09R0_OK;

	/* Remove compile time warnings about unused parameters, and check the
	 write buffer is not NULL.  NOTE - for simplicity, this example assumes the
	 write buffer length is adequate, so does not check for buffer overflows. */
	(void )xWriteBufferLen;
	configASSERT(pcWriteBuffer);

	/* Obtain the 1st parameter string: channel */
	pcParameterString1 =(int8_t* )FreeRTOS_CLIGetParameter(pcCommandString,1,&xParameterStringLength1);
	/* Obtain the 2nd parameter string: period */
	pcParameterString2 =(int8_t* )FreeRTOS_CLIGetParameter(pcCommandString,2,&xParameterStringLength2);
	/* Obtain the 3rd parameter string: timeout */
	pcParameterString3 =(int8_t* )FreeRTOS_CLIGetParameter(pcCommandString,3,&xParameterStringLength3);
	/* Obtain the 4th parameter string: port */
	pcParameterString4 =(int8_t* )FreeRTOS_CLIGetParameter(pcCommandString,4,&xParameterStringLength4);

	if(NULL != pcParameterString1){
		period =atoi((char* )pcParameterString1);
	}
	else{
		result =H09R0_ERR_WrongParams;
	}

	if(NULL != pcParameterString2){
		if(!strncmp((const char* )pcParameterString2,"inf",3)){
			timeout = portMAX_DELAY;
		}
		else{
			timeout =atoi((char* )pcParameterString2);
		}
	}
	else{
		result =H09R0_ERR_WrongParams;
	}
	/* streaming data to internal buffer (module parameter) */
	if(NULL != pcParameterString3 && !strncmp((const char* )pcParameterString3,"buffer",6)){
		strcpy((char* )pcWriteBuffer,(char* )pcMessageBuffer);
		switch(unit){
			case Celsius:
				StreamCToBuffer(&thermo_buffer,period,timeout);
				break;
			case Fahrenheit:
				StreamFToBuffer(&thermo_buffer,period,timeout);
				break;
			case Kelvin:
				StreamKToBuffer(&thermo_buffer,period,timeout);
				break;
			default:
				StreamCToBuffer(&thermo_buffer,period,timeout);
		}

		// Return right away here as we don't want to block the CLI
		return pdFALSE;
	}
	/* streaming data to port */
	else if(NULL != pcParameterString3 && NULL != pcParameterString4 && pcParameterString3[0] == 'p'){
		port =(uint8_t )atol((char* )pcParameterString3 + 1);
		module =(uint8_t )GetID((char* )pcParameterString4);
		if(module != (uint8_t )BOS_ERR_WrongName){
			sprintf((char* )pcWriteBuffer,(char* )pcMessageModule,port,module);

			switch(unit){
				case Celsius:
					StreamCToPort(port,module,period,timeout);
					break;
				case Fahrenheit:
					StreamFToPort(port,module,period,timeout);
					break;
				case Kelvin:
					StreamKToPort(port,module,period,timeout);
					break;
				default:
					StreamCToPort(port,module,period,timeout);
			}
			// Return right away here as we don't want to block the CLI
			return pdFALSE;
		}
		else{
			strcpy((char* )pcWriteBuffer,(char* )pcMessageWrongName);
		}
	}

	/* Stream to the CLI */
	else if(NULL == pcParameterString3){

		strcpy((char* )pcWriteBuffer,(char* )pcMessageCLI);
		writePxMutex(PcPort,(char* )pcWriteBuffer,strlen((char* )pcWriteBuffer),cmd50ms,HAL_MAX_DELAY);
		switch(unit){
			case Celsius:
				StreamCToCLI(period,timeout);
				break;
			case Fahrenheit:
				StreamFToCLI(period,timeout);
				break;
			case Kelvin:
				StreamKToCLI(period,timeout);
				break;
			default:
				StreamCToCLI(period,timeout);
		}

		/* Wait till the end of stream */
		while(startMeasurementRanging != STOP_MEASUREMENT_RANGING){
			taskYIELD();
		}
		/* clean terminal output */
		memset((char* )pcWriteBuffer,0,strlen((char* )pcWriteBuffer));
	}

	else{
		result =H09R0_ERR_WrongParams;
	}

	if(H09R0_ERR_WrongParams == result){
		strcpy((char* )pcWriteBuffer,(char* )pcMessageError);
	}

	/* There is no more data to return after this single string, so return pdFALSE. */
	return pdFALSE;

}

/*-----------------------------------------------------------*/

portBASE_TYPE demoCommand(int8_t *pcWriteBuffer,size_t xWriteBufferLen,const int8_t *pcCommandString){
	static const int8_t *pcMessage =(int8_t* )"Streaming thermocouble temp: \r\n";
	static const int8_t *pcMessageError =(int8_t* )"Wrong parameter\r\n";
	portBASE_TYPE xParameterStringLength1 =0;
	Module_Status result =H09R0_OK;

	/* Remove compile time warnings about unused parameters, and check the
	 write buffer is not NULL.  NOTE - for simplicity, this example assumes the
	 write buffer length is adequate, so does not check for buffer overflows. */
	(void )pcCommandString;
	(void )xWriteBufferLen;
	configASSERT(pcWriteBuffer);

	/* Respond to the command */

	strcpy((char* )pcWriteBuffer,(char* )pcMessage);
	writePxMutex(PcPort,(char* )pcWriteBuffer,strlen((char* )pcWriteBuffer),cmd50ms,HAL_MAX_DELAY);
	switch(unit){
		case Celsius:
			StreamCToCLI(500,10000);
			break;
		case Fahrenheit:
			StreamFToCLI(500,10000);
			break;
		case Kelvin:
			StreamKToCLI(500,10000);
			break;
		default:
			StreamCToCLI(500,10000);
	}

	/* Wait till the end of stream */
	while(startMeasurementRanging != STOP_MEASUREMENT_RANGING){
		Delay_ms(1);
	};

	if(result != H09R0_OK){
		strcpy((char* )pcWriteBuffer,(char* )pcMessageError);
	}

	/* clean terminal output */
	memset((char* )pcWriteBuffer,0,strlen((char* )pcWriteBuffer));

	/* There is no more data to return after this single string, so return
	 pdFALSE. */
	return pdFALSE;
}
/*-------------------------------------------*/

/*-------------------------------------------------------*/

