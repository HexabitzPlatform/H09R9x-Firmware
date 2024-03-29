/*
    BitzOS (BOS)V0.2.9 - Copyright (C) 2017-2023 Hexabitz
    All rights reserved

    File Name     : H09R9.c
    Description   : Source code for module H09R9.
					TEMPERATURE SENSOR

		Required MCU resources :

			>> USARTs 2,3,4,5,6 for module ports.
			>> PA6 for HX711 RATE control.
			>> PA9 for HX711 PD_SCK.
			>> PA10 for HX711 DOUT.
			>> Gain of ch1 is fixed at 128.
			>> Gain of ch2 is fixed at 32.

*/

/* Includes ------------------------------------------------------------------*/
#include "BOS.h"
#include "stdio.h"
#include "stdlib.h"
#include "string.h"
#include "H09R9_inputs.h"
/* Define UART variables */
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;
UART_HandleTypeDef huart4;
UART_HandleTypeDef huart5;
UART_HandleTypeDef huart6;

#define Delay 10
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
void Measure_temp_init(void);
float Measure_temp(void);
float bytesToFloat(uchar b0, uchar b1, uchar b2, uchar b3);
/* Exported variables */
extern FLASH_ProcessTypeDef pFlash;
extern uint8_t numOfRecordedSnippets;
extern I2C_HandleTypeDef hi2c2;
/* Module exported parameters ------------------------------------------------*/

float H09R9_temperature;
module_param_t modParam[NUM_MODULE_PARAMS] = {
{.paramPtr=&H09R9_temperature, .paramFormat=FMT_FLOAT, .paramName="temperature"},

};
#define MIN_MEMS_PERIOD_MS				100
#define MAX_MEMS_TIMEOUT_MS				0xFFFFFFFF
typedef void (*SampleMemsToPort)(uint8_t, uint8_t);
typedef void (*SampleMemsToString)(char *, size_t);
typedef void (*SampleMemsToBuffer)(float *buffer);

/*Define private variables*/
static bool stopStream = false;

//STM32CubeMonitor variables
float temp __attribute__((section(".mySection")));

//module variables
typedef unsigned char uchar;
static const uint8_t TSD305_ADDR=0x00<<1; // 0b0000000X

uint8_t SensorAddresses;
uint8_t ADC_Adress=0xAF;
uint8_t buf1[3]={0x00};
uint8_t buf2[3]={0x00};
uint8_t buf4[7]={0x00};
uint8_t h_add1,l_add1,h_add2,l_add2,h_add3,l_add3,h_add4,l_add4,h_add5,l_add5,h_add6,l_add6,h_add7,l_add7,h_add8,l_add8,h_add9,l_add9,h_add10,l_add10;
uint8_t h_add11,l_add11,h_add12,l_add12,h_add13,l_add13,h_add14,l_add14,h_add15,l_add15,h_add16,l_add16,h_add17,l_add17,h_add18,l_add18,h_add19,l_add19,h_add20,l_add20;
uint8_t ADC0,ADCsen3,ADCsen2,ADCsen1,ADCobj3,ADCobj2,ADCobj1;
int32_t ADCsen,ADCobj;
float offest,OffsetTC,ADCComp,ADCCompTC;
int16_t TSenMax_Value,TSenMin_Value;
float TC=0,RT=0,k0comp=0,k1comp=0,k2comp=0,k3comp=0,k4comp=0,k0Obj=0,k1Obj=0,k2Obj=0,k3Obj=0,k4Obj=0,TCF=0,Tsen=0;
uint8_t h_TSenMax, l_TSenMax,h_TSenMin,l_TSenMin;
uint8_t h_TC1, l_TC1,h_TC2,l_TC2;
uint8_t h_RT1, l_RT1,h_RT2,l_RT2;
/* Private function prototypes -----------------------------------------------*/
static Module_Status StreamMemsToBuf( float *buffer, uint32_t period, uint32_t timeout, SampleMemsToBuffer function);
static Module_Status StreamMemsToPort(uint8_t port, uint8_t module, uint32_t period, uint32_t timeout, SampleMemsToPort function);
static Module_Status StreamMemsToCLI(uint32_t period, uint32_t timeout, SampleMemsToString function);
static Module_Status PollingSleepCLISafe(uint32_t period);


/* Create CLI commands --------------------------------------------------------*/
static portBASE_TYPE SampleSensorCommand(int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString);
static portBASE_TYPE StreamSensorCommand(int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString);
static portBASE_TYPE StopStreamCommand(int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString);

/* CLI command structure : sample */
const CLI_Command_Definition_t SampleCommandDefinition = {
	(const int8_t *) "sample",
	(const int8_t *) "sample:\r\n Syntax: sample.\r\n\r\n",
	SampleSensorCommand,
	0

};
/* CLI command structure : stream */
const CLI_Command_Definition_t StreamCommandDefinition = {
	(const int8_t *) "stream",
	(const int8_t *) "stream:\r\n Syntax: stream (period in ms) (time in ms) [port] [module].\r\n\r\n",
	StreamSensorCommand,
	-1
};
/* CLI command structure : stop */
const CLI_Command_Definition_t StopCommandDefinition = {
	(const int8_t *) "stop",
	(const int8_t *) "stop:\r\n Syntax: stop\r\n \
\tStop the current streaming of MEMS values. r\n\r\n",
	StopStreamCommand,
	0
};

/* -----------------------------------------------------------------------
	|												 Private Functions	 														|
   -----------------------------------------------------------------------
*/
/**
  * @brief  System Clock Configuration
  *         The system Clock is configured as follow :
  *            System Clock source            = PLL (HSE)
  *            SYSCLK(Hz)                     = 48000000
  *            HCLK(Hz)                       = 48000000
  *            AHB Prescaler                  = 1
  *            APB1 Prescaler                 = 1
  *            HSE Frequency(Hz)              = 8000000
  *            PREDIV                         = 1
  *            PLLMUL                         = 6
  *            Flash Latency(WS)              = 1
  * @param  None
  * @retval None
  */
void SystemClock_Config(void)
{

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

	RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1);
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

	__SYSCFG_CLK_ENABLE();

	/* SysTick_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(SysTick_IRQn,0,0);


}



/* --- Register this module CLI Commands
*/
void RegisterModuleCLICommands(void)
{
	FreeRTOS_CLIRegisterCommand( &SampleCommandDefinition );
	FreeRTOS_CLIRegisterCommand( &StreamCommandDefinition );
	FreeRTOS_CLIRegisterCommand( &StopCommandDefinition);
}


void initialValue(void)
{

	temp=0;
}

/* --- Trigger ST factory bootloader update for a remote module.
 */
void remoteBootloaderUpdate(uint8_t src,uint8_t dst,uint8_t inport,uint8_t outport){

	uint8_t myOutport =0, lastModule =0;
	int8_t *pcOutputString;

	/* 1. Get route to destination module */
	myOutport =FindRoute(myID,dst);
	if(outport && dst == myID){ /* This is a 'via port' update and I'm the last module */
		myOutport =outport;
		lastModule =myID;
	}
	else if(outport == 0){ /* This is a remote update */
		if(NumberOfHops(dst)== 1)
		lastModule = myID;
		else
		lastModule = route[NumberOfHops(dst)-1]; /* previous module = route[Number of hops - 1] */
	}

	/* 2. If this is the source of the message, show status on the CLI */
	if(src == myID){
		/* Obtain the address of the output buffer.  Note there is no mutual
		 exclusion on this buffer as it is assumed only one command console
		 interface will be used at any one time. */
		pcOutputString =FreeRTOS_CLIGetOutputBuffer();

		if(outport == 0)		// This is a remote module update
			sprintf((char* )pcOutputString,pcRemoteBootloaderUpdateMessage,dst);
		else
			// This is a 'via port' remote update
			sprintf((char* )pcOutputString,pcRemoteBootloaderUpdateViaPortMessage,dst,outport);

		strcat((char* )pcOutputString,pcRemoteBootloaderUpdateWarningMessage);
		writePxITMutex(inport,(char* )pcOutputString,strlen((char* )pcOutputString),cmd50ms);
		Delay_ms(100);
	}

	/* 3. Setup my inport and outport for bootloader update */
	SetupPortForRemoteBootloaderUpdate(inport);
	SetupPortForRemoteBootloaderUpdate(myOutport);


	/* 5. Build a DMA stream between my inport and outport */
	StartScastDMAStream(inport,myID,myOutport,myID,BIDIRECTIONAL,0xFFFFFFFF,0xFFFFFFFF,false);
}

/*-----------------------------------------------------------*/

/* --- Setup a port for remote ST factory bootloader update:
 - Set baudrate to 57600
 - Enable even parity
 - Set datasize to 9 bits
 */
void SetupPortForRemoteBootloaderUpdate(uint8_t port){
	UART_HandleTypeDef *huart =GetUart(port);

	huart->Init.BaudRate =57600;
	huart->Init.Parity = UART_PARITY_EVEN;
	huart->Init.WordLength = UART_WORDLENGTH_9B;
	HAL_UART_Init(huart);

	/* The CLI port RXNE interrupt might be disabled so enable here again to be sure */
	__HAL_UART_ENABLE_IT(huart,UART_IT_RXNE);
}
/* --- H26R0 module initialization.
*/
void Module_Peripheral_Init(void)
{
	/* Array ports */
	MX_USART1_UART_Init();
	MX_USART2_UART_Init();
	MX_USART3_UART_Init();
	MX_USART4_UART_Init();
	MX_USART5_UART_Init();
	MX_USART6_UART_Init();
	/* initialize GPIO for module */
	SENSOR_GPIO_Init();
	/* initialize I2C for module */
	MX_I2C_Init();
	/* initialize color&proximity sensor */
	SENSOR_COEFFICIENTS_Init();

}

Module_Status Module_MessagingTask(uint16_t code, uint8_t port, uint8_t src, uint8_t dst, uint8_t shift)
{
	Module_Status result = H09R9_OK;
	uint32_t period = 0, timeout = 0;

	switch (code)
	{
		case CODE_H09R9_SAMPLE_TEMP:
		{
			SampleTemperatureToPort(cMessage[port-1][1+shift] ,cMessage[port-1][shift]);
			break;
		}
		case CODE_H09R9_STREAM_TEMP:
		{
			period = ((uint32_t) cMessage[port - 1][2 + shift] ) + ((uint32_t) cMessage[port - 1][3 + shift] << 8) + ((uint32_t) cMessage[port - 1][4 + shift] << 16) + ((uint32_t)cMessage[port - 1][5 + shift] << 24);
			timeout = ((uint32_t) cMessage[port - 1][6 + shift] ) + ((uint32_t) cMessage[port - 1][7 + shift] << 8) + ((uint32_t) cMessage[port - 1][8 + shift] << 16) + ((uint32_t)cMessage[port - 1][9 + shift] << 24);
			StreamTemperatureToPort(cMessage[port-1][1+shift] ,cMessage[port-1][shift], period, timeout);
			break;
		}
		case CODE_H09R9_STREAM_STOP:
		{
			stopStreamMems();
			result = H09R9_OK;
			break;
		}

		default:
			result = H09R9_ERR_UnknownMessage;
			break;
	}

	return result;
}

/*-----------------------------------------------------------*/
/* --- Save array topology and Command Snippets in Flash RO ---
*/
static Module_Status PollingSleepCLISafe(uint32_t period)
{
	const unsigned DELTA_SLEEP_MS = 100; // milliseconds
	long numDeltaDelay =  period / DELTA_SLEEP_MS;
	unsigned lastDelayMS = period % DELTA_SLEEP_MS;

	while (numDeltaDelay-- > 0) {
		vTaskDelay(pdMS_TO_TICKS(DELTA_SLEEP_MS));

		// Look for ENTER key to stop the stream
		for (uint8_t chr=0 ; chr<MSG_RX_BUF_SIZE ; chr++)
		{
			if (UARTRxBuf[PcPort-1][chr] == '\r') {
				UARTRxBuf[PcPort-1][chr] = 0;
				return H09R9_ERR_TERMINATED;
			}
		}

		if (stopStream)
			return H09R9_ERR_TERMINATED;
	}

	vTaskDelay(pdMS_TO_TICKS(lastDelayMS));
	return H09R9_OK;
}

static Module_Status StreamMemsToBuf( float *buffer, uint32_t period, uint32_t timeout, SampleMemsToBuffer function)

{
	Module_Status status = H09R9_OK;

	if (period < MIN_MEMS_PERIOD_MS)
		return H09R9_ERR_WrongParams;

	// TODO: Check if CLI is enable or not

	if (period > timeout)
		timeout = period;

	long numTimes = timeout / period;
	stopStream = false;

	while ((numTimes-- > 0) || (timeout >= MAX_MEMS_TIMEOUT_MS)) {
		function(buffer);

		vTaskDelay(pdMS_TO_TICKS(period));
		if (stopStream) {
			status = H09R9_ERR_TERMINATED;
			break;
		}
	}
	return status;
}

static Module_Status StreamMemsToPort(uint8_t port, uint8_t module, uint32_t period, uint32_t timeout, SampleMemsToPort function)
{
	Module_Status status = H09R9_OK;


	if (period < MIN_MEMS_PERIOD_MS)
		return H09R9_ERR_WrongParams;
	if (port == 0)
		return H09R9_ERR_WrongParams;
	if (port == PcPort) // Check if CLI is not enabled at that port!
		return H09R9_ERR_BUSY;

	if (period > timeout)
		timeout = period;

	long numTimes = timeout / period;
	stopStream = false;

	while ((numTimes-- > 0) || (timeout >= MAX_MEMS_TIMEOUT_MS)) {
		function(port, module);

		vTaskDelay(pdMS_TO_TICKS(period));
		if (stopStream) {
			status = H09R9_ERR_TERMINATED;
			break;
		}
	}
	return status;
}

static Module_Status StreamMemsToCLI(uint32_t period, uint32_t timeout, SampleMemsToString function)
{
	Module_Status status = H09R9_OK;
	int8_t *pcOutputString = NULL;

	if (period < MIN_MEMS_PERIOD_MS)
		return H09R9_ERR_WrongParams;

	// TODO: Check if CLI is enable or not

	if (period > timeout)
		timeout = period;

	long numTimes = timeout / period;
	stopStream = false;

	while ((numTimes-- > 0) || (timeout >= MAX_MEMS_TIMEOUT_MS)) {
		pcOutputString = FreeRTOS_CLIGetOutputBuffer();
		function((char *)pcOutputString, 100);


		writePxMutex(PcPort, (char *)pcOutputString, strlen((char *)pcOutputString), cmd500ms, HAL_MAX_DELAY);
		if (PollingSleepCLISafe(period) != H09R9_OK)
			break;
	}

	memset((char *) pcOutputString, 0, configCOMMAND_INT_MAX_OUTPUT_SIZE);
  sprintf((char *)pcOutputString, "\r\n");
	return status;
}


void SENSOR_COEFFICIENTS_Init(void){

	HAL_I2C_IsDeviceReady(&hi2c2,TSD305_ADDR,3,100);
	Delay_ms_no_rtos(Delay);
	/*----------sensor temperature range from TSenMin to TSenMax.---------------*/
	SensorAddresses=0x1B;
	HAL_I2C_Master_Transmit(&hi2c2, TSD305_ADDR, &SensorAddresses, 1,100);
	Delay_ms_no_rtos(Delay);
	HAL_I2C_Master_Receive(&hi2c2, TSD305_ADDR, buf1, 3,100);
	TSenMax_Value = (buf1[1] << 8) + buf1[2];
	SensorAddresses=0x1A;
	HAL_I2C_Master_Transmit(&hi2c2, TSD305_ADDR, &SensorAddresses, 1,100);
	Delay_ms_no_rtos(Delay);
	HAL_I2C_Master_Receive(&hi2c2, TSD305_ADDR, buf2, 3,100);
	TSenMin_Value = (buf2[1] << 8) + buf2[2];

	/*-------------------------Temperature Coefficient---------------------*/
	SensorAddresses=0x1E;
	HAL_I2C_Master_Transmit(&hi2c2, TSD305_ADDR, &SensorAddresses, 1,100);
	Delay_ms_no_rtos(Delay);
    HAL_I2C_Master_Receive(&hi2c2, TSD305_ADDR, buf1, 3,100);

	h_TC1=buf1[1];
	l_TC1=buf1[2];


	SensorAddresses=0x1F;
	HAL_I2C_Master_Transmit(&hi2c2, TSD305_ADDR, &SensorAddresses, 1,100);
	Delay_ms_no_rtos(Delay);
    HAL_I2C_Master_Receive(&hi2c2, TSD305_ADDR, buf2, 3,100);

	h_TC2=buf2[1];
	l_TC2=buf2[2];

	TC= bytesToFloat(h_TC1, l_TC1, h_TC2,l_TC2);

/*-------------------------Reference Temperature------------------------------*/
	SensorAddresses=0x20;
	HAL_I2C_Master_Transmit(&hi2c2, TSD305_ADDR, &SensorAddresses, 1,100);
	Delay_ms_no_rtos(Delay);
    HAL_I2C_Master_Receive(&hi2c2, TSD305_ADDR, buf1, 3,100);

	h_RT1=buf1[1];
	l_RT1=buf1[2];


	SensorAddresses=0x21;
	HAL_I2C_Master_Transmit(&hi2c2, TSD305_ADDR, &SensorAddresses, 1,100);
	Delay_ms_no_rtos(Delay);
    HAL_I2C_Master_Receive(&hi2c2, TSD305_ADDR, buf2, 3,100);

	h_RT2=buf2[1];
	l_RT2=buf2[2];

	RT= bytesToFloat(h_RT1, l_RT1, h_RT2,l_RT2);

/*---------------------Temperature Compensation-----------------------------------*/

/*-----------------------1-------------------------------------------------------*/
//calculate k4comp
	SensorAddresses=0x22;
	HAL_I2C_Master_Transmit(&hi2c2, TSD305_ADDR, &SensorAddresses, 1,100);
	Delay_ms_no_rtos(Delay);
	HAL_I2C_Master_Receive(&hi2c2, TSD305_ADDR, buf1, 3,100);

	h_add1=buf1[1];
	l_add1=buf1[2];


	SensorAddresses=0x23;
	HAL_I2C_Master_Transmit(&hi2c2, TSD305_ADDR, &SensorAddresses, 1,100);
	Delay_ms_no_rtos(Delay);
	HAL_I2C_Master_Receive(&hi2c2, TSD305_ADDR, buf2, 3,100);

	h_add2=buf2[1];
	l_add2=buf2[2];

	k4comp= bytesToFloat(h_add1, l_add1, h_add2,l_add2);
/*-----------------------2-------------------------------------------------------*/
//calculate k3comp
	SensorAddresses=0x24;
	HAL_I2C_Master_Transmit(&hi2c2, TSD305_ADDR, &SensorAddresses, 1,100);
	Delay_ms_no_rtos(Delay);
	HAL_I2C_Master_Receive(&hi2c2, TSD305_ADDR, buf1, 3,100);

	h_add3=buf1[1];
	l_add3=buf1[2];


	SensorAddresses=0x25;
	HAL_I2C_Master_Transmit(&hi2c2, TSD305_ADDR, &SensorAddresses, 1,100);
	Delay_ms_no_rtos(Delay);
	HAL_I2C_Master_Receive(&hi2c2, TSD305_ADDR, buf2, 3,100);

	h_add4=buf2[1];
	l_add4=buf2[2];

	k3comp= bytesToFloat(h_add3, l_add3, h_add4,l_add4);
/*-----------------------3-------------------------------------------------------*/
//calculate k2comp
	SensorAddresses=0x26;
	HAL_I2C_Master_Transmit(&hi2c2, TSD305_ADDR, &SensorAddresses, 1,100);
	Delay_ms_no_rtos(Delay);
	HAL_I2C_Master_Receive(&hi2c2, TSD305_ADDR, buf1, 3,100);

	h_add5=buf1[1];
	l_add5=buf1[2];


	SensorAddresses=0x27;
	HAL_I2C_Master_Transmit(&hi2c2, TSD305_ADDR, &SensorAddresses, 1,100);
	Delay_ms_no_rtos(Delay);
	HAL_I2C_Master_Receive(&hi2c2, TSD305_ADDR, buf2, 3,100);

	h_add6=buf2[1];
	l_add6=buf2[2];

	k2comp= bytesToFloat(h_add5, l_add5, h_add6,l_add6);
/*-----------------------4-------------------------------------------------------*/
//calculate k1comp
	SensorAddresses=0x28;
	HAL_I2C_Master_Transmit(&hi2c2, TSD305_ADDR, &SensorAddresses, 1,100);
	Delay_ms_no_rtos(Delay);
	HAL_I2C_Master_Receive(&hi2c2, TSD305_ADDR, buf1, 3,100);

	h_add7=buf1[1];
	l_add7=buf1[2];


	SensorAddresses=0x29;
	HAL_I2C_Master_Transmit(&hi2c2, TSD305_ADDR, &SensorAddresses, 1,100);
	Delay_ms_no_rtos(Delay);
	HAL_I2C_Master_Receive(&hi2c2, TSD305_ADDR, buf2, 3,100);

	h_add8=buf2[1];
	l_add8=buf2[2];

	k1comp= bytesToFloat(h_add7, l_add7, h_add8,l_add8);
/*-----------------------5-------------------------------------------------------*/
//calculate k0comp
	SensorAddresses=0x2A;
	HAL_I2C_Master_Transmit(&hi2c2, TSD305_ADDR, &SensorAddresses, 1,100);
	Delay_ms_no_rtos(Delay);
	HAL_I2C_Master_Receive(&hi2c2, TSD305_ADDR, buf1, 3,100);

	h_add9=buf1[1];
	l_add9=buf1[2];


	SensorAddresses=0x2B;
	HAL_I2C_Master_Transmit(&hi2c2, TSD305_ADDR, &SensorAddresses, 1,100);
	Delay_ms_no_rtos(Delay);
	HAL_I2C_Master_Receive(&hi2c2, TSD305_ADDR, buf2, 3,100);

	h_add10=buf2[1];
	l_add10=buf2[2];

	k0comp= bytesToFloat(h_add9, l_add9, h_add10,l_add10);


/*---------------------Object Temperature Determination---------------------------*/
/*-----------------------1-------------------------------------------------------*/
//calculate k4obj
	SensorAddresses=0x2E;
	HAL_I2C_Master_Transmit(&hi2c2, TSD305_ADDR, &SensorAddresses, 1,100);
	Delay_ms_no_rtos(Delay);
	HAL_I2C_Master_Receive(&hi2c2, TSD305_ADDR, buf1, 3,100);

	h_add11=buf1[1];
	l_add11=buf1[2];


	SensorAddresses=0x2F;
	HAL_I2C_Master_Transmit(&hi2c2, TSD305_ADDR, &SensorAddresses, 1,100);
	Delay_ms_no_rtos(Delay);
	HAL_I2C_Master_Receive(&hi2c2, TSD305_ADDR, buf2, 3,100);

	h_add12=buf2[1];
	l_add12=buf2[2];

	k4Obj= bytesToFloat(h_add11, l_add11, h_add12,l_add12);
/*-----------------------2-------------------------------------------------------*/
//calculate k3obj
	SensorAddresses=0x30;
	HAL_I2C_Master_Transmit(&hi2c2, TSD305_ADDR, &SensorAddresses, 1,100);
	Delay_ms_no_rtos(Delay);
	HAL_I2C_Master_Receive(&hi2c2, TSD305_ADDR, buf1, 3,100);

	h_add13=buf1[1];
	l_add13=buf1[2];


	SensorAddresses=0x31;
	HAL_I2C_Master_Transmit(&hi2c2, TSD305_ADDR, &SensorAddresses, 1,100);
	Delay_ms_no_rtos(Delay);
	HAL_I2C_Master_Receive(&hi2c2, TSD305_ADDR, buf2, 3,100);

	h_add14=buf2[1];
	l_add14=buf2[2];

	k3Obj= bytesToFloat(h_add13, l_add13, h_add14,l_add14);
/*-----------------------3-------------------------------------------------------*/
//calculate k2obj
	SensorAddresses=0x32;
	HAL_I2C_Master_Transmit(&hi2c2, TSD305_ADDR, &SensorAddresses, 1,100);
	Delay_ms_no_rtos(Delay);
	HAL_I2C_Master_Receive(&hi2c2, TSD305_ADDR, buf1, 3,100);

	h_add15=buf1[1];
	l_add15=buf1[2];


	SensorAddresses=0x33;
	HAL_I2C_Master_Transmit(&hi2c2, TSD305_ADDR, &SensorAddresses, 1,100);
	Delay_ms_no_rtos(Delay);
	HAL_I2C_Master_Receive(&hi2c2, TSD305_ADDR, buf2, 3,100);

	h_add16=buf2[1];
	l_add16=buf2[2];

	k2Obj= bytesToFloat(h_add15, l_add15, h_add16,l_add16);
/*-----------------------4-------------------------------------------------------*/
//calculate k1obj
	SensorAddresses=0x34;
	HAL_I2C_Master_Transmit(&hi2c2, TSD305_ADDR, &SensorAddresses, 1,100);
	Delay_ms_no_rtos(Delay);
	HAL_I2C_Master_Receive(&hi2c2, TSD305_ADDR, buf1, 3,100);

	h_add17=buf1[1];
	l_add17=buf1[2];


	SensorAddresses=0x35;
	HAL_I2C_Master_Transmit(&hi2c2, TSD305_ADDR, &SensorAddresses, 1,100);
	Delay_ms_no_rtos(Delay);
	HAL_I2C_Master_Receive(&hi2c2, TSD305_ADDR, buf2, 3,100);

	h_add18=buf2[1];
	l_add18=buf2[2];

	k1Obj= bytesToFloat(h_add17, l_add17, h_add18,l_add18);
/*-----------------------5-------------------------------------------------------*/
//calculate k0obj
	SensorAddresses=0x36;
	HAL_I2C_Master_Transmit(&hi2c2, TSD305_ADDR, &SensorAddresses, 1,100);
	Delay_ms_no_rtos(Delay);
	HAL_I2C_Master_Receive(&hi2c2, TSD305_ADDR, buf1, 3,100);

	h_add19=buf1[1];
	l_add19=buf1[2];


	SensorAddresses=0x37;
	HAL_I2C_Master_Transmit(&hi2c2, TSD305_ADDR, &SensorAddresses, 1,100);
	Delay_ms_no_rtos(Delay);
	HAL_I2C_Master_Receive(&hi2c2, TSD305_ADDR, buf2, 3,100);

	h_add20=buf2[1];
	l_add20=buf2[2];

	k0Obj= bytesToFloat(h_add19, l_add19, h_add20,l_add20);
}

float bytesToFloat(uchar b0, uchar b1, uchar b2, uchar b3)
{
    float output;

    *((uchar*)(&output) + 3) = b0;
    *((uchar*)(&output) + 2) = b1;
    *((uchar*)(&output) + 1) = b2;
    *((uchar*)(&output) + 0) = b3;

    return output;
}

void SampleTemperature(float *temp){

	HAL_I2C_IsDeviceReady(&hi2c2,TSD305_ADDR,1,10);
	HAL_Delay(Delay);

	HAL_I2C_Master_Transmit(&hi2c2, TSD305_ADDR, &ADC_Adress, 1,100);
	HAL_Delay(Delay);
			do {
	HAL_I2C_Master_Receive(&hi2c2, TSD305_ADDR, buf4, 7,100);

	ADC0=buf4[0];
	ADCobj3=buf4[1];
	ADCobj2=buf4[2];
	ADCobj1=buf4[3];
	ADCsen3=buf4[4];
	ADCsen2=buf4[5];
	ADCsen1=buf4[6];

			}while(buf4[0] == 0x60 );
	ADCsen=(ADCsen3<<16)|(ADCsen2<<8)| ADCsen1;
	ADCobj=(ADCobj3<<16)|(ADCobj2<<8)| ADCobj1;
		/*----------sensor temperature range from TSenMin to TSenMax.---------------*/

	float t= ((float)ADCsen)/16777216;
	Tsen=t*(TSenMax_Value-TSenMin_Value)+TSenMin_Value;
	/*-------------------------TC Correction Factor-------------------------*/

	TCF=1+((Tsen-RT)*TC);

	offest=k4comp*Tsen*Tsen*Tsen*Tsen+k3comp*Tsen*Tsen*Tsen+k2comp*Tsen*Tsen+k1comp*Tsen+k0comp;
	OffsetTC= offest * TCF;


	ADCComp = OffsetTC + (ADCobj - 8388608)/0.99;
	ADCCompTC = ADCComp / TCF;
	HAL_Delay(10);
	*temp=k4Obj*ADCCompTC*ADCCompTC*ADCCompTC*ADCCompTC+k3Obj*ADCCompTC*ADCCompTC*ADCCompTC+k2Obj*ADCCompTC*ADCCompTC+k1Obj*ADCCompTC+k0Obj;
}

void SampleTemperatureBuf(float *buffer)
{
	SampleTemperature(buffer);
}

void SampleTemperatureToPort(uint8_t port,uint8_t module)
{
	float buffer[1];
	static uint8_t temp[4];

	SampleTemperatureBuf(buffer);


	if(module == myID){
		temp[0] =*((__IO uint8_t* )(&buffer[0]) + 3);
		temp[1] =*((__IO uint8_t* )(&buffer[0]) + 2);
		temp[2] =*((__IO uint8_t* )(&buffer[0]) + 1);
		temp[3] =*((__IO uint8_t* )(&buffer[0]) + 0);


		writePxITMutex(port,(char* )&temp[0],4 * sizeof(uint8_t),10);
	}
	else{
		messageParams[0] =port;
		messageParams[1] =*((__IO uint8_t* )(&buffer[0]) + 3);
		messageParams[2] =*((__IO uint8_t* )(&buffer[0]) + 2);
		messageParams[3] =*((__IO uint8_t* )(&buffer[0]) + 1);
		messageParams[4] =*((__IO uint8_t* )(&buffer[0]) + 0);

		SendMessageToModule(module,CODE_PORT_FORWARD,sizeof(float)+1);
	}
}

void SampleTemperatureToString(char *cstring, size_t maxLen)
{
	float temprature = 0;
	SampleTemperature(&temprature);
	temp=temprature;
	snprintf(cstring, maxLen, "Temperature: %.2f\r\n", temprature);
}

Module_Status StreamTemperatureToBuffer(float *buffer, uint32_t period, uint32_t timeout)
{
	return StreamMemsToBuf(buffer, period, timeout, SampleTemperatureBuf);
}

Module_Status StreamTemperatureToPort(uint8_t port, uint8_t module, uint32_t period, uint32_t timeout)
{
	return StreamMemsToPort(port, module, period, timeout, SampleTemperatureToPort);
}

Module_Status StreamTemperatureToCLI(uint32_t period, uint32_t timeout)
{
	return StreamMemsToCLI(period, timeout, SampleTemperatureToString);
}

void stopStreamMems(void)
{
	stopStream = true;
}

static portBASE_TYPE SampleSensorCommand(int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString)
{
	// Make sure we return something
	*pcWriteBuffer = '\0';

	do {
			SampleTemperatureToString((char *)pcWriteBuffer, xWriteBufferLen);

		return pdFALSE;
	} while (0);

	snprintf((char *)pcWriteBuffer, xWriteBufferLen, "Error reading Sensor\r\n");
	return pdFALSE;
}

// Port Mode => false and CLI Mode => true
static bool StreamCommandParser(const int8_t *pcCommandString, bool *pPortOrCLI, uint32_t *pPeriod, uint32_t *pTimeout, uint8_t *pPort, uint8_t *pModule)
{
	const char *pPeriodMSStr = NULL;
	const char *pTimeoutMSStr = NULL;

	portBASE_TYPE periodStrLen = 0;
	portBASE_TYPE timeoutStrLen = 0;

	const char *pPortStr = NULL;
	const char *pModStr = NULL;

	portBASE_TYPE portStrLen = 0;
	portBASE_TYPE modStrLen = 0;

	pPeriodMSStr = (const char *)FreeRTOS_CLIGetParameter(pcCommandString, 1, &periodStrLen);
	pTimeoutMSStr = (const char *)FreeRTOS_CLIGetParameter(pcCommandString, 2, &timeoutStrLen);

	// At least 3 Parameters are required!
	if ((pPeriodMSStr == NULL) || (pTimeoutMSStr == NULL))
		return false;

	// TODO: Check if Period and Timeout are integers or not!
	*pPeriod = atoi(pPeriodMSStr);
	*pTimeout = atoi(pTimeoutMSStr);
	*pPortOrCLI = true;

	pPortStr = (const char *)FreeRTOS_CLIGetParameter(pcCommandString, 3, &portStrLen);
	pModStr = (const char *)FreeRTOS_CLIGetParameter(pcCommandString, 4, &modStrLen);

	if ((pModStr == NULL) && (pPortStr == NULL))
		return true;
	if ((pModStr == NULL) || (pPortStr == NULL))	// If user has provided 4 Arguments.
		return false;

	*pPort = atoi(pPortStr);
	*pModule = atoi(pModStr);
	*pPortOrCLI = false;

	return true;
}

static portBASE_TYPE StreamSensorCommand(int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString)
{
//	const char *const temperatureCmdName = "temp";

	uint32_t period = 0;
	uint32_t timeout = 0;
	uint8_t port = 0;
	uint8_t module = 0;

	bool portOrCLI = true; // Port Mode => false and CLI Mode => true

	const char *pSensName = NULL;
	portBASE_TYPE sensNameLen = 0;

	// Make sure we return something
	*pcWriteBuffer = '\0';

	if (!StreamCommandParser(pcCommandString, &portOrCLI, &period, &timeout, &port, &module)) {
		snprintf((char *)pcWriteBuffer, xWriteBufferLen, "Invalid Arguments\r\n");
		return pdFALSE;
	}

	do {

			if (portOrCLI) {
				StreamTemperatureToCLI(period, timeout);

			} else {
				StreamTemperatureToPort(port, module, period, timeout);

			}

		snprintf((char *)pcWriteBuffer, xWriteBufferLen, "\r\n");
		return pdFALSE;
	} while (0);

	snprintf((char *)pcWriteBuffer, xWriteBufferLen, "Error reading Sensor\r\n");
	return pdFALSE;
}

static portBASE_TYPE StopStreamCommand(int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString)
{
	// Make sure we return something
	pcWriteBuffer[0] = '\0';
	snprintf((char *)pcWriteBuffer, xWriteBufferLen, "Stopping Streaming MEMS...\r\n");

	stopStreamMems();
	return pdFALSE;
}

uint8_t SaveToRO(void)
{
	BOS_Status result = BOS_OK;
	HAL_StatusTypeDef FlashStatus = HAL_OK;
	uint16_t add = 2, temp = 0;
	uint8_t snipBuffer[sizeof(snippet_t)+1] = {0};

	HAL_FLASH_Unlock();

	/* Erase RO area */
	FLASH_PageErase(RO_START_ADDRESS);
	FlashStatus = FLASH_WaitForLastOperation((uint32_t)HAL_FLASH_TIMEOUT_VALUE);
	if(FlashStatus != HAL_OK) {
		return pFlash.ErrorCode;
	} else {
		/* Operation is completed, disable the PER Bit */
		CLEAR_BIT(FLASH->CR, FLASH_CR_PER);
	}

	/* Save number of modules and myID */
	if (myID)
	{
		temp = (uint16_t) (N<<8) + myID;
		HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, RO_START_ADDRESS, temp);
		FlashStatus = FLASH_WaitForLastOperation((uint32_t)HAL_FLASH_TIMEOUT_VALUE);
		if (FlashStatus != HAL_OK) {
			return pFlash.ErrorCode;
		} else {
			/* If the program operation is completed, disable the PG Bit */
			CLEAR_BIT(FLASH->CR, FLASH_CR_PG);
		}

	/* Save topology */
		for(uint8_t i=1 ; i<=N ; i++)
		{
			for(uint8_t j=0 ; j<=MaxNumOfPorts ; j++)
			{
				if (array[i-1][0]) {
					HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, RO_START_ADDRESS+add, array[i-1][j]);
					add += 2;
					FlashStatus = FLASH_WaitForLastOperation((uint32_t)HAL_FLASH_TIMEOUT_VALUE);
					if (FlashStatus != HAL_OK) {
						return pFlash.ErrorCode;
					} else {
						/* If the program operation is completed, disable the PG Bit */
						CLEAR_BIT(FLASH->CR, FLASH_CR_PG);
					}
				}
			}
		}
	}

	// Save Command Snippets
	int currentAdd = RO_MID_ADDRESS;
	for(uint8_t s=0 ; s<numOfRecordedSnippets ; s++)
	{
		if (snippets[s].cond.conditionType)
		{
			snipBuffer[0] = 0xFE;		// A marker to separate Snippets
			memcpy( (uint8_t *)&snipBuffer[1], (uint8_t *)&snippets[s], sizeof(snippet_t));
			// Copy the snippet struct buffer (20 x numOfRecordedSnippets). Note this is assuming sizeof(snippet_t) is even.
			for(uint8_t j=0 ; j<(sizeof(snippet_t)/2) ; j++)
			{
				HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, currentAdd, *(uint16_t *)&snipBuffer[j*2]);
				FlashStatus = FLASH_WaitForLastOperation((uint32_t)HAL_FLASH_TIMEOUT_VALUE);
				if (FlashStatus != HAL_OK) {
					return pFlash.ErrorCode;
				} else {
					/* If the program operation is completed, disable the PG Bit */
					CLEAR_BIT(FLASH->CR, FLASH_CR_PG);
					currentAdd += 2;
				}
			}
			// Copy the snippet commands buffer. Always an even number. Note the string termination char might be skipped
			for(uint8_t j=0 ; j<((strlen(snippets[s].cmd)+1)/2) ; j++)
			{
				HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, currentAdd, *(uint16_t *)(snippets[s].cmd+j*2));
				FlashStatus = FLASH_WaitForLastOperation((uint32_t)HAL_FLASH_TIMEOUT_VALUE);
				if (FlashStatus != HAL_OK) {
					return pFlash.ErrorCode;
				} else {
					/* If the program operation is completed, disable the PG Bit */
					CLEAR_BIT(FLASH->CR, FLASH_CR_PG);
					currentAdd += 2;
				}
			}
		}
	}

	HAL_FLASH_Lock();

	return result;
}

/* --- Clear array topology in SRAM and Flash RO ---
*/
uint8_t ClearROtopology(void)
{
	// Clear the array
	memset(array, 0, sizeof(array));
	N = 1; myID = 0;

	return SaveToRO();
}
/*-----------------------------------------------------------*/


/* --- Get the port for a given UART.
*/
uint8_t GetPort(UART_HandleTypeDef *huart)
{
	if (huart->Instance == USART4)
			return P1;
	else if (huart->Instance == USART2)
			return P2;
	else if (huart->Instance == USART6)
			return P3;
	else if (huart->Instance == USART3)
			return P4;
	else if (huart->Instance == USART1)
			return P5;
	else if (huart->Instance == USART5)
			return P6;

	return 0;
}

/*-----------------------------------------------------------*/
