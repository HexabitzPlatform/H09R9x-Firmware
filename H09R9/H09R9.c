/*
 BitzOS (BOS) V0.3.6 - Copyright (C) 2017-2024 Hexabitz
 All rights reserved

 File Name     : H09R9.c
 Description   : Source code for module H09R9.
 	 	 	 	 (Description_of_module)

(Description of Special module peripheral configuration):
>>
>>
>>

 */

/* Includes ------------------------------------------------------------------*/
#include "BOS.h"
#include "H09R9.h"
#include "H09R9_inputs.h"

/* Define UART variables */
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;
UART_HandleTypeDef huart4;
UART_HandleTypeDef huart5;
UART_HandleTypeDef huart6;

TaskHandle_t EXGTaskHandle = NULL;

extern I2C_HandleTypeDef hi2c2;

/* Private Variables *******************************************************/
int16_t TSenMax_Value, TSenMin_Value;
float Sample;
float TC = 0, RT = 0, k0comp = 0, k1comp = 0, k2comp = 0, k3comp = 0,
		k4comp = 0, k0Obj = 0, k1Obj = 0, k2Obj = 0, k3Obj = 0, k4Obj = 0;

/* Stream variables */
uint8_t port1, module1;
uint8_t port2 ,module2;
uint32_t Numofsamples2 ,timeout2;
uint8_t port3 ;
uint32_t Numofsamples3 ,timeout3;
uint8_t flag ;
uint8_t StreamingDataMode ;
static bool stopStream = false;

/* Global variables for sensor data used in ModuleParam */
float H09R9_temp = 0.0f;

/* Module Parameters */
ModuleParam_t ModuleParam[NUM_MODULE_PARAMS] ={
    { .ParamPtr = &H09R9_temp, .ParamFormat = FMT_FLOAT, .ParamName = "temperature" }
};

/* Local Typedef related to stream functions */
typedef void (*SampleMemsToPort)(uint8_t, uint8_t);
typedef void (*SampleMemsToString)(char *, size_t);
typedef void (*SampleMemsToBuffer)(float *buffer);

/* Private function prototypes *********************************************/
uint8_t ClearROtopology(void);
void Module_Peripheral_Init(void);
Module_Status Module_MessagingTask(uint16_t code,uint8_t port,uint8_t src,uint8_t dst,uint8_t shift);

/* Local function prototypes ***********************************************/
void GetSample(float *temp);
void SENSOR_COEFFICIENTS_Init(void);
void TemperatureTask(void *argument);
void ExportToPort(uint8_t port,uint8_t module);
float BytesToFloat(uchar b0, uchar b1, uchar b2, uchar b3);

/* Stream Functions */
void SampleTemperatureBuf(float *buffer);
void ExportToPortmes(uint8_t module,uint8_t port);
void SampleTemperatureToString(char *cstring, size_t maxLen);
static Module_Status StreamMemsToCLI(uint32_t Numofsamples, uint32_t timeout, SampleMemsToString function);
static Module_Status StreamMemsToBuf(float *buffer, uint32_t Numofsamples, uint32_t timeout, SampleMemsToBuffer function);
static Module_Status StreamMemsToTerminal(uint32_t Numofsamples, uint32_t timeout,uint8_t Port, SampleMemsToString function);
static Module_Status StreamMemsToPort(uint8_t module, uint8_t port , uint32_t Numofsamples, uint32_t timeout, SampleMemsToPort function);

/* Create CLI commands *****************************************************/
static portBASE_TYPE TSD305SampleCommand(int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString);
static portBASE_TYPE TSD305StreamcliCommand(int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString);
static portBASE_TYPE TSD305StreamportCommand(int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString);
static portBASE_TYPE TSD305SampleportportCommand(int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString);

/* CLI command structure ***************************************************/
/* CLI command structure : sample */
const CLI_Command_Definition_t TSD305SampleCommandDefinition = {
		(const int8_t*) "sample", /* The command string to type. */
		(const int8_t*) "sample:\r\nTake one sample measurement\r\n\r\n",
		TSD305SampleCommand, /* The function to run. */
0 /* No parameters are expected. */
};

/***************************************************************************/
/* CLI command structure : streamtocli */
const CLI_Command_Definition_t TSD305StreamcliCommandDefinition = {
		(const int8_t*) "streamtocli", /* The command string to type. */
		(const int8_t*) "streamtocli:\r\n Take several samples measurement\r\n\r\n",
		TSD305StreamcliCommand, /* The function to run. */
2 /* Multiple parameters are expected. */
};

/***************************************************************************/
/* CLI command structure : streamtoport */
const CLI_Command_Definition_t TSD305StreamportCommandDefinition = {
		(const int8_t*) "streamtoport", /* The command string to type. */
		(const int8_t*) "streamtoport:\r\n export several samples measurementr\n\r\n",
		TSD305StreamportCommand, /* The function to run. */
3 /* No parameters are expected. */
};

/***************************************************************************/
/* CLI command structure : sampletoport */
const CLI_Command_Definition_t TSD305SampletoportCommandDefinition = {
		(const int8_t*) "sampletoport", /* The command string to type. */
		(const int8_t*) "sampletoport:\r\n export one samples measurementr\r\n\r\n",
		TSD305SampleportportCommand, /* The function to run. */
1 /* one parameter is expected. */
};

/***************************************************************************/
/************************ Private function Definitions *********************/
/***************************************************************************/
/* @brief  System Clock Configuration
 *         This function configures the system clock as follows:
 *            - System Clock source            = PLL (HSE)
 *            - SYSCLK(Hz)                     = 64000000
 *            - HCLK(Hz)                       = 64000000
 *            - AHB Prescaler                  = 1
 *            - APB1 Prescaler                 = 1
 *            - HSE Frequency(Hz)              = 8000000
 *            - PLLM                           = 1
 *            - PLLN                           = 16
 *            - PLLP                           = 2
 *            - Flash Latency(WS)              = 2
 *            - Clock Source for UART1,UART2,UART3 = 16MHz (HSI)
 */
void SystemClock_Config(void){
	RCC_OscInitTypeDef RCC_OscInitStruct ={0};
	RCC_ClkInitTypeDef RCC_ClkInitStruct ={0};

	/** Configure the main internal regulator output voltage */
	HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

	/* Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI | RCC_OSCILLATORTYPE_HSE; // Enable both HSI and HSE oscillators
	RCC_OscInitStruct.HSEState = RCC_HSE_ON; // Enable HSE (External High-Speed Oscillator)
	RCC_OscInitStruct.HSIState = RCC_HSI_ON; // Enable HSI (Internal High-Speed Oscillator)
	RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV1; // No division on HSI
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT; // Default calibration value for HSI
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON; // Enable PLL
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE; // Set PLL source to HSE
	RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1; // Prescaler for PLL input
	RCC_OscInitStruct.PLL.PLLN =16; // Multiplication factor for PLL
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2; // PLLP division factor
	RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2; // PLLQ division factor
	RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2; // PLLR division factor
	HAL_RCC_OscConfig(&RCC_OscInitStruct);

	/** Initializes the CPU, AHB and APB buses clocks */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK; // Select PLL as the system clock source
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1; // AHB Prescaler set to 1
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1; // APB1 Prescaler set to 1

	HAL_RCC_ClockConfig(&RCC_ClkInitStruct,FLASH_LATENCY_2); // Configure system clocks with flash latency of 2 WS
}

/***************************************************************************/
/* enable stop mode regarding only UART1 , UART2 , and UART3 */
BOS_Status EnableStopModebyUARTx(uint8_t port){

	UART_WakeUpTypeDef WakeUpSelection;
	UART_HandleTypeDef *huart =GetUart(port);

	if((huart->Instance == USART1) || (huart->Instance == USART2) || (huart->Instance == USART3)){

		/* make sure that no UART transfer is on-going */
		while(__HAL_UART_GET_FLAG(huart, USART_ISR_BUSY) == SET);

		/* make sure that UART is ready to receive */
		while(__HAL_UART_GET_FLAG(huart, USART_ISR_REACK) == RESET);

		/* set the wake-up event:
		 * specify wake-up on start-bit detection */
		WakeUpSelection.WakeUpEvent = UART_WAKEUP_ON_STARTBIT;
		HAL_UARTEx_StopModeWakeUpSourceConfig(huart,WakeUpSelection);

		/* Enable the UART Wake UP from stop mode Interrupt */
		__HAL_UART_ENABLE_IT(huart,UART_IT_WUF);

		/* enable MCU wake-up by LPUART */
		HAL_UARTEx_EnableStopMode(huart);

		/* enter STOP mode */
		HAL_PWR_EnterSTOPMode(PWR_LOWPOWERREGULATOR_ON,PWR_STOPENTRY_WFI);
	}
	else
		return BOS_ERROR;

}

/***************************************************************************/
/* Enable standby mode regarding wake-up pins:
 * WKUP1: PA0  pin
 * WKUP4: PA2  pin
 * WKUP6: PB5  pin
 * WKUP2: PC13 pin
 * NRST pin
 *  */
BOS_Status EnableStandbyModebyWakeupPinx(WakeupPins_t wakeupPins){

	/* Clear the WUF FLAG */
	__HAL_PWR_CLEAR_FLAG(PWR_FLAG_WUF);

	/* Enable the WAKEUP PIN */
	switch(wakeupPins){

		case PA0_PIN:
			HAL_PWR_EnableWakeUpPin(PWR_WAKEUP_PIN1); /* PA0 */
			break;

		case PA2_PIN:
			HAL_PWR_EnableWakeUpPin(PWR_WAKEUP_PIN4); /* PA2 */
			break;

		case PB5_PIN:
			HAL_PWR_EnableWakeUpPin(PWR_WAKEUP_PIN6); /* PB5 */
			break;

		case PC13_PIN:
			HAL_PWR_EnableWakeUpPin(PWR_WAKEUP_PIN2); /* PC13 */
			break;

		case NRST_PIN:
			/* do no thing*/
			break;
	}

	/* Enable SRAM content retention in Standby mode */
	HAL_PWREx_EnableSRAMRetention();

	/* Finally enter the standby mode */
	HAL_PWR_EnterSTANDBYMode();

	return BOS_OK;
}

/***************************************************************************/
/* Disable standby mode regarding wake-up pins:
 * WKUP1: PA0  pin
 * WKUP4: PA2  pin
 * WKUP6: PB5  pin
 * WKUP2: PC13 pin
 * NRST pin
 *  */
BOS_Status DisableStandbyModeWakeupPinx(WakeupPins_t wakeupPins){

	/* The standby wake-up is same as a system RESET:
	 * The entire code runs from the beginning just as if it was a RESET.
	 * The only difference between a reset and a STANDBY wake-up is that, when the MCU wakes-up,
	 * The SBF status flag in the PWR power control/status register (PWR_CSR) is set */
	if(__HAL_PWR_GET_FLAG(PWR_FLAG_SB) != RESET){
		/* clear the flag */
		__HAL_PWR_CLEAR_FLAG(PWR_FLAG_SB);

		/* Disable  Wake-up Pinx */
		switch(wakeupPins){

			case PA0_PIN:
				HAL_PWR_DisableWakeUpPin(PWR_WAKEUP_PIN1); /* PA0 */
				break;

			case PA2_PIN:
				HAL_PWR_DisableWakeUpPin(PWR_WAKEUP_PIN4); /* PA2 */
				break;

			case PB5_PIN:
				HAL_PWR_DisableWakeUpPin(PWR_WAKEUP_PIN6); /* PB5 */
				break;

			case PC13_PIN:
				HAL_PWR_DisableWakeUpPin(PWR_WAKEUP_PIN2); /* PC13 */
				break;

			case NRST_PIN:
				/* do no thing*/
				break;
		}

		IND_blink(1000);

	}
	else
		return BOS_OK;

}

/***************************************************************************/
/* Save Command Topology in Flash RO */
uint8_t SaveTopologyToRO(void){

	HAL_StatusTypeDef flashStatus =HAL_OK;

	/* flashAdd is initialized with 8 because the first memory room in topology page
	 * is reserved for module's ID */
	uint16_t flashAdd =8;
	uint16_t temp =0;

	/* Unlock the FLASH control register access */
	HAL_FLASH_Unlock();

	/* Erase Topology page */
	FLASH_PageErase(FLASH_BANK_2,TOPOLOGY_PAGE_NUM);

	/* Wait for an Erase operation to complete */
	flashStatus =FLASH_WaitForLastOperation((uint32_t ) HAL_FLASH_TIMEOUT_VALUE);

	if(flashStatus != HAL_OK){
		/* return FLASH error code */
		return pFlash.ErrorCode;
	}

	else{
		/* Operation is completed, disable the PER Bit */
		CLEAR_BIT(FLASH->CR,FLASH_CR_PER);
	}

	/* Save module's ID and topology */
	if(myID){

		/* Save module's ID */
		temp =(uint16_t )(N << 8) + myID;

		/* Save module's ID in Flash memory */
		HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD,TOPOLOGY_START_ADDRESS,temp);

		/* Wait for a Write operation to complete */
		flashStatus =FLASH_WaitForLastOperation((uint32_t ) HAL_FLASH_TIMEOUT_VALUE);

		if(flashStatus != HAL_OK){
			/* return FLASH error code */
			return pFlash.ErrorCode;
		}

		else{
			/* If the program operation is completed, disable the PG Bit */
			CLEAR_BIT(FLASH->CR,FLASH_CR_PG);
		}

		/* Save topology */
		for(uint8_t row =1; row <= N; row++){
			for(uint8_t column =0; column <= MAX_NUM_OF_PORTS; column++){
				/* Check the module serial number
				 * Note: there isn't a module has serial number 0
				 */
				if(Array[row - 1][0]){
					/* Save each element in topology Array in Flash memory */
					HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD,TOPOLOGY_START_ADDRESS + flashAdd,Array[row - 1][column]);
					/* Wait for a Write operation to complete */
					flashStatus =FLASH_WaitForLastOperation((uint32_t ) HAL_FLASH_TIMEOUT_VALUE);
					if(flashStatus != HAL_OK){
						/* return FLASH error code */
						return pFlash.ErrorCode;
					}
					else{
						/* If the program operation is completed, disable the PG Bit */
						CLEAR_BIT(FLASH->CR,FLASH_CR_PG);
						/* update new flash memory address */
						flashAdd +=8;
					}
				}
			}
		}
	}
	/* Lock the FLASH control register access */
	HAL_FLASH_Lock();
}

/***************************************************************************/
/* Save Command Snippets in Flash RO */
uint8_t SaveSnippetsToRO(void){
	HAL_StatusTypeDef FlashStatus =HAL_OK;
	uint8_t snipBuffer[sizeof(Snippet_t) + 1] ={0};

	/* Unlock the FLASH control register access */
	HAL_FLASH_Unlock();
	/* Erase Snippets page */
	FLASH_PageErase(FLASH_BANK_2,SNIPPETS_PAGE_NUM);
	/* Wait for an Erase operation to complete */
	FlashStatus =FLASH_WaitForLastOperation((uint32_t ) HAL_FLASH_TIMEOUT_VALUE);

	if(FlashStatus != HAL_OK){
		/* return FLASH error code */
		return pFlash.ErrorCode;
	}
	else{
		/* Operation is completed, disable the PER Bit */
		CLEAR_BIT(FLASH->CR,FLASH_CR_PER);
	}

	/* Save Command Snippets */
	int currentAdd = SNIPPETS_START_ADDRESS;
	for(uint8_t index =0; index < NumOfRecordedSnippets; index++){
		/* Check if Snippet condition is true or false */
		if(Snippets[index].Condition.ConditionType){
			/* A marker to separate Snippets */
			snipBuffer[0] =0xFE;
			memcpy((uint32_t* )&snipBuffer[1],(uint8_t* )&Snippets[index],sizeof(Snippet_t));
			/* Copy the snippet struct buffer (20 x NumOfRecordedSnippets). Note this is assuming sizeof(Snippet_t) is even */
			for(uint8_t j =0; j < (sizeof(Snippet_t) / 4); j++){
				HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD,currentAdd,*(uint64_t* )&snipBuffer[j * 8]);
				FlashStatus =FLASH_WaitForLastOperation((uint32_t ) HAL_FLASH_TIMEOUT_VALUE);
				if(FlashStatus != HAL_OK){
					return pFlash.ErrorCode;
				}
				else{
					/* If the program operation is completed, disable the PG Bit */
					CLEAR_BIT(FLASH->CR,FLASH_CR_PG);
					currentAdd +=8;
				}
			}
			/* Copy the snippet commands buffer. Always an even number. Note the string termination char might be skipped */
			for(uint8_t j =0; j < ((strlen(Snippets[index].CMD) + 1) / 4); j++){
				HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD,currentAdd,*(uint64_t* )(Snippets[index].CMD + j * 4));
				FlashStatus =FLASH_WaitForLastOperation((uint32_t ) HAL_FLASH_TIMEOUT_VALUE);
				if(FlashStatus != HAL_OK){
					return pFlash.ErrorCode;
				}
				else{
					/* If the program operation is completed, disable the PG Bit */
					CLEAR_BIT(FLASH->CR,FLASH_CR_PG);
					currentAdd +=8;
				}
			}
		}
	}
	/* Lock the FLASH control register access */
	HAL_FLASH_Lock();
}

/***************************************************************************/
/* Clear Array topology in SRAM and Flash RO */
uint8_t ClearROtopology(void){
	/* Clear the Array */
	memset(Array,0,sizeof(Array));
	N =1;
	myID =0;
	
	return SaveTopologyToRO();
}

/***************************************************************************/
/* Trigger ST factory bootloader update for a remote module */
void RemoteBootloaderUpdate(uint8_t src,uint8_t dst,uint8_t inport,uint8_t outport){

	uint8_t myOutport =0, lastModule =0;
	int8_t *pcOutputString;

	/* 1. Get Route to destination module */
	myOutport =FindRoute(myID,dst);
	if(outport && dst == myID){ /* This is a 'via port' update and I'm the last module */
		myOutport =outport;
		lastModule =myID;
	}
	else if(outport == 0){ /* This is a remote update */
		if(NumberOfHops(dst)== 1)
		lastModule = myID;
		else
		lastModule = Route[NumberOfHops(dst)-1]; /* previous module = Route[Number of hops - 1] */
	}

	/* 2. If this is the source of the message, show status on the CLI */
	if(src == myID){
		/* Obtain the address of the output buffer.  Note there is no mutual
		 * exclusion on this buffer as it is assumed only one command console
		 * interface will be used at any one time. */
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

/***************************************************************************/
/* Setup a port for remote ST factory bootloader update:
 * Set baudrate to 57600
 * Enable even parity
 * Set datasize to 9 bits
 */
void SetupPortForRemoteBootloaderUpdate(uint8_t port){

	UART_HandleTypeDef *huart =GetUart(port);
	HAL_UART_DeInit(huart);
	huart->Init.Parity = UART_PARITY_EVEN;
	huart->Init.WordLength = UART_WORDLENGTH_9B;
	HAL_UART_Init(huart);

	/* The CLI port RXNE interrupt might be disabled so enable here again to be sure */
	__HAL_UART_ENABLE_IT(huart,UART_IT_RXNE);

}

/***************************************************************************/
/* H09R9 module initialization */
void Module_Peripheral_Init(void) {

	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();

	/* Array ports */
	MX_USART1_UART_Init();
	MX_USART2_UART_Init();
	MX_USART3_UART_Init();
	MX_USART4_UART_Init();
	MX_USART5_UART_Init();
	MX_USART6_UART_Init();

	MX_I2C2_Init();

	/* Circulating DMA Channels ON All Module */
	for (int i = 1; i <= NUM_OF_PORTS; i++) {
		if (GetUart(i) == &huart1) {
			dmaIndex[i - 1] = &(DMA1_Channel1->CNDTR);
		} else if (GetUart(i) == &huart2) {
			dmaIndex[i - 1] = &(DMA1_Channel2->CNDTR);
		} else if (GetUart(i) == &huart3) {
			dmaIndex[i - 1] = &(DMA1_Channel3->CNDTR);
		} else if (GetUart(i) == &huart4) {
			dmaIndex[i - 1] = &(DMA1_Channel4->CNDTR);
		} else if (GetUart(i) == &huart5) {
			dmaIndex[i - 1] = &(DMA1_Channel5->CNDTR);
		} else if (GetUart(i) == &huart6) {
			dmaIndex[i - 1] = &(DMA1_Channel6->CNDTR);
		}
	}

	/* Create module special task (if needed) */
	xTaskCreate(TemperatureTask, (const char*) "TemperatureTask", configMINIMAL_STACK_SIZE, NULL, osPriorityNormal - osPriorityIdle, &EXGTaskHandle);
}

/*-----------------------------------------------------------*/
/*H09R9 message processing task */
Module_Status Module_MessagingTask(uint16_t code, uint8_t port, uint8_t src, uint8_t dst, uint8_t shift) {
	Module_Status result = H09R9_OK;
	uint32_t Numofsamples;
	uint32_t timeout;

	switch (code) {

	case CODE_H09R9_SAMPLE_TEMP:
		ExportToPortmes(cMessage[port - 1][shift],
				cMessage[port - 1][1 + shift]);
		break;

	case CODE_H09R9_STREAM_TEMP:
		Numofsamples = ((uint32_t) cMessage[port - 1][2 + shift])
				+ ((uint32_t) cMessage[port - 1][3 + shift] << 8)
				+ ((uint32_t) cMessage[port - 1][4 + shift] << 16)
				+ ((uint32_t) cMessage[port - 1][5 + shift] << 24);
		timeout = ((uint32_t) cMessage[port - 1][6 + shift])
				+ ((uint32_t) cMessage[port - 1][7 + shift] << 8)
				+ ((uint32_t) cMessage[port - 1][8 + shift] << 16)
				+ ((uint32_t) cMessage[port - 1][9 + shift] << 24);
		StreamMemsToPort(cMessage[port - 1][shift], cMessage[port - 1][shift + 1], Numofsamples, timeout, ExportToPort);
		break;

	default:
		result = H09R9_ERR_UNKNOWNMESSAGE;
		break;

	}

	return result;
}

/***************************************************************************/
/* Get the port for a given UART */
uint8_t GetPort(UART_HandleTypeDef *huart) {

	if (huart->Instance == USART4)
		return P1;
	else if (huart->Instance == USART2)
		return P2;
	else if (huart->Instance == USART6)
		return P3;
	else if (huart->Instance == USART1)
		return P4;
	else if (huart->Instance == USART5)
		return P5;
	else if (huart->Instance == USART3)
		return P6;

	return 0;
}
/***************************************************************************/
/* Samples a module parameter value based on parameter index.
 * paramIndex: Index of the parameter (1-based index).
 * value: Pointer to store the sampled float value.
 */
Module_Status GetModuleParameter(uint8_t paramIndex, float *value) {
    Module_Status status = H09R9_OK;

    switch (paramIndex) {
        /* Sample temperature in Celsius */
        case 1:
            status = SampleTemperature(value);
            break;

        /* Invalid parameter index */
        default:
            status = H09R9_ERR_WRONGPARAMS;
            break;
    }

    return status;
}

/***************************************************************************/
/* Register this module CLI Commands */
void RegisterModuleCLICommands(void){
	FreeRTOS_CLIRegisterCommand(&TSD305SampleCommandDefinition);
	FreeRTOS_CLIRegisterCommand(&TSD305StreamcliCommandDefinition);
	FreeRTOS_CLIRegisterCommand(&TSD305StreamportCommandDefinition);
	FreeRTOS_CLIRegisterCommand(&TSD305SampletoportCommandDefinition);
}

/***************************************************************************/
/* Module special task function (if needed) */
void TemperatureTask(void *argument) {

	SENSOR_COEFFICIENTS_Init();

	while (0 == TSenMax_Value) {
		IND_ON();
		HAL_Delay(200);
		IND_OFF();
		HAL_Delay(200);
	}

	/* Infinite loop */
	for (;;) {
		/*  */

		switch (StreamingDataMode) {
		case SAMPLE_TEM:
			GetSample(&Sample);
			break;

		case SAMPLE_TO_PORT:
			ExportToPort(port1, module1);
			break;

		case STREAM_TO_PORT:
			StreamMemsToPort(port2, module2, Numofsamples2, timeout2,
					ExportToPort);
			break;

		case STREAM_TO_Terminal:
			StreamMemsToTerminal(Numofsamples3, timeout3, port3,
					SampleTemperatureToString);
			break;

		default:
			osDelay(10);
			break;
		}

		taskYIELD();
	}
}


/***************************************************************************/
/****************************** Local Functions ****************************/
/***************************************************************************/
static Module_Status PollingSleepCLISafe(uint32_t period, long Numofsamples) {
	const unsigned DELTA_SLEEP_MS = 100; // milliseconds
	long numDeltaDelay = period / DELTA_SLEEP_MS;
	unsigned lastDelayMS = period % DELTA_SLEEP_MS;

	while (numDeltaDelay-- > 0) {
		vTaskDelay(pdMS_TO_TICKS(DELTA_SLEEP_MS));

		// Look for ENTER key to stop the stream
		for (uint8_t chr = 0; chr < MSG_RX_BUF_SIZE; chr++) {
			if (UARTRxBuf[pcPort - 1][chr] == '\r' && Numofsamples > 0) {
				UARTRxBuf[pcPort - 1][chr] = 0;
				flag = 1;
				return H09R9_ERR_TERMINATED;
			}
		}

		if (stopStream)
			return H09R9_ERR_TERMINATED;
	}

	vTaskDelay(pdMS_TO_TICKS(lastDelayMS));
	return H09R9_OK;
}

/***************************************************************************/
static Module_Status StreamMemsToBuf(float *buffer, uint32_t Numofsamples, uint32_t timeout, SampleMemsToBuffer function) {
	Module_Status status = H09R9_OK;

	uint8_t cont;

	uint32_t period = timeout / Numofsamples;

	if (period < MIN_MEMS_PERIOD_MS)
		return H09R9_ERR_WRONGPARAMS;

	// TODO: Check if CLI is enable or not

	if (period > timeout)
		timeout = period;

	long numTimes = timeout / period;
	stopStream = false;

	while ((numTimes-- > 0) || (timeout >= MAX_MEMS_TIMEOUT_MS)) {
		float sample;
		function(&sample);
		buffer[cont] = sample;
		cont++;

		vTaskDelay(pdMS_TO_TICKS(period));
		if (stopStream) {
			status = H09R9_ERR_TERMINATED;
			break;
		}
	}
	return status;
}

/***************************************************************************/
static Module_Status StreamMemsToPort(uint8_t module, uint8_t port , uint32_t Numofsamples, uint32_t timeout, SampleMemsToPort function){
	Module_Status status = H09R9_OK;
	uint32_t period = timeout / Numofsamples;
	if (period < MIN_MEMS_PERIOD_MS)
		return H09R9_ERR_WRONGPARAMS;
	if (port == 0)
		return H09R9_ERR_WRONGPARAMS;
	if (port == pcPort) // Check if CLI is not enabled at that port!
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

/***************************************************************************/
static Module_Status StreamMemsToCLI(uint32_t Numofsamples, uint32_t timeout, SampleMemsToString function) {
	Module_Status status = H09R9_OK;
	int8_t *pcOutputString = NULL;
	uint32_t period = timeout / Numofsamples;
	if (period < MIN_MEMS_PERIOD_MS)
		return H09R9_ERR_WRONGPARAMS;

	// TODO: Check if CLI is enable or not
	for (uint8_t chr = 0; chr < MSG_RX_BUF_SIZE; chr++) {
		if (UARTRxBuf[pcPort - 1][chr] == '\r') {
			UARTRxBuf[pcPort - 1][chr] = 0;
		}
	}
	if (1 == flag) {
		flag = 0;
		static char *pcOKMessage = (int8_t*) "Stop stream !\n\r";
		writePxITMutex(pcPort, pcOKMessage, strlen(pcOKMessage), 10);
		return status;
	}
	if (period > timeout)
		timeout = period;

	long numTimes = timeout / period;
	stopStream = false;

	while ((numTimes-- > 0) || (timeout >= MAX_MEMS_TIMEOUT_MS)) {
		pcOutputString = FreeRTOS_CLIGetOutputBuffer();
		function((char*) pcOutputString, 100);

		writePxMutex(pcPort, (char*) pcOutputString,
				strlen((char*) pcOutputString), cmd500ms, HAL_MAX_DELAY);
		if (PollingSleepCLISafe(period, Numofsamples) != H09R9_OK)
			break;
	}

	memset((char*) pcOutputString, 0, configCOMMAND_INT_MAX_OUTPUT_SIZE);
	sprintf((char*) pcOutputString, "\r\n");

	return status;
}

/***************************************************************************/
static Module_Status StreamMemsToTerminal(uint32_t Numofsamples, uint32_t timeout, uint8_t Port, SampleMemsToString function) {
	Module_Status status = H09R9_OK;
	int8_t *pcOutputString = NULL;
	uint32_t period = timeout / Numofsamples;
	if (period < MIN_MEMS_PERIOD_MS)
		return H09R9_ERR_WRONGPARAMS;

	// TODO: Check if CLI is enable or not

	if (period > timeout)
		timeout = period;

	long numTimes = timeout / period;
	stopStream = false;

	while ((numTimes-- > 0) || (timeout >= MAX_MEMS_TIMEOUT_MS)) {
		pcOutputString = FreeRTOS_CLIGetOutputBuffer();
		function((char*) pcOutputString, 100);

		writePxMutex(Port, (char*) pcOutputString,
				strlen((char*) pcOutputString), cmd500ms, HAL_MAX_DELAY);
		if (PollingSleepCLISafe(period, Numofsamples) != H09R9_OK)
			break;
	}

	memset((char*) pcOutputString, 0, configCOMMAND_INT_MAX_OUTPUT_SIZE);
	sprintf((char*) pcOutputString, "\r\n");
	module1 = DEFAULT;
	return status;
}

/***************************************************************************/
void SENSOR_COEFFICIENTS_Init(void) {

	uint8_t h_TSenMax, l_TSenMax, h_TSenMin, l_TSenMin;
	uint8_t h_TC1, l_TC1, h_TC2, l_TC2;
	uint8_t h_RT1, l_RT1, h_RT2, l_RT2;
	uint8_t h_add1, l_add1, h_add2, l_add2, h_add3, l_add3, h_add4, l_add4,
			h_add5, l_add5, h_add6, l_add6, h_add7, l_add7, h_add8, l_add8,
			h_add9, l_add9, h_add10, l_add10;
	uint8_t h_add11, l_add11, h_add12, l_add12, h_add13, l_add13, h_add14,
			l_add14, h_add15, l_add15, h_add16, l_add16, h_add17, l_add17,
			h_add18, l_add18, h_add19, l_add19, h_add20, l_add20;
	uint8_t buf2[3] = { 0x00 };
	uint8_t buf1[3] = { 0x00 };

	uint8_t SensorAddresses;

	HAL_I2C_IsDeviceReady(I2C_HANDLER, TSD305_ADDR, 3, 100);
	Delay_ms_no_rtos(TempDelay);
	/*----------sensor temperature range from TSenMin to TSenMax.---------------*/
	SensorAddresses = 0x1B;
	HAL_I2C_Master_Transmit(I2C_HANDLER, TSD305_ADDR, &SensorAddresses, 1, 100);
	Delay_ms_no_rtos(TempDelay);
	HAL_I2C_Master_Receive(I2C_HANDLER, TSD305_ADDR, buf1, 3, 100);
	TSenMax_Value = (buf1[1] << 8) + buf1[2];
	SensorAddresses = 0x1A;
	HAL_I2C_Master_Transmit(I2C_HANDLER, TSD305_ADDR, &SensorAddresses, 1, 100);
	Delay_ms_no_rtos(TempDelay);
	HAL_I2C_Master_Receive(I2C_HANDLER, TSD305_ADDR, buf2, 3, 100);
	TSenMin_Value = (buf2[1] << 8) + buf2[2];

	/*-------------------------Temperature Coefficient---------------------*/
	SensorAddresses = 0x1E;
	HAL_I2C_Master_Transmit(I2C_HANDLER, TSD305_ADDR, &SensorAddresses, 1, 100);
	Delay_ms_no_rtos(TempDelay);
	HAL_I2C_Master_Receive(I2C_HANDLER, TSD305_ADDR, buf1, 3, 100);

	h_TC1 = buf1[1];
	l_TC1 = buf1[2];

	SensorAddresses = 0x1F;
	HAL_I2C_Master_Transmit(I2C_HANDLER, TSD305_ADDR, &SensorAddresses, 1, 100);
	Delay_ms_no_rtos(TempDelay);
	HAL_I2C_Master_Receive(I2C_HANDLER, TSD305_ADDR, buf2, 3, 100);

	h_TC2 = buf2[1];
	l_TC2 = buf2[2];

	TC = BytesToFloat(h_TC1, l_TC1, h_TC2, l_TC2);

	/*-------------------------Reference Temperature------------------------------*/
	SensorAddresses = 0x20;
	HAL_I2C_Master_Transmit(I2C_HANDLER, TSD305_ADDR, &SensorAddresses, 1, 100);
	Delay_ms_no_rtos(TempDelay);
	HAL_I2C_Master_Receive(I2C_HANDLER, TSD305_ADDR, buf1, 3, 100);

	h_RT1 = buf1[1];
	l_RT1 = buf1[2];

	SensorAddresses = 0x21;
	HAL_I2C_Master_Transmit(I2C_HANDLER, TSD305_ADDR, &SensorAddresses, 1, 100);
	Delay_ms_no_rtos(TempDelay);
	HAL_I2C_Master_Receive(I2C_HANDLER, TSD305_ADDR, buf2, 3, 100);

	h_RT2 = buf2[1];
	l_RT2 = buf2[2];

	RT = BytesToFloat(h_RT1, l_RT1, h_RT2, l_RT2);

	/*---------------------Temperature Compensation-----------------------------------*/

	/*-----------------------1-------------------------------------------------------*/
//calculate k4comp
	SensorAddresses = 0x22;
	HAL_I2C_Master_Transmit(I2C_HANDLER, TSD305_ADDR, &SensorAddresses, 1, 100);
	Delay_ms_no_rtos(TempDelay);
	HAL_I2C_Master_Receive(I2C_HANDLER, TSD305_ADDR, buf1, 3, 100);

	h_add1 = buf1[1];
	l_add1 = buf1[2];

	SensorAddresses = 0x23;
	HAL_I2C_Master_Transmit(I2C_HANDLER, TSD305_ADDR, &SensorAddresses, 1, 100);
	Delay_ms_no_rtos(TempDelay);
	HAL_I2C_Master_Receive(I2C_HANDLER, TSD305_ADDR, buf2, 3, 100);

	h_add2 = buf2[1];
	l_add2 = buf2[2];

	k4comp = BytesToFloat(h_add1, l_add1, h_add2, l_add2);
	/*-----------------------2-------------------------------------------------------*/
//calculate k3comp
	SensorAddresses = 0x24;
	HAL_I2C_Master_Transmit(I2C_HANDLER, TSD305_ADDR, &SensorAddresses, 1, 100);
	Delay_ms_no_rtos(TempDelay);
	HAL_I2C_Master_Receive(I2C_HANDLER, TSD305_ADDR, buf1, 3, 100);

	h_add3 = buf1[1];
	l_add3 = buf1[2];

	SensorAddresses = 0x25;
	HAL_I2C_Master_Transmit(I2C_HANDLER, TSD305_ADDR, &SensorAddresses, 1, 100);
	Delay_ms_no_rtos(TempDelay);
	HAL_I2C_Master_Receive(I2C_HANDLER, TSD305_ADDR, buf2, 3, 100);

	h_add4 = buf2[1];
	l_add4 = buf2[2];

	k3comp = BytesToFloat(h_add3, l_add3, h_add4, l_add4);
	/*-----------------------3-------------------------------------------------------*/
//calculate k2comp
	SensorAddresses = 0x26;
	HAL_I2C_Master_Transmit(I2C_HANDLER, TSD305_ADDR, &SensorAddresses, 1, 100);
	Delay_ms_no_rtos(TempDelay);
	HAL_I2C_Master_Receive(I2C_HANDLER, TSD305_ADDR, buf1, 3, 100);

	h_add5 = buf1[1];
	l_add5 = buf1[2];

	SensorAddresses = 0x27;
	HAL_I2C_Master_Transmit(I2C_HANDLER, TSD305_ADDR, &SensorAddresses, 1, 100);
	Delay_ms_no_rtos(TempDelay);
	HAL_I2C_Master_Receive(I2C_HANDLER, TSD305_ADDR, buf2, 3, 100);

	h_add6 = buf2[1];
	l_add6 = buf2[2];

	k2comp = BytesToFloat(h_add5, l_add5, h_add6, l_add6);
	/*-----------------------4-------------------------------------------------------*/
//calculate k1comp
	SensorAddresses = 0x28;
	HAL_I2C_Master_Transmit(I2C_HANDLER, TSD305_ADDR, &SensorAddresses, 1, 100);
	Delay_ms_no_rtos(TempDelay);
	HAL_I2C_Master_Receive(I2C_HANDLER, TSD305_ADDR, buf1, 3, 100);

	h_add7 = buf1[1];
	l_add7 = buf1[2];

	SensorAddresses = 0x29;
	HAL_I2C_Master_Transmit(I2C_HANDLER, TSD305_ADDR, &SensorAddresses, 1, 100);
	Delay_ms_no_rtos(TempDelay);
	HAL_I2C_Master_Receive(I2C_HANDLER, TSD305_ADDR, buf2, 3, 100);

	h_add8 = buf2[1];
	l_add8 = buf2[2];

	k1comp = BytesToFloat(h_add7, l_add7, h_add8, l_add8);
	/*-----------------------5-------------------------------------------------------*/
//calculate k0comp
	SensorAddresses = 0x2A;
	HAL_I2C_Master_Transmit(I2C_HANDLER, TSD305_ADDR, &SensorAddresses, 1, 100);
	Delay_ms_no_rtos(TempDelay);
	HAL_I2C_Master_Receive(I2C_HANDLER, TSD305_ADDR, buf1, 3, 100);

	h_add9 = buf1[1];
	l_add9 = buf1[2];

	SensorAddresses = 0x2B;
	HAL_I2C_Master_Transmit(I2C_HANDLER, TSD305_ADDR, &SensorAddresses, 1, 100);
	Delay_ms_no_rtos(TempDelay);
	HAL_I2C_Master_Receive(I2C_HANDLER, TSD305_ADDR, buf2, 3, 100);

	h_add10 = buf2[1];
	l_add10 = buf2[2];

	k0comp = BytesToFloat(h_add9, l_add9, h_add10, l_add10);

	/*---------------------Object Temperature Determination---------------------------*/
	/*-----------------------1-------------------------------------------------------*/
//calculate k4obj
	SensorAddresses = 0x2E;
	HAL_I2C_Master_Transmit(I2C_HANDLER, TSD305_ADDR, &SensorAddresses, 1, 100);
	Delay_ms_no_rtos(TempDelay);
	HAL_I2C_Master_Receive(I2C_HANDLER, TSD305_ADDR, buf1, 3, 100);

	h_add11 = buf1[1];
	l_add11 = buf1[2];

	SensorAddresses = 0x2F;
	HAL_I2C_Master_Transmit(I2C_HANDLER, TSD305_ADDR, &SensorAddresses, 1, 100);
	Delay_ms_no_rtos(TempDelay);
	HAL_I2C_Master_Receive(I2C_HANDLER, TSD305_ADDR, buf2, 3, 100);

	h_add12 = buf2[1];
	l_add12 = buf2[2];

	k4Obj = BytesToFloat(h_add11, l_add11, h_add12, l_add12);
	/*-----------------------2-------------------------------------------------------*/
//calculate k3obj
	SensorAddresses = 0x30;
	HAL_I2C_Master_Transmit(I2C_HANDLER, TSD305_ADDR, &SensorAddresses, 1, 100);
	Delay_ms_no_rtos(TempDelay);
	HAL_I2C_Master_Receive(I2C_HANDLER, TSD305_ADDR, buf1, 3, 100);

	h_add13 = buf1[1];
	l_add13 = buf1[2];

	SensorAddresses = 0x31;
	HAL_I2C_Master_Transmit(I2C_HANDLER, TSD305_ADDR, &SensorAddresses, 1, 100);
	Delay_ms_no_rtos(TempDelay);
	HAL_I2C_Master_Receive(I2C_HANDLER, TSD305_ADDR, buf2, 3, 100);

	h_add14 = buf2[1];
	l_add14 = buf2[2];

	k3Obj = BytesToFloat(h_add13, l_add13, h_add14, l_add14);
	/*-----------------------3-------------------------------------------------------*/
//calculate k2obj
	SensorAddresses = 0x32;
	HAL_I2C_Master_Transmit(I2C_HANDLER, TSD305_ADDR, &SensorAddresses, 1, 100);
	Delay_ms_no_rtos(TempDelay);
	HAL_I2C_Master_Receive(I2C_HANDLER, TSD305_ADDR, buf1, 3, 100);

	h_add15 = buf1[1];
	l_add15 = buf1[2];

	SensorAddresses = 0x33;
	HAL_I2C_Master_Transmit(I2C_HANDLER, TSD305_ADDR, &SensorAddresses, 1, 100);
	Delay_ms_no_rtos(TempDelay);
	HAL_I2C_Master_Receive(I2C_HANDLER, TSD305_ADDR, buf2, 3, 100);

	h_add16 = buf2[1];
	l_add16 = buf2[2];

	k2Obj = BytesToFloat(h_add15, l_add15, h_add16, l_add16);
	/*-----------------------4-------------------------------------------------------*/
//calculate k1obj
	SensorAddresses = 0x34;
	HAL_I2C_Master_Transmit(I2C_HANDLER, TSD305_ADDR, &SensorAddresses, 1, 100);
	Delay_ms_no_rtos(TempDelay);
	HAL_I2C_Master_Receive(I2C_HANDLER, TSD305_ADDR, buf1, 3, 100);

	h_add17 = buf1[1];
	l_add17 = buf1[2];

	SensorAddresses = 0x35;
	HAL_I2C_Master_Transmit(I2C_HANDLER, TSD305_ADDR, &SensorAddresses, 1, 100);
	Delay_ms_no_rtos(TempDelay);
	HAL_I2C_Master_Receive(I2C_HANDLER, TSD305_ADDR, buf2, 3, 100);

	h_add18 = buf2[1];
	l_add18 = buf2[2];

	k1Obj = BytesToFloat(h_add17, l_add17, h_add18, l_add18);
	/*-----------------------5-------------------------------------------------------*/
//calculate k0obj
	SensorAddresses = 0x36;
	HAL_I2C_Master_Transmit(I2C_HANDLER, TSD305_ADDR, &SensorAddresses, 1, 100);
	Delay_ms_no_rtos(TempDelay);
	HAL_I2C_Master_Receive(I2C_HANDLER, TSD305_ADDR, buf1, 3, 100);

	h_add19 = buf1[1];
	l_add19 = buf1[2];

	SensorAddresses = 0x37;
	HAL_I2C_Master_Transmit(I2C_HANDLER, TSD305_ADDR, &SensorAddresses, 1, 100);
	Delay_ms_no_rtos(TempDelay);
	HAL_I2C_Master_Receive(I2C_HANDLER, TSD305_ADDR, buf2, 3, 100);

	h_add20 = buf2[1];
	l_add20 = buf2[2];

	k0Obj = BytesToFloat(h_add19, l_add19, h_add20, l_add20);
}

/***************************************************************************/
float BytesToFloat(uchar b0, uchar b1, uchar b2, uchar b3) {
	float output;

	*((uchar*) (&output) + 3) = b0;
	*((uchar*) (&output) + 2) = b1;
	*((uchar*) (&output) + 1) = b2;
	*((uchar*) (&output) + 0) = b3;

	return output;
}

/***************************************************************************/
void GetSample(float *temp) {

	float offest, OffsetTC, ADCComp, ADCCompTC;
	float TCF = 0, Tsen = 0;
	uint8_t ADC0, ADCsen3, ADCsen2, ADCsen1, ADCobj3, ADCobj2, ADCobj1;
	int32_t ADCsen, ADCobj;
	uint8_t buf4[7] = { 0x00 };

	uint8_t ADC_Add = 0xAF;

	HAL_I2C_IsDeviceReady(I2C_HANDLER, TSD305_ADDR, 1, 10);
	HAL_Delay(TempDelay);

	HAL_I2C_Master_Transmit(I2C_HANDLER, TSD305_ADDR, &ADC_Add, 1, 100);
	HAL_Delay(TempDelay);
	do {
		HAL_I2C_Master_Receive(I2C_HANDLER, TSD305_ADDR, buf4, 7, 100);

		ADC0 = buf4[0];
		ADCobj3 = buf4[1];
		ADCobj2 = buf4[2];
		ADCobj1 = buf4[3];
		ADCsen3 = buf4[4];
		ADCsen2 = buf4[5];
		ADCsen1 = buf4[6];

	} while (buf4[0] == 0x60);
	ADCsen = (ADCsen3 << 16) | (ADCsen2 << 8) | ADCsen1;
	ADCobj = (ADCobj3 << 16) | (ADCobj2 << 8) | ADCobj1;
	/*----------sensor temperature range from TSenMin to TSenMax.---------------*/

	float t = ((float) ADCsen) / 16777216;
	Tsen = t * (TSenMax_Value - TSenMin_Value) + TSenMin_Value;
	/*-------------------------TC Correction Factor-------------------------*/

	TCF = 1 + ((Tsen - RT) * TC);

	offest = k4comp * Tsen * Tsen * Tsen * Tsen + k3comp * Tsen * Tsen * Tsen
			+ k2comp * Tsen * Tsen + k1comp * Tsen + k0comp;
	OffsetTC = offest * TCF;

	ADCComp = OffsetTC + (ADCobj - 8388608) / 0.99;
	ADCCompTC = ADCComp / TCF;
	HAL_Delay(10);

	*temp = k4Obj * ADCCompTC * ADCCompTC * ADCCompTC * ADCCompTC
			+ k3Obj * ADCCompTC * ADCCompTC * ADCCompTC
			+ k2Obj * ADCCompTC * ADCCompTC + k1Obj * ADCCompTC + k0Obj;
}

/***************************************************************************/
void SampleTemperatureBuf(float *buffer) {
	GetSample(buffer);
}

/***************************************************************************/
void ExportToPortmes(uint8_t module, uint8_t port) {
	float buffer[1];
	static uint8_t temp[4];
	Module_Status status = H09R9_OK;

	status = SampleTemperature(buffer);

	if (module == myID) {

		temp[0] = (uint8_t) ((*(uint32_t*) &buffer[0]) >> 0);
		temp[1] = (uint8_t) ((*(uint32_t*) &buffer[0]) >> 8);
		temp[2] = (uint8_t) ((*(uint32_t*) &buffer[0]) >> 16);
		temp[3] = (uint8_t) ((*(uint32_t*) &buffer[0]) >> 24);

		writePxITMutex(port, (char*) &temp[0], 4 * sizeof(uint8_t), 10);
	} else {
		if (H09R9_OK == status)
			MessageParams[1] = BOS_OK;
		else
			MessageParams[1] = BOS_ERROR;

		MessageParams[0] = FMT_FLOAT;
		MessageParams[2] = 1;
		MessageParams[3] = (uint8_t) ((*(uint32_t*) &buffer[0]) >> 0);
		MessageParams[4] = (uint8_t) ((*(uint32_t*) &buffer[0]) >> 8);
		MessageParams[5] = (uint8_t) ((*(uint32_t*) &buffer[0]) >> 16);
		MessageParams[6] = (uint8_t) ((*(uint32_t*) &buffer[0]) >> 24);

		SendMessageToModule(module, CODE_READ_RESPONSE, sizeof(float) + 3);
	}
}

/***************************************************************************/
void ExportToPort(uint8_t port, uint8_t module) {
	float buffer[1];
	static uint8_t temp[4];
	Module_Status status = H09R9_OK;

	status = SampleTemperature(buffer);

	if (module == myID) {

		temp[0] = (uint8_t) ((*(uint32_t*) &buffer[0]) >> 0);
		temp[1] = (uint8_t) ((*(uint32_t*) &buffer[0]) >> 8);
		temp[2] = (uint8_t) ((*(uint32_t*) &buffer[0]) >> 16);
		temp[3] = (uint8_t) ((*(uint32_t*) &buffer[0]) >> 24);

		writePxITMutex(port, (char*) &temp[0], 4 * sizeof(uint8_t), 10);
	} else {
		if (H09R9_OK == status)
			MessageParams[1] = BOS_OK;
		else
			MessageParams[1] = BOS_ERROR;

		MessageParams[0] = FMT_FLOAT;
		MessageParams[2] = (uint8_t) ((*(uint32_t*) &buffer[0]) >> 0);
		MessageParams[3] = (uint8_t) ((*(uint32_t*) &buffer[0]) >> 8);
		MessageParams[4] = (uint8_t) ((*(uint32_t*) &buffer[0]) >> 16);
		MessageParams[5] = (uint8_t) ((*(uint32_t*) &buffer[0]) >> 24);

		SendMessageToModule(module, CODE_READ_RESPONSE, sizeof(float) + 2);
	}
	module1 = DEFAULT;
}

/***************************************************************************/
void SampleTemperatureToString(char *cstring, size_t maxLen)
{
	float temprature = 0;
	float temp;

	GetSample(&temprature);
	temp=temprature;
	snprintf(cstring, maxLen, "Temperature: %.2f\r\n", temprature);
}

/***************************************************************************/
void stopStreamMems(void)
{
	stopStream = true;
}

/***************************************************************************/
/***************************** General Functions ***************************/
/***************************************************************************/
Module_Status SampleTemperature(float *temp) {
	Module_Status status = H09R9_OK;
	StreamingDataMode = SAMPLE_TEM;
	*temp = Sample;
	return status;
}

/***************************************************************************/
Module_Status SampleTemperatureToPort(uint8_t module, uint8_t port) {
	Module_Status status = H09R9_OK;
	StreamingDataMode = SAMPLE_TO_PORT;
	port1 = port;
	module1 = module;
	return status;

}

/***************************************************************************/
Module_Status StreamTemperatureToBuffer(float *buffer, uint32_t Numofsamples, uint32_t timeout)
{
	return StreamMemsToBuf(buffer, Numofsamples, timeout, SampleTemperatureBuf);
}

/***************************************************************************/
Module_Status StreamTemperatureToPort( uint8_t module,uint8_t port, uint32_t Numofsamples, uint32_t timeout)
{
	Module_Status status = H09R9_OK;
	StreamingDataMode=STREAM_TO_PORT;
	port2 = port ;
	module2 =module;
	Numofsamples2=Numofsamples;
	timeout2=timeout;
	return status;

}

/***************************************************************************/
Module_Status StreamTemperatureToTerminal(uint8_t port,uint32_t Numofsamples, uint32_t timeout)
{
	Module_Status status = H09R9_OK;
	StreamingDataMode=STREAM_TO_Terminal;
	port3 = port ;
	Numofsamples3=Numofsamples;
	timeout3=timeout;
	return status;

}

/***************************************************************************/
Module_Status StreamTemperatureToCLI(uint32_t period, uint32_t timeout)
{

	return StreamMemsToCLI(period, timeout, SampleTemperatureToString);
}

/***************************************************************************/
/********************************* Commands ********************************/
/***************************************************************************/
static portBASE_TYPE SampleSensorCommand(int8_t *pcWriteBuffer,
		size_t xWriteBufferLen, const int8_t *pcCommandString) {
	// Make sure we return something
	*pcWriteBuffer = '\0';

	do {
		SampleTemperatureToString((char*) pcWriteBuffer, xWriteBufferLen);

		return pdFALSE;
	} while (0);

	snprintf((char*) pcWriteBuffer, xWriteBufferLen,
			"Error reading Sensor\r\n");

	return pdFALSE;
}

/***************************************************************************/
// Port Mode => false and CLI Mode => true
static bool StreamCommandParser(const int8_t *pcCommandString, bool *pPortOrCLI,
		uint32_t *pPeriod, uint32_t *pTimeout, uint8_t *pPort, uint8_t *pModule) {
	const char *pPeriodMSStr = NULL;
	const char *pTimeoutMSStr = NULL;

	portBASE_TYPE periodStrLen = 0;
	portBASE_TYPE timeoutStrLen = 0;

	const char *pPortStr = NULL;
	const char *pModStr = NULL;

	portBASE_TYPE portStrLen = 0;
	portBASE_TYPE modStrLen = 0;

	pPeriodMSStr = (const char*) FreeRTOS_CLIGetParameter(pcCommandString, 1,
			&periodStrLen);
	pTimeoutMSStr = (const char*) FreeRTOS_CLIGetParameter(pcCommandString, 2,
			&timeoutStrLen);

	// At least 3 Parameters are required!
	if ((pPeriodMSStr == NULL) || (pTimeoutMSStr == NULL))
		return false;

	// TODO: Check if Period and Timeout are integers or not!
	*pPeriod = atoi(pPeriodMSStr);
	*pTimeout = atoi(pTimeoutMSStr);
	*pPortOrCLI = true;

	pPortStr = (const char*) FreeRTOS_CLIGetParameter(pcCommandString, 3, &portStrLen);
	pModStr = (const char*) FreeRTOS_CLIGetParameter(pcCommandString, 4, &modStrLen);

	if ((pModStr == NULL) && (pPortStr == NULL))
		return true;
	if ((pModStr == NULL) || (pPortStr == NULL))// If user has provided 4 Arguments.
		return false;

	*pPort = atoi(pPortStr);
	*pModule = atoi(pModStr);
	*pPortOrCLI = false;

	return true;
}

/***************************************************************************/
static portBASE_TYPE StreamSensorCommand(int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString) {

	uint32_t period = 0;
	uint32_t timeout = 0;
	uint8_t port = 0;
	uint8_t module = 0;

	bool portOrCLI = true; // Port Mode => false and CLI Mode => true

	const char *pSensName = NULL;
	portBASE_TYPE sensNameLen = 0;

	// Make sure we return something
	*pcWriteBuffer = '\0';

	if (!StreamCommandParser(pcCommandString, &portOrCLI, &period, &timeout,
			&port, &module)) {
		snprintf((char*) pcWriteBuffer, xWriteBufferLen,
				"Invalid Arguments\r\n");
		return pdFALSE;
	}

	do {
		if (portOrCLI)
			StreamTemperatureToCLI(period, timeout);
		else
			StreamTemperatureToPort(port, module, period, timeout);

		snprintf((char*) pcWriteBuffer, xWriteBufferLen, "\r\n");
		return pdFALSE;
	} while (0);

	snprintf((char*) pcWriteBuffer, xWriteBufferLen,
			"Error reading Sensor\r\n");

	return pdFALSE;
}

/***************************************************************************/
static portBASE_TYPE StopStreamCommand(int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString)
{
	// Make sure we return something
	pcWriteBuffer[0] = '\0';
	snprintf((char *)pcWriteBuffer, xWriteBufferLen, "Stopping Streaming MEMS...\r\n");

	stopStreamMems();
	return pdFALSE;
}

/***************************************************************************/
static portBASE_TYPE TSD305SampleCommand(int8_t *pcWriteBuffer,
		size_t xWriteBufferLen, const int8_t *pcCommandString) {
	Module_Status status = H09R9_OK;

	StreamMemsToCLI(1, 100, SampleTemperatureToString);
//StreamTemperatureToCLI(1, 100);
	return pdFALSE;
}

/***************************************************************************/
static portBASE_TYPE TSD305StreamcliCommand(int8_t *pcWriteBuffer,
		size_t xWriteBufferLen, const int8_t *pcCommandString) {
	Module_Status status = H09R9_OK;

	uint32_t Numofsamples, pTimeout;
	static int8_t *pcParameterString1, *pcParameterString2;
	portBASE_TYPE xParameterStringLength1 = 0, xParameterStringLength2 = 0;

	(void) xWriteBufferLen;

	pcParameterString1 = (int8_t*) FreeRTOS_CLIGetParameter(pcCommandString, 1,
			&xParameterStringLength1);
	pcParameterString2 = (int8_t*) FreeRTOS_CLIGetParameter(pcCommandString, 2,
			&xParameterStringLength2);

	Numofsamples = atoi(pcParameterString1);
	pTimeout = atoi(pcParameterString2);
//	StreamTemperatureToCLI(Numofsamples, pTimeout);
	StreamMemsToCLI(Numofsamples, pTimeout, SampleTemperatureToString);
	/* There is no more data to return after this single string, so return pdFALSE. */
	return pdFALSE;
}

/***************************************************************************/
static portBASE_TYPE TSD305StreamportCommand(int8_t *pcWriteBuffer,
		size_t xWriteBufferLen, const int8_t *pcCommandString) {
	Module_Status status = H09R9_OK;

	uint8_t Port;
	uint32_t Numofsamples, pTimeout;
	static int8_t *pcParameterString1, *pcParameterString2, *pcParameterString3;
	portBASE_TYPE xParameterStringLength1 = 0, xParameterStringLength2 = 0,
			xParameterStringLength3 = 0;

	(void) xWriteBufferLen;

	pcParameterString1 = (int8_t*) FreeRTOS_CLIGetParameter(pcCommandString, 1,
			&xParameterStringLength1);
	pcParameterString2 = (int8_t*) FreeRTOS_CLIGetParameter(pcCommandString, 2,
			&xParameterStringLength2);
	pcParameterString3 = (int8_t*) FreeRTOS_CLIGetParameter(pcCommandString, 3,
			&xParameterStringLength3);
	Port = atoi(pcParameterString1);
	Numofsamples = atoi(pcParameterString2);
	pTimeout = atoi(pcParameterString3);
	StreamMemsToPort(Port, 0, Numofsamples, pTimeout,ExportToPort);

	/* There is no more data to return after this single string, so return pdFALSE. */
	return pdFALSE;
}

/***************************************************************************/
static portBASE_TYPE TSD305SampleportportCommand(int8_t *pcWriteBuffer,
		size_t xWriteBufferLen, const int8_t *pcCommandString) {
	Module_Status status = H09R9_OK;
	uint8_t Port;
	static int8_t *pcParameterString1;
	portBASE_TYPE xParameterStringLength1 = 0;

	(void) xWriteBufferLen;

	pcParameterString1 = (int8_t*) FreeRTOS_CLIGetParameter(pcCommandString, 1,
			&xParameterStringLength1);

	Port = atoi(pcParameterString1);

	ExportToPort(Port, 0);

	/* There is no more data to return after this single string, so return pdFALSE. */
	return pdFALSE;
}

/***************************************************************************/
/***************** (C) COPYRIGHT HEXABITZ ***** END OF FILE ****************/
