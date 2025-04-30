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

TimerHandle_t xTimerStream = NULL;
TaskHandle_t EXGTaskHandle = NULL;

extern I2C_HandleTypeDef hi2c2;

/* Private Variables *******************************************************/
/* Sensor calibration and temperature sampling variables */
int16_t TSenMax_Value;           /* Maximum sensor temperature value */
int16_t TSenMin_Value;           /* Minimum sensor temperature value */
float Sample;                    /* Sampled temperature value */
float TC = 0;                    /* Temperature coefficient */
float RT = 0;                    /* Reference temperature */
float k0comp = 0;                /* Temperature compensation coefficient (constant term) */
float k1comp = 0;                /* Temperature compensation coefficient (linear term) */
float k2comp = 0;                /* Temperature compensation coefficient (quadratic term) */
float k3comp = 0;                /* Temperature compensation coefficient (cubic term) */
float k4comp = 0;                /* Temperature compensation coefficient (quartic term) */
float k0Obj = 0;                 /* Object temperature coefficient (constant term) */
float k1Obj = 0;                 /* Object temperature coefficient (linear term) */
float k2Obj = 0;                 /* Object temperature coefficient (quadratic term) */
float k3Obj = 0;                 /* Object temperature coefficient (cubic term) */
float k4Obj = 0;                 /* Object temperature coefficient (quartic term) */
uint8_t GetTemp;                 /* Temporary variable for temperature sampling state */

/* Private Variables *******************************************************/
/* Streaming variables */
static bool stopStream = false;         /* Flag to indicate whether to stop streaming process */
uint8_t PortModule = 0u;                /* Module ID for the destination port */
uint8_t PortNumber = 0u;                /* Physical port number used for streaming */
uint8_t StreamMode = 0u;                /* Current active streaming mode (to port, terminal, etc.) */
uint8_t TerminalPort = 0u;              /* Port number used to output data to a terminal */
uint8_t StopeCliStreamFlag = 0u;        /* Flag to request stopping a CLI stream operation */
uint32_t SampleCount = 0u;              /* Counter to track the number of samples streamed */
uint32_t PortNumOfSamples = 0u;         /* Total number of samples to be sent through the port */
uint32_t TerminalNumOfSamples = 0u;     /* Total number of samples to be streamed to the terminal */

/* Global variables for sensor data used in ModuleParam */
float H09R9_temp = 0.0f;

/* Module Parameters */
ModuleParam_t ModuleParam[NUM_MODULE_PARAMS] ={
    { .ParamPtr = &H09R9_temp, .ParamFormat = FMT_FLOAT, .ParamName = "temperature" }
};

/* Local Typedef related to stream functions */
typedef void (*SampleToString)(char*,size_t);
typedef void (*SampleToBuffer)(float *buffer);
/* Local function prototypes ***********************************************/
/* Stream Functions */
void StreamTimeCallback(TimerHandle_t xTimerStream);
void SampleTemperatureBuf(float *buffer);
void SampleTemperatureToString(char *cstring, size_t maxLen);

Module_Status SampleToTerminal(uint8_t dstPort);
Module_Status SampleToPort(uint8_t dstModule, uint8_t dstPort);
static Module_Status PollingSleepCLISafe(uint32_t period, long Numofsamples);
static Module_Status StreamToCLI(uint32_t Numofsamples, uint32_t timeout, SampleToString function);
static Module_Status StreamToBuf(float *buffer, uint32_t Numofsamples, uint32_t timeout, SampleToBuffer function);


/* Private function prototypes *********************************************/
uint8_t ClearROtopology(void);
void Module_Peripheral_Init(void);
Module_Status Module_MessagingTask(uint16_t code,uint8_t port,uint8_t src,uint8_t dst,uint8_t shift);

/* Local function prototypes ***********************************************/
void GetSample(float *temp);
void SENSOR_COEFFICIENTS_Init(void);
void TemperatureTask(void *argument);
float BytesToFloat(uchar b0, uchar b1, uchar b2, uchar b3);

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


	/* Create a timeout software timer StreamSamplsToPort() API */
	xTimerStream =xTimerCreate("StreamTimer",pdMS_TO_TICKS(1000),pdTRUE,(void* )1,StreamTimeCallback);

}

/*-----------------------------------------------------------*/
/*H09R9 message processing task */
Module_Status Module_MessagingTask(uint16_t code, uint8_t port, uint8_t src, uint8_t dst, uint8_t shift) {
	Module_Status result = H09R9_OK;
	uint32_t Numofsamples;
	uint32_t timeout;

	switch (code) {

	case CODE_H09R9_SAMPLE_TEMP:
		SampleToPort(cMessage[port - 1][shift],
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
		StreamtoPort(cMessage[port - 1][shift], cMessage[port - 1][shift + 1], Numofsamples, timeout);
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

		if (SAMPLE_TEM == GetTemp)
			GetSample(&Sample);

		taskYIELD();
	}
}


/***************************************************************************/
/****************************** Local Functions ****************************/
/***************************************************************************/
/* Streams temperature sensor data to a buffer.
 * buffer: Pointer to the buffer where data will be stored.
 * Numofsamples: Number of samples to take.
 * timeout: Timeout period for the operation.
 * function: Function pointer to the sampling function (e.g., SampleTemperatureBuf).
 */
static Module_Status StreamToBuf(float *buffer, uint32_t Numofsamples, uint32_t timeout, SampleToBuffer function) {
    Module_Status status = H09R9_OK;
    uint16_t StreamIndex = 0;
    uint32_t period = timeout / Numofsamples;

    /* Check if the calculated period is valid */
    if (period < MIN_PERIOD_MS)
        return H09R9_ERR_WRONGPARAMS;

    stopStream = false;

    /* Stream data to buffer */
    while ((Numofsamples-- > 0) || (timeout >= MAX_TIMEOUT_MS)) {
        float sample;
        function(&sample);
        buffer[StreamIndex] = sample;
        StreamIndex++;

        /* Delay for the specified period */
        vTaskDelay(pdMS_TO_TICKS(period));

        /* Check if streaming should be stopped */
        if (stopStream) {
            status = H09R9_ERR_TERMINATED;
            break;
        }
    }

    return status;
}

/***************************************************************************/
/* Samples temperature data into a buffer.
 * buffer: Pointer to the buffer where temperature data will be stored.
 */
void SampleTemperatureBuf(float *buffer) {
    float temperature;
    SampleTemperature(&temperature);
    *buffer = temperature;
}
/***************************************************************************/
/* Timer callback for streaming temperature data.
 * xTimerStream: Handle to the stream timer.
 */
void StreamTimeCallback(TimerHandle_t xTimerStream) {
    if (StreamMode == STREAM_MODE_TO_PORT) {
        if (PortNumOfSamples > 0) {
            SampleToPort(PortModule, PortNumber);
            PortNumOfSamples--;
        } else {
            xTimerStop(xTimerStream, 100);
        }
    } else if (StreamMode == STREAM_MODE_TO_TERMINAL) {
        if (TerminalNumOfSamples > 0) {
            SampleToTerminal(TerminalPort);
            TerminalNumOfSamples--;
        } else {
            xTimerStop(xTimerStream, 100);
        }
    }
}



/***************************************************************************/
/* Streams a single temperature sensor data sample to the terminal.
 * dstPort: Port number to stream data to.
 */
Module_Status SampleToTerminal(uint8_t dstPort) {
    Module_Status Status = H09R9_OK; /* Initialize operation status as success */
    int8_t *PcOutputString = NULL; /* Pointer to CLI output buffer */
    uint32_t Period = 0u; /* Calculated period for the operation */
    char CString[100] = {0}; /* Buffer for formatted output string */
    float temperature = 0.0f; /* Variable for temperature data */

    /* Get the CLI output buffer for writing */
    PcOutputString = FreeRTOS_CLIGetOutputBuffer();
    /* Sample temperature data in Celsius */
    if (SampleTemperature(&temperature) != H09R9_OK) {
        return H09R9_ERROR; /* Return error if sampling fails */
    }
    /* Format temperature data into a string */
    snprintf(CString, 50, "Temp(Celsius) | %.2f\r\n", temperature);
    /* Send the formatted string to the specified port */
    writePxMutex(dstPort, (char*)CString, strlen((char*)CString), cmd500ms, HAL_MAX_DELAY);

    /* Return final status indicating success or prior error */
    return Status;
}
/***************************************************************************/
/* Streams temperature data to the CLI.
 * Numofsamples: Number of samples to stream.
 * timeout: Timeout period for the operation.
 * function: Function pointer to the string formatting function (e.g., SampleTempCToString).
 */
static Module_Status StreamToCLI(uint32_t Numofsamples, uint32_t timeout, SampleToString function) {
    Module_Status status = H09R9_OK;
    uint32_t period = timeout / Numofsamples;
    char cstring[100] = {0};

    /* Check if the calculated period is valid */
    if (period < MIN_PERIOD_MS) {
        return H09R9_ERR_WRONGPARAMS;
    }

    stopStream = false;

    /* Stream data to CLI */
    while (Numofsamples-- > 0) {
        function(cstring, 50);
        writePxMutex(TerminalPort, cstring, strlen(cstring), cmd500ms, HAL_MAX_DELAY);

        /* Check for termination */
        status = PollingSleepCLISafe(period, Numofsamples);
        if (status != H09R9_OK) {
            break;
        }
    }

    return status;
}
/***************************************************************************/
/* Polling and sleep function to safely manage CLI stream.
 * @param period: The period to sleep in milliseconds.
 * @param Numofsamples: The number of samples to take.
 * @retval: Module status indicating success or error.
 */
static Module_Status PollingSleepCLISafe(uint32_t period, long Numofsamples) {
    const unsigned DELTA_SLEEP_MS = 100; // Sleep interval in milliseconds
    long numDeltaDelay = period / DELTA_SLEEP_MS; // Number of full sleep intervals
    unsigned lastDelayMS = period % DELTA_SLEEP_MS; // Remaining sleep time

    while (numDeltaDelay-- > 0) {
        // Delay for the specified interval
        vTaskDelay(pdMS_TO_TICKS(DELTA_SLEEP_MS));

        // Look for ENTER key to stop the stream
        for (uint8_t chr = 0; chr < MSG_RX_BUF_SIZE; chr++) {
            if (UARTRxBuf[pcPort - 1][chr] == '\r' && Numofsamples > 0) {
                UARTRxBuf[pcPort - 1][chr] = 0; // Clear buffer at ENTER key
                StopeCliStreamFlag = 1; // Set flag to stop CLI stream
                return H09R9_ERR_TERMINATED; // Return termination error
            }
        }

        // Check if streaming should be stopped
        if (stopStream) {
            return H09R9_ERR_TERMINATED; // Return termination error
        }
    }

    // Delay for the remaining period
    vTaskDelay(pdMS_TO_TICKS(lastDelayMS));

    return H09R9_OK; // Return success status
}
/***************************************************************************/
/* Initialize sensor coefficients for temperature measurement.
 * @brief: Reads calibration data from TSD305 sensor via I2C and calculates coefficients.
 * @retval: None
 */
void SENSOR_COEFFICIENTS_Init(void) {
    // Variables for high and low bytes of sensor data
    uint8_t h_TSenMax, l_TSenMax; // High and low bytes for maximum sensor temperature
    uint8_t h_TSenMin, l_TSenMin; // High and low bytes for minimum sensor temperature
    uint8_t h_TC1, l_TC1; // High and low bytes for temperature coefficient 1
    uint8_t h_TC2, l_TC2; // High and low bytes for temperature coefficient 2
    uint8_t h_RT1, l_RT1; // High and low bytes for reference temperature 1
    uint8_t h_RT2, l_RT2; // High and low bytes for reference temperature 2
    uint8_t h_add1, l_add1, h_add2, l_add2; // Bytes for k4comp coefficient
    uint8_t h_add3, l_add3, h_add4, l_add4; // Bytes for k3comp coefficient
    uint8_t h_add5, l_add5, h_add6, l_add6; // Bytes for k2comp coefficient
    uint8_t h_add7, l_add7, h_add8, l_add8; // Bytes for k1comp coefficient
    uint8_t h_add9, l_add9, h_add10, l_add10; // Bytes for k0comp coefficient
    uint8_t h_add11, l_add11, h_add12, l_add12; // Bytes for k4Obj coefficient
    uint8_t h_add13, l_add13, h_add14, l_add14; // Bytes for k3Obj coefficient
    uint8_t h_add15, l_add15, h_add16, l_add16; // Bytes for k2Obj coefficient
    uint8_t h_add17, l_add17, h_add18, l_add18; // Bytes for k1Obj coefficient
    uint8_t h_add19, l_add19, h_add20, l_add20; // Bytes for k0Obj coefficient
    uint8_t buf1[3] = { 0x00 }; // Buffer for receiving I2C data (first set)
    uint8_t buf2[3] = { 0x00 }; // Buffer for receiving I2C data (second set)
    uint8_t SensorAddresses; // I2C register address for sensor data

    // Check if TSD305 sensor is ready
    HAL_I2C_IsDeviceReady(I2C_HANDLER, TSD305_ADDR, 3, 100);
    Delay_ms_no_rtos(TempDelay);

    // Read sensor temperature range (TSenMax and TSenMin)
    SensorAddresses = 0x1B; // Register for TSenMax
    HAL_I2C_Master_Transmit(I2C_HANDLER, TSD305_ADDR, &SensorAddresses, 1, 100);
    Delay_ms_no_rtos(TempDelay);
    HAL_I2C_Master_Receive(I2C_HANDLER, TSD305_ADDR, buf1, 3, 100);
    TSenMax_Value = (buf1[1] << 8) + buf1[2]; // Combine high and low bytes

    SensorAddresses = 0x1A; // Register for TSenMin
    HAL_I2C_Master_Transmit(I2C_HANDLER, TSD305_ADDR, &SensorAddresses, 1, 100);
    Delay_ms_no_rtos(TempDelay);
    HAL_I2C_Master_Receive(I2C_HANDLER, TSD305_ADDR, buf2, 3, 100);
    TSenMin_Value = (buf2[1] << 8) + buf2[2]; // Combine high and low bytes

    // Read temperature coefficient (TC)
    SensorAddresses = 0x1E; // Register for TC1
    HAL_I2C_Master_Transmit(I2C_HANDLER, TSD305_ADDR, &SensorAddresses, 1, 100);
    Delay_ms_no_rtos(TempDelay);
    HAL_I2C_Master_Receive(I2C_HANDLER, TSD305_ADDR, buf1, 3, 100);
    h_TC1 = buf1[1]; // High byte of TC1
    l_TC1 = buf1[2]; // Low byte of TC1

    SensorAddresses = 0x1F; // Register for TC2
    HAL_I2C_Master_Transmit(I2C_HANDLER, TSD305_ADDR, &SensorAddresses, 1, 100);
    Delay_ms_no_rtos(TempDelay);
    HAL_I2C_Master_Receive(I2C_HANDLER, TSD305_ADDR, buf2, 3, 100);
    h_TC2 = buf2[1]; // High byte of TC2
    l_TC2 = buf2[2]; // Low byte of TC2
    TC = BytesToFloat(h_TC1, l_TC1, h_TC2, l_TC2); // Calculate TC

    // Read reference temperature (RT)
    SensorAddresses = 0x20; // Register for RT1
    HAL_I2C_Master_Transmit(I2C_HANDLER, TSD305_ADDR, &SensorAddresses, 1, 100);
    Delay_ms_no_rtos(TempDelay);
    HAL_I2C_Master_Receive(I2C_HANDLER, TSD305_ADDR, buf1, 3, 100);
    h_RT1 = buf1[1]; // High byte of RT1
    l_RT1 = buf1[2]; // Low byte of RT1

    SensorAddresses = 0x21; // Register for RT2
    HAL_I2C_Master_Transmit(I2C_HANDLER, TSD305_ADDR, &SensorAddresses, 1, 100);
    Delay_ms_no_rtos(TempDelay);
    HAL_I2C_Master_Receive(I2C_HANDLER, TSD305_ADDR, buf2, 3, 100);
    h_RT2 = buf2[1]; // High byte of RT2
    l_RT2 = buf2[2]; // Low byte of RT2
    RT = BytesToFloat(h_RT1, l_RT1, h_RT2, l_RT2); // Calculate RT

    // Read temperature compensation coefficients (k4comp to k0comp)
    SensorAddresses = 0x22; // Register for k4comp (add1, add2)
    HAL_I2C_Master_Transmit(I2C_HANDLER, TSD305_ADDR, &SensorAddresses, 1, 100);
    Delay_ms_no_rtos(TempDelay);
    HAL_I2C_Master_Receive(I2C_HANDLER, TSD305_ADDR, buf1, 3, 100);
    h_add1 = buf1[1]; // High byte of add1
    l_add1 = buf1[2]; // Low byte of add1

    SensorAddresses = 0x23; // Register for k4comp (add2)
    HAL_I2C_Master_Transmit(I2C_HANDLER, TSD305_ADDR, &SensorAddresses, 1, 100);
    Delay_ms_no_rtos(TempDelay);
    HAL_I2C_Master_Receive(I2C_HANDLER, TSD305_ADDR, buf2, 3, 100);
    h_add2 = buf2[1]; // High byte of add2
    l_add2 = buf2[2]; // Low byte of add2
    k4comp = BytesToFloat(h_add1, l_add1, h_add2, l_add2); // Calculate k4comp

    SensorAddresses = 0x24; // Register for k3comp (add3, add4)
    HAL_I2C_Master_Transmit(I2C_HANDLER, TSD305_ADDR, &SensorAddresses, 1, 100);
    Delay_ms_no_rtos(TempDelay);
    HAL_I2C_Master_Receive(I2C_HANDLER, TSD305_ADDR, buf1, 3, 100);
    h_add3 = buf1[1]; // High byte of add3
    l_add3 = buf1[2]; // Low byte of add3

    SensorAddresses = 0x25; // Register for k3comp (add4)
    HAL_I2C_Master_Transmit(I2C_HANDLER, TSD305_ADDR, &SensorAddresses, 1, 100);
    Delay_ms_no_rtos(TempDelay);
    HAL_I2C_Master_Receive(I2C_HANDLER, TSD305_ADDR, buf2, 3, 100);
    h_add4 = buf2[1]; // High byte of add4
    l_add4 = buf2[2]; // Low byte of add4
    k3comp = BytesToFloat(h_add3, l_add3, h_add4, l_add4); // Calculate k3comp

    SensorAddresses = 0x26; // Register for k2comp (add5, add6)
    HAL_I2C_Master_Transmit(I2C_HANDLER, TSD305_ADDR, &SensorAddresses, 1, 100);
    Delay_ms_no_rtos(TempDelay);
    HAL_I2C_Master_Receive(I2C_HANDLER, TSD305_ADDR, buf1, 3, 100);
    h_add5 = buf1[1]; // High byte of add5
    l_add5 = buf1[2]; // Low byte of add5

    SensorAddresses = 0x27; // Register for k2comp (add6)
    HAL_I2C_Master_Transmit(I2C_HANDLER, TSD305_ADDR, &SensorAddresses, 1, 100);
    Delay_ms_no_rtos(TempDelay);
    HAL_I2C_Master_Receive(I2C_HANDLER, TSD305_ADDR, buf2, 3, 100);
    h_add6 = buf2[1]; // High byte of add6
    l_add6 = buf2[2]; // Low byte of add6
    k2comp = BytesToFloat(h_add5, l_add5, h_add6, l_add6); // Calculate k2comp

    SensorAddresses = 0x28; // Register for k1comp (add7, add8)
    HAL_I2C_Master_Transmit(I2C_HANDLER, TSD305_ADDR, &SensorAddresses, 1, 100);
    Delay_ms_no_rtos(TempDelay);
    HAL_I2C_Master_Receive(I2C_HANDLER, TSD305_ADDR, buf1, 3, 100);
    h_add7 = buf1[1]; // High byte of add7
    l_add7 = buf1[2]; // Low byte of add7

    SensorAddresses = 0x29; // Register for k1comp (add8)
    HAL_I2C_Master_Transmit(I2C_HANDLER, TSD305_ADDR, &SensorAddresses, 1, 100);
    Delay_ms_no_rtos(TempDelay);
    HAL_I2C_Master_Receive(I2C_HANDLER, TSD305_ADDR, buf2, 3, 100);
    h_add8 = buf2[1]; // High byte of add8
    l_add8 = buf2[2]; // Low byte of add8
    k1comp = BytesToFloat(h_add7, l_add7, h_add8, l_add8); // Calculate k1comp

    SensorAddresses = 0x2A; // Register for k0comp (add9, add10)
    HAL_I2C_Master_Transmit(I2C_HANDLER, TSD305_ADDR, &SensorAddresses, 1, 100);
    Delay_ms_no_rtos(TempDelay);
    HAL_I2C_Master_Receive(I2C_HANDLER, TSD305_ADDR, buf1, 3, 100);
    h_add9 = buf1[1]; // High byte of add9
    l_add9 = buf1[2]; // Low byte of add9

    SensorAddresses = 0x2B; // Register for k0comp (add10)
    HAL_I2C_Master_Transmit(I2C_HANDLER, TSD305_ADDR, &SensorAddresses, 1, 100);
    Delay_ms_no_rtos(TempDelay);
    HAL_I2C_Master_Receive(I2C_HANDLER, TSD305_ADDR, buf2, 3, 100);
    h_add10 = buf2[1]; // High byte of add10
    l_add10 = buf2[2]; // Low byte of add10
    k0comp = BytesToFloat(h_add9, l_add9, h_add10, l_add10); // Calculate k0comp

    // Read object temperature determination coefficients (k4Obj to k0Obj)
    SensorAddresses = 0x2E; // Register for k4Obj (add11, add12)
    HAL_I2C_Master_Transmit(I2C_HANDLER, TSD305_ADDR, &SensorAddresses, 1, 100);
    Delay_ms_no_rtos(TempDelay);
    HAL_I2C_Master_Receive(I2C_HANDLER, TSD305_ADDR, buf1, 3, 100);
    h_add11 = buf1[1]; // High byte of add11
    l_add11 = buf1[2]; // Low byte of add11

    SensorAddresses = 0x2F; // Register for k4Obj (add12)
    HAL_I2C_Master_Transmit(I2C_HANDLER, TSD305_ADDR, &SensorAddresses, 1, 100);
    Delay_ms_no_rtos(TempDelay);
    HAL_I2C_Master_Receive(I2C_HANDLER, TSD305_ADDR, buf2, 3, 100);
    h_add12 = buf2[1]; // High byte of add12
    l_add12 = buf2[2]; // Low byte of add12
    k4Obj = BytesToFloat(h_add11, l_add11, h_add12, l_add12); // Calculate k4Obj

    SensorAddresses = 0x30; // Register for k3Obj (add13, add14)
    HAL_I2C_Master_Transmit(I2C_HANDLER, TSD305_ADDR, &SensorAddresses, 1, 100);
    Delay_ms_no_rtos(TempDelay);
    HAL_I2C_Master_Receive(I2C_HANDLER, TSD305_ADDR, buf1, 3, 100);
    h_add13 = buf1[1]; // High byte of add13
    l_add13 = buf1[2]; // Low byte of add13

    SensorAddresses = 0x31; // Register for k3Obj (add14)
    HAL_I2C_Master_Transmit(I2C_HANDLER, TSD305_ADDR, &SensorAddresses, 1, 100);
    Delay_ms_no_rtos(TempDelay);
    HAL_I2C_Master_Receive(I2C_HANDLER, TSD305_ADDR, buf2, 3, 100);
    h_add14 = buf2[1]; // High byte of add14
    l_add14 = buf2[2]; // Low byte of add14
    k3Obj = BytesToFloat(h_add13, l_add13, h_add14, l_add14); // Calculate k3Obj

    SensorAddresses = 0x32; // Register for k2Obj (add15, add16)
    HAL_I2C_Master_Transmit(I2C_HANDLER, TSD305_ADDR, &SensorAddresses, 1, 100);
    Delay_ms_no_rtos(TempDelay);
    HAL_I2C_Master_Receive(I2C_HANDLER, TSD305_ADDR, buf1, 3, 100);
    h_add15 = buf1[1]; // High byte of add15
    l_add15 = buf1[2]; // Low byte of add15

    SensorAddresses = 0x33; // Register for k2Obj (add16)
    HAL_I2C_Master_Transmit(I2C_HANDLER, TSD305_ADDR, &SensorAddresses, 1, 100);
    Delay_ms_no_rtos(TempDelay);
    HAL_I2C_Master_Receive(I2C_HANDLER, TSD305_ADDR, buf2, 3, 100);
    h_add16 = buf2[1]; // High byte of add16
    l_add16 = buf2[2]; // Low byte of add16
    k2Obj = BytesToFloat(h_add15, l_add15, h_add16, l_add16); // Calculate k2Obj

    SensorAddresses = 0x34; // Register for k1Obj (add17, add18)
    HAL_I2C_Master_Transmit(I2C_HANDLER, TSD305_ADDR, &SensorAddresses, 1, 100);
    Delay_ms_no_rtos(TempDelay);
    HAL_I2C_Master_Receive(I2C_HANDLER, TSD305_ADDR, buf1, 3, 100);
    h_add17 = buf1[1]; // High byte of add17
    l_add17 = buf1[2]; // Low byte of add17

    SensorAddresses = 0x35; // Register for k1Obj (add18)
    HAL_I2C_Master_Transmit(I2C_HANDLER, TSD305_ADDR, &SensorAddresses, 1, 100);
    Delay_ms_no_rtos(TempDelay);
    HAL_I2C_Master_Receive(I2C_HANDLER, TSD305_ADDR, buf2, 3, 100);
    h_add18 = buf2[1]; // High byte of add18
    l_add18 = buf2[2]; // Low byte of add18
    k1Obj = BytesToFloat(h_add17, l_add17, h_add18, l_add18); // Calculate k1Obj

    SensorAddresses = 0x36; // Register for k0Obj (add19, add20)
    HAL_I2C_Master_Transmit(I2C_HANDLER, TSD305_ADDR, &SensorAddresses, 1, 100);
    Delay_ms_no_rtos(TempDelay);
    HAL_I2C_Master_Receive(I2C_HANDLER, TSD305_ADDR, buf1, 3, 100);
    h_add19 = buf1[1]; // High byte of add19
    l_add19 = buf1[2]; // Low byte of add19

    SensorAddresses = 0x37; // Register for k0Obj (add20)
    HAL_I2C_Master_Transmit(I2C_HANDLER, TSD305_ADDR, &SensorAddresses, 1, 100);
    Delay_ms_no_rtos(TempDelay);
    HAL_I2C_Master_Receive(I2C_HANDLER, TSD305_ADDR, buf2, 3, 100);
    h_add20 = buf2[1]; // High byte of add20
    l_add20 = buf2[2]; // Low byte of add20
    k0Obj = BytesToFloat(h_add19, l_add19, h_add20, l_add20); // Calculate k0Obj
}

/***************************************************************************/
/* Convert four bytes to a float value.
 * @brief: Combines four bytes into a single float value using pointer manipulation.
 * @param b0: First byte of the float value.
 * @param b1: Second byte of the float value.
 * @param b2: Third byte of the float value.
 * @param b3: Fourth byte of the float value.
 * @retval: The resulting float value.
 */
float BytesToFloat(uint8_t b0, uint8_t b1, uint8_t b2, uint8_t b3) {
    float output; // Resulting float value

    // Assign bytes to float memory in reverse order
    *((uint8_t*)(&output) + 3) = b0; // Most significant byte
    *((uint8_t*)(&output) + 2) = b1;
    *((uint8_t*)(&output) + 1) = b2;
    *((uint8_t*)(&output) + 0) = b3; // Least significant byte

    return output; // Return the converted float
}

/***************************************************************************/
/* Sample temperature data from TSD305 sensor.
 * @brief: Reads ADC data via I2C, applies compensation, and calculates temperature.
 * @param temp: Pointer to store the calculated temperature value.
 * @retval: None
 */
void GetSample(float *temp) {
    float offest, OffsetTC, ADCComp, ADCCompTC; // Compensation and correction variables
    float TCF = 0; // Temperature correction factor
    float Tsen = 0; // Sensor temperature
    uint8_t ADC0; // Status byte from ADC
    uint8_t ADCsen3, ADCsen2, ADCsen1; // Bytes for sensor ADC value
    uint8_t ADCobj3, ADCobj2, ADCobj1; // Bytes for object ADC value
    int32_t ADCsen; // Combined sensor ADC value
    int32_t ADCobj; // Combined object ADC value
    uint8_t buf4[7] = { 0x00 }; // Buffer for receiving ADC data
    uint8_t ADC_Add = 0xAF; // ADC register address

    // Check if TSD305 sensor is ready
    HAL_I2C_IsDeviceReady(I2C_HANDLER, TSD305_ADDR, 1, 10);
    HAL_Delay(TempDelay);

    // Read ADC data
    HAL_I2C_Master_Transmit(I2C_HANDLER, TSD305_ADDR, &ADC_Add, 1, 100);
    HAL_Delay(TempDelay);
    do {
        HAL_I2C_Master_Receive(I2C_HANDLER, TSD305_ADDR, buf4, 7, 100);
        ADC0 = buf4[0]; // Status byte
        ADCobj3 = buf4[1]; // Object ADC high byte
        ADCobj2 = buf4[2]; // Object ADC middle byte
        ADCobj1 = buf4[3]; // Object ADC low byte
        ADCsen3 = buf4[4]; // Sensor ADC high byte
        ADCsen2 = buf4[5]; // Sensor ADC middle byte
        ADCsen1 = buf4[6]; // Sensor ADC low byte
    } while (buf4[0] == 0x60); // Wait until valid data is received

    // Combine ADC bytes
    ADCsen = (ADCsen3 << 16) | (ADCsen2 << 8) | ADCsen1; // Sensor ADC value
    ADCobj = (ADCobj3 << 16) | (ADCobj2 << 8) | ADCobj1; // Object ADC value

    // Calculate sensor temperature
    float t = ((float)ADCsen) / 16777216; // Normalize ADC value
    Tsen = t * (TSenMax_Value - TSenMin_Value) + TSenMin_Value; // Scale to temperature range

    // Calculate temperature correction factor
    TCF = 1 + ((Tsen - RT) * TC);

    // Apply temperature compensation
    offest = k4comp * Tsen * Tsen * Tsen * Tsen + k3comp * Tsen * Tsen * Tsen
            + k2comp * Tsen * Tsen + k1comp * Tsen + k0comp; // Polynomial compensation
    OffsetTC = offest * TCF; // Apply correction factor

    // Compensate ADC value
    ADCComp = OffsetTC + (ADCobj - 8388608) / 0.99; // Adjust ADC value
    ADCCompTC = ADCComp / TCF; // Apply final correction

    HAL_Delay(10); // Wait before final calculation

    // Calculate and store final temperature
    *temp = k4Obj * ADCCompTC * ADCCompTC * ADCCompTC * ADCCompTC
            + k3Obj * ADCCompTC * ADCCompTC * ADCCompTC
            + k2Obj * ADCCompTC * ADCCompTC + k1Obj * ADCCompTC + k0Obj; // Polynomial temperature
}

/***************************************************************************/
/* Convert temperature sample to a formatted string.
 * @brief: Samples temperature and formats it as a string with two decimal places.
 * @param cstring: Pointer to the output string buffer.
 * @param maxLen: Maximum length of the output string buffer.
 * @retval: None
 */
void SampleTemperatureToString(char *cstring, size_t maxLen) {
    float temprature = 0; // Temperature value from sensor
    float temp; // Temporary storage for temperature

    // Sample temperature
    GetSample(&temprature);
    temp = temprature; // Store temperature locally

    // Format temperature as string
    snprintf(cstring, maxLen, "Temperature: %.2f\r\n", temprature); // Output with two decimal places
}

/***************************************************************************/
/* Stop the streaming process.
 * @brief: Sets the stopStream flag to terminate streaming.
 * @retval: None
 */
void stopStreamMems(void) {
    stopStream = true; // Set flag to stop streaming
}

/***************************************************************************/
/***************************** General Functions ***************************/
/***************************************************************************/

/* Sample temperature from sensor.
 * @brief: Retrieves temperature data by calling GetSample and stores it in the provided pointer.
 * @param temp: Pointer to store the sampled temperature value.
 * @retval: Module status indicating success or error.
 */
Module_Status SampleTemperature(float *temp) {
    Module_Status status = H09R9_OK; // Initial status indicating success

    // Sample temperature from sensor
    GetSample(temp); // Retrieve temperature and store in temp

    return status; // Return success status
}

/***************************************************************************/
/* Samples temperature data and exports it to a specified port.
 * dstModule: The module number to export data from.
 * dstPort: The port number to export data to.
 */
Module_Status SampleToPort(uint8_t dstModule, uint8_t dstPort) {
    Module_Status Status = H09R9_OK;
    static uint8_t Temp[12] = {0}; /* Buffer for data transmission */
    float temperature = 0.0f;

    /* Check if the port and module ID are valid */
    if ((dstPort == 0) && (dstModule == myID)) {
        return H09R9_ERR_WRONGPARAMS;
    }

    /* Sample temperature data */
    if (SampleTemperature(&temperature) != H09R9_OK) {
        return H09R9_ERROR;
    }

    if (dstModule == myID || dstModule == 0) {
        /* LSB first */
        Temp[0] = (uint8_t)((*(uint32_t*)&temperature) >> 0);
        Temp[1] = (uint8_t)((*(uint32_t*)&temperature) >> 8);
        Temp[2] = (uint8_t)((*(uint32_t*)&temperature) >> 16);
        Temp[3] = (uint8_t)((*(uint32_t*)&temperature) >> 24);

        writePxITMutex(dstPort, (char*)&Temp[0], 4 * sizeof(uint8_t), 10);
    } else {
        /* LSB first */
        MessageParams[1] = (H09R9_OK == Status) ? BOS_OK : BOS_ERROR;
        MessageParams[0] = FMT_FLOAT;
        MessageParams[2] = 1;
        MessageParams[3] = (uint8_t)((*(uint32_t*)&temperature) >> 0);
        MessageParams[4] = (uint8_t)((*(uint32_t*)&temperature) >> 8);
        MessageParams[5] = (uint8_t)((*(uint32_t*)&temperature) >> 16);
        MessageParams[6] = (uint8_t)((*(uint32_t*)&temperature) >> 24);

        SendMessageToModule(dstModule, CODE_READ_RESPONSE, (sizeof(float) * 1) + 3);
    }

    /* Clear the temp buffer */
    memset(&Temp[0], 0, sizeof(Temp));

    return Status;
}
/***************************************************************************/
/*
 * brief: Streams temperature data to the specified port and module with a given number of samples.
 * param targetModule: The target module to which data will be streamed.
 * param portNumber: The port number on the module.
 * param numOfSamples: The number of samples to stream.
 * param streamTimeout: The interval (in milliseconds) between successive data transmissions.
 * retval: of type Module_Status indicating the success or failure of the operation.
 */
Module_Status StreamtoPort(uint8_t dstModule, uint8_t dstPort, uint32_t numOfSamples, uint32_t streamTimeout) {
    Module_Status Status = H09R9_OK;
    uint32_t SamplePeriod = 0u;

    /* Check timer handle and timeout validity */
    if ((NULL == xTimerStream) || (0 == streamTimeout) || (0 == numOfSamples)) {
        return H09R9_ERROR; /* Assuming H09R9_ERROR is defined in Module_Status */
    }

    /* Set streaming parameters */
    StreamMode = STREAM_MODE_TO_PORT;
    PortModule = dstModule;
    PortNumber = dstPort;
    PortNumOfSamples = numOfSamples;

    /* Calculate the period from timeout and number of samples */
    SamplePeriod = streamTimeout / numOfSamples;

    /* Stop (Reset) the TimerStream if it's already running */
    if (xTimerIsTimerActive(xTimerStream)) {
        if (pdFAIL == xTimerStop(xTimerStream, 100)) {
            return H09R9_ERROR;
        }
    }

    /* Start the stream timer */
    if (pdFAIL == xTimerStart(xTimerStream, 100)) {
        return H09R9_ERROR;
    }

    /* Update timer timeout - This also restarts the timer */
    if (pdFAIL == xTimerChangePeriod(xTimerStream, SamplePeriod, 100)) {
        return H09R9_ERROR;
    }

    return Status;
}

/***************************************************************************/
/*
 * brief: Streams temperature data to the specified terminal port with a given number of samples.
 * param targetPort: The port number on the terminal.
 * param numOfSamples: The number of samples to stream.
 * param streamTimeout: The interval (in milliseconds) between successive data transmissions.
 * retval: of type Module_Status indicating the success or failure of the operation.
 */
Module_Status StreamToTerminal(uint8_t dstPort, uint32_t numOfSamples, uint32_t streamTimeout) {
    Module_Status Status = H09R9_OK;
    uint32_t SamplePeriod = 0u;

    /* Check timer handle and timeout validity */
    if ((NULL == xTimerStream) || (0 == streamTimeout) || (0 == numOfSamples)) {
        return H09R9_ERROR; /* Assuming H09R9_ERROR is defined in Module_Status */
    }

    /* Set streaming parameters */
    StreamMode = STREAM_MODE_TO_TERMINAL;
    TerminalPort = dstPort;
    TerminalNumOfSamples = numOfSamples;

    /* Calculate the period from timeout and number of samples */
    SamplePeriod = streamTimeout / numOfSamples;

    /* Stop (Reset) the TimerStream if it's already running */
    if (xTimerIsTimerActive(xTimerStream)) {
        if (pdFAIL == xTimerStop(xTimerStream, 100)) {
            return H09R9_ERROR;
        }
    }

    /* Start the stream timer */
    if (pdFAIL == xTimerStart(xTimerStream, 100)) {
        return H09R9_ERROR;
    }

    /* Update timer timeout - This also restarts the timer */
    if (pdFAIL == xTimerChangePeriod(xTimerStream, SamplePeriod, 100)) {
        return H09R9_ERROR;
    }

    return Status;
}
/***************************************************************************/
/*
 * @brief: Streams temperature data to a buffer.
 * @param buffer: Pointer to the buffer where data will be stored.
 * @param Numofsamples: Number of samples to take.
 * @param timeout: Timeout period for the operation.
 * @retval: Module status indicating success or error.
 */
Module_Status StreamToBuffer(float *buffer, uint32_t Numofsamples, uint32_t timeout) {
    return StreamToBuf(buffer, Numofsamples, timeout, SampleTemperatureBuf);
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
			StreamToCLI(period, timeout, SampleTemperatureToString);
		else
			StreamtoPort(port, module, period, timeout);

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

	StreamToCLI(1, 100, SampleTemperatureToString);
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
	StreamToCLI(Numofsamples, pTimeout, SampleTemperatureToString);
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
	StreamtoPort(Port, 0, Numofsamples, pTimeout);

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

	SampleToPort(Port, 0);

	/* There is no more data to return after this single string, so return pdFALSE. */
	return pdFALSE;
}

/***************************************************************************/
/***************** (C) COPYRIGHT HEXABITZ ***** END OF FILE ****************/
