/*
 BitzOS (BOS) V0.4.0 - Copyright (C) 2017-2025 Hexabitz
 All rights reserved
 
 File Name     : H09R9.h
 Description   : Header file for module H09R9.
 	 	 	 	 (Description_of_module)

(Description of Special module peripheral configuration):
>>
>>
>>

 */

/* Define to prevent recursive inclusion ***********************************/
#ifndef H09R9_H
#define H09R9_H

/* Includes ****************************************************************/
#include "BOS.h"
#include "H09R9_MemoryMap.h"
#include "H09R9_uart.h"
#include "H09R9_gpio.h"
#include "H09R9_dma.h"
#include "H09R9_inputs.h"
#include "H09R9_eeprom.h"
#include "H09R9_i2c.h"

/* Exported Macros *********************************************************/
#define	MODULE_PN		_H09R9

/* Port-related Definitions */
#define	NUM_OF_PORTS	6
#define P_PROG 			P2		/* ST factory bootloader UART */

/* Define Available Ports */
#define _P1
#define _P2
#define _P3
#define _P4
#define _P5
#define _P6

/* Define Available USARTs */
#define _USART1
#define _USART2
#define _USART3
#define _USART4
#define _USART5
#define _USART6

/* Port-UART mapping */
#define UART_P1 &huart4
#define UART_P2 &huart2
#define UART_P3 &huart6
#define UART_P4 &huart1
#define UART_P5 &huart5
#define UART_P6 &huart3

/* Module-specific Hardware Definitions ************************************/
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

#define	USART3_TX_PIN		GPIO_PIN_8
#define	USART3_RX_PIN		GPIO_PIN_9
#define	USART3_TX_PORT		GPIOB
#define	USART3_RX_PORT		GPIOB
#define	USART3_AF			GPIO_AF4_USART3

#define	USART4_TX_PIN		GPIO_PIN_0
#define	USART4_RX_PIN		GPIO_PIN_1
#define	USART4_TX_PORT		GPIOA
#define	USART4_RX_PORT		GPIOA
#define	USART4_AF			GPIO_AF4_USART4

#define	USART5_TX_PIN		GPIO_PIN_3
#define	USART5_RX_PIN		GPIO_PIN_2
#define	USART5_TX_PORT		GPIOD
#define	USART5_RX_PORT		GPIOD
#define	USART5_AF			GPIO_AF3_USART5

#define	USART6_TX_PIN		GPIO_PIN_4
#define	USART6_RX_PIN		GPIO_PIN_5
#define	USART6_TX_PORT		GPIOA
#define	USART6_RX_PORT		GPIOA
#define USART6_AF           GPIO_AF3_USART6

/* I2C Pin Definition */
#define SENSOR_I2C_SCL_PIN  GPIO_PIN_3
#define SENSOR_I2C_SDA_PIN  GPIO_PIN_4
#define SENSOR_I2C_PORT     GPIOB

#define I2C_HANDLER         &hi2c2

/* Indicator LED */
#define _IND_LED_PORT		GPIOB
#define _IND_LED_PIN		GPIO_PIN_0

/* Module-specific Macro Definitions ***************************************/
#define TSD305_ADDR             0x00<<1
#define TempDelay               10
#define NUM_MODULE_PARAMS		1

#define SAMPLE_TEM              1


/* Macros definitions */
#define MIN_PERIOD_MS    		 100
#define MAX_TIMEOUT_MS		     0xFFFFFFFF

#define STREAM_MODE_TO_PORT      1
#define STREAM_MODE_TO_TERMINAL  2

/* Module-specific Type Definition *****************************************/
/* Module-status Type Definition */
typedef enum {
	H09R9_OK = 0,
	H09R9_ERR_UNKNOWNMESSAGE,
	H09R9_ERR_TEMPRATURE,
	H09R9_ERR_BUSY,
	H09R9_ERR_TIMEOUT,
	H09R9_ERR_IO,
	H09R9_ERR_TERMINATED,
	H09R9_ERR_WRONGPARAMS,
	H09R9_ERROR = 25
} Module_Status;

/* */
typedef unsigned char uchar;

/* Export UART variables */
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;
extern UART_HandleTypeDef huart4;
extern UART_HandleTypeDef huart5;
extern UART_HandleTypeDef huart6;

/* Define UART Init prototypes */
extern void MX_USART1_UART_Init(void);
extern void MX_USART2_UART_Init(void);
extern void MX_USART3_UART_Init(void);
extern void MX_USART4_UART_Init(void);
extern void MX_USART5_UART_Init(void);
extern void MX_USART6_UART_Init(void);
extern void SystemClock_Config(void);

/***************************************************************************/
/***************************** General Functions ***************************/
/***************************************************************************/
Module_Status SampleTemperature(float *temp);

Module_Status SampleToPort(uint8_t dstModule, uint8_t dstPort);
Module_Status StreamtoPort(uint8_t dstModule,uint8_t dstPort,uint32_t numOfSamples,uint32_t streamTimeout);
Module_Status StreamToTerminal(uint8_t dstPort,uint32_t numOfSamples,uint32_t streamTimeout);
Module_Status StreamToBuffer(float *buffer, uint32_t Numofsamples, uint32_t timeout);

#endif /* H09R9_H */
/***************** (C) COPYRIGHT HEXABITZ ***** END OF FILE ****************/
