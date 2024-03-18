/*
 BitzOS (BOS) V0.3.1 - Copyright (C) 2017-2024 Hexabitz
 All rights reserved
 
 File Name     : H09R9.h
 Description   : Header file for module H09R9.
 	 	 	 	 (Description_of_module)

(Description of Special module peripheral configuration):
>>
>>
>>

 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef H09R9_H
#define H09R9_H

/* Includes ------------------------------------------------------------------*/
#include "BOS.h"
#include "H09R9_MemoryMap.h"
#include "H09R9_uart.h"
#include "H09R9_gpio.h"
#include "H09R9_dma.h"
#include "H09R9_inputs.h"
#include "H09R9_eeprom.h"
#include "H09R9_i2c.h"
/* Exported definitions -------------------------------------------------------*/

#define	modulePN		_H09R9


/* Port-related definitions */
#define	NumOfPorts			6

#define P_PROG 				P2						/* ST factory bootloader UART */

/* Define available ports */
#define _P1 
#define _P2 
#define _P3 
#define _P4 
#define _P5 
#define _P6

/* Define available USARTs */
#define _Usart1 1
#define _Usart2 1
#define _Usart3 1
#define _Usart4 1
#define _Usart5 1
#define _Usart6 1

/* Port-UART mapping */
#define P1uart &huart4
#define P2uart &huart2
#define P3uart &huart6
#define P4uart &huart3
#define P5uart &huart1
#define P6uart &huart5

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
#define	USART6_AF			GPIO_AF4_USART6


/* Module-specific Definitions */
#define MIN_MEMS_PERIOD_MS				100
#define MAX_MEMS_TIMEOUT_MS				0xFFFFFFFF
/* Indicator LED */
#define _IND_LED_PORT										GPIOB
#define _IND_LED_PIN										GPIO_PIN_0

#define NUM_MODULE_PARAMS		1


/* H09R9 Module Special Timer */


/* H09R9 Module Special ADC */


/* H09R9 Module special parameters */
typedef enum
{
  H09R9_OK = 0,
  H09R9_ERR_UnknownMessage,
  H09R9_ERR_TEMPRATURE,
  H09R9_ERR_BUSY,
  H09R9_ERR_TIMEOUT,
  H09R9_ERR_IO,
  H09R9_ERR_TERMINATED,
  H09R9_ERR_WrongParams,
  H09R9_ERROR = 25
} Module_Status;

/* Module EEPROM Variables */
// Module Addressing Space 500 - 599
#define _EE_MODULE							500		

/* EXG Module_Status Type Definition */
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
extern void ExecuteMonitor(void);

/* -----------------------------------------------------------------------
 |								  APIs							          ||
/* -----------------------------------------------------------------------
 */

void SetupPortForRemoteBootloaderUpdate(uint8_t port);
void remoteBootloaderUpdate(uint8_t src,uint8_t dst,uint8_t inport,uint8_t outport);
void Error_Handler(void);
void SENSOR_COEFFICIENTS_Init(void);
void stopStreamMems(void);
float bytesToFloat(uchar b0, uchar b1, uchar b2, uchar b3);
void SampleTemperature(float *temp);
void SampleTemperatureToPort(uint8_t port,uint8_t module);
Module_Status StreamTemperatureToPort(uint8_t port, uint8_t module, uint32_t period, uint32_t timeout);
Module_Status StreamTemperatureToCLI(uint32_t period, uint32_t timeout);


/* -----------------------------------------------------------------------
 |								Commands							      ||
/* -----------------------------------------------------------------------
 */


#endif /* H09R9_H */

/************************ (C) COPYRIGHT HEXABITZ *****END OF FILE****/
