/*
 BitzOS (BOS) V0.2.9 - Copyright (C) 2017-2023 Hexabitz
 All rights reserved

 File Name     : H09R9_gpio.h
 Description   : Header file contains all the functions prototypes for
 the GPIO .

 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __H09Rx_gpio_H
#define __H09Rx_gpio_H
#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f0xx_hal.h"


extern void GPIO_Init(void);
extern void IND_LED_Init(void);
extern void SENSOR_GPIO_Init(void);



#ifdef __cplusplus
}
#endif
#endif /*__H23Rx_gpio_H */


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
