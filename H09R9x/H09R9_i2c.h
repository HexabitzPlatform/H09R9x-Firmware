/*
 BitzOS (BOS) V0.2.9 - Copyright (C) 2017-2023 Hexabitz
 All rights reserved

 File Name          : H09R9_i2c.h
 Description        : This file contains all the functions prototypes for
                      the i2c
*/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __i2c_H
#define __i2c_H
#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f0xx_hal.h"

extern I2C_HandleTypeDef hi2c1;


extern void MX_I2C_Init(void);
extern void MX_I2C1_Init(void);

#ifdef __cplusplus
}
#endif
#endif /*__i2c_H */


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
