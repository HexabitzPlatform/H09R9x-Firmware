/*
 BitzOS (BOS) V0.2.9 - Copyright (C) 2017-2023 Hexabitz
 All rights reserved

 File Name          : H09R9_i2c.c
 Description        : This file provides code for the configuration
                      of the I2C instances.

  */

/* Includes ------------------------------------------------------------------*/
#include "BOS.h"
#include <string.h>
#include <stdio.h>

//initialize i2c
void MX_I2C_Init(void);
void MX_I2C1_Init(void);
extern void Error_Handler(void);

I2C_HandleTypeDef hi2c2;



/*----------------------------------------------------------------------------*/
/* Configure I2C                                                             */
/*----------------------------------------------------------------------------*/

/** I2C Configuration
*/
void MX_I2C_Init(void)
{
  /* GPIO Ports Clock Enable */
  __GPIOC_CLK_ENABLE();
  __GPIOA_CLK_ENABLE();
  __GPIOD_CLK_ENABLE();
  __GPIOB_CLK_ENABLE();
  __GPIOF_CLK_ENABLE();   // for HSE and Boot0

  MX_I2C1_Init();
}



void MX_I2C1_Init(void)
 {

	/* USER CODE BEGIN I2C2_Init 0 */

	/* USER CODE END I2C2_Init 0 */

	/* USER CODE BEGIN I2C2_Init 1 */

	/* USER CODE END I2C2_Init 1 */
	hi2c2.Instance = I2C2;
	hi2c2.Init.Timing = 0x00303D5B/*Standard mode*/;
	hi2c2.Init.OwnAddress1 = 0;
	hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	hi2c2.Init.OwnAddress2 = 0;
	hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
	hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
	if (HAL_I2C_Init(&hi2c2) != HAL_OK) {
		Error_Handler();
	}

}

/*-----------------------------------------------------------*/

