/*
 BitzOS (BOS) V0.2.9 - Copyright (C) 2017-2023 Hexabitz
 All rights reserved

 File Name     : main.c
 Description   : Main program body.
 */
/* Includes ------------------------------------------------------------------*/
#include "BOS.h"

/* Private variables ---------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/

/* Main function ------------------------------------------------------------*/

int main(void){

	Module_Init();		//Initialize Module &  BitzOS

	//Don't place your code here.
	for(;;){}
}

/*-----------------------------------------------------------*/
int v ;
float teqmp;
HAL_StatusTypeDef d ;
HAL_StatusTypeDef s ;
static const uint8_t TSD305_ADDR=0x00<<1;
extern uint8_t ADC_Adress;
extern int16_t TSenMax_Value,TSenMin_Value;
int r ;
/* User Task */
void UserTask(void *argument){
//	HAL_Delay(100);
	do {
//		HAL_Delay(100);
			SENSOR_COEFFICIENTS_Init();
			r++;
	} while (0==TSenMax_Value);
//	SENSOR_COEFFICIENTS_Init();

	// put your code here, to run repeatedly.
	while(1){


//		d=HAL_I2C_IsDeviceReady(&hi2c2,TSD305_ADDR,1,10);
//		HAL_Delay(100);
//
//		s=HAL_I2C_Master_Transmit(&hi2c2, TSD305_ADDR, &ADC_Adress, 1,100);
		SampleTemperature(&teqmp);
//			if (0==TSenMax_Value)
//		{
//
//
//		}

v++;
	}
}

/*-----------------------------------------------------------*/
