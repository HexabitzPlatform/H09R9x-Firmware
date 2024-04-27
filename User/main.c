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

int main(void) {

	Module_Init();		//Initialize Module &  BitzOS

	//Don't place your code here.
	for (;;) {
	}
}

/*-----------------------------------------------------------*/
int v;
float teqmp;
HAL_StatusTypeDef d;
HAL_StatusTypeDef s;
static const uint8_t TSD305_ADDR = 0x00 << 1;
extern uint8_t ADC_Adress;
extern int16_t TSenMax_Value, TSenMin_Value;
int r;
float fr[20];
/* User Task */
void UserTask(void *argument) {
//	StreamTemperatureToPort(5, 0, 10, 10000);
//	StreamTemperatureToTerminal(10, 10000, 5);
	 StreamTemperatureToBuffer(fr, 19, 10000);
	// put your code here, to run repeatedly.
	while (1) {

//	SampleTemperatureToPort(5, 0);
//		ExportToPort(5, 0);
//		StreamTemperatureToCLI(100,10000);
//		HAL_UART_Transmit(&huart6, 10, 1, 1000);
//		SampleTemperature(&teqmp);
		v++;
	}
}

/*-----------------------------------------------------------*/
