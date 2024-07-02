/*
 BitzOS (BOS) V0.3.5 - Copyright (C) 2017-2024 Hexabitz
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
float f;
/* User Task */
void UserTask(void *argument) {
//	StreamTemperatureToTerminal(10, 10000, 3);
	uint32_t Numofsamples = 10;
		uint32_t timeout = 5000;
		messageParams[0] = 2; // module ID
		messageParams[1] = 1; // port
		memcpy(&messageParams[2], &Numofsamples, 4);
		memcpy(&messageParams[6], &timeout, 4);
		SendMessageToModule(2, CODE_H08R7_STREAM_PORT, 10);
	// put your code here, to run repeatedly.
	while (1) {
//		messageParams[0] = 2; // module ID
//		messageParams[1] = 1; // port SendMessageToModule(1,CODE_H08R7_SAMPLE_PORT, 2);
//		SendMessageToModule(2,CODE_H08R7_SAMPLE_PORT, 2);
//		SampleTemperatureToPort(3, 0);
//		SampleTemperature(&f);
//		SendMessageToModule(2,CODE_PING,0);
	}
}

/*-----------------------------------------------------------*/
