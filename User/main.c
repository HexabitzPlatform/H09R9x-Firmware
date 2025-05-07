/*
 BitzOS (BOS) V0.3.6 - Copyright (C) 2017-2024 Hexabitz
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
extern UART_HandleTypeDef huart4;
/*-----------------------------------------------------------*/
float adcalue,adcalue4,adcalue2,adcalu3;
uint8_t k[100];
uint8_t f ;
/* User Task */
void UserTask(void *argument){

  // put your code here, to run repeatedly.


		while (1) {
			if (f == 2) {
						Bridge(P1, P6);
				Bridge(P2, P3);
				Bridge(P4, P5);
				f = 0;
			}
			if (f == 1) {
				Unbridge(P1, P6);
				Unbridge(P2, P3);
				Unbridge(P4, P5);
				f = 0;
			}
		}

	}

/*-----------------------------------------------------------*/
/*-----------------------------------------------------------*/
