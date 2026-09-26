/*
 * WaterValve.c
 *
 *  Created on: 9 giu 2026
 *      Author: lucamaggiotanasi
 */
#include "WaterValve.h"
#include "main.h"

void WaterValveOpen(void){
	HAL_GPIO_WritePin(nWATER_VALVE_EN_GPIO_Port,nWATER_VALVE_EN_Pin,GPIO_PIN_RESET);
}

void WaterValveClose(void){
	HAL_GPIO_WritePin(nWATER_VALVE_EN_GPIO_Port,nWATER_VALVE_EN_Pin,GPIO_PIN_SET);
}

bool WaterValve_IsOpen(void)
{
	/* nWATER_VALVE_EN active LOW: RESET(0) = aperta. HAL_GPIO_ReadPin() su un
	 * pin di output STM32 legge comunque l'IDR (stato elettrico realmente
	 * pilotato sul pin in push-pull), non richiede una variabile di stato
	 * dedicata. */
	return (HAL_GPIO_ReadPin(nWATER_VALVE_EN_GPIO_Port, nWATER_VALVE_EN_Pin) == GPIO_PIN_RESET);
}
