/*
 * FloodSensor.c
 *
 *  Created on: 10 giu 2026
 *      Author: lucamaggiotanasi
 */
#include "FloodSensor.h"
#include "main.h"

static FloodSensor_t FloodSensor[2] = {
	    {
	        .id = FLOOD1,
			.flood=0,

	    },
	    {
	        .id = FLOOD2,
			.flood=0,

	    }
};

/**
 * @brief Restituisce lo stato del sensore di allagamento selezionato.
 */
bool FloodSensorGetStatus(FloodSensor_id_t flood_id){
    if (flood_id == FLOOD1)
    {
    	FloodSensor[flood_id].flood=!HAL_GPIO_ReadPin(nFLOOD_1_GPIO_Port,nFLOOD_1_Pin);
    }
    if (flood_id == FLOOD2)
    {
    	FloodSensor[flood_id].flood=!HAL_GPIO_ReadPin(nFLOOD_2_GPIO_Port,nFLOOD_2_Pin);
    }

    return FloodSensor[flood_id].flood;
}
