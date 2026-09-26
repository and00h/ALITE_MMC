/*
 * FloodSensor.h
 *
 *  Created on: 10 giu 2026
 *      Author: lucamaggiotanasi
 */

#ifndef FLOODSENSOR_FLOODSENSOR_H_
#define FLOODSENSOR_FLOODSENSOR_H_

#include "stm32h7xx_hal.h"
#include <stdbool.h>

typedef enum{
	FLOOD1,              /**< Istanza Sensore di allagamento 1 */
	FLOOD2                /**< Istanza Sensore di allagamento 2 */
}FloodSensor_id_t;

typedef struct{
	FloodSensor_id_t id;			//Sensor identificator
	bool flood;
} FloodSensor_t;

/**
 * @brief Restituisce lo stato del sensore di allagamento selezionato.
 */
bool FloodSensorGetStatus(FloodSensor_id_t flood_id);

#endif /* FLOODSENSOR_FLOODSENSOR_H_ */
