/*
 * WaterValve.h
 *
 *  Created on: 9 giu 2026
 *      Author: lucamaggiotanasi
 */

#ifndef WATERVALVE_WATERVALVE_H_
#define WATERVALVE_WATERVALVE_H_

#include <stdbool.h>

void WaterValveOpen(void);
void WaterValveClose(void);

/**
 * @brief Stato fisico ATTUALE della valvola, letto back dal pin di comando
 *        nWATER_VALVE_EN (active LOW) — non da una variabile in RAM: valido
 *        anche se il pin fosse mai scritto da un punto diverso da questo
 *        driver. Usato per rispecchiare nWATER_VALVE_STATUS istante per
 *        istante (vedi TaskInputs_Run(), task_inputs.c).
 * @retval true  = valvola aperta (nWATER_VALVE_EN LOW)
 * @retval false = valvola chiusa (nWATER_VALVE_EN HIGH)
 */
bool WaterValve_IsOpen(void);

#endif /* WATERVALVE_WATERVALVE_H_ */
