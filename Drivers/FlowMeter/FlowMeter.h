/*
 * FlowMeter.h
 *
 *  Created on: 9 giu 2026
 *      Author: lucamaggiotanasi
 */

#ifndef FLOWMETER_FLOWMETER_H_
#define FLOWMETER_FLOWMETER_H_

#include "stm32h7xx_hal.h"

/* Enum per identificare i flussometri senza accedere all'hardware */
typedef enum {
    FLOW_METER_1 = 0,
    FLOW_METER_2,
    FLOW_METER_COUNT /* Tiene traccia di quanti flussometri abbiamo */
} FlowMeter_ID_t;

/* ========================================================================== */
/* --- API PUBBLICA --- */
/* ========================================================================== */

/**
 * @brief  Inizializza un flussometro associandolo a un timer e canale specifici.
 * Le istanze hardware sono allocate internamente al driver.
 */
void  FlowMeter_Init(FlowMeter_ID_t id, TIM_HandleTypeDef *htim, uint32_t channel);

/**
 * @brief  Getter per ottenere la frequenza di una linea, filtrata con media
 *         mobile sugli ultimi 8 impulsi (FLOW_AVG_SAMPLES, FlowMeter.c) per
 *         smussare il rumore intrinseco della stima "a periodo singolo".
 */
float FlowMeter_GetFrequency(FlowMeter_ID_t id);

/**
 * @brief  Getter per ottenere il flusso in LPM di una linea (stessa media
 *         mobile di FlowMeter_GetFrequency(), convertita con il K-factor del
 *         sensore e poi linearizzata con la taratura vs sensore da banco —
 *         Q_reale = FLOW_CAL_GAIN*Q_opal + FLOW_CAL_OFFSET, vedi FlowMeter.c).
 *         Valida nel range di taratura (~2.5-5.3 L/min); fuori da quel range
 *         e' un'estrapolazione.
 */
float FlowMeter_GetFlowRate(FlowMeter_ID_t id);

/* ========================================================================== */
/* --- INTERFACCIA DI BACKEND  --- */
/* ========================================================================== */

/**
 * @brief  Da mettere in HAL_TIM_IC_CaptureCallback. Identifica il canale
 * attivo del timer ed aggiorna l'istanza corretta in autonomia.
 */
void  FlowMeter_CaptureCallback(TIM_HandleTypeDef *htim);

/**
 * @brief  Da mettere in HAL_TIM_PeriodElapsedCallback (evento Update/overflow
 *         del timer). Necessaria per contare gli overflow del contatore a 16
 *         bit tra due impulsi consecutivi del flussometro: a piena velocità
 *         (nessun prescaler configurato su TIM4) il contatore va in overflow
 *         molte volte tra un impulso e l'altro, e senza questo conteggio il
 *         calcolo del periodo si "aliasa", dando frequenze/portate errate e
 *         incoerenti da una lettura all'altra.
 */
void  FlowMeter_PeriodElapsedCallback(TIM_HandleTypeDef *htim);

#endif /* FLOWMETER_FLOWMETER_H_ */
