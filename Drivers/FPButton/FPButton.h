/*
 * FPButton.h
 *
 * Driver per il pulsante momentaneo del pannello frontale (nFP_BUTTON).
 *
 * HARDWARE:
 *   nFP_BUTTON  (PC15, EXTI15, active LOW, entrambi i fronti)
 *
 * EVENTI GENERATI (tramite callback applicativa):
 *   FPBTN_EVT_PRESS        — pressione singola breve
 *   FPBTN_EVT_LONG_PRESS   — pressione prolungata (emesso al rilascio)
 *   FPBTN_EVT_DOUBLE_PRESS — doppio click
 *
 * INTEGRAZIONE:
 *
 *   1) Registrare la callback prima di avviare il task:
 *        FPButton_RegisterCallback(my_event_cb);
 *
 *   2) Chiamare FPButton_Process(tick_ms) ogni tick_ms ms da task_inputs.
 *
 *   3) In Core/Src/stm32h7xx_it.c, dentro HAL_GPIO_EXTI_Callback():
 *
 *        #include "FPButton.h"
 *
 *        void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
 *        {
 *            if (GPIO_Pin == nFP_BUTTON_Pin) { FPButton_EXTI_Callback(); }
 *        }
 */

#ifndef DRIVERS_FPBUTTON_FPBUTTON_H_
#define DRIVERS_FPBUTTON_FPBUTTON_H_

#include <stdbool.h>
#include <stdint.h>

/* Soglie temporali [ms] */
#define FPBTN_DEBOUNCE_MS       20U
#define FPBTN_LONG_PRESS_MS     1000U
#define FPBTN_DOUBLE_PRESS_MS   400U

/* ========================================================================== */
/* --- CALLBACK --- */
/* ========================================================================== */

/** Tipo evento pulsante — usato come argomento della callback */
typedef enum {
    FPBTN_EVT_PRESS = 0,
    FPBTN_EVT_LONG_PRESS,
    FPBTN_EVT_DOUBLE_PRESS,
} FPButton_Evt_t;

/** Prototipo della callback applicativa per gli eventi pulsante */
typedef void (*FPButton_EventCb_t)(FPButton_Evt_t evt);

/**
 * @brief  Registra la callback che riceve gli eventi del pulsante.
 *         Chiamare dal layer applicativo (task_inputs) prima dello scheduler.
 * @param  cb  Puntatore alla funzione di callback (NULL per disabilitare).
 */
void FPButton_RegisterCallback(FPButton_EventCb_t cb);

/* ========================================================================== */
/* --- API DRIVER --- */
/* ========================================================================== */

/**
 * @brief  Callback EXTI per nFP_BUTTON (EXTI15).
 * @note   Chiamare da HAL_GPIO_EXTI_Callback() quando GPIO_Pin == nFP_BUTTON_Pin.
 *         Eseguita in contesto ISR — nessuna operazione bloccante.
 */
void FPButton_EXTI_Callback(void);

/**
 * @brief  Macchina a stati del pulsante.
 * @param  tick_ms  Millisecondi trascorsi dall'ultima chiamata (periodo del task chiamante).
 * @note   Chiamare ogni tick_ms ms da task_inputs.
 */
void FPButton_Process(uint32_t tick_ms);

#endif /* DRIVERS_FPBUTTON_FPBUTTON_H_ */
