/*
 * task_outputs.h
 *
 * Task di aggiornamento periodico delle uscite visive (LED pannello frontale).
 *
 * RESPONSABILITÀ:
 *   - Chiamare FPLED_Process() ogni FPLED_PROCESS_PERIOD_MS (1ms)
 *     per aggiornare gli effetti PWM dei LED (fade, blink)
 *
 * PRIORITÀ: osPriorityNormal1 (la più bassa tra i task applicativi:
 *           i LED non sono safety-critical)
 *
 * PERIODICITÀ: 1ms con vTaskDelayUntil (nessun drift)
 *
 * NOTA: questo task NON decide il colore o la modalità dei LED.
 *       Quelle decisioni appartengono alle action della FSM (fsm.c).
 *       Questo task esegue solo il "motore" degli effetti.
 */

#ifndef APP_TASKS_TASK_OUTPUTS_H_
#define APP_TASKS_TASK_OUTPUTS_H_

#include "cmsis_os.h"

/* Attributi del task - usati in freertos.c */
extern const osThreadAttr_t taskOutputs_attr;

/**
 * @brief Entry point del task outputs.
 *        Da passare a osThreadNew() in MX_FREERTOS_Init().
 */
void TaskOutputs_Run(void *arg);

#endif /* APP_TASKS_TASK_OUTPUTS_H_ */
