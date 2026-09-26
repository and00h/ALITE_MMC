/*
 * task_fsm.h
 *
 * Task dedicato alla macchina a stati di sistema.
 *
 * RESPONSABILITÀ:
 *   - Attendere eventi dalla coda eventQueueHandle (bloccante con osWaitForever)
 *   - Chiamare FSM_ProcessEvent() per ogni evento ricevuto
 *   - Non esegue mai I/O diretto: tutto avviene nelle action (fsm.c)
 *
 * PRIORITÀ: osPriorityNormal4 (alta ma sotto task_monitor per sicurezza)
 *
 * PERIODICITÀ: event-driven puro, nessun tick periodico.
 *              Il task si risveglia solo quando c'è un evento da processare.
 */

#ifndef APP_TASKS_TASK_FSM_H_
#define APP_TASKS_TASK_FSM_H_

#include "cmsis_os.h"

/* Attributi del task (stack, priorità) - usati in freertos.c */
extern const osThreadAttr_t taskFSM_attr;

/**
 * @brief Entry point del task FSM.
 *        Da passare a osThreadNew() in MX_FREERTOS_Init().
 */
void TaskFSM_Run(void *arg);

#endif /* APP_TASKS_TASK_FSM_H_ */
