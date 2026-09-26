/*
 * queues.h
 *
 * Punto unico di dichiarazione degli handle delle code FreeRTOS condivise
 * tra i task del sistema.
 *
 * REGOLA: nessun task dichiara code proprie per la comunicazione inter-task.
 *         Tutte le code sono create in Queues_Init() e accessibili tramite
 *         gli handle extern dichiarati qui.
 *
 * CODE PRESENTI:
 *   - eventQueueHandle : eventi verso la FSM (tutti i task -> task_fsm)
 *
 * AGGIUNGERE UNA CODA:
 *   1. Dichiarare l'handle extern qui
 *   2. Definirlo e crearlo in queues.c -> Queues_Init()
 *   3. Includere queues.h nel task che deve usarla
 */

#ifndef APP_TASKS_QUEUES_H_
#define APP_TASKS_QUEUES_H_

#include "cmsis_os.h"
#include "fsm_events.h"

/* ========================================================================== */
/* --- HANDLE DELLE CODE --- */
/* ========================================================================== */

/*
 * Coda eventi verso la FSM.
 * Produttori: task_monitor, task_comms, task_inputs (callback GPIO)
 * Consumatore: task_fsm (unico lettore)
 * Dimensione: 16 elementi - sufficiente per burst di eventi multipli
 */
extern osMessageQueueId_t eventQueueHandle;

/* ========================================================================== */
/* --- API --- */
/* ========================================================================== */

/**
 * @brief Crea tutte le code del sistema.
 *        Da chiamare in MX_FREERTOS_Init(), prima della creazione dei task.
 */
void Queues_Init(void);

/**
 * @brief Pubblica un evento nella coda FSM senza bloccare.
 * @note  Sicura da chiamare da task e da callback GPIO (non da ISR hard).
 *        Se la coda è piena l'evento viene scartato (non critico: il monitor
 *        ripubblicherà la condizione al ciclo successivo).
 * @param event  Evento da pubblicare.
 */
void Queue_PostEvent(SysEvent_t event);

#endif /* APP_TASKS_QUEUES_H_ */
