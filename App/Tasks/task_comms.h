/*
 * task_comms.h
 *
 * Task di comunicazione con le periferiche remote via RS485.
 *
 * RESPONSABILITÀ:
 *   - Connessione iniziale al driver LaseQ (LaseQ_Connect)
 *   - Polling ciclico TX/RX verso LaseQ ogni LQ_TRANSMIT_WINDOW ms
 *   - Pubblicazione di SYS_LASEQ_FAULT_EVENT in caso di timeout/errore
 *   - Gestione dei semafori DMA (TX e RX) per la sincronizzazione con HAL
 *
 * PRIORITÀ: osPriorityNormal2 (bassa: comunicazione non è safety-critical
 *           come il monitor, ma deve girare regolarmente)
 *
 * PERIODICITÀ: LQ_TRANSMIT_WINDOW ms (definito in LaseQ.h, default 5ms)
 *
 * SEMAFORI:
 *   sem_laseq_tx e sem_laseq_rx sono privati di questo task.
 *   Le callback HAL li rilasciano tramite i function pointer passati a LaseQ_Init().
 *   Non sono esposti all'esterno: nessun altro task li tocca.
 */

#ifndef APP_TASKS_TASK_COMMS_H_
#define APP_TASKS_TASK_COMMS_H_

#include <stdint.h>
#include "cmsis_os.h"

/* Attributi del task - usati in freertos.c */
extern const osThreadAttr_t taskComms_attr;

/**
 * @brief Inizializza i semafori DMA e registra le callback nel driver LaseQ.
 *        Da chiamare in MX_FREERTOS_Init() prima di osKernelStart(),
 *        dopo Queues_Init() e prima di osThreadNew().
 */
void TaskComms_Init(void);

/**
 * @brief Entry point del task di comunicazione.
 *        Da passare a osThreadNew() in MX_FREERTOS_Init().
 */
void TaskComms_Run(void *arg);

/*
 * NOTA 2026-07-22: TaskComms_GetPDPower()/TaskComms_RequestPSUConfigResend()
 * sono state rinominate TaskAmc_GetPDPower()/TaskAmc_RequestPSUConfigResend()
 * e spostate in task_amc.h, insieme a tutto il resto del servicing AMC
 * (scorporato in un task dedicato — vedi banner in cima a task_comms.c).
 */

#endif /* APP_TASKS_TASK_COMMS_H_ */
