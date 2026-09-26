/*
 * Watchdog.h
 *
 * Servizio di watchdog software (heartbeat per task) sopra l'IWDG1 hardware.
 *
 * PERCHÉ NON UN SEMPLICE REFRESH PERIODICO:
 *   Un HAL_IWDG_Refresh() chiamato ciecamente da un task sempre vivo (es. il
 *   default task, che fa solo osDelay(1)) non protegge da nulla: se un task
 *   applicativo (monitor, FSM, comms, rs485...) si blocca per sempre in un
 *   loop o in attesa di un semaforo/mutex mai rilasciato, il refresh continua
 *   comunque e l'IWDG non scatta mai. Questo modulo richiede che OGNI task
 *   critico segnali il proprio "battito" (Watchdog_Heartbeat) periodicamente;
 *   solo se tutti i battiti sono recenti Watchdog_Service() rinfresca l'IWDG1.
 *   Se anche un solo task si blocca, il refresh si interrompe e l'IWDG1
 *   (timeout ~32.7s con PSC=256, Reload=4095) resetta la scheda.
 *
 * INTEGRAZIONE:
 *   - Watchdog_Init(&hiwdg1) va chiamato una volta, dopo MX_IWDG1_Init()
 *     (quindi da Sys_HwInit() o subito prima, nel contesto del primo task
 *     che parte — vedi freertos.c).
 *   - Ogni task critico chiama Watchdog_Heartbeat(WDG_TASK_xxx) una volta
 *     per ogni giro del proprio loop principale.
 *   - Watchdog_Service() va chiamato periodicamente da un contesto sempre
 *     schedulabile (qui: il default task in main.c, USER CODE 5, che gira
 *     ogni 1ms) — NON deve dipendere dalla salute degli altri task.
 *
 * NON è necessario "vedere" tutti i task fin dal primo istante: finché un
 * task non ha ancora chiamato Watchdog_Heartbeat() almeno una volta dal boot,
 * Watchdog_Service() lo ignora (altrimenti il sistema si resetterebbe durante
 * il normale avvio, prima che tutti i task abbiano fatto il primo giro).
 */

#ifndef APP_WATCHDOG_WATCHDOG_H_
#define APP_WATCHDOG_WATCHDOG_H_

#include "stm32h7xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

/* ========================================================================== */
/* --- TASK MONITORATI --- */
/* ========================================================================== */

typedef enum {
    WDG_TASK_MONITOR = 0,   /**< TaskMonitor_Run (task_monitor.c)   */
    WDG_TASK_FSM,           /**< TaskFSM_Run     (task_fsm.c)       */
    WDG_TASK_COMMS,         /**< TaskComms_Run   (task_comms.c)     */
    WDG_TASK_OUTPUTS,       /**< TaskOutputs_Run (task_outputs.c)   */
    WDG_TASK_INPUTS,        /**< TaskInputs_Run  (task_inputs.c)    */
    WDG_TASK_RS485,         /**< Task_RS485      (task_rs485.c)     */
    WDG_TASK_AMC,           /**< TaskAmc_Run     (task_amc.c), NUOVO 2026-07-22
                                  (servicing AMC scorporato da task_comms.c) */
    WDG_NUM_TASKS,
} Watchdog_TaskId_t;

/**
 * Età massima ammessa di un battito prima di considerare il task bloccato
 * e smettere di rinfrescare l'IWDG1. Margine ampio (il task più lento del
 * gruppo gira su base ~100ms) rispetto al timeout IWDG1 (~32.7s), per non
 * generare falsi positivi durante picchi di carico transitori (es. scrittura
 * flash, log su SD).
 */
#define WDG_MAX_HEARTBEAT_AGE_MS   5000U

/* ========================================================================== */
/* --- API --- */
/* ========================================================================== */

/**
 * @brief  Inizializza il modulo watchdog software e memorizza l'handle IWDG1.
 *         Da chiamare una sola volta, dopo che lo scheduler è partito.
 */
void Watchdog_Init(IWDG_HandleTypeDef *hiwdg);

/**
 * @brief  Segnala che il task indicato è vivo. Da chiamare una volta per
 *         ogni iterazione del loop principale del task.
 */
void Watchdog_Heartbeat(Watchdog_TaskId_t task);

/**
 * @brief  Rinfresca l'IWDG1 SOLO se tutti i task che hanno già inviato
 *         almeno un battito risultano vivi entro WDG_MAX_HEARTBEAT_AGE_MS.
 *         Da chiamare periodicamente da un contesto sempre schedulabile
 *         (default task). Non bloccante, nessuna I/O.
 */
void Watchdog_Service(void);

/**
 * @brief  Legge i flag di reset RCC, logga la causa dell'ultimo reset
 *         (LOG_ERROR se IWDG1/WWDG1, LOG_WARN se low-power, LOG_INFO altrimenti)
 *         e li cancella per il prossimo ciclo. Da chiamare una sola volta,
 *         dopo SysLog_Init()/SysLog_Boot() (altrimenti il log è no-op).
 */
void Watchdog_LogResetCause(void);

#endif /* APP_WATCHDOG_WATCHDOG_H_ */
