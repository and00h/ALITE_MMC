/*
 * task_com_interface.h  --  MMC
 *
 * Task FreeRTOS dedicato allo scambio dati con la COM interface (SPI3),
 * separato dal loop di task_comms.c (LaseQ/AMC), che serve un protocollo
 * periodico a ciclo fisso — questo task e' invece guidato da un evento
 * asincrono e sporadico (nCOM_INT_IN, la COM interface scrive solo quando ha
 * un messaggio nuovo, non periodicamente): un'attesa bloccante indefinita su
 * un semaforo non deve condividere il ciclo di un altro protocollo.
 *
 * Vedi COM_interface.h per la sequenza completa (interrupt -> messaggio di
 * clock -> richiesta reale -> risposta) e per il significato delle due
 * callback registrate qui (OnMsgPending/OnXferCplt).
 *
 * WATCHDOG: questo task NON chiama Watchdog_Heartbeat() e non ha un proprio
 * Watchdog_TaskId_t (vedi Watchdog.h). Il task dorme indefinitamente in
 * attesa del semaforo msg-pending: un lungo periodo di silenzio della COM
 * interface e' normale (nessun traffico periodico atteso), non un sintomo
 * di blocco — registrarlo nel watchdog causerebbe un reset software del
 * sistema al primo silenzio piu' lungo di WDG_MAX_HEARTBEAT_AGE_MS (5s).
 *
 * PRIORITA'/STACK: stessa priorita' di task_comms (comunicazione hardware),
 * stack ridotto: nessuna printf/snprintf, solo chiamate a COM_interface.c
 * che usa buffer statici propri.
 */

#ifndef APP_TASKS_TASK_COM_INTERFACE_H_
#define APP_TASKS_TASK_COM_INTERFACE_H_

#include "cmsis_os.h"

/** Attributi del task (stack statico). */
extern const osThreadAttr_t taskComInterface_attr;

/**
 * @brief  Crea i semafori privati del task e inizializza il driver
 *         COM_interface (COM_Interface_Init(), con le callback interne
 *         OnMsgPending/OnXferCplt). Da chiamare da MX_FREERTOS_Init(),
 *         PRIMA di osThreadNew(TaskComInterface_Run, ...) — stesso momento/
 *         stesso motivo di TaskComms_Init() (sicuro pre-scheduler: solo
 *         creazione oggetti RTOS + salvataggio puntatori, nessuna chiamata
 *         HAL bloccante).
 */
void TaskComInterface_Init(void);

/**
 * @brief  Entry-point del task. Da passare a osThreadNew() in
 *         MX_FREERTOS_Init(). @param argument non utilizzato (passare NULL).
 */
void TaskComInterface_Run(void *argument);

#endif /* APP_TASKS_TASK_COM_INTERFACE_H_ */
