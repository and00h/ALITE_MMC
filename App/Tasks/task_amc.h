/*
 * task_amc.h
 *
 * Task dedicato alla comunicazione MMC -> AMC (Analog Module Controller,
 * STM32G473) via USART6. NUOVO 2026-07-22, scorporato da task_comms.c
 * (che fino a questa data serviva AMC ogni AMC_LOOP_DIVISOR cicli LaseQ,
 * cioe' ogni 200ms, con un contatore software).
 *
 * PERCHE' UN TASK DEDICATO:
 *   La compensazione dinamica di tensione PSU (vedi psu_voltage_comp.c
 *   lato AMC) impone un vincolo di sequenza: quando il setpoint di
 *   corrente verso LaseQ AUMENTA, AMC deve prima alzare la tensione PSU
 *   (rampa ascendente, g_config.comp_ramp_up_duration_ms = 1-5ms) e SOLO
 *   DOPO il nuovo setpoint, piu' alto, puo' essere inoltrato a LaseQ —
 *   altrimenti il regolatore lineare puo' saturare/andare in dropout
 *   durante il transitorio di corrente. Con AMC servito ogni 200ms questa
 *   finestra era troppo larga per essere utile in pratica: un task
 *   dedicato con periodo LQ_TRANSMIT_WINDOW (20ms, lo stesso di LaseQ,
 *   vedi task_comms.c) rende il servicing AMC abbastanza frequente da
 *   garantire che la rampa (1-5ms) sia gia' completa entro il ciclo
 *   successivo di task_comms.c, che e' dove vive il gating vero e proprio
 *   (confronto tra Setpoint_GetCurrentMa() e
 *   GetLaseQStatus().sw_current_setpoint, vedi task_comms.c).
 *
 * RESPONSABILITA':
 *   - Sequenza di configurazione all'avvio: CONFIG_SET (limiti PSU + flag/
 *     timing compensazione tensione), CONFIG_PD, CONFIG_GAIN_LUT x2 (SW/HW),
 *     CONFIG_PD_VALID (4 PD x N entry x 2 modi), CONFIG_VOLTAGE_LUT x N
 *     (LUT compensazione tensione, NUOVO 2026-07-22)
 *   - Heartbeat/status periodico (AMC_Transact) ogni LQ_TRANSMIT_WINDOW ms
 *   - Polling PE10 (AMC_FAULT_N, fault hardware AMC)
 *   - Conversione PD raw -> Watt tramite LUT (TaskAmc_GetPDPower())
 *   - Re-invio a richiesta RS485 (senza reboot) dei limiti PSU/flag di
 *     compensazione (TaskAmc_RequestPSUConfigResend()) e della LUT di
 *     compensazione tensione (TaskAmc_RequestVoltageCompLUTResend())
 *
 * PRIORITA': osPriorityNormal2 (stessa di TaskComms_Run: comunicazione RS485
 *            non safety-critical come il monitor, ma deve girare
 *            regolarmente e con bassa latenza per il vincolo di sequenza
 *            sopra).
 *
 * PERIODICITA': LQ_TRANSMIT_WINDOW ms (20ms, definito in LaseQ.h). La
 *               costante di protocollo AMC_HEARTBEAT_PERIOD_MS (ora anch'essa
 *               20ms, vedi AMC_protocol.h) resta per documentare il
 *               protocollo condiviso con AMC, ma il periodo reale del loop
 *               e' pilotato da LQ_TRANSMIT_WINDOW (vedi _Static_assert in
 *               task_amc.c).
 *
 * PROPRIETA' UART:
 *   huart6 (USART6 -> AMC) e' di proprieta' esclusiva di questo task tramite
 *   il driver AMC.c (che possiede i propri semafori interni, creati da
 *   AMC_Init() — vedi AMC.c). Nessun altro task/modulo deve chiamare le
 *   funzioni AMC_Send*()/AMC_Transact() direttamente (stesso principio gia'
 *   applicato a huart10/LaseQ in task_comms.c e a huart1/COM interface con
 *   s_uart_mutex in rs485_cmd.c).
 *
 * NESSUNA TaskAmc_Init() DEDICATA:
 *   AMC_Init() crea gia' i propri semafori internamente (vedi AMC.c, guard
 *   "s_sem_tx == NULL") ed e' chiamata da Sys_HwInit() (task Monitor,
 *   priorita' piu' alta, garantito eseguito prima del primo giro di questo
 *   task). Diversamente da LaseQ (task_comms.c possiede sem_tx/sem_rx
 *   esterni al driver), qui non serve alcuna inizializzazione lato task.
 */

#ifndef APP_TASKS_TASK_AMC_H_
#define APP_TASKS_TASK_AMC_H_

#include <stdint.h>
#include "cmsis_os.h"

/* Attributi del task - usati in freertos.c */
extern const osThreadAttr_t taskAmc_attr;

/**
 * @brief Entry point del task AMC. Da passare a osThreadNew() in
 *        MX_FREERTOS_Init(), DOPO TaskComms_Init() (ordine non critico:
 *        i due task non condividono stato RTOS, solo g_config in lettura).
 */
void TaskAmc_Run(void *arg);

/**
 * @brief  Restituisce la potenza [W] calcolata per i 4 fotodiodi.
 *         Aggiornata ad ogni AMC_Transact() riuscito tramite la LUT.
 *         (rinominata da TaskComms_GetPDPower() il 2026-07-22, il calcolo
 *         si e' spostato qui insieme al resto del servicing AMC)
 * @param  out_w  Array di output [4] in Watt (media mobile).
 */
void TaskAmc_GetPDPower(uint16_t out_w[4]);

/**
 * @brief  Richiede il re-invio ad AMC dei limiti PSU (g_config.psu_voltage_mv/
 *         psu_current_ma) e dei parametri di compensazione tensione
 *         (voltage_comp_enabled, comp_stabilize_delay_ms, comp_ramp_duration_ms,
 *         comp_ramp_up_duration_ms) tramite AMC_SendConfig(), senza attendere
 *         un reboot. (rinominata da TaskComms_RequestPSUConfigResend() il
 *         2026-07-22, la logica si e' spostata qui insieme al resto del
 *         servicing AMC)
 *
 *         Non bloccante: imposta un flag, applicato al prossimo ciclo di
 *         questo task (max LQ_TRANSMIT_WINDOW = 20ms). Il chiamante
 *         (tipicamente rs485_cmd.c dopo "SET PSU ..."/"SET VCOMP ...") e'
 *         responsabile di aver gia' verificato che lo stato FSM sia sicuro
 *         (SYS_IDLE, PSU spento) prima di chiamare questa funzione.
 */
void TaskAmc_RequestPSUConfigResend(void);

/**
 * @brief  Richiede il re-invio ad AMC dell'intera LUT di compensazione
 *         tensione (mA -> mV, vedi lut_manager.h/LUT_VoltageCompEntry_t)
 *         tramite AMC_SendVoltageCompLUTEntry(), una entry per volta in
 *         ordine crescente di current_ma, senza attendere un reboot.
 *         Da chiamare dopo "SET LUT VCOMP <entry> <current_ma> <voltage_mv>"
 *         (rs485_cmd.c, NUOVO 2026-07-22).
 *
 *         Non bloccante: imposta un flag, applicato al prossimo ciclo di
 *         questo task (max LQ_TRANSMIT_WINDOW = 20ms), stesso gate SYS_IDLE
 *         di TaskAmc_RequestPSUConfigResend().
 */
void TaskAmc_RequestVoltageCompLUTResend(void);

#endif /* APP_TASKS_TASK_AMC_H_ */
