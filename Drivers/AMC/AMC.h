/*
 * AMC.h
 *
 * Driver comunicazione MMC → AMC (Analog Module Controller).
 *
 * RESPONSABILITÀ:
 *   Gestione del heartbeat periodico verso il modulo AMC (STM32G473).
 *   Il modulo AMC genera il setpoint analogico per LaseQ in modalità HYBRID.
 *   Questo driver implementa il lato MMC (master) del protocollo AMC_protocol.h.
 *
 * NOTA HW - UART:
 *   USART6, full-duplex, no hardware flow control.
 *   Handle: huart6 (dichiarato in hal_handles.h).
 *
 * NOTA HW - GPIO INTER-MCU (MMC <-> AMC):
 *   PE11 (nHW_SETPOINT_EN): MMC -> AMC.  Linea di notifica HW (attualmente non utilizzata).
 *           Quando HIGH (assert): abilita l'uscita DAC dell'AMC sul path analogico di LaseQ.
 *           E' il meccanismo hardware complementare al protocollo UART: la FSM abilita
 *           il setpoint HW via PE11 e il protocollo coordina la modalita' operativa.
 *   PE10  (AMC_FAULT_N):    AMC -> MMC.  Segnale open-drain, attivo basso.
 *           L'AMC abbassa PE10 in caso di fault interno (sovratensione DAC, errore I2C, ecc.).
 *           Il MMC deve configurare PE10 come input con pull-up e collegare l'EXTI
 *           per rilevare il falling edge -> postare SYS_AMC_FAULT_EVENT alla FSM.
 *           Da implementare in stm32h7xx_it.c e task_inputs.c quando il firmware AMC sara' definito.
 *
 * UTILIZZO:
 *   AMC_Init() e' chiamata da Sys_HwInit() (task Monitor, priorita' piu'
 *   alta), prima che il task AMC dedicato (task_amc.c, NUOVO 2026-07-22,
 *   sostituisce il vecchio servicing dentro task_comms.c) faccia il primo
 *   giro. In task_amc, ogni AMC_HEARTBEAT_PERIOD_MS (ora 20ms, allineato a
 *   LQ_TRANSMIT_WINDOW):
 *     AMC_Transact(laser_mode, hw_setpoint_sel, qcw_active);
 *
 * NOTA COMUNICAZIONE NON BLOCCANTE (2026-07-02):
 *   Tutte le transazioni usano HAL_UART_Transmit_IT()/HAL_UART_Receive_IT()
 *   sincronizzate tramite semafori CMSIS-RTOS2, invece delle precedenti
 *   HAL_UART_Transmit()/HAL_UART_Receive() bloccanti. Motivo: con le
 *   chiamate bloccanti, il task chiamante (TaskComms) restava fermo dentro
 *   l'HAL fino a AMC_RESPONSE_TIMEOUT_MS ogni volta che AMC non rispondeva,
 *   impedendo allo scheduler di far girare altri task nel frattempo
 *   (sintomo osservato: fade del LED di stato non più fluido). Con IT il
 *   task si blocca solo sul semaforo (osSemaphoreAcquire), cedendo la CPU.
 *   Richiede il dispatch delle callback HAL_UART_TxCpltCallback/
 *   RxCpltCallback/ErrorCallback per huart6 verso AMC_ITTxCallback/
 *   AMC_ITRxCallback/AMC_ITErrorCallback (vedi stm32h7xx_it.c).
 */

#ifndef DRIVERS_AMC_AMC_H_
#define DRIVERS_AMC_AMC_H_

#include <stdbool.h>
#include <stdint.h>
#include "stm32h7xx_hal.h"
#include "AMC_protocol.h"

/* ============================================================================
 * GPIO AMC_FAULT_N (PE10) — da definire in CubeMX con label "AMC_FAULT_N"
 * Quando CubeMX genera le define, rimuovere il fallback e abilitare la guard.
 * ============================================================================ */
#ifdef AMC_FAULT_N_Pin
#  define AMC_FAULT_GPIO_READY   1
#else
/* Placeholder: finché il pin non è configurato in CubeMX la funzione restituisce false */
#  define AMC_FAULT_GPIO_READY   0
#  define AMC_FAULT_N_GPIO_Port  GPIOE
#  define AMC_FAULT_N_Pin        GPIO_PIN_10
#endif

/* ============================================================================
 * CODICI DI RITORNO
 * ============================================================================ */
typedef enum {
    AMC_OK              = 0,
    AMC_ERR_UART        = 1,
    AMC_ERR_TIMEOUT     = 2,
    AMC_ERR_INVALID_RSP = 3,
    AMC_ERR_DISABLED    = 4,   /**< Init chiamato con handle NULL */
} AMC_err_t;

/* ============================================================================
 * API
 * ============================================================================ */

/**
 * @brief  Inizializza il driver AMC.
 * @param  huart  Handle UART per la comunicazione con AMC.
 *                Passare NULL per disabilitare (comunicazione non attiva).
 */
void AMC_Init(UART_HandleTypeDef *huart);

/**
 * @brief  Invia un frame HEARTBEAT e riceve la risposta STATUS.
 *         Blocca fino a risposta o timeout (AMC_RESPONSE_TIMEOUT_MS).
 *
 * @param  laser_mode       Modalità operativa corrente (FSM_Mode_t cast a uint8_t).
 * @param  hw_setpoint_sel  0=EXT analog, 1=AMC DAC.
 * @param  qcw_active       1 se la modalità QCW è correntemente attiva lato
 *                          MMC (NUOVO 2026-07-22, vedi banner in
 *                          AMC_protocol.h su AMC_PayloadHeartbeat_t.qcw_active).
 * @param  setpoint_hw_enabled  1 se il toggle "SETPOINT HW" è attivo
 *                          (FSM_GetSetpointHwEnabled(), NUOVO 2026-07-22 —
 *                          vedi banner in AMC_protocol.h su
 *                          AMC_PayloadHeartbeat_t.setpoint_hw_enabled: NON
 *                          è lo stesso valore di hw_setpoint_sel sopra).
 * @retval AMC_OK se il frame STATUS ricevuto è valido.
 */
AMC_err_t AMC_Transact(uint8_t laser_mode, uint8_t hw_setpoint_sel, uint8_t qcw_active,
                       uint8_t setpoint_hw_enabled);

/**
 * @brief  Legge il GPIO PE10 (AMC_FAULT_N, open-drain attivo basso).
 *         Chiamare periodicamente in polling — EXTI10 è occupato.
 * @retval true se AMC segnala un fault interno (PE10 basso).
 *
 * @note   Il GPIO deve essere configurato in CubeMX come input pull-up
 *         con label "AMC_FAULT_N". Finché CubeMX non genera le define,
 *         la funzione restituisce sempre false (guard AMC_FAULT_GPIO_READY).
 */
bool AMC_ReadFaultPin(void);

/**
 * @brief  Invia la configurazione PSU all'AMC (MSG_CONFIG_SET) e attende ACK.
 *         Da chiamare una volta all'avvio, dopo il primo heartbeat valido.
 *         In caso di service/calibrazione può essere richiamata.
 *
 * @param  psu_voltage_limit_mv     Tensione massima PSU [mV] (Vmax, "SET PSU VOLTAGE").
 * @param  psu_current_limit_ma     Corrente massima PSU [mA].
 * @param  voltage_comp_enabled     1=compensazione dinamica tensione attiva (NUOVO 2026-07-22).
 * @param  comp_stabilize_delay_ms  Debounce dopo accensione laser [10-50ms].
 * @param  comp_ramp_duration_ms    Durata rampa lenta in discesa [50-100ms].
 * @param  comp_ramp_up_duration_ms Durata rampa veloce in salita [1-5ms].
 * @retval AMC_OK se AMC accetta la config (CONFIG_ACK.accepted == 1).
 */
AMC_err_t AMC_SendConfig(uint16_t psu_voltage_limit_mv, uint16_t psu_current_limit_ma,
                         uint8_t voltage_comp_enabled, uint8_t comp_stabilize_delay_ms,
                         uint8_t comp_ramp_duration_ms, uint8_t comp_ramp_up_duration_ms);

/**
 * @brief  Restituisce l'ultimo STATUS ricevuto dall'AMC.
 */
AMC_PayloadStatus_t AMC_GetLastStatus(void);

/**
 * @brief  Restituisce true se l'AMC è raggiungibile (ultimo transact OK).
 */
bool AMC_IsAlive(void);

/**
 * @brief  Restituisce la versione firmware di AMC letta dall'ultimo
 *         CONFIG_ACK ricevuto (AMC_PayloadConfigAck_t.fw_version, NUOVO
 *         2026-09-04, AMC_protocol.h v0x0006). Stringa vuota se AMC non ha
 *         ancora risposto a un CONFIG_SET in questa sessione, o se sta
 *         girando un firmware piu' vecchio che non popola ancora questo
 *         campo - trattare una stringa vuota come "sconosciuta", non come
 *         un errore. Usato da "GET FW" (rs485_cmd.c).
 *
 * @retval Puntatore a buffer statico interno, sempre NUL-terminated.
 */
const char *AMC_GetFwVersionStr(void);

/**
 * @brief  Invia la configurazione fotodiodi (MSG_CONFIG_PD) e attende ACK.
 *         Da chiamare all'avvio dopo AMC_SendConfig().
 */
AMC_err_t AMC_SendPDConfig(uint8_t pd_mask, uint8_t gain_windows,
                            uint16_t gain_settle_ms,
                            uint8_t stability_samples,
                            uint8_t stability_thresh);


/* ============================================================================
 * API v0.0003 — LUT GAIN e PD VALID
 * ============================================================================ */

/**
 * @brief  Invia le soglie di selezione finestra guadagno (MSG_CONFIG_GAIN_LUT).
 * @param  mode        0=SW (basato su current_ma), 1=HW (basato su pa0_adc)
 * @param  threshold   Array di 4 soglie
 */
AMC_err_t AMC_SendGainLUT(uint8_t mode, const uint16_t threshold[4]);

/**
 * @brief  Invia una entry della LUT di validazione fotodiodi (MSG_CONFIG_PD_VALID).
 *         Le entry DEVONO essere inviate in ordine crescente di setpoint.
 * @param  pd_idx       0-3
 * @param  mode         0=SW, 1=HW
 * @param  entry_idx    Indice entry (0-based)
 * @param  total        Numero totale entry per questo PD+mode
 * @param  setpoint     current_ma (SW) o pa0_adc (HW)
 * @param  pd_min       ADC minimo atteso
 * @param  pd_max       ADC massimo atteso
 */
AMC_err_t AMC_SendPDValidEntry(uint8_t pd_idx, uint8_t mode,
                               uint8_t entry_idx, uint8_t total,
                               uint16_t setpoint, uint16_t pd_min, uint16_t pd_max);

/**
 * @brief  Invia una entry della LUT di compensazione tensione PSU, mA->mV
 *         (MSG_CONFIG_VOLTAGE_LUT, NUOVO 2026-07-22). Le entry DEVONO
 *         essere inviate in ordine crescente di current_ma — vedi
 *         LUT_VoltageCompEntry_t (lut_manager.h) e psu_voltage_comp.h (AMC).
 * @param  entry_idx    Indice entry (0-based)
 * @param  total        Numero totale entry di questa LUT
 * @param  current_ma   Setpoint di corrente LaseQ [mA]
 * @param  voltage_mv   Tensione target PSU [mV], 0 = entry non valida
 */
AMC_err_t AMC_SendVoltageCompLUTEntry(uint8_t entry_idx, uint8_t total,
                                      uint16_t current_ma, uint16_t voltage_mv);

/**
 * @brief  Aggiorna il HEARTBEAT con il setpoint corrente in modalita' SW.
 *         Chiamare prima di AMC_Transact() ad ogni ciclo in SW mode.
 * @param  current_ma  Corrente comandata a LaseQ [mA]
 */
void AMC_SetCurrentSetpoint(uint16_t current_ma);

/**
 * @brief  Richiede ad AMC (protocollo v0.0005, campo
 *         voltage_restart_request) di trattare il prossimo aumento di
 *         current_setpoint_ma come se il laser venisse riacceso da zero
 *         ai fini della compensazione dinamica di tensione PSU — vedi
 *         AMC_protocol.h, banner AMC_PayloadHeartbeat_t e "VERSIONE
 *         HISTORY" 0x0005, per la motivazione completa (spunto di
 *         corrente/drop di tensione su un aumento di setpoint a laser
 *         gia' acceso, non solo alla vera accensione).
 *
 *         Da chiamare da task_comms.c nello stesso istante in cui si
 *         (ri)arma l'attesa VCOMP_SETPOINT_HOLD_CYCLES (vedi quel file),
 *         cioe' non appena si rileva un aumento del setpoint desiderato
 *         con compensazione attiva. One-shot: il flag viene incluso nel
 *         PROSSIMO heartbeat inviato da AMC_Transact() e poi azzerato
 *         automaticamente — non richiede di essere richiamato ad ogni
 *         ciclo ne' di essere esplicitamente disarmato dal chiamante.
 */
void AMC_RequestVoltageRestart(void);

/**
 * @brief  Restituisce i valori ADC grezzi dei 4 fotodiodi dall'ultimo STATUS.
 * @param  out_raw  Array di output [4] (valori ADC 12-bit).
 */
void AMC_GetPDRaw(uint16_t out_raw[4]);

/**
 * @brief  Restituisce la finestra di guadagno attiva dall'ultimo STATUS.
 * @retval gain_window (0-3)
 */
uint8_t AMC_GetGainWindow(void);

/**
 * @brief  true se AMC segnala (pd_status_mask bit6, protocollo v0.0005+,
 *         NUOVO 2026-07-28) che pd_raw[] nell'ultimo STATUS non e' stato
 *         aggiornato da oltre 500ms lato AMC — vedi Photodiode_IsStale()/
 *         PD_STALE_TIMEOUT_MS, AMC/App/Photodiode/photodiode.h.
 *         Rete di sicurezza applicativa: distingue "i fotodiodi leggono
 *         davvero questo valore" da "AMC continua a rispondere ma
 *         l'acquisizione ADC1 e' bloccata" (bug corretto il 2026-07-28,
 *         DMAContinuousRequests in MX_ADC1_Init/MX_ADC2_Init — questa
 *         funzione resta comunque utile per rilevare un'eventuale
 *         ricomparsa del problema, es. per un guasto hardware reale).
 *         false anche se AMC non e' raggiungibile (AMC_IsAlive()==false):
 *         in quel caso il problema e' gia' segnalato da FAULT_BIT_AMC.
 */
bool AMC_IsPDDataStale(void);

/* ============================================================================
 * CALLBACK ISR (uso interno — dispatch da stm32h7xx_it.c per huart6)
 * ============================================================================ */

/**
 * @brief  Da chiamare da HAL_UART_TxCpltCallback() quando huart->Instance == USART6.
 *         Non chiamare da codice applicativo.
 */
void AMC_ITTxCallback(UART_HandleTypeDef *huart);

/**
 * @brief  Da chiamare da HAL_UART_RxCpltCallback() quando huart->Instance == USART6.
 *         Non chiamare da codice applicativo.
 */
void AMC_ITRxCallback(UART_HandleTypeDef *huart);

/**
 * @brief  Da chiamare da HAL_UART_ErrorCallback() quando huart->Instance == USART6.
 *         Non chiamare da codice applicativo.
 */
void AMC_ITErrorCallback(UART_HandleTypeDef *huart);

#endif /* DRIVERS_AMC_AMC_H_ */
