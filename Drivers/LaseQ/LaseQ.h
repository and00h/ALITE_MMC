/*
 * LaseQ.h
 *
 * ============================================================================
 * DRIVER RS485 MASTER - STM32H723 -> LaseQ4 Slave (STM32G484)
 * ============================================================================
 *
 * Il master avvia ogni transazione:
 *   1. LaseQ_Transmit()   -> invia frame (DMA TX)
 *   2. ISR TxCplt         -> rilascia semaforo tx, avvia ricezione
 *   3. LaseQ_Receive()    -> attende frame risposta (DMA RX)
 *   4. ISR RxCplt         -> rilascia semaforo rx
 *   5. LaseQ_ParseResponse() -> valida e deserializza
 *
 * Il tipo di messaggio inviato determina il tipo di risposta attesa:
 *   MSG_CONTROL    -> risposta MSG_STATUS
 *   MSG_CONFIG_SET -> risposta MSG_CONFIG_ACK
 *   MSG_PING       -> risposta MSG_PONG
 *
 * DIPENDENZE:
 *   LaseQ.h  -->  laseq_protocol.h  (frame, payload, CRC)
 *   LaseQ.h  -X-> FreeRTOS          (i semafori sono gestiti dal layer superiore
 *                                    tramite callback, non direttamente qui)
 * ============================================================================
 */

#ifndef LASEQ_LASEQ_H_
#define LASEQ_LASEQ_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include "stm32h7xx_hal.h"
#include "laseq_protocol.h"

/* ============================================================================
 * CODICI DI RITORNO
 * ============================================================================ */
typedef enum {
    LASEQ_OK                = 0,
    LASEQ_ERR_UART          = 1,   /* Errore HAL durante TX o RX              */
    LASEQ_ERR_INVALID_RSP   = 2,   /* Frame risposta non valido (CRC, START,  */
                                   /* STOP, o MSG_TYPE inatteso)              */
    LASEQ_ERR_WRONG_SENDER  = 3,   /* Risposta da indirizzo inatteso          */
    LASEQ_ERR_CONFIG_NAK    = 4,   /* Slave ha rifiutato la configurazione    */
    LASEQ_ERR_TIMEOUT       = 5,   /* Timeout semaforo DMA TX o RX            */
} LaseQ_err_t;

/* Indirizzo RS485 default dello slave LaseQ */
#define LASEQ_SLAVE_ADDR_DEFAULT    0x02U

/*
 * Periodo di polling TX/RX verso LaseQ [ms].
 *
 * Minimo fisico a 115200 baud (trama 47 byte):
 *   TX: 47 × 10 / 115200 = 4.08ms
 *   RX: 4.08ms  → ciclo minimo ~9ms con processing
 * → 20ms garantisce margine anche con variabilità del LaseQ.
 *   Ridurre a 10ms solo se il baud rate è ≥ 230400.
 *   AMC_HEARTBEAT_PERIOD_MS deve essere multiplo intero di questo valore.
 */
#define LQ_TRANSMIT_WINDOW          20U

/*
 * Tempo di assestamento dopo l'accensione di LaseQ (LaseQSupplySet(1) in
 * LaseQ_Init()) prima del primo tentativo di trasmissione RS485.
 *
 * LaseQ4 richiede almeno 500ms dall'alimentazione per essere pronto sul
 * bus (boot della propria MCU, regolatori, init UART) — dato confermato
 * dall'elettronica LaseQ (2026-07-06). Senza questo margine, il primissimo
 * frame TX può arrivare mentre la UART/DMA di LaseQ non è ancora
 * inizializzata: essendo il protocollo a lunghezza fissa (nessuna
 * detection IDLE per resync), un frame perso/parziale in quella finestra
 * può disallineare la ricezione di LaseQ anche sui tentativi successivi,
 * anche se il frame inviato da MMC è corretto byte per byte.
 *
 * 600ms = 500ms richiesti + margine. Il delay va applicato con osDelay()
 * in TaskComms_Run() PRIMA del primo laseq_transact(), non qui: questo
 * header non ha visibilità sul task/scheduler.
 */
#define LASEQ_POWERON_SETTLE_MS     600U

/* ============================================================================
 * VARIABILI DI CONTROLLO (master -> slave, MSG_CONTROL)
 * Modificabili tramite le API pubbliche sotto.
 * ============================================================================ */
typedef struct {
    uint8_t     enable;
    uint8_t     ch_enable;
    uint8_t     gate;
    uint8_t     sw_control;
    uint8_t     analog_mode;
    uint8_t     control_state;
    uint8_t     interlock_status;
    uint8_t     clear_error;
    uint8_t     OPM;
    uint8_t     OPD;
    uint32_t    sw_current_setpoint;
} LaseQ_control_vars_t;

/* ============================================================================
 * VARIABILI DI STATO (slave -> master, MSG_STATUS)
 * Aggiornate da LaseQ_ParseResponse().
 * ============================================================================ */
typedef struct {
    uint8_t     driver_address;
    uint8_t     error_code;
    uint8_t     temperature[4];     /* 4 temp elementi attivi [°C]             */
    int8_t      temp_ambient_c;     /* SHT35 temperatura ambiente [°C]         */
    uint16_t    humidity;           /* SHT35: 0.01 %RH (es. 5000 = 50.00 %RH) */
    uint32_t    current_output[4];
    uint16_t    v_anode;
    uint8_t     enable;
    uint8_t     interlock_status;
    uint8_t     gate_status;
    uint8_t     OPM;
    uint8_t     OPD;
    uint32_t    sw_current_setpoint;
    uint8_t     hw_fault_source;    /* Dettaglio di error_code bit7 (FSM_FAULT_HW):
                                        bitmask FSM_HwFaultSource_t (fsm.h),
                                        protocollo v0x0003 (2026-07-16) */
    uint8_t     fsm_state;          /* Stato FSM dello slave (LaseQ4)          */
    char        fw_version[16];     /* Versione FW applicativo LaseQ (NUOVO
                                        0x0004, 2026-09-04), da CONFIG_ACK -
                                        stringa vuota se non ancora ricevuto
                                        un CONFIG_ACK in questa sessione, o
                                        se LaseQ gira un firmware piu' vecchio
                                        che non popola ancora questo campo:
                                        trattare come "sconosciuta", non come
                                        un errore. Vedi "GET FW" (rs485_cmd.c) */
    char        fw_build_date[18];  /* Data/ora di build LaseQ (NUOVO 0x0004),
                                        stesse condizioni/avvertenze sopra   */
} LaseQ_status_vars_t;

/* ============================================================================
 * API - INIZIALIZZAZIONE
 * ============================================================================ */

/**
 * @brief  Inizializza il driver RS485 master.
 *
 * @param  huart         Handle UART (DMA TX+RX configurato).
 * @param  slave_addr    Indirizzo RS485 dello slave da interrogare.
 * @param  on_tx_cplt    Callback ISR chiamata a TX completato
 *                       (tipicamente: xSemaphoreGiveFromISR).
 * @param  on_rx_cplt    Callback ISR chiamata a RX completato.
 */
void LaseQ_Init(UART_HandleTypeDef *huart,
                uint8_t             slave_addr,
                void (*on_tx_cplt)(void),
                void (*on_rx_cplt)(void));

/* ============================================================================
 * API - CONTROLLO LASER
 * Modificano le variabili di controllo interne.
 * Il nuovo valore viene inviato alla prossima chiamata di LaseQ_Transmit().
 * ============================================================================ */
void EnableLaseQ(void);
void DisableLaseQ(void);

/**
 * @brief  Gate laser. In FSM_MODE_SW/HYBRID il gate REALE resta il pin
 *         fisico nGATE_MC (BoardCtrl_GateMC_Open()/Close(), QCW_Start()/
 *         Stop()/QCW_TimerCallback() — collegato direttamente al
 *         regolatore di LaseQ): 'status' NON viene più propagato al campo
 *         RS485 s_ctrl.gate (MSG_CONTROL.gate), che resta sempre 0 — vedi
 *         commento in LaseQ.c per la motivazione (gate immediato via pin,
 *         non soggetto alla latenza del ciclo RS485/LQ_TRANSMIT_WINDOW).
 *         'status' è usato solo per il path hardware ANALOG/HYBRID
 *         (HWGateSet()), invariato.
 */
void LaseQGate(bool status);
void LaseQClearErr(void);
void LaseQSetMode(LaseQ_mode_t mode);
void LaseQSetCurrent(uint32_t current_ma);
void LaseQSetChannels(uint8_t ch_mask);

/* ============================================================================
 * API - TRANSAZIONE RS485
 * ============================================================================ */

/**
 * @brief  Costruisce e invia un frame MSG_CONTROL via DMA.
 *         Ritorna immediatamente; la ISR TxCplt rilascia il semaforo tx.
 */
LaseQ_err_t LaseQ_Transmit(void);

/**
 * @brief  Costruisce e invia un frame MSG_CONFIG_SET via DMA.
 *
 * @param  cfg  Puntatore al payload di configurazione da inviare.
 */
LaseQ_err_t LaseQ_TransmitConfig(const LaseQ_PayloadConfigSet_t *cfg);

/**
 * @brief  Costruisce e invia un frame MSG_PING via DMA.
 *
 * @param  seq  Numero di sequenza (verrà echeggiato nel PONG).
 */
LaseQ_err_t LaseQ_TransmitPing(uint16_t seq);

/**
 * @brief  Avvia la ricezione DMA di un frame risposta.
 *         Chiamare dopo aver ricevuto il semaforo tx (TX completato).
 */
LaseQ_err_t LaseQ_Receive(void);

/**
 * @brief  Valida e deserializza il frame ricevuto.
 *         Aggiorna le variabili di stato interne se MSG_STATUS.
 *         Aggiorna il risultato config se MSG_CONFIG_ACK.
 *
 * @param  expected_type  Tipo di messaggio atteso in risposta.
 * @retval LASEQ_OK, LASEQ_ERR_INVALID_RSP, LASEQ_ERR_WRONG_SENDER,
 *         LASEQ_ERR_CONFIG_NAK.
 */
LaseQ_err_t LaseQ_ParseResponse(LaseQ_MsgType_t expected_type);

/* ============================================================================
 * API - LETTURA STATO
 * ============================================================================ */

/**
 * @brief  Restituisce una copia delle ultime variabili di stato ricevute.
 */
LaseQ_status_vars_t GetLaseQStatus(void);

/**
 * @brief  Restituisce una copia delle variabili di controllo correnti
 *         (quello che MMC sta attualmente comandando a LaseQ — enable,
 *         ch_enable, gate, sw_control/analog_mode, setpoint richiesto —
 *         non necessariamente ancora confermato dallo slave, per quello
 *         vedi GetLaseQStatus()). Usata per diagnostica RS485 ("GET LQ",
 *         rs485_cmd.c) per mostrare ad es. quali canali sono comandati
 *         attivi (ch_enable).
 */
LaseQ_control_vars_t GetLaseQControl(void);

/* ============================================================================
 * API - CALLBACK ISR (da chiamare da stm32h7xx_it.c o override HAL weak)
 * ============================================================================ */
void LaseQ_TxCpltCallback(UART_HandleTypeDef *huart);
void LaseQ_RxCpltCallback(UART_HandleTypeDef *huart);

/**
 * @brief  Da chiamare da HAL_UART_ErrorCallback() quando huart->Instance == USART10.
 *         Non chiamare da codice applicativo.
 *
 *         ORE/FE/NE/PE: l'HAL abortisce già il transfer in corso
 *         (UART_IRQHandler chiama UART_EndTxTransfer()/UART_EndRxTransfer()
 *         prima di invocare questa callback) prima di rimettere huart in
 *         stato Ready. Rilasciamo entrambi i semafori (tx/rx) per sbloccare
 *         subito l'eventuale attesa pendente in laseq_transact(), invece di
 *         aspettare il timeout pieno (stesso schema di AMC_ITErrorCallback).
 */
void LaseQ_ErrorCallback(UART_HandleTypeDef *huart);

/* ============================================================================
 * API - GPIO HARDWARE (implementazione in LaseQ.c, usa pin CubeMX)
 * ============================================================================ */
void LaseQSupplySet(bool status);
void HWGateSet(bool status);
void HWEnableSet(bool status);
void LaseQInterlockSet(bool status);

#ifdef __cplusplus
}
#endif

#endif /* LASEQ_LASEQ_H_ */
