/*
 * SHT35.h
 *
 * Driver per il sensore di umidità e temperatura Sensirion SHT35.
 *
 * HARDWARE:
 *   I2C2 su PF0 (SENS_SDA) / PF1 (SENS_SCL)
 *   Indirizzo I2C: 0x44 (ADDR pin = GND) oppure 0x45 (ADDR pin = VDD)
 *
 * UTILIZZO:
 *   1. SHT35_Init(&hi2c2, wait_fn, signal_fn) una volta al boot
 *   2. Chiamare SHT35_Trigger() per avviare una misura
 *   3. Dopo almeno 15ms chiamare SHT35_Read() per leggere il risultato
 *   4. In task_monitor: trigger ogni ciclo, read al ciclo successivo
 *
 * CONVERSIONE:
 *   Temperatura [°C] = -45 + 175 * raw_T / 65535
 *   Umidità [%RH]    = 100 * raw_H / 65535
 *
 * NOTA: CRC a 8 bit su ogni coppia di byte (polinomio 0x31, init 0xFF).
 *       In caso di CRC errato la lettura viene scartata e i valori
 *       precedenti vengono mantenuti.
 *
 * DIPENDENZE:
 *   SHT35.h  -X->  FreeRTOS/CMSIS-RTOS. Il driver avvia le transazioni I2C in
 *   modalità IT ma non crea né usa semafori/mutex: l'attesa "che non blocchi
 *   lo scheduler" e la segnalazione del completamento sono delegate al layer
 *   applicativo tramite i function pointer passati a SHT35_Init() (stesso
 *   pattern già usato da LaseQ.h/task_comms.c per la UART verso LaseQ4).
 */

#ifndef DRIVERS_SHT35_SHT35_H_
#define DRIVERS_SHT35_SHT35_H_

#include <stdint.h>
#include <stdbool.h>
#include "stm32h7xx_hal.h"

/* ========================================================================== */
/* --- CONFIGURAZIONE --- */
/* ========================================================================== */

/** Indirizzo I2C del SHT35 (7-bit, senza bit R/W) */
#define SHT35_I2C_ADDR      0x44U

/** Timeout I2C [ms] */
#define SHT35_I2C_TIMEOUT   50U

/** Valore sentinella per dato non disponibile */
#define SHT35_INVALID_TEMP  INT16_MIN
#define SHT35_INVALID_HUM   INT16_MIN

/* ========================================================================== */
/* --- DATI --- */
/* ========================================================================== */

typedef struct {
    int16_t temperature_cdeg;   /**< Temperatura in centidegree Celsius (es. 2350 = 23.50°C) */
    int16_t humidity_cpct;      /**< Umidità relativa in centipercent (es. 6520 = 65.20%RH)  */
    bool    valid;              /**< true se l'ultimo ciclo read ha prodotto dati CRC-validi  */
} SHT35_Data_t;

/* ========================================================================== */
/* --- SINCRONIZZAZIONE (fornita dal layer applicativo) --- */
/* ========================================================================== */

/**
 * @brief  Attende il completamento di una transazione I2C IT in corso, senza
 *         bloccare lo scheduler (tipicamente osSemaphoreAcquire).
 * @param  timeout_ms  Timeout in millisecondi.
 * @retval true se il completamento è stato segnalato entro il timeout,
 *         false in caso di timeout.
 */
typedef bool (*SHT35_WaitFn_t)(uint32_t timeout_ms);

/**
 * @brief  Segnala il completamento (successo o errore) di una transazione
 *         I2C IT. Chiamata dal driver in contesto ISR (tipicamente
 *         osSemaphoreRelease).
 */
typedef void (*SHT35_SignalFn_t)(void);

/* ========================================================================== */
/* --- API PUBBLICA --- */
/* ========================================================================== */

/**
 * @brief  Inizializza il driver. Associa il bus I2C.
 * @param  hi2c      Puntatore all'handle I2C (tipicamente &hi2c2).
 * @param  wait_fn   Funzione applicativa di attesa (vedi SHT35_WaitFn_t).
 * @param  signal_fn Funzione applicativa di segnalazione (vedi SHT35_SignalFn_t).
 */
void SHT35_Init(I2C_HandleTypeDef *hi2c, SHT35_WaitFn_t wait_fn, SHT35_SignalFn_t signal_fn);

/**
 * @brief  Invia il comando di avvio misura (single-shot, alta ripetibilità).
 *         Non blocca: il sensore impiegherà ~15ms per completare la misura.
 * @retval true se il comando è stato inviato correttamente, false altrimenti.
 */
bool SHT35_Trigger(void);

/**
 * @brief  Legge i 6 byte di risultato dal sensore e aggiorna i dati interni.
 *         Da chiamare almeno 15ms dopo SHT35_Trigger().
 *         In caso di errore CRC il campo valid viene messo a false.
 * @retval true se la lettura ha avuto successo e il CRC è corretto.
 */
bool SHT35_Read(void);

/**
 * @brief  Restituisce gli ultimi dati validi letti.
 */
SHT35_Data_t SHT35_GetData(void);

/**
 * @brief  Restituisce l'umidità corrente in percentuale intera [%RH].
 *         Restituisce -1 se il dato non è disponibile.
 */
int8_t SHT35_GetHumidityPct(void);

/* ========================================================================== */
/* --- CALLBACK IT (da chiamare SOLO dal dispatcher in stm32h7xx_it.c) --- */
/* ========================================================================== */

/**
 * @brief  Da chiamare da HAL_I2C_MasterTxCpltCallback quando hi2c->Instance==I2C2.
 *         Invoca signal_fn (fornita a SHT35_Init) per sbloccare SHT35_Trigger().
 */
void SHT35_ITTxCallback(void);

/**
 * @brief  Da chiamare da HAL_I2C_MasterRxCpltCallback quando hi2c->Instance==I2C2.
 *         Invoca signal_fn per sbloccare SHT35_Read().
 */
void SHT35_ITRxCallback(void);

/**
 * @brief  Da chiamare da HAL_I2C_ErrorCallback quando hi2c->Instance==I2C2.
 *         Marca l'errore e invoca signal_fn per sbloccare la funzione in
 *         attesa (Trigger o Read).
 */
void SHT35_ITErrorCallback(void);

#endif /* DRIVERS_SHT35_SHT35_H_ */
