/*
 * AD7490.h
 *
 * Driver per l'ADC esterno AD7490WBCSZ (16 canali, SPI, 12 bit).
 *
 * HARDWARE:
 *   SPI5 (master, CPOL=0, CPHA=1, MSB first)
 *   PF6  = TEMP_ADC_nCS   (GPIO output, CS manuale)
 *   PF7  = TEMP_ADC_SCK   (SPI5_SCK)
 *   PF8  = TEMP_ADC_DOUT  (SPI5_MISO)
 *   PF9  = TEMP_ADC_DIN   (SPI5_MOSI)
 *
 * PROTOCOLLO AD7490 (semplificato, pipeline ADC):
 *   Il chip è un ADC pipeline: la conversione avviene durante il trasferimento
 *   SPI successivo. Per leggere un canale occorrono due transazioni:
 *     1a: invio control word per canale N → il chip avvia la conversione
 *     2a: invio dummy (o control word per canale N+1) → si riceve il risultato di N
 *
 *   Per scansionare tutti i 16 canali in sequenza:
 *     - Transazione 0: invio CTRL(CH0) → scarta risposta (garbage prima di reset)
 *     - Transazione 1: invio CTRL(CH1) → riceve risultato CH0
 *     - ...
 *     - Transazione 16: invio dummy   → riceve risultato CH15
 *
 *   AD7490_ScanAll() esegue questa sequenza e popola un array di 16 valori.
 *
 * CONTROL WORD (16 bit, MSB first) — verificata contro datasheet Analog
 * Devices AD7490 Rev. E, Table 9 (2026-07-06; una versione precedente di
 * questa mappa era sbagliata: mancava il bit SEQ e tutto il registro dopo
 * WRITE era sfasato di una posizione, con l'effetto netto di mettere il
 * chip in Full Shutdown a ogni transazione — vedi commento in AD7490.c):
 *   [15]    : WRITE = 1 (per attivare il control register)
 *   [14]    : SEQ = 0 (nessun sequencer)
 *   [13:10] : ADD[3:0] = indirizzo canale (0..15)
 *   [9]     : PM1
 *   [8]     : PM0  (PM[1:0] = 11 → normal operation)
 *   [7]     : SHADOW = 0
 *   [6]     : WEAK/TRI = 0 (DOUT torna a three-state a fine transfer)
 *   [5]     : RANGE = 1 (0V to Vref, unipolare)
 *   [4]     : CODING = 1 (straight binary, non complemento a 2)
 *   [3:0]   : don't care (0)
 *
 * RISPOSTA (16 bit):
 *   [15:12] : ADD[3:0] del canale convertito (riflesso, utile per verifica)
 *   [11:0]  : risultato 12 bit (0..4095)
 *
 * UTILIZZO:
 *   AD7490_Init(&hspi5, wait_fn, signal_fn);
 *
 *   uint16_t ntc_raw[AD7490_NUM_CHANNELS];
 *   AD7490_ScanAll(ntc_raw);
 *   // ntc_raw[0]  = NTC1, ntc_raw[1] = NTC2, ..., ntc_raw[15] = NTC16
 *
 * DIPENDENZE:
 *   AD7490.h  -X->  FreeRTOS/CMSIS-RTOS. Il driver avvia le transazioni SPI in
 *   modalità IT ma non crea né usa semafori/mutex: l'attesa "che non blocchi
 *   lo scheduler" e la segnalazione del completamento sono delegate al layer
 *   applicativo tramite i function pointer passati a AD7490_Init() (stesso
 *   pattern già usato da LaseQ.h/task_comms.c per la UART verso LaseQ4).
 */

#ifndef DRIVERS_TEMPSENSOR_AD7490_H_
#define DRIVERS_TEMPSENSOR_AD7490_H_

#include "stm32h7xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

/* ========================================================================== */
/* --- COSTANTI --- */
/* ========================================================================== */

#define AD7490_NUM_CHANNELS     16U    /**< Canali ADC disponibili */
#define AD7490_MAX_VALUE        4095U  /**< Risultato massimo (12 bit) */

/* ========================================================================== */
/* --- CODICI DI RITORNO --- */
/* ========================================================================== */

typedef enum {
    AD7490_OK          =  0,
    AD7490_ERR_SPI     = -1,   /**< Errore durante la transazione SPI          */
    AD7490_ERR_TIMEOUT = -2,   /**< Timeout SPI                                */
    AD7490_ERR_INIT    = -3,   /**< Driver non inizializzato                   */
} AD7490_err_t;

/* ========================================================================== */
/* --- SINCRONIZZAZIONE (fornita dal layer applicativo) --- */
/* ========================================================================== */

/**
 * @brief  Attende il completamento di una transazione SPI IT in corso, senza
 *         bloccare lo scheduler (tipicamente osSemaphoreAcquire).
 * @param  timeout_ms  Timeout in millisecondi.
 * @retval true se il completamento è stato segnalato entro il timeout,
 *         false in caso di timeout.
 */
typedef bool (*AD7490_WaitFn_t)(uint32_t timeout_ms);

/**
 * @brief  Segnala il completamento (successo o errore) di una transazione SPI
 *         IT. Chiamata dal driver in contesto ISR (tipicamente
 *         osSemaphoreRelease).
 */
typedef void (*AD7490_SignalFn_t)(void);

/* ========================================================================== */
/* --- API --- */
/* ========================================================================== */

/**
 * @brief  Inizializza il driver associandolo all'handle SPI.
 * @param  hspi      Puntatore all'handle SPI5 generato da CubeMX.
 * @param  wait_fn   Funzione applicativa di attesa (vedi AD7490_WaitFn_t).
 * @param  signal_fn Funzione applicativa di segnalazione (vedi AD7490_SignalFn_t).
 * @note   Da chiamare una volta prima di AD7490_ScanAll().
 *         CS viene portato HIGH (inattivo) durante l'init.
 */
void AD7490_Init(SPI_HandleTypeDef *hspi, AD7490_WaitFn_t wait_fn, AD7490_SignalFn_t signal_fn);

/**
 * @brief  Scansiona tutti i 16 canali e popola l'array dei risultati.
 *
 * @param  out  Array di 16 uint16_t in cui vengono scritti i valori raw [0..4095].
 *              out[0] = NTC1, out[1] = NTC2, ..., out[15] = NTC16.
 *
 * @retval AD7490_OK        Scansione completata correttamente.
 *         AD7490_ERR_SPI   Errore HAL SPI durante la transazione.
 *         AD7490_ERR_INIT  AD7490_Init() non è stato chiamato.
 *
 * @note   Durata tipica: 17 transazioni SPI da 2 byte = ~3µs @68 Mbps.
 *         Assenza di ritardi bloccanti: sicuro da chiamare da task.
 */
AD7490_err_t AD7490_ScanAll(uint16_t out[AD7490_NUM_CHANNELS]);

/**
 * @brief  Legge un singolo canale (due transazioni SPI).
 *
 * @param  channel  Canale da leggere [0..15].
 * @param  out      Puntatore dove scrivere il risultato grezzo.
 * @retval AD7490_OK o codice di errore.
 *
 * @note   Meno efficiente di ScanAll per letture multiple: preferire ScanAll
 *         quando servono più canali nello stesso ciclo.
 */
AD7490_err_t AD7490_ReadChannel(uint8_t channel, uint16_t *out);

/* ========================================================================== */
/* --- CALLBACK IT (da chiamare SOLO dal dispatcher in stm32h7xx_it.c) --- */
/* ========================================================================== */

/**
 * @brief  Da chiamare da HAL_SPI_TxRxCpltCallback quando hspi->Instance==SPI5.
 *         Invoca signal_fn (fornita ad AD7490_Init) per sbloccare spi_transact().
 */
void AD7490_ITCallback(void);

/**
 * @brief  Da chiamare da HAL_SPI_ErrorCallback quando hspi->Instance==SPI5.
 *         Marca l'errore e invoca signal_fn per sbloccare spi_transact().
 */
void AD7490_ITErrorCallback(void);

#endif /* DRIVERS_TEMPSENSOR_AD7490_H_ */
