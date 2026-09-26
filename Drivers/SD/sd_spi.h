/*
 * sd_spi.h
 *
 * Driver SPI per scheda microSD — livello fisico (transport layer).
 * Implementa il protocollo SPI Mode 0 secondo le specifiche SD/SDHC.
 *
 * HARDWARE:
 *   SPI3  (CPOL=0, CPHA=0, 8-bit, software CS)
 *   nSD_CS      PG15  GPIO_Output  – chip select (active LOW)
 *   nSD_PRESENT PB5   GPIO_Input   – card detect (active LOW = card presente)
 *
 * NOTA SPI3 DATA WIDTH (corretta 2026-07-17: il commento precedente
 * affermava un DataSize=4-bit per la COM interface, era un refuso — il
 * default CubeMX di MX_SPI3_Init() è già 8-bit, come questo driver):
 *   SPI3 è condivisa con la COM interface (Drivers/COM_interface), che usa
 *   anch'essa 8-bit/CPOL0/CPHA0 ma un proprio prescaler e NSSPMode diverso
 *   (SPI_NSS_PULSE_DISABLE qui vs il default CubeMX per la COM interface) —
 *   ogni driver riconfigura esplicitamente hspi3 prima di ogni propria
 *   transazione (spi_reconfigure() qui, com_spi_reconfigure() in
 *   COM_interface.c), per non dipendere da quale dei due ha usato il bus
 *   per ultimo. Un mutex (sd_spi_mutex, vedi SPI3Bus.h) protegge l'accesso
 *   al bus SPI3.
 *
 * VELOCITÀ:
 *   Init: prescaler impostato per ≤400kHz (SD init spec).
 *   Data: prescaler per ~8MHz dopo l'init (limite tipico in SPI Mode 0).
 *   I valori esatti dipendono dal clock SPI3 (kernel clock PCLK1 o HSI).
 *
 * UTILIZZO:
 *   // Da un task FreeRTOS dopo l'avvio dello scheduler:
 *   SD_Result_t r = SD_Init(&hspi3, spi3_mutex);
 *   if (r == SD_OK) {
 *       uint8_t buf[512];
 *       SD_ReadBlock(0, buf);   // legge settore 0
 *   }
 */

#ifndef DRIVERS_SD_SD_SPI_H_
#define DRIVERS_SD_SD_SPI_H_

#include "stm32h7xx_hal.h"
#include "cmsis_os.h"
#include <stdbool.h>
#include <stdint.h>

/* ========================================================================== */
/* --- COSTANTI --- */
/* ========================================================================== */

#define SD_BLOCK_SIZE   512U    /**< Dimensione blocco in byte (fisso per SPI mode) */

/* ========================================================================== */
/* --- TIPI --- */
/* ========================================================================== */

typedef enum {
    SD_OK             = 0,
    SD_ERR_NO_CARD,       /**< Scheda non presente (nSD_PRESENT HIGH)        */
    SD_ERR_INIT_FAIL,     /**< Inizializzazione fallita (CMD0/CMD8/ACMD41)   */
    SD_ERR_TIMEOUT,       /**< Timeout durante attesa risposta o busy        */
    SD_ERR_CRC,           /**< CRC o token di errore ricevuto                */
    SD_ERR_READ,          /**< Errore lettura blocco                         */
    SD_ERR_WRITE,         /**< Errore scrittura blocco                       */
    SD_ERR_PARAM,         /**< Parametro non valido                          */
} SD_Result_t;

typedef enum {
    SD_TYPE_NONE  = 0,
    SD_TYPE_V1,           /**< SD v1.x (byte addressing)                    */
    SD_TYPE_V2,           /**< SD v2.x standard capacity (byte addressing)  */
    SD_TYPE_SDHC,         /**< SD v2.x HC/XC (block addressing)             */
} SD_CardType_t;

/* ========================================================================== */
/* --- API --- */
/* ========================================================================== */

/**
 * @brief  Inizializza la scheda SD.
 *         Riconfigura SPI3 a 8-bit, esegue la sequenza di init SD (CMD0→CMD8→
 *         ACMD41→CMD58), poi aumenta la velocità SPI per il trasferimento dati.
 *         Da chiamare da un task FreeRTOS dopo l'avvio dello scheduler.
 *
 * @param  hspi       Puntatore all'handle SPI3 (da hal_handles.h).
 * @param  spi_mutex  Mutex FreeRTOS per l'accesso esclusivo a SPI3 (può essere
 *                    NULL se SPI3 è usato esclusivamente da SD in questa fase).
 * @return SD_OK in caso di successo.
 */
SD_Result_t SD_Init(SPI_HandleTypeDef *hspi, osMutexId_t spi_mutex);

/**
 * @brief  Legge un singolo blocco da 512 byte.
 * @param  block_addr  Indirizzo blocco (per SDHC) o byte (per SD v1/v2).
 *                     Con FatFS questa è sempre una LBA.
 * @param  buf         Buffer destinazione [512 byte].
 * @return SD_OK in caso di successo.
 */
SD_Result_t SD_ReadBlock(uint32_t block_addr, uint8_t *buf);

/**
 * @brief  Scrive un singolo blocco da 512 byte.
 * @param  block_addr  Indirizzo blocco.
 * @param  buf         Buffer sorgente [512 byte].
 * @return SD_OK in caso di successo.
 */
SD_Result_t SD_WriteBlock(uint32_t block_addr, const uint8_t *buf);

/**
 * @brief  Restituisce il tipo di scheda rilevato durante l'init.
 */
SD_CardType_t SD_GetCardType(void);

/**
 * @brief  Restituisce true se la scheda è fisicamente presente (nSD_PRESENT LOW).
 */
bool SD_IsPresent(void);

/**
 * @brief  Restituisce true se l'init è stata completata con successo.
 */
bool SD_IsReady(void);

#endif /* DRIVERS_SD_SD_SPI_H_ */
