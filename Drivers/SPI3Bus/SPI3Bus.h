/*
 * SPI3Bus.h
 *
 * Mutex condiviso per il bus SPI3, fisicamente multiplexato tra due
 * periferiche indipendenti (vedi hal_handles.h):
 *
 *   - Scheda microSD (Drivers/SD/sd_spi.c)      — CS: nSD_CS (PG15)
 *   - Modulo COM interface (Drivers/COM_interface) — CS: COM_SPI_nCS (PD0)
 *
 * Le due periferiche usano SPI3 8-bit Mode 0 (CPOL=0/CPHA=0), ciascuna
 * riconfigurata a runtime prima di ogni transazione (com_spi_reconfigure()
 * in COM_interface.c, spi_reconfigure() in sd_spi.c — SOLO prescaler e
 * NSSPMode differiscono tra le due, vedi sd_spi.h. Revisione 2026-07-17:
 * NON esiste una modalita' 4-bit per la COM interface, era un refuso di un
 * commento precedente — il default CubeMX di MX_SPI3_Init() e' gia' 8-bit)
 * e chip-select software indipendenti:
 * SENZA mutua esclusione, una transazione COM interface che parte mentre
 * SD sta ancora trasferendo (o viceversa) corromperebbe entrambe le
 * transazioni (CS/clock/data condivisi sullo stesso bus fisico).
 *
 * USO:
 *   Ogni driver che tocca hspi3 DEVE acquisire il mutex prima di
 *   qualunque operazione SPI3 (init, transfer, riconfigurazione data-size)
 *   e rilasciarlo subito dopo, CS incluso nella sezione protetta.
 *
 *   SPI3Bus_CreateMutex();                          // una volta, MX_FREERTOS_Init()
 *   ...
 *   if (SPI3Bus_Acquire(osWaitForever)) {
 *       // ... transazione SPI3 ...
 *       SPI3Bus_Release();
 *   }
 *
 * Il driver SD (sd_spi.c) accetta il mutex direttamente come parametro di
 * SD_Init() (osMutexId_t spi_mutex) — passargli SPI3Bus_GetMutex().
 * Il driver COM interface (COM_interface.c) usa SPI3Bus_Acquire()/Release()
 * internamente attorno a ogni transazione.
 *
 * THREAD SAFETY: il mutex va creato una sola volta, PRIMA che qualunque
 * task possa usare SPI3 — quindi in MX_FREERTOS_Init(), come
 * Config_CreateFlashMutex(). Prima della creazione (o se osKernelStart()
 * non è ancora avvenuto), Acquire()/Release() sono no-op che ritornano
 * sempre true: in quella finestra l'esecuzione è single-thread.
 */

#ifndef DRIVERS_SPI3BUS_SPI3BUS_H_
#define DRIVERS_SPI3BUS_SPI3BUS_H_

#include "cmsis_os.h"
#include <stdbool.h>
#include <stdint.h>

/**
 * @brief  Crea il mutex condiviso per SPI3. Da chiamare UNA VOLTA da
 *         MX_FREERTOS_Init() (freertos.c), DOPO osKernelInitialize().
 *         Idempotente (no-op se già creato).
 */
void SPI3Bus_CreateMutex(void);

/**
 * @brief  Restituisce l'handle del mutex, da passare direttamente a
 *         SD_Init() (Drivers/SD/sd_spi.h) come parametro spi_mutex.
 * @retval NULL se SPI3Bus_CreateMutex() non è ancora stata chiamata.
 */
osMutexId_t SPI3Bus_GetMutex(void);

/**
 * @brief  Acquisisce il mutex SPI3. Usata da COM_interface.c attorno a
 *         ogni transazione (l'equivalente per SD è interno a sd_spi.c,
 *         tramite l'handle passato a SD_Init()).
 * @param  timeout_ms  osWaitForever o timeout in ms (stile CMSIS-RTOS2).
 * @retval true se acquisito (o mutex non ancora creato: single-thread).
 */
bool SPI3Bus_Acquire(uint32_t timeout_ms);

/**
 * @brief  Rilascia il mutex SPI3 preso con SPI3Bus_Acquire().
 */
void SPI3Bus_Release(void);

#endif /* DRIVERS_SPI3BUS_SPI3BUS_H_ */
