/*
 * SPI3Bus.c
 *
 * Implementazione del mutex condiviso SPI3. Vedere SPI3Bus.h per la
 * documentazione completa. Stesso pattern di Config_CreateFlashMutex()/
 * Config_FlashMutexAcquire()/Release() (App/Config/config.c): mutex creato
 * dopo l'avvio del kernel, tollerante a un mutex ancora NULL in fase di
 * bootstrap pre-RTOS (single-thread, nessuna mutua esclusione necessaria).
 */

#include "SPI3Bus.h"

static osMutexId_t s_spi3_mutex = NULL;

void SPI3Bus_CreateMutex(void)
{
    if (s_spi3_mutex != NULL) {
        return; /* già creato */
    }
    static const osMutexAttr_t spi3_mutex_attr = {
        .name      = "spi3_bus_mtx",
        .attr_bits = osMutexPrioInherit,
    };
    s_spi3_mutex = osMutexNew(&spi3_mutex_attr);
}

osMutexId_t SPI3Bus_GetMutex(void)
{
    return s_spi3_mutex;
}

bool SPI3Bus_Acquire(uint32_t timeout_ms)
{
    if (s_spi3_mutex == NULL) {
        return true; /* bootstrap pre-RTOS: nessuna mutua esclusione necessaria */
    }
    return (osMutexAcquire(s_spi3_mutex, timeout_ms) == osOK);
}

void SPI3Bus_Release(void)
{
    if (s_spi3_mutex == NULL) {
        return;
    }
    osMutexRelease(s_spi3_mutex);
}
