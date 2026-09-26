/*
 * fw_slot.h (MMC, applicativo)
 *
 * Determina da quale slot fisico di flash (A @ 0x08040000 o B @ 0x08080000,
 * vedi OTA_SLOT_A_BASE/OTA_SLOT_B_BASE in MMC_Bootloader/App/OTA/ota_config.h)
 * il binario ATTUALMENTE IN ESECUZIONE e' stato linkato - esposto via RS485
 * dal comando "GET FW" (rs485_cmd.c), cosi' da poter sapere quale versione e'
 * davvero in esecuzione e in quale slot, SENZA dover entrare nel bootloader e
 * interrogare STATUS (utile in particolare dopo un flash diretto via
 * CubeProgrammer, che non passa dal journal metadati del bootloader e quindi
 * puo' lasciarlo disallineato rispetto a cosa e' fisicamente in flash).
 *
 * Rilevato A RUNTIME dall'indirizzo del vector table del binario stesso
 * (g_pfnVectors, primo simbolo di .isr_vector - il linker script lo pone
 * sempre come primissima word della FLASH, vedi MMC/STM32H723ZGTX_FLASH.ld
 * per Slot A e STM32H723ZGTX_FLASH_SlotB.ld per Slot B), NON da un define
 * manuale (-D SLOT_A/-D SLOT_B) da tenere sincrono a mano per ogni
 * configurazione di build: il risultato e' quindi corretto per costruzione,
 * indipendentemente da eventuali dimenticanze nella configurazione IDE.
 * Stesso principio gia' usato per il fix VTOR in main.c/system_stm32h7xx.c.
 */
#ifndef APP_CONFIG_FW_SLOT_H_
#define APP_CONFIG_FW_SLOT_H_

#include <stdint.h>

#define OTA_APP_SLOT_A_BASE   0x08040000UL
#define OTA_APP_SLOT_B_BASE   0x08080000UL
#define OTA_APP_SLOT_SIZE     (256UL * 1024UL)

/* Definito in Core/Startup/startup_stm32h723zgtx.s, stessa dichiarazione
 * gia' usata in main.c/system_stm32h7xx.c per il fix VTOR. */
extern uint32_t g_pfnVectors[];

/* 'A'/'B' se il binario e' linkato in uno dei due slot OTA, '?' per una
 * build "legacy" non linkata a nessuno slot (es. ancora a 0x08000000). */
static inline char FwSlot_GetActiveChar(void)
{
    uint32_t self_addr = (uint32_t)g_pfnVectors;

    if (self_addr >= OTA_APP_SLOT_A_BASE &&
        self_addr <  (OTA_APP_SLOT_A_BASE + OTA_APP_SLOT_SIZE)) {
        return 'A';
    }
    if (self_addr >= OTA_APP_SLOT_B_BASE &&
        self_addr <  (OTA_APP_SLOT_B_BASE + OTA_APP_SLOT_SIZE)) {
        return 'B';
    }
    return '?';
}

#endif /* APP_CONFIG_FW_SLOT_H_ */
