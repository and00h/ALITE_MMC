/*
 * eFuse.h
 *
 * Driver per la gestione degli eFuse di protezione presenti sul sistema.
 *
 * ISTANZE GESTITE:
 *   EFUSE_MAIN  : eFuse scheda madre (24V/48V principale)
 *   EFUSE_SAB   : eFuse modulo di sicurezza
 *   EFUSE_COM   : eFuse COM interface
 *   EFUSE_LASEQ : eFuse driver di corrente LaseQ
 *
 * ARCHITETTURA:
 *   Ogni eFuse espone tre segnali hardware:
 *     - SHDN  (output MCU): spegne l'eFuse in emergenza
 *     - GOOD  (input MCU):  1 = tensione di uscita regolare
 *     - FLT   (input MCU):  segnala overcurrent / fault termico
 *
 *   La corrente assorbita da ogni carico è disponibile via ADC (vedere
 *   eFuse_UpdateAll). Le letture vengono memorizzate internamente; chiamare
 *   eFuse_GetStatus() per leggerle dall'esterno.
 *
 * LOGICA SHDN:
 *   EFUSE_MAIN usa MAIN_PWR_SHDN (active HIGH, GPIO_PIN_SET = eFuse spento).
 *   Tutti gli altri usano pin con prefisso 'n' (active LOW, GPIO_PIN_RESET = eFuse spento).
 *
 * UTILIZZO:
 *   // Init (una volta all'avvio, prima di osKernelStart):
 *   EFuse_Init(&hadc1, &hadc3);
 *
 *   // Da task_monitor ogni 100ms:
 *   EFuse_UpdateAll();
 *
 *   // Da FSM action:
 *   EFuse_Disable(EFUSE_LASEQ);
 *
 *   // Lettura stato:
 *   EFuse_status_t st = EFuse_GetStatus(EFUSE_MAIN);
 *   if (!st.power_good || st.fault) { ... }
 */

#ifndef DRIVERS_EFUSE_EFUSE_H_
#define DRIVERS_EFUSE_EFUSE_H_

#include "stm32h7xx_hal.h"
#include <stdbool.h>
#include <stdint.h>

/* ========================================================================== */
/* --- ISTANZE --- */
/* ========================================================================== */

typedef enum {
    EFUSE_MAIN  = 0,   /**< eFuse scheda madre (MAIN_PWR_SHDN, active HIGH) */
    EFUSE_SAB,         /**< eFuse modulo sicurezza                           */
    EFUSE_COM,         /**< eFuse COM interface                              */
    EFUSE_LASEQ,       /**< eFuse driver di corrente LaseQ                   */

    EFUSE_COUNT        /**< Sentinel */
} EFuse_id_t;

/* ========================================================================== */
/* --- STATO --- */
/* ========================================================================== */

typedef struct {
    bool     enabled;       /**< true se SHDN non è attivo (eFuse alimentato)  */
    bool     power_good;    /**< true se il pin GOOD è asserted                */
    bool     fault;         /**< true se il pin FLT è asserted (overcurrent)   */
    uint16_t current_raw;   /**< Lettura ADC grezza [0..4095]                  */
    uint16_t current_ma;    /**< Corrente stimata [mA].
                                 Modello TPS16630: I_IMON = 27.9µA/A × I_load.
                                 V_IMON = I_IMON × R_sense → I_load = V_IMON / (27.9µA × R)
                                 R_MAIN=36kΩ  → scala ≈ 0.803 mA/LSB (max ~3285mA)
                                 R_periph=107kΩ → scala ≈ 0.270 mA/LSB (max ~1106mA)
                                 Precisione stimata: ±5% (Vref + tolleranze).       */
} EFuse_status_t;

/* ========================================================================== */
/* --- API --- */
/* ========================================================================== */

/**
 * @brief  Inizializza il driver.
 *         Abilita ogni eFuse (SHDN non attivo) SOLO se il bit corrispondente
 *         in g_config.efuse_enabled_mask è a 1 (bit0=MAIN, bit1=SAB,
 *         bit2=COM, bit3=LASEQ — config.h); se il bit è 0 l'eFuse resta
 *         spento (SHDN nel livello che lo disabilita) per tutta la sessione
 *         — la maschera viene letta una sola volta qui, non ricontrollata
 *         a runtime. Registra anche gli handle ADC per il monitoraggio
 *         delle correnti.
 *
 * @param  hadc1  Handle ADC1 (MAIN, SAB, LASEQ current monitors).
 * @param  hadc3  Handle ADC3 (COM current monitor).
 *
 * @note   Da chiamare prima di osKernelStart(), dopo MX_ADCx_Init(), e
 *         DOPO Config_Init() (richiede g_config già popolato).
 *         Gli handle devono già essere stati inizializzati da CubeMX.
 */
void EFuse_Init(ADC_HandleTypeDef *hadc1, ADC_HandleTypeDef *hadc3);

/**
 * @brief  Abilita un eFuse (de-asserta SHDN).
 * @param  id  Istanza da abilitare.
 */
void EFuse_Enable(EFuse_id_t id);

/**
 * @brief  Disabilita un eFuse (asserta SHDN) — spegne il carico a valle.
 * @param  id  Istanza da disabilitare.
 * @note   Chiamare da FSM action in caso di fault; l'operazione è immediata.
 */
void EFuse_Disable(EFuse_id_t id);

/**
 * @brief  Aggiorna lo stato di tutti gli eFuse (GPIO + lettura ADC).
 *         Da chiamare da task_monitor ogni MONITOR_PERIOD_MS.
 * @note   Le letture ADC sono polled (bloccanti per ~1µs ciascuna).
 *         Essendo chiamata a 100ms non impatta le performance.
 */
void EFuse_UpdateAll(void);

/**
 * @brief  Restituisce l'ultimo stato aggiornato da EFuse_UpdateAll().
 * @param  id  Istanza da leggere.
 * @return Copia della struttura di stato.
 */
EFuse_status_t EFuse_GetStatus(EFuse_id_t id);

/**
 * @brief  Verifica se un eFuse è operativo (abilitato, GOOD asserted, no fault).
 * @param  id  Istanza da verificare.
 * @return true se tutto ok, false in caso di anomalia.
 */
bool EFuse_IsOk(EFuse_id_t id);

/**
 * @brief  Spegne TUTTI gli eFuse contemporaneamente.
 *         Da usare in caso di emergenza (es. allagamento).
 */
void EFuse_DisableAll(void);

#endif /* DRIVERS_EFUSE_EFUSE_H_ */
