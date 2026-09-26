/*
 * WaterCooling.h
 *
 *  Created on: 9 giu 2026
 *      Author: lucamaggiotanasi
 */

#ifndef SERVICES_WATERCOOLING_WATERCOOLING_H_
#define SERVICES_WATERCOOLING_WATERCOOLING_H_


#include <stdbool.h>
#include <stdint.h>

// ── Soglie di controllo ────────────────────────────────────────────────────
// Definite anche in Config/config.h se vuoi centralizzarle
#define WC_FLOW_MIN_LPM        1.5f   // sotto questa soglia: anomalia flusso
#define WC_TEMP_MAX_INLET_C    35.0f  // temperatura massima ingresso
#define WC_TEMP_MAX_OUTLET_C   45.0f  // temperatura massima uscita
#define WC_FLOW_CHECK_DELAY_MS 3000   // ms dopo enable prima di controllare il flusso
                                      // (la valvola ha bisogno di tempo per aprirsi)

// ── Stato osservabile (opzionale, per debug o UI) ──────────────────────────
typedef enum {
    WC_STATE_OFF = 0,
    WC_STATE_RUNNING,
    WC_STATE_ERROR,
} WaterCooling_State_t;

// ── API pubblica ───────────────────────────────────────────────────────────

/**
 * @brief Abilita il water cooling: apre la valvola, avvia acquisizione flussometri.
 *        Da chiamare nella entry action dello stato FSM che richiede raffreddamento.
 */
void WaterCooling_Enable(void);

/**
 * @brief Disabilita il water cooling: chiude la valvola, ferma acquisizione.
 *        Da chiamare nella exit action o in stati di idle/standby.
 */
void WaterCooling_Disable(void);

/**
 * @brief Shutdown di emergenza: chiude valvola immediatamente senza sequenze.
 *        Chiamato dal task monitor quando rileva un errore critico.
 */
void WaterCooling_Emergency(void);

/**
 * @brief Getter dello stato attuale (per debug o display).
 */
WaterCooling_State_t WaterCooling_GetState(void);

/**
 * @brief Restituisce true se il sistema è abilitato e il flusso è nella norma.
 *        Usabile dalla FSM per verificare la precondizione prima di avanzare di stato.
 */
bool WaterCooling_IsReady(void);

#endif /* SERVICES_WATERCOOLING_WATERCOOLING_H_ */
