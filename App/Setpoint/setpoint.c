/*
 * setpoint.c
 *
 * Implementazione della gestione setpoint laser.
 * Vedere setpoint.h per la documentazione dell'API.
 */

#include "setpoint.h"
#include "config.h"     /* g_config.setpoint_lut_ma, SETPOINT_LUT_SIZE, ... */

/* ========================================================================== */
/* --- STATO INTERNO --- */
/* ========================================================================== */

/*
 * Percentuale setpoint attiva [0-100].
 * Inizializzata a 0 (sicuro: nessuna emissione all'avvio).
 * Modificata solo da Setpoint_SetPct() / Setpoint_Reset().
 * NOTA: quando s_direct_mode è attivo, questo valore resta quello
 * dell'ultima Setpoint_SetPct() e NON riflette la corrente erogata
 * (vedi Setpoint_SetCurrentMa() / Setpoint_IsDirectMode()).
 */
static uint8_t  s_setpoint_pct = 0U;

/*
 * Corrente diretta [mA] impostata via Setpoint_SetCurrentMa(), valida solo
 * quando s_direct_mode == true.
 */
static uint32_t s_setpoint_current_ma_direct = 0U;

/*
 * true se il setpoint attivo è stato impostato in modo diretto (bypass LUT).
 * Disattivato da ogni Setpoint_SetPct()/Setpoint_Reset(), che riportano la
 * sorgente del setpoint alla conversione percentuale normale.
 */
static bool s_direct_mode = false;

/* ========================================================================== */
/* --- API PUBBLICA --- */
/* ========================================================================== */

void Setpoint_SetPct(uint8_t pct)
{
    if (pct > SETPOINT_MAX_PCT) {
        pct = SETPOINT_MAX_PCT;
    }
    s_setpoint_pct = pct;
    s_direct_mode  = false;   /* torna alla conversione LUT normale */
}

uint8_t Setpoint_GetPct(void)
{
    return s_setpoint_pct;
}

uint32_t Setpoint_GetCurrentMa(void)
{
    if (s_direct_mode) {
        return s_setpoint_current_ma_direct;
    }
    return Setpoint_PctToCurrentMa(s_setpoint_pct);
}

void Setpoint_SetCurrentMa(uint32_t current_ma)
{
    if (current_ma > 0xFFFFU) {
        current_ma = 0xFFFFU;
    }
    s_setpoint_current_ma_direct = current_ma;
    s_direct_mode = true;
}

bool Setpoint_IsDirectMode(void)
{
    return s_direct_mode;
}

void Setpoint_Reset(void)
{
    s_setpoint_pct                = 0U;
    s_setpoint_current_ma_direct  = 0U;
    s_direct_mode                 = false;
}

uint32_t Setpoint_PctToCurrentMa(uint8_t pct)
{
    if (pct > SETPOINT_MAX_PCT) {
        pct = SETPOINT_MAX_PCT;
    }

    /*
     * Calcola l'indice inferiore nella LUT e la frazione intera residua.
     *
     * Esempio (SETPOINT_LUT_STEP_PCT = 5):
     *   pct = 37 → idx_lo = 7 (35%), frac = 2 (due punti sopra il 35%)
     *   pct = 40 → idx_lo = 8 (40%), frac = 0 (esatto, no interpolazione)
     *   pct = 100 → idx_lo = 20 (100%), frac = 0
     */
    uint8_t idx_lo = pct / SETPOINT_LUT_STEP_PCT;
    uint8_t frac   = pct % SETPOINT_LUT_STEP_PCT;

    /* Punto esatto o ultimo entry: nessuna interpolazione */
    if (frac == 0U || idx_lo >= (SETPOINT_LUT_SIZE - 1U)) {
        /* Clamp per sicurezza (non dovrebbe mai accadere con pct ≤ 100) */
        if (idx_lo >= SETPOINT_LUT_SIZE) {
            idx_lo = SETPOINT_LUT_SIZE - 1U;
        }
        return (uint32_t)g_config.setpoint_lut_ma[idx_lo];
    }

    /*
     * Interpolazione lineare a tratti tra entry adiacenti:
     *   result = lo + (hi - lo) * frac / STEP
     *
     * Usa int32_t per gestire correttamente il caso hi < lo (LUT non monotonica
     * dopo calibrazione). Il risultato è clampato a 0 in caso di underflow.
     */
    int32_t lo     = (int32_t)g_config.setpoint_lut_ma[idx_lo];
    int32_t hi     = (int32_t)g_config.setpoint_lut_ma[idx_lo + 1U];
    int32_t result = lo + ((hi - lo) * (int32_t)frac) / (int32_t)SETPOINT_LUT_STEP_PCT;

    return (result > 0) ? (uint32_t)result : 0U;
}
