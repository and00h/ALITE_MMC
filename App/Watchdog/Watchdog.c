/*
 * Watchdog.c
 *
 * Vedere Watchdog.h per architettura e razionale.
 */

#include "Watchdog.h"
#include "cmsis_os.h"
#include "sys_log.h"

/* ========================================================================== */
/* --- STATO INTERNO --- */
/* ========================================================================== */

static IWDG_HandleTypeDef *s_hiwdg = NULL;

/* Tick (ms, configTICK_RATE_HZ=1000 → 1 tick = 1ms) dell'ultimo battito
 * ricevuto per ciascun task. volatile: scritto da task diversi, letto da
 * Watchdog_Service() nel contesto del default task. */
static volatile uint32_t s_last_beat_ms[WDG_NUM_TASKS];

/* true dal primo battito in poi: prima di allora il task è "non ancora
 * partito" e non deve bloccare il refresh (altrimenti reset già in boot). */
static volatile bool s_seen[WDG_NUM_TASKS];

/* ========================================================================== */
/* --- API PUBBLICA --- */
/* ========================================================================== */

void Watchdog_Init(IWDG_HandleTypeDef *hiwdg)
{
    s_hiwdg = hiwdg;

    uint32_t now = osKernelGetTickCount();
    for (uint32_t i = 0U; i < (uint32_t)WDG_NUM_TASKS; i++) {
        s_last_beat_ms[i] = now;
        s_seen[i]         = false;
    }
}

void Watchdog_Heartbeat(Watchdog_TaskId_t task)
{
    if (task >= WDG_NUM_TASKS) {
        return;
    }
    s_last_beat_ms[task] = osKernelGetTickCount();
    s_seen[task]         = true;
}

void Watchdog_Service(void)
{
    if (s_hiwdg == NULL) {
        return; /* Watchdog_Init() non ancora chiamata */
    }

    uint32_t now = osKernelGetTickCount();

    for (uint32_t i = 0U; i < (uint32_t)WDG_NUM_TASKS; i++) {
        if (!s_seen[i]) {
            /* Task non ancora partito dal boot: non è un blocco, è normale
             * sequenza di avvio. Non impedisce il refresh. */
            continue;
        }

        /* Sottrazione unsigned: corretta anche in caso di wraparound di
         * osKernelGetTickCount() (48+ giorni continui a 1kHz). */
        uint32_t age_ms = now - s_last_beat_ms[i];

        if (age_ms > WDG_MAX_HEARTBEAT_AGE_MS) {
            /* Almeno un task è considerato bloccato: NON rinfreschiamo.
             * L'IWDG1 scadrà e resetterà la scheda; Watchdog_LogResetCause()
             * lo registrerà al prossimo boot. */
            return;
        }
    }

    HAL_IWDG_Refresh(s_hiwdg);
}

void Watchdog_LogResetCause(void)
{
    if (__HAL_RCC_GET_FLAG(RCC_FLAG_IWDG1RST)) {
        SysLog_Event(LOG_ERROR, "RESET: IWDG1 (watchdog) - blocco software sospetto");
    } else if (__HAL_RCC_GET_FLAG(RCC_FLAG_WWDG1RST)) {
        SysLog_Event(LOG_ERROR, "RESET: WWDG1 (window watchdog)");
    } else if (__HAL_RCC_GET_FLAG(RCC_FLAG_LPWR1RST)) {
        SysLog_Event(LOG_WARN,  "RESET: ingresso illegale in low-power");
    } else if (__HAL_RCC_GET_FLAG(RCC_FLAG_BORRST)) {
        SysLog_Event(LOG_INFO,  "RESET: brown-out (BOR)");
    } else if (__HAL_RCC_GET_FLAG(RCC_FLAG_PINRST)) {
        SysLog_Event(LOG_INFO,  "RESET: pin NRST");
    } else if (__HAL_RCC_GET_FLAG(RCC_FLAG_SFTRST)) {
        SysLog_Event(LOG_INFO,  "RESET: software (NVIC_SystemReset)");
    } else if (__HAL_RCC_GET_FLAG(RCC_FLAG_PORRST)) {
        SysLog_Event(LOG_INFO,  "RESET: power-on (POR/PDR)");
    } else {
        SysLog_Event(LOG_INFO,  "RESET: causa non determinata (nessun flag RCC attivo)");
    }

    /* Cancella i flag per poter distinguere correttamente la causa
     * del PROSSIMO reset. */
    __HAL_RCC_CLEAR_RESET_FLAGS();
}
