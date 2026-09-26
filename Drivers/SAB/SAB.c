/*
 * SAB.c
 *
 * Implementazione del driver per il modulo SAB (Safety Arm Block).
 * Vedere SAB.h per la documentazione completa.
 *
 * SEQUENZA DI ARMO:
 *   SAB_PowerOn()  → nSAB_PWR_SHDN HIGH (alimenta il modulo)
 *   SAB_Enable()   → nSAB_EN LOW → SAB_STATE_GUARD (SAB_GUARD_DELAY_MS)
 *                  → SAB_STATE_ENABLING (monitora INTLCK + TEST)
 *                  → SAB_STATE_ARMED se tutto OK, evento SAB_EVT_ARMED
 *
 * GUARD DELAY:
 *   Dopo l'asserzione di nSAB_EN i segnali di stato non sono validi per
 *   SAB_GUARD_DELAY_MS (1s). Durante questo periodo i pin non vengono letti.
 *
 * ENABLING:
 *   – TEST HIGH in qualsiasi momento → SAB_EVT_TEST_FAIL immediato.
 *   – INTLCK devono essere entrambi LOW (chiuso/OK) entro timeout_ms dal
 *     termine del guard.
 *   – Quando INTLCK chiusi + TEST OK → SAB_STATE_ARMED + SAB_EVT_ARMED.
 *
 * ARMED:
 *   – Qualsiasi INTLCK HIGH (aperto) → SAB_EVT_INTERLOCK_OPEN.
 *   – Qualsiasi TEST HIGH   → SAB_EVT_TEST_FAIL.
 */

#include "SAB.h"
#include "main.h"          /* Pin definitions */
#include "FreeRTOS.h"
#include "task.h"
#include "config.h"        /* g_config.error_mask */
#include "task_monitor.h"  /* ERR_BIT_SAB_TEST_A / ERR_BIT_SAB_TEST_B */

/* ============================================================================
 * FALLBACK DEFINE — nSAB_PWR_SHDN (PC1)
 * Rimuovere quando CubeMX genera le define col label corretto.
 * ============================================================================ */
#ifndef nSAB_PWR_SHDN_Pin
#  define nSAB_PWR_SHDN_GPIO_Port   GPIOC
#  define nSAB_PWR_SHDN_Pin         GPIO_PIN_1
#endif

/* ============================================================================
 * STATO INTERNO
 * ============================================================================ */

static SAB_State_t   s_state              = SAB_STATE_DISABLED;
static SAB_EventCb_t s_event_cb           = NULL;
static uint32_t      s_configured_timeout = 0U;

/* Guard delay dopo nSAB_EN: decrementato in GUARD */
static uint32_t s_guard_ms   = 0U;

/* Timeout interlock: decrementato in ENABLING */
static uint32_t s_timeout_ms = 0U;

/*
 * Flag volatili aggiornati dalle callback EXTI.
 * true = pin HIGH = interlock APERTO.
 * default: true (aperto) = stato sicuro prima dell'armo.
 */
static volatile bool s_intlck_a_open = true;
static volatile bool s_intlck_b_open = true;

/* ============================================================================
 * HELPER: verifica pin TEST
 * ============================================================================ */

/*
 * I pin TEST devono essere LOW (GPIO_PIN_RESET) per indicare assenza di guasti.
 * Un livello HIGH indica un guasto nel circuito di diagnostica del SAB.
 *
 * BUGFIX (2026-07-16): a differenza degli INTLCK (vera catena di sicurezza
 * ridondante, la cui risposta resta SEMPRE attiva indipendentemente da
 * error_mask — vedi banner in SAB.h), i canali TEST sono un self-test
 * diagnostico per cui è prevista mascherabilità indipendente per canale
 * (ERR_BIT_SAB_TEST_A/B, task_monitor.h) quando in una data configurazione
 * HW un canale non è effettivamente cablato/popolato. Prima di questo fix
 * la funzione ignorava del tutto la maschera e faceva l'AND diretto dei due
 * pin: con un solo canale abilitato in error_mask (es. 0xEBE: bit5=TEST_A
 * attivo, bit6=TEST_B mascherato) un canale B non cablato/flottante a HIGH
 * causava comunque SAB_STATE_FAULT, perché test_pins_ok() non aveva modo
 * di saperlo. Stesso pattern già in uso in Key_InterlockSatisfied()/
 * Lid_InterlockSatisfied(): bit NON settato in error_mask -> canale ignorato
 * (considerato sempre OK), bit settato -> il pin va davvero letto LOW.
 */
static bool test_pins_ok(void)
{
    GPIO_PinState a = HAL_GPIO_ReadPin(nSAB_A_TEST_GPIO_Port, nSAB_A_TEST_Pin);
    GPIO_PinState b = HAL_GPIO_ReadPin(nSAB_B_TEST_GPIO_Port, nSAB_B_TEST_Pin);

    bool a_ok = !(g_config.error_mask & ERR_BIT_SAB_TEST_A) || (a == GPIO_PIN_RESET);
    bool b_ok = !(g_config.error_mask & ERR_BIT_SAB_TEST_B) || (b == GPIO_PIN_RESET);

    return a_ok && b_ok;
}

/* ============================================================================
 * API PUBBLICA
 * ============================================================================ */

void SAB_Init(uint16_t timeout_ms, SAB_EventCb_t cb)
{
    s_configured_timeout = timeout_ms;
    s_event_cb           = cb;
    s_state              = SAB_STATE_DISABLED;
    s_guard_ms           = 0U;
    s_timeout_ms         = 0U;
    s_intlck_a_open      = true;
    s_intlck_b_open      = true;

    /* Sicurezza all'avvio: modulo spento, nSAB_EN HIGH */
    HAL_GPIO_WritePin(nSAB_EN_GPIO_Port,       nSAB_EN_Pin,       GPIO_PIN_SET);
    HAL_GPIO_WritePin(nSAB_PWR_SHDN_GPIO_Port, nSAB_PWR_SHDN_Pin, GPIO_PIN_RESET);
}

void SAB_PowerOn(void)
{
    /* Alimenta il modulo SAB: nSAB_PWR_SHDN HIGH */
    HAL_GPIO_WritePin(nSAB_PWR_SHDN_GPIO_Port, nSAB_PWR_SHDN_Pin, GPIO_PIN_SET);
}

void SAB_PowerOff(void)
{
    /* Prima disabilita (nSAB_EN HIGH), poi spegni l'alimentazione.
     * NESSUNA guardia temporale qui: vedi banner "GUARDIA 10s" in SAB.h —
     * usare solo dove non c'è rischio di scaricare condensatori sotto
     * carico (es. action_init_ok() al boot). */
    SAB_Disable();
    SAB_ShutdownSupply();
}

void SAB_ShutdownSupply(void)
{
    HAL_GPIO_WritePin(nSAB_PWR_SHDN_GPIO_Port, nSAB_PWR_SHDN_Pin, GPIO_PIN_RESET);
}

/* SAB_BOOT_DISCHARGE (rimovibile, vedi banner in fsm.c e in SAB.h) */
void SAB_RawEnPulse(bool assert_low)
{
    /* Scrittura diretta del pin, SENZA passare da SAB_Enable()/SAB_Disable():
     * non aggiorna s_state (resta SAB_STATE_DISABLED), non avvia guard/
     * enabling/interlock monitoring. Uso esclusivo: routine SAB_BOOT_DISCHARGE
     * in fsm.c, per il singolo impulso di scarica capacità al boot. */
    HAL_GPIO_WritePin(nSAB_EN_GPIO_Port, nSAB_EN_Pin,
                       assert_low ? GPIO_PIN_RESET : GPIO_PIN_SET);
}

void SAB_Enable(void)
{
    if (s_state != SAB_STATE_DISABLED) {
        return;   /* Già in corso di armo o armato: no-op */
    }

    s_intlck_a_open = true;   /* reset flag: leggeremo in ENABLING */
    s_intlck_b_open = true;
    s_guard_ms      = SAB_GUARD_DELAY_MS;
    s_state         = SAB_STATE_GUARD;

    /* Asserisce nSAB_EN LOW: avvia il modulo SAB */
    HAL_GPIO_WritePin(nSAB_EN_GPIO_Port, nSAB_EN_Pin, GPIO_PIN_RESET);
}

void SAB_Disable(void)
{
    s_state      = SAB_STATE_DISABLED;
    s_guard_ms   = 0U;
    s_timeout_ms = 0U;

    /* Deassert nSAB_EN HIGH: disabilita il modulo */
    HAL_GPIO_WritePin(nSAB_EN_GPIO_Port, nSAB_EN_Pin, GPIO_PIN_SET);
}

bool SAB_IsInterlockOk(void)
{
    return (s_state == SAB_STATE_ARMED);
}

bool SAB_IsFault(void)
{
    return (s_state == SAB_STATE_FAULT);
}

/* Funzioni interne mantenute per la FSM */
SAB_State_t SAB_GetState(void)
{
    return s_state;
}

bool SAB_IsArmed(void)
{
    return (s_state == SAB_STATE_ARMED);
}

/*
 * SAB_GetInterlockStatus() / SAB_GetTestStatus() (dal 2026-07-15):
 * espongono lo stato RAW per canale (A/B), usate da freertos.c
 * (sab_event_cb, mascherabilità indipendente ERR_BIT_SAB_INTLCK_A/B e
 * ERR_BIT_SAB_TEST_A/B) e da fsm.c (event_detail_name(), dettaglio
 * "GET ALARM") — vedi banner in SAB.h. Non influenzano in alcun modo
 * s_state/la FSM di questo modulo: sola lettura diagnostica.
 */
void SAB_GetInterlockStatus(bool *intlck_a_open, bool *intlck_b_open)
{
    taskENTER_CRITICAL();
    bool a = s_intlck_a_open;
    bool b = s_intlck_b_open;
    taskEXIT_CRITICAL();

    if (intlck_a_open != NULL) { *intlck_a_open = a; }
    if (intlck_b_open != NULL) { *intlck_b_open = b; }
}

void SAB_GetTestStatus(bool *test_a_fault, bool *test_b_fault)
{
    GPIO_PinState a = HAL_GPIO_ReadPin(nSAB_A_TEST_GPIO_Port, nSAB_A_TEST_Pin);
    GPIO_PinState b = HAL_GPIO_ReadPin(nSAB_B_TEST_GPIO_Port, nSAB_B_TEST_Pin);

    if (test_a_fault != NULL) { *test_a_fault = (a == GPIO_PIN_SET); }
    if (test_b_fault != NULL) { *test_b_fault = (b == GPIO_PIN_SET); }
}

/* ============================================================================
 * FSM — chiamata ogni tick_ms ms da task_inputs
 * ============================================================================ */

void SAB_Process(uint32_t tick_ms)
{
    bool a_open, b_open;

    switch (s_state) {

        /* ------------------------------------------------------------------ */
        case SAB_STATE_DISABLED:
            /* Nessuna elaborazione */
            break;

        /* ------------------------------------------------------------------ */
        case SAB_STATE_GUARD:
            /*
             * nSAB_EN è LOW ma i pin non sono ancora validi.
             * Non leggere nulla — attendere SAB_GUARD_DELAY_MS.
             */
            if (s_guard_ms > tick_ms) {
                s_guard_ms -= tick_ms;
            } else {
                s_guard_ms = 0U;

                /*
                 * Guard scaduto: leggi lo stato attuale dei pin interlock
                 * e inizia il countdown del timeout.
                 *
                 * POLARITA' (confermata 2026-07-15): LOW/RESET = chiuso/OK,
                 * HIGH/SET = aperto/fault (vedi banner in SAB.h — un
                 * precedente tentativo di "fix" aveva invertito questa
                 * lettura per errore, corretto qui: il codice originale era
                 * giusto). Il fix reale del mancato rilevamento apertura era
                 * l'EXTI su PA0/PC2 rigenerato a IT_RISING_FALLING invece di
                 * IT_RISING.
                 */
                taskENTER_CRITICAL();
                s_intlck_a_open = (HAL_GPIO_ReadPin(nSAB_INTLCK_A_STATUS_GPIO_Port,
                                                    nSAB_INTLCK_A_STATUS_Pin) == GPIO_PIN_SET);
                s_intlck_b_open = (HAL_GPIO_ReadPin(nSAB_INTLCK_B_STATUS_GPIO_Port,
                                                    nSAB_INTLCK_B_STATUS_Pin) == GPIO_PIN_SET);
                taskEXIT_CRITICAL();

                s_timeout_ms = s_configured_timeout;
                s_state      = SAB_STATE_ENABLING;
            }
            break;

        /* ------------------------------------------------------------------ */
        case SAB_STATE_ENABLING:
            /*
             * Monitora simultaneamente INTLCK e TEST.
             * TEST ha priorità: se uno è HIGH il modulo SAB ha un guasto interno
             * e non ha senso aspettare la chiusura degli interlock.
             */

            /* --- Verifica pin TEST (guasto diagnostica) --- */
            if (!test_pins_ok()) {
                s_state = SAB_STATE_FAULT;
                if (s_event_cb) { s_event_cb(SAB_EVT_TEST_FAIL); }
                break;
            }

            /* --- Snapshot thread-safe degli interlock --- */
            taskENTER_CRITICAL();
            a_open = s_intlck_a_open;
            b_open = s_intlck_b_open;
            taskEXIT_CRITICAL();

            /* --- Entrambi gli interlock chiusi e TEST OK → ARMED --- */
            if (!a_open && !b_open) {
                s_state = SAB_STATE_ARMED;
                if (s_event_cb) { s_event_cb(SAB_EVT_ARMED); }
                break;
            }

            /* --- Countdown timeout interlock --- */
            if (s_timeout_ms > tick_ms) {
                s_timeout_ms -= tick_ms;
            } else {
                s_timeout_ms = 0U;
                s_state      = SAB_STATE_FAULT;
                if (s_event_cb) { s_event_cb(SAB_EVT_TIMEOUT); }
            }
            break;

        /* ------------------------------------------------------------------ */
        case SAB_STATE_ARMED:
            /*
             * Monitoraggio continuo di tutti i pin.
             * Qualsiasi anomalia → FAULT + evento specifico.
             */

            /* Snapshot thread-safe degli interlock */
            taskENTER_CRITICAL();
            a_open = s_intlck_a_open;
            b_open = s_intlck_b_open;
            taskEXIT_CRITICAL();

            if (a_open || b_open) {
                s_state = SAB_STATE_FAULT;
                if (s_event_cb) { s_event_cb(SAB_EVT_INTERLOCK_OPEN); }
                break;
            }

            if (!test_pins_ok()) {
                s_state = SAB_STATE_FAULT;
                if (s_event_cb) { s_event_cb(SAB_EVT_TEST_FAIL); }
            }
            break;

        /* ------------------------------------------------------------------ */
        case SAB_STATE_FAULT:
            /* Stato terminale: rimane in FAULT fino a SAB_Disable() */
            break;
    }
}

/* ============================================================================
 * CALLBACK ISR
 * ============================================================================ */

void SAB_IntlckA_EXTI_Callback(void)
{
    /*
     * Leggi stato pin al momento dell'interrupt: HIGH/SET = aperto (polarità
     * originale confermata corretta il 2026-07-15 — vedi banner SAB.h).
     * IT_RISING_FALLING (main.c, rigenerato da CubeMX): cattura sia
     * l'apertura (fronte di salita) sia la richiusura (fronte di discesa),
     * utile per la diagnostica live "GET SAB".
     */
    s_intlck_a_open = (HAL_GPIO_ReadPin(nSAB_INTLCK_A_STATUS_GPIO_Port,
                                        nSAB_INTLCK_A_STATUS_Pin) == GPIO_PIN_SET);
}

void SAB_IntlckB_EXTI_Callback(void)
{
    /* HIGH/SET = aperto — vedi commento in SAB_IntlckA_EXTI_Callback() sopra. */
    s_intlck_b_open = (HAL_GPIO_ReadPin(nSAB_INTLCK_B_STATUS_GPIO_Port,
                                        nSAB_INTLCK_B_STATUS_Pin) == GPIO_PIN_SET);
}
