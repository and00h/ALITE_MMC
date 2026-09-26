/*
 * task_inputs.c
 *
 * Task di acquisizione e processing degli ingressi (5ms).
 * Vedere task_inputs.h per la documentazione.
 *
 * LOGICA EDGE DETECTION:
 *   Tutti i segnali EXT isolati sono active-LOW.
 *   s_prev_* memorizza lo stato logico invertito (true = segnale attivo).
 *   Falling edge pin (HIGH→LOW) = attivazione segnale = rising edge logico.
 *
 * LOGICA CHIAVI:
 *   SYS_KEY_REMOVED_EVENT viene postato se l'interlock chiave (Key_InterlockSatisfied(),
 *   che considera solo i contatti NON mascherati in g_config.error_mask —
 *   vedi ERR_BIT_KEY_A/ERR_BIT_KEY_B in task_monitor.h) passa da soddisfatto
 *   a non soddisfatto mentre la FSM è in SYS_ON, SYS_ENABLED o SYS_EMISSION.
 *   In stati precedenti (IDLE, ACTIVE) la rimozione è normale e non genera errori.
 *   Un contatto mascherato (non montato in produzione) non viene mai
 *   considerato: né per bloccare l'accensione né per generare rimozioni.
 *
 * LOGICA COPERCHI (dal 2026-07-15):
 *   SYS_LID_OPEN_EVENT viene postato con lo stesso identico schema delle
 *   chiavi sopra, ma su Lid_InterlockSatisfied() (ERR_BIT_LID1/ERR_BIT_LID2
 *   in task_monitor.h). Vedi anche il guard in action_enter_on() (fsm.c),
 *   che blocca l'ingresso in SYS_ON se un coperchio non mascherato è aperto.
 */

#include "FreeRTOS.h"
#include "task.h"
#include "task_inputs.h"
#include "queues.h"
#include "fsm.h"
#include "fsm_events.h"
#include "Key.h"
#include "Lid.h"
#include "FPButton.h"
#include "SAB.h"
#include "EXT_interface.h"
#include "WaterValve.h"  /* WaterValve_IsOpen(): sync nWATER_VALVE_STATUS */
#include "main.h"      /* Pin definitions */
#include "Watchdog.h"

/* ========================================================================== */
/* --- COSTANTE PERIODO TASK --- */
/* ========================================================================== */

#define INPUTS_TASK_PERIOD_MS   5U   /* Periodo task_inputs [ms] */

/* ========================================================================== */
/* --- ALLOCAZIONE STATICA --- */
/* ========================================================================== */

static StaticTask_t taskInputs_tcb;
static StackType_t  taskInputs_stack[256];

const osThreadAttr_t taskInputs_attr = {
    .name       = "Inputs",
    .stack_mem  = &taskInputs_stack[0],
    .stack_size = sizeof(taskInputs_stack),
    .priority   = osPriorityNormal2,
    .cb_mem     = &taskInputs_tcb,
    .cb_size    = sizeof(taskInputs_tcb),
};

/* ========================================================================== */
/* --- CALLBACK DRIVER → APPLICAZIONE --- */
/* ========================================================================== */

static void fpbtn_event_cb(FPButton_Evt_t evt)
{
    static const SysEvent_t ev_map[] = {
        [FPBTN_EVT_PRESS]        = SYS_BUTTON_PRESS_EVENT,
        [FPBTN_EVT_LONG_PRESS]   = SYS_BUTTON_LONG_PRESS_EVENT,
        [FPBTN_EVT_DOUBLE_PRESS] = SYS_BUTTON_DOUBLE_PRESS_EVENT,
    };
    if ((unsigned)evt < 3U) {
        Queue_PostEvent(ev_map[evt]);
    }
}

/* ========================================================================== */
/* --- STATO INTERNO: EDGE DETECTION INGRESSI EXT --- */
/* ========================================================================== */

/*
 * Stato logico (true = segnale attivo, ovvero pin fisico LOW)
 * Inizializzati a false (segnali inattivi) per non generare eventi spuri
 * al primo ciclo di task.
 */
static bool s_prev_ext_ctrl   = false;
static bool s_prev_sys_on     = false;
static bool s_prev_enable_in  = false;
static bool s_prev_clr_err    = false;

/* Stato precedente dell'interlock chiave (per rilevare rimozione) */
static bool s_prev_key_ok  = false;

/* Stato precedente dell'interlock coperchi (dal 2026-07-15, stesso schema
 * di s_prev_key_ok — per rilevare apertura) */
static bool s_prev_lid_ok  = false;

/* ========================================================================== */
/* --- FUNZIONI PRIVATE --- */
/* ========================================================================== */

/*
 * @brief Legge un pin active-LOW e restituisce lo stato logico:
 *        true  = segnale attivo (pin LOW)
 *        false = segnale inattivo (pin HIGH)
 */
static inline bool read_active_low(GPIO_TypeDef *port, uint16_t pin)
{
    return (HAL_GPIO_ReadPin(port, pin) == GPIO_PIN_RESET);
}

/*
 * @brief Polling nEXT_CTL_iso (PD14, active LOW isolato).
 *        Falling edge → modalità ANALOG attivata → SYS_EXT_CTRL_EVENT
 *        Rising edge  → modalità ANALOG disattivata → SYS_EXT_CTRL_OFF_EVENT
 */
static void Inputs_PollExtCtrl(void)
{
    bool cur = read_active_low(nEXT_CTL_iso_GPIO_Port, nEXT_CTL_iso_Pin);

    if (cur && !s_prev_ext_ctrl) {
        /* Falling edge fisico = attivazione controllo EXT */
        Queue_PostEvent(SYS_EXT_CTRL_EVENT);
    } else if (!cur && s_prev_ext_ctrl) {
        /* Rising edge fisico = rilascio controllo EXT */
        Queue_PostEvent(SYS_EXT_CTRL_OFF_EVENT);
    }

    s_prev_ext_ctrl = cur;
}

/*
 * @brief Polling nSYS_ON_iso (PD15, active LOW isolato).
 *        Solo in modalità ANALOG: post eventi SYS_ON/SYS_OFF.
 */
static void Inputs_PollSysOn(void)
{
    if (FSM_GetMode() != FSM_MODE_ANALOG) return;

    bool cur = read_active_low(nSYS_ON_iso_GPIO_Port, nSYS_ON_iso_Pin);

    if (cur && !s_prev_sys_on) {
        Queue_PostEvent(SYS_EXT_SYSON_EVENT);
    } else if (!cur && s_prev_sys_on) {
        Queue_PostEvent(SYS_EXT_SYSOFF_EVENT);
    }

    s_prev_sys_on = cur;
}

/*
 * @brief Polling nENABLE_IN_iso (PG2, active LOW isolato).
 *        Solo in modalità ANALOG: post eventi ENABLE/DISABLE.
 */
static void Inputs_PollEnableIn(void)
{
    if (FSM_GetMode() != FSM_MODE_ANALOG) return;

    bool cur = read_active_low(nENABLE_IN_iso_GPIO_Port, nENABLE_IN_iso_Pin);

    if (cur && !s_prev_enable_in) {
        Queue_PostEvent(SYS_EXT_ENABLE_EVENT);
    } else if (!cur && s_prev_enable_in) {
        Queue_PostEvent(SYS_EXT_DISABLE_EVENT);
    }

    s_prev_enable_in = cur;
}

/*
 * @brief Polling nCLR_ERR_iso (active LOW isolato).
 *        Solo in modalità ANALOG (richiesto esplicitamente, 2026-07-27 —
 *        stesso schema di Inputs_PollSysOn()/PollEnableIn() sopra: in SW
 *        mode il clear dell'errore recuperabile passa solo da RS485 "CERR",
 *        mai invariato). Falling edge fisico (HIGH->LOW, attivazione
 *        segnale, stessa convenzione di Inputs_PollExtCtrl/PollSysOn/
 *        PollEnableIn) -> SYS_EXT_CLR_ERR_EVENT.
 *
 *        BUGFIX STORICO (segnalato 2026-07-27): prima dell'introduzione di
 *        questa funzione, EXT_interface.clr_err veniva aggiornato da
 *        EXT_AcquireIN() ma nessun punto del firmware ne rilevava il fronte
 *        per postare l'evento — il pin fisico non aveva quindi alcun
 *        effetto sulla FSM in nessuna modalità (a differenza di CERR via
 *        RS485, che posta SYS_CLR_ERR_EVENT direttamente da rs485_cmd.c,
 *        sempre attivo indipendentemente da s_mode — vedi banner
 *        "ECCEZIONE CLR_ERR" in FSM_ProcessEvent(), fsm.c).
 */
static void Inputs_PollClrErr(void)
{
    if (FSM_GetMode() != FSM_MODE_ANALOG) return;

    bool cur = read_active_low(nCLR_ERR_iso_GPIO_Port, nCLR_ERR_iso_Pin);

    if (cur && !s_prev_clr_err) {
        Queue_PostEvent(SYS_EXT_CLR_ERR_EVENT);
    }

    s_prev_clr_err = cur;
}

/*
 * @brief Controlla l'interlock chiave negli stati armati.
 *        Se un contatto NON mascherato (vedi Key_InterlockSatisfied()) viene
 *        rimosso mentre il sistema è in ON/ENABLED/EMISSION posta
 *        SYS_KEY_REMOVED_EVENT. Un contatto mascherato (non montato) non
 *        contribuisce mai: non richiesto per l'interlock, non genera
 *        rimozioni.
 */
static void Inputs_CheckKeys(void)
{
    bool key_ok = Key_InterlockSatisfied();
    SysState_t state = FSM_GetState();

    /* Rilevamento rimozione chiave solo negli stati "armati" */
    if (state == SYS_ON || state == SYS_ENABLED || state == SYS_EMISSION) {
        if (s_prev_key_ok && !key_ok) {
            /* Chiave rimossa mentre sistema era armato */
            Queue_PostEvent(SYS_KEY_REMOVED_EVENT);
        }
    }

    s_prev_key_ok = key_ok;
}

/*
 * @brief Controlla l'interlock coperchi negli stati armati (dal 2026-07-15,
 *        stesso schema di Inputs_CheckKeys() per le chiavi).
 *        Se un sensore NON mascherato (vedi Lid_InterlockSatisfied()) si
 *        apre mentre il sistema è in ON/ENABLED/EMISSION posta
 *        SYS_LID_OPEN_EVENT. Un sensore mascherato (non montato) non
 *        contribuisce mai.
 */
static void Inputs_CheckLids(void)
{
    bool lid_ok = Lid_InterlockSatisfied();
    SysState_t state = FSM_GetState();

    /* Rilevamento apertura solo negli stati "armati" */
    if (state == SYS_ON || state == SYS_ENABLED || state == SYS_EMISSION) {
        if (s_prev_lid_ok && !lid_ok) {
            /* Coperchio aperto mentre sistema era armato */
            Queue_PostEvent(SYS_LID_OPEN_EVENT);
        }
    }

    s_prev_lid_ok = lid_ok;
}

/* ========================================================================== */
/* --- TASK --- */
/* ========================================================================== */

void TaskInputs_Run(void *arg)
{
    (void)arg;

    /* Inizializzazione stato precedente al valore corrente dei pin
     * per evitare eventi spuri al primo ciclo */
    s_prev_ext_ctrl  = read_active_low(nEXT_CTL_iso_GPIO_Port, nEXT_CTL_iso_Pin);
    s_prev_sys_on    = read_active_low(nSYS_ON_iso_GPIO_Port, nSYS_ON_iso_Pin);
    s_prev_enable_in = read_active_low(nENABLE_IN_iso_GPIO_Port, nENABLE_IN_iso_Pin);
    s_prev_clr_err   = read_active_low(nCLR_ERR_iso_GPIO_Port, nCLR_ERR_iso_Pin);
    s_prev_key_ok    = Key_InterlockSatisfied();
    s_prev_lid_ok    = Lid_InterlockSatisfied();

    /* Registra la callback del pulsante frontale */
    FPButton_RegisterCallback(fpbtn_event_cb);

    TickType_t xLastWakeTime = xTaskGetTickCount();

    for (;;)
    {
        Watchdog_Heartbeat(WDG_TASK_INPUTS);

        /* --- Acquisizione ingressi EXT analogici e digitali --- */
        EXT_AcquireIN(0);

        /*
         * nWATER_VALVE_STATUS: richiesto esplicitamente (2026-07-27) che
         * segua lo stato REALE della valvola istante per istante, in
         * QUALSIASI mode (SW o ANALOG) — non solo ai due punti fissi
         * (action_enter_idle/active, fsm.c) impostati finora. Letto back
         * dal pin di comando reale (WaterValve_IsOpen()), quindi corretto
         * anche se il valvola cambia stato per vie diverse dalla FSM (es.
         * WaterCooling_Disable() chiamata altrove).
         */
        EXT_SetValveStatus(WaterValve_IsOpen());

        /* --- Input fisici pannello frontale --- */
        Key_Process(INPUTS_TASK_PERIOD_MS);
        Lid_Process(INPUTS_TASK_PERIOD_MS);
        FPButton_Process(INPUTS_TASK_PERIOD_MS);

        /* --- Ingressi EXT isolati con edge detection --- */
        Inputs_PollExtCtrl();
        Inputs_PollSysOn();
        Inputs_PollEnableIn();
        Inputs_PollClrErr();

        /* --- Check rimozione chiave / apertura coperchio --- */
        Inputs_CheckKeys();
        Inputs_CheckLids();

        /* --- Safety Arm Block --- */
        SAB_Process(INPUTS_TASK_PERIOD_MS);

        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(INPUTS_TASK_PERIOD_MS));
    }
}
