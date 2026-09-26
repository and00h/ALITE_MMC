/*
 * task_fsm.c
 *
 * Implementazione del task della macchina a stati di sistema.
 *
 * SEQUENZA DI INIT (INIT → IDLE o INIT → FAULT):
 *   Prima di entrare nel loop eventi, il task esegue:
 *
 *   0. SAB_BOOT_DISCHARGE (rimovibile, vedi banner in fsm.c): routine non
 *      bloccante di scarica capacità sul modulo SAB (FSM_StartBootDischarge()
 *      + attesa di FSM_IsBootDischargeDone(), avanzata da FSM_Tick()).
 *
 *   Poi la sequenza di check hardware per verificare l'integrità del sistema:
 *
 *   1. Attesa di stabilizzazione alimentazioni (osDelay breve).
 *   2. EFuse_UpdateAll(): aggiorna le letture ADC degli eFuse.
 *   3. Verifica eFuse MAIN: deve essere ok (alimentazione principale presente).
 *   4. Verifica eFuse LASEQ: deve essere ok (driver di corrente alimentato).
 *
 *   Se tutti i check passano → SYS_CHECK_OK_EVENT → FSM transisce a IDLE.
 *   Se almeno un check fallisce → SYS_CHECK_FAIL_EVENT → FSM transisce a FAULT.
 *
 * NOTA: L'allagamento NON fa parte di questa sequenza (non è un "check
 *       hardware di boot" al pari di eFuse MAIN/LASEQ): un flood già
 *       presente al power-on deve generare esattamente la stessa risposta
 *       di un flood rilevato a macchina già operativa — mascherabile via
 *       g_config.fault_mask (FAULT_BIT_FLOOD1/FLOOD2, dal 2026-07-14 — vedi
 *       redesign severità + split per sensore in task_monitor.h), con
 *       WaterCooling_Emergency()
 *       e taglio eFuse ritardato di 6s — non un fault "hardware bloccante"
 *       incondizionato. Questa logica vive esclusivamente in
 *       task_monitor.c (Monitor_CheckFlood(), ciclo 100ms), che gira già
 *       PRIMA di questa sequenza di init (task_monitor ha priorità
 *       Normal5, superiore a questo task Normal4: il suo primo ciclo,
 *       incluso il flood check, è già completato entro i primi 100ms
 *       dall'avvio) — quindi un allagamento presente al power-on viene
 *       già gestito correttamente senza bisogno di duplicare il check qui.
 *       I check di temperatura e flusso sono demandati allo stesso task
 *       per lo stesso motivo (richiedono stabilizzazione del sistema).
 */

#include "FreeRTOS.h"
#include "task.h"
#include "task_fsm.h"
#include "fsm.h"
#include "queues.h"
#include "eFuse.h"
#include "Watchdog.h"

/*
 * Timeout del wait sulla coda eventi nel loop principale: puramente per
 * heartbeat (Watchdog.h). In condizioni normali il sistema può restare a
 * lungo senza eventi (nessun pulsante, nessun fault, nessun comando) —
 * comportamento legittimo, non un blocco. Il timeout NON cambia la logica
 * di elaborazione eventi (osOK dalla coda si comporta come prima); serve
 * solo a far ripassare periodicamente il task dal punto in cui segnala
 * Watchdog_Heartbeat(WDG_TASK_FSM), altrimenti un lungo periodo di quiete
 * verrebbe scambiato per un blocco reale.
 */
#define TASK_FSM_HEARTBEAT_TIMEOUT_MS   2000U

/* ========================================================================== */
/* --- ALLOCAZIONE STATICA (nessun heap, dimensione garantita a compile time) */
/* ========================================================================== */

static StaticTask_t taskFSM_tcb;
static StackType_t  taskFSM_stack[384]; /* 384 word = 1536 byte (aumentato per init sequence) */

const osThreadAttr_t taskFSM_attr = {
    .name       = "FSM",
    .stack_mem  = &taskFSM_stack[0],
    .stack_size = sizeof(taskFSM_stack),
    .priority   = osPriorityNormal4,    /* Sotto task_monitor (Normal5) */
    .cb_mem     = &taskFSM_tcb,
    .cb_size    = sizeof(taskFSM_tcb),
};

/* ========================================================================== */
/* --- TASK --- */
/* ========================================================================== */

/* ========================================================================== */
/* --- SEQUENZA DI INIT --- */
/* ========================================================================== */

/*
 * @brief  Esegue i check hardware di avvio e posta l'evento appropriato.
 *
 *         Chiamata UNA SOLA VOLTA prima del loop eventi principale.
 *         Non usa HAL_GetTick(): usa osDelay() perché siamo nel contesto
 *         del task FSM, mai da ISR.
 *
 * @retval true  = tutti i check superati → posta SYS_CHECK_OK_EVENT
 *         false = almeno un check fallito → posta SYS_CHECK_FAIL_EVENT
 */
static bool TaskFSM_RunInitSequence(void)
{
    bool ok = true;

    /*
     * Attesa di stabilizzazione: task_monitor (priorità Normal5, superiore a
     * questo task Normal4) ha già eseguito EFuse_UpdateAll() nel suo primo ciclo
     * entro i primi 100ms dall'avvio. Aspettiamo 150ms per essere certi che
     * almeno un ciclo completo sia stato completato prima di leggere i valori.
     *
     * NON chiamare EFuse_UpdateAll() qui: TaskMonitor e TaskFSM condividono
     * gli stessi handle HAL ADC (hadc1, hadc3). Chiamate concorrenti causano
     * contesa sull'ADC → timeout o stato inconsistente → FSM bloccata in INIT.
     */
    osDelay(150);

    /* --- Check eFuse MAIN (alimentazione principale 24V/48V) --- */
    if (!EFuse_IsOk(EFUSE_MAIN)) {
        ok = false;
    }

    /* --- Check eFuse LaseQ (alimentazione driver corrente) --- */
    if (!EFuse_IsOk(EFUSE_LASEQ)) {
        ok = false;
    }

    /*
     * L'allagamento NON è incluso qui volutamente: vedi commento di testata
     * del file. Un flood al power-on è gestito da Monitor_CheckFlood()
     * (task_monitor.c), già in esecuzione a questo punto, con la stessa
     * logica (mascherabile, water cooling emergency, taglio eFuse ritardato)
     * usata a macchina operativa — non un fallimento di questa sequenza.
     */

    if (ok) {
        Queue_PostEvent(SYS_CHECK_OK_EVENT);
    } else {
        Queue_PostEvent(SYS_CHECK_FAIL_EVENT);
    }

    return ok;
}

/* ========================================================================== */
/* --- TASK --- */
/* ========================================================================== */

void TaskFSM_Run(void *arg)
{
    (void)arg;

    FSM_Init();

    /*
     * FEATURE-FLAG "SAB_BOOT_DISCHARGE" (rimovibile, vedi banner completo in
     * fsm.c — cercare "SAB_BOOT_DISCHARGE" per l'elenco dei punti da
     * toccare in caso di rimozione). Introdotta il 2026-07-07: prima di
     * poter eseguire la sequenza di check hardware ed entrare in IDLE, la
     * FSM esegue un ciclo di scarica capacità sul modulo SAB (non è
     * garantito che lo spegnimento precedente sia avvenuto da uno stato con
     * capacità già scariche). Routine non bloccante, avanzata da FSM_Tick():
     * il polling sotto (osDelay + FSM_Tick(), Watchdog_Heartbeat incluso)
     * NON impedisce agli altri task di girare regolarmente, e non introduce
     * alcuna attesa bloccante (HAL_Delay/osDelay lunghi) — si limita a
     * ripassare periodicamente finché FSM_IsBootDischargeDone() non è vero.
     *
     * PER RIMUOVERE: eliminare questo blocco (FSM_StartBootDischarge() +
     * while) e richiamare TaskFSM_RunInitSequence() subito dopo FSM_Init(),
     * come prima dell'introduzione della feature.
     */
    FSM_StartBootDischarge();
    while (!FSM_IsBootDischargeDone()) {
        Watchdog_Heartbeat(WDG_TASK_FSM);
        osDelay(5);        /* poll a bassa frequenza, nessun busy-wait */
        FSM_Tick();
    }

    /*
     * FAULT LATCH (dal 2026-07-14, vedi banner "FAULT LATCH" in
     * task_monitor.h): se FSM_Init() ha già rilevato un fault latched da un
     * boot precedente (g_config.fault_latch_active) e portato lo stato
     * direttamente a SYS_FAULT, la normale sequenza di check hardware va
     * SALTATA — altrimenti TaskFSM_RunInitSequence() posterebbe
     * SYS_CHECK_OK/FAIL_EVENT, che la tabella di transizione non gestisce da
     * SYS_FAULT (nessun effetto, ma comunque concettualmente sbagliato:
     * un fault latched non deve mai essere "ri-verificato" dall'hardware,
     * solo esplicitamente resettato via "FRST").
     */
    if (FSM_GetState() != SYS_FAULT) {
        /* Sequenza di init: verifica hardware e transisce INIT → IDLE o INIT → FAULT */
        TaskFSM_RunInitSequence();
    }

    SysEvent_t event;

    for (;;)
    {
        Watchdog_Heartbeat(WDG_TASK_FSM);

        /*
         * Blocca il task finché non arriva un evento nella coda, o al più
         * TASK_FSM_HEARTBEAT_TIMEOUT_MS (vedi commento sopra la define):
         * scaduto il timeout senza eventi, si ripassa dal giro del loop
         * senza fare nulla — CPU quasi zero in assenza di eventi, come
         * prima con osWaitForever, solo con un check periodico in più.
         */
        if (osMessageQueueGet(eventQueueHandle, &event, NULL, TASK_FSM_HEARTBEAT_TIMEOUT_MS) == osOK)
        {
            FSM_ProcessEvent(event);
        }

        /*
         * Avanzamento "a tempo" della FSM (guardia 10s spegnimento SAB —
         * vedi fsm.c/fsm.h per il dettaglio). Chiamato ad ogni giro, sia che
         * sopra sia arrivato un evento (subito) sia che si sia scaduto il
         * timeout (al più ogni TASK_FSM_HEARTBEAT_TIMEOUT_MS): il confronto
         * interno è su deadline assoluta, quindi la cadenza variabile di
         * questo loop non ne compromette la correttezza.
         */
        FSM_Tick();
    }
}
