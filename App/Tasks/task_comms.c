/*
 * task_comms.c
 *
 * Task di comunicazione. Gestisce due canali:
 *   1. RS485 verso LaseQ    (ogni LQ_TRANSMIT_WINDOW ms = 20ms @115200baud)
 *   2. Fault alimentazione COM interface (nCOM_PWR_FLT, polling ogni ciclo,
 *      vedi COM_Interface_Update()) — NON lo scambio dati vero e proprio con
 *      la COM interface, che dal 2026-07-17 vive nel task dedicato
 *      task_com_interface.c (guidato da interrupt/semaforo, non da questo
 *      loop periodico — vedi COM_interface.h per i dettagli)
 *
 * NOTA 2026-07-22: il servicing di AMC (UART verso AMC, ex punto 3 di
 * questo task) e' stato spostato in un task dedicato, task_amc.c — vedi
 * quel file per la motivazione (compensazione dinamica tensione PSU,
 * richiede un ciclo AMC comparabile in frequenza a quello di LaseQ). Questo
 * task NON chiama piu' alcuna funzione AMC_*() direttamente; il "ponte" tra
 * i due task e' il solo gating del setpoint verso LaseQ, vedi punto 2a nel
 * loop principale sotto.
 *
 * STARTUP SEQUENCE:
 *   1. Connessione iniziale LaseQ (5 tentativi × 500ms)
 *   2. Loop ciclico LaseQ + fault COM interface
 *
 * NOTA BAUD RATE:
 *   A 115200 baud una trama da 47 byte occupa ~4.1ms per direzione.
 *   LQ_TRANSMIT_WINDOW = 20ms garantisce margine per TX + processing + RX.
 *   Ridurre a 10ms solo se baud rate ≥ 230400.
 */

#include "FreeRTOS.h"
#include "task.h"
#include "task_comms.h"
#include "queues.h"
#include "fsm.h"
#include "LaseQ.h"
#include "COM_interface.h"
#include "task_monitor.h"
#include "config.h"
#include "AMC.h"
#include <stdbool.h>
#include "hal_handles.h"
#include "setpoint.h"
#include "Watchdog.h"
#include "sys_log.h"
#include "QCW.h"

/* ========================================================================== */
/* --- PARAMETRI ATTESA SETPOINT ASCENDENTE (VCOMP) --- */
/* ========================================================================== */

/*
 * Tempo MASSIMO di attesa (rete di sicurezza), dopo un aumento del setpoint
 * di corrente, prima di inoltrarlo comunque a LaseQ quando la compensazione
 * dinamica di tensione PSU e' attiva (vedi punto 2a nel loop principale
 * sotto). Il PSU impiega diverso tempo per andare a regime rispetto alla
 * sola rampa DAC lato AMC (g_config.comp_ramp_up_duration_ms, 1-5ms): il
 * margine qui sotto era, fino al 2026-07-27, un'attesa CIECA fissa (alzata
 * da 500ms a 2000ms lo stesso giorno, sulla base di un assestamento
 * elettrico reale misurato di circa 1.5s).
 *
 * REVISIONATO 2026-07-27 (v2): invece di aspettare sempre alla cieca per
 * l'intera durata, il rilascio ora avviene NON APPENA LaseQ conferma (via
 * GetLaseQStatus().v_anode, aggiornato ogni LQ_TRANSMIT_WINDOW=20ms dalla
 * transazione RS485 normale, nessun costo aggiuntivo) che la tensione PSU ha
 * effettivamente raggiunto Vmax entro VCOMP_VANODE_TOLERANCE_MV — vedi ramo
 * "hold_cycles_left > 0" sotto. VCOMP_SETPOINT_HOLD_MS resta come timeout
 * MASSIMO/rete di sicurezza (sensore v_anode assente, rumore, LUT non
 * configurata, guasto PSU): garantisce comunque un limite superiore finito
 * all'attesa, stessa invariante "non blocca mai a tempo indefinito" gia'
 * documentata sotto per la prima versione del gating.
 * Si applica SOLO quando comp_active e' vero (vedi sotto): a VCOMP
 * disattiva il nuovo setpoint resta istantaneo, invariato.
 * Espresso in numero di cicli del loop (non in ms assoluti/HAL_GetTick())
 * per restare coerente con lo stile gia' in uso in questo file e in
 * task_amc.c (vedi AMC_MAX_MISS_COUNT), dato che il loop gira a passo fisso
 * LQ_TRANSMIT_WINDOW via vTaskDelayUntil.
 */
#define VCOMP_SETPOINT_HOLD_MS      2000U
#define VCOMP_SETPOINT_HOLD_CYCLES  (VCOMP_SETPOINT_HOLD_MS / LQ_TRANSMIT_WINDOW)

_Static_assert(VCOMP_SETPOINT_HOLD_MS % LQ_TRANSMIT_WINDOW == 0U,
               "VCOMP_SETPOINT_HOLD_MS deve essere multiplo di LQ_TRANSMIT_WINDOW");

/*
 * Tolleranza per considerare "raggiunto Vmax" il valore misurato da LaseQ
 * (v_anode, mV) rispetto al limite configurato (g_config.psu_voltage_mv).
 * Valore di partenza prudenziale, non validato su banco: aggiustare in base
 * al rumore/quantizzazione reali del canale di misura VANODE (vedi "GET LQ",
 * campo VANODE) e alla precisione richiesta. Se serve renderlo regolabile
 * da RS485 senza ricompilare, si puo' aggiungere un comando dedicato (es.
 * "SET VCOMP TOLERANCE <mV>") in un secondo momento.
 */
#define VCOMP_VANODE_TOLERANCE_MV   500U

/* ========================================================================== */
/* --- ALLOCAZIONE STATICA --- */
/* ========================================================================== */

static StaticTask_t taskComms_tcb;
static StackType_t  taskComms_stack[512];

const osThreadAttr_t taskComms_attr = {
    .name       = "Comms",
    .stack_mem  = &taskComms_stack[0],
    .stack_size = sizeof(taskComms_stack),
    .priority   = osPriorityNormal2,
    .cb_mem     = &taskComms_tcb,
    .cb_size    = sizeof(taskComms_tcb),
};

/* ========================================================================== */
/* --- SEMAFORI PRIVATI (DMA TX/RX LaseQ) --- */
/* ========================================================================== */

static osSemaphoreId_t sem_tx = NULL;
static osSemaphoreId_t sem_rx = NULL;

static void OnTxComplete(void) { osSemaphoreRelease(sem_tx); }
static void OnRxComplete(void) { osSemaphoreRelease(sem_rx); }

/* ========================================================================== */
/* --- FUNZIONI PRIVATE --- */
/* ========================================================================== */

static LaseQ_err_t laseq_transact(uint32_t timeout_ms)
{
    LaseQ_err_t err;

    /*
     * Drain difensivo: se la transazione precedente è finita in errore a
     * metà (vedi LaseQ_ErrorCallback, che rilascia sia sem_tx che sem_rx
     * per sbloccare subito l'attesa pendente), può restare un token
     * "orfano" sul semaforo non atteso in quel momento. Senza questo
     * drain, il prossimo osSemaphoreAcquire lo consumerebbe subito senza
     * attendere il vero completamento del nuovo trasferimento DMA,
     * disallineando permanentemente TX/RX (stesso schema già usato in
     * AMC.c / amc_transceive()). Acquire con timeout 0 = non bloccante.
     */
    (void)osSemaphoreAcquire(sem_tx, 0U);
    (void)osSemaphoreAcquire(sem_rx, 0U);

    err = LaseQ_Transmit();
    if (err != LASEQ_OK) return err;
    if (osSemaphoreAcquire(sem_tx, timeout_ms) != osOK) return LASEQ_ERR_TIMEOUT;

    err = LaseQ_Receive();
    if (err != LASEQ_OK) return err;
    if (osSemaphoreAcquire(sem_rx, timeout_ms) != osOK) return LASEQ_ERR_TIMEOUT;

    return LaseQ_ParseResponse(LASEQ_MSG_STATUS);
}

/* ========================================================================== */
/* --- TASK --- */
/* ========================================================================== */

void TaskComms_Run(void *arg)
{
    (void)arg;

    /*
     * Stato errori LaseQ: dichiarato qui (non più dentro il blocco "3. Loop
     * principale" come in origine) perché ora deve sopravvivere anche al
     * fallimento della connessione iniziale, sotto.
     */
    uint8_t lq_err_count    = 0;
    bool    lq_fault_posted = false;

    /*
     * Tracking per la decodifica di MSG_STATUS.error_code (LaseQ, vedi
     * FSM_ErrorCode_t in fsm.h) — categoria indipendente da lq_err_count/
     * lq_fault_posted sopra (quelli rilevano la PERDITA di comunicazione,
     * questi decodificano il CONTENUTO di una risposta valida):
     *   s_lq_last_err_code : ultimo error_code visto, per loggare solo sui
     *                        cambiamenti (non ad ogni ciclo da 20ms).
     *   lq_err_posted      : SYS_LASEQ_ERROR_EVENT già postato per la
     *                        condizione bit 0-5 corrente (evita repost ad
     *                        ogni ciclo finché il bit resta attivo).
     *   lq_hw_err_posted   : SYS_LASEQ_ERROR_EVENT già postato per il bit 7
     *                        (FSM_FAULT_HW: PWR_OK basso o interlock
     *                        aperto) corrente — flag separata da
     *                        lq_err_posted perché il bit 7 ha un gate di
     *                        stato diverso (SYS_ON, non SYS_ACTIVE — vedi
     *                        BUGFIX sotto). Dal 2026-07-15 (v2) posta
     *                        SYS_LASEQ_ERROR_EVENT come gli altri bit, MAI
     *                        più SYS_LASEQ_FAULT_EVENT — vedi banner
     *                        SEVERITÀ in fsm.h.
     */
    uint8_t lq_last_err_code  = 0U;
    bool    lq_err_posted     = false;
    bool    lq_hw_err_posted  = false;

    /*
     * Assestamento accensione LaseQ: LaseQSupplySet(1) è già stato chiamato
     * in LaseQ_Init() (TaskComms_Init(), pre-scheduler), ma da lì a qui
     * passa solo il tempo incidentale impiegato da Sys_HwInit() nel task
     * Monitor (priorità più alta) — non garantito, non dimensionato sul
     * boot di LaseQ. Aspettiamo qui esplicitamente LASEQ_POWERON_SETTLE_MS
     * (vedi LaseQ.h) prima del primissimo tentativo di trasmissione:
     * altrimenti il primo frame può arrivare mentre la UART/DMA di LaseQ
     * non è ancora pronta, disallineando la ricezione (protocollo a
     * lunghezza fissa, nessun resync via IDLE) anche sui retry successivi.
     */
    osDelay(LASEQ_POWERON_SETTLE_MS);

    /* ------------------------------------------------------------------ */
    /* 1. Connessione iniziale LaseQ                                       */
    /* ------------------------------------------------------------------ */
    /*
     * DISACCOPPIATO da AMC (2026-07-02): in origine, se LaseQ non rispondeva
     * entro questi tentativi, l'intero task veniva eliminato con
     * vTaskDelete(NULL) — uccidendo anche tutta la comunicazione AMC, che
     * sta più sotto nella stessa funzione e non veniva mai raggiunta.
     * Risultato: zero traffico su USART6 (AMC) ogni volta che LaseQ non era
     * collegato/pronto all'avvio, anche se AMC era perfettamente
     * funzionante. Ora ci limitiamo a segnalare il fault e proseguire: il
     * loop principale (punto 2, che già gestisce un LaseQ che continua a
     * non rispondere ciclo per ciclo) parte comunque, e AMC gira in
     * parallelo nel proprio task dedicato (task_amc.c, dal 2026-07-22).
     * Permette di testare/debuggare LaseQ e AMC in modo indipendente, anche
     * con uno dei due non collegato sul banco.
     */
    {
        uint8_t retries = 5;
        LaseQ_err_t err = LASEQ_ERR_TIMEOUT;
        while (retries-- > 0) {
            err = laseq_transact(500);
            if (err == LASEQ_OK) break;
            osDelay(200);
        }
        if (err != LASEQ_OK) {
            /*
             * NON postiamo SYS_LASEQ_FAULT_EVENT né tocchiamo lq_fault_posted
             * qui: a questo punto la FSM è quasi certamente ancora in
             * SYS_INIT/SYS_IDLE, dove l'evento viene scartato di proposito
             * (fault LaseQ/AMC rilevabili solo da SYS_ACTIVE in poi). Se
             * settassimo lq_fault_posted=true, il primo post — ignorato a
             * vuoto dalla FSM — bloccherebbe (via "!lq_fault_posted") il
             * repost genuino del loop principale una volta raggiunto
             * ACTIVE, lasciando LaseQ scollegato senza mai andare in fault.
             * Ci limitiamo a loggare: il rilevamento vero e proprio lo fa
             * il loop principale sotto (10 cicli falliti = 200ms).
             */
            SysLog_Event(LOG_WARN, "LaseQ: nessuna risposta dopo %u tentativi all'avvio", 5U);
        }
    }

    /* ------------------------------------------------------------------ */
    /* 2. Loop principale                                                  */
    /* ------------------------------------------------------------------ */
    /*
     * NOTA 2026-07-22: la configurazione AMC (limiti PSU, CONFIG_PD, LUT
     * gain/PD_VALID/compensazione tensione) non e' piu' qui — vive nella
     * sequenza di avvio di task_amc.c (task dedicato), che gira in
     * parallelo a questo task.
     */
    TickType_t xLastWakeTime = xTaskGetTickCount();

    /* lq_err_count / lq_fault_posted: dichiarate all'inizio della funzione
     * (vedi punto 1), non più qui, per sopravvivere anche a un fallimento
     * della connessione iniziale. */

    for (;;)
    {
        Watchdog_Heartbeat(WDG_TASK_COMMS);

        /* -------------------------------------------------------------- */
        /* 2a. LaseQ: transazione ogni LQ_TRANSMIT_WINDOW ms              */
        /* -------------------------------------------------------------- */

        /*
         * In FSM_MODE_SW (incluso il toggle "HYBRID2"/setpoint HW, vedi
         * fsm.h): aggiorna il setpoint corrente prima della trasmissione.
         * In FSM_MODE_ANALOG: il setpoint arriva dall'hardware (LPWR_SET_ISO
         * / AMC DAC); LaseQ ignora sw_current_setpoint in quel modo, ma per
         * chiarezza non lo scriviamo.
         *
         * ATTESA SETPOINT ASCENDENTE (revisionato, prima versione 2026-07-22):
         * Se la compensazione dinamica di tensione PSU e' attiva
         * (g_config.voltage_comp_enabled), NON attiva SETPOINT HW
         * (FSM_GetSetpointHwEnabled(), dove i cambi di riferimento sono
         * istantanei/imprevedibili e la compensazione non deve intervenire
         * — vedi banner psu_voltage_comp.h lato AMC) e NON in QCW
         * (QCW_IsActive(), dove LASE_Q_OUT_5V commuta al ritmo
         * dell'impulsazione): un AUMENTO del setpoint desiderato rispetto
         * all'ultimo osservato NON viene inoltrato subito a LaseQ. Si
         * mantiene per VCOMP_SETPOINT_HOLD_CYCLES cicli (2000ms, non solo
         * un ciclo/20ms come nella prima versione) l'ultimo valore che
         * LaseQ ha CONFERMATO di applicare (GetLaseQStatus().sw_current_setpoint),
         * dando tempo al PSU (task AMC dedicato, task_amc.c) di arrivare
         * davvero a regime — non solo di completare la rampa DAC
         * (g_config.comp_ramp_up_duration_ms = 1-5ms), che e' molto piu'
         * rapida dell'assestamento elettrico reale. Nessuno stato
         * condiviso con task_amc.c: la sincronizzazione resta temporale
         * (stesso periodo LQ_TRANSMIT_WINDOW), non basata su un ACK (AMC
         * non conferma il setpoint applicato, vedi AMC_protocol.h).
         * Un setpoint in DISCESA non ha bisogno di attesa (la rampa di
         * discesa lato AMC e' lenta, 50-100ms, nessun rischio dropout) e
         * interrompe subito un'eventuale attesa in corso.
         *
         * ATTENZIONE bug della prima versione: il gating confrontava il
         * setpoint desiderato col valore CONFERMATO da LaseQ
         * (desired_ma > GetLaseQStatus().sw_current_setpoint). Siccome il
         * valore confermato non puo' aggiornarsi se non si smette MAI di
         * trattenerlo, la condizione restava vera all'infinito: il nuovo
         * setpoint non veniva mai inoltrato. Qui il conteggio dei cicli da
         * attendere (hold_cycles_left) e' invece un contatore INDIPENDENTE
         * da qualunque stato di LaseQ/AMC: decrementa incondizionatamente
         * ad ogni ciclo (ramo "attesa in corso" sotto) e arriva SEMPRE a
         * zero in un tempo finito, dopo il quale il ramo "else" inoltra il
         * setpoint corrente — quindi il nuovo valore viene sempre e
         * comunque inviato, mai piu' bloccato a tempo indefinito.
         */
        if (FSM_GetMode() == FSM_MODE_SW) {
            uint32_t desired_ma = Setpoint_GetCurrentMa();
            static uint32_t old_setpoint      = 0;
            static uint32_t hold_setpoint_ma  = 0;
            static uint32_t hold_cycles_left  = 0;

            bool comp_active = (g_config.voltage_comp_enabled != 0U) &&
                                !FSM_GetSetpointHwEnabled() &&
                                !QCW_IsActive();

            if (!comp_active || (desired_ma < old_setpoint)) {
                /* VCOMP non attiva, o setpoint in discesa: nessun'attesa,
                 * si inoltra subito. Azzerare hold_cycles_left qui e'
                 * essenziale: un'attesa ancora armata da un aumento
                 * precedente non deve restare pendente e ritardare in modo
                 * scorretto un futuro aumento, una volta che comp_active
                 * torna vero. */
                hold_cycles_left = 0U;
                LaseQSetCurrent(desired_ma);
            } else if (desired_ma > old_setpoint) {
                /* Nuovo salto ascendente: (ri)arma l'attesa per
                 * VCOMP_SETPOINT_HOLD_CYCLES cicli (2000ms), mantenendo
                 * l'ultimo valore CONFERMATO da LaseQ. Un ulteriore
                 * aumento durante l'attesa la riarma per intero: si
                 * aspettano sempre 2000ms dall'ULTIMO aumento, non dal
                 * primo.
                 *
                 * AMC_RequestVoltageRestart() (NUOVO 2026-07-27, protocollo
                 * v0.0005): un aumento di corrente a laser gia' acceso
                 * produce uno spunto/drop di tensione esattamente come una
                 * vera accensione — senza questo segnale AMC interpolerebbe
                 * subito verso la tensione compensata (bassa) del nuovo
                 * setpoint, senza il margine di Vmax che serve ad assorbire
                 * lo spunto. Richiesto qui, non nel ramo "attesa in corso"
                 * sotto: un solo restart per salto ascendente, riarmato
                 * insieme all'attesa se arriva un ulteriore aumento nel
                 * frattempo (stessa logica di hold_cycles_left sopra). */
                hold_setpoint_ma  = GetLaseQStatus().sw_current_setpoint;
                hold_cycles_left  = VCOMP_SETPOINT_HOLD_CYCLES;
                AMC_RequestVoltageRestart();
                LaseQSetCurrent(hold_setpoint_ma);
            } else if (hold_cycles_left > 0U) {
                /*
                 * Attesa in corso, setpoint invariato: rilascio ANTICIPATO
                 * se v_anode (misura reale PSU da LaseQ, GetLaseQStatus(),
                 * aggiornata ogni LQ_TRANSMIT_WINDOW) conferma di aver
                 * raggiunto Vmax entro VCOMP_VANODE_TOLERANCE_MV — non serve
                 * aspettare alla cieca l'intera finestra se il PSU e' gia'
                 * pronto. Altrimenti continua a trattenere e decrementa
                 * incondizionatamente come rete di sicurezza (timeout
                 * MASSIMO, vedi banner VCOMP_SETPOINT_HOLD_MS sopra) —
                 * questo e' l'unico altro punto che fa avanzare il conteggio
                 * verso zero.
                 */
                uint16_t vmax_mv = (g_config.psu_voltage_mv > 0 && g_config.psu_voltage_mv <= 0xFFFFU)
                                   ? (uint16_t)g_config.psu_voltage_mv : 0xFFFFU;
                uint16_t v_anode = GetLaseQStatus().v_anode;
                int32_t  diff_mv = (int32_t)vmax_mv - (int32_t)v_anode;
                if (diff_mv < 0) { diff_mv = -diff_mv; }

                if ((uint32_t)diff_mv <= VCOMP_VANODE_TOLERANCE_MV) {
                    hold_cycles_left = 0U;
                    LaseQSetCurrent(desired_ma);
                } else {
                    LaseQSetCurrent(hold_setpoint_ma);
                    hold_cycles_left--;
                }
            } else {
                /* Attesa scaduta (o mai armata): inoltra il setpoint
                 * corrente. Raggiunto sempre entro un tempo finito, vedi
                 * nota sopra sul bug della prima versione. */
                LaseQSetCurrent(desired_ma);
            }
            old_setpoint = desired_ma;
        }

        LaseQ_err_t lq_err = laseq_transact(LQ_TRANSMIT_WINDOW);

        if (lq_err == LASEQ_OK) {
            lq_err_count    = 0;
            lq_fault_posted = false;
            TaskMonitor_SetFaultBit(FAULT_BIT_LASEQ, false);

            LaseQ_status_vars_t st = GetLaseQStatus();
            TaskMonitor_UpdateLaseQTelemetry(
                st.temperature,               /* 4 temp. elementi di potenza [°C] (driver) */
                st.temp_ambient_c,            /* SHT35 temperatura ambiente [°C, int8_t] */
                (uint8_t)(st.humidity / 100U) /* 0.01%RH → %RH */
            );

            /*
             * ---------------------------------------------------------
             * error_code (MSG_STATUS byte 1): natura interna dell'errore
             * segnalato da LaseQ (FSM_ErrorCode_t, fsm.h). Categoria
             * indipendente dal fault di comunicazione sopra: qui la
             * transazione RS485 è RIUSCITA, è LaseQ stesso a riportare
             * una condizione interna nel payload.
             *
             * BUGFIX 2026-07-15 (v2): TUTTI i bit (0-5 e 7) -> SYS_ERROR
             * (SYS_LASEQ_ERROR_EVENT, recuperabile via CERR), MAI più
             * SYS_FAULT — vedi banner SEVERITÀ in fsm.h per il perché
             * (il bit 7/FSM_FAULT_HW copre anche "interlock aperto", un
             * evento plausibile in esercizio normale, non un motivo per
             * bloccare la macchina dietro FRST). SYS_FAULT resta riservato
             * ESCLUSIVAMENTE alla perdita di comunicazione RS485 (ramo
             * "else if" sotto, FAULT_BIT_LASEQ).
             *   bit 0-5 -> SYS_LASEQ_ERROR_EVENT, gate da SYS_ACTIVE.
             *   bit 7   -> SYS_LASEQ_ERROR_EVENT, gate da SYS_ON (vedi
             *              BUGFIX PWR_OK sotto, invariato da questo fix).
             *   bit 6   -> riservato, ignorato.
             * Entrambi gated dallo stesso bit ERR_BIT_LASEQ_INTERNAL
             * (error_mask): mascherarlo sospende la notifica per l'intera
             * categoria "condizione interna LaseQ".
             *
             * TaskMonitor_SetLaseQErrorCode()/SetErrorBit() aggiornano solo
             * la telemetria/bookkeeping (GET LQERR, GET ERR): non postano
             * eventi FSM, se ne occupa direttamente questo blocco (stesso
             * schema già usato sopra per FAULT_BIT_LASEQ/lq_fault_posted).
             */
            uint8_t err_code = st.error_code;

            if (err_code != lq_last_err_code) {
                if (err_code != 0U) {
                    char errname[96];
                    FSM_FormatErrorCode(err_code, errname, sizeof(errname));
                    SysLog_Event(LOG_ERROR, "LASEQ ERR 0x%02X: %s", err_code, errname);
                } else {
                    SysLog_Event(LOG_INFO, "LASEQ ERR cleared");
                }
                lq_last_err_code = err_code;
            }
            TaskMonitor_SetLaseQErrorCode(err_code);

            /*
             * Bookkeeping telemetria (GET ERR): attivo se QUALSIASI bit
             * rilevante (0-5 o 7) è impostato — indipendente dal gate di
             * stato usato sotto per decidere QUANDO postare l'evento FSM.
             */
            TaskMonitor_SetErrorBit(ERR_BIT_LASEQ_INTERNAL,
                                     (err_code & FSM_ERRCODE_RECOVERABLE_MASK) != 0U);

            /*
             * Bit 7 (FSM_FAULT_HW): gate su SYS_ON, non SYS_ACTIVE.
             * FSM_FAULT_HW significa "PWR_OK basso o interlock aperto" lato
             * LaseQ (fsm.h) — PWR_OK dipende da PSU1/PSU2, che sono
             * ESPLICITAMENTE spenti in SYS_ACTIVE (vedi fsm.h:38, "Water
             * cooling attivo, alimentatori spenti") e accesi solo da SYS_ON
             * in poi (fsm.h:39). Con il gate a SYS_ACTIVE, ogni SOFF
             * (SYS_ENABLED/EMISSION/ON -> SYS_ACTIVE, che spegne PSU1/2 in
             * action_enter_active()) farebbe leggere a LaseQ un PWR_OK
             * basso del tutto atteso. BUGFIX 2026-07-15 (v2): posta
             * SYS_LASEQ_ERROR_EVENT (SYS_ERROR) come gli altri bit, MAI più
             * SYS_LASEQ_FAULT_EVENT — vedi banner SEVERITÀ in fsm.h:
             * un'apertura dell'interlock LaseQ è un evento plausibile in
             * esercizio normale, non un motivo per bloccare la macchina
             * dietro il comando FRST protetto. Il clear_error necessario
             * per far uscire LaseQ dal proprio ERROR_STATE è già inviato
             * da action_clear_error() (fsm.c) via LaseQClearErr().
             */
            if ((err_code & FSM_FAULT_HW) != 0U) {
                if (!lq_hw_err_posted && FSM_GetState() >= SYS_ON &&
                    (g_config.error_mask & ERR_BIT_LASEQ_INTERNAL)) {
                    Queue_PostEvent(SYS_LASEQ_ERROR_EVENT);
                    lq_hw_err_posted = true;
                }
            } else {
                lq_hw_err_posted = false;
            }

            /*
             * Bit 0-5: comportamento storico invariato, gate da SYS_ACTIVE
             * (a differenza del bit 7 sopra, questi non dipendono da
             * PSU1/PSU2 acceso/spento). Elencati esplicitamente (invece di
             * "RECOVERABLE_MASK & ~FSM_FAULT_HW") per chiarezza: sono gli
             * unici bit FSM_ERR_* diversi dal bit 7/FSM_FAULT_HW gestito
             * a parte sopra.
             */
            if ((err_code & (FSM_ERR_TEMP_DRIVER | FSM_ERR_TEMP_AMBIENT |
                             FSM_ERR_VANODE | FSM_ERR_CURRENT_MON |
                             FSM_ERR_RS485_TIMEOUT | FSM_ERR_SATURATION)) != 0U) {
                if (!lq_err_posted && FSM_GetState() >= SYS_ACTIVE &&
                    (g_config.error_mask & ERR_BIT_LASEQ_INTERNAL)) {
                    Queue_PostEvent(SYS_LASEQ_ERROR_EVENT);
                    lq_err_posted = true;
                }
            } else {
                lq_err_posted = false;
            }
        } else if (FSM_GetState() >= SYS_ACTIVE) {
            /*
             * Il fault comms è rilevabile solo da SYS_ACTIVE in poi: sotto
             * (SYS_INIT/SYS_IDLE) la FSM scarta comunque SYS_LASEQ_FAULT_EVENT
             * di proposito. Incrementare/postare anche in quegli stati
             * "consumerebbe" lq_fault_posted con un post ignorato dalla FSM,
             * bloccando (via "!lq_fault_posted") il repost genuino una volta
             * raggiunto ACTIVE — LaseQ scollegato non andrebbe mai in fault.
             * Restando fuori da questo ramo mentre si è sotto ACTIVE,
             * lq_err_count/lq_fault_posted restano "vergini": appena si
             * entra in ACTIVE il conteggio riparte da zero e i 10 cicli
             * (200ms) contano per davvero.
             */
            lq_err_count++;
            /* 10 err × 20ms = 200ms di silenzio prima del fault.
             * Mascherabile con FAULT_BIT_LASEQ (g_config.fault_mask) — i
             * fault di comunicazione (LaseQ/AMC/COM interface) sono una
             * categoria separata da error_mask dal 2026-07-13, vedi
             * task_monitor.h. ERR_BIT_LASEQ (error_mask) è deprecato e non
             * più controllato qui. */
            if (lq_err_count >= 10U && !lq_fault_posted &&
                (g_config.fault_mask & FAULT_BIT_LASEQ)) {
                Queue_PostEvent(SYS_LASEQ_FAULT_EVENT);
                lq_fault_posted = true;
            }
            TaskMonitor_SetFaultBit(FAULT_BIT_LASEQ, lq_err_count >= 10U);
        }

        /* -------------------------------------------------------------- */
        /* 2b. COM interface: sorveglianza alimentazione (nCOM_PWR_FLT)   */
        /* -------------------------------------------------------------- */
        /*
         * Disaccoppiata dal ciclo AMC (2026-07-17): non gestisce piu' alcun
         * heartbeat/miss-count su nCOM_INT_IN (quel pin e' ora solo il
         * segnale "messaggio pronto", gestito dal task dedicato
         * task_com_interface.c, guidato da interrupt/semaforo, NON da
         * questo loop periodico). Qui resta solo il polling del fault di
         * alimentazione (nCOM_PWR_FLT), eseguito ogni ciclo (LQ_TRANSMIT_WINDOW).
         */
        COM_Interface_Update();

        /*
         * NOTA 2026-07-22: il servicing AMC (fault pin, heartbeat/status,
         * re-invio config/LUT su richiesta RS485, conversione PD->Watt)
         * vive ora in task_amc.c (task dedicato, stesso periodo
         * LQ_TRANSMIT_WINDOW) — vedi banner in cima a questo file.
         */

        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(LQ_TRANSMIT_WINDOW));
    }
}

/* ========================================================================== */
/* --- INIZIALIZZAZIONE --- */
/* ========================================================================== */

void TaskComms_Init(void)
{
    sem_tx = osSemaphoreNew(1, 0, NULL);
    sem_rx = osSemaphoreNew(1, 0, NULL);
    LaseQ_Init(&huart10, LASEQ_SLAVE_ADDR_DEFAULT, OnTxComplete, OnRxComplete);
    /* AMC_Init già chiamato in MX_FREERTOS_Init() prima della creazione dei task */
}
