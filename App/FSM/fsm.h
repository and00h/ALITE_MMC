/*
 * fsm.h
 *
 * Interfaccia pubblica della macchina a stati di sistema.
 *
 * ARCHITETTURA:
 *   La FSM è implementata come tabella di transizioni bidimensionale
 *   [stato_corrente][evento] -> {stato_prossimo, action_fn}.
 *   Ogni cella contiene il puntatore alla funzione di transizione (action)
 *   da eseguire e lo stato di destinazione.
 *
 *   Le action vengono eseguite DURANTE la transizione (transition action),
 *   non all'ingresso/uscita degli stati. Questo modello è più semplice da
 *   implementare e sufficiente per questo sistema.
 *
 * THREAD SAFETY:
 *   FSM_ProcessEvent() è l'unico punto di modifica dello stato corrente.
 *   Deve essere chiamata esclusivamente dal task_fsm, mai da ISR o altri task.
 *   Gli altri task comunicano con la FSM SOLO tramite la coda eventi (queues.h).
 */

#ifndef APP_FSM_FSM_H_
#define APP_FSM_FSM_H_

#include "fsm_events.h"
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

/* ========================================================================== */
/* --- STATI DI SISTEMA --- */
/* ========================================================================== */

typedef enum {

    SYS_INIT = 0,   /**< Avvio: init hardware, lettura config, check periferiche */
    SYS_IDLE,       /**< Sistema pronto, alimentatori spenti, water cooling spento */
    SYS_ACTIVE,     /**< Water cooling attivo, alimentatori spenti, pronto per ON */
    SYS_ON,         /**< Alimentatori ON, driver corrente disabilitato */
    SYS_ENABLED,    /**< Driver corrente abilitato, emissione non ancora attiva */
    SYS_EMISSION,   /**< Emissione laser attiva */
    SYS_ERROR,      /**< Errore recuperabile: emissione disabilitata, attesa clear */
    SYS_FAULT,      /**< Fault critico: sistema bloccato, richiede intervento */

    SYS_NUM_STATES  /**< Sentinel: numero totale di stati */

} SysState_t;

/* ========================================================================== */
/* --- TIPO DELLA TABELLA DI TRANSIZIONI --- */
/* ========================================================================== */

/*
 * Funzione di transizione: eseguita atomicamente quando la FSM processa
 * un evento. Riceve lo stato di partenza e l'evento per permettere
 * action condivise tra più transizioni (es. sys_fault usata da tutti gli stati).
 */
typedef void (*FSM_ActionFn_t)(SysState_t from, SysEvent_t event);

typedef struct {
    SysState_t    next_state;   /**< Stato di destinazione (SYS_NUM_STATES = transizione non definita) */
    FSM_ActionFn_t action;      /**< Funzione da eseguire durante la transizione (NULL = nessuna azione) */
} SysStateTransition_t;

/* ========================================================================== */
/* --- MODALITÀ OPERATIVA --- */
/* ========================================================================== */

/*
 * La modalità determina chi comanda le transizioni ON/ENABLED/EMISSION.
 * Solo due valori: FSM_MODE_SW e FSM_MODE_ANALOG (HYBRID NON è più un
 * valore di s_mode dedicato — vedi "TOGGLE INDIPENDENTI" sotto).
 *
 * REGOLA DI ISOLAMENTO MODALE (revisionata 2026-07-27, vedi blocchi dedicati
 * in FSM_ProcessEvent()):
 *   All'avvio s_mode è sempre FSM_MODE_SW (sicuro, intenzione utente ignota).
 *
 *   nEXT_CTL_iso LOW (SYS_EXT_CTRL_EVENT, task_inputs.c):
 *     → s_mode = FSM_MODE_ANALOG, in QUALUNQUE stato tranne SYS_INIT/
 *       SYS_FAULT (no-op se già ANALOG — fronte ridondante, nessuna
 *       conseguenza). Effetto sullo STATO, secondo lo stato di provenienza:
 *         SYS_IDLE               → sale a SYS_ACTIVE (action_ext_ctrl_on()):
 *           altrimenti l'esterno resterebbe bloccato in IDLE (nSYS_ON_iso/
 *           nENABLE_IN_iso funzionano solo da ACTIVE/ON, comandi SW
 *           bloccati in ANALOG).
 *         SYS_ENABLED/SYS_EMISSION → scende a SYS_ON (stessa sicurezza di
 *           SDIS/EXT_DISABLE, action_enter_on()): non si resta armati
 *           cambiando canale di controllo.
 *         Altrove (ACTIVE/ON/ERROR) → stato invariato, solo s_mode cambia.
 *       Azzera anche i toggle indipendenti (vedi sotto).
 *
 *   SYS_SET_MODE_SW_EVENT / SYS_SET_MODE_ANALOG_EVENT (COM/RS485):
 *     → Accettati da SYS_IDLE, SYS_ACTIVE, SYS_ON (stato invariato) e da
 *       SYS_ENABLED/SYS_EMISSION (scende SEMPRE a SYS_ON, in QUALUNQUE
 *       direzione — stesso motivo del pin sopra). Ignorati da ERROR/FAULT.
 *       A differenza del pin fisico, da SYS_IDLE NON forzano mai ACTIVE.
 *       Azzerano anche i toggle indipendenti.
 *
 *   SYS_SET_MODE_HYBRID_EVENT (COM/RS485):
 *     → Alias legacy, stesso range di stati di SW/ANALOG sopra: equivale a
 *       SYS_SET_MODE_SW_EVENT seguito dall'attivazione di ENTRAMBI i toggle
 *       indipendenti sotto (comportamento byte-per-byte identico alla
 *       vecchia FSM_MODE_HYBRID unica).
 *
 * TOGGLE INDIPENDENTI "HYBRID1"/"HYBRID2" (validi solo in FSM_MODE_SW):
 *   SYS_SET_GATE_HW_ON/OFF_EVENT      → "HYBRID1": nGATE_HW_EN abilitato,
 *       byte di controllo LaseQ IDENTICI a SW puro (sw_control=1,
 *       analog_mode=0). Il gate resta comunque comandato via firmware
 *       (nGATE_MC, LaseQGate()) — invariato rispetto a SW.
 *   SYS_SET_SETPOINT_HW_ON/OFF_EVENT  → "HYBRID2": LaseQ riceve
 *       analog_mode=1 (setpoint di potenza valido = quello hardware), ma
 *       il comportamento della FSM di MMC resta quello di SW mode in
 *       tutto il resto (nessun'altra differenza).
 *   Accettati con lo stesso range di stati di SET MODE (IDLE/ACTIVE/ON,
 *   SENZA l'estensione a ENABLED/EMISSION di SET MODE sopra), con il
 *   vincolo aggiuntivo s_mode == FSM_MODE_SW. Azzerati automaticamente a
 *   ogni cambio di modalità (vedi reset_hybrid_toggles() in fsm.c).
 *
 * BLOCCO BIDIREZIONALE in FSM_ProcessEvent() (isolamento modale):
 *   FSM_MODE_ANALOG → ignora: SYS_START, SYS_ON, SYS_ENABLE, SYS_LASERON,
 *                              SYS_EMISSION_REQUEST, SYS_CLR_ERR, BUTTON_*.
 *   FSM_MODE_SW     → ignora: SYS_EXT_SYSON, SYS_EXT_SYSOFF,
 *                              SYS_EXT_ENABLE, SYS_EXT_DISABLE,
 *                              SYS_EXT_CLR_ERR, SYS_EXT_CTRL_OFF.
 *   SYS_EXT_SYSON/SYSOFF/ENABLE/DISABLE/CLR_ERR (pin fisici) hanno inoltre
 *   effetto SOLO quando s_mode == FSM_MODE_ANALOG (gate applicato a monte,
 *   nel polling stesso — vedi Inputs_PollSysOn/PollEnableIn/PollClrErr in
 *   task_inputs.c — non in questo filtro).
 *
 * USCITA DA ANALOG:
 *   nEXT_CTL_iso HIGH (SYS_EXT_CTRL_OFF_EVENT) → SEMPRE SYS_IDLE (qualunque
 *   stato di provenienza, ERROR incluso) + s_mode = FSM_MODE_SW, toggle
 *   indipendenti azzerati. No-op se s_mode è già SW (filtrato a monte).
 *
 * OUTPUT nCMD_RDY_iso: segue SEMPRE e SOLO s_mode (mai lo stato FSM) —
 *   asserito (LOW) in ANALOG, deasserito (HIGH) in SW — vedi
 *   sync_cmd_rdy_status() in fsm.c, chiamata ad ogni punto sopra.
 *
 * PROTOCOLLO AMC (compatibilità): il byte "laser_mode" nell'heartbeat verso
 * AMC (AMC_protocol.h, condiviso tra le due board) documenta 0=SW,
 * 1=HYBRID, 2=ANALOG — valori STORICI indipendenti dalla numerazione
 * interna di questo enum. Vedi FSM_GetLaserModeWire() sotto: non castare
 * mai FSM_Mode_t direttamente sul wire.
 */
typedef enum {
    FSM_MODE_SW     = 0,   /**< Gate e setpoint via firmware (RS485), salvo i toggle indipendenti sopra */
    FSM_MODE_ANALOG = 1,   /**< Gate e setpoint via hardware EXT (nGATE_IN_iso) */
} FSM_Mode_t;

/* ========================================================================== */
/* --- ERROR_CODE LASEQ (MSG_STATUS byte 1, protocollo RS485 verso LaseQ) ---
 *
 * Bitmask OR-accumulata (più condizioni possono essere attive insieme)
 * riportata da LaseQ nel campo `error_code` di LaseQ_status_vars_t/
 * LaseQ_PayloadStatus_t (vedi LaseQ.h/laseq_protocol.h), decodificata da
 * task_comms.c dopo ogni transazione RS485 riuscita (laseq_transact() ==
 * LASEQ_OK). Diversa categoria rispetto a FAULT_BIT_LASEQ (task_monitor.h):
 * quella rileva la PERDITA di comunicazione (nessuna risposta valida per
 * 10 cicli), questa decodifica invece il CONTENUTO di una risposta valida
 * che segnala una condizione interna di LaseQ.
 *
 * SEVERITÀ — BUGFIX 2026-07-15 (v2, "interlock plausibile"):
 *   TUTTI i bit di error_code (0-5 e 7) → SYS_LASEQ_ERROR_EVENT,
 *   transizione a SYS_ERROR (recuperabile via CERR — vedi fsm_events.h).
 *   bit 6 → riservato/non usato, ignorato.
 *
 *   Prima di questo bugfix, il bit 7 (FSM_FAULT_HW, nome storico) veniva
 *   escalato a SYS_LASEQ_FAULT_EVENT → SYS_FAULT (non recuperabile,
 *   richiede "FRST" protetto da login). Verificato sul firmware LaseQ4
 *   (App/Tasks/hw_signals_handler.c, banner "FILOSOFIA ERROR vs FAULT"):
 *   LaseQ stesso tratta OGNI condizione hardware — OCP, OVP, PWR_OK basso
 *   E interlock aperto, tutte incluse in questo bit — come EVENT_ERROR
 *   (ERROR_STATE, recuperabile via EVENT_RESET/clear_error), MAI come
 *   EVENT_FAULT (riservato a criticità firmware — config corrotta,
 *   assert — mai generato in pratica). Un'apertura dell'interlock è un
 *   evento plausibile in normale esercizio (sportello aperto per
 *   manutenzione, ecc.): bastano l'interlock richiuso + un clear_error
 *   (già inviato da action_clear_error(), fsm.c, tramite LaseQClearErr())
 *   per tornare operativi — non c'era motivo di bloccare l'intera
 *   macchina dietro il comando FRST protetto.
 *
 *   SYS_FAULT (non recuperabile) resta riservato ESCLUSIVAMENTE alla
 *   PERDITA di comunicazione RS485 (FAULT_BIT_LASEQ, task_monitor.h — 10
 *   cicli falliti = 200ms): quella condizione sì giustifica il fault,
 *   perché la macchina non può più sapere in che stato sia il driver.
 *
 *   Gate diversi in task_comms.c per motivi storici (invariati da questo
 *   bugfix): bit 0-5 verificati da SYS_ACTIVE in poi, bit 7 SOLO da
 *   SYS_ON in poi (PWR_OK dipende da PSU1/PSU2, spenti in SYS_ACTIVE —
 *   vedi commento in task_comms.c). Entrambi i rami condividono lo stesso
 *   bit di maschera ERR_BIT_LASEQ_INTERNAL (error_mask).
 */
typedef enum {
    FSM_ERR_TEMP_DRIVER   = 0x01U, /**< Almeno uno dei 4 elementi di potenza fuori soglia temperatura */
    FSM_ERR_TEMP_AMBIENT  = 0x02U, /**< Temperatura ambiente (SHT35) fuori soglia */
    FSM_ERR_VANODE        = 0x04U, /**< Tensione anodo fuori range */
    FSM_ERR_CURRENT_MON   = 0x08U, /**< Almeno uno dei 4 canali con corrente monitorata fuori soglia */
    FSM_ERR_RS485_TIMEOUT = 0x10U, /**< Timeout comunicazione RS485 (watchdog), rilevato lato LaseQ */
    FSM_ERR_SATURATION    = 0x20U, /**< Comparatore di saturazione scattato su almeno un canale */
    /* Bit 6 (0x40): riservato, non usato — ignorato in decodifica */
    FSM_FAULT_HW          = 0x80U, /**< PWR_OK basso o interlock aperto (nome storico "FAULT_HW" — vedi banner sopra: -> SYS_ERROR, non SYS_FAULT) */
} FSM_ErrorCode_t;

/**
 * Tutti i bit di error_code che portano a SYS_LASEQ_ERROR_EVENT (SYS_ERROR):
 * bit 0-5 (FSM_ERR_*) + bit 7 (FSM_FAULT_HW) — vedi banner SEVERITÀ sopra.
 * NESSUN bit di error_code porta più a SYS_FAULT (solo la perdita di
 * comunicazione RS485, FAULT_BIT_LASEQ in task_monitor.h, ci porta).
 */
#define FSM_ERRCODE_RECOVERABLE_MASK  0xBFU  /**< bit 0-5 e 7: FSM_ERR_* e FSM_FAULT_HW -> SYS_ERROR */

/**
 * @brief  Decodifica error_code in una stringa leggibile con i nomi dei bit
 *         attivi separati da '|' (es. "TEMP_DRIVER|VANODE"), oppure "NONE"
 *         se error_code == 0. Usata per il log SD (SysLog_Event) e per il
 *         comando RS485 "GET LQERR" (rs485_cmd.c).
 * @param  error_code  Bitmask grezza (FSM_ErrorCode_t OR-accumulata).
 * @param  buf         Buffer di destinazione.
 * @param  buf_size    Dimensione di buf.
 */
void FSM_FormatErrorCode(uint8_t error_code, char *buf, size_t buf_size);

/*
 * DETTAGLIO DI FSM_FAULT_HW (Luca, 2026-07-16)
 * ============================================================================
 * FSM_FAULT_HW (bit7 sopra) è un unico bit per tre cause hardware distinte
 * lato LaseQ: guasto genuino su PWR_OK, interlock non confermato al momento
 * dell'abilitazione, interlock aperto durante ENABLED oltre la finestra di
 * grazia (100ms -> 500ms, vedi Lase-Q4_v2.1/App/FSM/FSM.c). Mirror ESATTO
 * dei valori di FSM_HwFaultSource_t in Lase-Q4_v2.1/App/FSM/FSM.h — tenere
 * sincronizzati manualmente (stesso schema già in uso per FSM_ErrorCode_t
 * stesso, che è a sua volta un mirror). Trasmesso da LaseQ nel payload
 * MSG_STATUS.hw_fault_source (protocollo v0x0003, laseq_protocol.h) e
 * copiato in LaseQ_status_vars_t.hw_fault_source (LaseQ.h/.c). Decodificato
 * in "GET LQ" (rs485_cmd.c) accanto a error_code.
 */
typedef enum {
    HW_FAULT_NONE              = 0x00U,
    HW_FAULT_PWR_OK            = 0x01U, /**< PWR_OK basso (guasto linea alimentazione stadio driver) */
    HW_FAULT_INTERLOCK_ENABLE  = 0x02U, /**< Interlock non confermato al momento dell'abilitazione */
    HW_FAULT_INTERLOCK_RUNTIME = 0x04U, /**< Interlock apertosi durante ENABLED, non confermato entro la finestra di grazia */
} FSM_HwFaultSource_t;

/**
 * @brief  Decodifica hw_fault_source in una stringa leggibile (es.
 *         "INTLCK_RUNTIME"), oppure "NONE" se 0. Stesso schema di
 *         FSM_FormatErrorCode(). Usata da "GET LQ" (rs485_cmd.c).
 * @param  hw_fault_source  Bitmask grezza (FSM_HwFaultSource_t OR-accumulata).
 * @param  buf              Buffer di destinazione.
 * @param  buf_size         Dimensione di buf.
 */
void FSM_FormatHwFaultSource(uint8_t hw_fault_source, char *buf, size_t buf_size);

/* ========================================================================== */
/* --- API PUBBLICA --- */
/* ========================================================================== */

/**
 * @brief  Inizializza la FSM portandola allo stato SYS_INIT.
 *         Da chiamare una sola volta all'avvio del task_fsm.
 */
void FSM_Init(void);

/**
 * @brief  Processa un evento: cerca la transizione nella tabella,
 *         esegue l'action e aggiorna lo stato corrente.
 * @note   Se la transizione non è definita per la coppia (stato, evento),
 *         l'evento viene silenziosamente scartato.
 * @param  event  Evento da processare.
 */
void FSM_ProcessEvent(SysEvent_t event);

/**
 * @brief  Avanzamento "a tempo" della FSM, indipendente dagli eventi.
 *
 *         Da chiamare periodicamente dal loop di TaskFSM_Run (ad ogni giro,
 *         sia che osMessageQueueGet ritorni un evento sia che scada per
 *         timeout). Gestisce due meccanismi temporali indipendenti, entrambi
 *         requisiti hardware fissi legati al modulo SAB (vedi fsm.c):
 *
 *         1. Guardia 10s su spegnimento SAB (s_sab_poweroff_pending): quando
 *            la finestra dal fronte di discesa di nSAB_EN è scaduta,
 *            rimuove l'alimentazione del modulo SAB (SAB_ShutdownSupply()).
 *            Vedi banner "GUARDIA 10s" in SAB.h e sab_request_shutdown() in
 *            fsm.c.
 *
 *         2. SAB_BOOT_DISCHARGE (rimovibile, vedi banner "FEATURE-FLAG:
 *            SAB_BOOT_DISCHARGE" in fsm.c): avanza la routine di scarica
 *            capacità SAB eseguita al boot, avviata da
 *            FSM_StartBootDischarge() e interrogabile con
 *            FSM_IsBootDischargeDone().
 *
 * @note   Non tocca lo stato della FSM, non deve mai postare eventi che
 *         causino transizioni: usa solo API idempotenti (SAB on/off/pulse).
 *
 * @note   Questa dichiarazione e la chiamata da task_fsm.c vanno mantenute
 *         finché resta attivo almeno uno dei due blocchi sopra.
 */
void FSM_Tick(void);

/**
 * @brief  SAB_BOOT_DISCHARGE (rimovibile, vedi banner "FEATURE-FLAG:
 *         SAB_BOOT_DISCHARGE" in fsm.c). Avvia la routine di scarica
 *         capacità SAB al boot (SAB_PowerOn() → attesa stabilizzazione
 *         SAB_GUARD_DELAY_MS → impulso nSAB_EN 10ms → nSAB_EN HIGH/fronte
 *         di salita che innesca la scarica → attesa 5s → SAB_PowerOff()),
 *         non bloccante, avanzata da FSM_Tick(). Da chiamare UNA SOLA
 *         VOLTA, subito dopo FSM_Init() e dopo l'inizializzazione di tutte
 *         le periferiche, PRIMA di TaskFSM_RunInitSequence() (vedi
 *         task_fsm.c).
 */
void FSM_StartBootDischarge(void);

/**
 * @brief  SAB_BOOT_DISCHARGE (rimovibile, vedi sopra). Restituisce true
 *         quando la routine di scarica al boot è terminata (SAB rispento).
 *         Da interrogare in task_fsm.c prima di eseguire
 *         TaskFSM_RunInitSequence().
 */
bool FSM_IsBootDischargeDone(void);

/**
 * @brief  SAB_REARM_BLOCK (rimovibile, vedi banner "FEATURE-FLAG:
 *         SAB_REARM_BLOCK" in fsm.c). True se il riarmo SAB (transizione
 *         SYS_ON→SYS_ENABLED) è attualmente bloccato: la finestra di
 *         scarica capacità (~10s) innescata dall'ultima uscita da
 *         SYS_ENABLED/SYS_EMISSION verso SYS_ON non è ancora trascorsa.
 *         Da interrogare in rs485_cmd.c (cmd_sen()) PRIMA di rispondere OK,
 *         per evitare una risposta fuorviante quando SYS_ENABLE_EVENT
 *         verrebbe comunque scartato da FSM_ProcessEvent().
 */
bool FSM_IsSabRearmBlocked(void);

/**
 * @brief  Restituisce lo stato corrente della FSM.
 * @note   Lettura sicura da altri task (lo stato è un intero, lettura atomica su ARM).
 */
SysState_t FSM_GetState(void);

/**
 * @brief  Restituisce la modalità operativa corrente.
 * @retval FSM_MODE_SW o FSM_MODE_ANALOG.
 */
FSM_Mode_t FSM_GetMode(void);

/**
 * @brief  Restituisce lo stato del toggle indipendente "HYBRID1"
 *         (SYS_SET_GATE_HW_ON/OFF_EVENT). Significativo solo quando
 *         FSM_GetMode() == FSM_MODE_SW; in FSM_MODE_ANALOG il gate HW è
 *         sempre abilitato indipendentemente da questo flag.
 */
bool FSM_GetGateHwEnabled(void);

/**
 * @brief  Restituisce lo stato del toggle indipendente "HYBRID2"
 *         (SYS_SET_SETPOINT_HW_ON/OFF_EVENT). Significativo solo quando
 *         FSM_GetMode() == FSM_MODE_SW; in FSM_MODE_ANALOG il setpoint HW
 *         è sempre attivo indipendentemente da questo flag.
 */
bool FSM_GetSetpointHwEnabled(void);

/**
 * @brief  Byte "laser_mode" da inviare ad AMC (AMC_Transact(), heartbeat
 *         protocollo AMC_protocol.h), con la codifica STORICA del
 *         protocollo — 0=SW, 1=HYBRID, 2=ANALOG — indipendente dalla
 *         numerazione interna di FSM_Mode_t (che non ha più un valore
 *         HYBRID dedicato). NON castare mai FSM_GetMode() direttamente:
 *         usare sempre questa funzione per costruire il byte di wire.
 *
 * @retval 0 se FSM_MODE_SW e nessun toggle attivo
 * @retval 1 se FSM_MODE_SW con FSM_GetSetpointHwEnabled() attivo ("HYBRID2")
 * @retval 2 se FSM_MODE_ANALOG
 */
uint8_t FSM_GetLaserModeWire(void);

/**
 * @brief  Restituisce l'ultimo evento che ha causato un fault critico
 *         NON recuperabile (SYS_FAULT — es. FLOOD, PSU, LASEQ_COMMS,
 *         AMC_COMMS, COM_COMMS, CHECK_FAIL). NON include le cause di
 *         SYS_ERROR (recuperabile) — vedi FSM_GetLastErrorEvent() sotto.
 * @retval SYS_NUM_EVENTS se non si è mai verificato alcun fault dall'avvio.
 * @note   NON viene azzerato da reset di stato: resta l'ultima causa nota
 *         finché non se ne verifica una nuova. Utile per diagnosi a
 *         posteriori via RS485 (comando "GET FAULT").
 * @note   BUGFIX (2026-07-14): prima di questa data GET FAULT poteva
 *         restituire anche cause di SYS_ERROR (es. "KEY"), fuorviante dato
 *         il nome del comando — vedi banner sulle variabili statiche in
 *         fsm.c per i dettagli della separazione.
 */
SysEvent_t FSM_GetLastFaultEvent(void);

/**
 * @brief  Come FSM_GetLastFaultEvent(), ma restituisce direttamente la
 *         stringa leggibile (es. "FLOOD", "LASEQ_COMMS", "NONE"
 *         se nessun fault si è mai verificato).
 */
const char *FSM_GetLastFaultName(void);

/**
 * @brief  Restituisce l'ultimo evento che ha causato un errore
 *         RECUPERABILE (SYS_ERROR — es. KEY, SAB_TIMEOUT, SAB_INTLCK,
 *         FLOW, TEMP, LASEQ_ERROR), clearabile via CERR. NON include le
 *         cause di SYS_FAULT (non recuperabile) — vedi
 *         FSM_GetLastFaultEvent() sopra.
 * @retval SYS_NUM_EVENTS se non si è mai verificato alcun errore dall'avvio.
 * @note   NON viene azzerato da CLR_ERR: resta l'ultima causa nota finché
 *         non se ne verifica una nuova. Utile per diagnosi a posteriori
 *         via RS485 (comando "GET ALARM").
 */
SysEvent_t FSM_GetLastErrorEvent(void);

/**
 * @brief  Come FSM_GetLastErrorEvent(), ma restituisce direttamente la
 *         stringa leggibile (es. "KEY", "FLOW", "NONE" se nessun errore
 *         si è mai verificato).
 */
const char *FSM_GetLastErrorName(void);

/**
 * @brief  Richiede il reset di un fault non recuperabile (SYS_FAULT),
 *         incluso un fault latched persistito da un boot precedente (vedi
 *         banner "FAULT LATCH" in task_monitor.h). Posta
 *         SYS_FAULT_RESET_EVENT nella coda FSM: ha effetto SOLO se lo stato
 *         corrente è SYS_FAULT (altrimenti scartato in silenzio, nessuna
 *         transizione definita per gli altri stati — vedi state_machine[]
 *         in fsm.c). Riporta a SYS_IDLE, ripulisce
 *         g_config.fault_latch_active (persistito) e i flag di debounce
 *         interni al monitor (task_monitor.c) — vedi action_fault_reset().
 *
 * @note   Da chiamare SOLO dopo un controllo di autenticazione — comando
 *         RS485 "FRST" (rs485_cmd.c, protetto da login) o un futuro
 *         handler COM interface (protocollo applicativo non ancora
 *         definito). MAI da un comando operativo libero: un reset di
 *         SYS_FAULT non richiede solo l'autorizzazione ma la consapevolezza
 *         esplicita dell'operatore che la causa è stata verificata/risolta.
 */
void FSM_RequestFaultReset(void);

/**
 * @brief  True se un fault attualmente attivo è "latched" — cioè
 *         g_config.fault_latch_active intersecato con fault_latch_mask è
 *         non nullo, quindi richiede un comando "FRST" esplicito per essere
 *         superato, anche dopo un power-cycle (vedi banner "FAULT LATCH" in
 *         task_monitor.h). Utile per diagnostica RS485 (distingue un
 *         SYS_FAULT "normale", che un power-cycle cancella, da uno
 *         latched, che richiede intervento esplicito).
 */
bool FSM_IsFaultLatched(void);

/**
 * @brief  Restituisce il valore corrente di HW_SETPOINT_SEL.
 *
 * 0 = LPWR_SET_ISO grezzo (default, AMC DAC non attivo)
 * 1 = DAC di AMC (segnale corretto/linearizzato)
 *
 * Utile a task_comms per comunicare ad AMC se il suo DAC è atteso attivo.
 * Il valore è letto da g_config.hw_setpoint_sel una sola volta in FSM_Init().
 */
uint8_t FSM_GetHwSetpointSel(void);

/**
 * @brief  Imposta a runtime la copia live di HW_SETPOINT_SEL usata da
 *         FSM (action_enter_emission()) e da task_amc.c (heartbeat verso
 *         AMC) — s_hw_setpoint_sel, altrimenti aggiornata solo una volta
 *         in FSM_Init() da g_config.hw_setpoint_sel.
 *
 * NON scrive g_config.hw_setpoint_sel né il pin PF14: è compito del
 * chiamante tenerli allineati (vedi comando RS485 "SET SETPOINTCOMP
 * ON|OFF", rs485_cmd.c, che aggiorna config+pin+questa copia live insieme,
 * stesso schema di "SET TERM ON|OFF").
 *
 * @param  sel  0 = LPWR_SET_ISO grezzo (EXT), 1 = DAC di AMC.
 */
void FSM_SetHwSetpointSel(uint8_t sel);

#endif /* APP_FSM_FSM_H_ */
