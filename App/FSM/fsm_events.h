/*
 * fsm_events.h
 *
 * Definizione centralizzata di tutti gli eventi che possono essere
 * pubblicati nella coda della FSM di sistema.
 *
 * REGOLA: ogni sorgente di eventi (task_monitor, task_comms, task_inputs)
 * include questo header per scrivere nella coda. La FSM lo include per
 * leggere e processare gli eventi.
 *
 * AGGIUNGERE EVENTI: inserire il nuovo valore nell'enum SysEvent_t e
 * aggiornare la tabella di transizioni in fsm.c. Non modificare mai
 * i valori numerici esistenti (potrebbero essere salvati in log).
 */

#ifndef APP_FSM_FSM_EVENTS_H_
#define APP_FSM_FSM_EVENTS_H_

/* ========================================================================== */
/* --- EVENTI DI SISTEMA --- */
/* ========================================================================== */

typedef enum {

    /* --- Avvio e check iniziale --- */
    SYS_CHECK_OK_EVENT,             /**< Init completato senza errori -> IDLE */
    SYS_CHECK_FAIL_EVENT,           /**< Init fallito -> FAULT */

    /* --- Comandi operativi (da pulsante, interfaccia EXT, software) --- */
    SYS_START_EVENT,                /**< Avvio sistema -> ACTIVE */
    SYS_STOP_EVENT,                 /**< Arresto sistema -> IDLE */
    SYS_ON_EVENT,                   /**< Accensione alimentatori/driver -> ON */
    SYS_OFF_EVENT,                  /**< Spegnimento alimentatori/driver -> ACTIVE */
    SYS_ENABLE_EVENT,               /**< Abilitazione emissione -> ENABLED */
    SYS_DISABLE_EVENT,              /**< Disabilitazione emissione -> ON */
    SYS_LASERON_EVENT,              /**< Avvio emissione laser -> EMISSION */
    SYS_LASEROFF_EVENT,             /**< Stop emissione laser -> ENABLED */

    /* --- Input da pulsante fisico --- */
    SYS_BUTTON_PRESS_EVENT,         /**< Pressione singola */
    SYS_BUTTON_LONG_PRESS_EVENT,    /**< Pressione lunga */
    SYS_BUTTON_DOUBLE_PRESS_EVENT,  /**< Doppia pressione */

    /* --- Interfaccia EXT (connettore esterno) --- */
    SYS_EXT_CTRL_EVENT,             /**< Controllo esterno attivato */
    SYS_EXT_CTRL_OFF_EVENT,         /**< Controllo esterno disattivato */
    SYS_EXT_SYSON_EVENT,            /**< Comando SYS_ON da EXT */
    SYS_EXT_SYSOFF_EVENT,           /**< Comando SYS_OFF da EXT */
    SYS_EXT_ENABLE_EVENT,           /**< Comando ENABLE da EXT */
    SYS_EXT_DISABLE_EVENT,          /**< Comando DISABLE da EXT */
    SYS_EXT_CLR_ERR_EVENT,          /**< Clear errore da EXT */

    /* --- Gestione errori recuperabili --- */
    SYS_ERROR_EVENT,                /**< Errore recuperabile rilevato -> ERROR */
    SYS_LASEQ_ERROR_EVENT,          /**< error_code LaseQ, bit 0-5 e bit 7 (FSM_ErrorCode_t, fsm.h) ->
                                      *   ERROR. Postato da task_comms.c dopo una transazione RS485
                                      *   riuscita il cui MSG_STATUS.error_code ha almeno un bit
                                      *   rilevante attivo, gated da ERR_BIT_LASEQ_INTERNAL in
                                      *   g_config.error_mask (vedi task_monitor.h). Diverso da
                                      *   SYS_LASEQ_FAULT_EVENT sotto: qui la comunicazione FUNZIONA, è
                                      *   LaseQ stesso a segnalare una condizione interna (temperature,
                                      *   vanode, corrente, saturazione, RS485 watchdog lato LaseQ, bit
                                      *   0-5 — o PWR_OK basso/interlock aperto, bit 7, dal 2026-07-15
                                      *   v2: vedi banner SEVERITÀ in fsm.h, entrambi recuperabili via
                                      *   CERR, MAI più fault). */
    SYS_CLR_ERR_EVENT,              /**< Clear errore -> ACTIVE */

    /* --- Emissione (richiesta/stop via interfaccia EXT analogica o SW) --- */
    SYS_EMISSION_REQUEST_EVENT,     /**< Richiesta avvio emissione (gate HW) -> EMISSION */
    SYS_STOP_EMISSION_EVENT,        /**< Stop emissione -> ENABLED */

    /* --- Chiave di sicurezza --- */
    SYS_KEY_REMOVED_EVENT,          /**< Chiave rimossa durante operatività -> ERROR */

    /* --- Coperchio (dal 2026-07-15) --- */
    SYS_LID_OPEN_EVENT,             /**< Coperchio aperto durante operatività -> ERROR */

    /* --- SAB (Safety Arm Block) --- */
    SYS_SAB_INTERLOCK_OPEN_EVENT,   /**< Interlock SAB aperto inaspettatamente -> ERROR */
    SYS_SAB_TIMEOUT_EVENT,          /**< Timeout chiusura interlock SAB -> ERROR */
    SYS_SAB_TEST_FAIL_EVENT,        /**< Test SAB fallito (nSAB_A/B_TEST != 0) -> ERROR */

    /* --- Warning (non causano transizioni FSM, solo comunicazione esterna) ---
     *
     * I warning vengono intercettati in FSM_ProcessEvent() e scartati senza
     * consultare la tabella delle transizioni. Rimangono in questo enum perché
     * possono essere pubblicati nella coda dalla stessa Queue_PostEvent()
     * usata per gli eventi di stato. Il task_comms li legge dal registro
     * TaskMonitor_GetWarnings() per includerli nella telemetria RS485/COM.
     */
    SYS_HUMIDITY_WARN_EVENT,        /**< Umidità ambiente fuori soglia (MMC o LaseQ) */
    SYS_TEMP_WARN_EVENT,            /**< Temperatura fuori soglia warn (non critica) */

    /* --- Fault non recuperabili (sicurezza) ---
     *
     * NOTA: SYS_FLOW_ERROR_EVENT e SYS_TEMP_ERROR_EVENT restano elencati
     * QUI (posizione/valore numerico invariati, mai rinumerare l'enum — vedi
     * banner in cima al file) ma NON sono fault non recuperabili: portano a
     * SYS_ERROR (recuperabile via CERR), non a SYS_FAULT — da cui il suffisso
     * _ERROR_EVENT invece di _FAULT_EVENT (rinominati dal 2026-07-14 per
     * coerenza con la convenzione ERROR=recuperabile / FAULT=non
     * recuperabile, vedi state_machine[] in fsm.c).
     */
    SYS_FAULT_EVENT,                /**< Fault critico -> FAULT */
    SYS_FLOOD_FAULT_EVENT,          /**< Allagamento rilevato -> FAULT */
    SYS_FLOW_ERROR_EVENT,           /**< Flusso acqua insufficiente -> ERROR */
    SYS_TEMP_ERROR_EVENT,           /**< Temperatura fuori soglia critica -> ERROR */
    SYS_PSU_FAULT_EVENT,            /**< Fault alimentatore -> FAULT */
    SYS_LASEQ_FAULT_EVENT,          /**< -> FAULT. Postato da task_comms.c ESCLUSIVAMENTE per perdita
                                      *   comunicazione RS485 verso LaseQ (10 cicli falliti, gated da
                                      *   FAULT_BIT_LASEQ in g_config.fault_mask).
                                      *   BUGFIX 2026-07-15 (v2): prima esisteva anche un secondo caso
                                      *   (MSG_STATUS.error_code bit 7/FSM_FAULT_HW attivo — PWR_OK
                                      *   basso o interlock aperto lato LaseQ) che portava anch'esso
                                      *   qui; rimosso perché un'apertura dell'interlock LaseQ è un
                                      *   evento plausibile in esercizio normale, non un motivo per
                                      *   bloccare la macchina dietro il comando FRST protetto — ora
                                      *   quel bit porta a SYS_LASEQ_ERROR_EVENT (SYS_ERROR, sotto),
                                      *   vedi banner SEVERITÀ in fsm.h e ERR_BIT_LASEQ_INTERNAL in
                                      *   task_monitor.h. */
    SYS_AMC_FAULT_EVENT,            /**< Perdita comunicazione AMC -> FAULT */
    SYS_COM_FAULT_EVENT,            /**< Perdita comunicazione / power fault COM interface -> FAULT
                                      *   Postato da COM_Interface.c (nCOM_PWR_FLT o timeout
                                      *   heartbeat nCOM_INT_IN), gated da FAULT_BIT_COM in
                                      *   g_config.fault_mask (vedi task_monitor.h). */

    /* --- Comandi di cambio modalità (da COM/RS485) ---
     *
     * Gestiti in FSM_ProcessEvent() PRIMA della tabella di transizioni:
     * non causano cambio di stato FSM, modificano solo s_mode (restando
     * nello stato corrente). Accettati da SYS_IDLE, SYS_ACTIVE, SYS_ON;
     * ignorati da SYS_ENABLED/SYS_EMISSION/SYS_ERROR/SYS_FAULT.
     *
     * SYS_SET_MODE_HYBRID_EVENT: FSM_Mode_t non ha più un valore HYBRID
     * dedicato (vedi fsm.h) — dal 2026-07-07 HYBRID1/HYBRID2 sono due
     * toggle indipendenti (SYS_SET_GATE_HW_* / SYS_SET_SETPOINT_HW_* sotto)
     * validi solo in FSM_MODE_SW. Questo evento resta definito (i valori
     * numerici di questo enum non vanno mai cambiati, potrebbero essere
     * salvati in log) e viene mantenuto come alias legacy: impone
     * FSM_MODE_SW e attiva entrambi i toggle (equivalente alla vecchia
     * HYBRID unica), per compatibilità con client RS485/COM esistenti.
     */
    SYS_SET_MODE_SW_EVENT,          /**< Richiesta esplicita modalità SW (da COM/RS485)     */
    SYS_SET_MODE_HYBRID_EVENT,      /**< Alias legacy: SW + entrambi i toggle GATE_HW/SETPOINT_HW (da COM/RS485) */
    SYS_SET_MODE_ANALOG_EVENT,      /**< Richiesta esplicita modalità ANALOG (da COM/RS485) */

    /* --- Toggle indipendenti "HYBRID1"/"HYBRID2", validi solo in FSM_MODE_SW ---
     *
     * Gestiti in FSM_ProcessEvent() insieme ai comandi SET MODE sopra:
     * stesso range di stati ammessi (IDLE/ACTIVE/ON), più il vincolo
     * aggiuntivo s_mode == FSM_MODE_SW. Azzerati automaticamente ad ogni
     * cambio di modalità (SET MODE SW/ANALOG/HYBRID, o pin nEXT_CTL_iso) —
     * vedi reset_hybrid_toggles() in fsm.c.
     */
    SYS_SET_GATE_HW_ON_EVENT,       /**< "HYBRID1": abilita nGATE_HW_EN pur restando in SW mode */
    SYS_SET_GATE_HW_OFF_EVENT,      /**< Disabilita nGATE_HW_EN (default) */
    SYS_SET_SETPOINT_HW_ON_EVENT,   /**< "HYBRID2": manda analog_mode=1 a LaseQ pur restando in SW mode */
    SYS_SET_SETPOINT_HW_OFF_EVENT,  /**< Disabilita il setpoint HW (default) */

    /* --- Reset fault non recuperabile (da COM/RS485, protetto da login) ---
     *
     * SYS_FAULT_RESET_EVENT: unica uscita possibile da SYS_FAULT (che
     * altrimenti non ha transizioni in uscita nella tabella — vedi fsm.c).
     * Riporta a SYS_IDLE, ripulisce il fault latch persistente
     * (g_config.fault_latch_active, vedi FAULT LATCH in task_monitor.h) e i
     * flag di debounce in task_monitor.c che altrimenti ri-triggererebbero
     * subito lo stesso fault. Comando RS485 "FRST" (protetto da login) —
     * vedi FSM_RequestFaultReset() in fsm.h. Non confondere con
     * SYS_CLR_ERR_EVENT (quello è per SYS_ERROR, recuperabile senza
     * restrizioni di login).
     */
    SYS_FAULT_RESET_EVENT,          /**< Reset esplicito di SYS_FAULT -> IDLE (protetto) */

    /* --- Sentinel: deve rimanere l'ultimo elemento --- */
    SYS_NUM_EVENTS

} SysEvent_t;

#endif /* APP_FSM_FSM_EVENTS_H_ */
