/*
 * SAB.h
 *
 * Driver per il modulo SAB (Safety Arm Block).
 *
 * HARDWARE:
 *   nSAB_PWR_SHDN    PC1    GPIO_Output, active LOW  – alimentazione modulo SAB
 *                           LOW = spento, HIGH = alimentato
 *   nSAB_EN          PA1    GPIO_Output, active LOW  – abilita il modulo SAB
 *                           HIGH = disabilitato, LOW = abilitato
 *   nSAB_INTLCK_A_STATUS  GPIO_Input – stato interlock canale A (LOW = chiuso/OK, HIGH = aperto/fault)
 *   nSAB_INTLCK_B_STATUS  GPIO_Input – stato interlock canale B (LOW = chiuso/OK, HIGH = aperto/fault)
 *   nSAB_A_TEST      GPIO_Input – diagnostica canale A (HIGH = guasto rilevato)
 *   nSAB_B_TEST      GPIO_Input – diagnostica canale B (HIGH = guasto rilevato)
 *
 *   NOTA POLARITA'/EXTI (2026-07-15): un primo test a banco (SEN con
 *   interlock volutamente aperto, nessuna transizione a ERROR) aveva fatto
 *   ipotizzare una polarita' invertita su INTLCK_STATUS — ipotesi poi
 *   SMENTITA da verifica diretta di Luca: LOW = chiuso/OK e' confermato
 *   corretto, la lettura pin in SAB.c non e' mai cambiata di significato.
 *   Il trigger EXTI su PA0/PC2_C e' comunque stato rigenerato da CubeMX da
 *   IT_RISING a IT_RISING_FALLING (cattura anche la richiusura, utile per
 *   "GET SAB"), ma la causa del mancato rilevamento originale resta da
 *   confermare sul campo dopo questo giro di modifiche — vedi indagine
 *   "GET SAB" in rs485_cmd.c per isolare l'anello che si interrompe.
 *
 * SEQUENZA DI ARMO:
 *   1. SAB_PowerOn()  → nSAB_PWR_SHDN HIGH: alimenta il modulo SAB.
 *      Chiamare in action_enter_on() (stato SYS_ON).
 *   2. SAB_Enable()   → nSAB_EN LOW: avvia la procedura di armo.
 *      Chiamare in action_enter_enabled() (transizione SYS_ON→SYS_ENABLED).
 *   3. GUARD: il driver attende SAB_GUARD_DELAY_MS prima di leggere i pin
 *      (i segnali non sono validi immediatamente dopo l'asserzione di nSAB_EN).
 *   4. ENABLING: il driver monitora tutti i pin:
 *      – INTLCK_A/B_STATUS devono andare LOW (chiuso/OK) entro timeout_ms
 *        (passato a SAB_Init).
 *      – A_TEST/B_TEST devono rimanere LOW (se HIGH → TEST_FAIL immediato).
 *   5. Quando entrambi gli INTLCK sono LOW (chiuso) e TEST sono LOW → ARMED.
 *      Viene invocata la callback con SAB_EVT_ARMED.
 *   6. In ARMED il driver monitora continuamente tutti i pin:
 *      – Qualsiasi INTLCK HIGH (aperto) → SAB_EVT_INTERLOCK_OPEN.
 *      – Qualsiasi TEST HIGH   → SAB_EVT_TEST_FAIL.
 *      NOTA (dal 2026-07-15): questi due eventi restano generici (un solo
 *      SAB_Evt_t per entrambi i canali A/B, stesso principio già usato per
 *      SYS_KEY_REMOVED_EVENT/SYS_LID_OPEN_EVENT in fsm.c) — il canale
 *      specifico che ha causato l'evento si ricostruisce a posteriori con
 *      SAB_GetInterlockStatus()/SAB_GetTestStatus() (sotto), usate da
 *      freertos.c (sab_event_cb, mascherabile per canale via
 *      ERR_BIT_SAB_INTLCK_A/B e ERR_BIT_SAB_TEST_A/B in task_monitor.h) e
 *      da fsm.c (event_detail_name(), dettaglio "GET ALARM").
 *
 *      MASCHERABILITÀ E SAB_STATE_FAULT (rivisto 2026-07-16): gli INTLCK
 *      sono la vera catena di sicurezza ridondante — la transizione a
 *      SAB_STATE_FAULT su interlock aperto resta SEMPRE attiva su QUALSIASI
 *      canale, indipendentemente da error_mask (la maschera agisce solo
 *      sulla notifica verso la FSM di sistema, mai sulla risposta interna
 *      del modulo SAB). I canali TEST invece sono un self-test diagnostico:
 *      da questo bugfix (vedi test_pins_ok() in SAB.c) un canale TEST
 *      mascherato in error_mask (ERR_BIT_SAB_TEST_A/B a 0, tipicamente
 *      perché quel canale non è cablato in una data configurazione HW) NON
 *      causa più SAB_STATE_FAULT — a differenza degli INTLCK, l'assenza di
 *      un canale TEST non compromette la ridondanza del percorso di
 *      sicurezza (che dipende dagli INTLCK), solo la sua autodiagnostica.
 *   7. SAB_Disable()       → nSAB_EN HIGH, stato → DISABLED.
 *   8. SAB_ShutdownSupply() → nSAB_PWR_SHDN LOW (sola rimozione alimentazione,
 *      senza toccare nSAB_EN — usata dalla FSM per rispettare i 10s di
 *      guardia dopo il fronte di discesa di nSAB_EN, vedi sotto).
 *   9. SAB_PowerOff()      → SAB_Disable() + SAB_ShutdownSupply() immediati
 *      (nessuna guardia): usata solo dove non c'è rischio di scaricare
 *      condensatori sotto carico (es. action_init_ok() al boot, SAB mai armato).
 *
 * GUARDIA 10s SU nSAB_PWR_SHDN (requisito hardware, vedi fsm.c):
 *   Il fronte di discesa di nSAB_EN (SAB_Disable(), sia da solo che dentro
 *   SAB_PowerOff()) avvia la scarica di alcune capacità sul modulo SAB.
 *   Rimuovere l'alimentazione (nSAB_PWR_SHDN LOW) PRIMA che siano trascorsi
 *   almeno 10s da quel fronte impedisce alle capacità di scaricarsi
 *   correttamente, compromettendo il riavvio successivo. La FSM (fsm.c)
 *   NON chiama quasi mai SAB_PowerOff() direttamente per le transizioni che
 *   escono da SYS_ENABLED/SYS_EMISSION: usa invece SAB_Disable() subito
 *   (fronte di discesa immediato, sempre sicuro) seguito da
 *   SAB_ShutdownSupply() posticipata di almeno 10s (vedi sab_request_shutdown()
 *   / FSM_Tick() in fsm.c). Questo file (SAB.c/.h) non implementa la
 *   guardia: si limita a esporre le due primitive separate.
 *
 * SAB_BOOT_DISCHARGE (rimovibile, requisito introdotto il 2026-07-07, vedi
 * banner completo in fsm.c):
 *   Allo spegnimento non è garantito che le capacità sul modulo SAB siano
 *   già scariche (es. spegnimento improvviso, mancanza rete). Per questo,
 *   ad ogni boot, PRIMA di transire da SYS_INIT a SYS_IDLE, la FSM esegue un
 *   ciclo di scarica forzata: SAB_PowerOn() → attesa SAB_GUARD_DELAY_MS di
 *   stabilizzazione alimentazione → impulso nSAB_EN LOW 10ms
 *   (SAB_RawEnPulse(), sotto) → nSAB_EN HIGH (è QUESTO fronte di salita a
 *   innescare la scarica, non quello di discesa dell'impulso), attesa 5s →
 *   SAB_PowerOff(). L'attesa di stabilizzazione dopo SAB_PowerOn() è
 *   necessaria: senza di essa il fronte di salita su nSAB_EN non risulta
 *   rilevato dal circuito di scarica (osservato su hardware).
 *   SAB_RawEnPulse() scrive il pin direttamente, bypassando SAB_State_t: non
 *   è un vero e proprio armo (niente guard/interlock/test), solo un impulso
 *   hardware per avviare la scarica. Non richiede la guardia dei 10s vista
 *   sopra: SAB_PowerOff() qui è sempre sicuro, il modulo non era armato.
 *
 * THREAD SAFETY:
 *   SAB_IntlckA/B_EXTI_Callback() sono chiamate dall'ISR e scrivono solo
 *   flag volatili. SAB_Process(tick_ms) li legge con sezione critica FreeRTOS.
 */

#ifndef DRIVERS_SAB_SAB_H_
#define DRIVERS_SAB_SAB_H_

#include <stdbool.h>
#include <stdint.h>

/* ========================================================================== */
/* --- CONFIGURAZIONE --- */
/* ========================================================================== */

/**
 * Tempo di guardia dopo l'asserzione di nSAB_EN prima di iniziare a leggere
 * i pin di stato. Necessario perché i segnali non sono validi immediatamente.
 */
#define SAB_GUARD_DELAY_MS      1000U

/* ========================================================================== */
/* --- STATO --- */
/* ========================================================================== */

typedef enum {
    SAB_STATE_DISABLED = 0,  /**< nSAB_EN HIGH, modulo non abilitato */
    SAB_STATE_GUARD,          /**< nSAB_EN LOW, guard delay in corso (pin non letti) */
    SAB_STATE_ENABLING,       /**< Guard scaduto, attesa chiusura INTLCK + verifica TEST */
    SAB_STATE_ARMED,          /**< INTLCK chiusi, TEST OK, monitoraggio attivo */
    SAB_STATE_FAULT,          /**< Errore (timeout/interlock aperto/test fallito) */
} SAB_State_t;

/* ========================================================================== */
/* --- CALLBACK --- */
/* ========================================================================== */

/** Tipo evento SAB — usato come argomento della callback applicativa */
typedef enum {
    SAB_EVT_ARMED = 0,          /**< Armo completato: INTLCK chiusi + TEST OK */
    SAB_EVT_TIMEOUT,            /**< Interlock non chiusi entro timeout_ms */
    SAB_EVT_INTERLOCK_OPEN,     /**< Interlock aperto in stato ARMED */
    SAB_EVT_TEST_FAIL,          /**< Pin TEST HIGH (guasto diagnostica) */
} SAB_Evt_t;

/** Prototipo della callback applicativa per gli eventi SAB */
typedef void (*SAB_EventCb_t)(SAB_Evt_t evt);

/* ========================================================================== */
/* --- API PUBBLICA --- */
/* ========================================================================== */

/**
 * @brief  Inizializza il modulo SAB. nSAB_EN e nSAB_PWR_SHDN portati in stato
 *         sicuro (modulo spento). Da chiamare in MX_FREERTOS_Init().
 * @param  timeout_ms  Timeout [ms] per la chiusura degli interlock dopo il guard
 *                     (tipicamente g_config.sab_interlock_timeout_ms, es. 2000).
 * @param  cb          Callback applicativa per gli eventi SAB. Può essere NULL.
 */
void SAB_Init(uint16_t timeout_ms, SAB_EventCb_t cb);

/**
 * @brief  Alimenta il modulo SAB (nSAB_PWR_SHDN HIGH).
 *         Da chiamare in action_enter_on() quando si entra in SYS_ON.
 *         Non avvia l'armo: usare SAB_Enable() in seguito.
 */
void SAB_PowerOn(void);

/**
 * @brief  Disabilita e spegne il modulo SAB, SENZA alcuna guardia temporale.
 *         Chiama SAB_Disable() poi SAB_ShutdownSupply() immediatamente.
 *
 *         Usare SOLO dove non c'è rischio di scaricare condensatori sotto
 *         carico — tipicamente action_init_ok() al boot (SAB non è mai stato
 *         armato). Per le transizioni che escono da SYS_ENABLED/SYS_EMISSION
 *         la FSM (fsm.c) NON chiama questa funzione: usa invece SAB_Disable()
 *         + SAB_ShutdownSupply() posticipata di almeno 10s (vedi banner
 *         "GUARDIA 10s" più sopra e sab_request_shutdown() in fsm.c).
 */
void SAB_PowerOff(void);

/**
 * @brief  Rimuove SOLO l'alimentazione del modulo SAB (nSAB_PWR_SHDN LOW),
 *         senza toccare nSAB_EN. Non tocca lo stato SAB_State_t: se il
 *         modulo non era già stato disabilitato con SAB_Disable(), lo stato
 *         interno resterebbe incoerente con l'hardware — il chiamante deve
 *         garantire che SAB_Disable() sia già stato invocato in precedenza.
 *
 *         Pensata per essere chiamata in differita rispetto a SAB_Disable()
 *         (vedi banner "GUARDIA 10s" più sopra), per rispettare i 10s di
 *         guardia richiesti dal fronte di discesa di nSAB_EN prima di
 *         rimuovere l'alimentazione e permettere la corretta scarica delle
 *         capacità del modulo.
 */
void SAB_ShutdownSupply(void);

/**
 * @brief  SAB_BOOT_DISCHARGE (rimovibile, vedi banner più sopra e in fsm.c).
 *         Scrive DIRETTAMENTE il pin nSAB_EN, bypassando SAB_State_t: non
 *         avvia guard/enabling/interlock monitoring, non è un vero armo.
 *         Uso esclusivo: impulso di scarica capacità al boot (fsm.c).
 *         Qualsiasi altro utilizzo deve passare da SAB_Enable()/SAB_Disable().
 * @param  assert_low  true = nSAB_EN LOW (impulso di scarica),
 *                     false = nSAB_EN HIGH (ripristino).
 */
void SAB_RawEnPulse(bool assert_low);

/**
 * @brief  Avvia la procedura di armo: nSAB_EN LOW → guard → monitoring.
 *         Da chiamare in action_enter_enabled() (transizione SYS_ON→SYS_ENABLED).
 *         No-op se lo stato non è DISABLED.
 */
void SAB_Enable(void);

/**
 * @brief  Disabilita il modulo SAB: nSAB_EN HIGH, stato → DISABLED.
 *         Idempotente. Non spegne l'alimentazione (usare SAB_PowerOff per questo).
 */
void SAB_Disable(void);

/**
 * @brief  Processa la FSM del SAB. Da chiamare ogni tick_ms ms da task_inputs.
 * @param  tick_ms  Millisecondi trascorsi dall'ultima chiamata.
 */
void SAB_Process(uint32_t tick_ms);

/**
 * @brief  Interlock chiuso e sistema di sicurezza sano (stato ARMED).
 *         Questa è l'informazione rilevante per l'utente: indica che il percorso
 *         di sicurezza è integro e il sistema può operare.
 * @return true  = interlock chiuso, diagnostica OK.
 *         false = non ancora armato, in attesa, oppure fault.
 */
bool SAB_IsInterlockOk(void);

/**
 * @brief  Il sistema di sicurezza è in stato di fault (qualsiasi causa:
 *         interlock aperto, timeout o diagnostica interna fallita).
 *         Questa è l'informazione rilevante per l'utente quando c'è un errore.
 * @return true  = fault attivo (stato FAULT).
 *         false = nessun fault.
 */
bool SAB_IsFault(void);

/* ========================================================================== */
/* --- API INTERNA (uso driver/FSM) --- */
/* ========================================================================== */

/**
 * @brief  Restituisce lo stato corrente della FSM SAB (uso interno).
 *         Preferire SAB_IsInterlockOk() / SAB_IsFault() per logica applicativa.
 */
SAB_State_t SAB_GetState(void);

/**
 * @brief  Restituisce true se lo stato è ARMED (alias di SAB_IsInterlockOk).
 *         Mantenuto per compatibilità interna con il polling in action_enter_enabled().
 */
bool SAB_IsArmed(void);

/**
 * @brief  Stato RAW (non mascherato) dei due canali interlock, letto
 *         dall'ultimo aggiornamento noto (EXTI o polling in GUARD/ENABLING —
 *         vedi s_intlck_a_open/b_open in SAB.c).
 *         Usata da freertos.c (sab_event_cb, per decidere quale canale
 *         notificare in base a ERR_BIT_SAB_INTLCK_A/B) e da fsm.c
 *         (event_detail_name(), per il dettaglio "SAB_INTLCK_A|B" di
 *         "GET ALARM") — NON cambia il comportamento di sicurezza del
 *         modulo SAB, che resta indipendente da error_mask.
 * @param  intlck_a_open  Puntatore di uscita (nullable): true = canale A aperto.
 * @param  intlck_b_open  Puntatore di uscita (nullable): true = canale B aperto.
 */
void SAB_GetInterlockStatus(bool *intlck_a_open, bool *intlck_b_open);

/**
 * @brief  Stato RAW (non mascherato) dei due canali del pin TEST, letto
 *         al momento della chiamata (polling diretto dei pin, come
 *         test_pins_ok() interno — nessuno stato cache necessario).
 *         Stesso uso di SAB_GetInterlockStatus() sopra, ma per
 *         ERR_BIT_SAB_TEST_A/B e il dettaglio "SAB_TEST_A|B".
 * @param  test_a_fault  Puntatore di uscita (nullable): true = canale A in guasto (pin HIGH).
 * @param  test_b_fault  Puntatore di uscita (nullable): true = canale B in guasto (pin HIGH).
 */
void SAB_GetTestStatus(bool *test_a_fault, bool *test_b_fault);

/* ========================================================================== */
/* --- CALLBACK ISR --- */
/* ========================================================================== */

/**
 * @brief  Callback EXTI per interlock canale A (nSAB_INTLCK_A, PA0).
 *         Da chiamare in HAL_GPIO_EXTI_Callback().
 */
void SAB_IntlckA_EXTI_Callback(void);

/**
 * @brief  Callback EXTI per interlock canale B (nSAB_INTLCK_B, PC2_C).
 *         Da chiamare in HAL_GPIO_EXTI_Callback().
 */
void SAB_IntlckB_EXTI_Callback(void);

#endif /* DRIVERS_SAB_SAB_H_ */
