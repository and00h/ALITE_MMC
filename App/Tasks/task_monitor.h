/*
 * task_monitor.h
 *
 * Task di monitoraggio periodico della sicurezza del sistema (100ms).
 *
 * RESPONSABILITÀ:
 *   - Flood sensor: sempre attivo, ogni ciclo
 *   - Flusso acqua: attivo quando water cooling è RUNNING, con inibizione iniziale
 *   - Temperature NTC (AD7490): 6 sensori su board MMC
 *   - Temperatura e umidità LaseQ: lette dalla telemetria RS485
 *   - Umidità MMC (SHT35): trigger/read alternati ogni 200ms
 *   - PSU: DC_OK e ALARM
 *   - Applicazione warning_mask / error_mask prima di pubblicare eventi
 *
 * POLITICA EVENTI:
 *   INIT / IDLE / ACTIVE : solo warning (s_active_warnings), nessun fault/error
 *   SYS_ON e successivi  : fault → SYS_TEMP_ERROR_EVENT / SYS_FLOW_ERROR_EVENT ...
 *
 * Ogni anomalia viene segnalata UNA sola volta (flag "già segnalato"), resettato
 * quando la condizione rientra (isteresi implicita nella soglia).
 *
 * PRIORITÀ: osPriorityNormal5 (la più alta tra i task applicativi)
 * PERIODICITÀ: MONITOR_PERIOD_MS (100ms)
 */

#ifndef APP_TASKS_TASK_MONITOR_H_
#define APP_TASKS_TASK_MONITOR_H_

#include "cmsis_os.h"
#include <stdint.h>
#include <stdbool.h>

/* ========================================================================== */
/* --- COSTANTI --- */
/* ========================================================================== */

#define MONITOR_PERIOD_MS    100U   /* Periodo task [ms] */
#define FLOW_INHIBIT_MS     3000U   /* Inibizione flusso dopo apertura valvola [ms] */

/* ========================================================================== */
/* --- NTC SENSOR IDs ---
 *
 * Usati come valori in g_config.ntc_ch_map[ch].
 * NTC_NUM_SENSORS: numero di sensori riconosciuti.
 * 0xFF (in ntc_ch_map) = canale non utilizzato.
 */
/* ========================================================================== */

typedef enum {
    NTC_SENSOR_WATER_IN  = 0,   /**< Temperatura ingresso acqua refrigerazione (serigrafia NTC1, CH14) */
    NTC_SENSOR_WATER_OUT = 1,   /**< Temperatura uscita acqua refrigerazione (serigrafia NTC2, CH15) */
    /*
     * NTC_SENSOR_DRIVER: dal 2026-07-14 NON è più la sorgente della
     * temperatura "driver di corrente" — quel ruolo è passato alle 4
     * temperature riportate da LaseQ (MSG_STATUS.temperature[0..3], vedi
     * WARN_BIT_TEMP_DRIVER sotto e Monitor_CheckLaseQTelemetry() in
     * task_monitor.c). Non più letto da check_ntc_sensor(): resta definito
     * (numerazione stabile) ma è di fatto generico/non assegnato, come le
     * altre serigrafie NTC senza uso funzionale, in attesa di un'eventuale
     * nuova destinazione d'uso. NON associare più questo id alla serigrafia
     * NTC3/CH12 in fase di commissioning: quel canale è ora NTC_SENSOR_DIODE1
     * (vedi sotto) — mappatura aggiornata 2026-07-15.
     */
    NTC_SENSOR_DRIVER    = 2,   /**< [NON PIÙ USATO] generico, nessuna serigrafia dedicata — vedi commento sopra */
    /*
     * NTC_SENSOR_SPLICE: dal 2026-07-15 spostata dalla scheda su serigrafia
     * NTC16 (CH0) — le serigrafie NTC3/NTC4 (CH12/CH13), che ospitavano
     * splice/driver nella documentazione precedente, sono ora occupate dai
     * due sensori del diodo laser (vedi NTC_SENSOR_DIODE1/DIODE2 sotto).
     */
    NTC_SENSOR_SPLICE    = 3,   /**< Temperatura splice fibra ottica (serigrafia NTC16, CH0) */
    /*
     * NTC_SENSOR_DIODE1/DIODE2 (dal 2026-07-15): il diodo laser è
     * monitorato da DUE sensori NTC fisici indipendenti, entrambi da
     * leggere e verificare separatamente (soglie/bit/device-enable propri
     * — vedi WARN_BIT_TEMP_DIODE1/DIODE2 sopra e check_ntc_sensor() in
     * task_monitor.c). DIODE2 riusa l'id precedentemente "NTC7" (generico,
     * mai assegnato) invece di aggiungere un nuovo slot — NTC_NUM_SENSORS
     * resta 16.
     */
    NTC_SENSOR_DIODE1    = 4,   /**< Temperatura diodo laser 1 (serigrafia NTC3, CH12) */
    NTC_SENSOR_AMBIENT   = 5,   /**< Temperatura ambiente (board MMC) (serigrafia NTC6, CH11) */
    NTC_SENSOR_DIODE2     = 6,   /**< Temperatura diodo laser 2 (serigrafia NTC4, CH13) — ex NTC_SENSOR_NTC7 */
    /*
     * Serigrafie NTC8-NTC16 (tranne NTC16=SPLICE sopra): nessun uso
     * funzionale assegnato ancora, id generici NTCx (x = numero serigrafia)
     * in attesa di destinazione d'uso. Mapping canale AD7490 in
     * Config_LoadDefaults() (App/Config/config.c) e via comando RS485
     * "SET NTC MAP <serigrafia 1-16> <sensore>".
     */
    NTC_SENSOR_NTC8       = 7,   /**< Generico, non ancora assegnato (serigrafia NTC8) */
    NTC_SENSOR_NTC9       = 8,   /**< Generico, non ancora assegnato (serigrafia NTC9) */
    NTC_SENSOR_NTC10      = 9,   /**< Generico, non ancora assegnato (serigrafia NTC10) */
    NTC_SENSOR_NTC11      = 10,  /**< Generico, non ancora assegnato (serigrafia NTC11) */
    NTC_SENSOR_NTC12      = 11,  /**< Generico, non ancora assegnato (serigrafia NTC12) */
    NTC_SENSOR_NTC13      = 12,  /**< Generico, non ancora assegnato (serigrafia NTC13) */
    /*
     * NTC_SENSOR_PSU_TEMP/PWR_EL_TEMP (dal 2026-07-22, ex id generici
     * "NTC14"/"NTC15" — già documentati come destinati a quelle serigrafie,
     * ora assegnati formalmente): sonde non ancora cablate fisicamente in
     * campo (comunicato dall'utente), destinazione prevista serigrafia
     * NTC14 (CH2) e NTC15 (CH1) — vedi ntc_ch_map in Config_LoadDefaults().
     * Device-enable dedicato: bit 7/8 di temp_sensor_enabled_mask.
     */
    NTC_SENSOR_PSU_TEMP    = 13,  /**< Temperatura PSU (serigrafia NTC14, CH2 — non ancora cablato) */
    NTC_SENSOR_PWR_EL_TEMP = 14,  /**< Temperatura elettronica di potenza (serigrafia NTC15, CH1 — non ancora cablato) */
    /*
     * ATTENZIONE: questo id si chiama "NTC16" per continuità storica di
     * numerazione, ma la serigrafia fisica NTC16 (CH0) è ora assegnata di
     * default a NTC_SENSOR_SPLICE (vedi sopra) — questo id generico NON ha
     * più una serigrafia "naturale" corrispondente, va mappato via
     * "SET NTC MAP <serigrafia> NTC16" solo se serve un canale generico
     * aggiuntivo, non per la serigrafia NTC16 stessa.
     */
    NTC_SENSOR_NTC16      = 15,  /**< Generico, non ancora assegnato — vedi nota sopra */
    NTC_NUM_SENSORS       = 16,
} NTC_SensorId_t;

/* ========================================================================== */
/* --- BITMASK WARNING (per g_config.warning_mask) ---
 *
 * Se il bit corrispondente è 0 in warning_mask, il warning è mascherato:
 * non viene postato l'evento SYS_HUMIDITY_WARN_EVENT o SYS_TEMP_WARN_EVENT,
 * ma il bit in s_active_warnings viene comunque aggiornato (per telemetria).
 */
/* ========================================================================== */
/*
 * RICOMPATTATO 2026-07-15 (contestuale al reset di CONFIG_MAGIC a
 * 0xA55A0001 + erase di settore, vedi config.h): i bit sono raggruppati
 * per ARGOMENTO invece che per ordine di introduzione storica — ogni coppia
 * "gemella" (stessa misura, sorgente MMC vs LaseQ, o due sensori/canali
 * dello stesso elemento fisico) occupa bit ADIACENTI. Prima di questo
 * reset i bit aggiunti in coda per non rompere compatibilità con record
 * già in flash (es. WARN_BIT_TEMP_DIODE2 in bit 12, lontano dal suo
 * gemello WARN_BIT_TEMP_DIODE1 in bit 6) — non più necessario, l'erase di
 * settore garantisce che nessun record con la vecchia numerazione resti
 * in flash. Ordine risultante (0-12): HUMIDITY_MMC/LASEQ, TEMP_WATER_IN/
 * OUT, TEMP_DRIVER, TEMP_SPLICE, TEMP_DIODE1/DIODE2, TEMP_AMBIENT/
 * LQ_AMBIENT, FLOW, DEW_MMC/LASEQ.
 */
#define WARN_BIT_HUMIDITY_MMC     (1UL <<  0)  /**< Umidità SHT35 > soglia */
#define WARN_BIT_HUMIDITY_LASEQ   (1UL <<  1)  /**< Umidità LaseQ > soglia */
#define WARN_BIT_TEMP_WATER_IN    (1UL <<  2)  /**< Temp acqua ingresso warn */
#define WARN_BIT_TEMP_WATER_OUT   (1UL <<  3)  /**< Temp acqua uscita warn */
/*
 * WARN_BIT_TEMP_DRIVER: dal 2026-07-14 riflette le 4 temperature degli
 * elementi di potenza riportate da LaseQ (MSG_STATUS.temperature[0..3]),
 * NON più il sensore NTC locale CH12 (NTC_SENSOR_DRIVER, ora inutilizzato
 * per questo scopo — vedi task_monitor.h/.c). Basta UNA delle 4 temperature
 * fuori soglia per attivare warning/error — vedi Monitor_CheckLaseQTelemetry().
 */
#define WARN_BIT_TEMP_DRIVER      (1UL <<  4)  /**< Almeno una delle 4 temp. elementi potenza LaseQ fuori soglia */
#define WARN_BIT_TEMP_SPLICE      (1UL <<  5)  /**< Temp splice warn */
/*
 * WARN_BIT_TEMP_DIODE1/DIODE2 (dal 2026-07-15, bit adiacenti dal reset
 * 2026-07-15): il diodo laser ha DUE sensori NTC fisici distinti
 * (serigrafia NTC3/NTC4 sulla scheda, CH12/CH13 su AD7490) — letti e
 * verificati INDIPENDENTEMENTE, ciascuno con soglie e bit mascherabile
 * propri (stesso principio già usato per KEY_A/KEY_B e FLOOD1/FLOOD2) —
 * vedi check_ntc_sensor(NTC_SENSOR_DIODE1/DIODE2, ...) in task_monitor.c.
 */
#define WARN_BIT_TEMP_DIODE1      (1UL <<  6)  /**< Temp diodo laser 1 warn */
#define WARN_BIT_TEMP_DIODE2      (1UL <<  7)  /**< Temp diodo laser 2 warn */
#define WARN_BIT_TEMP_AMBIENT     (1UL <<  8)  /**< Temp ambiente NTC (board MMC) warn */
#define WARN_BIT_TEMP_LQ_AMBIENT  (1UL <<  9)  /**< Temp ambiente LaseQ (SHT35 interno) warn */
#define WARN_BIT_FLOW             (1UL << 10)  /**< Flusso acqua warn */
#define WARN_BIT_DEW_MMC          (1UL << 11)  /**< Dew point MMC (SHT35): T_amb vicina al punto di condensa */
#define WARN_BIT_DEW_LASEQ        (1UL << 12)  /**< Dew point LaseQ (SHT35): T_amb vicina al punto di condensa */
/*
 * WARN_BIT_TEMP_PSU/PWR_EL/MB (dal 2026-07-22, nuovi slot temperatura):
 * PSU/PWR_EL sono NTC fisici (vedi NTC_SENSOR_PSU_TEMP/PWR_EL_TEMP),
 * MB è la temperatura dell'SHT35 già montato su MMC (finora usato solo
 * per umidità/dew point, vedi WARN_BIT_HUMIDITY_MMC/WARN_BIT_DEW_MMC) —
 * da NON confondere con WARN_BIT_TEMP_AMBIENT (quello è l'NTC sulla
 * scocca/aria esterna, questo è un sensore su scheda elettronica).
 */
#define WARN_BIT_TEMP_PSU         (1UL << 13)  /**< Temp PSU warn (NTC, non ancora cablato) */
#define WARN_BIT_TEMP_PWR_EL      (1UL << 14)  /**< Temp elettronica di potenza warn (NTC, non ancora cablato) */
#define WARN_BIT_TEMP_MB          (1UL << 15)  /**< Temp scheda madre warn (SHT35-MMC) */

/* ========================================================================== */
/* --- BITMASK ERRORI RECUPERABILI (per g_config.error_mask) ---
 *
 * Contiene ESCLUSIVAMENTE eventi che portano a SYS_ERROR (recuperabile via
 * CERR) — vedi BITMASK FAULT sotto per gli eventi -> SYS_FAULT. Se il bit è
 * 0 in error_mask, l'evento non viene pubblicato nella coda FSM.
 *
 * RICOMPATTATO 2026-07-15 (contestuale al reset di CONFIG_MAGIC a
 * 0xA55A0001 + erase di settore, vedi config.h): stesso criterio del
 * warning_mask sopra -- bit raggruppati per argomento, ogni coppia gemella
 * su bit adiacenti (SAB_INTLCK_A/B, SAB_TEST_A/B, KEY_A/B, DEW_MMC/LASEQ),
 * non piu' sparsi per compatibilita' con vecchi record (non piu'
 * necessaria, vedi erase di settore). Ordine risultante (0-13): FLOW,
 * TEMP, SAB_TIMEOUT, SAB_INTLCK_A/B, SAB_TEST_A/B, KEY_A/B, DEW_MMC/LASEQ,
 * LASEQ_INTERNAL, LID1/LID2.
 */
/* ========================================================================== */

#define ERR_BIT_FLOW           (1UL <<  0)  /**< Flusso acqua fault -> SYS_ERROR (recuperabile) */
#define ERR_BIT_TEMP           (1UL <<  1)  /**< Temperatura fault -> SYS_ERROR (recuperabile) */
#define ERR_BIT_SAB_TIMEOUT    (1UL <<  2)  /**< SAB interlock timeout (entrambi i canali, vedi SAB.h) */
/*
 * ERR_BIT_SAB_INTLCK_A/B, ERR_BIT_SAB_TEST_A/B (dal 2026-07-15): il modulo
 * SAB ha due canali RIDONDANTI indipendenti (nSAB_INTLCK_A/B_STATUS,
 * nSAB_A/B_TEST — vedi SAB.h), letti/discriminati separatamente da
 * SAB_GetInterlockStatus()/SAB_GetTestStatus(). A differenza di
 * ERR_BIT_KEY_A/KEY_B e ERR_BIT_LID1/LID2 (varianti di prodotto, un
 * contatto può legittimamente non essere montato), i due canali INTLCK sono
 * la stessa catena di sicurezza duplicata per ridondanza: mascherare un
 * canale INTLCK REALMENTE cablato NON disabilita la risposta di sicurezza
 * (il modulo SAB.c va comunque in SAB_STATE_FAULT su QUALSIASI canale
 * interlock, indipendentemente da error_mask — vedi banner in freertos.c),
 * maschera SOLO la notifica SYS_ERROR/telemetria verso la FSM di sistema
 * per quel canale.
 *
 * ERR_BIT_SAB_TEST_A/B (rivisto 2026-07-16): a differenza degli INTLCK
 * sopra, i canali TEST sono un self-test diagnostico, non la catena di
 * sicurezza ridondante in sé — un canale TEST realmente NON cablato in una
 * data configurazione HW va mascherato esattamente come una chiave/coperchio
 * non montato: da questo bugfix (test_pins_ok() in SAB.c) mascherare
 * ERR_BIT_SAB_TEST_A/B impedisce anche la transizione interna a
 * SAB_STATE_FAULT per quel canale, non solo la notifica FSM. Usare comunque
 * la maschera solo per canali TEST realmente assenti/non popolati, mai per
 * mascherare un guasto diagnostico reale su un canale effettivamente
 * cablato. Dal 2026-07-15 i quattro bit sono adiacenti (3-6) invece che
 * sparsi, vedi banner RICOMPATTATO sopra.
 */
#define ERR_BIT_SAB_INTLCK_A   (1UL <<  3)  /**< SAB interlock canale A aperto */
#define ERR_BIT_SAB_INTLCK_B   (1UL <<  4)  /**< SAB interlock canale B aperto */
#define ERR_BIT_SAB_TEST_A     (1UL <<  5)  /**< SAB test canale A fallito */
#define ERR_BIT_SAB_TEST_B     (1UL <<  6)  /**< SAB test canale B fallito */
/*
 * Chiavi A/B: contatti fisicamente e logicamente indipendenti (nKEY_STATUS_A
 * su PC13, nKEY_STATUS_B su PC14) — non fanno parte del modulo SAB (che ha
 * i propri canali interlock A/B su pin separati, vedi SAB.h). Spesso solo
 * un contatto è montato in produzione: il bit relativo al contatto NON
 * montato va mascherato (bit a 0) così quel canale viene ignorato invece
 * di bloccare permanentemente l'accensione. Vedi Key_InterlockSatisfied().
 */
#define ERR_BIT_KEY_A          (1UL <<  7)  /**< Chiave A non inserita/rimossa */
#define ERR_BIT_KEY_B          (1UL <<  8)  /**< Chiave B non inserita/rimossa */
#define ERR_BIT_DEW_MMC        (1UL <<  9)  /**< Dew point MMC: condensazione imminente (default: mascherato) */
#define ERR_BIT_DEW_LASEQ      (1UL << 10)  /**< Dew point LaseQ: condensazione imminente */
/*
 * ERR_BIT_LASEQ_INTERNAL (bit 11): gate per SYS_LASEQ_ERROR_EVENT, postato
 * da task_comms.c quando MSG_STATUS.error_code (LaseQ, vedi FSM_ErrorCode_t
 * in fsm.h) ha almeno un bit rilevante attivo — dal 2026-07-15 (v2) TUTTI i
 * bit 0-5 (temperature, vanode, corrente, saturazione, RS485 watchdog lato
 * LaseQ) E il bit 7 (FSM_FAULT_HW: PWR_OK basso o interlock aperto).
 * PRIMA di questo bugfix il bit 7 era gated da FAULT_BIT_LASEQ (fault_mask)
 * e portava a SYS_FAULT: cambiato perché un'apertura dell'interlock LaseQ
 * è un evento plausibile in esercizio normale (vedi banner SEVERITÀ in
 * fsm.h) — SYS_FAULT resta riservato ESCLUSIVAMENTE alla perdita di
 * comunicazione RS485 (FAULT_BIT_LASEQ, sotto, ora unico consumatore di
 * quel bit).
 */
#define ERR_BIT_LASEQ_INTERNAL (1UL << 11)  /**< error_code LaseQ, bit 0-5 e bit 7 (FSM_ErrorCode_t) */
/*
 * LID1/LID2 (bit 12, 13, dal 2026-07-15): sensori apertura coperchio
 * (LID1_OPEN/LID2_OPEN, PE2/PE3), fisicamente e logicamente indipendenti
 * come le chiavi A/B — stesso principio: il sensore NON montato in
 * produzione va mascherato (bit a 0) invece di bloccare permanentemente
 * l'accensione. Vedi Lid_InterlockSatisfied() (Drivers/Lid/Lid.c) e
 * action_enter_on()/SYS_LID_OPEN_EVENT in fsm.c.
 */
#define ERR_BIT_LID1           (1UL << 12)  /**< Coperchio 1 aperto */
#define ERR_BIT_LID2           (1UL << 13)  /**< Coperchio 2 aperto */

/* ========================================================================== */
/* --- BITMASK FAULT NON RECUPERABILI (per g_config.fault_mask) ---
 *
 * Contiene ESCLUSIVAMENTE eventi che portano a SYS_FAULT (non recuperabile
 * via CERR, richiede "GET ALARM"/comando FRST — vedi FAULT LATCH sotto):
 * fault di comunicazione/link (LaseQ, AMC, COM interface) e fault operativi
 * critici (allagamento, PSU). Se il bit è 0 in fault_mask, il fault non
 * viene pubblicato nella coda FSM.
 *
 * RICOMPATTATO 2026-07-14: FLOOD non è più un bit unico ma due indipendenti
 * (FLOOD1/FLOOD2), per poter mascherare singolarmente un sensore non montato
 * — stesso pattern già usato per le chiavi A/B in error_mask.
 * ========================================================================== */

#define FAULT_BIT_LASEQ        (1UL <<  0)  /**< Perdita comunicazione RS485 verso LaseQ (10 cicli falliti = 200ms) */
#define FAULT_BIT_AMC          (1UL <<  1)  /**< Perdita comunicazione UART verso AMC (miss count heartbeat) o AMC_FAULT_N asserito */
#define FAULT_BIT_COM          (1UL <<  2)  /**< Fault comunicazione/alimentazione verso COM interface (nCOM_PWR_FLT o timeout heartbeat nCOM_INT_IN) */
/*
 * FAULT_BIT_FLOOD1/FLOOD2 (bit 3, 4): un bit per sensore fisico
 * (FloodSensorGetStatus(FLOOD1)/FLOOD2)) — mascherare il bit del sensore
 * NON montato in produzione (spesso se ne monta uno solo), stesso principio
 * di ERR_BIT_KEY_A/KEY_B. ATTENZIONE: mascherare un sensore REALMENTE
 * collegato disabilita l'INTERA risposta ad allagamento per quel canale
 * (WaterCooling_Emergency() + taglio eFuse), non solo la notifica FSM —
 * vedi Monitor_CheckFlood() in task_monitor.c. Solo per debug/commissioning.
 */
#define FAULT_BIT_FLOOD1       (1UL <<  3)  /**< Allagamento rilevato da FLOOD1 */
#define FAULT_BIT_FLOOD2       (1UL <<  4)  /**< Allagamento rilevato da FLOOD2 */
#define FAULT_BIT_PSU          (1UL <<  5)  /**< Fault alimentatore (alarm o DC_OK mancante) */

/* ========================================================================== */
/* --- FAULT LATCH (per g_config.fault_latch_mask) ---
 *
 * Stessa numerazione bit di FAULT_BIT_* sopra (fault_mask) — NON un nuovo
 * set di costanti. Determina, PER OGNI TIPO di fault, se al verificarsi
 * deve restare "latched" (persistito in flash, g_config.fault_latch_active)
 * anche dopo un power-cycle: se latched, al prossimo avvio la FSM transisce
 * direttamente INIT -> FAULT (bypassando il check hardware e SYS_IDLE),
 * finché non arriva un comando esplicito di reset (RS485 "FRST", protetto
 * da login — vedi FSM_RequestFaultReset() in fsm.h/rs485_cmd.c "GET ALARM"/
 * "FRST"). Bit a 0 in fault_latch_mask: il fault resta non recuperabile
 * finché attivo (come oggi), ma un power-cycle lo "dimentica" normalmente
 * (comportamento storico, invariato).
 *
 * Default (Config_LoadDefaults()): solo FAULT_BIT_FLOOD1/FLOOD2 latched —
 * un allagamento è l'unico caso per cui riprendere automaticamente al
 * prossimo power-cycle è considerato pericoloso senza intervento umano che
 * verifichi la situazione. Gli altri fault (comunicazione, PSU) restano
 * non latched di default: un power-cycle li fa ritentare naturalmente.
 * ========================================================================== */


/* ========================================================================== */
/* --- ATTRIBUTI TASK --- */
/* ========================================================================== */

extern const osThreadAttr_t taskMonitor_attr;

/* ========================================================================== */
/* --- API PUBBLICA --- */
/* ========================================================================== */

/** @brief Entry point del task monitor. */
void TaskMonitor_Run(void *arg);

/**
 * @brief  Azzera il timer di inibizione del controllo flusso.
 *         Da chiamare in action_enter_active() dopo WaterCooling_Enable().
 */
void TaskMonitor_ResetFlowInhibit(void);

/**
 * @brief  Avvia il timer di inibizione del controllo PSU DC_OK.
 *         Da chiamare in action_enter_on() subito dopo il comando PSU enable.
 *         Inibisce Monitor_CheckPSU() per g_config.psu_dc_ok_delay_ms millisecondi,
 *         lasciando il tempo all'alimentatore di raggiungere DC_OK stabile.
 */
void TaskMonitor_ResetPSUInhibit(void);

/**
 * @brief  Restituisce la bitmask dei warning attivi.
 *         Thread-safe: volatile uint32_t, lettura atomica su ARM Cortex-M.
 */
uint32_t TaskMonitor_GetWarnings(void);

/**
 * @brief  Aggiorna la bitmask degli errori attivi rilevati dal monitor.
 *         Usata da task_comms per la telemetria. Thread-safe.
 *         AGGIORNAMENTO 2026-07-31: restituisce il bitmask CONGELATO
 *         (s_latched_errors), non piu' il valore "live" — resta invariato
 *         (tutte le cause accumulate) per tutta la durata di SYS_ERROR,
 *         azzerato solo da TaskMonitor_ClearLatchedErrors() all'uscita
 *         reale dall'errore. Vedi banner su s_latched_errors in
 *         task_monitor.c.
 */
uint32_t TaskMonitor_GetErrors(void);

/**
 * @brief  Azzera il bitmask errori congelato (s_latched_errors) restituito
 *         da TaskMonitor_GetErrors(). Chiamare ESCLUSIVAMENTE dal percorso
 *         di successo di action_clear_error() (fsm.c, transizione
 *         SYS_ERROR -> SYS_ACTIVE realmente completata) — mai su un
 *         tentativo di CERR respinto. Vedi banner in task_monitor.c.
 */
void TaskMonitor_ClearLatchedErrors(void);

/**
 * @brief  Apre il gate di accumulo di s_latched_errors (2026-07-31, bugfix
 *         "GET ERR sempre FLOW=1 dopo CERR"). Chiamare ESCLUSIVAMENTE da
 *         action_enter_error() (fsm.c), a ogni ingresso reale in SYS_ERROR.
 *         Vedi banner su s_latched_errors/s_error_latch_active in
 *         task_monitor.c.
 */
void TaskMonitor_BeginErrorLatch(void);

/**
 * @brief  Seed one-shot diretto in s_latched_errors, bypassando s_active_errors
 *         (2026-07-31, bugfix "GET ERR non mostra più FLOW dopo un errore di
 *         flusso, solo interlock"). Chiamare ESCLUSIVAMENTE da
 *         action_enter_error() (fsm.c) per cause il cui check continuo
 *         diventa inattendibile proprio a causa dell'ingresso in errore
 *         (es. FLOW, vedi WaterCooling_Emergency()). Vedi banner completo in
 *         task_monitor.c.
 * @param  bit_mask  Uno o più ERR_BIT_* da impostare (OR) nel latch.
 */
void TaskMonitor_SeedLatchedError(uint32_t bit_mask);

/**
 * @brief  Restituisce la bitmask dei fault NON RECUPERABILI attivi
 *         (tutti i FAULT_BIT_*, vedi sopra — comunicazione LASEQ/AMC/COM,
 *         più FLOOD1/FLOOD2/PSU dal 2026-07-14). Aggiornata da task_comms.c
 *         (LaseQ/AMC), COM_interface.c (COM) e task_monitor.c stesso
 *         (FLOOD1/FLOOD2/PSU), tramite TaskMonitor_SetFaultBit(). Thread-safe:
 *         volatile uint32_t, lettura atomica su ARM Cortex-M.
 */
uint32_t TaskMonitor_GetFaults(void);

/**
 * @brief  Bitmask (1<<id, indice = NTC_SensorId_t) dei sensori NTC
 *         ATTUALMENTE in fault di temperatura (fault_temp_rep[] interno).
 *         SYS_TEMP_ERROR_EVENT è condiviso da tutti i sensori di
 *         temperatura (locali NTC + LaseQ, vedi
 *         TaskMonitor_IsLaseQDriverTempFault()/IsLaseQAmbientTempFault()
 *         sotto): questa funzione permette di ricostruire QUALE sensore ha
 *         causato l'evento — usata da fsm.c per il dettaglio mostrato da
 *         "GET ALARM" (dal 2026-07-15).
 */
uint16_t TaskMonitor_GetTempFaultSensors(void);

/**
 * @brief  true se il gruppo delle 4 temperature driver LaseQ (MSG_STATUS.
 *         temperature[0..3]) è ATTUALMENTE oltre soglia fault. Vedi
 *         TaskMonitor_GetTempFaultSensors() sopra per il contesto d'uso.
 */
bool TaskMonitor_IsLaseQDriverTempFault(void);

/**
 * @brief  true se la temperatura ambiente LaseQ (SHT35 interno) è
 *         ATTUALMENTE oltre soglia fault. Vedi
 *         TaskMonitor_GetTempFaultSensors() sopra per il contesto d'uso.
 */
bool TaskMonitor_IsLaseQAmbientTempFault(void);

/**
 * @brief  true se la temperatura MB_TEMP (SHT35-MMC) è ATTUALMENTE oltre
 *         soglia fault. MB_TEMP non è indicizzato in
 *         TaskMonitor_GetTempFaultSensors() (non è un NTC/NTC_SensorId_t):
 *         va controllato separatamente, stesso motivo di
 *         TaskMonitor_IsLaseQAmbientTempFault() sopra.
 */
bool TaskMonitor_IsMBTempFault(void);

/**
 * @brief  Imposta/azzera un singolo bit nella bitmask dei fault attivi
 *         (telemetria "GET FAULTS" via RS485). Non posta eventi FSM: solo
 *         bookkeeping per lo stato riportato via RS485/COM. Il post
 *         dell'evento SYS_*_FAULT_EVENT resta a carico del chiamante
 *         (task_comms.c, COM_interface.c), gated su g_config.fault_mask.
 */
void TaskMonitor_SetFaultBit(uint32_t fault_bit, bool active);

/**
 * @brief  Imposta/azzera un singolo bit nella bitmask degli errori attivi
 *         (s_active_errors, telemetria "GET ERR" via RS485) — stesso schema
 *         di TaskMonitor_SetFaultBit() ma per error_mask/ERR_BIT_*. Non
 *         posta eventi FSM: solo bookkeeping, il post resta a carico del
 *         chiamante (task_comms.c per ERR_BIT_LASEQ_INTERNAL).
 */
void TaskMonitor_SetErrorBit(uint32_t error_bit, bool active);

/**
 * @brief  FAULT LATCH (vedi banner in questo file): ripulisce i flag di
 *         debounce interni al monitor per i fault classificabili come
 *         latched (FLOOD1, FLOOD2, PSU — flood1/2_reported,
 *         flood_shutdown_armed/done, psu_reported), così una condizione
 *         ancora fisicamente presente venga ri-valutata/ri-segnalata da
 *         zero al ciclo successivo invece di restare silenziosamente
 *         "già segnalata" per sempre.
 * @note   Da chiamare SOLO da action_fault_reset() (fsm.c, comando RS485
 *         "FRST" protetto da login) — mai da un percorso non autenticato.
 * @note   NON riabilita un eFuse eventualmente disabilitato per sicurezza
 *         (es. EFUSE_MAIN dopo un taglio da allagamento): per scelta di
 *         design richiede sempre un power-cycle fisico, anche dopo FRST.
 */
void TaskMonitor_ResetFaultLatches(void);

/**
 * @brief  Aggiorna l'ultimo error_code grezzo (8 bit) ricevuto da LaseQ
 *         (MSG_STATUS byte 1, vedi FSM_ErrorCode_t in fsm.h). Da chiamare
 *         da task_comms.c dopo ogni transazione RS485 riuscita. Usata da
 *         TaskMonitor_GetLaseQErrorCode() (diagnostica "GET LQERR").
 */
void TaskMonitor_SetLaseQErrorCode(uint8_t error_code);

/**
 * @brief  Ultimo error_code grezzo (8 bit) ricevuto da LaseQ. 0 se nessun
 *         errore è mai stato segnalato dall'avvio (o dall'ultima riconnessione).
 */
uint8_t TaskMonitor_GetLaseQErrorCode(void);

/**
 * @brief  Legge le ultime temperature/umidità LaseQ ricevute via RS485
 *         (MSG_STATUS), cache aggiornata da TaskMonitor_UpdateLaseQTelemetry()
 *         — usata per diagnostica RS485 ("GET TEMP", rs485_cmd.c).
 *         NON indica se la comunicazione con LaseQ è attualmente attiva:
 *         se LaseQ è disconnesso i valori restano quelli dell'ultima
 *         risposta valida (stesso comportamento di GET TEMP per gli NTC
 *         locali) — verificare TaskMonitor_GetFaults()/FAULT_BIT_LASEQ per
 *         lo stato del link.
 *
 * @param  driver_temp_c   Buffer di uscita, 4 temperature elementi di
 *                         potenza [°C] (MSG_STATUS.temperature[0..3]).
 * @param  ambient_temp_c  Puntatore di uscita, temperatura ambiente SHT35
 *                         LaseQ [°C, firmato].
 * @param  humidity_pct    Puntatore di uscita, umidità SHT35 LaseQ [%RH].
 */
void TaskMonitor_GetLaseQTemps(uint8_t driver_temp_c[4],
                                int8_t  *ambient_temp_c,
                                uint8_t *humidity_pct);

/**
 * @brief  Punto di condensa (Td) e margine (T_water_in - Td) dell'ultimo
 *         calcolo valido per l'aria MMC (SHT35 locale) — vedi
 *         check_dew_margin()/Monitor_CheckHumidity() in task_monitor.c.
 *         Usato per diagnostica RS485 ("GET DEW", rs485_cmd.c).
 *
 * @param  td_c      Puntatore di uscita (nullable): Td [°C].
 * @param  margin_c  Puntatore di uscita (nullable): margine [°C]
 *                    (>0 = sopra Td/sicuro, <0 = sotto Td/condensa).
 * @retval true se i valori sono attuali (WaterCooling RUNNING e NTC
 *         water_in valido all'ultimo ciclo), false se stantii/non
 *         significativi (cooling fermo o NTC non valido: *td_c e *margin_c
 *         restano comunque scritti con l'ultimo valore noto, se richiesti).
 */
bool TaskMonitor_GetDewMMC(float *td_c, float *margin_c);

/**
 * @brief  Come TaskMonitor_GetDewMMC(), ma per l'aria LaseQ (SHT35 di LaseQ,
 *         MSG_STATUS.temp_ambient_c/humidity) — stessa superficie fredda di
 *         riferimento (T_water_in, condivisa tra MMC e LaseQ).
 */
bool TaskMonitor_GetDewLaseQ(float *td_c, float *margin_c);

/**
 * @brief  Aggiorna la telemetria LaseQ ricevuta via RS485.
 *         Da chiamare da task_comms ad ogni STATUS frame ricevuto.
 *
 * @param  driver_temp_c   Le 4 temperature degli elementi di potenza [°C]
 *                         (MSG_STATUS.temperature[0..3]) — dal 2026-07-14
 *                         questa è la sorgente della temperatura "driver di
 *                         corrente" (WARN_BIT_TEMP_DRIVER/ERR_BIT_TEMP), in
 *                         sostituzione del sensore NTC locale CH12
 *                         (NTC_SENSOR_DRIVER, non più monitorato per questo
 *                         scopo — vedi Monitor_CheckTemperatures() in
 *                         task_monitor.c).
 * @param  ambient_temp_c  Temperatura ambiente SHT35 [°C, int8_t, firmato]
 * @param  humidity_pct    Umidità SHT35 [%RH, 0-100]
 */
void TaskMonitor_UpdateLaseQTelemetry(const uint8_t driver_temp_c[4],
                                      int8_t  ambient_temp_c,
                                      uint8_t humidity_pct);

/**
 * @brief  Legge la temperatura di un sensore NTC in decimi di grado.
 *
 * @param  id       ID sensore (NTC_SENSOR_WATER_IN .. NTC_SENSOR_AMBIENT).
 * @param  out_c10  Puntatore dove scrivere la temperatura [°C × 10].
 * @return true se il sensore è valido e la lettura è disponibile.
 */
bool TaskMonitor_GetNTCTempC10(NTC_SensorId_t id, int16_t *out_c10);

/**
 * @brief  Restituisce la maschera a 16 bit dei canali AD7490 attivi.
 *         Un bit è 1 se ntc_ch_map[ch] != 0xFF (canale mappato a un sensore).
 *         Può essere usata come identificatore visivo di quali canali
 *         sono configurati senza iterare ntc_ch_map.
 */
uint16_t TaskMonitor_GetNTCActiveMask(void);

#endif /* APP_TASKS_TASK_MONITOR_H_ */
