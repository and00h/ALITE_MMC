/*
 * task_monitor.c
 *
 * Monitoraggio periodico sicurezza: temperature NTC, umidità, flusso, PSU, flood.
 *
 * LOGICA DI PUBBLICAZIONE EVENTI:
 *   - Ogni anomalia viene segnalata UNA SOLA VOLTA (flag "già segnalato").
 *     Il flag si resetta quando la condizione rientra.
 *   - Prima di postare, si verifica:
 *       a) warning_mask / error_mask in g_config (bit abilitato?)
 *       b) stato FSM corrente (fault solo da SYS_ON in poi)
 *   - I warning aggiornano sempre s_active_warnings per la telemetria,
 *     indipendentemente dalla maschera e dallo stato.
 *
 * CONVERSIONE NTC (modello beta):
 *   R_ntc = R_series * raw / (4095 - raw)   [partitore VCC→Rseries→NTC→GND]
 *   T_K   = 1 / (1/298.15 + ln(R_ntc/R0) / beta)
 *   T_C   = T_K - 273.15
 *
 * TELEMETRIA LASEQ:
 *   temperature[0]  = driver (temperatura elemento attivo) [°C, uint8_t]
 *   temperature[1-3]= altri elementi attivi [°C, uint8_t]
 *   temp_ambient_c  = SHT35 temperatura ambiente [°C, int8_t]
 *   humidity        = SHT35 umidità [0.01 %RH, uint16_t]
 *   Aggiornata da task_comms via TaskMonitor_UpdateLaseQTelemetry().
 */

#include "FreeRTOS.h"
#include "task.h"
#include "task_monitor.h"
#include "queues.h"
#include "fsm.h"
#include "WaterCooling.h"
#include "FloodSensor.h"
#include "FlowMeter.h"
#include "PSU.h"
#include "SHT35.h"
#include "config.h"
#include "eFuse.h"
#include "SAB.h"           /* SAB_GetInterlockStatus()/GetTestStatus(): Monitor_CheckSAB() */
#include "Key.h"           /* Key_GetKeyStatus(): Monitor_CheckKey() */
#include "Lid.h"           /* Lid_GetStatus(): Monitor_CheckLid() */
#include "AD7490.h"
#include "Watchdog.h"
#include "hal_handles.h"   /* hspi5, hi2c2 */
#include "sys_log.h"       /* SysLog_Event: log finale prima del taglio eFuse su flood */

#include <math.h>   /* logf */
#include <string.h>

/* ========================================================================== */
/* --- ALLOCAZIONE STATICA --- */
/* ========================================================================== */

static StaticTask_t taskMonitor_tcb;
static StackType_t  taskMonitor_stack[640]; /* Aumentato per float math (logf) */

const osThreadAttr_t taskMonitor_attr = {
    .name       = "Monitor",
    .stack_mem  = &taskMonitor_stack[0],
    .stack_size = sizeof(taskMonitor_stack),
    .priority   = osPriorityNormal5,
    .cb_mem     = &taskMonitor_tcb,
    .cb_size    = sizeof(taskMonitor_tcb),
};

/* ========================================================================== */
/* --- SEMAFORI PRIVATI (IT SPI/I2C sensori) --- */
/* ========================================================================== */
/*
 * AD7490.c e SHT35.c sono driver puri: non includono FreeRTOS/CMSIS-RTOS e
 * non creano semafori. L'attesa "senza bloccare lo scheduler" del
 * completamento IT e la relativa segnalazione dal contesto ISR sono di
 * competenza di questo layer applicativo, tramite i function pointer passati
 * a AD7490_Init()/SHT35_Init() — stesso pattern già usato da LaseQ_Init() /
 * OnTxComplete() / OnRxComplete() in task_comms.c per la UART verso LaseQ4.
 */
static osSemaphoreId_t s_ad7490_sem = NULL;
static osSemaphoreId_t s_sht35_sem  = NULL;

static bool ad7490_wait(uint32_t timeout_ms)
{
    return osSemaphoreAcquire(s_ad7490_sem, timeout_ms) == osOK;
}
static void ad7490_signal(void)
{
    /* Rilevamento automatico del contesto IRQ da parte di
     * osSemaphoreRelease() (cmsis_os2.c, IRQ_Context()). */
    osSemaphoreRelease(s_ad7490_sem);
}

static bool sht35_wait(uint32_t timeout_ms)
{
    return osSemaphoreAcquire(s_sht35_sem, timeout_ms) == osOK;
}
static void sht35_signal(void)
{
    osSemaphoreRelease(s_sht35_sem);
}

/* ========================================================================== */
/* --- STATO INTERNO --- */
/* ========================================================================== */

static volatile uint32_t flow_inhibit_ms  = 0;
static volatile uint32_t psu_inhibit_ms   = 0;   /* inibizione DC_OK check dopo SON */
static volatile uint32_t s_active_warnings = 0;
static volatile uint32_t s_active_errors   = 0;

/*
 * s_latched_errors (NUOVO 2026-07-31, richiesta esplicita): bitmask "congelato"
 * degli errori — è QUESTO, non s_active_errors, che TaskMonitor_GetErrors()
 * restituisce (letto da "GET ERR" RS485 e da COM_App_HandleStatus() per
 * COM_STAT_OP_ERR_ACTIVE). s_active_errors resta il bitmask "live" (usato
 * internamente dalle funzioni di check per le decisioni di posting evento,
 * *_rep flags, ecc. — invariato), ma da solo aveva un problema: veniva
 * ricalcolato ogni ciclo indipendentemente dallo stato FSM, quindi un bit
 * spariva silenziosamente non appena la condizione fisica rientrava (es.
 * flusso tornato in range) anche se la macchina era ancora bloccata in
 * SYS_ERROR in attesa di CERR — "GET ERR" mostrava 0x0 con la macchina
 * ancora in errore.
 *
 * Regola: mentre s_error_latch_active è true, ad ogni ciclo di
 * TaskMonitor_Run() s_latched_errors viene SOLO arricchito (OR) con
 * s_active_errors corrente, mai decrementato — vedi il ciclo principale
 * sotto. Azzerato ESCLUSIVAMENTE da TaskMonitor_ClearLatchedErrors(),
 * chiamata da action_clear_error() (fsm.c) solo sul percorso di successo
 * (CERR realmente completato, non un tentativo respinto — vedi commento
 * lì). Così un errore rimane visibile con TUTTE le cause accumulate per
 * tutta la durata di SYS_ERROR, sparisce solo quando si esce davvero
 * dall'errore.
 *
 * BUGFIX (2026-07-31, "GET ERR mostra sempre FLOW=1 anche dopo CERR"):
 * la prima versione di questo meccanismo usava FSM_GetState() == SYS_ERROR
 * come gate, letto da QUESTO task (TaskMonitor, task separato). Ma
 * action_clear_error() (fsm.c) è BLOCCANTE fino a ~500ms (polling conferma
 * LaseQ) e FSM_ProcessEvent() aggiorna lo stato "ufficiale" (s_current_state)
 * SOLO dopo che l'intera action è tornata — quindi per tutta quella finestra,
 * ANCHE DOPO che TaskMonitor_ClearLatchedErrors() ha già azzerato il latch
 * (chiamata alla fine di action_clear_error(), success path), FSM_GetState()
 * poteva ancora restituire SYS_ERROR: un ciclo di TaskMonitor_Run() capitato
 * in quella finestra ri-arricchiva IMMEDIATAMENTE s_latched_errors con
 * s_active_errors corrente (race cross-task) — nessuna sottrazione prevista
 * per design, quindi il bit restava "incollato".
 *
 * Fix: il gate non usa più FSM_GetState() (letto in modo indipendente e
 * potenzialmente sfasato da questo task) ma un flag dedicato, scritto SOLO
 * dal task FSM esattamente negli stessi due punti che già gestiscono
 * s_latched_errors: TaskMonitor_BeginErrorLatch() (chiamata da
 * action_enter_error(), ad ogni ingresso reale in SYS_ERROR) lo mette a
 * true; TaskMonitor_ClearLatchedErrors() (chiamata da action_clear_error(),
 * solo sul percorso di successo) lo rimette a false NELLO STESSO momento in
 * cui azzera il latch — un solo writer (task FSM), un solo reader (questo
 * task), nessuna finestra in cui i due possano disallinearsi.
 */
static volatile uint32_t s_latched_errors = 0;

/*
 * Flag "siamo genuinamente in un episodio di SYS_ERROR non ancora chiuso"
 * — vedi banner sopra. NON leggere FSM_GetState() per questo scopo: usare
 * SOLO questo flag come gate dell'accumulo in TaskMonitor_Run().
 */
static volatile bool s_error_latch_active = false;

/*
 * Bitmask fault di comunicazione (FAULT_BIT_LASEQ/AMC/COM) attivi in questo
 * momento — puro bookkeeping per telemetria ("GET FAULTS" via RS485),
 * aggiornato da task_comms.c e COM_interface.c tramite
 * TaskMonitor_SetFaultBit(). NON usata per decidere se postare l'evento FSM
 * (quello resta gated su g_config.fault_mask nel chiamante).
 */
static volatile uint32_t s_active_faults = 0;

/*
 * Ultimo error_code grezzo (8 bit) ricevuto da LaseQ (MSG_STATUS byte 1,
 * FSM_ErrorCode_t in fsm.h) — puro bookkeeping per diagnostica RS485
 * ("GET LQERR"), aggiornato da task_comms.c tramite
 * TaskMonitor_SetLaseQErrorCode(). Dal 2026-07-15 (v2) il bit
 * ERR_BIT_LASEQ_INTERNAL in s_active_errors sopra riflette TUTTI i bit
 * rilevanti (0-5 e 7, vedi TaskMonitor_SetErrorBit chiamata da
 * task_comms.c e banner SEVERITÀ in fsm.h); questo campo conserva
 * comunque il byte completo per la decodifica a posteriori (bit-per-bit,
 * incluso il riservato bit 6).
 */
static volatile uint8_t s_lq_error_code = 0U;

/* Buffer raw NTC dall'AD7490 */
static uint16_t s_ntc_raw[AD7490_NUM_CHANNELS];

/* Temperature calcolate per i sensori NTC (°C × 10 per evitare float globale) */
static int16_t  s_ntc_temp_c10[NTC_NUM_SENSORS]; /* 1/10 °C */
static bool     s_ntc_valid[NTC_NUM_SENSORS];     /* false se raw non valido */

/*
 * Telemetria LaseQ (aggiornata da task_comms).
 * s_lq_driver_temp_c[4]: le 4 temperature degli elementi di potenza
 * (MSG_STATUS.temperature[0..3]) — dal 2026-07-14 sorgente della
 * temperatura "driver di corrente" (WARN_BIT_TEMP_DRIVER/ERR_BIT_TEMP),
 * in sostituzione dell'NTC locale CH12. Vedi Monitor_CheckLaseQTelemetry().
 */
static volatile uint8_t s_lq_driver_temp_c[4] = {0};
static volatile int8_t  s_lq_ambient_temp_c = 0;   /* int8_t: range -128..+127 °C */
static volatile uint8_t s_lq_humidity_pct   = 0;

/* Cache lettura SHT35 MMC per il calcolo dew point */
static float s_mmc_temp_c   = 25.0f;   /* ultima T letta da SHT35 MMC [°C] */
static float s_mmc_humid_pct = 50.0f;  /* ultima RH letta da SHT35 MMC [%] */

/*
 * Flag "già segnalato" — evitano flood della coda eventi.
 * flood1/2_reported: indipendenti dal 2026-07-14 (FAULT_BIT_FLOOD1/FLOOD2
 * mascherabili singolarmente) — vedi Monitor_CheckFlood().
 */
static bool flood1_reported   = false;
static bool flood2_reported   = false;

/*
 * Countdown non bloccante prima del taglio dell'eFuse principale su
 * allagamento (vedi Monitor_CheckFlood()). NON usare osDelay() qui: questo
 * task (TaskMonitor, priorità più alta) deve continuare a girare
 * regolarmente ogni MONITOR_PERIOD_MS — Watchdog_Heartbeat() compreso.
 * Un blocco di 5-6s supererebbe WDG_MAX_HEARTBEAT_AGE_MS (5000ms, vedi
 * Watchdog.h), facendo scattare uno stop del refresh IWDG1 proprio durante
 * l'emergenza. Il countdown viene invece decrementato di MONITOR_PERIOD_MS
 * ad ogni ciclo, stesso schema già usato per flow_inhibit_ms/psu_inhibit_ms.
 */
#define FLOOD_SHUTDOWN_DELAY_MS   6000U   /* margine per completare scritture flash/log prima del taglio */
static bool     flood_shutdown_armed = false;  /* countdown in corso, non riarmabile */
static bool     flood_shutdown_done  = false;  /* eFuse già tagliato, non ripetere */
static uint32_t flood_shutdown_ms    = 0U;     /* tempo residuo [ms] prima del taglio */
static bool flow_fault_rep    = false;
static bool psu_reported      = false;

static bool warn_hum_mmc_rep   = false;
static bool warn_hum_lq_rep    = false;
static bool warn_mb_rep        = false;  /**< MB_TEMP (SHT35-MMC) warning già segnalato */
static bool fault_mb_rep       = false;  /**< MB_TEMP (SHT35-MMC) fault già segnalato */
static bool warn_temp_rep[NTC_NUM_SENSORS] = { false };
/*
 * BUGFIX (2026-07-14): era una singola variabile temp_fault_rep condivisa
 * da TUTTI i sensori NTC passati a check_ntc_sensor() (vedi call site sotto).
 * Con più sensori monitorati in sequenza nello stesso ciclo, il ramo "tutto
 * ok" di un sensore resetta incondizionatamente *fault_rep_ptr = false
 * (check_ntc_sensor(), branch finale) — se un altro sensore è in fault,
 * quel reset "cancella" la sua segnalazione già fatta, causando un nuovo
 * Queue_PostEvent(SYS_TEMP_WARN_EVENT/SYS_TEMP_ERROR_EVENT) ad ogni ciclo
 * (100ms) finché il sensore resta guasto, invece che una sola volta —
 * flood continuo sulla coda eventi FSM. Fix: un flag indipendente per
 * sensore, stesso schema già usato per warn_temp_rep[] sopra.
 */
static bool fault_temp_rep[NTC_NUM_SENSORS] = { false };
static bool warn_lq_driver_rep  = false;
static bool warn_lq_ambient_rep = false;
static bool warn_flow_rep       = false;

/*
 * Stato FAULT (non solo "già segnalato") per le due sorgenti di temperatura
 * LaseQ, distinte dai fault_temp_rep[] locali NTC — usate da
 * TaskMonitor_IsLaseQDriverTempFault()/IsLaseQAmbientTempFault() (dal
 * 2026-07-15) per permettere a "GET ALARM" di distinguere QUALE sorgente ha
 * causato SYS_TEMP_ERROR_EVENT (altrimenti ambiguo: lo stesso evento è
 * condiviso da 11 sorgenti indipendenti — 8 NTC locali + MB_TEMP (SHT35-MMC)
 * + driver/ambient LaseQ — vedi fsm.c). warn_lq_driver_rep/warn_lq_ambient_rep sopra non
 * bastano: restano true anche durante il solo WARNING, non solo il FAULT.
 */
static bool lq_driver_temp_fault  = false;
static bool lq_ambient_temp_fault = false;

/* Flag dew point */
static bool warn_dew_mmc_rep   = false;
static bool err_dew_mmc_rep    = false;
static bool warn_dew_lq_rep    = false;
static bool err_dew_lq_rep     = false;

/*
 * Cache Td (punto di condensa) e margine (T_water_in - Td) calcolati da
 * check_dew_margin(), per diagnostica RS485 ("GET DEW", rs485_cmd.c) — vedi
 * TaskMonitor_GetDewMMC()/GetDewLaseQ(). Aggiornati SOLO quando il guard di
 * check_dew_margin() è soddisfatto (WaterCooling RUNNING e NTC water_in
 * valido, vedi Monitor_CheckHumidity()/Monitor_CheckLaseQTelemetry()); i
 * flag s_dew_*_valid indicano se i valori sono attuali (false = cooling
 * fermo o NTC non valido, valori non significativi/stantii).
 */
static bool  s_dew_mmc_valid    = false;
static float s_dew_mmc_td_c     = 0.0f;
static float s_dew_mmc_margin_c = 0.0f;
static bool  s_dew_lq_valid     = false;
static float s_dew_lq_td_c      = 0.0f;
static float s_dew_lq_margin_c  = 0.0f;

/* SHT35: trigger/read alternati ogni SHT35_TRIGGER_CYCLES cicli */
#define SHT35_TRIGGER_CYCLES    2   /* ogni 2×100ms = 200ms */
static uint8_t s_sht35_cycle     = 0;
static bool    s_sht35_triggered = false;

/* ========================================================================== */
/* --- FORWARD PROTOTYPES (funzioni statiche usate prima della definizione) -- */
/* ========================================================================== */

static float dew_point_c(float temp_c, float rh_pct);
static void  check_dew_margin(float amb_temp_c, float rh_pct, float surface_temp_c,
                               uint32_t warn_bit, uint32_t err_bit,
                               bool *warn_rep, bool *err_rep,
                               float *out_td_c, float *out_margin_c);

/* ========================================================================== */
/* --- MACRO HELPER --- */
/* ========================================================================== */

/*
 * Posta un evento FAULT se:
 *   - Il bit error_mask è abilitato
 *   - Lo stato corrente è >= SYS_ON (fault solo quando sistema è operativo)
 * Usare per fault recuperabili (SYS_ERROR).
 */
#define MAYBE_POST_ERROR(event, err_bit)                                      \
    do {                                                                      \
        if (g_config.error_mask & (err_bit)) {                                \
            SysState_t _st = FSM_GetState();                                  \
            if (_st >= SYS_ON) {                                              \
                Queue_PostEvent(event);                                       \
            }                                                                 \
        }                                                                     \
    } while (0)

/*
 * Posta un evento FAULT non recuperabile se il bit è abilitato.
 * Flood e PSU sono fault indipendentemente dallo stato.
 *
 * BUGFIX/REDESIGN (2026-07-14): usava g_config.error_mask (ERR_BIT_FLOOD/
 * ERR_BIT_PSU) — spostato su g_config.fault_mask (FAULT_BIT_FLOOD/
 * FAULT_BIT_PSU, task_monitor.h) per coerenza di severità: questi eventi
 * portano SEMPRE a SYS_FAULT, quindi appartengono alla maschera fault, non
 * a quella error (ora riservata a eventi -> SYS_ERROR).
 */
#define MAYBE_POST_FAULT(event, fault_bit)                                    \
    do {                                                                      \
        if (g_config.fault_mask & (fault_bit)) {                              \
            Queue_PostEvent(event);                                          \
        }                                                                     \
    } while (0)

/*
 * Posta SYS_TEMP_WARN_EVENT se il bit warning è abilitato.
 * Aggiorna sempre s_active_warnings.
 */
#define POST_WARNING_TEMP(warn_bit)                                           \
    do {                                                                      \
        s_active_warnings |= (warn_bit);                                      \
        if (g_config.warning_mask & (warn_bit)) {                             \
            Queue_PostEvent(SYS_TEMP_WARN_EVENT);                             \
        }                                                                     \
    } while (0)

#define POST_WARNING_HUM(warn_bit)                                            \
    do {                                                                      \
        s_active_warnings |= (warn_bit);                                      \
        if (g_config.warning_mask & (warn_bit)) {                             \
            Queue_PostEvent(SYS_HUMIDITY_WARN_EVENT);                         \
        }                                                                     \
    } while (0)

/* ========================================================================== */
/* --- CONVERSIONE NTC --- */
/* ========================================================================== */

/*
 * @brief  Converte un valore ADC grezzo in temperatura [°C × 10].
 *         Modello beta: T = 1 / (1/T0 + ln(R/R0)/beta)
 *         Circuito: VCC → R_series → NTC → GND  (ADC misura tensione su NTC)
 *
 * @param  raw          Valore grezzo AD7490 [0-4095]
 * @param  r0_ohm       Resistenza NTC a T0=25°C [Ω]
 * @param  beta         Coefficiente beta [K]
 * @param  rseries_ohm  Resistenza serie [Ω]
 * @param  out_c10      Puntatore a int16_t dove scrivere T [°C × 10]
 * @retval true se la conversione è valida, false se raw fuori range (cortocircuito/aperto)
 */
static bool ntc_raw_to_celsius10(uint16_t raw,
                                  uint32_t r0_ohm, uint16_t beta,
                                  uint16_t rseries_ohm,
                                  int16_t *out_c10)
{
    /* Protezione: raw=0 → NTC in corto; raw=4095 → NTC aperto */
    if (raw == 0 || raw >= 4095U) {
        return false;
    }

    /*
     * R_ntc = R_series * raw / (4095 - raw)
     * T_K   = 1 / (1/298.15 + ln(R_ntc / R0) / beta)
     */
    float r_ntc = (float)rseries_ohm * (float)raw / (float)(4095U - raw);
    float ln_ratio = logf(r_ntc / (float)r0_ohm);
    float t_k = 1.0f / (1.0f / 298.15f + ln_ratio / (float)beta);
    float t_c = t_k - 273.15f;

    /* Clamp a range ragionevole prima di convertire a int16 */
    if (t_c < -273.0f) t_c = -273.0f;
    if (t_c >  999.0f) t_c =  999.0f;

    *out_c10 = (int16_t)(t_c * 10.0f);
    return true;
}

/* ========================================================================== */
/* --- AGGIORNAMENTO LETTURE NTC --- */
/* ========================================================================== */

static void update_ntc_readings(void)
{
    uint32_t r0      = g_config.ntc_r0_ohm;
    uint16_t rseries = g_config.ntc_rseries_ohm;

    /* Reset validità */
    for (uint8_t s = 0; s < NTC_NUM_SENSORS; s++) {
        s_ntc_valid[s] = false;
    }

    /* Scansiona i 16 canali AD7490 e mappa ai sensor_id */
    for (uint8_t ch = 0; ch < AD7490_NUM_CHANNELS; ch++) {
        uint8_t sensor_id = g_config.ntc_ch_map[ch];

        if (sensor_id == 0xFF || sensor_id >= NTC_NUM_SENSORS) {
            continue; /* Canale non utilizzato */
        }

        /*
         * Beta PER CANALE (dal 2026-07-22, era scalare unico globale): letto
         * qui dentro il loop, indicizzato per canale AD7490 fisico (0-15),
         * NON per sensor_id — vedi commento in config.h su ntc_beta[16].
         */
        uint16_t beta = g_config.ntc_beta[ch];

        int16_t temp_c10 = 0;
        bool ok = ntc_raw_to_celsius10(s_ntc_raw[ch], r0, beta, rseries, &temp_c10);

        if (ok) {
            s_ntc_temp_c10[sensor_id] = temp_c10;
            s_ntc_valid[sensor_id]    = true;
        }
    }
}

/* ========================================================================== */
/* --- CONTROLLI SICUREZZA --- */
/* ========================================================================== */

static void Monitor_CheckFlood(void)
{
    /*
     * FLOOD1/FLOOD2 valutati INDIPENDENTEMENTE dal 2026-07-14: ogni sensore
     * ha il proprio bit in fault_mask (FAULT_BIT_FLOOD1/FLOOD2), il proprio
     * flag di debounce e il proprio bit in TaskMonitor_GetFaults() — così un
     * sensore non montato in produzione può essere mascherato senza perdere
     * la protezione dell'altro. La risposta di sicurezza fisica
     * (WaterCooling_Emergency + countdown taglio eFuse) scatta se ALMENO UNO
     * dei due sensori NON mascherati rileva allagamento.
     *
     * ATTENZIONE (vedi anche task_monitor.h): mascherare un sensore
     * REALMENTE collegato disabilita l'INTERA risposta per quel canale
     * (non solo la notifica) — solo per debug/commissioning, mai con acqua
     * realmente collegata su quel sensore.
     */
    bool raw1 = FloodSensorGetStatus(FLOOD1);
    bool raw2 = FloodSensorGetStatus(FLOOD2);

    bool masked1 = (g_config.fault_mask & FAULT_BIT_FLOOD1) != 0U;
    bool masked2 = (g_config.fault_mask & FAULT_BIT_FLOOD2) != 0U;

    bool flood1 = masked1 && raw1;
    bool flood2 = masked2 && raw2;

    if (!masked1) {
        flood1_reported = false;
        TaskMonitor_SetFaultBit(FAULT_BIT_FLOOD1, false);
    } else if (flood1) {
        TaskMonitor_SetFaultBit(FAULT_BIT_FLOOD1, true);
        if (!flood1_reported) {
            MAYBE_POST_FAULT(SYS_FLOOD_FAULT_EVENT, FAULT_BIT_FLOOD1);
            flood1_reported = true;
        }
    } else {
        flood1_reported = false;
        TaskMonitor_SetFaultBit(FAULT_BIT_FLOOD1, false);
    }

    if (!masked2) {
        flood2_reported = false;
        TaskMonitor_SetFaultBit(FAULT_BIT_FLOOD2, false);
    } else if (flood2) {
        TaskMonitor_SetFaultBit(FAULT_BIT_FLOOD2, true);
        if (!flood2_reported) {
            MAYBE_POST_FAULT(SYS_FLOOD_FAULT_EVENT, FAULT_BIT_FLOOD2);
            flood2_reported = true;
        }
    } else {
        flood2_reported = false;
        TaskMonitor_SetFaultBit(FAULT_BIT_FLOOD2, false);
    }

    bool flood = flood1 || flood2;

    if (flood) {
        /* Risposta idraulica immediata: chiudere l'acqua non aspetta il countdown */
        WaterCooling_Emergency();

        /*
         * Arma il countdown di spegnimento UNA SOLA VOLTA (non si riarma né
         * si allunga su riletture successive di flood==true, incluse quelle
         * dopo un eventuale rientro spurio del sensore — vedi ramo "else"
         * sotto: una volta rilevato l'allagamento il taglio dell'eFuse
         * principale è ormai deciso e non va rimandato).
         */
        if (!flood_shutdown_armed && !flood_shutdown_done) {
            flood_shutdown_armed = true;
            flood_shutdown_ms    = FLOOD_SHUTDOWN_DELAY_MS;
            SysLog_Event(LOG_ERROR,
                         "FLOOD: allagamento rilevato (FLOOD1:%d FLOOD2:%d), eFuse principale tra %lums",
                         (int)flood1, (int)flood2,
                         (unsigned long)FLOOD_SHUTDOWN_DELAY_MS);
        }
    }
    /*
     * NON annulla un countdown già armato quando flood torna false: un
     * rientro spurio del sensore (bounce, spruzzo isolato) non deve
     * rimandare un taglio di sicurezza già deciso — fail-secure.
     */

    /*
     * Countdown non bloccante (vedi commento sulle variabili sopra):
     * decrementato di MONITOR_PERIOD_MS ad ogni ciclo mentre armato.
     */
    if (flood_shutdown_armed) {
        if (flood_shutdown_ms > MONITOR_PERIOD_MS) {
            flood_shutdown_ms -= MONITOR_PERIOD_MS;
        } else {
            flood_shutdown_ms = 0U;

            /*
             * Prima di tagliare l'alimentazione principale:
             *
             * 1) Flash (Config_Save()/LUT_Save(), protette dal mutex
             *    condiviso creato in Config_CreateFlashMutex()): con 6s di
             *    margine una scrittura in corso ha già finito da tempo in
             *    condizioni normali; acquisire+rilasciare il mutex qui è
             *    solo una rete di sicurezza aggiuntiva, con timeout breve
             *    per non ritardare indefinitamente un taglio di sicurezza
             *    se qualcosa restasse anomalmente bloccato.
             * 2) Log su SD (SysLog_Event): write_line() in sys_log.c fa
             *    f_open→f_write→f_close per OGNI riga, quindi al ritorno
             *    della chiamata il dato è già fisicamente sul supporto —
             *    nessun flush separato necessario, basta che la chiamata
             *    sia completata PRIMA di EFuse_Disable().
             */
            if (Config_FlashMutexAcquire(200U)) {
                Config_FlashMutexRelease();
            }

            SysLog_Event(LOG_ERROR,
                         "FLOOD: margine di %lums esaurito, taglio eFuse principale",
                         (unsigned long)FLOOD_SHUTDOWN_DELAY_MS);

            //Allagamento confermato da almeno FLOOD_SHUTDOWN_DELAY_MS: spengo il fusibile principale per limitare rischio cortocircuiti
            EFuse_Disable(EFUSE_MAIN);

            flood_shutdown_armed = false;
            flood_shutdown_done  = true;
        }
    }
}

static void Monitor_CheckFlow(void)
{
    /*
     * flow_sensor_enabled_mask (g_config, bit0=FLOW_METER_1): device-enable
     * dedicato, stesso pattern di psu_enabled_mask/contactor_enabled_mask —
     * a differenza di WARN_BIT_FLOW (che maschera solo la NOTIFICA), questo
     * bit disabilita interamente il check quando il flussometro non è
     * fisicamente installato (vedi FlowMeter_Init(FLOW_METER_2,...) commentato
     * in main.c). Solo FLOW_METER_1 è cablato oggi: bit1 riservato per un
     * futuro secondo flussometro.
     */
    if (!(g_config.flow_sensor_enabled_mask & 0x01U)) {
        flow_fault_rep  = false;
        warn_flow_rep   = false;
        s_active_warnings &= ~WARN_BIT_FLOW;
        s_active_errors   &= ~ERR_BIT_FLOW;
        return;
    }

    /*
     * BUGFIX (2026-07-31, "GET ERR mostra sempre FLOW=1 anche dopo CERR e
     * dopo che il flusso è rientrato"): questi due return anticipati
     * azzeravano solo s_active_warnings, MAI s_active_errors — a differenza
     * del ramo "else" più sotto (flusso rientrato in soglia), che è l'UNICO
     * punto che faceva s_active_errors &= ~ERR_BIT_FLOW. Se il pompaggio non
     * è WC_STATE_RUNNING (es. dopo WaterCooling_Emergency() in un errore di
     * flusso: resta WC_STATE_ERROR anche dopo un CERR riuscito finché non
     * si rientra in SYS_ON) o durante l'inibizione post-apertura valvola,
     * questa funzione smette di rivalutare il flusso — il bit restava
     * "congelato" al valore del guasto originale a tempo indeterminato,
     * indipendentemente dal flusso fisico reale. In nessuno dei due casi
     * ha senso dichiarare un errore di flusso attivo (o non stiamo neppure
     * cercando di mantenere flusso, o siamo deliberatamente in una finestra
     * di grazia): va quindi chiarito qui, non lasciato al valore precedente.
     */
    if (WaterCooling_GetState() != WC_STATE_RUNNING) {
        flow_fault_rep  = false;
        warn_flow_rep   = false;
        s_active_warnings &= ~WARN_BIT_FLOW;
        s_active_errors   &= ~ERR_BIT_FLOW;
        return;
    }

    /* Inibizione dopo apertura valvola */
    if (flow_inhibit_ms > 0) {
        flow_inhibit_ms = (flow_inhibit_ms > MONITOR_PERIOD_MS)
                          ? (flow_inhibit_ms - MONITOR_PERIOD_MS) : 0U;
        flow_fault_rep  = false;
        s_active_errors &= ~ERR_BIT_FLOW;
        return;
    }

    float flow     = FlowMeter_GetFlowRate(FLOW_METER_1);
    float flow_min_err  = (float)g_config.flow_min_err_lpm_x10  / 10.0f;
    float flow_min_warn = (float)g_config.flow_min_warn_lpm_x10 / 10.0f;
    float flow_max_err  = (float)g_config.flow_max_err_lpm_x10  / 10.0f;
    float flow_max_warn = (float)g_config.flow_max_warn_lpm_x10 / 10.0f;

    bool fault = (flow < flow_min_err) || (flow > flow_max_err);
    bool warn  = !fault && ((flow < flow_min_warn) || (flow > flow_max_warn));

    if (fault) {
        /* Il fault di flusso è un errore operativo: l'evento FSM viene postato
         * solo da SYS_ON in poi. In ACTIVE il raffreddamento si sta avviando
         * (pompa non ancora a regime) e un flusso basso è fisiologico.
         *
         * Il flag viene tenuto a false mentre stato < SYS_ON: così la prima
         * verifica in SYS_ON trova il flag libero e posta immediatamente
         * l'evento, anche se la condizione era già presente in ACTIVE. */
        SysState_t _st = FSM_GetState();
        if (_st < SYS_ON) {
            flow_fault_rep = false;   /* non operativi: non postare */
        } else if (!flow_fault_rep) {
            if (g_config.error_mask & ERR_BIT_FLOW) {
                Queue_PostEvent(SYS_FLOW_ERROR_EVENT);
            }
            flow_fault_rep = true;
        }
        s_active_errors |= ERR_BIT_FLOW;
    } else {
        flow_fault_rep = false;
        s_active_errors &= ~ERR_BIT_FLOW;
    }

    if (warn) {
        s_active_warnings |= WARN_BIT_FLOW;
        if (!warn_flow_rep) {
            if (g_config.warning_mask & WARN_BIT_FLOW) {
                Queue_PostEvent(SYS_TEMP_WARN_EVENT); /* riuso evento generico */
            }
            warn_flow_rep = true;
        }
    } else {
        s_active_warnings &= ~WARN_BIT_FLOW;
        warn_flow_rep = false;
    }
}

/*
 * @brief  Rispecchia in s_active_errors lo stato LIVE dei quattro canali SAB
 *         (INTLCK A/B, TEST A/B), letto direttamente da SAB.c ad ogni ciclo.
 *
 *         NUOVO (2026-07-31, bugfix): prima di questa funzione nessun punto
 *         del firmware scriveva mai ERR_BIT_SAB_INTLCK_A/B/ERR_BIT_SAB_TEST_A/B
 *         in s_active_errors — l'apertura di un interlock arrivava alla FSM
 *         SOLO come evento one-shot (SYS_SAB_INTERLOCK_OPEN_EVENT, via
 *         sab_event_cb() in freertos.c) e produceva correttamente il
 *         dettaglio "Interlock A/B" in "GET ALARM" (FSM_GetLastErrorName()),
 *         ma "GET ERR"/COM_STAT_OP_ERR_ACTIVE (TaskMonitor_GetErrors())
 *         restava sempre a 0x0 per questa causa: i due comandi leggono fonti
 *         diverse (evento loggato vs bitmask accumulato) e solo la prima
 *         veniva popolata.
 *
 *         A differenza di check_ntc_sensor()/Monitor_CheckFlow() non serve
 *         gestire soglie warn/err separate (i pin SAB sono già booleani
 *         OK/fault) né *_rep/Queue_PostEvent: la notifica FSM resta gestita
 *         da sab_event_cb() (freertos.c, guidata da EXTI) — questa funzione
 *         si limita a tenere il bitmask errori coerente con lo stato fisico
 *         corrente, cosi' "GET ERR" riflette sempre la realtà, e (insieme al
 *         latch in TaskMonitor_Run() sotto) resta visibile per tutta la
 *         durata di SYS_ERROR anche se l'interlock si richiude prima del CERR.
 *
 *         g_config.sab_enabled: se il modulo SAB è disabilitato (variante di
 *         prodotto senza SAB, o disattivato per test banco) i pin non sono
 *         significativi — stesso principio di device-enable di
 *         temp_sensor_enabled_mask/flow_sensor_enabled_mask.
 *
 *         BUGFIX (2026-07-31, "interlock/SAB_TEST compaiono in GET ERR anche
 *         se l'errore che ha portato a SYS_ERROR è tutt'altro, es. FLOW, e
 *         la macchina era ancora in SYS_ON"): SAB_GetInterlockStatus()/
 *         SAB_GetTestStatus() leggono i pin RAW, indipendentemente da quanto
 *         il modulo SAB sia effettivamente armato. Finché SAB_Enable() non è
 *         stato chiamato (action_enter_enabled(), transizione SYS_ON→
 *         SYS_ENABLED), il modulo è SAB_STATE_DISABLED e nSAB_INTLCK_A/B_STATUS
 *         può benissimo leggere "aperto" semplicemente perché non è ancora
 *         stato richiesto l'armo — non è un fault, è la condizione normale
 *         in IDLE/ACTIVE/ON. Senza questo filtro, QUALSIASI altra causa che
 *         portasse la macchina in SYS_ERROR mentre si è ancora in SYS_ON (es.
 *         flusso fuori soglia) si trascinava dietro, nel latch, un falso
 *         positivo su interlock/test SAB solo perché il modulo non era mai
 *         stato armato. Simmetricamente in SAB_STATE_GUARD (subito dopo
 *         nSAB_EN, i pin non sono ancora validi per costruzione — vedi
 *         SAB_GUARD_DELAY_MS in SAB.h) altrettanto non significativo.
 *         Rilevante da SAB_STATE_ENABLING in poi (compreso il timeout, già
 *         gestito a parte come seed one-shot in action_enter_error()) e per
 *         tutta la durata di SAB_STATE_ARMED/SAB_STATE_FAULT — cioè esattamente
 *         "dalla richiesta di transizione ON→ENABLED in poi", come da
 *         requisito esplicito: la logica di transizione FSM (sab_event_cb()
 *         in freertos.c) era già corretta in questo senso, il problema era
 *         solo qui, nel check continuo per la persistenza in GET ERR.
 */
static void Monitor_CheckSAB(void)
{
    if (!g_config.sab_enabled) {
        s_active_errors &= ~(ERR_BIT_SAB_INTLCK_A | ERR_BIT_SAB_INTLCK_B
                              | ERR_BIT_SAB_TEST_A | ERR_BIT_SAB_TEST_B);
        return;
    }

    {
        SAB_State_t sab_st = SAB_GetState();
        if (sab_st == SAB_STATE_DISABLED || sab_st == SAB_STATE_GUARD) {
            s_active_errors &= ~(ERR_BIT_SAB_INTLCK_A | ERR_BIT_SAB_INTLCK_B
                                  | ERR_BIT_SAB_TEST_A | ERR_BIT_SAB_TEST_B);
            return;
        }
    }

    bool intlck_a_open = false, intlck_b_open = false;
    bool test_a_fault  = false, test_b_fault  = false;

    SAB_GetInterlockStatus(&intlck_a_open, &intlck_b_open);
    SAB_GetTestStatus(&test_a_fault, &test_b_fault);

    if (intlck_a_open) { s_active_errors |= ERR_BIT_SAB_INTLCK_A; }
    else                { s_active_errors &= ~ERR_BIT_SAB_INTLCK_A; }

    if (intlck_b_open) { s_active_errors |= ERR_BIT_SAB_INTLCK_B; }
    else                { s_active_errors &= ~ERR_BIT_SAB_INTLCK_B; }

    if (test_a_fault) { s_active_errors |= ERR_BIT_SAB_TEST_A; }
    else               { s_active_errors &= ~ERR_BIT_SAB_TEST_A; }

    if (test_b_fault) { s_active_errors |= ERR_BIT_SAB_TEST_B; }
    else               { s_active_errors &= ~ERR_BIT_SAB_TEST_B; }
}

/*
 * @brief  Rispecchia in s_active_errors lo stato LIVE dei due contatti
 *         chiave (KEY_A/B), letto direttamente da Key.c ad ogni ciclo.
 *
 *         NUOVO (2026-07-31, stessa richiesta di Monitor_CheckSAB() sopra,
 *         generalizzata a tutte le cause di SYS_ERROR): prima di questa
 *         funzione ERR_BIT_KEY_A/B non venivano mai scritti in
 *         s_active_errors — la rimozione chiave arrivava alla FSM solo come
 *         evento one-shot (SYS_KEY_REMOVED_EVENT), "GET ERR" restava a 0x0
 *         per questa causa. Nessuna gestione warn/err o *_rep: i contatti
 *         sono booleani OK/fault, la notifica FSM resta a carico di chi
 *         posta SYS_KEY_REMOVED_EVENT (invariato). g_config.error_mask NON
 *         viene applicato qui (stesso principio di check_ntc_sensor()/
 *         Monitor_CheckFlow(): la maschera filtra solo l'evento/notifica, mai
 *         il bit "live" — un contatto non montato in produzione ha comunque
 *         il proprio bit ERR_BIT_KEY_* mascherato a monte in error_mask, che
 *         a sua volta impedisce la transizione a SYS_ERROR: il bit qui
 *         semplicemente non ha mai occasione di essere osservato mentre in
 *         errore, coerente col resto del bitmask).
 */
static void Monitor_CheckKey(void)
{
    if (Key_GetKeyStatus(KEY_A)) { s_active_errors &= ~ERR_BIT_KEY_A; }
    else                          { s_active_errors |=  ERR_BIT_KEY_A; }

    if (Key_GetKeyStatus(KEY_B)) { s_active_errors &= ~ERR_BIT_KEY_B; }
    else                          { s_active_errors |=  ERR_BIT_KEY_B; }
}

/*
 * @brief  Rispecchia in s_active_errors lo stato LIVE dei due sensori
 *         coperchio (LID1/2), letto direttamente da Lid.c ad ogni ciclo.
 *         Stesso schema/motivazione di Monitor_CheckKey() sopra.
 */
static void Monitor_CheckLid(void)
{
    if (Lid_GetStatus(LID_1)) { s_active_errors |=  ERR_BIT_LID1; }
    else                       { s_active_errors &= ~ERR_BIT_LID1; }

    if (Lid_GetStatus(LID_2)) { s_active_errors |=  ERR_BIT_LID2; }
    else                       { s_active_errors &= ~ERR_BIT_LID2; }
}

static void Monitor_CheckPSU(void)
{
    SysState_t state = FSM_GetState();

    if (state < SYS_ON) {
        /* Sistema non ancora in ON: reset stato e inibizione */
        psu_reported  = false;
        psu_inhibit_ms = 0U;
        TaskMonitor_SetFaultBit(FAULT_BIT_PSU, false);
        return;
    }

    /*
     * Inibizione dopo SON: l'alimentatore ha bisogno di tempo per raggiungere
     * tensione stabile e asserire DC_OK. Il timer viene avviato da
     * TaskMonitor_ResetPSUInhibit() chiamato in action_enter_on().
     * Durante l'inibizione: decrementa timer, resetta il flag segnalato
     * (così al primo controllo reale partiremo da uno stato pulito) e
     * lascia lo stato fault invariato (no allarmi prematuri).
     */
    if (psu_inhibit_ms > 0U) {
        psu_inhibit_ms = (psu_inhibit_ms > MONITOR_PERIOD_MS)
                         ? (psu_inhibit_ms - MONITOR_PERIOD_MS) : 0U;
        psu_reported = false;
        return;
    }

    PSU_t psu1 = PSUGetStatus(PSU1);
    PSU_t psu2 = PSUGetStatus(PSU2);

    bool fault = psu1.alarm || psu2.alarm ||
                 (psu1.status && !psu1.dc_ok) ||
                 (psu2.status && !psu2.dc_ok);

    /*
     * REDESIGN (2026-07-14): s_active_errors/"GET ERR" -> TaskMonitor_
     * SetFaultBit()/"GET FAULTS", e il gate del Queue_PostEvent da
     * g_config.error_mask (ERR_BIT_PSU) a g_config.fault_mask
     * (FAULT_BIT_PSU) — vedi banner in task_monitor.h. PSU porta sempre a
     * SYS_FAULT (non recuperabile), quindi appartiene alla maschera fault.
     */
    if (fault) {
        TaskMonitor_SetFaultBit(FAULT_BIT_PSU, true);
        if (!psu_reported) {
            MAYBE_POST_FAULT(SYS_PSU_FAULT_EVENT, FAULT_BIT_PSU);
            psu_reported = true;
        }
    } else {
        TaskMonitor_SetFaultBit(FAULT_BIT_PSU, false);
        psu_reported = false;
    }
}

/*
 * @brief  Controlla una coppia di soglie warn/err per un sensore NTC.
 *         Aggiorna s_active_warnings, s_active_errors e posta eventi.
 *
 * @param  sensor_id       Indice in NTC_SensorId_t
 * @param  min_err_c       Soglia minima fault [°C]
 * @param  min_warn_c      Soglia minima warning [°C]
 * @param  max_warn_c      Soglia massima warning [°C]
 * @param  max_err_c       Soglia massima fault [°C]
 * @param  warn_bit        Bit WARN_BIT_* corrispondente
 * @param  temp_enable_bit Bit (0-8) in g_config.temp_sensor_enabled_mask
 *                         corrispondente a questo sensore (device-enable
 *                         dedicato, stesso pattern di psu_enabled_mask).
 * @param  warn_rep        Puntatore al flag "warning già segnalato"
 * @param  fault_rep_ptr   Puntatore al flag "fault già segnalato"
 */
static void check_ntc_sensor(uint8_t sensor_id,
                              int16_t min_err_c,  int16_t min_warn_c,
                              int16_t max_warn_c, int16_t max_err_c,
                              uint32_t warn_bit, uint8_t temp_enable_bit,
                              bool *warn_rep, bool *fault_rep_ptr)
{
    /*
     * temp_sensor_enabled_mask (g_config): device-enable dedicato, stesso
     * pattern di psu_enabled_mask/contactor_enabled_mask — un sensore NTC
     * non fisicamente installato va escluso qui, non solo mascherato in
     * notifica. Sostituisce il precedente riuso improprio di warning_mask
     * come "sensore cablato o no" (warning_mask torna a mascherare SOLO la
     * notifica dell'evento, come per tutti gli altri WARN_BIT_*).
     */
    if (!(g_config.temp_sensor_enabled_mask & (1U << temp_enable_bit))) {
        s_active_warnings &= ~warn_bit;
        s_active_errors   &= ~ERR_BIT_TEMP;
        *warn_rep      = false;
        *fault_rep_ptr = false;
        return; /* Sensore non abilitato: non monitorato */
    }

    bool fault;
    bool warn;

    if (!s_ntc_valid[sensor_id]) {
        /* raw==0 o raw>=4095: NTC in corto o circuito aperto (scollegato).
         * Trattato come fault, non come "dato assente". */
        fault = true;
        warn  = false;
    } else {
        /* Convertire soglie in decimi di grado per confronto coerente */
        int16_t t = s_ntc_temp_c10[sensor_id];
        int16_t min_err_c10  = (int16_t)(min_err_c  * 10);
        int16_t min_warn_c10 = (int16_t)(min_warn_c * 10);
        int16_t max_warn_c10 = (int16_t)(max_warn_c * 10);
        int16_t max_err_c10  = (int16_t)(max_err_c  * 10);

        fault = (t <= min_err_c10) || (t >= max_err_c10);
        warn  = !fault && ((t <= min_warn_c10) || (t >= max_warn_c10));
    }

    if (fault) {
        s_active_warnings |= warn_bit;  /* fault implica warning nell'UI */
        if (!(*fault_rep_ptr)) {
            /* Fault: solo da SYS_ON in poi */
            SysState_t state = FSM_GetState();
            if (state >= SYS_ON) {
                if (g_config.error_mask & ERR_BIT_TEMP) {
                    Queue_PostEvent(SYS_TEMP_ERROR_EVENT);
                }
                s_active_errors |= ERR_BIT_TEMP;
            } else {
                /* Prima di SYS_ON: solo warning */
                if (g_config.warning_mask & warn_bit) {
                    Queue_PostEvent(SYS_TEMP_WARN_EVENT);
                }
            }
            *fault_rep_ptr = true;
        }
    } else if (warn) {
        s_active_warnings |= warn_bit;
        if (!(*warn_rep)) {
            POST_WARNING_TEMP(warn_bit);
            *warn_rep = true;
        }
    } else {
        /* Tutto ok: reset flag e warning bit */
        s_active_warnings &= ~warn_bit;
        *warn_rep      = false;
        *fault_rep_ptr = false;
        s_active_errors &= ~ERR_BIT_TEMP; /* Solo se tutti i sensori tornano ok */
    }
}

static void Monitor_CheckTemperatures(void)
{
    update_ntc_readings();

    check_ntc_sensor(NTC_SENSOR_WATER_IN,
                     g_config.temp_water_in_min_err,  g_config.temp_water_in_min_warn,
                     g_config.temp_water_in_max_warn,  g_config.temp_water_in_max_err,
                     WARN_BIT_TEMP_WATER_IN, 0U,
                     &warn_temp_rep[NTC_SENSOR_WATER_IN], &fault_temp_rep[NTC_SENSOR_WATER_IN]);

    check_ntc_sensor(NTC_SENSOR_WATER_OUT,
                     g_config.temp_water_out_min_err,  g_config.temp_water_out_min_warn,
                     g_config.temp_water_out_max_warn,  g_config.temp_water_out_max_err,
                     WARN_BIT_TEMP_WATER_OUT, 1U,
                     &warn_temp_rep[NTC_SENSOR_WATER_OUT], &fault_temp_rep[NTC_SENSOR_WATER_OUT]);

    /*
     * NTC_SENSOR_DRIVER (CH12) NON viene più controllato qui dal
     * 2026-07-14: la temperatura "driver di corrente" (WARN_BIT_TEMP_DRIVER)
     * arriva ora dalle 4 temperature riportate da LaseQ
     * (MSG_STATUS.temperature[0..3]), verificate in
     * Monitor_CheckLaseQTelemetry() sotto — vedi commento su
     * NTC_SENSOR_DRIVER in task_monitor.h.
     */

    check_ntc_sensor(NTC_SENSOR_SPLICE,
                     g_config.temp_splice_min_err,  g_config.temp_splice_min_warn,
                     g_config.temp_splice_max_warn,  g_config.temp_splice_max_err,
                     WARN_BIT_TEMP_SPLICE, 3U,
                     &warn_temp_rep[NTC_SENSOR_SPLICE], &fault_temp_rep[NTC_SENSOR_SPLICE]);

    /*
     * DIODE1/DIODE2 (dal 2026-07-15): il diodo laser ha due sensori NTC
     * fisici indipendenti (serigrafia NTC3/CH12 e NTC4/CH13) — letti e
     * verificati SEPARATAMENTE, ciascuno con soglie/bit/device-enable
     * propri (stesso principio di WATER_IN/WATER_OUT o KEY_A/KEY_B). Un
     * fault su uno dei due NON implica l'altro: entrambi vanno monitorati
     * indipendentemente, es. per rilevare uno squilibrio termico tra i due
     * lati del componente.
     */
    check_ntc_sensor(NTC_SENSOR_DIODE1,
                     g_config.temp_diode_min_err,  g_config.temp_diode_min_warn,
                     g_config.temp_diode_max_warn,  g_config.temp_diode_max_err,
                     WARN_BIT_TEMP_DIODE1, 4U,
                     &warn_temp_rep[NTC_SENSOR_DIODE1], &fault_temp_rep[NTC_SENSOR_DIODE1]);

    check_ntc_sensor(NTC_SENSOR_DIODE2,
                     g_config.temp_diode2_min_err,  g_config.temp_diode2_min_warn,
                     g_config.temp_diode2_max_warn,  g_config.temp_diode2_max_err,
                     WARN_BIT_TEMP_DIODE2, 6U,
                     &warn_temp_rep[NTC_SENSOR_DIODE2], &fault_temp_rep[NTC_SENSOR_DIODE2]);

    check_ntc_sensor(NTC_SENSOR_AMBIENT,
                     g_config.temp_ambient_min_err,  g_config.temp_ambient_min_warn,
                     g_config.temp_ambient_max_warn,  g_config.temp_ambient_max_err,
                     WARN_BIT_TEMP_AMBIENT, 5U,
                     &warn_temp_rep[NTC_SENSOR_AMBIENT], &fault_temp_rep[NTC_SENSOR_AMBIENT]);

    /*
     * PSU_TEMP / PWR_EL_TEMP (NUOVO 2026-07-22): due nuovi slot NTC
     * funzionali, stesso schema degli altri sopra. Non ancora cablati in
     * campo (device-enable bit 7/8 lasciati a 0 in Config_LoadDefaults()
     * finché la sonda fisica non viene installata) — se il canale non è
     * abilitato, check_ntc_sensor() ritorna subito senza generare fault.
     */
    check_ntc_sensor(NTC_SENSOR_PSU_TEMP,
                     g_config.temp_psu_min_err,  g_config.temp_psu_min_warn,
                     g_config.temp_psu_max_warn,  g_config.temp_psu_max_err,
                     WARN_BIT_TEMP_PSU, 7U,
                     &warn_temp_rep[NTC_SENSOR_PSU_TEMP], &fault_temp_rep[NTC_SENSOR_PSU_TEMP]);

    check_ntc_sensor(NTC_SENSOR_PWR_EL_TEMP,
                     g_config.temp_pwr_el_min_err,  g_config.temp_pwr_el_min_warn,
                     g_config.temp_pwr_el_max_warn,  g_config.temp_pwr_el_max_err,
                     WARN_BIT_TEMP_PWR_EL, 8U,
                     &warn_temp_rep[NTC_SENSOR_PWR_EL_TEMP], &fault_temp_rep[NTC_SENSOR_PWR_EL_TEMP]);
}

static void Monitor_CheckLaseQTelemetry(void)
{
    /*
     * Temperature dei 4 elementi di potenza LaseQ (temperature[0..3] in
     * MSG_STATUS, escluso temp_ambient_c che è gestito a parte sotto).
     * Confronto con le soglie del driver (temp_driver_*): stesso tipo di
     * sensore NTC, stessa fisica del punto di misura, indipendentemente da
     * quale dei 4 elementi sia effettivamente attivo.
     *
     * BASTA UNA SOLA temperatura fuori soglia per considerare l'intero
     * gruppo in warning/fault — non serve che siano tutte e 4 fuori range
     * (ogni elemento di potenza è un punto di guasto indipendente).
     * Soglie in °C interi (uint8_t).
     *
     * Sostituisce dal 2026-07-14 il vecchio controllo su un solo valore
     * (temperature[0]) e il sensore NTC locale CH12 (NTC_SENSOR_DRIVER,
     * non più letto per questo scopo — vedi Monitor_CheckTemperatures()
     * sopra). WARN_BIT_TEMP_LQ_DRIVER è deprecato: WARN_BIT_TEMP_DRIVER
     * copre ora questo intero gruppo.
     */
    bool lq_drv_fault = false;
    bool lq_drv_warn  = false;

    for (uint8_t i = 0U; i < 4U; i++) {
        uint8_t t = s_lq_driver_temp_c[i];

        if ((t <= (uint8_t)g_config.temp_driver_min_err) ||
            (t >= (uint8_t)g_config.temp_driver_max_err)) {
            lq_drv_fault = true;
        } else if ((t <= (uint8_t)g_config.temp_driver_min_warn) ||
                   (t >= (uint8_t)g_config.temp_driver_max_warn)) {
            lq_drv_warn = true;
        }
    }

    /*
     * Stato FAULT esposto via TaskMonitor_IsLaseQDriverTempFault() (vedi
     * dichiarazione statica lq_driver_temp_fault sopra) — riflette la
     * condizione raw di soglia, non il flag "già segnalato" (warn_lq_driver_rep,
     * che resta true anche solo per il warning).
     */
    lq_driver_temp_fault = lq_drv_fault;

    if (lq_drv_fault) {
        s_active_warnings |= WARN_BIT_TEMP_DRIVER;
        SysState_t state = FSM_GetState();
        if (state >= SYS_ON && !warn_lq_driver_rep) {
            /* "se non mascherato": gated da ERR_BIT_TEMP in error_mask,
             * stesso bit già usato da check_ntc_sensor() per gli altri
             * sensori di temperatura — nessuna maschera dedicata separata. */
            if (g_config.error_mask & ERR_BIT_TEMP) {
                Queue_PostEvent(SYS_TEMP_ERROR_EVENT);
            }
            s_active_errors |= ERR_BIT_TEMP;
            warn_lq_driver_rep = true;
        } else if (state < SYS_ON && !warn_lq_driver_rep) {
            POST_WARNING_TEMP(WARN_BIT_TEMP_DRIVER);
            warn_lq_driver_rep = true;
        }
    } else if (lq_drv_warn) {
        s_active_warnings |= WARN_BIT_TEMP_DRIVER;
        if (!warn_lq_driver_rep) {
            POST_WARNING_TEMP(WARN_BIT_TEMP_DRIVER);
            warn_lq_driver_rep = true;
        }
    } else {
        s_active_warnings &= ~WARN_BIT_TEMP_DRIVER;
        s_active_errors   &= ~ERR_BIT_TEMP;
        warn_lq_driver_rep = false;
    }

    /*
     * Temperatura ambiente LaseQ (SHT35, campo temp_ambient_c in MSG_STATUS).
     * int8_t firmato: le soglie di config sono int16_t → cast coerente.
     */
    int8_t lq_amb = s_lq_ambient_temp_c;

    /*
     * Dal 2026-07-15: soglie DEDICATE per l'ambiente interno LaseQ
     * (temp_lq_ambient_*), non più condivise con l'ambiente board MMC
     * (temp_ambient_*, letto da SHT-35 su MMC). L'interno di LaseQ è
     * fisiologicamente più caldo per via della vicinanza di componenti
     * elettronici di potenza — vedi campo temp_lq_ambient_* in config.h.
     */
    bool lq_amb_fault = ((int16_t)lq_amb <= g_config.temp_lq_ambient_min_err) ||
                        ((int16_t)lq_amb >= g_config.temp_lq_ambient_max_err);
    bool lq_amb_warn  = !lq_amb_fault &&
                        (((int16_t)lq_amb <= g_config.temp_lq_ambient_min_warn) ||
                         ((int16_t)lq_amb >= g_config.temp_lq_ambient_max_warn));

    /* Stato FAULT esposto via TaskMonitor_IsLaseQAmbientTempFault() (vedi
     * commento analogo su lq_driver_temp_fault sopra). */
    lq_ambient_temp_fault = lq_amb_fault;

    if (lq_amb_fault) {
        s_active_warnings |= WARN_BIT_TEMP_LQ_AMBIENT;
        SysState_t state = FSM_GetState();
        if (state >= SYS_ON && !warn_lq_ambient_rep) {
            if (g_config.error_mask & ERR_BIT_TEMP) {
                Queue_PostEvent(SYS_TEMP_ERROR_EVENT);
            }
            warn_lq_ambient_rep = true;
        } else if (state < SYS_ON && !warn_lq_ambient_rep) {
            POST_WARNING_TEMP(WARN_BIT_TEMP_LQ_AMBIENT);
            warn_lq_ambient_rep = true;
        }
    } else if (lq_amb_warn) {
        s_active_warnings |= WARN_BIT_TEMP_LQ_AMBIENT;
        if (!warn_lq_ambient_rep) {
            POST_WARNING_TEMP(WARN_BIT_TEMP_LQ_AMBIENT);
            warn_lq_ambient_rep = true;
        }
    } else {
        s_active_warnings &= ~WARN_BIT_TEMP_LQ_AMBIENT;
        warn_lq_ambient_rep = false;
    }

    /*
     * Umidità LaseQ (humidity in MSG_STATUS, [%RH uint8_t, 0-100]).
     * Soglia separata rispetto all'MMC (laseq_humidity_max_warn_pct).
     * Solo warning: umidità alta non è un fault immediato ma va comunicata.
     */
    uint8_t lq_hum = s_lq_humidity_pct;
    if (lq_hum > g_config.laseq_humidity_max_warn_pct) {
        s_active_warnings |= WARN_BIT_HUMIDITY_LASEQ;
        if (!warn_hum_lq_rep) {
            POST_WARNING_HUM(WARN_BIT_HUMIDITY_LASEQ);
            warn_hum_lq_rep = true;
        }
    } else {
        s_active_warnings &= ~WARN_BIT_HUMIDITY_LASEQ;
        warn_hum_lq_rep = false;
    }

    /*
     * Dew point LaseQ.
     * L'aria interna al LaseQ è descritta da (temp_ambient_c, humidity).
     * La superficie fredda di riferimento è sempre T_water_in: il loop
     * idraulico raffredda sia la scheda MMC che il LaseQ.
     * Guard: cooling RUNNING e NTC water_in valido.
     */
    if (WaterCooling_GetState() == WC_STATE_RUNNING &&
        s_ntc_valid[NTC_SENSOR_WATER_IN])
    {
        float lq_temp_f  = (float)(int8_t)s_lq_ambient_temp_c;
        float lq_rh_f    = (float)lq_hum;   /* già in %RH [0-100] */
        float t_water_in = (float)s_ntc_temp_c10[NTC_SENSOR_WATER_IN] / 10.0f;

        check_dew_margin(lq_temp_f, lq_rh_f, t_water_in,
                         WARN_BIT_DEW_LASEQ, ERR_BIT_DEW_LASEQ,
                         &warn_dew_lq_rep, &err_dew_lq_rep,
                         &s_dew_lq_td_c, &s_dew_lq_margin_c);
        s_dew_lq_valid = true;
    } else {
        s_active_warnings &= ~WARN_BIT_DEW_LASEQ;
        s_active_errors   &= ~ERR_BIT_DEW_LASEQ;
        warn_dew_lq_rep    = false;
        err_dew_lq_rep     = false;
        s_dew_lq_valid     = false;
    }
}

/* ============================================================================
 * DEW POINT — formula di Magnus (range valido: -40°C .. +60°C, RH 1..100%)
 *
 * γ(T, RH) = (a × T) / (b + T) + ln(RH / 100)
 * Td       = b × γ / (a - γ)
 *
 * Costanti Magnus: a = 17.625, b = 243.04 °C
 *
 * NOTA SULLA LOGICA DI CHECK:
 *   La condensazione avviene sulle SUPERFICI FREDDE, non nell'aria.
 *   Il parametro critico è la distanza tra la superficie più fredda del
 *   sistema (T_water_in, liquido di raffreddamento in ingresso) e il punto
 *   di rugiada dell'aria calcolato dalle misure SHT35.
 *
 *   margine = T_water_in - Td(T_amb, RH)
 *
 *   Se margine < dew_err_delta_c  → errore (condensazione imminente)
 *   Se margine < dew_warn_delta_c → warning
 *
 *   Il check è significativo SOLO quando il sistema di raffreddamento è
 *   in funzione (WC_STATE_RUNNING) e il sensore NTC water_in è valido.
 * ============================================================================ */
static float dew_point_c(float temp_c, float rh_pct)
{
    /* Clamp RH per evitare log(0) o valori negativi */
    if (rh_pct < 1.0f)   rh_pct = 1.0f;
    if (rh_pct > 100.0f) rh_pct = 100.0f;

    const float a = 17.625f;
    const float b = 243.04f;
    float gamma_val = (a * temp_c) / (b + temp_c) + logf(rh_pct / 100.0f);
    return (b * gamma_val) / (a - gamma_val);
}

/*
 * @brief  Controlla il margine tra T_water_in e il punto di condensa dell'aria.
 *         Condensa = rischio quando la superficie fredda (acqua) scende sotto Td.
 *
 * @param  amb_temp_c    Temperatura aria misurata da SHT35 [°C]
 * @param  rh_pct        Umidità relativa misurata da SHT35 [%RH, 0-100]
 * @param  surface_temp_c  Temperatura superficie fredda = T_water_in [°C]
 * @param  warn_bit      WARN_BIT_DEW_MMC o WARN_BIT_DEW_LASEQ
 * @param  err_bit       ERR_BIT_DEW_MMC o ERR_BIT_DEW_LASEQ
 * @param  warn_rep      Puntatore al flag "warning già segnalato"
 * @param  err_rep       Puntatore al flag "error già segnalato"
 * @param  out_td_c      Puntatore di uscita (nullable): Td calcolato [°C],
 *                        per cache diagnostica (TaskMonitor_GetDewMMC/LaseQ).
 * @param  out_margin_c  Puntatore di uscita (nullable): margine
 *                        surface_temp_c - Td [°C], stesso uso di out_td_c.
 *
 * @note   Chiamare solo quando WaterCooling è RUNNING e NTC_SENSOR_WATER_IN è valido.
 */
static void check_dew_margin(float amb_temp_c, float rh_pct, float surface_temp_c,
                              uint32_t warn_bit, uint32_t err_bit,
                              bool *warn_rep, bool *err_rep,
                              float *out_td_c, float *out_margin_c)
{
    float td     = dew_point_c(amb_temp_c, rh_pct);
    float margin = surface_temp_c - td;  /* >0: superficie sopra Td (ok)
                                            <0: superficie sotto Td (condensa!) */

    if (out_td_c     != NULL) { *out_td_c     = td;     }
    if (out_margin_c != NULL) { *out_margin_c = margin; }

    bool in_err  = (margin < (float)g_config.dew_err_delta_c);
    bool in_warn = !in_err && (margin < (float)g_config.dew_warn_delta_c);

    if (in_err) {
        /* Error: condensazione imminente entro delta_err.
         * Usa SYS_TEMP_ERROR_EVENT → SYS_ERROR (recuperabile via CERR, non
         * SYS_FAULT — vedi state_machine[] in fsm.c). Solo da SYS_ON in poi;
         * prima si degrada a warning. */
        s_active_warnings |= warn_bit;
        s_active_errors   |= err_bit;

        if (!(*err_rep)) {
            if (g_config.error_mask & err_bit) {
                SysState_t st = FSM_GetState();
                if (st >= SYS_ON) {
                    Queue_PostEvent(SYS_TEMP_ERROR_EVENT);
                } else {
                    /* Prima di ON: degrada a warning */
                    if (g_config.warning_mask & warn_bit) {
                        Queue_PostEvent(SYS_HUMIDITY_WARN_EVENT);
                    }
                }
            }
            *err_rep  = true;
            *warn_rep = true;
        }
    } else if (in_warn) {
        /* Warning: condensazione vicina entro delta_warn */
        s_active_warnings |= warn_bit;
        s_active_errors   &= ~err_bit;

        if (!(*warn_rep)) {
            if (g_config.warning_mask & warn_bit) {
                Queue_PostEvent(SYS_HUMIDITY_WARN_EVENT);
            }
            *warn_rep = true;
        }
        *err_rep = false;
    } else {
        /* Margine sufficiente: reset */
        s_active_warnings &= ~warn_bit;
        s_active_errors   &= ~err_bit;
        *warn_rep = false;
        *err_rep  = false;
    }
}

static void Monitor_CheckHumidity(void)
{
    s_sht35_cycle++;

    if (s_sht35_cycle < SHT35_TRIGGER_CYCLES) {
        return;
    }
    s_sht35_cycle = 0;

    if (s_sht35_triggered) {
        SHT35_Read();
        s_sht35_triggered = false;

        int8_t hum = SHT35_GetHumidityPct();

        /* --- Umidità assoluta --- */
        if (hum >= 0 && (uint8_t)hum > g_config.humidity_max_warn_pct) {
            s_active_warnings |= WARN_BIT_HUMIDITY_MMC;
            if (!warn_hum_mmc_rep) {
                POST_WARNING_HUM(WARN_BIT_HUMIDITY_MMC);
                warn_hum_mmc_rep = true;
            }
        } else {
            s_active_warnings &= ~WARN_BIT_HUMIDITY_MMC;
            warn_hum_mmc_rep = false;
        }

        /* --- Dew point MMC ---
         *
         * Calcola Td dall'aria (SHT35 MMC), confronta con T_water_in (NTC).
         * Guard: cooling deve essere RUNNING e sensore NTC water_in valido.
         */
        SHT35_Data_t sht = SHT35_GetData();
        /* Guard: aggiorna solo se il dato è valido.
         * Senza guard, SHT35 non connesso → temperature_cdeg = INT16_MIN
         * (-32768 centideg = -327°C) → dew point calcolato falso-sicuro
         * (Td molto negativo → margine enorme → nessun allarme). */
        if (sht.valid) {
            s_mmc_temp_c = (float)sht.temperature_cdeg / 100.0f;  /* centideg → °C */
        }
        s_mmc_humid_pct = (hum >= 0) ? (float)hum : s_mmc_humid_pct;

        /*
         * MB_TEMP (NUOVO 2026-07-22): soglie di temperatura sulla lettura
         * SHT35-MMC, finora usata SOLO per umidità/dew point sopra, MAI
         * confrontata con soglie di temperatura proprie. Va tenuta distinta
         * da NTC_SENSOR_AMBIENT (quello è l'NTC fisico sulla scocca, aria
         * esterna al rack): MB_TEMP misura un sensore su PCB, fisiologicamente
         * più caldo — vedi commento su temp_mb_* in config.h. Stesso schema
         * di check_dew_margin() sopra: usa l'ultimo valore noto di
         * s_mmc_temp_c anche se questo particolare ciclo non ha una lettura
         * fresca (sht.valid==false), coerente con come viene già trattato
         * per il calcolo del dew point.
         */
        {
            int16_t t10 = (int16_t)(s_mmc_temp_c * 10.0f);
            int16_t min_err_c10  = (int16_t)(g_config.temp_mb_min_err  * 10);
            int16_t min_warn_c10 = (int16_t)(g_config.temp_mb_min_warn * 10);
            int16_t max_warn_c10 = (int16_t)(g_config.temp_mb_max_warn * 10);
            int16_t max_err_c10  = (int16_t)(g_config.temp_mb_max_err  * 10);

            bool mb_fault = (t10 <= min_err_c10) || (t10 >= max_err_c10);
            bool mb_warn  = !mb_fault && ((t10 <= min_warn_c10) || (t10 >= max_warn_c10));

            if (mb_fault) {
                s_active_warnings |= WARN_BIT_TEMP_MB;
                if (!fault_mb_rep) {
                    SysState_t state = FSM_GetState();
                    if (state >= SYS_ON) {
                        if (g_config.error_mask & ERR_BIT_TEMP) {
                            Queue_PostEvent(SYS_TEMP_ERROR_EVENT);
                        }
                        s_active_errors |= ERR_BIT_TEMP;
                    } else if (g_config.warning_mask & WARN_BIT_TEMP_MB) {
                        Queue_PostEvent(SYS_TEMP_WARN_EVENT);
                    }
                    fault_mb_rep = true;
                }
            } else if (mb_warn) {
                s_active_warnings |= WARN_BIT_TEMP_MB;
                if (!warn_mb_rep) {
                    POST_WARNING_TEMP(WARN_BIT_TEMP_MB);
                    warn_mb_rep = true;
                }
            } else {
                s_active_warnings &= ~WARN_BIT_TEMP_MB;
                warn_mb_rep  = false;
                fault_mb_rep = false;
                s_active_errors &= ~ERR_BIT_TEMP;
            }
        }

        if (WaterCooling_GetState() == WC_STATE_RUNNING &&
            s_ntc_valid[NTC_SENSOR_WATER_IN])
        {
            float t_water_in = (float)s_ntc_temp_c10[NTC_SENSOR_WATER_IN] / 10.0f;

            check_dew_margin(s_mmc_temp_c, s_mmc_humid_pct, t_water_in,
                             WARN_BIT_DEW_MMC, ERR_BIT_DEW_MMC,
                             &warn_dew_mmc_rep, &err_dew_mmc_rep,
                             &s_dew_mmc_td_c, &s_dew_mmc_margin_c);
            s_dew_mmc_valid = true;
        } else {
            /* Cooling fermo o NTC non valido: reset flag, nessun allarme */
            s_active_warnings &= ~WARN_BIT_DEW_MMC;
            s_active_errors   &= ~ERR_BIT_DEW_MMC;
            warn_dew_mmc_rep   = false;
            err_dew_mmc_rep    = false;
            s_dew_mmc_valid    = false;
        }
    } else {
        SHT35_Trigger();
        s_sht35_triggered = true;
    }
}

/* ========================================================================== */
/* --- API PUBBLICA --- */
/* ========================================================================== */

void TaskMonitor_ResetFlowInhibit(void)
{
    flow_inhibit_ms = FLOW_INHIBIT_MS;
    flow_fault_rep  = false;
    warn_flow_rep   = false;
}

void TaskMonitor_ResetPSUInhibit(void)
{
    /*
     * Carica il timer di inibizione dal valore configurabile.
     * Chiamare da action_enter_on() subito dopo PSU_Enable() / contactor close.
     * Il monitor ignorerà gli allarmi PSU per psu_dc_ok_delay_ms millisecondi.
     */
    psu_inhibit_ms = (uint32_t)g_config.psu_dc_ok_delay_ms;
    psu_reported   = false;

    /*
     * Reset dei flag "già segnalato" per temperatura e LaseQ.
     *
     * check_ntc_sensor() e Monitor_CheckLaseQTelemetry() impostano i flag a
     * true anche quando stato < SYS_ON (dopo aver postato un warning), il che
     * impedisce di rilevare la condizione come fault alla prima verifica in ON.
     *
     * Resettando qui, il ciclo di monitor successivo a SYS_ON trova i flag
     * liberi e posta correttamente SYS_TEMP_ERROR_EVENT se le condizioni
     * anomale sono ancora presenti.
     */
    for (uint8_t i = 0; i < NTC_NUM_SENSORS; i++) {
        fault_temp_rep[i] = false;
    }
    warn_lq_driver_rep  = false;
    warn_lq_ambient_rep = false;
}

void TaskMonitor_ResetFaultLatches(void)
{
    flood1_reported      = false;
    flood2_reported      = false;
    flood_shutdown_armed = false;
    flood_shutdown_done  = false;
    flood_shutdown_ms    = 0U;
    psu_reported         = false;
}

uint32_t TaskMonitor_GetWarnings(void)
{
    return s_active_warnings;
}

uint32_t TaskMonitor_GetErrors(void)
{
    /* Vedi banner su s_latched_errors sopra: NON s_active_errors (live) —
     * questo e' il bitmask "congelato" per tutta la durata di SYS_ERROR. */
    return s_latched_errors;
}

/*
 * @brief  Azzera il bitmask errori congelato. Chiamare SOLO dal percorso di
 *         successo di action_clear_error() (fsm.c), mai da un CERR respinto
 *         (es. LaseQ non ancora confermato) — vedi banner su
 *         s_latched_errors sopra.
 *
 *         Azzera anche ERR_BIT_SAB_TIMEOUT: e' l'UNICA causa di SYS_ERROR
 *         senza un check continuo (nessun pin "timeout in corso" da
 *         interrogare, vedi seed one-shot in action_enter_error()) — senza
 *         questo, resterebbe "incollato" in s_active_errors e verrebbe
 *         rilatchato fin dal primo ciclo del PROSSIMO episodio di errore,
 *         anche se non più pertinente. Tutte le altre cause (SAB INTLCK/TEST
 *         A/B, KEY A/B, LID1/2, FLOW, TEMP, DEW, LASEQ_INTERNAL) hanno un
 *         check continuo che le mantiene sempre coerenti con lo stato fisico
 *         reale — nessun azzeramento forzato necessario per quelle.
 */
void TaskMonitor_ClearLatchedErrors(void)
{
    s_error_latch_active = false;   /* PRIMA di azzerare: chiude subito il gate letto da TaskMonitor_Run() */
    s_latched_errors = 0U;
    s_active_errors &= ~ERR_BIT_SAB_TIMEOUT;
}

/*
 * @brief  Apre il gate di accumulo di s_latched_errors. Chiamare SOLO da
 *         action_enter_error() (fsm.c), a ogni ingresso reale in SYS_ERROR
 *         — vedi banner su s_latched_errors/s_error_latch_active sopra.
 */
void TaskMonitor_BeginErrorLatch(void)
{
    s_error_latch_active = true;
}

/*
 * @brief  Inserisce direttamente un bit nel bitmask CONGELATO (s_latched_errors),
 *         bypassando s_active_errors. Chiamare SOLO da action_enter_error()
 *         (fsm.c) come seed one-shot per cause la cui verifica continua
 *         (Monitor_Check*()) smette di essere attendibile PROPRIO a causa
 *         dell'ingresso in errore.
 *
 *         BUGFIX (2026-07-31, "GET ERR non mostra più FLOW dopo un errore di
 *         flusso, solo interlock"): a differenza di SAB_TIMEOUT (che non ha
 *         MAI avuto un check continuo, seedato con TaskMonitor_SetErrorBit()
 *         + gate FSM_GetState()/s_error_latch_active esistente), FLOW ha un
 *         check continuo (Monitor_CheckFlow()) che perde però la sua
 *         attendibilità nell'ISTANTE stesso in cui si entra in errore: se
 *         ev == SYS_FLOW_ERROR_EVENT, action_enter_error() chiama
 *         WaterCooling_Emergency() (WC_STATE_ERROR), e Monitor_CheckFlow()
 *         smette da quel momento di rivalutare il flusso (early return su
 *         "non RUNNING", che ora azzera correttamente s_active_errors —
 *         vedi bugfix precedente) — quindi il bit ERR_BIT_FLOW impostato in
 *         s_active_errors un istante prima (dal check che ha rilevato il
 *         guasto originale) viene ripulito dallo STESSO ciclo di
 *         TaskMonitor_Run() che dovrebbe latchare l'errore, PRIMA che il
 *         passo di OR-in arrivi a leggerlo: seedare s_active_errors (come
 *         per SAB_TIMEOUT) non basterebbe, verrebbe comunque cancellato in
 *         tempo. Scrivendo direttamente qui in s_latched_errors si bypassa
 *         del tutto questa corsa.
 */
void TaskMonitor_SeedLatchedError(uint32_t bit_mask)
{
    s_latched_errors |= bit_mask;
}

uint32_t TaskMonitor_GetFaults(void)
{
    return s_active_faults;
}

/*
 * Dettaglio sorgente per SYS_TEMP_ERROR_EVENT (dal 2026-07-15) — quell'evento
 * è condiviso da 11 sorgenti indipendenti (8 sensori NTC via check_ntc_sensor()
 * + MB_TEMP (SHT35-MMC) + 2 sorgenti LaseQ, driver/ambient), quindi di per sé non basta a dire
 * QUALE sensore ha causato il fault (stessa ambiguità già notata per
 * SYS_KEY_REMOVED_EVENT/FLOOD1-2/PSU1-2). Usate da fsm.c per costruire il
 * dettaglio mostrato da "GET ALARM" (vedi event_detail_name()).
 */
uint16_t TaskMonitor_GetTempFaultSensors(void)
{
    uint16_t mask = 0U;
    for (uint8_t i = 0U; i < NTC_NUM_SENSORS; i++) {
        if (fault_temp_rep[i]) {
            mask |= (uint16_t)(1U << i);
        }
    }
    return mask;
}

bool TaskMonitor_IsLaseQDriverTempFault(void)
{
    return lq_driver_temp_fault;
}

bool TaskMonitor_IsLaseQAmbientTempFault(void)
{
    return lq_ambient_temp_fault;
}

bool TaskMonitor_IsMBTempFault(void)
{
    return fault_mb_rep;
}

void TaskMonitor_SetFaultBit(uint32_t fault_bit, bool active)
{
    if (active) {
        s_active_faults |= fault_bit;
    } else {
        s_active_faults &= ~fault_bit;
    }
}

void TaskMonitor_SetErrorBit(uint32_t error_bit, bool active)
{
    if (active) {
        s_active_errors |= error_bit;
    } else {
        s_active_errors &= ~error_bit;
    }
}

void TaskMonitor_SetLaseQErrorCode(uint8_t error_code)
{
    s_lq_error_code = error_code;
}

uint8_t TaskMonitor_GetLaseQErrorCode(void)
{
    return s_lq_error_code;
}

void TaskMonitor_GetLaseQTemps(uint8_t driver_temp_c[4],
                                int8_t  *ambient_temp_c,
                                uint8_t *humidity_pct)
{
    if (driver_temp_c != NULL) {
        for (uint8_t i = 0U; i < 4U; i++) {
            driver_temp_c[i] = s_lq_driver_temp_c[i];
        }
    }
    if (ambient_temp_c != NULL) { *ambient_temp_c = s_lq_ambient_temp_c; }
    if (humidity_pct   != NULL) { *humidity_pct   = s_lq_humidity_pct;  }
}

bool TaskMonitor_GetDewMMC(float *td_c, float *margin_c)
{
    if (td_c     != NULL) { *td_c     = s_dew_mmc_td_c;     }
    if (margin_c != NULL) { *margin_c = s_dew_mmc_margin_c; }
    return s_dew_mmc_valid;
}

bool TaskMonitor_GetDewLaseQ(float *td_c, float *margin_c)
{
    if (td_c     != NULL) { *td_c     = s_dew_lq_td_c;     }
    if (margin_c != NULL) { *margin_c = s_dew_lq_margin_c; }
    return s_dew_lq_valid;
}

void TaskMonitor_UpdateLaseQTelemetry(const uint8_t driver_temp_c[4],
                                      int8_t  ambient_temp_c,
                                      uint8_t humidity_pct)
{
    /*
     * Chiamata da task_comms nel contesto del suo task.
     * Le variabili scalari sono volatile uint8_t/int8_t: scrittura atomica
     * su ARM, thread-safe. L'array s_lq_driver_temp_c[4] NON è atomico nel
     * suo insieme (4 scritture separate, un solo scrittore: task_comms) —
     * nel caso limite Monitor_CheckLaseQTelemetry() (task_monitor, unico
     * lettore) potrebbe leggere un mix di valori vecchi/nuovi durante
     * l'aggiornamento, tollerabile: si autocorregge al ciclo successivo
     * (20ms), nessuna sezione critica necessaria per pura telemetria.
     */
    for (uint8_t i = 0U; i < 4U; i++) {
        s_lq_driver_temp_c[i] = driver_temp_c[i];
    }
    s_lq_ambient_temp_c = ambient_temp_c;
    s_lq_humidity_pct   = humidity_pct;
}

bool TaskMonitor_GetNTCTempC10(NTC_SensorId_t id, int16_t *out_c10)
{
    if (id >= NTC_NUM_SENSORS || out_c10 == NULL) return false;
    if (!s_ntc_valid[id]) return false;
    *out_c10 = s_ntc_temp_c10[id];
    return true;
}

uint16_t TaskMonitor_GetNTCActiveMask(void)
{
    uint16_t mask = 0U;
    for (uint8_t ch = 0U; ch < AD7490_NUM_CHANNELS; ch++) {
        if (g_config.ntc_ch_map[ch] != 0xFFU) {
            mask |= (uint16_t)(1U << ch);
        }
    }
    return mask;
}

/* ========================================================================== */
/* --- TASK --- */
/* ========================================================================== */

void TaskMonitor_Run(void *arg)
{
    (void)arg;

    /*
     * Init hardware (SPI, I2C, ADC, UART, SD) — deve stare qui, DOPO
     * osKernelStart(), perché i timeout HAL (TIM1) non sono affidabili
     * prima che il scheduler sia avviato.
     * TaskMonitor ha la priorità più alta (Normal5): completa Sys_HwInit
     * prima che gli altri task ricevano il loro primo time-slice.
     */
    extern void Sys_HwInit(void);
    Sys_HwInit();

    /*
     * Semafori binari per la sincronizzazione IT di AD7490/SHT35 (vedi nota
     * sopra) e registrazione dei driver tramite le rispettive Init(): deve
     * avvenire DOPO Sys_HwInit() (richiede osKernelStart() per i timeout HAL)
     * e PRIMA del primo AD7490_ScanAll()/SHT35_Trigger() nel loop sotto.
     */
    s_ad7490_sem = osSemaphoreNew(1U, 0U, NULL);
    s_sht35_sem  = osSemaphoreNew(1U, 0U, NULL);
    AD7490_Init(&hspi5, ad7490_wait, ad7490_signal);
    SHT35_Init(&hi2c2, sht35_wait, sht35_signal);

    TickType_t xLastWakeTime = xTaskGetTickCount();

    for (;;)
    {
        Watchdog_Heartbeat(WDG_TASK_MONITOR);

        /* Aggiornamento hardware: eFuse + scan NTC */
        EFuse_UpdateAll();
        AD7490_ScanAll(s_ntc_raw);

        /* Controlli sicurezza in ordine di priorità */
        Monitor_CheckFlood();
        Monitor_CheckFlow();
        Monitor_CheckPSU();
        Monitor_CheckTemperatures();
        Monitor_CheckLaseQTelemetry();
        Monitor_CheckHumidity();
        Monitor_CheckSAB();
        Monitor_CheckKey();
        Monitor_CheckLid();

        /*
         * Latch errori (2026-07-31, richiesta esplicita): mentre il gate
         * s_error_latch_active è aperto, arricchisce SOLO in OR il bitmask
         * congelato con quanto appena calcolato dai check sopra — mai una
         * sottrazione qui, vedi banner su s_latched_errors. Gate NON basato
         * su FSM_GetState() (vedi bugfix "GET ERR sempre FLOW=1 dopo CERR"
         * nel banner): aperto da TaskMonitor_BeginErrorLatch() (chiamata da
         * action_enter_error()), chiuso da TaskMonitor_ClearLatchedErrors()
         * (chiamata da action_clear_error() sul percorso di successo) —
         * unico altro punto che tocca s_latched_errors.
         */
        if (s_error_latch_active) {
            s_latched_errors |= s_active_errors;
        }

        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(MONITOR_PERIOD_MS));
    }
}
