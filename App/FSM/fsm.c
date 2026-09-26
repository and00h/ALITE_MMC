/*
 * fsm.c
 *
 * Implementazione della macchina a stati di sistema.
 *
 * COME AGGIUNGERE UNO STATO:
 *   1. Aggiungere il valore in SysState_t (fsm.h)
 *   2. Aggiungere la riga nella tabella state_machine[][]
 *   3. Implementare le action necessarie in questo file
 *
 * COME AGGIUNGERE UN EVENTO:
 *   1. Aggiungere il valore in SysEvent_t (fsm_events.h)
 *   2. Popolare le celle della tabella per gli stati che devono reagirvi
 *
 * COME AGGIUNGERE UN'ACTION:
 *   1. Dichiarare la funzione static in questo file
 *   2. Aggiungere il prototipo nella sezione "Prototipi action" qui sotto
 *   3. Implementarla nella sezione "Implementazione action"
 */

#include "fsm.h"
#include "WaterCooling.h"
#include "LED.h"
#include "PSU.h"
#include "Contactors.h"
#include "LaseQ.h"
#include "EXT_interface.h"
#include "SAB.h"
#include "Key.h"
#include "Lid.h"
#include "task_monitor.h"
#include "config.h"
#include "BoardCtrl.h"

#include <stdio.h>     /* snprintf: FSM_FormatErrorCode */
#include "cmsis_os.h"  /* osDelay */
#include "queues.h"    /* Queue_PostEvent: usato nel polling SAB in action_enter_enabled */
#include "sys_log.h"   /* SysLog_StateChange, SysLog_Event */
#include "rs485_cmd.h" /* Rs485Cmd_NotifyFault */
#include "QCW.h"       /* QCW_Start/Stop/SetParams: modalità CW/QCW in SYS_EMISSION */

/* ========================================================================== */
/* --- STATO INTERNO --- */
/* ========================================================================== */

static SysState_t s_current_state = SYS_INIT;

/*
 * s_last_fault_event: ultimo evento che ha causato SYS_FAULT (non
 * recuperabile — impostato SOLO in action_fault()). MAI azzerato altrove —
 * resta leggibile anche dopo un reset di stato, finché non se ne verifica
 * uno nuovo, per permettere una diagnosi a posteriori "qual è stato l'ultimo
 * fault critico" via RS485 GET FAULT.
 * SYS_NUM_EVENTS = sentinel "nessun fault dall'avvio".
 *
 * s_last_error_event: stesso schema ma per SYS_ERROR (recuperabile via
 * CERR — impostato SOLO in action_enter_error()), letto via RS485 GET ALARM.
 *
 * BUGFIX (2026-07-14): prima delle due esisteva UNA sola variabile
 * (s_last_fault_event) aggiornata da ENTRAMBE le funzioni: "GET FAULT"
 * poteva quindi rispondere con una causa recuperabile come "KEY" o "SAB_*",
 * fuorviante dato il nome del comando. Separate in due variabili + due
 * comandi RS485 distinti (GET FAULT / GET ALARM), vedi
 * FSM_GetLastFaultEvent()/Name() e le nuove FSM_GetLastErrorEvent()/Name().
 */
static SysEvent_t s_last_fault_event = SYS_NUM_EVENTS;
static SysEvent_t s_last_error_event = SYS_NUM_EVENTS;

/*
 * Dettaglio della sorgente specifica per gli eventi "generici" (dal
 * 2026-07-15) — vedi event_detail_name() sotto per la spiegazione completa.
 * Stringa vuota ("") se l'evento non è ambiguo o se non è stato possibile
 * ricostruire una sorgente specifica: in quel caso FSM_GetLastFaultName()/
 * FSM_GetLastErrorName() ricadono su fault_event_name() come prima.
 */
/*
 * 80 byte: copre il caso peggiore, tutte le 8 sorgenti TEMP simultaneamente
 * ("WATER_IN|WATER_OUT|SPLICE|DIODE1|DIODE2|AMBIENT|LQ_DRIVER|LQ_AMBIENT",
 * 68 caratteri + terminatore) — vedi case SYS_TEMP_ERROR_EVENT sotto.
 */
/*
 * 128 byte (era 80 fino al 2026-07-22): con l'aggiunta di PSU_TEMP,
 * PWR_EL_TEMP e MB_TEMP alla lista di sorgenti in event_detail_name(),
 * il worst-case teorico (tutte le sorgenti attive insieme) supera 80 byte.
 */
static char s_last_fault_detail[128] = "";
static char s_last_error_detail[128] = "";

/* Stringa leggibile per i fault event (usata in action_fault per il log) */
static const char *fault_event_name(SysEvent_t ev)
{
    switch (ev) {
        case SYS_FAULT_EVENT:               return "FAULT";
        case SYS_FLOOD_FAULT_EVENT:         return "FLOOD";
        case SYS_FLOW_ERROR_EVENT:          return "FLOW";
        case SYS_TEMP_ERROR_EVENT:          return "TEMP";
        case SYS_PSU_FAULT_EVENT:           return "PSU";
        case SYS_LASEQ_FAULT_EVENT:         return "LASEQ_COMMS";
        case SYS_AMC_FAULT_EVENT:           return "AMC_COMMS";
        case SYS_COM_FAULT_EVENT:           return "COM_COMMS";
        case SYS_CHECK_FAIL_EVENT:          return "INIT_CHECK";
        case SYS_KEY_REMOVED_EVENT:         return "KEY";
        case SYS_LID_OPEN_EVENT:            return "LID";
        case SYS_SAB_TIMEOUT_EVENT:         return "SAB_TIMEOUT";
        case SYS_SAB_INTERLOCK_OPEN_EVENT:  return "SAB_INTLCK";
        case SYS_SAB_TEST_FAIL_EVENT:       return "SAB_TEST";
        case SYS_ERROR_EVENT:               return "ERROR";
        case SYS_LASEQ_ERROR_EVENT:         return "LASEQ_ERR";
        default:                             return "UNKNOWN";
    }
}

/*
 * @brief  Alcuni SysEvent_t sono "generici": raggruppano più sorgenti
 *         fisiche indipendenti sotto lo stesso nome (es. SYS_KEY_REMOVED_EVENT
 *         → "KEY", che non dice se è la chiave A, la B, o entrambe).
 *         Aggiungere un SysEvent_t per ogni singola sorgente richiederebbe
 *         popolare una nuova colonna in OGNI riga di state_machine[][] per
 *         ogni stato interessato (vedi banner "COME AGGIUNGERE UN EVENTO" in
 *         testa al file) solo per un dettaglio diagnostico, senza alcuna
 *         differenza di comportamento FSM tra le sotto-sorgenti — stesso
 *         principio già usato per SYS_LASEQ_FAULT_EVENT (causa specifica
 *         via "GET LQERR", non un evento dedicato).
 *
 *         Questa funzione ricostruisce quindi il dettaglio interrogando lo
 *         stato LIVE dei singoli sottosistemi (Key_GetKeyStatus(),
 *         TaskMonitor_GetFaults(), PSUGetStatus(),
 *         TaskMonitor_GetTempFaultSensors()/IsLaseQ*TempFault()), chiamata
 *         SINCRONAMENTE da action_fault()/action_enter_error() nello stesso
 *         istante in cui l'evento viene processato — quindi affidabile,
 *         niente race con un aggiornamento successivo del sensore.
 *
 *         Se più sorgenti sono contemporaneamente in fault (es. entrambe le
 *         chiavi rimosse), tutte compaiono nel risultato separate da "|".
 *         Scrive "" in buf se l'evento non è tra quelli ambigui gestiti qui,
 *         o se — caso limite, es. condizione già rientrata tra il post
 *         dell'evento e il processing — nessuna sorgente risulta più
 *         attiva: il chiamante ricade allora su fault_event_name(ev).
 */
static void event_detail_name(SysEvent_t ev, char *buf, size_t sz)
{
    buf[0] = '\0';

    switch (ev) {

    case SYS_KEY_REMOVED_EVENT: {
        /* Solo i contatti NON mascherati contano — stesso criterio di
         * Key_InterlockSatisfied() (Key.c). Un contatto mascherato (non
         * montato) non può essere la causa dell'errore. */
        bool a_req  = (g_config.error_mask & ERR_BIT_KEY_A) != 0U;
        bool b_req  = (g_config.error_mask & ERR_BIT_KEY_B) != 0U;
        bool a_bad  = a_req && !Key_GetKeyStatus(KEY_A);
        bool b_bad  = b_req && !Key_GetKeyStatus(KEY_B);

        if (a_bad && b_bad)     { snprintf(buf, sz, "KEY_A|KEY_B"); }
        else if (a_bad)         { snprintf(buf, sz, "KEY_A"); }
        else if (b_bad)         { snprintf(buf, sz, "KEY_B"); }
        break;
    }

    case SYS_LID_OPEN_EVENT: {
        /* Stesso schema di SYS_KEY_REMOVED_EVENT sopra, ma per i sensori
         * coperchio — vedi Lid_InterlockSatisfied() (Drivers/Lid/Lid.c). */
        bool l1_req = (g_config.error_mask & ERR_BIT_LID1) != 0U;
        bool l2_req = (g_config.error_mask & ERR_BIT_LID2) != 0U;
        bool l1_bad = l1_req && Lid_GetStatus(LID_1);
        bool l2_bad = l2_req && Lid_GetStatus(LID_2);

        if (l1_bad && l2_bad)   { snprintf(buf, sz, "LID1|LID2"); }
        else if (l1_bad)        { snprintf(buf, sz, "LID1"); }
        else if (l2_bad)        { snprintf(buf, sz, "LID2"); }
        break;
    }

    case SYS_SAB_INTERLOCK_OPEN_EVENT: {
        /* Stesso schema di SYS_KEY_REMOVED_EVENT/SYS_LID_OPEN_EVENT sopra,
         * ma per i due canali ridondanti SAB — vedi banner
         * ERR_BIT_SAB_INTLCK_A/B (task_monitor.h) e SAB_GetInterlockStatus()
         * (SAB.h/SAB.c). NOTA: qui "mascherato" sopprime solo il dettaglio/
         * la notifica, il modulo SAB è comunque andato in FAULT su questo
         * canale indipendentemente da error_mask. */
        bool a_req, b_req, a_open, b_open;
        a_req = (g_config.error_mask & ERR_BIT_SAB_INTLCK_A) != 0U;
        b_req = (g_config.error_mask & ERR_BIT_SAB_INTLCK_B) != 0U;
        SAB_GetInterlockStatus(&a_open, &b_open);
        a_open = a_req && a_open;
        b_open = b_req && b_open;

        if (a_open && b_open)  { snprintf(buf, sz, "SAB_INTLCK_A|SAB_INTLCK_B"); }
        else if (a_open)       { snprintf(buf, sz, "SAB_INTLCK_A"); }
        else if (b_open)       { snprintf(buf, sz, "SAB_INTLCK_B"); }
        break;
    }

    case SYS_SAB_TIMEOUT_EVENT: {
        /*
         * BUGFIX (Luca, 2026-07-16): SYS_SAB_TIMEOUT_EVENT (SAB_STATE_
         * ENABLING che scade senza che gli interlock si chiudano, vedi
         * SAB_Process()/SAB_STATE_ENABLING in SAB.c) restava un evento
         * "cieco" — nessun dettaglio canale, a differenza di
         * SYS_SAB_INTERLOCK_OPEN_EVENT sopra (stesso identico dato fisico,
         * SAB_GetInterlockStatus(), ma raccolto solo per l'apertura da
         * ARMED, non per il mancato arrivo a ARMED). Concettualmente lo
         * scatenante è lo stesso — l'interlock è aperto — quindi va
         * riportato allo stesso modo, pur mantenendo "SAB_TIMEOUT" come
         * categoria (distingue "mai arrivato ad armarsi" da "apertura dopo
         * l'armo", utile in diagnostica). Stesso criterio di masking di
         * SYS_SAB_INTERLOCK_OPEN_EVENT sopra.
         */
        bool a_req, b_req, a_open, b_open;
        a_req = (g_config.error_mask & ERR_BIT_SAB_INTLCK_A) != 0U;
        b_req = (g_config.error_mask & ERR_BIT_SAB_INTLCK_B) != 0U;
        SAB_GetInterlockStatus(&a_open, &b_open);
        a_open = a_req && a_open;
        b_open = b_req && b_open;

        if (a_open && b_open)  { snprintf(buf, sz, "SAB_TIMEOUT(SAB_INTLCK_A|SAB_INTLCK_B)"); }
        else if (a_open)       { snprintf(buf, sz, "SAB_TIMEOUT(SAB_INTLCK_A)"); }
        else if (b_open)       { snprintf(buf, sz, "SAB_TIMEOUT(SAB_INTLCK_B)"); }
        /* Nessun canale interlock risulta aperto al momento del processing
         * (raro: rientrato tra evento e processing, oppure causa era
         * SAB_EVT_TEST_FAIL non intercettato) — buf resta "", si ricade sul
         * nome generico "SAB_TIMEOUT" (fault_event_name()). */
        break;
    }

    case SYS_SAB_TEST_FAIL_EVENT: {
        /* Stesso schema del caso SAB_INTERLOCK sopra, ma per i pin TEST. */
        bool a_req, b_req, a_fault, b_fault;
        a_req = (g_config.error_mask & ERR_BIT_SAB_TEST_A) != 0U;
        b_req = (g_config.error_mask & ERR_BIT_SAB_TEST_B) != 0U;
        SAB_GetTestStatus(&a_fault, &b_fault);
        a_fault = a_req && a_fault;
        b_fault = b_req && b_fault;

        if (a_fault && b_fault) { snprintf(buf, sz, "SAB_TEST_A|SAB_TEST_B"); }
        else if (a_fault)       { snprintf(buf, sz, "SAB_TEST_A"); }
        else if (b_fault)       { snprintf(buf, sz, "SAB_TEST_B"); }
        break;
    }

    case SYS_FLOOD_FAULT_EVENT: {
        uint32_t f  = TaskMonitor_GetFaults();
        bool f1_bad = (f & FAULT_BIT_FLOOD1) != 0U;
        bool f2_bad = (f & FAULT_BIT_FLOOD2) != 0U;

        if (f1_bad && f2_bad)   { snprintf(buf, sz, "FLOOD1|FLOOD2"); }
        else if (f1_bad)        { snprintf(buf, sz, "FLOOD1"); }
        else if (f2_bad)        { snprintf(buf, sz, "FLOOD2"); }
        break;
    }

    case SYS_PSU_FAULT_EVENT: {
        PSU_t psu1 = PSUGetStatus(PSU1);
        PSU_t psu2 = PSUGetStatus(PSU2);
        /* Stessa condizione di fault usata da Monitor_CheckPSU()
         * (task_monitor.c): alarm oppure DC_OK mancante ad alimentatore
         * acceso. */
        bool p1_bad = psu1.alarm || (psu1.status && !psu1.dc_ok);
        bool p2_bad = psu2.alarm || (psu2.status && !psu2.dc_ok);

        if (p1_bad && p2_bad)   { snprintf(buf, sz, "PSU1|PSU2"); }
        else if (p1_bad)        { snprintf(buf, sz, "PSU1"); }
        else if (p2_bad)        { snprintf(buf, sz, "PSU2"); }
        break;
    }

    case SYS_TEMP_ERROR_EVENT: {
        /*
         * Fino a 11 sorgenti indipendenti condividono questo evento: 8
         * sensori NTC locali (bitmask, vedi TaskMonitor_GetTempFaultSensors(),
         * include PSU_TEMP/PWR_EL_TEMP dal 2026-07-22) + MB_TEMP (SHT35-MMC,
         * dal 2026-07-22, non è un NTC quindi non è nella bitmask, verificato
         * a parte come LQ_DRIVER/LQ_AMBIENT sotto) + 2 sorgenti LaseQ (driver
         * a 4 elementi, ambiente SHT35 interno). Nomi limitati ai soli
         * sensori realmente verificati da check_ntc_sensor() in
         * Monitor_CheckTemperatures() — gli id generici rimasti (NTC8..
         * NTC13/NTC16) non sono mai chiamati con fault_temp_rep, quindi non
         * possono comparire qui.
         */
        uint16_t mask = TaskMonitor_GetTempFaultSensors();
        char tmp[128];
        size_t used = 0U;
        tmp[0] = '\0';

        static const struct { NTC_SensorId_t id; const char *name; } s_ntc_names[] = {
            { NTC_SENSOR_WATER_IN,     "WATER_IN"     },
            { NTC_SENSOR_WATER_OUT,    "WATER_OUT"    },
            { NTC_SENSOR_SPLICE,       "SPLICE"       },
            { NTC_SENSOR_DIODE1,       "DIODE1"       },
            { NTC_SENSOR_DIODE2,       "DIODE2"       },
            { NTC_SENSOR_AMBIENT,      "AMBIENT"      },
            { NTC_SENSOR_PSU_TEMP,     "PSU_TEMP"     },
            { NTC_SENSOR_PWR_EL_TEMP,  "PWR_EL_TEMP"  },
        };

        for (size_t i = 0U; i < (sizeof(s_ntc_names) / sizeof(s_ntc_names[0])); i++) {
            if ((mask & (1U << s_ntc_names[i].id)) != 0U) {
                int wn = snprintf(&tmp[used], sizeof(tmp) - used,
                                   "%s%s", (used != 0U) ? "|" : "", s_ntc_names[i].name);
                if (wn > 0 && (size_t)wn < (sizeof(tmp) - used)) { used += (size_t)wn; }
            }
        }
        if (TaskMonitor_IsMBTempFault()) {
            int wn = snprintf(&tmp[used], sizeof(tmp) - used,
                               "%sMB_TEMP", (used != 0U) ? "|" : "");
            if (wn > 0 && (size_t)wn < (sizeof(tmp) - used)) { used += (size_t)wn; }
        }
        if (TaskMonitor_IsLaseQDriverTempFault()) {
            int wn = snprintf(&tmp[used], sizeof(tmp) - used,
                               "%sLQ_DRIVER", (used != 0U) ? "|" : "");
            if (wn > 0 && (size_t)wn < (sizeof(tmp) - used)) { used += (size_t)wn; }
        }
        if (TaskMonitor_IsLaseQAmbientTempFault()) {
            int wn = snprintf(&tmp[used], sizeof(tmp) - used,
                               "%sLQ_AMBIENT", (used != 0U) ? "|" : "");
            if (wn > 0 && (size_t)wn < (sizeof(tmp) - used)) { used += (size_t)wn; }
        }

        if (used != 0U) { snprintf(buf, sz, "%s", tmp); }
        break;
    }

    default:
        break;
    }
}

/*
 * FAULT LATCH (dal 2026-07-14) — vedi banner "FAULT LATCH" in
 * task_monitor.h. Due helper usati da action_fault() (persiste il latch
 * all'ingresso in SYS_FAULT) e da FSM_Init() (rileva un latch già presente
 * da un boot precedente e forza INIT -> FAULT).
 */

/*
 * @brief  Mappa un singolo FAULT_BIT_* (task_monitor.h) sul SysEvent_t
 *         corrispondente — usata SOLO per ricostruire un nome leggibile
 *         (log SD, "GET FAULT") quando la FSM entra in SYS_FAULT direttamente
 *         da un latch persistito, senza che l'evento originale sia stato
 *         ripostato in coda. FAULT_BIT_FLOOD1/FLOOD2 condividono lo stesso
 *         evento generico (nessuna distinzione a livello di SysEvent_t,
 *         vedi task_monitor.c Monitor_CheckFlood()).
 * @param  bit  Un SOLO bit (isolato dal chiamante, non una bitmask).
 */
static SysEvent_t fault_bit_to_event(uint32_t bit)
{
    switch (bit) {
        case FAULT_BIT_LASEQ:  return SYS_LASEQ_FAULT_EVENT;
        case FAULT_BIT_AMC:    return SYS_AMC_FAULT_EVENT;
        case FAULT_BIT_COM:    return SYS_COM_FAULT_EVENT;
        case FAULT_BIT_FLOOD1: return SYS_FLOOD_FAULT_EVENT;
        case FAULT_BIT_FLOOD2: return SYS_FLOOD_FAULT_EVENT;
        case FAULT_BIT_PSU:    return SYS_PSU_FAULT_EVENT;
        default:                return SYS_FAULT_EVENT;
    }
}

/*
 * @brief  Se i fault ATTUALMENTE ATTIVI (TaskMonitor_GetFaults()) intersecano
 *         g_config.fault_latch_mask, persiste i bit corrispondenti in
 *         g_config.fault_latch_active — IMMEDIATAMENTE (Config_Save(), non
 *         attende un "SAVE CONFIG" manuale): un fault latched deve
 *         sopravvivere anche a un power-cycle avvenuto un istante dopo.
 *         Idempotente: non ri-scrive la flash se i bit sono già latched
 *         (evita usura flash inutile su fault persistenti/ricorrenti).
 *         Chiamata da action_fault() a ogni ingresso in SYS_FAULT.
 */
static void fault_latch_check_and_persist(void)
{
    uint32_t active   = TaskMonitor_GetFaults();
    uint32_t to_latch = active & g_config.fault_latch_mask;

    if (to_latch != 0U && (g_config.fault_latch_active & to_latch) != to_latch) {
        g_config.fault_latch_active |= to_latch;
        Config_Save();
    }
}

/*
 * @brief  Decodifica error_code (FSM_ErrorCode_t, fsm.h) in una stringa
 *         "BIT0|BIT1|..." con i nomi dei bit attivi, oppure "NONE" se 0.
 *         Vedi dichiarazione in fsm.h per gli usi (log SD, GET LQERR).
 */
void FSM_FormatErrorCode(uint8_t error_code, char *buf, size_t buf_size)
{
    if (buf == NULL || buf_size == 0U) return;

    static const struct { uint8_t bit; const char *name; } s_bits[] = {
        { FSM_ERR_TEMP_DRIVER,   "TEMP_DRIVER"   },
        { FSM_ERR_TEMP_AMBIENT,  "TEMP_AMBIENT"  },
        { FSM_ERR_VANODE,        "VANODE"        },
        { FSM_ERR_CURRENT_MON,   "CURRENT_MON"   },
        { FSM_ERR_RS485_TIMEOUT, "RS485_TIMEOUT" },
        { FSM_ERR_SATURATION,    "SATURATION"    },
        { FSM_FAULT_HW,          "FAULT_HW"      },
    };

    buf[0] = '\0';
    size_t used = 0U;
    bool   first = true;

    for (size_t i = 0U; i < (sizeof(s_bits) / sizeof(s_bits[0])); i++) {
        if ((error_code & s_bits[i].bit) == 0U) continue;

        int n = snprintf(&buf[used], buf_size - used, "%s%s",
                          first ? "" : "|", s_bits[i].name);
        if (n < 0 || (size_t)n >= (buf_size - used)) break; /* troncato: esci */
        used += (size_t)n;
        first = false;
    }

    if (first) {
        /* Nessun bit noto attivo (error_code == 0, o solo bit 6 riservato) */
        snprintf(buf, buf_size, "NONE");
    }
}

/*
 * @brief  Decodifica hw_fault_source (FSM_HwFaultSource_t, fsm.h) in una
 *         stringa "BIT0|BIT1|..." con i nomi dei bit attivi, oppure "NONE"
 *         se 0. Dettaglio del bit generico FSM_FAULT_HW di error_code —
 *         stesso schema di FSM_FormatErrorCode() sopra. Vedi dichiarazione
 *         in fsm.h per gli usi ("GET LQ", rs485_cmd.c).
 */
void FSM_FormatHwFaultSource(uint8_t hw_fault_source, char *buf, size_t buf_size)
{
    if (buf == NULL || buf_size == 0U) return;

    static const struct { uint8_t bit; const char *name; } s_bits[] = {
        { HW_FAULT_PWR_OK,            "PWR_OK"            },
        { HW_FAULT_INTERLOCK_ENABLE,  "INTLCK_ENABLE"      },
        { HW_FAULT_INTERLOCK_RUNTIME, "INTLCK_RUNTIME"     },
    };

    buf[0] = '\0';
    size_t used = 0U;
    bool   first = true;

    for (size_t i = 0U; i < (sizeof(s_bits) / sizeof(s_bits[0])); i++) {
        if ((hw_fault_source & s_bits[i].bit) == 0U) continue;

        int n = snprintf(&buf[used], buf_size - used, "%s%s",
                          first ? "" : "|", s_bits[i].name);
        if (n < 0 || (size_t)n >= (buf_size - used)) break; /* troncato: esci */
        used += (size_t)n;
        first = false;
    }

    if (first) {
        snprintf(buf, buf_size, "NONE");
    }
}

/*
 * s_mode: modalità operativa corrente. Solo SW o ANALOG (vedi fsm.h,
 * sezione "MODALITÀ OPERATIVA": HYBRID non è più un valore di s_mode).
 *
 * Inizializzata a FSM_MODE_SW all'avvio (modalità sicura — intenzione
 * dell'utente sconosciuta). Modificata da (vedi blocchi dedicati in
 * FSM_ProcessEvent(), aggiornati 2026-07-27):
 *   SYS_EXT_CTRL_EVENT (pin, fronte discendente nEXT_CTL_iso) → ANALOG
 *     Qualunque stato tranne INIT/FAULT (no-op se già ANALOG). Da IDLE sale
 *     anche a SYS_ACTIVE (action_ext_ctrl_on); da ENABLED/EMISSION scende a
 *     SYS_ON; altrove lo stato resta invariato.
 *   SYS_SET_MODE_ANALOG_EVENT (IDLE/ACTIVE/ON/ENABLED/EMISSION) → ANALOG
 *   SYS_SET_MODE_SW_EVENT / SYS_SET_MODE_HYBRID_EVENT
 *     (IDLE/ACTIVE/ON/ENABLED/EMISSION)                        → SW
 *     (entrambi i comandi SET MODE: da ENABLED/EMISSION scendono a SYS_ON,
 *     in QUALUNQUE direzione — vedi drop_to_on lì)
 *   SYS_EXT_CTRL_OFF_EVENT (pin, fronte di salita nEXT_CTL_iso,
 *     action_ext_ctrl_off) → SW, e forza SEMPRE SYS_IDLE (qualunque stato,
 *     ERROR incluso) — no-op se già SW (filtrato dall'isolamento modale).
 *
 * FSM_ProcessEvent() usa s_mode come gate per filtrare gli eventi
 * in base al canale di controllo attivo (vedi isolamento modale in fsm.h),
 * e sync_cmd_rdy_status() lo riflette SEMPRE su nCMD_RDY_iso ad ogni
 * cambio (ANALOG=asserito/LOW, SW=deasserito/HIGH — indipendente dallo
 * stato FSM, vedi banner sopra sync_cmd_rdy_status()).
 */
static FSM_Mode_t s_mode = FSM_MODE_SW;

/*
 * s_gate_hw_enabled / s_setpoint_hw_enabled: toggle indipendenti
 * "HYBRID1"/"HYBRID2" (vedi banner "MODALITÀ OPERATIVA" in fsm.h).
 * Significativi SOLO quando s_mode == FSM_MODE_SW; consultati in
 * action_enter_enabled() e in FSM_GetLaserModeWire().
 *
 * Azzerati da reset_hybrid_toggles(), chiamata da: FSM_Init(), ogni SET
 * MODE SW/ANALOG/HYBRID andato a buon fine, action_ext_ctrl_on() e
 * action_ext_ctrl_off() — nessuno stato "attivo" sopravvive a un cambio
 * di modalità, va sempre riarmato esplicitamente nella nuova modalità.
 */
static bool s_gate_hw_enabled     = false;
static bool s_setpoint_hw_enabled = false;

static void reset_hybrid_toggles(void)
{
    s_gate_hw_enabled     = false;
    s_setpoint_hw_enabled = false;
}

/*
 * @brief  Applica SUBITO a LaseQ (via LaseQSetMode()) il mode byte
 *         (sw_control/analog_mode) coerente con s_mode/s_setpoint_hw_enabled
 *         correnti — indipendentemente dallo stato FSM corrente.
 *
 * Prima di questa funzione, analog_mode veniva scritto in s_ctrl SOLO da
 * action_enter_enabled(): selezionare ANALOG mode (o il toggle SETPOINTHW)
 * mentre il sistema non era ancora SYS_ENABLED lasciava LaseQ con il byte
 * della modalità precedente fino al successivo ingresso in ENABLED — non
 * un problema di sicurezza (l'uscita di corrente resta comunque governata
 * da "enable", gestito separatamente da EnableLaseQ()/DisableLaseQ()), ma
 * un ritardo indesiderato nella telemetria/wire verso LaseQ.
 *
 * Chiamata ogni volta che s_mode o s_setpoint_hw_enabled cambiano: nel
 * blocco SET MODE / SET GATEHW/SETPOINTHW di FSM_ProcessEvent(), in
 * action_ext_ctrl_on()/action_ext_ctrl_off() e in FSM_Init(). Resta
 * comunque chiamata anche da action_enter_enabled() (ridondante ma
 * innocua: stesso valore, vedi sync_laseq_mode() lì) come rete di
 * sicurezza aggiuntiva.
 */
static void sync_laseq_mode(void)
{
    LaseQSetMode((s_mode == FSM_MODE_ANALOG || s_setpoint_hw_enabled) ? ANALOG : SW);
}

/*
 * @brief  OUTPUT nCMD_RDY_iso (richiesto esplicitamente, 2026-07-27): segue
 *         SOLO il canale di controllo attivo (s_mode), MAI lo stato FSM:
 *           s_mode == FSM_MODE_ANALOG -> nCMD_RDY_iso asserito (LOW)
 *           s_mode == FSM_MODE_SW     -> nCMD_RDY_iso deasserito (HIGH)
 *         (HYBRID non è un valore separato di s_mode — resta FSM_MODE_SW,
 *         vedi FSM_GetLaserModeWire() — quindi conta come SW anche qui.)
 *
 *         Chiamata ad OGNI cambio di s_mode: apply_mode_analog(),
 *         action_ext_ctrl_off(), blocco SET MODE SW/ANALOG/HYBRID in
 *         FSM_ProcessEvent(), FSM_Init(). Le action di ingresso stato
 *         (action_enter_active/on/enabled/...) NON la chiamano più: prima
 *         della revisione del 2026-07-27 nCMD_RDY_iso veniva erroneamente
 *         asserito/deasserito in funzione dello stato ACTIVE invece che
 *         della modalità di controllo.
 */
static void sync_cmd_rdy_status(void)
{
    EXT_SetCmdStatus(s_mode == FSM_MODE_ANALOG);
}

/*
 * s_hw_setpoint_sel: configurazione sorgente setpoint HW per LaseQ.
 *
 * Letto una volta da g_config.hw_setpoint_sel in FSM_Init().
 * Rilevante solo quando LaseQ riceve analog_mode=1: FSM_MODE_ANALOG, oppure
 * FSM_MODE_SW con il toggle indipendente "HYBRID2"/setpoint HW attivo
 * (s_setpoint_hw_enabled) — vedi action_enter_enabled(). In SW puro LaseQ
 * ignora il pin analogico.
 *
 *   0 = LPWR_SET_ISO grezzo dall'EXT interface → LaseQ (default)
 *   1 = DAC di AMC (segnale corretto/linearizzato) → LaseQ
 *
 * La scelta tra 0 e 1 dipende dalla configurazione utente, non dal modo FSM.
 * Default fabbrica 0 (AMC DAC non ancora implementato in produzione).
 * Modificabile a runtime via comando RS485 "SET SETPOINTCOMP ON|OFF"
 * (rs485_cmd.c), che aggiorna g_config.hw_setpoint_sel, il pin PF14
 * (BoardCtrl_SetpointSel()) e questa copia live insieme, tramite
 * FSM_SetHwSetpointSel() — vedi fsm.h.
 */
static uint8_t s_hw_setpoint_sel = 0U;

/* ============================================================================
 * GUARDIA 10s SU SPEGNIMENTO SAB (requisito hardware — vedi SAB.h)
 * ============================================================================
 * Il fronte di discesa di nSAB_EN (SAB_Disable()) avvia la scarica di alcune
 * capacità sul modulo SAB, necessaria per un riavvio corretto. Rimuovere
 * l'alimentazione (nSAB_PWR_SHDN LOW, SAB_ShutdownSupply()) prima che siano
 * trascorsi almeno SAB_POWEROFF_GUARD_MS da quel fronte impedisce la scarica
 * corretta.
 *
 * Le action che escono da SYS_ENABLED/SYS_EMISSION verso uno stato in cui il
 * SAB deve restare spento (action_enter_active/idle/error/fault e
 * action_clear_error — TUTTE tranne action_enter_on(), che ridisarma senza
 * mai togliere alimentazione, vedi sotto) NON chiamano più SAB_PowerOff()
 * direttamente: chiamano sab_request_shutdown(), che disarma SUBITO
 * (SAB_Disable(), sempre sicuro farlo immediatamente) e pianifica
 * SAB_ShutdownSupply() non prima di SAB_POWEROFF_GUARD_MS (vedi FSM_Tick()).
 *
 * Se il SAB viene riarmato (SAB_Enable(), in action_enter_enabled()) prima
 * che la finestra scada, lo spegnimento pianificato viene annullato da
 * sab_cancel_pending_shutdown() — altrimenti FSM_Tick() taglierebbe
 * l'alimentazione a un SAB tornato attivo. action_enter_on() (SYS_ON) non
 * chiama mai SAB_PowerOff/ShutdownSupply: si limita a SAB_Disable()+
 * SAB_PowerOn(), lasciando il modulo alimentato ma disarmato in attesa del
 * prossimo SAB_Enable() — nessuna guardia necessaria in quel percorso,
 * l'alimentazione non viene mai rimossa.
 *
 * action_init_ok() (SYS_INIT→SYS_IDLE, al boot) resta l'unica eccezione:
 * chiama ancora SAB_PowerOff() diretto, senza guardia — il SAB non è mai
 * stato armato, nessun condensatore da scaricare.
 *
 * Deadline assoluto (HAL_GetTick()) invece di un countdown decrementato a
 * periodo fisso: il loop di TaskFSM_Run non gira a periodo fisso
 * (osMessageQueueGet con timeout, si sblocca prima se arriva un evento),
 * quindi un countdown "a ogni giro" sotto-conterebbe il tempo reale
 * trascorso. Stesso schema usato da SAB_BOOT_DISCHARGE più sotto.
 */
#define SAB_POWEROFF_GUARD_MS   2000U   /* SAB_POWEROFF_GUARD */

static bool     s_sab_poweroff_pending  = false; /* SAB_POWEROFF_GUARD: spegnimento pianificato */
static uint32_t s_sab_poweroff_deadline = 0U;     /* SAB_POWEROFF_GUARD: HAL_GetTick() di scadenza */

/*
 * @brief  Disarma SUBITO il SAB (SAB_Disable()) e pianifica la rimozione
 *         dell'alimentazione non prima di SAB_POWEROFF_GUARD_MS.
 *
 * Idempotente rispetto alla deadline: se uno spegnimento è già pianificato
 * (s_sab_poweroff_pending true) NON la posticipa ulteriormente — il fronte
 * di discesa di nSAB_EN è già avvenuto alla prima chiamata (SAB_Disable() è
 * comunque sempre sicura da richiamare, scrive lo stesso valore sul pin),
 * quindi i 10s vanno contati da quel primo fronte, non da eventuali chiamate
 * ridondanti successive (es. action_enter_error() seguita a ruota da
 * action_clear_error() se l'operatore fa CERR quasi subito).
 */
static void sab_request_shutdown(void)
{
    SAB_Disable();

    if (!s_sab_poweroff_pending) {
        s_sab_poweroff_pending  = true;
        s_sab_poweroff_deadline = HAL_GetTick() + SAB_POWEROFF_GUARD_MS;
    }
}

/*
 * @brief  Annulla uno spegnimento SAB pianificato (sab_request_shutdown()).
 *         Da chiamare ogni volta che il SAB viene riarmato (SAB_Enable(),
 *         in action_enter_enabled()) prima che la finestra sia scaduta:
 *         altrimenti FSM_Tick() taglierebbe l'alimentazione a un SAB tornato
 *         attivo, non essendoci più nulla da "posticipare".
 */
static void sab_cancel_pending_shutdown(void)
{
    s_sab_poweroff_pending = false;
}

/* ============================================================================
 * FEATURE-FLAG: SAB_BOOT_DISCHARGE
 * ============================================================================
 * Aggiunta il 2026-07-07 su richiesta esplicita: requisito hardware (scarica
 * capacità SAB) ma con una procedura specifica che potrebbe cambiare o non
 * essere più necessaria in futuro (es. se l'hardware del modulo SAB viene
 * rivisto). Tutto il codice che la implementa è marcato con questo stesso tag
 * "SAB_BOOT_DISCHARGE" — cercalo (grep) per trovare OGNI punto da toccare in
 * caso di rimozione:
 *
 *   fsm.h        : dichiarazioni FSM_StartBootDischarge()/FSM_IsBootDischargeDone()
 *   fsm.c        : questo blocco (define + enum + variabili statiche)
 *   fsm.c        : blocco dentro FSM_Tick() (avanzamento della routine)
 *   fsm.c        : chiamate a FSM_StartBootDischarge()/FSM_IsBootDischargeDone()
 *   task_fsm.c   : logica di attesa nel loop di TaskFSM_Run (vedi commenti lì)
 *   SAB.h/SAB.c  : SAB_RawEnPulse() (usata SOLO da questa routine — se la
 *                  feature viene rimossa, rimuovere anche questa primitiva)
 *
 * PER RIMUOVERE COMPLETAMENTE LA FEATURE:
 *   1. Eliminare tutti i blocchi elencati sopra (cercare "SAB_BOOT_DISCHARGE").
 *   2. In task_fsm.c, TaskFSM_Run() torna a chiamare TaskFSM_RunInitSequence()
 *      subito dopo FSM_Init(), come prima dell'introduzione della feature.
 *
 * COSA FA (vedi anche banner in SAB.h):
 *   Al boot, prima che la FSM possa transire da SYS_INIT a SYS_IDLE (quindi
 *   PRIMA della sequenza di check hardware in TaskFSM_RunInitSequence()):
 *     1. SAB_PowerOn()                    — alimenta il modulo SAB.
 *     2. Attesa SAB_GUARD_DELAY_MS        — il pin nSAB_EN (e il circuito che
 *        ne rileva i fronti per la scarica) è a valle dell'alimentazione
 *        appena attivata: senza questa attesa il fronte di salita del punto
 *        4 rischia di non essere rilevato (osservato su hardware: nSAB_EN
 *        non risultava asserito). Stesso identico requisito di
 *        stabilizzazione già documentato per SAB_Enable() in SAB.h (da cui
 *        si riusa la costante SAB_GUARD_DELAY_MS).
 *     3. SAB_RawEnPulse(true)  (nSAB_EN LOW)  — impulso di scarica, 10ms.
 *     4. SAB_RawEnPulse(false) (nSAB_EN HIGH) — fronte di salita: è questo
 *        fronte (non quello di discesa del punto 3) a innescare la scarica
 *        delle capacità — attesa 5s da qui.
 *     5. SAB_PowerOff()                   — modulo rispento (mai armato:
 *        nessuna guardia SAB_POWEROFF_GUARD necessaria, vedi sopra).
 *   Non bloccante: stesso schema a deadline assoluto (HAL_GetTick()) del
 *   resto del file, avanzato da FSM_Tick(). I PSU sono già spenti in questa
 *   finestra (nessuna action li ha ancora accesi: si è ancora in SYS_INIT).
 * ============================================================================
 */
#define SAB_BOOT_DISCHARGE_PULSE_MS   10U     /* SAB_BOOT_DISCHARGE: durata impulso nSAB_EN LOW */
#define SAB_BOOT_DISCHARGE_WAIT_MS  3000U     /* SAB_BOOT_DISCHARGE: attesa scarica dopo il fronte di salita */
/* SAB_BOOT_DISCHARGE_POWERUP_MS: riusa SAB_GUARD_DELAY_MS (SAB.h) — stessa
 * attesa di stabilizzazione alimentazione già richiesta da SAB_Enable(). */
#define SAB_BOOT_DISCHARGE_POWERUP_MS SAB_GUARD_DELAY_MS

typedef enum {
    SAB_BOOT_DISCHARGE_IDLE = 0, /* SAB_BOOT_DISCHARGE: non avviata o completata */
    SAB_BOOT_DISCHARGE_POWERUP,  /* SAB_BOOT_DISCHARGE: SAB_PowerOn() fatto, attesa stabilizzazione */
    SAB_BOOT_DISCHARGE_PULSE,    /* SAB_BOOT_DISCHARGE: nSAB_EN forzato LOW, attesa impulso */
    SAB_BOOT_DISCHARGE_WAIT,     /* SAB_BOOT_DISCHARGE: nSAB_EN HIGH, attesa scarica capacità */
} SabBootDischargeState_t;

static SabBootDischargeState_t s_boot_discharge_state    = SAB_BOOT_DISCHARGE_IDLE; /* SAB_BOOT_DISCHARGE */
static uint32_t                s_boot_discharge_deadline = 0U;                      /* SAB_BOOT_DISCHARGE */
static bool                    s_boot_discharge_done     = false;                   /* SAB_BOOT_DISCHARGE */

/*
 * @brief  SAB_BOOT_DISCHARGE: avvia la routine di scarica capacità SAB al
 *         boot. Da chiamare UNA SOLA VOLTA, dopo FSM_Init() e dopo
 *         l'inizializzazione di tutte le periferiche, PRIMA di eseguire
 *         TaskFSM_RunInitSequence() (vedi task_fsm.c).
 */
void FSM_StartBootDischarge(void)
{
    SAB_PowerOn();
    /* nSAB_EN NON viene toccato qui: resta al valore impostato da SAB_Init()
     * (HIGH) durante l'attesa di stabilizzazione — vedi SAB_BOOT_DISCHARGE_POWERUP
     * in FSM_Tick(). */

    s_boot_discharge_state    = SAB_BOOT_DISCHARGE_POWERUP;
    s_boot_discharge_deadline = HAL_GetTick() + SAB_BOOT_DISCHARGE_POWERUP_MS;
    s_boot_discharge_done     = false;
}

/*
 * @brief  SAB_BOOT_DISCHARGE: true quando la routine di scarica al boot è
 *         terminata (SAB rispento). Da interrogare in task_fsm.c prima di
 *         eseguire TaskFSM_RunInitSequence().
 */
bool FSM_IsBootDischargeDone(void)
{
    return s_boot_discharge_done;
}

/* ============================================================================
 * FEATURE-FLAG: SAB_ON_EXIT_PULSE
 * ============================================================================
 * Aggiunta il 2026-07-07 su richiesta esplicita: work-around per un buco
 * scoperto nel meccanismo di scarica capacità SAB. Tutto il codice è marcato
 * con questo tag — cercalo (grep) per trovare l'unico punto da toccare in
 * caso di rimozione: il blocco `if (from == SYS_ON) { ... }` dentro
 * action_enter_active() più sotto.
 *
 * PERCHÉ SERVE:
 *   SAB viene ALIMENTATO in action_enter_on() (SAB_PowerOn(), ingresso in
 *   SYS_ON) ma viene ARMATO (SAB_Enable(), nSAB_EN LOW) solo più avanti, in
 *   action_enter_enabled() (ingresso in SYS_ENABLED). Se il sistema esce da
 *   SYS_ON senza mai essere passato da SYS_ENABLED (transizione ON→ACTIVE,
 *   es. SYS_OFF_EVENT), nSAB_EN non ha MAI generato un fronte: è rimasto
 *   HIGH dall'avvio. In questo caso sab_request_shutdown() (chiamata subito
 *   sopra in action_enter_active()) esegue un SAB_Disable() che è un no-op
 *   sul piano elettrico (nSAB_EN già HIGH) — nessun fronte di salita, quindi
 *   nessun innesco della scarica capacità, anche se il modulo è rimasto
 *   alimentato (e le sue capacità potenzialmente cariche) per tutto il tempo
 *   trascorso in SYS_ON.
 *
 * COSA FA:
 *   Solo quando from == SYS_ON, dopo essersi assicurati che i PSU siano già
 *   spenti (psu_turn_off_all() sopra), genera manualmente l'impulso
 *   nSAB_EN basso/alto (SAB_RawEnPulse(), stessa primitiva di
 *   SAB_BOOT_DISCHARGE — bypassa SAB_State_t, che qui è comunque già
 *   SAB_STATE_DISABLED: SAB non era mai stato armato). Il fronte di salita
 *   finale innesca la scarica SUBITO, senza aspettare i
 *   SAB_POWEROFF_GUARD_MS della rimozione alimentazione già pianificata da
 *   sab_request_shutdown() — non c'è nulla da "disarmare con calma" qui, il
 *   modulo non era armato.
 *
 * Blocca l'esecuzione dell'action per SAB_ON_EXIT_PULSE_MS (pochi ms):
 * accettabile, questa action gira sempre nel contesto del task_fsm (mai da
 * ISR), stesso pattern di osDelay già usato in action_enter_on()
 * (contactor_psu_delay_ms).
 * ============================================================================
 */
#define SAB_ON_EXIT_PULSE_MS   10U   /* SAB_ON_EXIT_PULSE: durata impulso nSAB_EN LOW */

/* ============================================================================
 * FEATURE-FLAG: SAB_REARM_BLOCK
 * ============================================================================
 * Aggiunta il 2026-07-07 su richiesta esplicita. Tutto il codice è marcato
 * con questo tag — cercalo (grep) per trovare OGNI punto da toccare in caso
 * di rimozione:
 *
 *   fsm.h        : dichiarazione FSM_IsSabRearmBlocked()
 *   fsm.c        : questo blocco (define + variabili statiche + helper)
 *   fsm.c        : armo della guardia in action_enter_on()
 *   fsm.c        : controllo in FSM_ProcessEvent() (blocca il riarmo)
 *   rs485_cmd.c  : err_sab_guard() + controllo in cmd_sen()
 *
 * PERCHÉ SERVE:
 *   La transizione SYS_ENABLED/SYS_EMISSION → SYS_ON (SYS_DISABLE_EVENT e
 *   affini, action_enter_on()) disarma il SAB (SAB_Disable(): nSAB_EN
 *   HIGH — un VERO fronte di salita, dato che il SAB era armato) ma NON lo
 *   spegne (SAB_PowerOn() subito dopo lo mantiene alimentato: si può
 *   tornare in SYS_ENABLED senza dover ripassare da SYS_ACTIVE). Quel
 *   fronte di salita innesca comunque il ciclo hardware di scarica
 *   capacità (~10s, stesso meccanismo di SAB_BOOT_DISCHARGE/
 *   SAB_ON_EXIT_PULSE). Senza questa guardia, un riarmo (SYS_ON→
 *   SYS_ENABLED, cioè un nuovo SAB_Enable()/nSAB_EN LOW) potrebbe avvenire
 *   PRIMA che la scarica sia completa.
 *
 * COSA FA:
 *   action_enter_on() arma la guardia SOLO quando from == SYS_ENABLED o
 *   SYS_EMISSION (gli unici casi in cui SAB_Disable() produce un fronte
 *   reale — da SYS_ACTIVE il SAB era già disarmato, nessun fronte, nessuna
 *   scarica in corso). Per SAB_REARM_BLOCK_MS successivi, FSM_ProcessEvent()
 *   scarta silenziosamente SYS_ENABLE_EVENT/SYS_EXT_ENABLE_EVENT (gli unici
 *   due eventi che portano a SYS_ENABLED, vedi tabella [SYS_ON]).
 *   FSM_IsSabRearmBlocked() espone lo stesso controllo a rs485_cmd.c, per
 *   evitare che SEN risponda OK mentre l'evento verrebbe scartato (stesso
 *   principio già applicato per l'isolamento modale — vedi err_mode()).
 *
 * Deadline assoluto (HAL_GetTick()), stesso motivo delle altre guardie in
 * questo file: pulizia "lazy", nessun bisogno di FSM_Tick() qui (a
 * differenza di SAB_POWEROFF_GUARD, non c'è un'azione da eseguire allo
 * scadere — solo un gate da riaprire, verificato al bisogno).
 * ============================================================================
 */
#define SAB_REARM_BLOCK_MS   5000U   /* SAB_REARM_BLOCK */

static bool     s_sab_rearm_block_active   = false; /* SAB_REARM_BLOCK */
static uint32_t s_sab_rearm_block_deadline = 0U;     /* SAB_REARM_BLOCK */

/*
 * @brief  SAB_REARM_BLOCK: true se il riarmo SAB (SYS_ON→SYS_ENABLED) è
 *         attualmente bloccato (finestra di scarica capacità in corso).
 *         Pulizia lazy: se la finestra è scaduta, azzera il flag qui.
 */
static bool sab_rearm_blocked(void)
{
    if (!s_sab_rearm_block_active) {
        return false;
    }

    if ((int32_t)(HAL_GetTick() - s_sab_rearm_block_deadline) >= 0) {
        s_sab_rearm_block_active = false;
        return false;
    }

    return true;
}

/* ========================================================================== */
/* --- PROTOTIPI DELLE ACTION (funzioni statiche, non visibili all'esterno) -- */
/* ========================================================================== */

static void action_init_ok       (SysState_t from, SysEvent_t ev);
static void action_fault         (SysState_t from, SysEvent_t ev);
static void action_fault_reset   (SysState_t from, SysEvent_t ev);
static void action_enter_idle    (SysState_t from, SysEvent_t ev);
static void action_enter_active  (SysState_t from, SysEvent_t ev);
static void action_enter_on      (SysState_t from, SysEvent_t ev);
static void action_enter_enabled (SysState_t from, SysEvent_t ev);
static void action_enter_emission(SysState_t from, SysEvent_t ev);
static void action_enter_error   (SysState_t from, SysEvent_t ev);
static void action_clear_error   (SysState_t from, SysEvent_t ev);
static void action_ext_ctrl_on   (SysState_t from, SysEvent_t ev);
static void action_ext_ctrl_off  (SysState_t from, SysEvent_t ev);

/*
 * apply_mode_analog(): non è una "action" di tabella (nessuno stato di
 * destinazione fisso, vedi banner sopra la sua definizione) ma è chiamata
 * direttamente da FSM_ProcessEvent() (blocco SYS_EXT_CTRL_EVENT), PRIMA
 * della propria definizione nel file (sezione "IMPLEMENTAZIONE DELLE
 * ACTION" più sotto) — richiede quindi un prototipo qui, a differenza delle
 * action_* sopra che sono referenziate solo dalla tabella state_machine[][]
 * (già dichiarata dopo questi prototipi).
 */
static void apply_mode_analog(void);

/* ========================================================================== */
/* --- TABELLA DI TRANSIZIONI --- */
/*
 * [stato_corrente][evento] = { stato_prossimo, action }
 *
 * Le celle non popolate restano {0, NULL} (SYS_INIT, NULL).
 * FSM_ProcessEvent() controlla che next_state != SYS_NUM_STATES prima
 * di eseguire la transizione: le righe non popolate vengono scartate.
 *
 * Per rendere esplicita una transizione "non definita" usiamo la macro
 * NO_TRANS: rende il codice più leggibile della cella vuota.
 */
/* ========================================================================== */

#define NO_TRANS  { SYS_NUM_STATES, NULL }

static const SysStateTransition_t state_machine[SYS_NUM_STATES][SYS_NUM_EVENTS] = {

    /* ------------------------------------------------------------------ */
    [SYS_INIT] = {
        [SYS_CHECK_OK_EVENT]    = { SYS_IDLE,  action_init_ok },
        [SYS_CHECK_FAIL_EVENT]  = { SYS_FAULT, action_fault   },
        [SYS_FAULT_EVENT]       = { SYS_FAULT, action_fault   },
    },

    /* ------------------------------------------------------------------ */
    [SYS_IDLE] = {
        [SYS_START_EVENT]           = { SYS_ACTIVE, action_enter_active },
        [SYS_BUTTON_PRESS_EVENT]    = { SYS_ACTIVE, action_enter_active },
        /* SYS_EXT_CTRL_EVENT: gestito in un blocco dedicato in
         * FSM_ProcessEvent(), non tramite questa tabella — vedi commento lì
         * (il target di stato dipende dal mode/stato di provenienza a runtime). */
        [SYS_EXT_CTRL_OFF_EVENT]    = { SYS_IDLE,   action_ext_ctrl_off },
        [SYS_FAULT_EVENT]           = { SYS_FAULT,  action_fault        },
    },

    /* ------------------------------------------------------------------ */
    [SYS_ACTIVE] = {
        [SYS_STOP_EVENT]                = { SYS_IDLE,  action_enter_idle  },
        [SYS_ON_EVENT]                  = { SYS_ON,    action_enter_on    },
        [SYS_BUTTON_LONG_PRESS_EVENT]   = { SYS_ON,    action_enter_on    },
        [SYS_EXT_SYSON_EVENT]           = { SYS_ON,    action_enter_on    },
        [SYS_BUTTON_DOUBLE_PRESS_EVENT] = { SYS_IDLE,  action_enter_idle  },
        [SYS_EXT_CTRL_OFF_EVENT]        = { SYS_IDLE,  action_ext_ctrl_off},
        [SYS_SAB_INTERLOCK_OPEN_EVENT]  = { SYS_ERROR, action_enter_error },
        [SYS_SAB_TIMEOUT_EVENT]         = { SYS_ERROR, action_enter_error },
        [SYS_SAB_TEST_FAIL_EVENT]       = { SYS_ERROR, action_enter_error },
        [SYS_ERROR_EVENT]               = { SYS_ERROR, action_enter_error },
        [SYS_LASEQ_ERROR_EVENT]         = { SYS_ERROR, action_enter_error },
        [SYS_FAULT_EVENT]               = { SYS_FAULT, action_fault       },
        [SYS_FLOOD_FAULT_EVENT]         = { SYS_FAULT, action_fault       },
        [SYS_LASEQ_FAULT_EVENT]         = { SYS_FAULT, action_fault       },
        [SYS_AMC_FAULT_EVENT]           = { SYS_FAULT, action_fault       },
        [SYS_COM_FAULT_EVENT]           = { SYS_FAULT, action_fault       },
        /*
         * SYS_FLOW_ERROR_EVENT e SYS_TEMP_ERROR_EVENT non sono definiti qui:
         * in ACTIVE il sistema sta avviando il raffreddamento e queste
         * condizioni sono attese (pompa non a regime, termostati non
         * stabilizzati). I fault operativi vengono controllati solo
         * da SYS_ON in poi (vedi righe SYS_ON, SYS_ENABLED, SYS_EMISSION).
         * task_monitor posta questi eventi solo quando FSM_GetState() >= SYS_ON.
         */
    },

    /* ------------------------------------------------------------------ */
    [SYS_ON] = {
        [SYS_OFF_EVENT]                 = { SYS_ACTIVE,  action_enter_active  },
        [SYS_ENABLE_EVENT]              = { SYS_ENABLED, action_enter_enabled },
        [SYS_EXT_ENABLE_EVENT]          = { SYS_ENABLED, action_enter_enabled },
        [SYS_BUTTON_DOUBLE_PRESS_EVENT] = { SYS_ACTIVE,  action_enter_active  },
        [SYS_EXT_SYSOFF_EVENT]          = { SYS_ACTIVE,  action_enter_active  },
        [SYS_EXT_CTRL_OFF_EVENT]        = { SYS_IDLE,    action_ext_ctrl_off  },
        [SYS_KEY_REMOVED_EVENT]         = { SYS_ERROR,   action_enter_error   },
        [SYS_LID_OPEN_EVENT]            = { SYS_ERROR,   action_enter_error   },
        [SYS_SAB_INTERLOCK_OPEN_EVENT]  = { SYS_ERROR,   action_enter_error   },
        [SYS_SAB_TIMEOUT_EVENT]         = { SYS_ERROR,   action_enter_error   },
        [SYS_SAB_TEST_FAIL_EVENT]       = { SYS_ERROR,   action_enter_error   },
        [SYS_ERROR_EVENT]               = { SYS_ERROR,   action_enter_error   },
        [SYS_LASEQ_ERROR_EVENT]         = { SYS_ERROR,   action_enter_error   },
        [SYS_FAULT_EVENT]               = { SYS_FAULT,   action_fault         },
        [SYS_FLOOD_FAULT_EVENT]         = { SYS_FAULT,   action_fault         },
        [SYS_FLOW_ERROR_EVENT]          = { SYS_ERROR,   action_enter_error   },
        [SYS_TEMP_ERROR_EVENT]          = { SYS_ERROR,   action_enter_error   },
        [SYS_PSU_FAULT_EVENT]           = { SYS_FAULT,   action_fault         },
        [SYS_LASEQ_FAULT_EVENT]         = { SYS_FAULT,   action_fault         },
        [SYS_AMC_FAULT_EVENT]           = { SYS_FAULT,   action_fault         },
        [SYS_COM_FAULT_EVENT]           = { SYS_FAULT,   action_fault         },
    },

    /* ------------------------------------------------------------------ */
    [SYS_ENABLED] = {
        [SYS_OFF_EVENT]                 = { SYS_ACTIVE,   action_enter_active  },
        [SYS_DISABLE_EVENT]             = { SYS_ON,       action_enter_on      },
        [SYS_EXT_DISABLE_EVENT]         = { SYS_ON,       action_enter_on      },
        [SYS_BUTTON_DOUBLE_PRESS_EVENT] = { SYS_ON,       action_enter_on      },
        [SYS_LASERON_EVENT]             = { SYS_EMISSION, action_enter_emission},
        [SYS_EMISSION_REQUEST_EVENT]    = { SYS_EMISSION, action_enter_emission},
        [SYS_EXT_SYSOFF_EVENT]          = { SYS_ACTIVE,   action_enter_active  },
        [SYS_EXT_CTRL_OFF_EVENT]        = { SYS_IDLE,     action_ext_ctrl_off  },
        [SYS_KEY_REMOVED_EVENT]         = { SYS_ERROR,    action_enter_error   },
        [SYS_LID_OPEN_EVENT]            = { SYS_ERROR,    action_enter_error   },
        [SYS_SAB_INTERLOCK_OPEN_EVENT]  = { SYS_ERROR,    action_enter_error   },
        [SYS_SAB_TIMEOUT_EVENT]         = { SYS_ERROR,    action_enter_error   },
        [SYS_SAB_TEST_FAIL_EVENT]       = { SYS_ERROR,    action_enter_error   },
        [SYS_ERROR_EVENT]               = { SYS_ERROR,    action_enter_error   },
        [SYS_LASEQ_ERROR_EVENT]         = { SYS_ERROR,    action_enter_error   },
        [SYS_FAULT_EVENT]               = { SYS_FAULT,    action_fault         },
        [SYS_FLOOD_FAULT_EVENT]         = { SYS_FAULT,    action_fault         },
        [SYS_FLOW_ERROR_EVENT]          = { SYS_ERROR,    action_enter_error   },
        [SYS_TEMP_ERROR_EVENT]          = { SYS_ERROR,    action_enter_error   },
        [SYS_PSU_FAULT_EVENT]           = { SYS_FAULT,    action_fault         },
        [SYS_LASEQ_FAULT_EVENT]         = { SYS_FAULT,    action_fault         },
        [SYS_AMC_FAULT_EVENT]           = { SYS_FAULT,    action_fault         },
        [SYS_COM_FAULT_EVENT]           = { SYS_FAULT,    action_fault         },
    },

    /* ------------------------------------------------------------------ */
    [SYS_EMISSION] = {
        [SYS_OFF_EVENT]                 = { SYS_ACTIVE,   action_enter_active  },
        [SYS_DISABLE_EVENT]             = { SYS_ON,       action_enter_on      },
        [SYS_LASEROFF_EVENT]            = { SYS_ENABLED,  action_enter_enabled },
        [SYS_STOP_EMISSION_EVENT]       = { SYS_ENABLED,  action_enter_enabled },
        [SYS_BUTTON_DOUBLE_PRESS_EVENT] = { SYS_ENABLED,  action_enter_enabled },
        [SYS_EXT_DISABLE_EVENT]         = { SYS_ON,       action_enter_on      },
        [SYS_EXT_CTRL_OFF_EVENT]        = { SYS_IDLE,     action_ext_ctrl_off  },
        [SYS_EXT_SYSOFF_EVENT]          = { SYS_ACTIVE,   action_enter_active  },
        [SYS_KEY_REMOVED_EVENT]         = { SYS_ERROR,    action_enter_error   },
        [SYS_LID_OPEN_EVENT]            = { SYS_ERROR,    action_enter_error   },
        [SYS_SAB_INTERLOCK_OPEN_EVENT]  = { SYS_ERROR,    action_enter_error   },
        [SYS_SAB_TIMEOUT_EVENT]         = { SYS_ERROR,    action_enter_error   },
        [SYS_SAB_TEST_FAIL_EVENT]       = { SYS_ERROR,    action_enter_error   },
        [SYS_ERROR_EVENT]               = { SYS_ERROR,    action_enter_error   },
        [SYS_LASEQ_ERROR_EVENT]         = { SYS_ERROR,    action_enter_error   },
        [SYS_FAULT_EVENT]               = { SYS_FAULT,    action_fault         },
        [SYS_FLOOD_FAULT_EVENT]         = { SYS_FAULT,    action_fault         },
        [SYS_FLOW_ERROR_EVENT]          = { SYS_ERROR,    action_enter_error   },
        [SYS_TEMP_ERROR_EVENT]          = { SYS_ERROR,    action_enter_error   },
        [SYS_PSU_FAULT_EVENT]           = { SYS_FAULT,    action_fault         },
        [SYS_LASEQ_FAULT_EVENT]         = { SYS_FAULT,    action_fault         },
        [SYS_AMC_FAULT_EVENT]           = { SYS_FAULT,    action_fault         },
        [SYS_COM_FAULT_EVENT]           = { SYS_FAULT,    action_fault         },
    },

    /* ------------------------------------------------------------------ */
    [SYS_ERROR] = {
        [SYS_CLR_ERR_EVENT]             = { SYS_ACTIVE, action_clear_error },
        [SYS_EXT_CLR_ERR_EVENT]         = { SYS_ACTIVE, action_clear_error },
        [SYS_BUTTON_LONG_PRESS_EVENT]   = { SYS_ACTIVE, action_clear_error },
        /* NUOVO (2026-07-27): fronte di salita nEXT_CTL_iso deve riportare
         * in IDLE "in qualunque stato ci si trovi", ERROR incluso. */
        [SYS_EXT_CTRL_OFF_EVENT]        = { SYS_IDLE,   action_ext_ctrl_off},
        [SYS_FAULT_EVENT]               = { SYS_FAULT,  action_fault       },
        [SYS_FLOOD_FAULT_EVENT]         = { SYS_FAULT,  action_fault       },
        [SYS_LASEQ_FAULT_EVENT]         = { SYS_FAULT,  action_fault       },
        [SYS_AMC_FAULT_EVENT]           = { SYS_FAULT,  action_fault       },
        [SYS_COM_FAULT_EVENT]           = { SYS_FAULT,  action_fault       },
    },

    /* ------------------------------------------------------------------ */
    [SYS_FAULT] = {
        /*
         * SYS_FAULT_RESET_EVENT (dal 2026-07-14): unica uscita possibile,
         * riservata al comando "FRST" (RS485, protetto da login) o a un
         * futuro handler COM interface — MAI raggiungibile da un normale
         * comando operativo (CERR è per SYS_ERROR, non tocca questo stato).
         * Vedi action_fault_reset() e banner "FAULT LATCH" in
         * task_monitor.h/fsm_events.h.
         */
        [SYS_FAULT_RESET_EVENT]         = { SYS_IDLE,   action_fault_reset },
    },
};

/* ========================================================================== */
/* --- API PUBBLICA --- */
/* ========================================================================== */

void FSM_Init(void)
{
    s_current_state = SYS_INIT;

    /*
     * Modalità sicura all'avvio: la macchina non conosce ancora l'intenzione
     * dell'utente. La modalità effettiva viene determinata dalla prima
     * interazione (EXT_CTRL_iso LOW, oppure comando COM/RS485).
     * Vedi isolamento modale in fsm.h e FSM_ProcessEvent().
     */
    s_mode = FSM_MODE_SW;
    reset_hybrid_toggles();
    sync_laseq_mode();      /* SW/analog_mode=0 di default, coerente con s_mode */
    sync_cmd_rdy_status();  /* nCMD_RDY_iso=1 (HIGH) di default, coerente con SW */

    /*
     * Unica lettura di g_config per la configurazione HW setpoint.
     * Vedi commento di s_hw_setpoint_sel per la semantica.
     */
    s_hw_setpoint_sel = g_config.hw_setpoint_sel;

    /*
     * FAULT LATCH (dal 2026-07-14, vedi banner "FAULT LATCH" in
     * task_monitor.h): se un fault latched da un boot precedente è ancora
     * presente in flash (g_config.fault_latch_active), la FSM transisce
     * direttamente INIT -> FAULT QUI, prima ancora del boot discharge SAB e
     * del check hardware normale (TaskFSM_RunInitSequence(), task_fsm.c —
     * che verifica FSM_GetState() e salta l'intera sequenza se già in
     * SYS_FAULT). Richiede un comando esplicito "FRST" per uscirne.
     *
     * Isola il bit più basso attivo per ricostruire un nome leggibile
     * (log SD, "GET FAULT") — se più bit fossero latched insieme, il primo
     * ne determina il nome ma action_fault() ri-valuta comunque l'intera
     * intersezione con fault_latch_mask (nessuna perdita di informazione
     * sullo stato, solo sulla stringa di log per questo evento specifico).
     */
    uint32_t latched = g_config.fault_latch_active & g_config.fault_latch_mask;
    if (latched != 0U) {
        uint32_t lowest_bit = latched & (~latched + 1U);
        action_fault(SYS_INIT, fault_bit_to_event(lowest_bit));
        s_current_state = SYS_FAULT;
        SysLog_StateChange(SYS_INIT, SYS_FAULT);
    }
}

SysState_t FSM_GetState(void)
{
    return s_current_state;
}

FSM_Mode_t FSM_GetMode(void)
{
    return s_mode;
}

bool FSM_GetGateHwEnabled(void)
{
    return s_gate_hw_enabled;
}

bool FSM_GetSetpointHwEnabled(void)
{
    return s_setpoint_hw_enabled;
}

uint8_t FSM_GetLaserModeWire(void)
{
    /* Codifica STORICA del protocollo AMC (AMC_protocol.h): 0=SW, 1=HYBRID,
     * 2=ANALOG. Va mantenuta byte-per-byte anche se FSM_Mode_t non ha più
     * un valore HYBRID dedicato — vedi commento su FSM_GetLaserModeWire()
     * in fsm.h. */
    if (s_mode == FSM_MODE_ANALOG) {
        return 2U;
    }
    return s_setpoint_hw_enabled ? 1U : 0U;
}

SysEvent_t FSM_GetLastFaultEvent(void)
{
    return s_last_fault_event;
}

SysEvent_t FSM_GetLastErrorEvent(void)
{
    return s_last_error_event;
}

bool FSM_IsSabRearmBlocked(void)
{
    /* SAB_REARM_BLOCK (rimovibile, vedi banner sopra le variabili
     * statiche/prototipi action): stessa funzione statica usata dal
     * controllo interno in FSM_ProcessEvent(), esposta a rs485_cmd.c per
     * evitare risposte "OK" fuorvianti su SEN quando l'evento verrebbe
     * comunque scartato. */
    return sab_rearm_blocked();
}

const char *FSM_GetLastFaultName(void)
{
    if (s_last_fault_event == SYS_NUM_EVENTS) {
        return "NONE";
    }
    /*
     * Dettaglio sorgente (dal 2026-07-15, vedi event_detail_name()) se
     * disponibile — es. "FLOOD1" invece del generico "FLOOD" — altrimenti
     * ricade sul nome dell'evento come prima ("KEY", "PSU", ...).
     */
    if (s_last_fault_detail[0] != '\0') {
        return s_last_fault_detail;
    }
    return fault_event_name(s_last_fault_event);
}

const char *FSM_GetLastErrorName(void)
{
    if (s_last_error_event == SYS_NUM_EVENTS) {
        return "NONE";
    }
    if (s_last_error_detail[0] != '\0') {
        return s_last_error_detail;
    }
    return fault_event_name(s_last_error_event);
}

void FSM_RequestFaultReset(void)
{
    Queue_PostEvent(SYS_FAULT_RESET_EVENT);
}

bool FSM_IsFaultLatched(void)
{
    return (g_config.fault_latch_active & g_config.fault_latch_mask) != 0U;
}

uint8_t FSM_GetHwSetpointSel(void)
{
    return s_hw_setpoint_sel;
}

void FSM_SetHwSetpointSel(uint8_t sel)
{
    /* Solo la copia live: config in flash e pin PF14 restano a carico del
     * chiamante (vedi comando RS485 "SET SETPOINTCOMP ON|OFF",
     * rs485_cmd.c). Riletta da action_enter_emission() al prossimo ingresso
     * in SYS_EMISSION e da task_amc.c ad ogni heartbeat verso AMC. */
    s_hw_setpoint_sel = (sel != 0U) ? 1U : 0U;
}

void FSM_ProcessEvent(SysEvent_t event)
{
    /* Protezione bounds: ignora eventi fuori range */
    if (event >= SYS_NUM_EVENTS) return;

    /* Warning: non causano transizioni (intercettati prima della tabella) */
    if (event == SYS_HUMIDITY_WARN_EVENT) return;
    if (event == SYS_TEMP_WARN_EVENT)     return;

    /* ------------------------------------------------------------------
     * Comandi di cambio modalità (da COM/RS485).
     *
     * Accettati da SYS_IDLE, SYS_ACTIVE, SYS_ON (nessuna transizione di
     * stato: modificano solo s_mode, restando nello stato corrente).
     *
     * SYS_SET_MODE_SW_EVENT / SYS_SET_MODE_ANALOG_EVENT — NUOVO, richiesto
     * esplicitamente 2026-07-27 — accettati anche da SYS_ENABLED/
     * SYS_EMISSION: la macchina non può restare armata cambiando canale di
     * controllo sotto i piedi, quindi si scende a SYS_ON, in QUALUNQUE
     * direzione (SW->ANALOG o ANALOG->SW), riusando action_enter_on() per
     * la stessa sequenza di sicurezza di SDIS/EXT_DISABLE (SAB_REARM_BLOCK,
     * disarmo) — MA solo se la modalità cambia davvero: un comando
     * ridondante (es. "SET MODE ANALOG" mentre si è già in ANALOG ed
     * EMISSION) non deve interrompere un'emissione in corso senza motivo.
     * SYS_SET_MODE_HYBRID_EVENT NON è incluso in questa estensione (resta
     * IDLE/ACTIVE/ON come prima): è solo un alias SW con toggle GATE_HW/
     * SETPOINT_HW, non un cambio di canale di controllo — fuori scope per
     * ENABLED/EMISSION, non richiesto.
     * Tutti ignorati da SYS_ERROR/SYS_FAULT (invariato).
     *
     * A differenza del pin fisico nEXT_CTL_iso (SYS_EXT_CTRL_EVENT, gestito
     * in un blocco dedicato più sotto: stato preservato in ogni stato tranne
     * IDLE->ACTIVE ed ENABLED/EMISSION->ON), questi comandi SW->ANALOG non
     * forzano mai IDLE->ACTIVE: da IDLE restano in IDLE.
     * ------------------------------------------------------------------ */
    if (event == SYS_SET_MODE_SW_EVENT ||
        event == SYS_SET_MODE_HYBRID_EVENT ||
        event == SYS_SET_MODE_ANALOG_EVENT) {

        bool drop_to_on = (event != SYS_SET_MODE_HYBRID_EVENT) &&
                          (s_current_state == SYS_ENABLED ||
                           s_current_state == SYS_EMISSION);

        if (s_current_state == SYS_IDLE ||
            s_current_state == SYS_ACTIVE ||
            s_current_state == SYS_ON ||
            drop_to_on) {

            FSM_Mode_t new_mode = (event == SYS_SET_MODE_ANALOG_EVENT) ? FSM_MODE_ANALOG
                                                                        : FSM_MODE_SW;

            /*
             * NUOVO (richiesto esplicitamente, 2026-07-27): passaggio in
             * ANALOG consentito solo se nVEXT_GOOD=0 (EXT_IsVextGood()==true,
             * interfaccia EXT correttamente alimentata). rs485_cmd.c ha già
             * verificato questa condizione PRIMA di postare l'evento e
             * risposto con un errore esplicito se non soddisfatta — questo
             * controllo è una difesa in profondità (silenziosa) contro
             * un'eventuale perdita di alimentazione EXT nella finestra tra
             * la validazione del comando e il processing di questo evento.
             * SETPOINT HW / GATE HW (blocco toggle sotto) NON sono soggetti
             * a questo vincolo.
             */
            if (new_mode == FSM_MODE_ANALOG && !EXT_IsVextGood()) {
                return;
            }

            /* Comando ridondante da ENABLED/EMISSION (mode già quello
             * richiesto): nessuna conseguenza, non interrompe l'emissione. */
            if (drop_to_on && new_mode == s_mode) {
                return;
            }

            s_mode = new_mode;

            /* Ogni cambio di modalità riparte da toggle azzerati: vanno
             * riattivati esplicitamente nella nuova modalità (vedi
             * reset_hybrid_toggles()). L'alias legacy HYBRID li riattiva
             * entrambi subito dopo, per compatibilità. */
            reset_hybrid_toggles();
            if (event == SYS_SET_MODE_HYBRID_EVENT) {
                s_gate_hw_enabled     = true;
                s_setpoint_hw_enabled = true;
            }

            /* Applica SUBITO il nuovo byte a LaseQ, non solo al prossimo
             * ingresso in SYS_ENABLED (vedi sync_laseq_mode()), e aggiorna
             * nCMD_RDY_iso in base alla nuova modalità. */
            sync_laseq_mode();
            sync_cmd_rdy_status();

            if (drop_to_on) {
                SysState_t prev_state = s_current_state;
                action_enter_on(prev_state, event);
                s_current_state = SYS_ON;
                SysLog_StateChange(prev_state, s_current_state);
            }
        }
        return;
    }

    /* ------------------------------------------------------------------
     * SYS_EXT_CTRL_EVENT — fronte discendente fisico di nEXT_CTL_iso
     * (richiesto esplicitamente, 2026-07-27). Gestito qui, FUORI dalla
     * tabella di transizione: a differenza di ogni altra cella {stato
     * successivo, action} a target fisso, qui il target dipende dallo stato
     * di provenienza a runtime — non rappresentabile con una singola cella.
     *
     * Regole:
     *   - Già in ANALOG: nessuna conseguenza (fronte ridondante), in
     *     QUALUNQUE stato — incluso ENABLED/EMISSION (nessuna discesa
     *     spuria a SYS_ON per un fronte che non cambia nulla).
     *   - SYS_FAULT / SYS_INIT: nessun cambio modalità possibile (FAULT si
     *     esce solo con FRST; INIT è pre-boot).
     *   - SYS_IDLE: sale a SYS_ACTIVE (altrimenti l'esterno resterebbe
     *     bloccato in IDLE: nSYS_ON_iso/nENABLE_IN_iso funzionano solo da
     *     ACTIVE/ON, e i comandi SW sono bloccati in ANALOG).
     *   - SYS_ENABLED / SYS_EMISSION: scende a SYS_ON (stessa sequenza di
     *     sicurezza di SDIS/EXT_DISABLE, riusa action_enter_on()).
     *   - Ogni altro stato (ACTIVE, ON, ERROR): solo cambio modalità, stato
     *     invariato.
     *   - NUOVO (richiesto esplicitamente, 2026-07-27): il passaggio non
     *     avviene affatto se nVEXT_GOOD=1 (EXT_IsVextGood()==false,
     *     interfaccia EXT non alimentata correttamente) — nessun messaggio
     *     di errore, essendo un pin fisico: il fronte viene semplicemente
     *     ignorato, s_mode resta SW.
     * ------------------------------------------------------------------ */
    if (event == SYS_EXT_CTRL_EVENT) {
        if (s_mode == FSM_MODE_ANALOG) {
            return;   /* già in ANALOG: nessuna conseguenza */
        }
        if (s_current_state == SYS_FAULT || s_current_state == SYS_INIT) {
            return;
        }
        if (!EXT_IsVextGood()) {
            return;   /* nVEXT_GOOD=1: EXT interface non alimentata, ANALOG non consentito */
        }

        SysState_t prev_state = s_current_state;

        if (prev_state == SYS_IDLE) {
            action_ext_ctrl_on(prev_state, event);   /* mode=ANALOG + entra in ACTIVE */
            s_current_state = SYS_ACTIVE;
        } else if (prev_state == SYS_ENABLED || prev_state == SYS_EMISSION) {
            apply_mode_analog();
            action_enter_on(prev_state, event);      /* scende a ON, sicurezza SDIS */
            s_current_state = SYS_ON;
        } else {
            /* SYS_ACTIVE / SYS_ON / SYS_ERROR: solo modalità, stato invariato */
            apply_mode_analog();
        }

        if (s_current_state != prev_state) {
            SysLog_StateChange(prev_state, s_current_state);
        }
        return;
    }

    /* ------------------------------------------------------------------
     * Toggle indipendenti "HYBRID1"/"HYBRID2" (da COM/RS485).
     * Stesso range di stati ammessi dei comandi SET MODE sopra, con il
     * vincolo aggiuntivo che la modalità corrente sia FSM_MODE_SW (in
     * FSM_MODE_ANALOG questi toggle non hanno senso: gate e setpoint HW
     * sono già sempre attivi via hardware EXT). Non causano transizioni.
     * ------------------------------------------------------------------ */
    if (event == SYS_SET_GATE_HW_ON_EVENT  || event == SYS_SET_GATE_HW_OFF_EVENT ||
        event == SYS_SET_SETPOINT_HW_ON_EVENT || event == SYS_SET_SETPOINT_HW_OFF_EVENT) {

        bool state_ok = (s_current_state == SYS_IDLE ||
                         s_current_state == SYS_ACTIVE ||
                         s_current_state == SYS_ON);

        if (state_ok && s_mode == FSM_MODE_SW) {
            switch (event) {
                case SYS_SET_GATE_HW_ON_EVENT:      s_gate_hw_enabled     = true;  break;
                case SYS_SET_GATE_HW_OFF_EVENT:     s_gate_hw_enabled     = false; break;
                case SYS_SET_SETPOINT_HW_ON_EVENT:  s_setpoint_hw_enabled = true;  break;
                case SYS_SET_SETPOINT_HW_OFF_EVENT: s_setpoint_hw_enabled = false; break;
                default: break;
            }

            /* SETPOINT_HW cambia analog_mode; GATE_HW non lo tocca (solo
             * nGATE_HW_EN) — chiamata comunque in entrambi i casi: idempotente
             * e più semplice da mantenere che distinguere i due eventi qui. */
            sync_laseq_mode();
        }
        return;
    }

    /* ------------------------------------------------------------------
     * ISOLAMENTO MODALE — filtro bidirezionale per canale di controllo.
     *
     * FSM_MODE_ANALOG: la macchina è controllata dall'interfaccia EXT.
     *   Blocca tutti i comandi SW, COM/RS485 e da pulsante che causerebbero
     *   transizioni di stato. Fault e safety events passano sempre.
     *
     * FSM_MODE_SW: la macchina è controllata via firmware (RS485/pulsante),
     *   eventualmente con i toggle indipendenti sopra. Blocca tutti i
     *   comandi provenienti dai pin EXT state-change. SYS_EXT_CTRL_EVENT non
     *   compare in questo filtro: gestito interamente in un blocco dedicato
     *   PRIMA di questo punto (vedi sopra) — è il gate per entrare in
     *   ANALOG, non raggiunge mai questo switch né la tabella.
     *
     * ECCEZIONE CLR_ERR: SYS_CLR_ERR_EVENT (RS485/COM) e
     * SYS_EXT_CLR_ERR_EVENT (pin nCLR_ERR_iso) NON sono mai bloccati da
     * questo filtro, in NESSUNO dei due modi — il clear di un errore
     * recuperabile deve essere sempre accettato da entrambi i canali,
     * indipendentemente da quale stia effettivamente pilotando la macchina.
     * ------------------------------------------------------------------ */
    if (s_mode == FSM_MODE_ANALOG) {
        switch (event) {
            case SYS_START_EVENT:
            case SYS_STOP_EVENT:
            case SYS_ON_EVENT:
            case SYS_OFF_EVENT:
            case SYS_ENABLE_EVENT:
            case SYS_DISABLE_EVENT:
            case SYS_LASERON_EVENT:
            case SYS_LASEROFF_EVENT:
            case SYS_EMISSION_REQUEST_EVENT:
            case SYS_STOP_EMISSION_EVENT:
            case SYS_BUTTON_PRESS_EVENT:
            case SYS_BUTTON_LONG_PRESS_EVENT:
            case SYS_BUTTON_DOUBLE_PRESS_EVENT:
                return;  /* ignorato in ANALOG mode (SYS_CLR_ERR_EVENT escluso: vedi sopra) */
            default:
                break;
        }
    } else {
        /* FSM_MODE_SW (con o senza i toggle indipendenti GATE_HW/SETPOINT_HW) */
        switch (event) {
            case SYS_EXT_SYSON_EVENT:
            case SYS_EXT_SYSOFF_EVENT:
            case SYS_EXT_ENABLE_EVENT:
            case SYS_EXT_DISABLE_EVENT:
            case SYS_EXT_CTRL_OFF_EVENT:  /* nessun effetto in SW mode */
                return;  /* ignorato in SW mode (SYS_EXT_CLR_ERR_EVENT escluso: vedi sopra) */
            default:
                break;
        }
    }

    /* ------------------------------------------------------------------
     * SAB_REARM_BLOCK (rimovibile, vedi banner sopra le variabili
     * statiche/prototipi action). Scarta SOLO i due eventi che portano a
     * SYS_ENABLED (vedi tabella [SYS_ON]) mentre la finestra di scarica
     * capacità SAB è in corso.
     * ------------------------------------------------------------------ */
    if ((event == SYS_ENABLE_EVENT || event == SYS_EXT_ENABLE_EVENT) &&
        sab_rearm_blocked()) {
        return;
    }

    const SysStateTransition_t *t = &state_machine[s_current_state][event];

    /*
     * Transizione non definita per questa coppia (stato, evento): scarta.
     *
     * Due casi da gestire:
     *
     *   a) Cella esplicitamente marcata NO_TRANS = {SYS_NUM_STATES, NULL}.
     *      (usata con la macro NO_TRANS per chiarezza nel codice).
     *
     *   b) Cella NON inizializzata della tabella statica.
     *      In C, gli elementi non assegnati di un array statico vengono
     *      zero-inizializzati: {0, NULL}. Poiché SYS_INIT == 0, queste
     *      celle hanno next_state == SYS_INIT e action == NULL.
     *      Senza questo secondo controllo, FSM_ProcessEvent eseguirebbe
     *      silenziosamente una "transizione" a SYS_INIT per qualsiasi
     *      evento non definito nello stato corrente — reset indesiderato.
     *
     * Tutte le transizioni valide nella tabella hanno action != NULL,
     * quindi il predicato (next_state == SYS_INIT && action == NULL)
     * identifica univocamente le celle non inizializzate.
     */
    if (t->next_state == SYS_NUM_STATES) return;              /* caso a) */
    if (t->next_state == SYS_INIT && t->action == NULL) return; /* caso b) */

    /* Esegui l'action prima di aggiornare lo stato */
    if (t->action != NULL) {
        t->action(s_current_state, event);
    }

    SysState_t prev_state = s_current_state;
    s_current_state = t->next_state;

    /* Logga ogni transizione di stato */
    SysLog_StateChange(prev_state, s_current_state);
}

/*
 * FSM_Tick() gestisce le guardie/routine temporali indipendenti dagli
 * eventi, tutte a deadline assoluto (HAL_GetTick()):
 *
 *   1. SAB_POWEROFF_GUARD (vedi banner "GUARDIA 10s SU SPEGNIMENTO SAB"
 *      sopra le variabili statiche): requisito hardware — vedi
 *      sab_request_shutdown().
 *
 *   2. SAB_BOOT_DISCHARGE (vedi banner "FEATURE-FLAG: SAB_BOOT_DISCHARGE"
 *      sopra le variabili statiche): routine di scarica capacità SAB al
 *      boot, avviata da FSM_StartBootDischarge().
 */
void FSM_Tick(void)
{
    /* --- SAB_POWEROFF_GUARD --- */
    if (s_sab_poweroff_pending) {
        if ((int32_t)(HAL_GetTick() - s_sab_poweroff_deadline) >= 0) {
            s_sab_poweroff_pending = false;

            /*
             * SAB_Disable() è già stata chiamata subito da
             * sab_request_shutdown(): qui rimuoviamo solo l'alimentazione,
             * trascorsi almeno SAB_POWEROFF_GUARD_MS dal fronte di discesa
             * di nSAB_EN. Nessun check sullo stato FSM corrente: qui non
             * c'è "destinazione" da rispettare — se il SAB fosse stato
             * riarmato nel frattempo, sab_cancel_pending_shutdown()
             * (chiamata da SAB_Enable() in action_enter_enabled()) avrebbe
             * già azzerato s_sab_poweroff_pending, e non saremmo arrivati
             * qui.
             */
            SAB_ShutdownSupply();
        }
    }

    /* --- SAB_BOOT_DISCHARGE (rimovibile, vedi banner sopra) --- */
    switch (s_boot_discharge_state) {
        case SAB_BOOT_DISCHARGE_POWERUP:
            if ((int32_t)(HAL_GetTick() - s_boot_discharge_deadline) >= 0) {
                /* Alimentazione stabile da almeno SAB_BOOT_DISCHARGE_POWERUP_MS:
                 * ora l'impulso su nSAB_EN è garantito rilevabile. */
                SAB_RawEnPulse(true);   /* nSAB_EN LOW: avvia l'impulso di scarica */
                s_boot_discharge_state    = SAB_BOOT_DISCHARGE_PULSE;
                s_boot_discharge_deadline = HAL_GetTick() + SAB_BOOT_DISCHARGE_PULSE_MS;
            }
            break;

        case SAB_BOOT_DISCHARGE_PULSE:
            if ((int32_t)(HAL_GetTick() - s_boot_discharge_deadline) >= 0) {
                SAB_RawEnPulse(false);   /* nSAB_EN HIGH: fine impulso */
                s_boot_discharge_state    = SAB_BOOT_DISCHARGE_WAIT;
                s_boot_discharge_deadline = HAL_GetTick() + SAB_BOOT_DISCHARGE_WAIT_MS;
            }
            break;

        case SAB_BOOT_DISCHARGE_WAIT:
            if ((int32_t)(HAL_GetTick() - s_boot_discharge_deadline) >= 0) {
                /* Mai armato (SAB_RawEnPulse non tocca SAB_State_t): nessuna
                 * guardia SAB_POWEROFF_GUARD necessaria, spegnimento diretto. */
                SAB_PowerOff();
                s_boot_discharge_state = SAB_BOOT_DISCHARGE_IDLE;
                s_boot_discharge_done  = true;
            }
            break;

        case SAB_BOOT_DISCHARGE_IDLE:
        default:
            break;
    }
}

/* ========================================================================== */
/* --- IMPLEMENTAZIONE DELLE ACTION --- */
/*
 * Convenzione: ogni action si occupa di portare il sistema nello stato
 * di destinazione (attuatori, LED, output EXT). Non legge s_current_state
 * (usa il parametro "from" se serve distinguere la provenienza).
 */
/* ========================================================================== */

/*
 * @brief Abilita i PSU in base alla maschera di configurazione.
 */
static inline void psu_turn_on_masked(void)
{
    if (g_config.psu_enabled_mask & 0x01) PSUTurnOn(PSU1);
    if (g_config.psu_enabled_mask & 0x02) PSUTurnOn(PSU2);
}

static inline void psu_turn_off_all(void)
{
    PSUTurnOff(PSU1);
    PSUTurnOff(PSU2);
}

static inline void contactors_on_masked(void)
{
    if (g_config.contactor_enabled_mask & 0x01) ContactorTurnOn(CONTACTOR1);
    if (g_config.contactor_enabled_mask & 0x02) ContactorTurnOn(CONTACTOR2);
}

static inline void contactors_off_all(void)
{
    ContactorTurnOff(CONTACTOR1);
    ContactorTurnOff(CONTACTOR2);
}

/*
 * Alias locali → BoardCtrl API.
 * Mantengono la leggibilità del codice action senza esporre HAL all'application layer.
 *
 * gate_mc_close(): chiama SEMPRE QCW_Stop() invece di BoardCtrl_GateMC_Close()
 * diretta — QCW_Stop() ferma il timer TIM16 (se l'impulsazione QCW era in
 * corso) e chiude comunque il gate via BoardCtrl_GateMC_Close() al suo
 * interno (no-op sul timer se QCW non era attivo). Garantisce che OGNI
 * uscita da SYS_EMISSION (qualunque causa: comando, errore, fault) fermi
 * l'impulsazione senza dover toccare i punti di chiamata esistenti.
 */
#define gate_mc_close()      QCW_Stop()
#define gate_mc_open()       BoardCtrl_GateMC_Open()
#define gate_hw_enable()     BoardCtrl_GateHW_Enable()
#define gate_hw_disable()    BoardCtrl_GateHW_Disable()

/*
 * @brief Init completato correttamente -> IDLE
 *        Spegne tutto in modo sicuro. LED verde slow fade.
 */
static void action_init_ok(SysState_t from, SysEvent_t ev)
{
    (void)from; (void)ev;

    WaterCooling_Enable();
    psu_turn_off_all();
    contactors_off_all();
    DisableLaseQ();
    gate_mc_close();
    gate_hw_disable();
    BoardCtrl_SetpointSel(false);
    SAB_PowerOff();   /* spegne alimentazione SAB: nSAB_EN HIGH + nSAB_PWR_SHDN LOW */

    SetBoardLED(GREEN);
    SetLEDMode(STATUS_LED, DOUBLE_FADE);

    /*
     * nCMD_RDY_iso NON viene più toccato qui (né da nessun'altra action di
     * ingresso stato): segue SOLO s_mode, aggiornato da sync_cmd_rdy_status()
     * ad ogni cambio modalità (vedi banner "OUTPUT nCMD_RDY_iso" più sopra).
     * READY/ERROR/EMISSION_RDY restano invece puramente funzione dello STATO
     * (tabella richiesta esplicitamente, 2026-07-27), applicati qui per IDLE.
     */
    EXT_SetSysStatus(false);
    EXT_SetError(false);
    EXT_EmissionRdyStatus(false);
}

/*
 * @brief Fault critico non recuperabile.
 *        Spegnimento di emergenza nell'ordine corretto. LED rosso blink.
 */
static void action_fault(SysState_t from, SysEvent_t ev)
{
    (void)from;

    /* Prima di tutto: chiude il gate e disabilita il driver corrente */
    gate_mc_close();
    gate_hw_disable();
    BoardCtrl_SetpointSel(false);
    LaseQSetMode(SW);
    DisableLaseQ();
    LaseQGate(false);

    /* Poi: spegni alimentatori e contattori */
    psu_turn_off_all();
    contactors_off_all();

    /* Disarma SUBITO il SAB; alimentazione rimossa non prima di
     * SAB_POWEROFF_GUARD_MS (vedi banner "GUARDIA 10s" sopra). */
    sab_request_shutdown();

    /*
     * Water cooling: rimane acceso per dissipare calore residuo.
     * Viene spento solo in caso di fault idrico (allagamento).
     */
    if (ev == SYS_FLOOD_FAULT_EVENT) {
        WaterCooling_Emergency();
        WaterCooling_Disable();
    }

    SetBoardLED(RED);
    SetLEDMode(STATUS_LED, TRIPLE_BLINK);
    SetLEDMode(EMISSION_LED, OFF);

    /* nCMD_RDY_iso non toccato qui: segue solo s_mode (sync_cmd_rdy_status()) */
    EXT_SetError(true);
    EXT_SetSysStatus(false);
    EXT_EmissionRdyStatus(false);

    /* Log su SD (sempre) e notifica RS485 (solo se "LOG ERR ON", vedi
     * Rs485Cmd_NotifyFault) — non recuperabile: prefisso "FAULT" */
    s_last_fault_event = ev;
    event_detail_name(ev, s_last_fault_detail, sizeof(s_last_fault_detail));
    SysLog_Event(LOG_ERROR, "FAULT: %s",
                 (s_last_fault_detail[0] != '\0') ? s_last_fault_detail : fault_event_name(ev));
    Rs485Cmd_NotifyFault(ev, false);

    /* FAULT LATCH (dal 2026-07-14): persiste in flash se questo fault deve
     * sopravvivere a un power-cycle — vedi banner sopra e in task_monitor.h. */
    fault_latch_check_and_persist();
}

/*
 * @brief Reset esplicito di SYS_FAULT -> IDLE (comando "FRST", RS485
 *        protetto da login — vedi FSM_RequestFaultReset()). Unica uscita
 *        possibile da SYS_FAULT.
 *
 *        Ripulisce PRIMA il fault latch persistente (g_config.
 *        fault_latch_active, flash) e i flag di debounce interni al monitor
 *        (TaskMonitor_ResetFaultLatches() — flood1/flood2/psu "reported",
 *        countdown taglio eFuse) che altrimenti ri-triggererebbero
 *        immediatamente lo stesso fault al primo ciclo di monitor
 *        successivo, anche se la condizione fisica è nel frattempo
 *        rientrata. Poi riusa la stessa sequenza di spegnimento sicuro di
 *        action_init_ok() per l'ingresso in IDLE (DRY: nessuna duplicazione).
 *
 * @note  NON riabilita un eFuse eventualmente disabilitato per sicurezza
 *        (es. EFUSE_MAIN dopo un taglio da allagamento, vedi
 *        Monitor_CheckFlood()/EFuse_Disable() in task_monitor.c): per
 *        scelta di design richiede sempre un power-cycle fisico, anche
 *        dopo FRST — il comando ripristina solo la macchina a stati (FSM),
 *        non l'alimentazione fisica già tagliata per sicurezza.
 */
static void action_fault_reset(SysState_t from, SysEvent_t ev)
{
    (void)from; (void)ev;

    g_config.fault_latch_active = 0U;
    Config_Save();

    TaskMonitor_ResetFaultLatches();

    /*
     * Comunica a LaseQ il clear dell'errore, stesso meccanismo "singolo
     * messaggio" di action_clear_error() (SYS_ERROR -> SYS_ACTIVE, vedi
     * sopra): mancava qui (bugfix 2026-07-15, segnalato — solo il reset dei
     * flag/latch lato MMC veniva fatto, non l'handshake verso il driver).
     * Anche uscendo da SYS_FAULT (FRST) LaseQ può avere un proprio errore
     * interno latchato (es. residuo di un comm-loss prolungato, o di un
     * error_code mai acquisito durante la disconnessione) che va sbloccato
     * esplicitamente lato driver, non solo azzerato lato MMC.
     */
    LaseQClearErr();

    SysLog_Event(LOG_ERROR, "FRST: fault reset richiesto, ripristino IDLE");

    /* Stessa sequenza di spegnimento sicuro dell'ingresso normale in IDLE */
    action_init_ok(from, ev);
}

/*
 * @brief Transizione verso IDLE.
 *        Water cooling spento, tutto fermo. LED bianco slow blink.
 */
static void action_enter_idle(SysState_t from, SysEvent_t ev)
{
    (void)from; (void)ev;

    gate_mc_close();
    gate_hw_disable();
    BoardCtrl_SetpointSel(false);
    LaseQSetMode(SW);
    DisableLaseQ();
    LaseQGate(false);
    psu_turn_off_all();
    contactors_off_all();
    WaterCooling_Enable();
    /* Disarma SUBITO il SAB; alimentazione rimossa non prima di
     * SAB_POWEROFF_GUARD_MS (vedi banner "GUARDIA 10s" sopra). */
    sab_request_shutdown();

    SetBoardLED(GREEN);
    SetLEDMode(STATUS_LED, DOUBLE_FADE);
    SetLEDMode(EMISSION_LED, OFF);

    /*
     * EXT_SetError(false) aggiunto (2026-07-27): questa action è ora
     * raggiungibile anche da SYS_ERROR (fronte di salita nEXT_CTL_iso,
     * "in qualunque stato ci si trovi serve tornare in IDLE") — nCMD_RDY_iso
     * non viene più toccato qui, segue solo s_mode.
     */
    EXT_SetSysStatus(false);
    EXT_SetError(false);
    EXT_EmissionRdyStatus(false);
    EXT_SetValveStatus(false);
}

/*
 * @brief Applica il passaggio di modalità SW -> ANALOG: aggiorna s_mode,
 *        i toggle HYBRID, il byte LaseQ e nCMD_RDY_iso. NON tocca lo stato
 *        FSM — a carico del chiamante (vedi i vari punti di chiamata: fronte
 *        discendente nEXT_CTL_iso in FSM_ProcessEvent(), comando SET MODE
 *        ANALOG, action_ext_ctrl_on() sotto).
 */
static void apply_mode_analog(void)
{
    s_mode = FSM_MODE_ANALOG;
    reset_hybrid_toggles();   /* i toggle SW-only non hanno senso in ANALOG */
    sync_laseq_mode();        /* analog_mode=1 subito, non solo a SYS_ENABLED */
    sync_cmd_rdy_status();
}

/*
 * @brief Controllo esterno EXT attivato da SYS_IDLE: modalità ANALOG -> ACTIVE.
 *        Come action_enter_active ma imposta anche s_mode = ANALOG.
 *        No-op se già in ANALOG (nessuna conseguenza, come richiesto per un
 *        fronte discendente ridondante).
 */
static void action_ext_ctrl_on(SysState_t from, SysEvent_t ev)
{
    if (s_mode == FSM_MODE_ANALOG) {
        return;
    }
    apply_mode_analog();
    action_enter_active(from, ev);
}

/*
 * @brief Controllo esterno EXT rilasciato: torna a IDLE ripristinando la modalità
 *        di default (SW). Chiamata SOLO quando s_mode era ANALOG (il filtro
 *        di isolamento modale in FSM_ProcessEvent() scarta SYS_EXT_CTRL_OFF_EVENT
 *        quando s_mode è già SW — "nessun effetto se già in software").
 */
static void action_ext_ctrl_off(SysState_t from, SysEvent_t ev)
{
    /* Ritorno a IDLE in modalità SW (default sicuro al rilascio del controllo EXT) */
    s_mode = FSM_MODE_SW;
    reset_hybrid_toggles();   /* riparte da SW "puro", va riarmato esplicitamente */
    sync_laseq_mode();        /* SW/analog_mode=0 subito (action_enter_idle lo rifà comunque) */
    sync_cmd_rdy_status();
    action_enter_idle(from, ev);
}

/*
 * @brief Transizione verso ACTIVE.
 *        Water cooling abilitato, alimentatori ancora spenti.
 *        SAB spento: viene alimentato solo in SYS_ON, abilitato in SYS_ENABLED.
 *        LED verde fast blink.
 *
 * @note  SAB_ON_EXIT_PULSE (rimovibile, vedi banner sopra le variabili
 *        statiche/prototipi action): se from == SYS_ON, genera manualmente
 *        l'impulso di scarica capacità su nSAB_EN, perché in quel percorso
 *        sab_request_shutdown() non produce alcun fronte (SAB era
 *        alimentato ma mai armato).
 */
static void action_enter_active(SysState_t from, SysEvent_t ev)
{
    (void)ev;

    /* Assicura che laser, gate e alimentatori siano spenti */
    gate_mc_close();
    DisableLaseQ();
    LaseQGate(false);
    psu_turn_off_all();
    contactors_off_all();

    /* Avvia il water cooling e resetta il timer di inibizione flusso */
    WaterCooling_Enable();
    TaskMonitor_ResetFlowInhibit();

    /* SAB: disarma SUBITO (verrà rialimentato in action_enter_on); la
     * rimozione effettiva dell'alimentazione è posticipata di almeno
     * SAB_POWEROFF_GUARD_MS (vedi banner "GUARDIA 10s" sopra). */
    sab_request_shutdown();

    /* SAB_ON_EXIT_PULSE (rimovibile, vedi banner sopra): da SYS_ON, SAB era
     * alimentato ma mai armato — sab_request_shutdown() sopra non genera
     * alcun fronte su nSAB_EN (era già HIGH). PSU già spenti (sopra):
     * inneschiamo qui manualmente la scarica capacità, subito. */
    if (from == SYS_ON) {
        SAB_RawEnPulse(true);            /* nSAB_EN LOW */
        osDelay(SAB_ON_EXIT_PULSE_MS);
        SAB_RawEnPulse(false);           /* nSAB_EN HIGH: fronte di salita, innesca la scarica */
    }

    SetBoardLED(GREEN);
    SetLEDMode(STATUS_LED, SLOW_FADE);
    SetLEDMode(EMISSION_LED, OFF);

    /*
     * nCMD_RDY_iso NON viene più asserito qui (revisione 2026-07-27): non è
     * più legato allo stato ACTIVE, segue SOLO s_mode tramite
     * sync_cmd_rdy_status() (vedi apply_mode_analog()/action_ext_ctrl_off()/
     * blocco SET MODE in FSM_ProcessEvent()). nREADY_iso resta deasserito
     * (HIGH, sys_ready=false) fino a SYS_ON, dove action_enter_on() lo asserisce.
     */
    EXT_SetSysStatus(false);
    EXT_SetError(false);
    EXT_EmissionRdyStatus(false);
    EXT_SetValveStatus(true);
}

/*
 * @brief Transizione verso ON.
 *        Contattori chiusi → ritardo → PSU accesi. Driver disabilitato.
 *        Alimenta il modulo SAB (nSAB_PWR_SHDN HIGH): l'armo avverrà
 *        in action_enter_enabled() prima di passare a SYS_ENABLED.
 *
 * @note  Usa osDelay() perché questa action è sempre eseguita nel contesto
 *        del task_fsm, mai da ISR.
 */
static void action_enter_on(SysState_t from, SysEvent_t ev)
{
    (void)ev;

    /*
     * Verifica chiavi: contatti A/B indipendenti dal modulo SAB (pin
     * separati, vedi Key.h vs SAB.h) — NON vanno gated su g_config.sab_enabled,
     * che riguarda solo la presenza del modulo di sicurezza analogico, non
     * dei contatti chiave. Ogni contatto è richiesto solo se il suo bit
     * (ERR_BIT_KEY_A/ERR_BIT_KEY_B) è abilitato in error_mask — permette di
     * mascherare il contatto non montato in produzione (spesso se ne monta
     * uno solo) o entrambi per debug (vedi Key_InterlockSatisfied()).
     * Soluzione architetturale: Inputs_CheckKeys() (task_inputs.c) rileva
     * solo la RIMOZIONE mentre il sistema è già armato (fronte true→false)
     * — se la chiave è già aperta PRIMA di questa transizione non c'è
     * fronte da rilevare. Per questo qui non basta un return silenzioso:
     * la FSM aggiorna comunque s_current_state = SYS_ON subito dopo questa
     * action (invariante architetturale, vedi FSM_ProcessEvent), quindi
     * senza postare un evento resteremmo bloccati in un "falso ON" — stato
     * ON ma tutto spento, nessun fault visibile. Postiamo esplicitamente
     * SYS_KEY_REMOVED_EVENT (stesso schema già usato per il timeout SAB in
     * action_enter_enabled()): la FSM lo processa subito dopo essere
     * entrata in ON e transita in SYS_ERROR.
     */
    if (!Key_InterlockSatisfied()) {
        Queue_PostEvent(SYS_KEY_REMOVED_EVENT);
        /* Chiave(i) richiesta(e) mancante(i): non accendere nulla */
        return;
    }

    /*
     * Verifica coperchi (dal 2026-07-15): stesso identico schema del check
     * chiavi sopra, con Lid_InterlockSatisfied() (Drivers/Lid/Lid.h) —
     * ogni sensore è richiesto solo se il suo bit (ERR_BIT_LID1/ERR_BIT_LID2)
     * è abilitato in error_mask. Se un coperchio non mascherato è aperto,
     * l'ingresso in SYS_ON viene bloccato allo stesso modo di una chiave
     * mancante.
     */
    if (!Lid_InterlockSatisfied()) {
        Queue_PostEvent(SYS_LID_OPEN_EVENT);
        /* Coperchio/i richiesto/i aperto/i: non accendere nulla */
        return;
    }

    /*
     * BUGFIX (2026-07-16): driver LaseQ disabilitato PRIMA di toccare il SAB.
     *
     * Bug osservato: da SYS_ENABLED/SYS_EMISSION, SDIS (SYS_DISABLE_EVENT)
     * porta qui e SAB_Disable() (poco sotto) produce un VERO fronte su
     * nSAB_EN (il SAB era armato), che apre l'interlock verso LaseQ. LaseQ
     * si aspetta di ricevere enable=0 da MMC entro una finestra di grazia
     * breve (100ms, originariamente 50ms) dall'apertura dell'interlock, pena
     * un proprio errore interno (interlock aperto con enable ancora attivo
     * dal suo punto di vista). Prima di questo fix, DisableLaseQ()/
     * LaseQGate(false) venivano chiamate solo in fondo a questa funzione,
     * DOPO l'intera sequenza di riaccensione contattori/PSU (compreso
     * osDelay(g_config.contactor_psu_delay_ms)) — un ritardo che eccede
     * sistematicamente la finestra di grazia, facendo andare in errore
     * LaseQ e trascinando anche MMC (SYS_LASEQ_ERROR/FAULT_EVENT).
     * La transizione gemella verso SYS_ACTIVE (SOFF, action_enter_active())
     * non soffriva del problema perché lì DisableLaseQ() è già la seconda
     * istruzione della action, ben prima di sab_request_shutdown().
     * Fix: stesso ordine di action_enter_active() — disabilitare LaseQ
     * SUBITO, prima di qualunque intervento sul SAB. Nessun effetto
     * collaterale per gli altri "from" (ACTIVE/IDLE): il driver deve
     * comunque risultare disabilitato in SYS_ON a prescindere dallo stato
     * di provenienza.
     */
    gate_mc_close();
    gate_hw_disable();
    BoardCtrl_SetpointSel(false);
    DisableLaseQ();
    LaseQGate(false);

    /*
     * SAB: disabilita prima (nSAB_EN HIGH, in caso venissimo da ENABLED),
     * poi alimenta il modulo (nSAB_PWR_SHDN HIGH).
     * L'armo (nSAB_EN LOW + guard + monitoring) avviene in action_enter_enabled().
     */
    if (g_config.sab_enabled) {

        /*
         * SAB_REARM_BLOCK (rimovibile, vedi banner sopra le variabili
         * statiche/prototipi action). Solo da SYS_ENABLED/SYS_EMISSION il
         * SAB_Disable() qui sotto produce un VERO fronte di salita su
         * nSAB_EN (era armato): innesca il ciclo hardware di scarica
         * capacità (~10s). SAB resta alimentato (SAB_PowerOn() sotto):
         * senza questa guardia si potrebbe rientrare in SYS_ENABLED prima
         * che la scarica sia completa.
         */
        if (from == SYS_ENABLED || from == SYS_EMISSION) {
            s_sab_rearm_block_active   = true;
            s_sab_rearm_block_deadline = HAL_GetTick() + SAB_REARM_BLOCK_MS;
        }

        SAB_Disable();

        /*
         * Annulla un eventuale spegnimento SAB già pianificato da
         * sab_request_shutdown() (es. quello armato da action_enter_active()
         * quando si è passati per SYS_ACTIVE prima di questo ingresso in
         * SYS_ON — sab_request_shutdown() viene chiamata incondizionatamente
         * lì, anche se il SAB non era acceso). Senza questa cancellazione,
         * SAB_PowerOn() sotto ridà alimentazione al modulo ma il timer
         * pianificato allora continua a correre: se scade DOPO questo punto,
         * FSM_Tick() chiama SAB_ShutdownSupply() e toglie l'alimentazione
         * appena ridata — bug osservato: SAB si accende per un attimo in
         * SYS_ON e si rispegne subito dopo. Stesso identico problema (e
         * stessa soluzione) già gestito in action_enter_enabled() prima di
         * SAB_Enable().
         */
        sab_cancel_pending_shutdown();

        SAB_PowerOn();
    }

    /* Accensione nell'ordine corretto: contattori → ritardo → PSU */
    contactors_on_masked();

    if (g_config.contactor_psu_delay_ms > 0) {
        osDelay(g_config.contactor_psu_delay_ms);
    }

    PSUVoltageSet(g_config.psu_voltage_mv);
    PSUCurrentSet(g_config.psu_current_ma);
    psu_turn_on_masked();

    /*
     * Avvia il timer di inibizione del check DC_OK nel monitor.
     * L'alimentatore ha bisogno di psu_dc_ok_delay_ms per raggiungere
     * tensione stabile e asserire DC_OK. Senza questa chiamata il monitor
     * segnalerebbe un fault PSU immediatamente dopo l'accensione.
     */
    TaskMonitor_ResetPSUInhibit();

    /* Driver corrente, gate e setpoint HW: già disabilitati subito dopo i
     * check chiave/coperchio (vedi BUGFIX 2026-07-16 sopra) — non ripetere
     * qui. */

    SetBoardLED(YELLOW);
    SetLEDMode(STATUS_LED, STATIC);
    SetLEDMode(EMISSION_LED, OFF);

    /*
     * BUGFIX (segnalato 2026-07-27): qui veniva richiamata per errore
     * EXT_SetCmdStatus(true) (già asserito dall'ingresso in ACTIVE, vedi
     * action_enter_active() — chiamata ridondante) invece di
     * EXT_SetSysStatus(true). nREADY_iso deve asserirsi (LOW) proprio qui,
     * all'ingresso in SYS_ON: prima di questo punto contattori/PSU non sono
     * ancora accesi, il sistema non è "ready" nel senso del segnale.
     * nCMD_RDY_iso non toccato qui: segue solo s_mode (sync_cmd_rdy_status()).
     */
    EXT_SetSysStatus(true);
    EXT_SetError(false);
    EXT_EmissionRdyStatus(false);
}

/* ERROR_STATE — Lase-Q4_v2.1/App/FSM/FSM.h, FSM_State_t (INIT=0, IDLE=1,
 * ENABLED=2, ERROR_STATE=3, ...). MMC non condivide un header con LaseQ per
 * questo enum: il valore va tenuto sincronizzato manualmente con quel file. */
#define LASEQ_FSM_STATE_ERROR   3U

/*
 * @brief  Verifica che LaseQ non sia bloccato nel proprio ERROR_STATE prima
 *         di riabilitarlo (chiamata da action_enter_enabled(), SYS_ON ->
 *         SYS_ENABLED). Se lo è, invia clear_error e attende conferma con lo
 *         stesso schema bloccante di action_clear_error() (vedi sotto),
 *         prima di lasciar procedere con EnableLaseQ().
 *
 * BUGFIX (richiesta esplicita, "consistenza degli stati" — Luca): il bit 7
 * di error_code (FSM_FAULT_HW) è notificato da task_comms.c solo quando
 * FSM_GetState() >= SYS_ON (per evitare falsi positivi da PWR_OK basso
 * durante SYS_ACTIVE, dove i PSU sono spenti — vedi banner "BUGFIX PWR_OK"
 * in task_comms.c). Se MMC scende sotto SYS_ON (es. un SOFF) proprio mentre
 * LaseQ finisce nel proprio ERROR_STATE, MMC non se ne accorge mai: nessun
 * SYS_LASEQ_ERROR_EVENT viene postato, lo stato di MMC resta "pulito" anche
 * se LaseQ è bloccato in errore. Un successivo SEN chiamava EnableLaseQ()
 * alla cieca: RS485 manda enable=1, ma rs485_handler.c (LaseQ) lo ignora
 * silenziosamente se il proprio stato non è IDLE — "SEN" tornava comunque
 * OK, MMC risultava in SYS_ENABLED, ma LaseQ restava fermo in ERROR_STATE,
 * driver mai realmente attivo. Questo check chiude il buco alla fonte, nel
 * punto in cui MMC sta per fidarsi di un enable riuscito.
 *
 * @retval true   LaseQ non era in errore, oppure lo era ma si è ripreso
 *                entro il timeout: sicuro procedere con EnableLaseQ().
 * @retval false  LaseQ resta in ERROR_STATE: NON procedere.
 */
static bool laseq_ensure_not_in_error(void)
{
    LaseQ_status_vars_t st = GetLaseQStatus();

    if (st.fsm_state != LASEQ_FSM_STATE_ERROR) {
        return true;   /* nessun problema noto */
    }

    SysLog_Event(LOG_WARN,
                 "SEN: LaseQ risulta in ERROR_STATE (fsm_state=%u) - invio clear_error prima di riabilitare",
                 (unsigned)st.fsm_state);

    LaseQClearErr();

    const uint32_t POLL_MS  = 20U;
    const uint32_t MAX_WAIT = 500U;
    uint32_t        elapsed = 0U;

    do {
        osDelay(POLL_MS);
        elapsed += POLL_MS;
        st = GetLaseQStatus();
    } while (st.fsm_state == LASEQ_FSM_STATE_ERROR && elapsed < MAX_WAIT);

    if (st.fsm_state == LASEQ_FSM_STATE_ERROR) {
        char errname[96];
        FSM_FormatErrorCode(st.error_code, errname, sizeof(errname));
        SysLog_Event(LOG_ERROR,
                     "SEN rifiutato: LaseQ ancora in ERROR_STATE dopo %ums (0x%02X: %s)",
                     (unsigned)elapsed, st.error_code, errname);
        return false;
    }

    return true;
}

/*
 * @brief Transizione verso ENABLED.
 *        Prima di abilitare il driver corrente, completa l'armo del SAB:
 *          1. SAB_Enable() → nSAB_EN LOW → guard 1s → monitoring INTLCK+TEST
 *          2. Attesa bloccante (polling SAB_GetState) fino ad armo o fault.
 *             Totale max: SAB_GUARD_DELAY_MS + timeout_ms + margine ≈ 4s.
 *          3. Se SAB in FAULT: la callback ha già postato l'evento di errore
 *             alla coda FSM; usciamo (la FSM transiterà in ERROR al prossimo tick).
 *          4. Se SAB ARMED: verifica che LaseQ non sia bloccato nel proprio
 *             ERROR_STATE (laseq_ensure_not_in_error(), vedi sopra) prima di
 *             procedere con EnableLaseQ() e setup normale.
 *        L'attesa bloccante è legittima: questa action gira nel task_fsm e
 *        usa osDelay() (come action_enter_on). La FSM non processa altri
 *        eventi durante il blocco, il che è sicuro perché siamo in transizione.
 *
 *        In modalità ANALOG: nGATE_HW_EN controllato dall'esterno,
 *        nGATE_MC rimane LOW (gate software chiuso, non necessario agire).
 */
static void action_enter_enabled(SysState_t from, SysEvent_t ev)
{
    (void)from; (void)ev;

    /* ------------------------------------------------------------------ */
    /* Armo SAB (se abilitato)                                             */
    /* ------------------------------------------------------------------ */
    if (g_config.sab_enabled) {
        /* Annulla un eventuale spegnimento posticipato ancora pendente
         * (sab_request_shutdown(), vedi banner "GUARDIA 10s" più sopra):
         * il SAB sta per essere riarmato, FSM_Tick() non deve più tagliargli
         * l'alimentazione da sotto. */
        sab_cancel_pending_shutdown();
        SAB_Enable();

        /*
         * Polling bloccante: max SAB_GUARD_DELAY_MS + timeout + 500ms margine.
         * Il timeout interlock (s_configured_timeout) è passato a SAB_Init()
         * da freertos.c come g_config.sab_interlock_timeout_ms (tipico: 2000ms).
         * Il driver SAB chiama la callback (sab_event_cb) se va in FAULT,
         * postando l'evento di errore in coda. Qui usciamo subito appena
         * rileviamo FAULT, senza postare duplicati.
         */
        const uint32_t POLL_MS    = 20U;
        const uint32_t MAX_WAIT   = SAB_GUARD_DELAY_MS + 500U;   /* ~1.5s */
        uint32_t       elapsed    = 0U;
        bool           sab_armed  = false;

        /*
         * Check stato PRIMA del delay: se SAB è già ARMED (es. ritorno da
         * EMISSION→ENABLED dove era già armato) usciamo immediatamente senza
         * attese inutili. Se DISABLED (SAB_Enable() no-op per stato != DISABLED),
         * questo non accade in pratica (SAB_Enable è già stato chiamato sopra).
         */
        while (elapsed < MAX_WAIT) {
            SAB_State_t st = SAB_GetState();
            if (st == SAB_STATE_ARMED) {
                sab_armed = true;
                break;
            }
            if (st == SAB_STATE_FAULT) {
                /* Driver già ha postato l'evento di fault via callback */
                break;
            }
            osDelay(POLL_MS);
            elapsed += POLL_MS;
        }

        if (!sab_armed) {
            /*
             * SAB non armato (fault o timeout FSM).
             * Se il driver non è già in FAULT (es. elapsed senza che il driver
             * abbia rilevato il fault autonomamente), postiamo un timeout —
             * solo se ERR_BIT_SAB_TIMEOUT non è mascherato (stesso bit del
             * post via callback in sab_event_cb(), freertos.c).
             *
             * BUGFIX (Luca, 2026-07-16): mancava un return incondizionato
             * qui. Se SAB_GetState() era già SAB_STATE_FAULT (il driver ha
             * già postato l'evento via callback), il ramo sopra non veniva
             * eseguito e — SENZA return — l'esecuzione proseguiva dritta a
             * EnableLaseQ() qualche riga più sotto: LaseQ veniva abilitato
             * "alla cieca" con l'interlock fisico ancora aperto (SAB mai
             * armato), falliva a sua volta il proprio check locale
             * (nENABLE-clocked, guardia 3-5ms in state_enabled(), FSM.c) e
             * si LATCHAVA nel proprio ERROR_STATE (error_code 0x80).
             * Da quel momento ogni "GET LQ" mostrava solo ERR:0x80, perché
             * LaseQ ririporta l'errore latched ad ogni ciclo RS485 (ogni
             * 20ms), mentre SYS_SAB_TIMEOUT_EVENT era un post singolo,
             * facilmente "sepolto" dietro il rumore continuo del LaseQ
             * error — da cui l'impressione che l'errore fosse "sempre e
             * solo LaseQ", mascherando la vera causa scatenante (interlock
             * SAB mai chiuso). Con il return, EnableLaseQ() non viene MAI
             * raggiunta se SAB non è armato, indipendentemente da
             * error_mask (che condiziona solo la notifica dell'evento, mai
             * la sicurezza dell'abilitazione).
             */
            if (SAB_GetState() != SAB_STATE_FAULT &&
                (g_config.error_mask & ERR_BIT_SAB_TIMEOUT)) {
                Queue_PostEvent(SYS_SAB_TIMEOUT_EVENT);
            }
            return;
        }
    }

    /* ------------------------------------------------------------------ */
    /* SAB armato (o non configurato): verifica LaseQ, poi abilita driver  */
    /* ------------------------------------------------------------------ */
    if (!laseq_ensure_not_in_error()) {
        Queue_PostEvent(SYS_LASEQ_ERROR_EVENT);
        return;
    }

    EnableLaseQ();

    switch (s_mode) {
        case FSM_MODE_ANALOG:
            /*
             * LaseQ legge il setpoint dall'ingresso analogico (ANALOG mode).
             * HW_SETPOINT_SEL seleziona la sorgente di quel segnale:
             *   0 = LPWR_SET_ISO grezzo (utente EXT) — default fabbrica
             *   1 = DAC di AMC (segnale corretto/linearizzato)
             * La scelta è in s_hw_setpoint_sel (letto da g_config in FSM_Init).
             * nGATE_HW_EN = 0: abilita il path HW del gate; è l'EXT interface
             * a guidare gate (nGATE_IN_iso) — la FSM non lo controlla.
             *
             * LaseQSetMode() è già stata applicata "live" da sync_laseq_mode()
             * quando s_mode è diventato ANALOG (vedi action_ext_ctrl_on()):
             * la richiamiamo comunque qui come rete di sicurezza ridondante,
             * innocua perché idempotente (stesso valore).
             */
            sync_laseq_mode();
            LaseQGate(false);
            gate_mc_close();
            BoardCtrl_SetpointSel(s_hw_setpoint_sel != 0U);
            gate_hw_enable();               /* nGATE_HW_EN=0: path HW gate attivo */
            SetLEDMode(EMISSION_LED, FAST_FADE);
            SetBoardLED(LIGHT_BLUE);
            break;

        case FSM_MODE_SW:
        default:
            /*
             * Gate e setpoint interamente sotto controllo firmware (RS485),
             * salvo i due toggle indipendenti "HYBRID1"/"HYBRID2" (vedi
             * s_gate_hw_enabled/s_setpoint_hw_enabled e banner "MODALITÀ
             * OPERATIVA" in fsm.h) — entrambi false di default (SW puro).
             *
             * "HYBRID2" (s_setpoint_hw_enabled): LaseQ riceve analog_mode=1
             * (setpoint di potenza valido = quello hardware) e
             * BoardCtrl_SetpointSel segue la stessa semantica HW_SETPOINT_SEL
             * di ANALOG mode. Il gate resta comunque sotto controllo SW
             * (nGATE_MC): la FSM apre/chiude il gate su
             * SYS_EMISSION_REQUEST_EVENT / SYS_LASEROFF_EVENT come in SW
             * puro — nessuna differenza di comportamento FSM.
             *
             * "HYBRID1" (s_gate_hw_enabled): nGATE_HW_EN abilitato, ma i
             * byte di controllo LaseQ restano quelli di SW puro
             * (sw_control=1, analog_mode=0 se HYBRID2 non è anche attivo).
             *
             * Il setpoint SW (sw_current_setpoint) viene comunque inviato a
             * LaseQ via RS485 anche con HYBRID2 attivo (LaseQ lo ignora
             * quando analog_mode=1) — invariato rispetto a SW puro, per
             * mantenere il comportamento FSM di MMC identico in ogni caso.
             *
             * LaseQSetMode() è già stata applicata "live" da sync_laseq_mode()
             * quando s_setpoint_hw_enabled è cambiato (vedi il blocco
             * SET SETPOINTHW in FSM_ProcessEvent()): richiamata qui come rete
             * di sicurezza ridondante, innocua perché idempotente.
             */
            sync_laseq_mode();
            LaseQGate(false);
            gate_mc_close();
            BoardCtrl_SetpointSel(s_setpoint_hw_enabled && (s_hw_setpoint_sel != 0U));
            if (s_gate_hw_enabled) {
                gate_hw_enable();            /* "HYBRID1": nGATE_HW_EN=0 */
            } else {
                gate_hw_disable();           /* SW puro: nGATE_HW_EN=1 */
            }
            SetLEDMode(EMISSION_LED, STATIC);
            SetBoardLED(WHITE);
            break;
    }

    SetLEDMode(STATUS_LED, STATIC);
    /*
     * READY/ERROR/EMISSION_RDY esplicitati per intero anche qui (2026-07-27):
     * in precedenza sys_ready=true "ereditava" da SYS_ON senza essere
     * riscritto — reso esplicito per rendere ogni action autosufficiente,
     * indipendentemente dallo stato di provenienza. nCMD_RDY_iso non toccato:
     * segue solo s_mode.
     */
    EXT_SetSysStatus(true);
    EXT_SetError(false);
    EXT_EmissionRdyStatus(true);
}

/*
 * @brief Transizione verso EMISSION.
 *        Gate aperto → emissione laser attiva.
 *        nGATE_MC aperto (LOW → laser acceso, vedi BoardCtrl.c).
 *
 *        CW (default) vs QCW: in FSM_MODE_SW, se g_config.qcw_enabled è
 *        attivo, nGATE_MC non viene aperto staticamente ma impulsato da
 *        QCW_Start() (TIM16 + frequenza/duty da g_config.qcw_freq_hz /
 *        qcw_duty_pct, vedi Drivers/QCW/QCW.h e "SET FREQ"/"SET DUTY"/
 *        "SET QCW" in rs485_cmd.c). In FSM_MODE_ANALOG (o QCW disattivo)
 *        comportamento invariato: gate aperto staticamente.
 */
static void action_enter_emission(SysState_t from, SysEvent_t ev)
{
    (void)from; (void)ev;

    LaseQGate(true);

    if (s_mode == FSM_MODE_SW && g_config.qcw_enabled) {
        QCW_SetParams(g_config.qcw_freq_hz, g_config.qcw_duty_pct);
        QCW_Start();          /* impulsa nGATE_MC via TIM16 (QCW) */
    } else {
        gate_mc_open();       /* nGATE_MC aperto staticamente (CW) = emissione abilitata */
    }

    SetBoardLED(PURPLE);
    SetLEDMode(STATUS_LED, STATIC);
    SetLEDMode(EMISSION_LED, SLOW_FADE);

    /*
     * READY/ERROR/EMISSION_RDY esplicitati (2026-07-27): stessi valori di
     * SYS_ENABLED (già garantiti da action_enter_enabled(), da cui si
     * arriva sempre) — ripetuti qui per rendere l'action autosufficiente.
     * nCMD_RDY_iso non toccato: segue solo s_mode.
     */
    EXT_SetSysStatus(true);
    EXT_SetError(false);
    EXT_EmissionRdyStatus(true);
}

/*
 * @brief Transizione verso ERROR (errore recuperabile).
 *        Gate chiuso, emissione disabilitata, PSU e contattori spenti.
 *        LED rosso slow blink.
 *
 * BUGFIX (richiesta esplicita): prima di questo fix PSU e contattori
 * restavano accesi in SYS_ERROR (spenti solo più avanti, dentro
 * action_clear_error(), cioè solo se/quando l'operatore faceva CERR — nel
 * frattempo, per tutta la durata dell'errore, PSU e contattori restavano
 * inutilmente sotto tensione). Ora vengono spenti SUBITO all'ingresso in
 * SYS_ERROR, stesso ordine già usato da action_fault() per SYS_FAULT
 * (gate/driver prima, poi PSU/contattori, poi la richiesta di spegnimento
 * SAB). action_clear_error() continua comunque a richiamarli: idempotente,
 * nessun problema se sono già spenti da qui.
 */
static void action_enter_error(SysState_t from, SysEvent_t ev)
{
    (void)from;

    /* Apre il gate del latch errori (2026-07-31, bugfix "GET ERR sempre
     * FLOW=1 dopo CERR") — vedi banner su s_latched_errors/
     * s_error_latch_active in task_monitor.c. Va fatto ad ogni ingresso
     * reale in SYS_ERROR, prima di qualunque altra cosa in questa action. */
    TaskMonitor_BeginErrorLatch();

    gate_mc_close();
    gate_hw_disable();
    BoardCtrl_SetpointSel(false);
    LaseQSetMode(SW);
    LaseQGate(false);
    DisableLaseQ();

    /* PSU e contattori spenti subito: in ERROR non siamo più in condizione
     * operativa, nessun motivo di tenerli sotto tensione fino a un
     * eventuale CERR (che può non arrivare mai). */
    psu_turn_off_all();
    contactors_off_all();

    /* Disarma SUBITO il SAB: in ERROR non siamo più in condizione operativa.
     * Alimentazione rimossa non prima di SAB_POWEROFF_GUARD_MS (vedi
     * banner "GUARDIA 10s" sopra). */
    sab_request_shutdown();

    /*
     * Errore idrico specifico (flusso fuori range): stessa risposta di
     * sicurezza sul circuito di raffreddamento usata in action_fault,
     * anche se qui l'errore resta recuperabile via CERR (non è un fault
     * bloccante). La condizione fisica del circuito idrico non cambia
     * solo perché l'errore è ora clearable.
     */
    if (ev == SYS_FLOW_ERROR_EVENT) {
        WaterCooling_Emergency();

        /*
         * Seed diretto nel latch (2026-07-31, bugfix "GET ERR non mostra più
         * FLOW dopo un errore di flusso, solo interlock"): WaterCooling_Emergency()
         * appena chiamata sopra porta WaterCooling_GetState() a WC_STATE_ERROR,
         * il che fa sì che Monitor_CheckFlow() (task_monitor.c) prenda da
         * SUBITO il suo ramo "not running" — che ora (bugfix precedente)
         * azzera correttamente s_active_errors's ERR_BIT_FLOW. Questo accade
         * PRIMA che il passo di OR-in in TaskMonitor_Run() abbia la
         * possibilità di latchare il bit appena impostato dal check che ha
         * rilevato il guasto originale: seedare s_active_errors qui (come
         * per SAB_TIMEOUT sotto) non basterebbe, verrebbe comunque ripulito
         * in tempo dallo stesso ciclo. Scrivendo direttamente nel latch si
         * bypassa la corsa: vedi banner completo in TaskMonitor_SeedLatchedError()
         * (task_monitor.c).
         */
        TaskMonitor_SeedLatchedError(ERR_BIT_FLOW);
    }

    /*
     * ERR_BIT_SAB_TIMEOUT (2026-07-31, bugfix): a differenza degli interlock
     * A/B (ora rispecchiati in continuo da Monitor_CheckSAB(), task_monitor.c
     * — stato fisico persistente, leggibile in ogni momento via
     * SAB_GetInterlockStatus()) il timeout di chiusura SAB è un evento
     * puntuale, non uno stato: non esiste un pin/flag "timeout in corso" da
     * interrogare più tardi, l'unica occasione per registrarlo è ORA, alla
     * transizione stessa. Senza questo seed "GET ERR" restava a 0x0 per
     * questa causa esattamente come per gli interlock prima del fix — anche
     * se "GET ALARM" (FSM_GetLastErrorName(), sotto) la mostrava già
     * correttamente. Ripulito da TaskMonitor_ClearLatchedErrors() all'uscita
     * reale da SYS_ERROR (action_clear_error() sotto).
     */
    if (ev == SYS_SAB_TIMEOUT_EVENT) {
        TaskMonitor_SetErrorBit(ERR_BIT_SAB_TIMEOUT, true);
    }

    /*
     * Seed diretto INTLCK/TEST (2026-07-31, bugfix — stessa corsa di
     * ERR_BIT_FLOW sopra, ma per una causa SAB genuina): sab_request_shutdown()
     * poco più sopra chiama SAB_Disable(), che porta SAB_GetState() a
     * SAB_STATE_DISABLED SUBITO, per QUALSIASI causa di SYS_ERROR (non solo
     * SAB). Monitor_CheckSAB() (task_monitor.c) ora ignora deliberatamente i
     * canali quando lo stato è DISABLED/GUARD (bugfix "interlock compare in
     * GET ERR anche in SYS_ON con SAB non armato") — ma questo vorrebbe dire
     * che, anche per un errore REALMENTE causato da interlock aperto o test
     * fallito, il check continuo smette di scrivere il bit nello stesso ciclo
     * in cui dovrebbe essere latchato, esattamente come per FLOW/WaterCooling_
     * Emergency() sopra. SAB_GetInterlockStatus()/SAB_GetTestStatus() restano
     * comunque accurate qui (SAB_Disable() non tocca la cache s_intlck_a/b_open
     * né i pin TEST, letti raw — vedi SAB.c): si seeda quindi direttamente il
     * latch con il canale realmente responsabile, letto ORA, prima che
     * sab_request_shutdown() renda muto il check continuo.
     */
    if (ev == SYS_SAB_INTERLOCK_OPEN_EVENT) {
        bool a_open = false, b_open = false;
        SAB_GetInterlockStatus(&a_open, &b_open);
        if (a_open) { TaskMonitor_SeedLatchedError(ERR_BIT_SAB_INTLCK_A); }
        if (b_open) { TaskMonitor_SeedLatchedError(ERR_BIT_SAB_INTLCK_B); }
    }
    if (ev == SYS_SAB_TEST_FAIL_EVENT) {
        bool a_fault = false, b_fault = false;
        SAB_GetTestStatus(&a_fault, &b_fault);
        if (a_fault) { TaskMonitor_SeedLatchedError(ERR_BIT_SAB_TEST_A); }
        if (b_fault) { TaskMonitor_SeedLatchedError(ERR_BIT_SAB_TEST_B); }
    }

    SetBoardLED(RED);
    SetLEDMode(STATUS_LED, FAST_BLINK);
    SetLEDMode(EMISSION_LED, OFF);

    /*
     * NUOVO (richiesto esplicitamente, 2026-07-27): nREADY_iso deve tornare
     * deasserito (HIGH, sys_ready=false) in SYS_ERROR, indipendentemente
     * da come si è arrivati qui (anche da SYS_ON/ENABLED/EMISSION, dove
     * era asserito) — coerente con la tabella "IDLE/ACTIVE" (non ready).
     * nCMD_RDY_iso non toccato: segue solo s_mode.
     */
    EXT_SetSysStatus(false);
    EXT_SetError(true);
    EXT_EmissionRdyStatus(false);

    /*
     * Log su SD (sempre) e notifica RS485 (solo se "LOG ERR ON") per
     * QUALSIASI causa di SYS_ERROR — standardizzato: prima solo FLOW/TEMP
     * notificavano, le altre cause (SAB, chiave rimossa, CLR_ERR manuale)
     * restavano silenziose. Recuperabile via CERR: prefisso "ALARM".
     */
    s_last_error_event = ev;
    event_detail_name(ev, s_last_error_detail, sizeof(s_last_error_detail));
    SysLog_Event(LOG_ERROR, "ALARM: %s",
                 (s_last_error_detail[0] != '\0') ? s_last_error_detail : fault_event_name(ev));
    Rs485Cmd_NotifyFault(ev, true);
}

/*
 * @brief Clear errore recuperabile → torna ad ACTIVE, MA SOLO SE LaseQ ha
 *        realmente confermato la ripresa (GET LQ error_code == 0).
 *
 * BUGFIX (richiesta esplicita, vedi anche banner "SEVERITÀ" in fsm.h e
 * TaskComms_Run() in task_comms.c): PRIMA di questo fix, CERR spediva
 * clear_error a LaseQ (fire-and-forget, LaseQClearErr()) e la FSM tornava
 * SEMPRE ad ACTIVE indipendentemente dall'esito reale — FSM_ProcessEvent()
 * imposta incondizionatamente s_current_state = t->next_state subito dopo
 * questa action. Se LaseQ non usciva davvero dal proprio errore interno
 * (es. interlock ancora aperto lato LaseQ, GET LQ → ERR:0x80/FSM_FAULT_HW),
 * il sistema tornava comunque operativo: da ACTIVE l'operatore poteva
 * proseguire fino a SYS_ON/SYS_ENABLED con LaseQ palesemente ancora in
 * errore, perché il relativo evento in TaskComms_Run() (SYS_LASEQ_ERROR_EVENT)
 * restava "già postato" (lq_hw_err_posted/lq_err_posted) dalla prima
 * occorrenza e non veniva mai ri-notificato finché l'errore non si fosse
 * davvero risolto lato LaseQ.
 *
 * Ora: invia clear_error, poi ATTENDE (polling bloccante nel task FSM,
 * stesso pattern del polling SAB in action_enter_enabled()) che
 * TaskMonitor_GetLaseQErrorCode() (aggiornato da task_comms.c ad ogni
 * transazione RS485 riuscita, ogni LQ_TRANSMIT_WINDOW=20ms) confermi
 * error_code == 0 prima di dichiarare l'errore risolto. Se il timeout scade
 * con error_code ancora diverso da zero, il sistema NON deve riabilitarsi:
 * si posta SYS_LASEQ_ERROR_EVENT, che la tabella di transizione instrada
 * su SYS_ERROR anche da SYS_ACTIVE (vedi state_machine[SYS_ACTIVE]
 * [SYS_LASEQ_ERROR_EVENT]) — FSM_ProcessEvent() porterà comunque lo stato
 * a SYS_ACTIVE per questa singola transizione (stesso schema già usato da
 * action_enter_on()/action_enter_enabled() per i loro check equivalenti),
 * ma l'evento appena accodato lo riporta immediatamente in SYS_ERROR al
 * giro successivo della coda FSM, senza mai attraversare SYS_ON/SYS_ENABLED.
 */
static void action_clear_error(SysState_t from, SysEvent_t ev)
{
    (void)from; (void)ev;

    gate_mc_close();
    gate_hw_disable();
    BoardCtrl_SetpointSel(false);
    LaseQSetMode(SW);

    /*
     * Comunica a LaseQ il clear dell'errore: un SOLO messaggio con
     * clear_error=1 (LaseQClearErr() imposta s_ctrl.clear_error, spedito
     * al prossimo ciclo RS485 di task_comms). Il "singolo messaggio" è
     * garantito da LaseQ_ParseResponse() (LaseQ.c): appena arriva una
     * risposta valida allo STATUS successivo, s_ctrl.clear_error viene
     * riportato a 0 — non resta mai "incollato" a 1 sui cicli successivi.
     */
    LaseQClearErr();

    /*
     * Conferma bloccante: fino a CLEAR_ERR_CONFIRM_MAX_WAIT_MS (25 cicli
     * RS485 da LQ_TRANSMIT_WINDOW=20ms, ampio margine su TX+processing+RX
     * lato LaseQ), rilegge error_code ogni CLEAR_ERR_CONFIRM_POLL_MS.
     */
    {
        const uint32_t POLL_MS  = 20U;   /* CLEAR_ERR_CONFIRM_POLL_MS */
        const uint32_t MAX_WAIT = 500U;  /* CLEAR_ERR_CONFIRM_MAX_WAIT_MS */
        uint32_t elapsed  = 0U;
        uint8_t  err_code;

        do {
            osDelay(POLL_MS);
            elapsed += POLL_MS;
            err_code = TaskMonitor_GetLaseQErrorCode();
        } while (err_code != 0U && elapsed < MAX_WAIT);

        if (err_code != 0U) {
            char errname[96];
            FSM_FormatErrorCode(err_code, errname, sizeof(errname));
            SysLog_Event(LOG_ERROR,
                         "CERR rifiutato: LaseQ ancora in errore dopo %ums (0x%02X: %s)",
                         (unsigned)elapsed, err_code, errname);
            Queue_PostEvent(SYS_LASEQ_ERROR_EVENT);
            return;
        }
    }

    psu_turn_off_all();
    contactors_off_all();

    /* Disarma SUBITO il SAB (il riarmo avverrà in action_enter_on/enabled);
     * idempotente rispetto a un eventuale spegnimento già pianificato da
     * action_enter_error() poco prima (vedi sab_request_shutdown()) — se
     * l'operatore fa CERR quasi subito, i 10s restano contati dal fronte
     * di discesa originale, non da questa chiamata. */
    sab_request_shutdown();

    SetBoardLED(GREEN);
    SetLEDMode(STATUS_LED, SLOW_FADE);
    SetLEDMode(EMISSION_LED, OFF);

    /*
     * Questa transizione porta a SYS_ACTIVE: stessi valori READY/ERROR/
     * EMISSION_RDY di action_enter_active() (sys_ready=false, error=false,
     * emission_rdy=false). nCMD_RDY_iso non toccato: segue solo s_mode
     * (sync_cmd_rdy_status()), non cambia mai su un CLR_ERR.
     */
    EXT_SetError(false);
    EXT_SetSysStatus(false);
    EXT_EmissionRdyStatus(false);

    /*
     * action_clear_error porta a SYS_ACTIVE.
     *
     * BUGFIX (2026-07-31, "GET ERR mostra sempre FLOW=1 anche dopo CERR e
     * anche se il flusso è già rientrato prima del SON successivo"): il
     * commento originale qui sotto assumeva "il water cooling era già
     * attivo, non viene spento in ERROR" — vero per le cause di errore
     * diverse da FLOW, ma FALSO per un errore di flusso: action_enter_error()
     * chiama WaterCooling_Emergency() quando ev == SYS_FLOW_ERROR_EVENT, che
     * porta WaterCooling_GetState() a WC_STATE_ERROR (vedi WaterCooling.c) e
     * NON a WC_STATE_RUNNING. Senza un WaterCooling_Enable() qui, restava in
     * WC_STATE_ERROR per tutta la durata di SYS_ACTIVE (fino al prossimo
     * SYS_ON, dove action_enter_on()/action_enter_active() lo riavviano) —
     * e con lo stato diverso da RUNNING, Monitor_CheckFlow() (task_monitor.c)
     * salta ogni rivalutazione del flusso ad ogni ciclo, quindi il bit
     * ERR_BIT_FLOW restava congelato al valore del guasto originale
     * indipendentemente dal flusso fisico reale. Stesso richiamo già fatto
     * da action_enter_active() (SYS_IDLE/ON/ENABLED/EMISSION → ACTIVE):
     * qui va rifatto esplicitamente perché clear_error è una transizione
     * diretta ERROR→ACTIVE con logica propria, non richiama
     * action_enter_active(). Idempotente se il water cooling non era mai
     * stato fermato (cause di errore non-FLOW).
     */
    WaterCooling_Enable();
    TaskMonitor_ResetFlowInhibit();

    /*
     * Azzera il bitmask errori congelato (2026-07-31, richiesta esplicita):
     * SOLO qui, dopo la conferma LaseQ — se la funzione fosse uscita prima
     * (return nel blocco di polling sopra, CERR respinto) questa riga non
     * viene MAI raggiunta, quindi tutte le cause accumulate durante l'episodio
     * di errore restano visibili in "GET ERR"/COM_STAT_OP_ERR_ACTIVE finché
     * l'uscita da SYS_ERROR non è realmente completata. Vedi banner su
     * s_latched_errors in task_monitor.c.
     */
    TaskMonitor_ClearLatchedErrors();
}
