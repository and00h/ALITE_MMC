/*
 * rs485_cmd.c  --  MMC
 *
 * Implementazione interfaccia di comando ASCII su RS485 (USART1).
 * DE gestita automaticamente dall'hardware USART1 in RS485 mode (nativo CubeMX).
 *
 * ARCHITETTURA:
 *   - HAL_UARTEx_ReceiveToIdle_DMA() avvia la ricezione in DMA.
 *   - Rs485Cmd_OnRxIdle() (chiamata da ISR) segnala arrivo frame via thread flag.
 *   - Rs485Cmd_Update() blocca su osThreadFlagsWait(); quando il log è attivo usa
 *     un timeout per inviare la riga periodica anche senza comandi.
 *   - Risposta tramite HAL_UART_Transmit() bloccante (frame brevi, < 5ms).
 *
 * AUTENTICAZIONE:
 *   Alcuni comandi che modificano parametri di configurazione e sicurezza richiedono
 *   una sessione autenticata (LOGIN). La sessione scade automaticamente dopo
 *   RS485_SESSION_TIMEOUT_MS di inattività.
 *   Default password: "1234" (case-insensitive, modificabile con SET PASSWORD).
 *   La password viene memorizzata in RAM e non persiste tra i riavvii.
 *
 * COMANDI PROTETTI DA LOGIN:
 *   SET TEMP   <sensor> <WARN|ERR> <MIN|MAX> <val>
 *   SET FLOW   <WARN|ERR> <MIN|MAX> <val_x10>
 *   SET HUM    <MMC|LASEQ> <val_pct>
 *   SET DEW    <WARN|ERR> DELTA <val>
 *   SET DELAY  <PSU|SAB|CONTACTOR> <ms>
 *   SET PSU    <VOLTAGE|CURRENT|MASK> <val>   (solo in stato IDLE)
 *   SET CONTACTOR MASK <0-3>                  (solo in stato IDLE)
 *   SET EFUSE  MASK <0-15>                    (bit0=MAIN..bit3=LASEQ, richiede power-cycle)
 *   SET PD     MASK <0-15>                    (solo in stato IDLE)
 *   SET ERR    MASK <num>
 *   SET WARN   MASK <num>
 *   SET FAULT  MASK <num>                     (SOLO comunicazioni: COM interface/LaseQ/AMC)
 *   SET FLOW   MASK <0-3>                     (device-enable, bit0=FLOW_METER_1)
 *   SET TEMP   MASK <0-0x1FF>                 (device-enable, bit0=WATER_IN..bit6=DIODE2,bit7=PSU_TEMP,bit8=PWR_EL_TEMP)
 *   SET CURRENT <mA>                          (solo FSM_MODE_SW, bypassa LUT)
 *   SET NTC    BETA <serigrafia 1-16> <beta>  (NUOVO 2026-07-22, coefficiente Beta per-canale)
 *   SET VCOMP  ON|OFF                         (NUOVO 2026-07-22, compensazione dinamica tensione PSU, solo stato IDLE)
 *   SET VCOMP  DELAY|RAMP|RAMPUP <ms>         (NUOVO 2026-07-22, timing compensazione, solo stato IDLE)
 *   SET LUT    ...  (incl. VCOMP <entry> <current_ma> <voltage_mv>, NUOVO 2026-07-22, solo stato IDLE)
 *   SET PASSWORD <old> <new>
 *   SAVE LUT  /  RESET LUT  /  SAVE CONFIG
 *   GET LQ                                    (parametri dettagliati LaseQ: Vanode/correnti/setpoint/canali/ERR)
 *   GET MODULES                               (stato ON/OFF + corrente eFuse: MAIN/SAB/COM/LASEQ)
 *
 * LOG STREAM (CSV, separatore ";" — per acquisizione .csv su PC, ridisegnato
 * 2026-07-27):
 *   LOG ON [interval_s]  — avvia stream periodico (default 3s) su RS485.
 *                          Invia "OK", poi UNA riga di intestazione CSV,
 *                          poi una riga dati (prima immediata, poi ogni
 *                          interval_s): DATE;TIME;SP_PCT;CUR_CMD_MA;
 *                          CUR_ECHO_MA;MB_TEMP_C;MB_HUM_PCT;WATER_IN_C;
 *                          WATER_OUT_C;DIODE1_C;DIODE2_C;AMBIENT_C;SPLICE_C;
 *                          PSU_TEMP_C;PWR_EL_TEMP_C;LQ_DRV0..3_C;LQ_AMB_C;
 *                          LQ_HUM_PCT;VANODE_MV;FLOW1_LMIN;FLOW2_LMIN;WARN;
 *                          ERR;FAULT;PD0..3_RAW (fotodiodi NON riscalati in
 *                          Watt);PD_STALE (NUOVO 2026-07-28: 1=AMC segnala
 *                          pd_raw non aggiornato da >500ms, vedi
 *                          AMC_IsPDDataStale()).
 *   LOG OFF              — interrompe lo stream.
 *
 * LOG ERRORI (distinto dallo stream sopra):
 *   LOG ERR ON   — abilita la notifica non sollecitata "ALARM <x>"/"FAULT <x>"
 *                  ad ogni transizione verso SYS_ERROR/SYS_FAULT.
 *   LOG ERR OFF  — disabilita (default a ogni avvio: NON persistito).
 *
 * SEQUENZA STATI FSM (guidata da RS485 in SW mode):
 *   IDLE ──START──> ACTIVE ──SON──> ON ──SEN──> ENABLED ──PON──> EMISSION
 *   ERROR ──CERR──> ACTIVE
 *
 * THREAD-SAFETY:
 *   Rs485Cmd_OnRxIdle() scrive s_rx_size e segnala il task via osThreadFlagsSet.
 *   s_task_handle è scritto una sola volta in Rs485Cmd_Init() prima che l'ISR
 *   possa attivarsi, quindi non serve protezione aggiuntiva.
 */

#include "rs485_cmd.h"

#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>
#include <stdarg.h>       /* send_line(): formattazione varargs, vedi sotto */

#include "cmsis_os.h"           /* osThreadFlagsWait/Set */
#include "hal_handles.h"        /* huart1, hrtc */
#include "fsm_events.h"         /* SysEvent_t */
#include "fsm.h"                /* FSM_GetState(), FSM_GetMode() */
#include "queues.h"             /* Queue_PostEvent() */
#include "setpoint.h"           /* Setpoint_SetPct() / GetPct() */
#include "task_comms.h"
#include "task_amc.h"           /* TaskAmc_GetPDPower(), TaskAmc_RequestPSUConfigResend() */
#include "lut_manager.h"        /* LUT_Set*() */
#include "task_monitor.h"       /* TaskMonitor_GetErrors(), GetWarnings() */
#include "AD7490.h"             /* AD7490_NUM_CHANNELS */
#include "config.h"             /* g_config, Config_Save() */
#include "fw_version.h"         /* FW_VERSION_STR/FW_BUILD_DATE_STR: GET FW */
#include "fw_slot.h"             /* FwSlot_GetActiveChar(): GET FW (campo SLOT) */
#include "ota_update_trigger.h" /* OTA_TRIGGER_BKP_REGISTER/MAGIC: FW UPDATE */
#include "SHT35.h"              /* SHT35_GetData(), SHT35_GetHumidityPct() */
#include "FlowMeter.h"          /* FlowMeter_GetFlowRate() */
#include "BoardCtrl.h"          /* BoardCtrl_SetTermination() */
#include "EXT_interface.h"      /* EXT_SetReducedSetpointRange() */
#include "QCW.h"                /* QCW_SetParams(), QCW_MIN/MAX_* */
#include "LaseQ.h"               /* GetLaseQStatus()/GetLaseQControl(): GET LQ */
#include "AMC.h"                 /* AMC_GetPDRaw(): LOG CSV (fotodiodi non riscalati) */
#include "eFuse.h"                /* EFuse_GetStatus(): GET MODULES */
#include "SAB.h"                  /* SAB_GetState()/GetInterlockStatus()/GetTestStatus(): GET SAB */

/* -------------------------------------------------------------------------- */
/* Flag per notifica ISR → task                                                */
/* -------------------------------------------------------------------------- */

#define RS485_RX_FLAG   (1UL << 0)
#define RS485_TX_FLAG   (1UL << 1)  /* notifica completamento TX DMA → task */

/* Timeout massimo di attesa in Rs485Cmd_Update() quando il log periodico è
 * disattivo: prima era osWaitForever, ora limitato per permettere al task
 * RS485 di richiamare Watchdog_Heartbeat() anche senza traffico sul bus
 * (vedi Watchdog.h). Non cambia il comportamento osservabile: allo scadere,
 * senza log attivo, la funzione ritorna senza fare nulla (v. sotto). */
#define RS485_IDLE_HEARTBEAT_MS   2000U

/* Forward declaration: restart_rx è usata in send_response (definita più avanti) */
static void restart_rx(void);

static osThreadId_t s_task_handle = NULL;

/*
 * Mutex su huart1: serializza l'accesso alla periferica tra il task RS485
 * (send_response(), sequenza Abort→TX DMA→restart RX) e Rs485Cmd_NotifyFault()
 * chiamata in contesto task FSM (fsm.c) per notificare fault/errori in modo
 * asincrono. L'HAL UART non è thread-safe: due contesti che toccano huart1
 * in parallelo (uno in TX_DMA, l'altro in TX bloccante) possono corrompere
 * lo stato interno (gState) e bloccare il sistema — causa identificata del
 * freeze "quasi ad ogni comando RS485".
 *
 * osMutexPrioInherit evita l'inversione di priorità: il task FSM ha priorità
 * più alta del task RS485 e non deve restare bloccato a lungo in attesa che
 * un task a priorità inferiore rilasci il mutex.
 */
static osMutexId_t s_uart_mutex = NULL;

/* -------------------------------------------------------------------------- */
/* Formato di visualizzazione codici errore/warning                            */
/* -------------------------------------------------------------------------- */

typedef enum {
    NUM_FMT_HEX = 0,   /* 0x0000ABCD  (default) */
    NUM_FMT_DEC,        /* 2748                  */
    NUM_FMT_BIN,        /* b00101011             */
} NumFmt_t;

static NumFmt_t s_err_fmt   = NUM_FMT_HEX;
static NumFmt_t s_warn_fmt  = NUM_FMT_HEX;
static NumFmt_t s_fault_fmt = NUM_FMT_HEX;

/* -------------------------------------------------------------------------- */
/* Log stream                                                                  */
/* -------------------------------------------------------------------------- */

static bool     s_log_active      = false;
static uint32_t s_log_interval_ms = 3000U;   /* default 3 s (formato CSV, vedi send_log_row()) */

/*
 * Log errori RS485 ("LOG ERR ON"/"LOG ERR OFF"): distinto dallo stream
 * periodico sopra. Quando attivo, ogni transizione verso SYS_ERROR/SYS_FAULT
 * viene notificata non sollecitata via Rs485Cmd_NotifyFault() (vedi sotto).
 * Flag di sessione: MAI persistito in config, sempre false a ogni avvio.
 */
static bool     s_log_err_enabled = false;

/* -------------------------------------------------------------------------- */
/* Autenticazione                                                              */
/*                                                                             */
/* Password in RAM (case-insensitive di fatto, str_upper() applicata prima).  */
/* La sessione scade dopo RS485_SESSION_TIMEOUT_MS di inattività.             */
/* -------------------------------------------------------------------------- */

#define RS485_AUTH_PW_DEFAULT       "092021"
#define RS485_SESSION_TIMEOUT_MS    (5UL * 60UL * 1000UL)   /* 5 min */

static char     s_password[16]   = RS485_AUTH_PW_DEFAULT;
static bool     s_authenticated   = false;
static uint32_t s_auth_last_tick  = 0U;

/* -------------------------------------------------------------------------- */
/* Buffers                                                                     */
/* -------------------------------------------------------------------------- */

/*
 * Buffer DMA: allineato a 32 byte (dimensione cache line su Cortex-M7).
 * Necessario per SCB_InvalidateDCache_by_Addr() che richiede indirizzo e
 * dimensione multipli di 32.  Il DMA scrive in RAM bypassando la D-cache;
 * la CPU deve invalidare le linee interessate prima di leggere.
 */
static uint8_t  s_dma_buf[RS485_CMD_BUF_SIZE] __attribute__((aligned(32)));

/*
 * Buffer TX DMA: allineato a 32 byte (dimensione cache line Cortex-M7).
 * Necessario per SCB_CleanDCache_by_Addr(): la CPU scrive qui i dati da
 * trasmettere, poi flush cache → RAM prima che il DMA li legga.
 * Dimensionato a RS485_RSP_BUF_SIZE; HELP usa invii multipli (chunk).
 */
static uint8_t  s_tx_buf[RS485_RSP_BUF_SIZE]  __attribute__((aligned(32)));

/*
 * Buffer dedicato per l'header/riga CSV di "LOG ON" (vedi send_log_header()/
 * send_log_row() sotto): NON riusa s_rsp_buf (256 byte) perché l'header CSV,
 * con ~30 colonne, supera quella dimensione (~274 byte). send_response()
 * trasmette comunque a chunk da sizeof(s_tx_buf) indipendentemente dalla
 * lunghezza della stringa sorgente, quindi l'unico vincolo reale è poter
 * COMPORRE la stringa intera con snprintf() prima dell'invio — da qui un
 * buffer locale più grande, non un ingrandimento di RS485_RSP_BUF_SIZE
 * (usato da tutti gli altri comandi, non necessario altrove).
 */
static char     s_log_csv_buf[320];

static char     s_cmd_buf[RS485_CMD_BUF_SIZE];
static char     s_rsp_buf[RS485_RSP_BUF_SIZE];

static volatile uint16_t s_rx_size  = 0U;

/* ========================================================================== */
/* HELPERS INTERNI                                                             */
/* ========================================================================== */

static void send_response(const char *str)
{
    /*
     * RS485 half-duplex echo: il transceiver riflette ogni byte TX sulla linea
     * RX. Se il DMA RX è attivo durante la trasmissione, cattura l'eco della
     * risposta come se fosse un nuovo comando → al ciclo successivo il parser
     * riceve un buffer corrotto (es. "GET STATE\r\nOK STATE:INIT\r\n") e
     * fallisce su tok[1].
     *
     * Soluzione: interrompiamo il DMA RX prima di trasmettere e lo riavviamo
     * solo dopo che l'ultimo byte TX è stato confermato dall'ISR.
     * HAL_UART_AbortReceive() è sincrono e non chiama RxEventCallback.
     */
    /* Esclusivo su huart1: vedi commento su s_uart_mutex. Timeout lungo perché
     * questo è il task "proprietario" della periferica in condizioni normali;
     * l'unico contendente (Rs485Cmd_NotifyFault) rilascia in pochi ms. */
    osMutexAcquire(s_uart_mutex, osWaitForever);

    HAL_UART_AbortReceive(&huart1);

    uint16_t len    = (uint16_t)strlen(str);
    uint16_t offset = 0U;

    while (offset < len) {
        uint16_t chunk = len - offset;
        if (chunk > (uint16_t)sizeof(s_tx_buf)) {
            chunk = (uint16_t)sizeof(s_tx_buf);
        }

        memcpy(s_tx_buf, str + offset, chunk);

        /*
         * H7 D-cache: la CPU ha scritto in s_tx_buf (in D1 AXI SRAM, cacheable).
         * SCB_CleanDCache_by_Addr() flush le linee dirty → RAM, in modo che
         * il DMA TX legga i dati aggiornati e non quelli stantii in cache.
         * L'indirizzo e la dimensione devono essere multipli di 32 byte.
         */
        SCB_CleanDCache_by_Addr((uint32_t *)s_tx_buf, (int32_t)sizeof(s_tx_buf));

        if (HAL_UART_Transmit_DMA(&huart1, s_tx_buf, chunk) == HAL_OK) {
            /* Attende notifica da Rs485Cmd_OnTxCplt() (via HAL_UART_TxCpltCallback ISR) */
            osThreadFlagsWait(RS485_TX_FLAG, osFlagsWaitAny, 200U);
        }

        offset += chunk;
    }

    /* Riavvia DMA RX solo dopo TX completato: il buffer è ora pulito da eco */
    restart_rx();

    osMutexRelease(s_uart_mutex);
}

/*
 * @brief  Formatta UNA riga (senza "OK", senza "\r\n" nel formato passato) e
 *         la trasmette subito come risposta a sé stante — dal 2026-07-15,
 *         usata da tutti i comandi GET che restituiscono più elementi
 *         omogenei (liste/tabelle: GET NTC MAP, GET MODULES, GET TEMP, GET
 *         DEW, GET LQ, GET CONFIG, GET THRESHOLDS), un elemento per riga
 *         invece di un'unica riga lunga — più leggibile su terminale.
 *
 *         Convenzione: la PRIMA riga di ogni comando multi-riga porta il
 *         prefisso "OK " (segnala il successo del comando, stesso
 *         significato di sempre); le righe successive sono dati puri, senza
 *         prefisso — stesso schema già usato da "HELP" (help_a/help_b).
 *
 *         Ogni riga è una trasmissione UART separata (send_response()
 *         gestisce già mutex/DMA per singola chiamata): più lento di una
 *         singola risposta lunga, ma trascurabile a 115200 baud per il
 *         numero di righe in gioco qui (max ~20), e evita di dover
 *         ingrandire s_rsp_buf per i comandi con più elementi (GET
 *         THRESHOLDS supera 256 byte se compattato su una riga sola).
 *
 * @param  fmt  Stringa di formato printf-style, SENZA "\r\n" finale
 *              (aggiunto automaticamente).
 */
static void send_line(const char *fmt, ...)
{
    /*
     * 128 byte: copre anche la riga più lunga in uso (GET LQ "ERR:...",
     * che include FSM_FormatErrorCode() — fino a 7 nomi di bit concatenati
     * con "|", ~80 caratteri da soli). Righe più lunghe vengono troncate in
     * modo sicuro da vsnprintf (nessun overflow), solo meno leggibili.
     */
    char buf[128];
    va_list ap;

    va_start(ap, fmt);
    int wn = vsnprintf(buf, sizeof(buf) - 2U, fmt, ap);
    va_end(ap);

    if (wn < 0) { return; }
    size_t len = ((size_t)wn < sizeof(buf) - 2U) ? (size_t)wn : (sizeof(buf) - 2U - 1U);
    buf[len]     = '\r';
    buf[len + 1U] = '\n';
    buf[len + 2U] = '\0';

    send_response(buf);
}

static void ok(const char *extra)
{
    if (extra && *extra) {
        snprintf(s_rsp_buf, sizeof(s_rsp_buf), "OK %s\r\n", extra);
    } else {
        snprintf(s_rsp_buf, sizeof(s_rsp_buf), "OK\r\n");
    }
    send_response(s_rsp_buf);
}

static void err(const char *reason)
{
    snprintf(s_rsp_buf, sizeof(s_rsp_buf), "ERR %s\r\n", reason);
    send_response(s_rsp_buf);
}

static void err_state(const char *cmd, const char *required_state)
{
    snprintf(s_rsp_buf, sizeof(s_rsp_buf),
             "ERR %s requires state %s (current: %s)\r\n",
             cmd, required_state, Rs485Cmd_StateStr(FSM_GetState()));
    send_response(s_rsp_buf);
}

/*
 * @brief  Risposta di errore per i comandi di sequenza FSM (START/STOP/SON/
 *         SOFF/SEN/SDIS/PON/POFF/CERR) quando il canale di controllo attivo
 *         è ANALOG (FSM_GetMode() == FSM_MODE_ANALOG).
 *
 * In ANALOG mode, FSM_ProcessEvent() scarta silenziosamente questi eventi
 * (isolamento modale, vedi fsm.h/fsm.c): senza questo controllo esplicito,
 * il comando superava il solo check di stato (spesso soddisfatto, dato che
 * in ANALOG mode la FSM avanza comunque tramite i pin EXT) e rispondeva
 * "OK" pur non avendo alcun effetto — fuorviante per chi pilota via RS485.
 */
static void err_mode(const char *cmd)
{
    snprintf(s_rsp_buf, sizeof(s_rsp_buf),
             "ERR %s ignored: control channel is EXT (ANALOG mode)\r\n",
             cmd);
    send_response(s_rsp_buf);
}

/*
 * @brief  SAB_REARM_BLOCK (rimovibile, vedi banner "FEATURE-FLAG:
 *         SAB_REARM_BLOCK" in fsm.c). Risposta di errore per SEN quando la
 *         finestra di scarica capacità SAB (~10s dopo l'ultima uscita da
 *         SYS_ENABLED/SYS_EMISSION verso SYS_ON) è ancora in corso:
 *         FSM_ProcessEvent() scarterebbe comunque SYS_ENABLE_EVENT, quindi
 *         senza questo controllo esplicito SEN risponderebbe "OK" pur non
 *         avendo alcun effetto (stesso principio di err_mode() sopra).
 */
static void err_sab_guard(const char *cmd)
{
    snprintf(s_rsp_buf, sizeof(s_rsp_buf),
             "ERR %s ignored: SAB discharge window active, retry shortly\r\n",
             cmd);
    send_response(s_rsp_buf);
}

/*
 * @brief  true se lo stato corrente è tra quelli in cui FSM_ProcessEvent()
 *         accetta SET GATEHW/SETPOINTHW (SYS_IDLE, SYS_ACTIVE, SYS_ON —
 *         vedi fsm.c, invariato). Fuori da questo range l'evento verrebbe
 *         scartato silenziosamente: usato per rispondere con un errore
 *         esplicito invece di un "OK" fuorviante.
 */
static bool mode_cmd_state_ok(void)
{
    SysState_t st = FSM_GetState();
    return (st == SYS_IDLE || st == SYS_ACTIVE || st == SYS_ON);
}

/*
 * @brief  true se lo stato corrente è tra quelli in cui FSM_ProcessEvent()
 *         accetta SET MODE SW/HYBRID/ANALOG: SYS_IDLE, SYS_ACTIVE, SYS_ON
 *         (nessuna transizione di stato) più — NUOVO, richiesto
 *         esplicitamente 2026-07-27 — SYS_ENABLED/SYS_EMISSION (scende
 *         sempre a SYS_ON, vedi fsm.c, blocco SET MODE in
 *         FSM_ProcessEvent()). Fuori da questo range (SYS_ERROR/SYS_FAULT)
 *         l'evento verrebbe scartato silenziosamente.
 */
static bool mode_set_cmd_state_ok(void)
{
    SysState_t st = FSM_GetState();
    return (st == SYS_IDLE || st == SYS_ACTIVE || st == SYS_ON ||
            st == SYS_ENABLED || st == SYS_EMISSION);
}

static void str_upper(char *s)
{
    for (; *s; ++s) *s = (char)toupper((unsigned char)*s);
}

static int tokenize(char *s, char **tok, int max_tok)
{
    int n = 0;
    char *p = s;
    while (*p && n < max_tok) {
        while (*p == ' ' || *p == '\t') ++p;
        if (*p == '\0') break;
        tok[n++] = p;
        while (*p && *p != ' ' && *p != '\t') ++p;
        if (*p) *p++ = '\0';
    }
    return n;
}

static void format_bitmask(uint32_t val, NumFmt_t fmt, char *buf, size_t size)
{
    switch (fmt) {
        case NUM_FMT_DEC:
            snprintf(buf, size, "%lu", (unsigned long)val);
            break;
        case NUM_FMT_BIN: {
            size_t pos = 0;
            if (pos < size - 1U) buf[pos++] = 'b';
            for (int bit = 31; bit >= 0 && pos < size - 1U; --bit) {
                buf[pos++] = ((val >> bit) & 1U) ? '1' : '0';
            }
            buf[pos] = '\0';
            break;
        }
        case NUM_FMT_HEX:
        default:
            snprintf(buf, size, "0x%08lX", (unsigned long)val);
            break;
    }
}

/**
 * @brief  Auto-rileva il formato numerico e parsa il valore.
 *
 *   0x{cifre} / 0X{cifre}  → HEX   (case-insensitive, strtoul accetta a-f/A-F)
 *   b{cifre}  / B{cifre}   → BIN
 *   {cifre}                → DEC
 *
 * @note   Completamente case-insensitive: il prefisso viene confrontato via
 *         toupper() indipendentemente da eventuali pre-elaborazioni (str_upper).
 *         Esempi equivalenti: 0x0F2 = 0X0f2 = 0X0F2 ; B101 = b101.
 */
static bool parse_number(const char *s, uint32_t *out)
{
    if (!s || !*s) return false;

    char *end;
    unsigned long val;

    /* Uppercase dei primi due caratteri per confronto case-insensitive */
    char c0 = (char)toupper((unsigned char)s[0]);
    char c1 = s[1] ? (char)toupper((unsigned char)s[1]) : '\0';

    if (c0 == '0' && c1 == 'X') {
        if (!s[2]) return false;
        val = strtoul(s + 2, &end, 16);
        if (end == s + 2 || *end != '\0') return false;
    } else if (c0 == 'B') {
        if (!s[1]) return false;
        val = strtoul(s + 1, &end, 2);
        if (end == s + 1 || *end != '\0') return false;
    } else {
        val = strtoul(s, &end, 10);
        if (end == s || *end != '\0') return false;
    }

    *out = (uint32_t)val;
    return true;
}

static void restart_rx(void)
{
    /*
     * Riavvia ricezione DMA+IDLE.  Chiamata da task context dopo che il
     * comando corrente è stato copiato in s_cmd_buf e processato.
     * HT interrupt disabilitato: per frame brevi vogliamo solo l'evento IDLE,
     * non il callback a metà buffer.
     */
    __HAL_UART_CLEAR_FLAG(&huart1, UART_CLEAR_OREF | UART_CLEAR_FEF | UART_CLEAR_NEF);
    HAL_UARTEx_ReceiveToIdle_DMA(&huart1, s_dma_buf, sizeof(s_dma_buf));
    __HAL_DMA_DISABLE_IT(huart1.hdmarx, DMA_IT_HT);
}

/* -------------------------------------------------------------------------- */
/* Helper: nome stato FSM                                                      */
/* -------------------------------------------------------------------------- */

const char *Rs485Cmd_StateStr(SysState_t st)
{
    switch (st) {
        case SYS_INIT:     return "INIT";
        case SYS_IDLE:     return "IDLE";
        case SYS_ACTIVE:   return "ACTIVE";
        case SYS_ON:       return "ON";
        case SYS_ENABLED:  return "ENABLED";
        case SYS_EMISSION: return "EMISSION";
        case SYS_ERROR:    return "ERROR";
        case SYS_FAULT:    return "FAULT";
        default:           return "?";
    }
}

/* ========================================================================== */
/* AUTENTICAZIONE                                                              */
/* ========================================================================== */

/**
 * @brief  Macro di guardia: se non autenticati, risponde ERR e ritorna.
 *         Se autenticati, aggiorna il timestamp di inattività.
 */
#define REQUIRE_AUTH() \
    do { \
        if (!s_authenticated) { err("LOGIN required"); return; } \
        s_auth_last_tick = osKernelGetTickCount(); \
    } while (0)

/**
 * @brief  Controlla la scadenza della sessione.
 *         Da chiamare a ogni iterazione del loop (anche senza comandi).
 */
static void check_session_timeout(void)
{
    if (!s_authenticated) return;
    uint32_t elapsed = osKernelGetTickCount() - s_auth_last_tick;
    if (elapsed >= RS485_SESSION_TIMEOUT_MS) {
        s_authenticated = false;
    }
}

/* ========================================================================== */
/* RTC — lettura e scrittura                                                   */
/* ========================================================================== */

/**
 * @brief  Legge RTC e riempie time_str "HH:MM:SS" e date_str "YYYY-MM-DD".
 *         NOTA: su STM32H7 si DEVE leggere sempre sia GetTime sia GetDate
 *         (nell'ordine Time→Date) per sbloccare i shadow register.
 */
static void rtc_get_strings(char *time_str, size_t tlen,
                             char *date_str, size_t dlen)
{
    RTC_TimeTypeDef t = {0};
    RTC_DateTypeDef d = {0};
    HAL_RTC_GetTime(&hrtc, &t, RTC_FORMAT_BIN);
    HAL_RTC_GetDate(&hrtc, &d, RTC_FORMAT_BIN);   /* sblocca shadow register */
    if (time_str && tlen >= 9U) {
        snprintf(time_str, tlen, "%02u:%02u:%02u",
                 (unsigned)t.Hours, (unsigned)t.Minutes, (unsigned)t.Seconds);
    }
    if (date_str && dlen >= 11U) {
        snprintf(date_str, dlen, "20%02u-%02u-%02u",
                 (unsigned)d.Year, (unsigned)d.Month, (unsigned)d.Date);
    }
}

/* ========================================================================== */
/* LOG STREAM                                                                  */
/* ========================================================================== */

/**
 * @brief  Invia l'intestazione CSV dello stream "LOG ON" (una volta sola,
 *         subito dopo l'"OK", prima della prima riga dati — vedi cmd_log()).
 *
 * Separatore ";" (non ",") — richiesto esplicitamente 2026-07-27 per import
 * diretto in Excel/Numbers con locale IT (dove "," è il separatore
 * decimale, non di colonna).
 *
 * Colonne (richieste esplicitamente, 2026-07-27, per acquisizione .csv su
 * PC — vedi send_log_row() per il dettaglio di ciascuna):
 *   DATE;TIME;SP_PCT;CUR_CMD_MA;CUR_ECHO_MA;
 *   MB_TEMP_C;MB_HUM_PCT;
 *   WATER_IN_C;WATER_OUT_C;DIODE1_C;DIODE2_C;AMBIENT_C;SPLICE_C;PSU_TEMP_C;PWR_EL_TEMP_C;
 *   LQ_DRV0_C;LQ_DRV1_C;LQ_DRV2_C;LQ_DRV3_C;LQ_AMB_C;LQ_HUM_PCT;
 *   VANODE_MV;FLOW1_LMIN;FLOW2_LMIN;
 *   WARN;ERR;FAULT;
 *   PD0_RAW;PD1_RAW;PD2_RAW;PD3_RAW;PD_STALE
 */
static void send_log_header(void)
{
    snprintf(s_log_csv_buf, sizeof(s_log_csv_buf),
             "DATE;TIME;SP_PCT;CUR_CMD_MA;CUR_ECHO_MA;"
             "MB_TEMP_C;MB_HUM_PCT;"
             "WATER_IN_C;WATER_OUT_C;DIODE1_C;DIODE2_C;AMBIENT_C;SPLICE_C;PSU_TEMP_C;PWR_EL_TEMP_C;"
             "LQ_DRV0_C;LQ_DRV1_C;LQ_DRV2_C;LQ_DRV3_C;LQ_AMB_C;LQ_HUM_PCT;"
             "VANODE_MV;FLOW1_LMIN;FLOW2_LMIN;"
             "WARN;ERR;FAULT;"
             "PD0_RAW;PD1_RAW;PD2_RAW;PD3_RAW;PD_STALE\r\n");
    send_response(s_log_csv_buf);
}

/**
 * @brief  Invia una riga dati CSV dello stream "LOG ON" (una ogni
 *         s_log_interval_ms, default 3s — vedi cmd_log()/Rs485Cmd_Update()).
 *
 * Note sui campi (vedi banner send_log_header() per l'elenco colonne):
 *   - CUR_CMD_MA: corrente COMANDATA a LaseQ (GetLaseQControl(), "SP_CMD" in
 *     GET LQ) — quella che MMC sta chiedendo, non necessariamente ancora
 *     applicata da LaseQ.
 *   - CUR_ECHO_MA: corrente ECHEGGIATA/confermata da LaseQ (GetLaseQStatus(),
 *     "SP_ECHO" in GET LQ) — richiesto esplicitamente 2026-07-27, accanto al
 *     comandato, per poter verificare quanto impiega LaseQ a recepire un
 *     cambio di setpoint.
 *   - Temperature NTC: solo gli 8 sensori funzionali/nominati (stesso set di
 *     WATER_IN..PWR_EL_TEMP in GET TEMP) — gli id generici NTC8..NTC13/NTC16
 *     sono esclusi per mantenere un numero di colonne CSV fisso e
 *     significativo. Campo vuoto (nessun valore fra due separatori) se il
 *     sensore non è mappato (TaskMonitor_GetNTCTempC10() ritorna false).
 *   - VANODE_MV: tensione PSU misurata da LaseQ (GetLaseQStatus().v_anode),
 *     stesso campo di "VANODE" in GET LQ.
 *   - FLOW1/2: entrambi i canali, come GET FLOW (solo FLOW_METER_1 è
 *     fisicamente cablato ad oggi).
 *   - WARN/ERR/FAULT: bitmask esadecimali (TaskMonitor_GetWarnings()/
 *     GetErrors()/GetFaults()), stesso formato di GET WARN/ERR/FAULT.
 *   - PD0..3_RAW: valori ADC 12-bit NON riscalati (AMC_GetPDRaw()) — a
 *     differenza di TaskAmc_GetPDPower() (usato da send_log_line() prima
 *     del redesign CSV), qui NON viene applicata la conversione LUT->Watt,
 *     richiesto esplicitamente ("dati letti dai fotodiodi, ancora non
 *     riscalati").
 *   - PD_STALE (NUOVO 2026-07-28): AMC_IsPDDataStale(), 1 se AMC segnala
 *     (pd_status_mask bit6, protocollo v0.0005+) che pd_raw[] non e' stato
 *     aggiornato da oltre 500ms lato AMC — es. acquisizione ADC1 bloccata.
 *     Aggiunto dopo un caso reale in campo (2026-07-28) in cui PD0..3_RAW
 *     restavano fissi in ogni condizione a causa di un bug di
 *     configurazione ADC1/ADC2 lato AMC (DMAContinuousRequests, risolto in
 *     main.c/MX_ADC1_Init — vedi AMC_protocol.h per i dettagli); questa
 *     colonna permette di distinguere a colpo d'occhio da un LOG "un
 *     valore costante perche' e' cosi'" da "un valore costante perche' non
 *     si aggiorna piu'", senza dover ragionare sulla plausibilita' fisica
 *     dei numeri.
 */
static void send_log_row(void)
{
    char ts[10], ds[12];
    rtc_get_strings(ts, sizeof(ts), ds, sizeof(ds));

    unsigned      sp_pct     = (unsigned)Setpoint_GetPct();
    unsigned long cur_cmd_ma  = (unsigned long)GetLaseQControl().sw_current_setpoint;
    unsigned long cur_echo_ma = (unsigned long)GetLaseQStatus().sw_current_setpoint;

    /* MMC: temperatura scheda (SHT35) + umidità */
    SHT35_Data_t d      = SHT35_GetData();
    int          mb_cdeg = (int)d.temperature_cdeg;
    int          mb_ti   = mb_cdeg / 100;
    int          mb_tf   = (mb_cdeg < 0 ? -mb_cdeg : mb_cdeg) % 100;
    unsigned     mb_hum  = (unsigned)SHT35_GetHumidityPct();

    /* NTC funzionali, stesso ordine delle colonne dell'header sopra */
    static const NTC_SensorId_t s_csv_ntc_ids[8] = {
        NTC_SENSOR_WATER_IN, NTC_SENSOR_WATER_OUT, NTC_SENSOR_DIODE1, NTC_SENSOR_DIODE2,
        NTC_SENSOR_AMBIENT,  NTC_SENSOR_SPLICE,    NTC_SENSOR_PSU_TEMP, NTC_SENSOR_PWR_EL_TEMP,
    };
    char ntc_s[8][8];
    for (uint8_t i = 0U; i < 8U; i++) {
        int16_t t_c10;
        if (TaskMonitor_GetNTCTempC10(s_csv_ntc_ids[i], &t_c10)) {
            int ti = t_c10 / 10, tf = (t_c10 < 0 ? -t_c10 : t_c10) % 10;
            snprintf(ntc_s[i], sizeof(ntc_s[i]), "%d.%01d", ti, tf);
        } else {
            ntc_s[i][0] = '\0';   /* sensore non mappato: campo vuoto */
        }
    }

    /* LaseQ: temperature driver/ambiente + umidità (stesso set di GET TEMP LQ) */
    uint8_t lq_drv[4];
    int8_t  lq_amb;
    uint8_t lq_hum;
    TaskMonitor_GetLaseQTemps(lq_drv, &lq_amb, &lq_hum);

    unsigned vanode_mv = (unsigned)GetLaseQStatus().v_anode;

    float f1  = FlowMeter_GetFlowRate(FLOW_METER_1);
    float f2  = FlowMeter_GetFlowRate(FLOW_METER_2);
    int   f1i = (int)(f1 * 10.0f);
    int   f2i = (int)(f2 * 10.0f);

    char emask[12], wmask[12], fmask[12];
    format_bitmask(TaskMonitor_GetErrors(),   NUM_FMT_HEX, emask, sizeof(emask));
    format_bitmask(TaskMonitor_GetWarnings(), NUM_FMT_HEX, wmask, sizeof(wmask));
    format_bitmask(TaskMonitor_GetFaults(),   NUM_FMT_HEX, fmask, sizeof(fmask));

    uint16_t pd_raw[4] = {0U};
    AMC_GetPDRaw(pd_raw);
    unsigned pd_stale = AMC_IsPDDataStale() ? 1U : 0U;

    snprintf(s_log_csv_buf, sizeof(s_log_csv_buf),
             "%s;%s;%u;%lu;%lu;"
             "%d.%02d;%u;"
             "%s;%s;%s;%s;%s;%s;%s;%s;"
             "%d;%d;%d;%d;%d;%u;"
             "%u;%d.%01u;%d.%01u;"
             "%s;%s;%s;"
             "%u;%u;%u;%u;%u\r\n",
             ds, ts, sp_pct, cur_cmd_ma, cur_echo_ma,
             mb_ti, mb_tf, mb_hum,
             ntc_s[0], ntc_s[1], ntc_s[2], ntc_s[3], ntc_s[4], ntc_s[5], ntc_s[6], ntc_s[7],
             (int)lq_drv[0], (int)lq_drv[1], (int)lq_drv[2], (int)lq_drv[3],
             (int)lq_amb, (unsigned)lq_hum,
             vanode_mv,
             f1i / 10, (unsigned)((f1i < 0 ? -f1i : f1i) % 10),
             f2i / 10, (unsigned)((f2i < 0 ? -f2i : f2i) % 10),
             emask, wmask, fmask,
             (unsigned)pd_raw[0], (unsigned)pd_raw[1], (unsigned)pd_raw[2], (unsigned)pd_raw[3],
             pd_stale);
    send_response(s_log_csv_buf);
}

/* ========================================================================== */
/* TABELLA SENSORI TEMPERATURA (per SET/GET TEMP)                             */
/* ========================================================================== */

typedef struct {
    const char *name;
    int16_t    *min_err;
    int16_t    *min_warn;
    int16_t    *max_warn;
    int16_t    *max_err;
} TempSensorEntry_t;

/* Macro per ridurre la ripetizione: espande in entry con puntatori a g_config */
#define TEMP_ENTRY(lbl, pfx) \
    { lbl, \
      &g_config.temp_##pfx##_min_err,  &g_config.temp_##pfx##_min_warn, \
      &g_config.temp_##pfx##_max_warn, &g_config.temp_##pfx##_max_err }

static const TempSensorEntry_t s_temp_sensors[] = {
    TEMP_ENTRY("WATER_IN",  water_in),
    TEMP_ENTRY("WATER_OUT", water_out),
    TEMP_ENTRY("DRIVER",    driver),
    TEMP_ENTRY("SPLICE",    splice),
    /* DIODE1/DIODE2 (dal 2026-07-15): diodo laser a due sensori NTC fisici
     * indipendenti — vedi NTC_SENSOR_DIODE1/DIODE2 in task_monitor.h. Campo
     * config sottostante per DIODE1 resta "diode" (nome storico, invariato
     * per compatibilità di offset in Config_t), DIODE2 usa "diode2". */
    TEMP_ENTRY("DIODE1",    diode),
    TEMP_ENTRY("DIODE2",    diode2),
    TEMP_ENTRY("AMBIENT",   ambient),
    /* LQ_AMBIENT (dal 2026-07-15): soglie DEDICATE per l'ambiente interno
     * LaseQ (SHT35 su LaseQ), separate da AMBIENT (board MMC) — l'interno
     * di LaseQ è più caldo per via della vicinanza di componenti elettronici
     * di potenza. Vedi Monitor_CheckLaseQTelemetry() in task_monitor.c e
     * campo temp_lq_ambient_* in config.h. */
    TEMP_ENTRY("LQ_AMBIENT", lq_ambient),
    /*
     * PSU_TEMP / PWR_EL_TEMP / MB_TEMP (NUOVO 2026-07-22):
     *   PSU_TEMP/PWR_EL_TEMP — nuovi slot NTC funzionali (non ancora
     *     cablati), stesso schema di WATER_IN/DRIVER/... — vedi
     *     NTC_SENSOR_PSU_TEMP/PWR_EL_TEMP in task_monitor.h.
     *   MB_TEMP — SHT35 su scheda MMC (ex "ambient" della board, rinominato
     *     per non confonderlo con NTC_SENSOR_AMBIENT: quest'ultimo è un NTC
     *     dedicato all'ambiente/scocca, MB_TEMP è invece la temperatura
     *     dell'elettronica MMC misurata dall'SHT35 locale — vedi
     *     Monitor_CheckHumidity() in task_monitor.c). Soglie proprie,
     *     diverse da AMBIENT: un componente elettronico raggiunge
     *     facilmente 35°C+, valori normali per temperatura ambiente (tipico
     *     20-27°C) sarebbero concettualmente sbagliati qui.
     */
    TEMP_ENTRY("PSU_TEMP",    psu),
    TEMP_ENTRY("PWR_EL_TEMP", pwr_el),
    TEMP_ENTRY("MB_TEMP",     mb),
};

#define TEMP_SENSOR_COUNT  (sizeof(s_temp_sensors) / sizeof(s_temp_sensors[0]))

/* Tabella nomi NTC sensor_id → stringa (indice = NTC_SensorId_t) */
static const char * const s_ntc_sensor_names[NTC_NUM_SENSORS] = {
    "WATER_IN",   /* NTC_SENSOR_WATER_IN  = 0 */
    "WATER_OUT",  /* NTC_SENSOR_WATER_OUT = 1 */
    /* NTC_SENSOR_DRIVER dal 2026-07-14 non è più la sorgente della
     * temperatura driver (vedi task_monitor.h) — id generico, nessuna
     * serigrafia dedicata da assegnare (vedi nota in task_monitor.h). Nome
     * "NTC_GEN2" (non "NTC12": quel nome è già usato da NTC_SENSOR_NTC12,
     * id=11 sotto — evita la collisione di nomi che c'era prima). */
    "NTC_GEN2",   /* NTC_SENSOR_DRIVER    = 2  (ex "DRIVER", ora generico, id storico) */
    "SPLICE",     /* NTC_SENSOR_SPLICE    = 3  (dal 2026-07-15: serigrafia 16, CH0) */
    "DIODE1",     /* NTC_SENSOR_DIODE1    = 4  (serigrafia 3, CH12) */
    "AMBIENT",    /* NTC_SENSOR_AMBIENT   = 5 */
    "DIODE2",     /* NTC_SENSOR_DIODE2    = 6  (serigrafia 4, CH13) — ex "NTC7" */
    "NTC8",       /* NTC_SENSOR_NTC8      = 7  (id generico, non ancora assegnato) */
    "NTC9",       /* NTC_SENSOR_NTC9      = 8  (id generico, non ancora assegnato) */
    "NTC10",      /* NTC_SENSOR_NTC10     = 9  (id generico, non ancora assegnato) */
    "NTC11",      /* NTC_SENSOR_NTC11     = 10 (id generico, non ancora assegnato) */
    "NTC12",      /* NTC_SENSOR_NTC12     = 11 (id generico, non ancora assegnato) */
    "NTC13",      /* NTC_SENSOR_NTC13     = 12 (id generico, non ancora assegnato) */
    /* PSU_TEMP/PWR_EL_TEMP (NUOVO 2026-07-22, ex id generici NTC14/NTC15):
     * non ancora cablati (destinati a serigrafia NTC14/CH2 e NTC15/CH1),
     * ma il nome NON è più generico — vedi NTC_SENSOR_PSU_TEMP/PWR_EL_TEMP
     * in task_monitor.h e s_temp_sensors[] sopra (soglie SET/GET TEMP). */
    "PSU_TEMP",    /* NTC_SENSOR_PSU_TEMP    = 13 */
    "PWR_EL_TEMP", /* NTC_SENSOR_PWR_EL_TEMP = 14 */
    /* NTC_SENSOR_NTC16 (id=15): nome storico, ma la serigrafia fisica 16
     * è ora SPLICE (id=3, sopra) — questo id resta generico/disponibile,
     * mappalo a qualunque canale via "SET NTC MAP", non necessariamente
     * alla serigrafia 16 (che va invece mappata a SPLICE). */
    "NTC16",      /* NTC_SENSOR_NTC16     = 15 (id generico, non ancora assegnato) */
};

/*
 * Corrispondenza serigrafia PCB (numero stampato sul silkscreen, 1-16) →
 * canale fisico AD7490 (0-15) — instradamento fisso dal routing di scheda
 * (schema Opal MMC), NON configurabile. Indice array = serigrafia-1.
 *
 * Dal 2026-07-15: "SET/GET NTC MAP" lavora in termini di NUMERO DI
 * SERIGRAFIA (quello leggibile sulla scheda), non più di canale AD7490
 * grezzo — più comodo per chi cablando/mappando in campo legge il numero
 * stampato sul connettore, non il canale interno del multiplexer. Questa
 * tabella converte da un sistema all'altro; g_config.ntc_ch_map resta
 * internamente indicizzato per canale AD7490 (nessuna modifica al formato
 * persistito in flash).
 */
static const uint8_t s_ntc_serigrafia_to_ch[16] = {
    14, 15, 12, 13, 10, 11, 8, 9, 7, 6, 5, 4, 3, 2, 1, 0
    /* serigrafia: 1   2   3   4   5  6  7 8 9 10 11 12 13 14 15 16 */
};

/* ========================================================================== */
/* GESTORI COMANDI — SEQUENZA FSM                                             */
/* ========================================================================== */

/*
 * NOTA COMUNE a tutti i comandi sotto: ciascuno posta un evento SW che
 * FSM_ProcessEvent() scarta silenziosamente se FSM_GetMode() ==
 * FSM_MODE_ANALOG (isolamento modale — il canale di controllo attivo è
 * l'interfaccia EXT, non RS485). Il check "if (FSM_GetMode() ==
 * FSM_MODE_ANALOG)" qui replica ESATTAMENTE la stessa condizione, solo per
 * poter rispondere con un errore esplicito invece di un fuorviante "OK"
 * seguito da nessun effetto reale. Va tenuto aggiornato in coppia con la
 * lista eventi bloccati in FSM_ProcessEvent() (fsm.c).
 */

static void cmd_start(void)
{
    if (FSM_GetMode() == FSM_MODE_ANALOG) { err_mode("START"); return; }
    if (FSM_GetState() != SYS_IDLE) { err_state("START", "IDLE"); return; }
    Queue_PostEvent(SYS_START_EVENT);
    ok(NULL);
}

static void cmd_stop(void)
{
    if (FSM_GetMode() == FSM_MODE_ANALOG) { err_mode("STOP"); return; }
    if (FSM_GetState() != SYS_ACTIVE) { err_state("STOP", "ACTIVE"); return; }
    Queue_PostEvent(SYS_STOP_EVENT);
    ok(NULL);
}

static void cmd_son(void)
{
    if (FSM_GetMode() == FSM_MODE_ANALOG) { err_mode("SON"); return; }
    if (FSM_GetState() != SYS_ACTIVE) { err_state("SON", "ACTIVE"); return; }
    Queue_PostEvent(SYS_ON_EVENT);
    ok(NULL);
}

static void cmd_soff(void)
{
    if (FSM_GetMode() == FSM_MODE_ANALOG) { err_mode("SOFF"); return; }
    SysState_t st = FSM_GetState();
    if (st != SYS_ON && st != SYS_ENABLED && st != SYS_EMISSION) {
        err_state("SOFF", "ON|ENABLED|EMISSION"); return;
    }
    Queue_PostEvent(SYS_OFF_EVENT);
    ok(NULL);
}

static void cmd_sen(void)
{
    if (FSM_GetMode() == FSM_MODE_ANALOG) { err_mode("SEN"); return; }
    if (FSM_GetState() != SYS_ON) { err_state("SEN", "ON"); return; }
    if (FSM_IsSabRearmBlocked()) { err_sab_guard("SEN"); return; }
    Queue_PostEvent(SYS_ENABLE_EVENT);
    ok(NULL);
}

static void cmd_sdis(void)
{
    if (FSM_GetMode() == FSM_MODE_ANALOG) { err_mode("SDIS"); return; }
    SysState_t st = FSM_GetState();
    if (st != SYS_ENABLED && st != SYS_EMISSION) {
        err_state("SDIS", "ENABLED|EMISSION"); return;
    }
    Queue_PostEvent(SYS_DISABLE_EVENT);
    ok(NULL);
}

static void cmd_pon(void)
{
    if (FSM_GetMode() == FSM_MODE_ANALOG) { err_mode("PON"); return; }
    if (FSM_GetState() != SYS_ENABLED) { err_state("PON", "ENABLED"); return; }
    Queue_PostEvent(SYS_LASERON_EVENT);
    ok(NULL);
}

static void cmd_poff(void)
{
    if (FSM_GetMode() == FSM_MODE_ANALOG) { err_mode("POFF"); return; }
    if (FSM_GetState() != SYS_EMISSION) { err_state("POFF", "EMISSION"); return; }
    Queue_PostEvent(SYS_LASEROFF_EVENT);
    ok(NULL);
}

static void cmd_cerr(void)
{
    /* CERR è l'unica eccezione all'isolamento modale (vedi fsm.c): valido
     * da RS485/COM in QUALSIASI modo, anche FSM_MODE_ANALOG — nessun check
     * su FSM_GetMode() qui, a differenza degli altri comandi di sequenza. */
    if (FSM_GetState() != SYS_ERROR) { err_state("CERR", "ERROR"); return; }
    Queue_PostEvent(SYS_CLR_ERR_EVENT);
    ok(NULL);
}

/*
 * FRST: reset esplicito di SYS_FAULT -> IDLE (dal 2026-07-14) — [PROTETTO,
 * REQUIRE_AUTH()]. Controparte di CERR ma per SYS_FAULT (non recuperabile):
 * a differenza di CERR, RICHIEDE login, perché un fault non recuperabile
 * (incluso un fault "latched" persistito da un boot precedente — vedi
 * banner "FAULT LATCH" in task_monitor.h) implica una condizione che deve
 * essere consapevolmente verificata/risolta da un operatore autorizzato
 * prima di rimettere in funzione la macchina, non un semplice "riprova".
 * Vedi FSM_RequestFaultReset()/action_fault_reset() in fsm.h/fsm.c.
 */
static void cmd_frst(void)
{
    REQUIRE_AUTH();
    if (FSM_GetState() != SYS_FAULT) { err_state("FRST", "FAULT"); return; }
    FSM_RequestFaultReset();
    ok(NULL);
}

/* ========================================================================== */
/* GESTORI COMANDI — AUTENTICAZIONE                                           */
/* ========================================================================== */

static void cmd_login(char **tok, int n)
{
    if (n < 2) { err("USAGE: LOGIN <password>"); return; }
    if (s_authenticated) { ok("already authenticated"); return; }
    if (strcmp(tok[1], s_password) != 0) { err("wrong password"); return; }
    s_authenticated  = true;
    s_auth_last_tick = osKernelGetTickCount();
    ok("authenticated - session timeout 5min");
}

static void cmd_logout(void)
{
    s_authenticated = false;
    ok("logged out");
}

/* ========================================================================== */
/* GESTORI COMANDI — SET                                                       */
/* ========================================================================== */

static void cmd_set(char **tok, int n)
{
    if (n < 2) { err("USAGE: SET <subcmd>"); return; }

    /* ------------------------------------------------------------------
     * SET MODE SW|HW|ANALOG
     *
     * "HW"/"HYBRID" restano accettati come alias legacy (Queue_PostEvent
     * SYS_SET_MODE_HYBRID_EVENT): impostano FSM_MODE_SW con entrambi i
     * toggle indipendenti sotto (GATEHW + SETPOINTHW) attivi — stesso
     * comportamento byte-per-byte della vecchia modalità HYBRID unica.
     * Per attivarli singolarmente usare SET GATEHW / SET SETPOINTHW.
     * ------------------------------------------------------------------ */
    if (strcmp(tok[1], "MODE") == 0) {
        if (n < 3) { err("USAGE: SET MODE SW|HW|ANALOG"); return; }

        SysEvent_t mode_ev;
        bool       is_hybrid;
        if (strcmp(tok[2], "SW") == 0) {
            mode_ev = SYS_SET_MODE_SW_EVENT;
            is_hybrid = false;
        } else if (strcmp(tok[2], "HW") == 0 || strcmp(tok[2], "HYBRID") == 0) {
            mode_ev = SYS_SET_MODE_HYBRID_EVENT;
            is_hybrid = true;
        } else if (strcmp(tok[2], "ANALOG") == 0) {
            mode_ev = SYS_SET_MODE_ANALOG_EVENT;
            is_hybrid = false;
        } else {
            err("MODE: SW|HW|ANALOG"); return;
        }

        /*
         * FSM_ProcessEvent() accetta SW/ANALOG da IDLE/ACTIVE/ON (nessuna
         * transizione di stato) e — richiesto esplicitamente 2026-07-27 —
         * anche da ENABLED/EMISSION (scende a SYS_ON, vedi fsm.c). HW/HYBRID
         * NON è incluso in questa estensione (resta IDLE/ACTIVE/ON come
         * prima: è solo un alias SW con toggle GATE_HW/SETPOINT_HW, non un
         * cambio di canale di controllo). Senza questo check rispondevamo
         * "OK" anche da stati in cui il comando non ha alcun effetto —
         * fuorviante.
         */
        bool state_ok = is_hybrid ? mode_cmd_state_ok() : mode_set_cmd_state_ok();
        if (!state_ok) {
            err_state("MODE", is_hybrid ? "IDLE|ACTIVE|ON" : "IDLE|ACTIVE|ON|ENABLED|EMISSION");
            return;
        }

        /*
         * NUOVO (richiesto esplicitamente, 2026-07-27): "SET MODE ANALOG"
         * consentito solo se nVEXT_GOOD=0 (EXT_IsVextGood()==true,
         * interfaccia EXT correttamente alimentata) — altrimenti la
         * sorgente di controllo esterna (nSYS_ON_iso/nENABLE_IN_iso/
         * nCLR_ERR_iso, setpoint analogico) non è affidabile. A differenza
         * del pin fisico nEXT_CTL_iso (che viene semplicemente ignorato),
         * qui va restituito un errore esplicito. SETPOINT HW / GATE HW
         * (sotto) NON sono soggetti a questo vincolo.
         */
        if (mode_ev == SYS_SET_MODE_ANALOG_EVENT && !EXT_IsVextGood()) {
            err("MODE ANALOG refused: EXT interface not properly powered");
            return;
        }

        Queue_PostEvent(mode_ev);
        ok(NULL);

    /* ------------------------------------------------------------------
     * SET GATEHW ON|OFF   ("HYBRID1": nGATE_HW_EN attivo pur restando in
     *                       SW mode — byte LaseQ identici a SW puro)
     * ------------------------------------------------------------------ */
    } else if (strcmp(tok[1], "GATEHW") == 0) {
        if (n < 3) { err("USAGE: SET GATEHW ON|OFF"); return; }
        if (!mode_cmd_state_ok()) { err_state("GATEHW", "IDLE|ACTIVE|ON"); return; }
        /* Toggle valido solo in FSM_MODE_SW (vedi fsm.c): in ANALOG mode
         * verrebbe scartato silenziosamente senza questo check. */
        if (FSM_GetMode() != FSM_MODE_SW) { err_mode("GATEHW"); return; }
        if      (strcmp(tok[2], "ON")  == 0) Queue_PostEvent(SYS_SET_GATE_HW_ON_EVENT);
        else if (strcmp(tok[2], "OFF") == 0) Queue_PostEvent(SYS_SET_GATE_HW_OFF_EVENT);
        else { err("GATEHW: ON|OFF"); return; }
        ok(NULL);

    /* ------------------------------------------------------------------
     * SET SETPOINTHW ON|OFF   ("HYBRID2": LaseQ riceve analog_mode=1 pur
     *                           restando in SW mode)
     * ------------------------------------------------------------------ */
    } else if (strcmp(tok[1], "SETPOINTHW") == 0) {
        if (n < 3) { err("USAGE: SET SETPOINTHW ON|OFF"); return; }
        if (!mode_cmd_state_ok()) { err_state("SETPOINTHW", "IDLE|ACTIVE|ON"); return; }
        if (FSM_GetMode() != FSM_MODE_SW) { err_mode("SETPOINTHW"); return; }
        if      (strcmp(tok[2], "ON")  == 0) Queue_PostEvent(SYS_SET_SETPOINT_HW_ON_EVENT);
        else if (strcmp(tok[2], "OFF") == 0) Queue_PostEvent(SYS_SET_SETPOINT_HW_OFF_EVENT);
        else { err("SETPOINTHW: ON|OFF"); return; }
        ok(NULL);

    /* ------------------------------------------------------------------
     * SET SETPOINT <0-100>
     * SET SETPOINT RANGE 10V|6V
     * (nessun login — vedi nREDUCED_SETPOINT_RANGE_Pin, active LOW)
     *
     * Imposta il range del setpoint esterno. Altri valori di tensione
     * (diversi da "10V"/"6V") restituiscono errore e non modificano
     * l'impostazione corrente. Stesso schema di "SET TERM ON|OFF": il pin
     * viene applicato subito e la preferenza persistita subito in flash
     * (Config_Save()), non in attesa del comando separato "SAVE CONFIG".
     * ------------------------------------------------------------------ */
    } else if (strcmp(tok[1], "SETPOINT") == 0) {
        if (n < 3) { err("USAGE: SET SETPOINT <0-100>|RANGE 10V|6V"); return; }

        if (strcmp(tok[2], "RANGE") == 0) {
            if (n < 4) { err("USAGE: SET SETPOINT RANGE 10V|6V"); return; }
            bool reduced;
            if      (strcmp(tok[3], "10V") == 0) reduced = false;
            else if (strcmp(tok[3], "6V")  == 0) reduced = true;
            else { err("SETPOINT RANGE: 10V|6V"); return; }

            g_config.reduced_setpoint_range = reduced ? 1U : 0U;
            EXT_SetReducedSetpointRange(reduced);

            Config_err_t e = Config_Save();
            if (e == CONFIG_OK) {
                snprintf(s_rsp_buf, sizeof(s_rsp_buf), "OK SETPOINT RANGE:%s\r\n",
                         reduced ? "6V" : "10V");
            } else {
                snprintf(s_rsp_buf, sizeof(s_rsp_buf),
                         "OK SETPOINT RANGE:%s (WARN: save failed, code %d)\r\n",
                         reduced ? "6V" : "10V", (int)e);
            }
            send_response(s_rsp_buf);

        } else {
            if (FSM_GetMode() != FSM_MODE_SW) { err("SETPOINT only in SW mode"); return; }
            int v = atoi(tok[2]);
            if (v < 0 || v > 100) { err("range: 0-100"); return; }
            Setpoint_SetPct((uint8_t)v);
            snprintf(s_rsp_buf, sizeof(s_rsp_buf), "OK SP:%d%% CUR:%lumA\r\n",
                     v, (unsigned long)Setpoint_GetCurrentMa());
            send_response(s_rsp_buf);
        }

    /* ------------------------------------------------------------------
     * SET CURRENT <mA>  [PROTETTO]
     *
     * Imposta il setpoint direttamente in corrente, bypassando la
     * conversione percentuale/LUT (vedi Setpoint_SetCurrentMa() in
     * setpoint.h/.c) — utile in service mode per bench test/verifica
     * calibrazione. Stesso vincolo di modalità di SET SETPOINT: solo in
     * FSM_MODE_SW (in ANALOG il setpoint è gestito dall'hardware esterno,
     * task_comms non chiama LaseQSetCurrent()/AMC_SetCurrentSetpoint()).
     * A differenza di SET SETPOINT, richiede login: agisce direttamente
     * sulla corrente erogata senza il vincolo del passo 5% della LUT.
     *
     * LIMITE 12000mA: cap di sicurezza applicato ESCLUSIVAMENTE qui, non in
     * Setpoint_SetCurrentMa() (che resta un'API generale clampata solo al
     * range uint16_t) né sulla LUT setpoint (SET LUT SETPOINT, 0-65535):
     * essendo l'unico percorso che bypassa del tutto la LUT calibrata,
     * questo comando è il punto giusto per un tetto esplicito e non deve
     * propagarsi ad altri chiamanti dell'API. Valore allineato al limite
     * "lifecycle 12000mA" citato in config.c/Config_LoadDefaults() come
     * corrente massima ammessa oltre la nominale 10500mA.
     * ------------------------------------------------------------------ */
    } else if (strcmp(tok[1], "CURRENT") == 0) {
        REQUIRE_AUTH();
        if (n < 3) { err("USAGE: SET CURRENT <mA>"); return; }
        if (FSM_GetMode() != FSM_MODE_SW) { err("CURRENT only in SW mode"); return; }
        int ma = atoi(tok[2]);
        if (ma < 0 || ma > 12000) { err("range: 0-12000"); return; }
        Setpoint_SetCurrentMa((uint32_t)ma);
        snprintf(s_rsp_buf, sizeof(s_rsp_buf), "OK CUR:%dmA (direct)\r\n", ma);
        send_response(s_rsp_buf);

    /* ------------------------------------------------------------------
     * SET LUT GAIN|VALID|POWER|SETPOINT|VCOMP ...  [PROTETTO]
     * ------------------------------------------------------------------ */
    } else if (strcmp(tok[1], "LUT") == 0) {
        REQUIRE_AUTH();
        if (n < 3) { err("USAGE: SET LUT GAIN|VALID|POWER|SETPOINT|VCOMP|HWPWR ..."); return; }

        if (strcmp(tok[2], "GAIN") == 0) {
            if (n < 8) { err("USAGE: SET LUT GAIN SW|HW t0 t1 t2 t3"); return; }
            uint8_t mode;
            if      (strcmp(tok[3], "SW") == 0) mode = 0U;
            else if (strcmp(tok[3], "HW") == 0) mode = 1U;
            else { err("GAIN mode: SW|HW"); return; }
            for (uint8_t i = 0U; i < 4U; i++) {
                int v2 = atoi(tok[4U + i]);
                if (v2 < 0 || v2 > 65535) { err("threshold range: 0-65535"); return; }
                LUT_SetGainThreshold(mode, i, (uint16_t)v2);
            }
            ok(NULL);

        } else if (strcmp(tok[2], "VALID") == 0) {
            if (n < 9) { err("USAGE: SET LUT VALID pd SW|HW entry sp min max"); return; }
            int pd = atoi(tok[3]);
            if (pd < 0 || pd > 3) { err("pd: 0-3"); return; }
            uint8_t mode;
            if      (strcmp(tok[4], "SW") == 0) mode = 0U;
            else if (strcmp(tok[4], "HW") == 0) mode = 1U;
            else { err("VALID mode: SW|HW"); return; }
            int entry = atoi(tok[5]);
            if (entry < 0 || entry > 3) { err("entry: 0-3"); return; }
            int sp = atoi(tok[6]), mn = atoi(tok[7]), mx = atoi(tok[8]);
            if (sp < 0 || sp > 65535 || mn < 0 || mn > 65535 || mx < 0 || mx > 65535) {
                err("values range: 0-65535"); return;
            }
            LUT_SetPDValidEntry(mode, (uint8_t)pd, (uint8_t)entry,
                                (uint16_t)sp, (uint16_t)mn, (uint16_t)mx);
            ok(NULL);

        } else if (strcmp(tok[2], "POWER") == 0) {
            if (n < 8) { err("USAGE: SET LUT POWER pd win entry adc watt"); return; }
            int pd = atoi(tok[3]), win = atoi(tok[4]), entry = atoi(tok[5]);
            int adc = atoi(tok[6]), watt = atoi(tok[7]);
            if (pd < 0 || pd > 3)         { err("pd: 0-3");       return; }
            if (win < 0 || win > 3)       { err("win: 0-3");      return; }
            if (entry < 0 || entry > 4)   { err("entry: 0-4");    return; }
            if (adc < 0 || adc > 65535)   { err("adc: 0-65535");  return; }
            if (watt < 0 || watt > 65535) { err("watt: 0-65535"); return; }
            LUT_SetPDPowerEntry((uint8_t)pd, (uint8_t)win, (uint8_t)entry,
                                (uint16_t)adc, (uint16_t)watt);
            ok(NULL);

        /*
         * SET LUT SETPOINT idx current_ma  --  a differenza di GAIN/VALID/
         * POWER sopra (che agiscono sulla LUT_Store_t di lut_manager.c, RAM-
         * only finché non arriva SAVE LUT), questa entry appartiene a
         * g_config.setpoint_lut_ma[] (config.c/.h): la tabella power%->
         * current_ma usata da setpoint.c per convertire SET SETPOINT <0-100>
         * in mA per LaseQ. Config_SetLUTEntry() persiste subito in flash
         * (chiama Config_Save() internamente), quindi non serve un SAVE
         * separato per questa sotto-tabella.
         */
        } else if (strcmp(tok[2], "SETPOINT") == 0) {
            if (n < 5) { err("USAGE: SET LUT SETPOINT idx current_ma"); return; }
            int idx = atoi(tok[3]);
            int ma  = atoi(tok[4]);
            if (idx < 0 || idx >= (int)SETPOINT_LUT_SIZE) {
                err("idx: 0-20");
                return;
            }
            if (ma < 0 || ma > 65535) { err("current_ma: 0-65535"); return; }

            Config_err_t e = Config_SetLUTEntry((uint8_t)idx, (uint16_t)ma);
            if (e == CONFIG_OK) {
                snprintf(s_rsp_buf, sizeof(s_rsp_buf),
                         "OK SETPOINT[%d]:%dmA\r\n", idx, ma);
                send_response(s_rsp_buf);
            } else {
                snprintf(s_rsp_buf, sizeof(s_rsp_buf),
                         "ERR setpoint LUT save failed (code %d)\r\n", (int)e);
                send_response(s_rsp_buf);
            }

        /*
         * SET LUT VCOMP <entry 0-9> <current_ma> <voltage_mv>  [PROTETTO]
         *
         * LUT compensazione tensione PSU (mA->mV, NUOVO 2026-07-22, vedi
         * lut_manager.h/LUT_VoltageCompEntry_t) — 10 entry hardcoded
         * (LUT_VCOMP_SIZE), stesso schema RAM-only di GAIN/VALID/POWER sopra
         * (richiede SAVE LUT per persistere in flash) MA, A DIFFERENZA di
         * quelle, propaga subito il cambiamento ad AMC (stesso motivo di
         * SET PSU/SET VCOMP sopra: la LUT guida un comportamento hardware
         * attivo, non solo una conversione applicativa locale come
         * PD_VALID/POWER) — stesso vincolo SYS_IDLE.
         * voltage_mv=0 è il sentinel "entry non valida" (fallback a Vmax
         * lato AMC, vedi psu_voltage_comp.c) — se invece diverso da zero,
         * viene qui clampato a g_config.psu_voltage_mv (Vmax, SET PSU
         * VOLTAGE): la tensione PSU non deve MAI superare quel limite,
         * indipendentemente da cosa contenga la LUT.
         * Le entry vanno inviate/lette in ordine CRESCENTE di current_ma
         * (vedi task_amc.c, sequenza di avvio).
         */
        } else if (strcmp(tok[2], "VCOMP") == 0) {
            if (FSM_GetState() != SYS_IDLE) {
                err_state("SET LUT VCOMP", "IDLE");
                return;
            }
            if (n < 6) { err("USAGE: SET LUT VCOMP <entry 0-9> <current_ma> <voltage_mv>"); return; }
            int entry = atoi(tok[3]);
            if (entry < 0 || entry >= (int)LUT_VCOMP_SIZE) { err("entry: 0-9"); return; }
            int ma = atoi(tok[4]);
            int mv = atoi(tok[5]);
            if (ma < 0 || ma > 65535) { err("current_ma: 0-65535"); return; }
            if (mv < 0 || mv > 65535) { err("voltage_mv: 0-65535"); return; }

            /*
             * Clipping: la tensione non deve mai superare Vmax (SET PSU
             * VOLTAGE) — mv==0 resta il sentinel "non valida" (fallback a
             * Vmax lato AMC), non va clampato.
             */
            uint16_t vmax_mv = (g_config.psu_voltage_mv > 0 && g_config.psu_voltage_mv <= 0xFFFFU)
                               ? (uint16_t)g_config.psu_voltage_mv : 0xFFFFU;
            if (mv != 0 && (uint16_t)mv > vmax_mv) {
                mv = (int)vmax_mv;
            }

            LUT_SetVoltageCompEntry((uint8_t)entry, (uint16_t)ma, (uint16_t)mv);
            TaskAmc_RequestVoltageCompLUTResend();
            snprintf(s_rsp_buf, sizeof(s_rsp_buf),
                     "OK LUT VCOMP[%d]:%dmA->%dmV (inviato ad AMC, SAVE LUT per persistere)\r\n",
                     entry, ma, mv);
            send_response(s_rsp_buf);

        /*
         * SET LUT HWPWR <entry 0-10> <adc_raw 0-65535> <power_pct 0-100>
         *
         * LUT conversione RAW ADC (PC5/LPWR_SET_ISO, EXT_interface.c) -> %
         * potenza HW (NUOVO 2026-08-01, vedi lut_manager.h/LUT_HwPowerEntry_t)
         * — stesso schema RAM-only di GAIN/VALID/POWER sopra (richiede SAVE
         * LUT per persistere in flash). Usata SOLO per reporting locale (GET
         * STATUS RS485 / STATUS COM interface): nessun invio ad AMC/LaseQ,
         * quindi nessun vincolo di stato macchina.
         */
        } else if (strcmp(tok[2], "HWPWR") == 0) {
            if (n < 6) { err("USAGE: SET LUT HWPWR <entry 0-10> <adc_raw 0-65535> <power_pct 0-100>"); return; }
            int entry = atoi(tok[3]);
            if (entry < 0 || entry >= (int)LUT_HWPWR_SIZE) { err("entry: 0-10"); return; }
            int adc = atoi(tok[4]);
            int pct = atoi(tok[5]);
            if (adc < 0 || adc > 65535) { err("adc_raw: 0-65535"); return; }
            if (pct < 0 || pct > 100)   { err("power_pct: 0-100"); return; }

            LUT_SetHwPowerEntry((uint8_t)entry, (uint16_t)adc, (uint8_t)pct);
            snprintf(s_rsp_buf, sizeof(s_rsp_buf),
                     "OK LUT HWPWR[%d]:adc=%d->%d%% (SAVE LUT per persistere)\r\n",
                     entry, adc, pct);
            send_response(s_rsp_buf);

        } else {
            err("LUT subcmd: GAIN|VALID|POWER|SETPOINT|VCOMP|HWPWR");
        }

    /* ------------------------------------------------------------------
     * SET ERR  HEX|DEC|BIN  (libero)
     * SET ERR  MASK <num>   [PROTETTO]
     * ------------------------------------------------------------------ */
    } else if (strcmp(tok[1], "ERR") == 0) {
        if (n < 3) { err("USAGE: SET ERR HEX|DEC|BIN|MASK <num>"); return; }

        if (strcmp(tok[2], "MASK") == 0) {
            REQUIRE_AUTH();
            if (n < 4) { err("USAGE: SET ERR MASK <num>"); return; }
            uint32_t val;
            if (!parse_number(tok[3], &val)) { err("formato: 0x../B../dec (case-insensitive)"); return; }
            g_config.error_mask = val;
            char vs[36]; format_bitmask(val, NUM_FMT_HEX, vs, sizeof(vs));
            snprintf(s_rsp_buf, sizeof(s_rsp_buf), "OK ERR MASK:%s\r\n", vs);
            send_response(s_rsp_buf);
        }
        else if (strcmp(tok[2], "HEX") == 0) { s_err_fmt = NUM_FMT_HEX; ok("ERR fmt: HEX"); }
        else if (strcmp(tok[2], "DEC") == 0) { s_err_fmt = NUM_FMT_DEC; ok("ERR fmt: DEC"); }
        else if (strcmp(tok[2], "BIN") == 0) { s_err_fmt = NUM_FMT_BIN; ok("ERR fmt: BIN"); }
        else { err("SET ERR: HEX|DEC|BIN|MASK <num>"); }

    /* ------------------------------------------------------------------
     * SET WARN HEX|DEC|BIN  (libero)
     * SET WARN MASK <num>   [PROTETTO]
     * ------------------------------------------------------------------ */
    } else if (strcmp(tok[1], "WARN") == 0) {
        if (n < 3) { err("USAGE: SET WARN HEX|DEC|BIN|MASK <num>"); return; }

        if (strcmp(tok[2], "MASK") == 0) {
            REQUIRE_AUTH();
            if (n < 4) { err("USAGE: SET WARN MASK <num>"); return; }
            uint32_t val;
            if (!parse_number(tok[3], &val)) { err("formato: 0x../B../dec (case-insensitive)"); return; }
            g_config.warning_mask = val;
            char vs[36]; format_bitmask(val, NUM_FMT_HEX, vs, sizeof(vs));
            snprintf(s_rsp_buf, sizeof(s_rsp_buf), "OK WARN MASK:%s\r\n", vs);
            send_response(s_rsp_buf);
        }
        else if (strcmp(tok[2], "HEX") == 0) { s_warn_fmt = NUM_FMT_HEX; ok("WARN fmt: HEX"); }
        else if (strcmp(tok[2], "DEC") == 0) { s_warn_fmt = NUM_FMT_DEC; ok("WARN fmt: DEC"); }
        else if (strcmp(tok[2], "BIN") == 0) { s_warn_fmt = NUM_FMT_BIN; ok("WARN fmt: BIN"); }
        else { err("SET WARN: HEX|DEC|BIN|MASK <num>"); }

    /* ------------------------------------------------------------------
     * SET TEMP <sensor> <WARN|ERR> <MIN|MAX> <val_degC>  [PROTETTO]
     *
     * Sensori: WATER_IN WATER_OUT DRIVER SPLICE DIODE1 DIODE2 AMBIENT
     *          LQ_AMBIENT (ambiente interno LaseQ, soglie separate da
     *          AMBIENT dal 2026-07-15)
     *          PSU_TEMP PWR_EL_TEMP MB_TEMP (NUOVO 2026-07-22 — vedi
     *          s_temp_sensors[] sopra per il dettaglio)
     * Esempio: SET TEMP DIODE1 WARN MAX 40
     * Il valore è in gradi Celsius interi con segno.
     * ------------------------------------------------------------------ */
    } else if (strcmp(tok[1], "TEMP") == 0) {
        REQUIRE_AUTH();
        if (n < 3) { err("USAGE: SET TEMP MASK <0-0x1FF> | SET TEMP <sensor> <WARN|ERR> <MIN|MAX> <val_degC>"); return; }

        /*
         * SET TEMP MASK <0-0x1FF>  [PROTETTO]
         *
         * Device-enable dedicato per i sensori NTC "funzionali" (bit0=
         * WATER_IN, bit1=WATER_OUT, bit2=DRIVER, bit3=SPLICE, bit4=DIODE1,
         * bit5=AMBIENT, bit6=DIODE2, bit7=PSU_TEMP, bit8=PWR_EL_TEMP —
         * NUOVO 2026-07-22: 2 bit aggiunti, mask allargata da uint8_t a
         * uint16_t in Config_t — stesso ordine di NTC_SensorId_t), stesso
         * pattern di SET PSU MASK / SET FLOW MASK: un sensore non cablato
         * va escluso dal check, non solo mascherato in notifica (vedi
         * g_config.temp_sensor_enabled_mask, task_monitor.c check_ntc_sensor()).
         * MB_TEMP non ha un bit qui: è un SHT35 di bordo, sempre presente
         * (stesso motivo per cui AMBIENT/LQ_AMBIENT via SHT35 non hanno un
         * bit device-enable dedicato — solo gli NTC opzionali ne hanno uno).
         */
        if (strcmp(tok[2], "MASK") == 0) {
            if (n < 4) { err("USAGE: SET TEMP MASK <0-0x1FF>"); return; }
            uint32_t mask;
            if (!parse_number(tok[3], &mask) || mask > 0x1FFU) {
                err("mask: 0-0x1FF (bit0=WATER_IN..bit6=DIODE2,bit7=PSU_TEMP,bit8=PWR_EL_TEMP)"); return;
            }
            g_config.temp_sensor_enabled_mask = (uint16_t)mask;
            snprintf(s_rsp_buf, sizeof(s_rsp_buf),
                     "OK TEMP MASK:0x%03X (SAVE CONFIG per persistere)\r\n",
                     (unsigned)mask);
            send_response(s_rsp_buf);
            return;
        }

        if (n < 6) { err("USAGE: SET TEMP <sensor> <WARN|ERR> <MIN|MAX> <val_degC>"); return; }

        /* Ricerca sensore nella tabella */
        const TempSensorEntry_t *ts = NULL;
        for (size_t i = 0U; i < TEMP_SENSOR_COUNT; i++) {
            if (strcmp(tok[2], s_temp_sensors[i].name) == 0) {
                ts = &s_temp_sensors[i]; break;
            }
        }
        if (!ts) {
            err("sensor: WATER_IN|WATER_OUT|DRIVER|SPLICE|DIODE1|DIODE2|AMBIENT|LQ_AMBIENT|PSU_TEMP|PWR_EL_TEMP|MB_TEMP");
            return;
        }

        bool is_err  = (strcmp(tok[3], "ERR")  == 0);
        bool is_warn = (strcmp(tok[3], "WARN") == 0);
        if (!is_err && !is_warn) { err("level: WARN|ERR"); return; }

        bool is_min = (strcmp(tok[4], "MIN") == 0);
        bool is_max = (strcmp(tok[4], "MAX") == 0);
        if (!is_min && !is_max) { err("limit: MIN|MAX"); return; }

        int val = atoi(tok[5]);
        if (val < -50 || val > 200) { err("val: -50..200 degC"); return; }

        int16_t *p = is_err  ? (is_min ? ts->min_err  : ts->max_err)
                             : (is_min ? ts->min_warn : ts->max_warn);
        *p = (int16_t)val;
        snprintf(s_rsp_buf, sizeof(s_rsp_buf),
                 "OK TEMP %s %s %s:%d degC\r\n",
                 tok[2], tok[3], tok[4], val);
        send_response(s_rsp_buf);

    /* ------------------------------------------------------------------
     * SET FLOW MASK <0-3>  [PROTETTO]
     *
     * Device-enable dedicato (bit0=FLOW_METER_1, bit1=FLOW_METER_2), stesso
     * pattern di SET PSU MASK / SET CONTACTOR MASK: un flussometro non
     * cablato va escluso qui, non solo mascherato in notifica (vedi
     * g_config.flow_sensor_enabled_mask, task_monitor.c Monitor_CheckFlow()).
     * A differenza di PSU/CONTACTOR non richiede stato IDLE: il flusso è
     * monitorato solo quando WaterCooling è RUNNING, cambiare la maschera
     * fuori da quella condizione non ha effetti immediati pericolosi.
     *
     * SET FLOW <WARN|ERR> <MIN|MAX> <val_lpm_x10>  [PROTETTO]
     *
     * Valori in L/min × 10 (es. 15 = 1.5 L/min, 30 = 3.0 L/min).
     * ------------------------------------------------------------------ */
    } else if (strcmp(tok[1], "FLOW") == 0) {
        REQUIRE_AUTH();
        if (n < 3) { err("USAGE: SET FLOW MASK <0-3> | SET FLOW <WARN|ERR> <MIN|MAX> <val_x10>"); return; }

        if (strcmp(tok[2], "MASK") == 0) {
            if (n < 4) { err("USAGE: SET FLOW MASK <0-3>"); return; }
            int mask = atoi(tok[3]);
            if (mask < 0 || mask > 0x03) { err("mask: 0-3 (bit0=FLOW_METER_1 bit1=FLOW_METER_2)"); return; }
            g_config.flow_sensor_enabled_mask = (uint8_t)mask;
            snprintf(s_rsp_buf, sizeof(s_rsp_buf),
                     "OK FLOW MASK:0x%02X (SAVE CONFIG per persistere)\r\n",
                     (unsigned)mask);
            send_response(s_rsp_buf);
            return;
        }

        if (n < 5) { err("USAGE: SET FLOW <WARN|ERR> <MIN|MAX> <val_x10>"); return; }

        bool is_err  = (strcmp(tok[2], "ERR")  == 0);
        bool is_warn = (strcmp(tok[2], "WARN") == 0);
        if (!is_err && !is_warn) { err("level: WARN|ERR"); return; }

        bool is_min = (strcmp(tok[3], "MIN") == 0);
        bool is_max = (strcmp(tok[3], "MAX") == 0);
        if (!is_min && !is_max) { err("limit: MIN|MAX"); return; }

        int val = atoi(tok[4]);
        if (val < 0 || val > 600) { err("val_x10: 0-600 (0..60 L/min)"); return; }

        uint16_t *p;
        if      (is_warn && is_min) p = &g_config.flow_min_warn_lpm_x10;
        else if (is_warn && is_max) p = &g_config.flow_max_warn_lpm_x10;
        else if (is_err  && is_min) p = &g_config.flow_min_err_lpm_x10;
        else                        p = &g_config.flow_max_err_lpm_x10;
        *p = (uint16_t)val;
        snprintf(s_rsp_buf, sizeof(s_rsp_buf),
                 "OK FLOW %s %s:%d.%01u L/min\r\n",
                 tok[2], tok[3], val / 10, (unsigned)(val % 10));
        send_response(s_rsp_buf);

    /* ------------------------------------------------------------------
     * SET HUM <MMC|LASEQ> <val_pct>  [PROTETTO]
     *
     * Soglia massima umidità relativa assoluta (warning).
     * ------------------------------------------------------------------ */
    } else if (strcmp(tok[1], "HUM") == 0) {
        REQUIRE_AUTH();
        if (n < 4) { err("USAGE: SET HUM <MMC|LASEQ> <val_pct>"); return; }
        int val = atoi(tok[3]);
        if (val < 0 || val > 100) { err("val_pct: 0-100"); return; }

        if      (strcmp(tok[2], "MMC")   == 0) g_config.humidity_max_warn_pct       = (uint8_t)val;
        else if (strcmp(tok[2], "LASEQ") == 0) g_config.laseq_humidity_max_warn_pct = (uint8_t)val;
        else { err("sensor: MMC|LASEQ"); return; }

        snprintf(s_rsp_buf, sizeof(s_rsp_buf), "OK HUM %s WARN_MAX:%d%%\r\n", tok[2], val);
        send_response(s_rsp_buf);

    /* ------------------------------------------------------------------
     * SET DEW <WARN|ERR> DELTA <val_degC>  [PROTETTO]
     *
     * Margine rispetto al dew point: avviso se T_water_in - Td < delta.
     * ------------------------------------------------------------------ */
    } else if (strcmp(tok[1], "DEW") == 0) {
        REQUIRE_AUTH();
        if (n < 5) { err("USAGE: SET DEW <WARN|ERR> DELTA <val_degC>"); return; }
        if (strcmp(tok[3], "DELTA") != 0) { err("SET DEW <WARN|ERR> DELTA <val>"); return; }
        int val = atoi(tok[4]);
        if (val < 0 || val > 20) { err("val: 0-20 degC"); return; }

        if      (strcmp(tok[2], "WARN") == 0) g_config.dew_warn_delta_c = (int8_t)val;
        else if (strcmp(tok[2], "ERR")  == 0) g_config.dew_err_delta_c  = (int8_t)val;
        else { err("level: WARN|ERR"); return; }

        snprintf(s_rsp_buf, sizeof(s_rsp_buf), "OK DEW %s DELTA:%d degC\r\n", tok[2], val);
        send_response(s_rsp_buf);

    /* ------------------------------------------------------------------
     * SET DELAY <PSU|SAB|CONTACTOR> <ms>  [PROTETTO]
     *
     * PSU       — inibizione controllo DC_OK dopo accensione alimentatore
     * SAB       — timeout interlock SAB
     * CONTACTOR — ritardo contattore dopo PSU enable
     * ------------------------------------------------------------------ */
    } else if (strcmp(tok[1], "DELAY") == 0) {
        REQUIRE_AUTH();
        if (n < 4) { err("USAGE: SET DELAY <PSU|SAB|CONTACTOR> <ms>"); return; }
        int val = atoi(tok[3]);
        if (val < 0 || val > 10000) { err("val: 0-10000 ms"); return; }

        if      (strcmp(tok[2], "PSU")       == 0) g_config.psu_dc_ok_delay_ms      = (uint16_t)val;
        else if (strcmp(tok[2], "SAB")       == 0) g_config.sab_interlock_timeout_ms = (uint16_t)val;
        else if (strcmp(tok[2], "CONTACTOR") == 0) g_config.contactor_psu_delay_ms   = (uint16_t)val;
        else { err("target: PSU|SAB|CONTACTOR"); return; }

        snprintf(s_rsp_buf, sizeof(s_rsp_buf), "OK DELAY %s:%dms\r\n", tok[2], val);
        send_response(s_rsp_buf);

    /* ------------------------------------------------------------------
     * SET PSU <VOLTAGE|CURRENT|MASK> <val>  [PROTETTO]
     *
     * VOLTAGE — limite tensione PSU [mV], inviato ad AMC (DAC)
     * CURRENT — limite corrente PSU [mA], inviato ad AMC (DAC)
     * MASK    — quali PSU abilitare (bit0=PSU1, bit1=PSU2), locale a MMC,
     *           non inviato ad AMC — vedi psu_turn_on_masked() in fsm.c
     *
     * Consentito SOLO in stato IDLE (PSU spento): il nuovo limite viene
     * scritto in g_config e re-inviato subito ad AMC tramite
     * TaskAmc_RequestPSUConfigResend() (applicato entro LQ_TRANSMIT_WINDOW
     * = 20ms dal task AMC dedicato, task_amc.c, unico proprietario della
     * UART verso AMC dal 2026-07-22). Il valore persiste in flash solo dopo
     * un successivo SAVE CONFIG.
     * ------------------------------------------------------------------ */
    } else if (strcmp(tok[1], "PSU") == 0) {
        REQUIRE_AUTH();
        if (n < 4) { err("USAGE: SET PSU <VOLTAGE|CURRENT|MASK> <val>"); return; }

        if (FSM_GetState() != SYS_IDLE) {
            err_state("SET PSU", "IDLE");
            return;
        }

        /*
         * MASK: quali PSU abilitare in action_enter_on() (fsm.c,
         * psu_turn_on_masked() — bit0=PSU1, bit1=PSU2). Solo 2 PSU fisici
         * (vedi PSU.h), quindi range 0-3. Scrive g_config direttamente,
         * come VOLTAGE/CURRENT: serve SAVE CONFIG per persistere in flash.
         * Non richiede TaskAmc_RequestPSUConfigResend() (non è un
         * parametro inviato ad AMC, è locale a MMC).
         */
        if (strcmp(tok[2], "MASK") == 0) {
            int mask = atoi(tok[3]);
            if (mask < 0 || mask > 0x03) { err("mask: 0-3 (bit0=PSU1 bit1=PSU2)"); return; }
            g_config.psu_enabled_mask = (uint8_t)mask;
            snprintf(s_rsp_buf, sizeof(s_rsp_buf),
                     "OK PSU MASK:0x%02X (SAVE CONFIG per persistere)\r\n",
                     (unsigned)mask);
            send_response(s_rsp_buf);
            return;
        }

        int val = atoi(tok[3]);
        if (val < 0 || val > 65535) { err("val: 0-65535"); return; }

        if      (strcmp(tok[2], "VOLTAGE") == 0) g_config.psu_voltage_mv = val;
        else if (strcmp(tok[2], "CURRENT") == 0) g_config.psu_current_ma = val;
        else { err("target: VOLTAGE|CURRENT|MASK"); return; }

        TaskAmc_RequestPSUConfigResend();

        snprintf(s_rsp_buf, sizeof(s_rsp_buf),
                 "OK PSU %s:%d (inviato ad AMC, SAVE CONFIG per persistere)\r\n",
                 tok[2], val);
        send_response(s_rsp_buf);

    /* ------------------------------------------------------------------
     * SET VCOMP ON|OFF                     [PROTETTO]
     * SET VCOMP DELAY  <ms 10-50>          [PROTETTO]
     * SET VCOMP RAMP   <ms 50-100>         [PROTETTO]
     * SET VCOMP RAMPUP <ms 1-5>            [PROTETTO]
     *
     * Compensazione dinamica tensione PSU (NUOVO 2026-07-22, solo modalità
     * SW non-QCW e senza SETPOINT HW attivo — vedi psu_voltage_comp.c lato
     * AMC e il gating in task_comms.c): di default la tensione PSU resta al
     * valore massimo (SET PSU VOLTAGE, Vmax) per reggere il transitorio di
     * corrente all'accensione laser; se abilitata (ON), dopo DELAY ms di
     * assestamento (verificato via LASE_Q_OUT_5V) la tensione scende
     * linearmente in RAMP ms verso il target letto dalla LUT
     * corrente->tensione (vedi SET LUT VCOMP sotto). Un aumento di setpoint
     * a laser acceso richiede invece che la tensione risalga velocemente
     * (RAMPUP ms) PRIMA che il nuovo setpoint, più alto, sia inoltrato a
     * LaseQ — altrimenti il regolatore lineare rischia saturazione/dropout.
     * Allo spegnimento del laser la tensione torna ISTANTANEAMENTE a Vmax
     * (nessuna rampa).
     *
     * Consentito SOLO in stato IDLE, stesso vincolo di SET PSU VOLTAGE|
     * CURRENT (flag/timing inviati ad AMC nello stesso frame CONFIG_SET dei
     * limiti PSU, vedi AMC_SendConfig()): scritto in g_config e re-inviato
     * subito tramite TaskAmc_RequestPSUConfigResend() (applicato entro
     * LQ_TRANSMIT_WINDOW = 20ms dal task_amc.c). Persiste in flash solo
     * dopo un successivo SAVE CONFIG.
     * ------------------------------------------------------------------ */
    } else if (strcmp(tok[1], "VCOMP") == 0) {
        REQUIRE_AUTH();
        if (n < 3) {
            err("USAGE: SET VCOMP ON|OFF | SET VCOMP DELAY|RAMP|RAMPUP <ms>");
            return;
        }
        if (FSM_GetState() != SYS_IDLE) {
            err_state("SET VCOMP", "IDLE");
            return;
        }

        if (strcmp(tok[2], "ON") == 0 || strcmp(tok[2], "OFF") == 0) {
            g_config.voltage_comp_enabled = (strcmp(tok[2], "ON") == 0) ? 1U : 0U;
            TaskAmc_RequestPSUConfigResend();
            snprintf(s_rsp_buf, sizeof(s_rsp_buf),
                     "OK VCOMP:%s (inviato ad AMC, SAVE CONFIG per persistere)\r\n",
                     g_config.voltage_comp_enabled ? "ON" : "OFF");
            send_response(s_rsp_buf);
            return;
        }

        if (n < 4) { err("USAGE: SET VCOMP DELAY|RAMP|RAMPUP <ms>"); return; }
        int ms = atoi(tok[3]);

        if (strcmp(tok[2], "DELAY") == 0) {
            if (ms < 10 || ms > 50) { err("DELAY: 10-50 ms"); return; }
            g_config.comp_stabilize_delay_ms = (uint8_t)ms;
        } else if (strcmp(tok[2], "RAMP") == 0) {
            if (ms < 50 || ms > 100) { err("RAMP: 50-100 ms"); return; }
            g_config.comp_ramp_duration_ms = (uint8_t)ms;
        } else if (strcmp(tok[2], "RAMPUP") == 0) {
            if (ms < 1 || ms > 5) { err("RAMPUP: 1-5 ms"); return; }
            g_config.comp_ramp_up_duration_ms = (uint8_t)ms;
        } else {
            err("VCOMP subcmd: ON|OFF|DELAY|RAMP|RAMPUP");
            return;
        }

        TaskAmc_RequestPSUConfigResend();
        snprintf(s_rsp_buf, sizeof(s_rsp_buf),
                 "OK VCOMP %s:%dms (inviato ad AMC, SAVE CONFIG per persistere)\r\n",
                 tok[2], ms);
        send_response(s_rsp_buf);

    /* ------------------------------------------------------------------
     * SET CONTACTOR MASK <0-3>  [PROTETTO]
     *
     * Quali contattori sono fisicamente collegati (bit0=CONTACTOR1,
     * bit1=CONTACTOR2) — vedi Contactors.c: un contattore mascherato (bit a
     * 0) non viene mai acceso (ContactorTurnOn() diventa no-op sul GPIO,
     * a prescindere dal chiamante) e il suo pin di stato ausiliario non
     * viene mai letto (ContactorGetStatus() ritorna sempre status=false
     * senza toccare il GPIO, evitando di propagare un valore indefinito
     * da un pin verosimilmente floating/non cablato).
     *
     * Consentito SOLO in stato IDLE (contattori aperti): scrive g_config
     * direttamente, serve SAVE CONFIG per persistere in flash — stesso
     * schema di SET PSU MASK.
     * ------------------------------------------------------------------ */
    } else if (strcmp(tok[1], "CONTACTOR") == 0) {
        REQUIRE_AUTH();
        if (n < 4) { err("USAGE: SET CONTACTOR MASK <0-3>"); return; }

        if (FSM_GetState() != SYS_IDLE) {
            err_state("SET CONTACTOR", "IDLE");
            return;
        }

        if (strcmp(tok[2], "MASK") != 0) { err("target: MASK"); return; }

        int mask = atoi(tok[3]);
        if (mask < 0 || mask > 0x03) { err("mask: 0-3 (bit0=CONTACTOR1 bit1=CONTACTOR2)"); return; }
        g_config.contactor_enabled_mask = (uint8_t)mask;

        snprintf(s_rsp_buf, sizeof(s_rsp_buf),
                 "OK CONTACTOR MASK:0x%02X (SAVE CONFIG per persistere)\r\n",
                 (unsigned)mask);
        send_response(s_rsp_buf);

    /* ------------------------------------------------------------------
     * SET EFUSE MASK <0-15>  [PROTETTO]
     *
     * Quali eFuse restano alimentati (bit0=MAIN/scheda madre, bit1=SAB,
     * bit2=COM interface, bit3=LASEQ — stessa numerazione di EFuse_id_t,
     * eFuse.h). Un modulo mascherato (bit a 0) resta SEMPRE con SHDN
     * asserito (spento) fin dal boot — EFuse_Init() legge la maschera una
     * sola volta, PRIMA di qualunque comunicazione verso quel modulo.
     *
     * A DIFFERENZA di SET PSU/CONTACTOR MASK: qui NON c'è alcun effetto
     * immediato. Un modulo già acceso in questa sessione NON viene spento
     * scrivendo questo comando — serve SAVE CONFIG seguito da un
     * power-cycle (o reset) perché EFuse_Init() rilegga la nuova maschera.
     * Il comando aggiorna comunque subito g_config (RAM), così un
     * successivo GET CONFIG/SAVE CONFIG riflette il nuovo valore.
     *
     * ATTENZIONE bit0 (MAIN): è l'eFuse principale 24V/48V della scheda
     * madre, a valle del quale sta la maggior parte del sistema — mascherarlo
     * è una scelta drastica (macchina di fatto non alimentata a valle),
     * lasciata comunque disponibile perché richiesta esplicitamente.
     * ------------------------------------------------------------------ */
    } else if (strcmp(tok[1], "EFUSE") == 0) {
        REQUIRE_AUTH();
        if (n < 4) { err("USAGE: SET EFUSE MASK <0-15>"); return; }
        if (strcmp(tok[2], "MASK") != 0) { err("target: MASK"); return; }

        int mask = atoi(tok[3]);
        if (mask < 0 || mask > 0x0F) {
            err("mask: 0-15 (bit0=MAIN bit1=SAB bit2=COM bit3=LASEQ)");
            return;
        }
        g_config.efuse_enabled_mask = (uint32_t)mask;

        snprintf(s_rsp_buf, sizeof(s_rsp_buf),
                 "OK EFUSE MASK:0x%02X (SAVE CONFIG + power-cycle per applicare)\r\n",
                 (unsigned)mask);
        send_response(s_rsp_buf);

    /* ------------------------------------------------------------------
     * SET PD MASK <0-15>  [PROTETTO]
     *
     * "Photodiode mask" — riusa g_config.pd_mask (bit0=PD1..bit3=PD4),
     * già esistente e inviato ad AMC via MSG_CONFIG_PD all'avvio (vedi
     * task_comms.c). Stesso pattern di SET PSU MASK / SET CONTACTOR MASK:
     * un PD disabilitato viene azzerato lato MMC immediatamente (vedi
     * task_comms.c, s_pd_power_w) e il mask va ri-applicato lato AMC con un
     * riavvio (non esiste un comando AMC dedicato di solo aggiornamento
     * mask, a differenza di CONFIG_SET per PSU — vedi AMC_SendPDConfig()).
     * Consentito SOLO in stato IDLE, come PSU/CONTACTOR.
     * ------------------------------------------------------------------ */
    } else if (strcmp(tok[1], "PD") == 0) {
        REQUIRE_AUTH();
        if (n < 4) { err("USAGE: SET PD MASK <0-15>"); return; }

        if (FSM_GetState() != SYS_IDLE) {
            err_state("SET PD", "IDLE");
            return;
        }

        if (strcmp(tok[2], "MASK") != 0) { err("target: MASK"); return; }

        int mask = atoi(tok[3]);
        if (mask < 0 || mask > 0x0F) { err("mask: 0-15 (bit0=PD1..bit3=PD4)"); return; }
        g_config.pd_mask = (uint8_t)mask;

        snprintf(s_rsp_buf, sizeof(s_rsp_buf),
                 "OK PD MASK:0x%02X (SAVE CONFIG per persistere, riavvio per applicare lato AMC)\r\n",
                 (unsigned)mask);
        send_response(s_rsp_buf);

    /* ------------------------------------------------------------------
     * SET FAULT MASK <num>       [PROTETTO]
     * SET FAULT LATCHMASK <num>  [PROTETTO]
     * SET FAULT HEX|DEC|BIN      (libero)
     *
     * Categoria SEPARATA da SET ERR/SET WARN: fault di comunicazione/
     * timeout interni (COM interface, LaseQ, AMC) — vedi FAULT_BIT_* in
     * task_monitor.h. Stesso schema esatto di SET ERR/SET WARN.
     *
     * LATCHMASK (dal 2026-07-14, vedi banner "FAULT LATCH" in
     * task_monitor.h): sottoinsieme di FAULT_BIT_* che, quando attivo,
     * viene "latchato" in g_config.fault_latch_active — persiste al
     * power-cycle (Config_Save()) e forza INIT → FAULT diretto al prossimo
     * boot finché non si invia "FRST" (vedi cmd_frst()). Bit NON presenti
     * in LATCHMASK si comportano come prima: SYS_FAULT si risolve con un
     * semplice power reset. Default: solo FLOOD1/FLOOD2 (vedi
     * Config_LoadDefaults()).
     * ------------------------------------------------------------------ */
    } else if (strcmp(tok[1], "FAULT") == 0) {
        if (n < 3) { err("USAGE: SET FAULT HEX|DEC|BIN|MASK|LATCHMASK <num>"); return; }

        if (strcmp(tok[2], "MASK") == 0) {
            REQUIRE_AUTH();
            if (n < 4) { err("USAGE: SET FAULT MASK <num>"); return; }
            uint32_t val;
            if (!parse_number(tok[3], &val)) { err("formato: 0x../B../dec (case-insensitive)"); return; }
            g_config.fault_mask = val;
            char vs[36]; format_bitmask(val, NUM_FMT_HEX, vs, sizeof(vs));
            snprintf(s_rsp_buf, sizeof(s_rsp_buf), "OK FAULT MASK:%s\r\n", vs);
            send_response(s_rsp_buf);
        }
        else if (strcmp(tok[2], "LATCHMASK") == 0) {
            REQUIRE_AUTH();
            if (n < 4) { err("USAGE: SET FAULT LATCHMASK <num>"); return; }
            uint32_t val;
            if (!parse_number(tok[3], &val)) { err("formato: 0x../B../dec (case-insensitive)"); return; }
            g_config.fault_latch_mask = val;
            char vs[36]; format_bitmask(val, NUM_FMT_HEX, vs, sizeof(vs));
            snprintf(s_rsp_buf, sizeof(s_rsp_buf), "OK FAULT LATCHMASK:%s\r\n", vs);
            send_response(s_rsp_buf);
        }
        else if (strcmp(tok[2], "HEX") == 0) { s_fault_fmt = NUM_FMT_HEX; ok("FAULT fmt: HEX"); }
        else if (strcmp(tok[2], "DEC") == 0) { s_fault_fmt = NUM_FMT_DEC; ok("FAULT fmt: DEC"); }
        else if (strcmp(tok[2], "BIN") == 0) { s_fault_fmt = NUM_FMT_BIN; ok("FAULT fmt: BIN"); }
        else { err("SET FAULT: HEX|DEC|BIN|MASK|LATCHMASK <num>"); }

    /* ------------------------------------------------------------------
     * SET FREQ <hz>  (nessun login, dal 2026-07-20)
     *
     * Frequenza di impulsazione QCW [Hz] — vedi Drivers/QCW/QCW.h.
     * Applicata subito se QCW è già in corso (QCW_SetParams(), senza
     * glitch: la fase in corso completa con i parametri precedenti).
     * ------------------------------------------------------------------ */
    } else if (strcmp(tok[1], "FREQ") == 0) {
        if (n < 3) { err("USAGE: SET FREQ <hz>"); return; }
        int hz = atoi(tok[2]);
        if (hz < (int)QCW_MIN_FREQ_HZ || hz > (int)QCW_MAX_FREQ_HZ) {
            snprintf(s_rsp_buf, sizeof(s_rsp_buf),
                     "ERR hz: %u-%u\r\n", (unsigned)QCW_MIN_FREQ_HZ, (unsigned)QCW_MAX_FREQ_HZ);
            send_response(s_rsp_buf);
            return;
        }
        g_config.qcw_freq_hz = (uint16_t)hz;
        /*
         * Vincolo Ton >= QCW_MIN_ON_TIME_US (dal 2026-08-02, vedi QCW.h):
         * il duty gia' configurato (g_config.qcw_duty_pct) potrebbe non
         * garantire piu' un Ton sufficiente alla NUOVA frequenza (es. duty
         * 1% valido a bassa freq, ma a 50000Hz darebbe un Ton ~0.2us). Clamp
         * automatico del duty, freq_hz non toccata.
         */
        g_config.qcw_duty_pct = QCW_ClampDutyForMinOnTime(g_config.qcw_freq_hz, g_config.qcw_duty_pct);
        if (QCW_IsActive()) { QCW_SetParams(g_config.qcw_freq_hz, g_config.qcw_duty_pct); }
        snprintf(s_rsp_buf, sizeof(s_rsp_buf), "OK FREQ:%dHz DUTY:%u%%\r\n",
                 hz, (unsigned)g_config.qcw_duty_pct);
        send_response(s_rsp_buf);

    /* ------------------------------------------------------------------
     * SET DUTY <pct>  (nessun login, dal 2026-07-20)
     *
     * Duty cycle di impulsazione QCW [%] — vedi Drivers/QCW/QCW.h.
     * ------------------------------------------------------------------ */
    } else if (strcmp(tok[1], "DUTY") == 0) {
        if (n < 3) { err("USAGE: SET DUTY <pct>"); return; }
        int pct = atoi(tok[2]);
        if (pct < (int)QCW_MIN_DUTY_PCT || pct > (int)QCW_MAX_DUTY_PCT) {
            snprintf(s_rsp_buf, sizeof(s_rsp_buf),
                     "ERR pct: %u-%u\r\n", (unsigned)QCW_MIN_DUTY_PCT, (unsigned)QCW_MAX_DUTY_PCT);
            send_response(s_rsp_buf);
            return;
        }
        /*
         * Vincolo Ton >= QCW_MIN_ON_TIME_US (dal 2026-08-02, vedi QCW.h):
         * il pct richiesto e' nominalmente valido (1-99%) ma alla frequenza
         * GIA' configurata (g_config.qcw_freq_hz) potrebbe dare un Ton
         * troppo corto — clamp automatico verso l'alto in quel caso.
         */
        g_config.qcw_duty_pct = QCW_ClampDutyForMinOnTime(g_config.qcw_freq_hz, (uint8_t)pct);
        if (QCW_IsActive()) { QCW_SetParams(g_config.qcw_freq_hz, g_config.qcw_duty_pct); }
        snprintf(s_rsp_buf, sizeof(s_rsp_buf), "OK DUTY:%u%%\r\n", (unsigned)g_config.qcw_duty_pct);
        send_response(s_rsp_buf);

    /* ------------------------------------------------------------------
     * SET QCW ON|OFF  (nessun login, dal 2026-07-20)
     *
     * Attiva/disattiva la modalità QCW per le emissioni successive (e per
     * quella IN CORSO se il sistema è già in SYS_EMISSION, FSM_MODE_SW):
     *   ON  → se già in EMISSION in CW, passa immediatamente a impulsato
     *         (QCW_SetParams() + QCW_Start(), gate SW chiuso prima di
     *         riaprirlo impulsato per evitare una fase ON prolungata spuria).
     *   OFF → se già in EMISSION in QCW, ferma l'impulsazione e torna
     *         immediatamente a CW (gate riaperto staticamente).
     * In FSM_MODE_ANALOG il flag viene comunque salvato ma non ha effetto
     * (il gate SW non è usato in ANALOG, vedi action_enter_emission()).
     * ------------------------------------------------------------------ */
    } else if (strcmp(tok[1], "QCW") == 0) {
        if (n < 3) { err("USAGE: SET QCW ON|OFF"); return; }

        bool want_on;
        if      (strcmp(tok[2], "ON")  == 0) want_on = true;
        else if (strcmp(tok[2], "OFF") == 0) want_on = false;
        else { err("QCW: ON|OFF"); return; }

        g_config.qcw_enabled = want_on ? 1U : 0U;

        bool live = (FSM_GetState() == SYS_EMISSION) && (FSM_GetMode() == FSM_MODE_SW);
        if (live) {
            if (want_on) {
                QCW_SetParams(g_config.qcw_freq_hz, g_config.qcw_duty_pct);
                QCW_Start();      /* QCW_Start() è no-op se già attivo */
            } else {
                QCW_Stop();       /* ferma il timer e chiude il gate */
                BoardCtrl_GateMC_Open(); /* torna subito a CW: emissione resta attiva */
            }
        }

        snprintf(s_rsp_buf, sizeof(s_rsp_buf), "OK QCW:%s\r\n", want_on ? "ON" : "OFF");
        send_response(s_rsp_buf);

    /* ------------------------------------------------------------------
     * SET PASSWORD <old> <new>  [PROTETTO]
     *
     * La password è memorizzata in RAM, non persiste tra i riavvii.
     * Lunghezza: 4-15 caratteri. Case-insensitive (str_upper applicata).
     * ------------------------------------------------------------------ */
    } else if (strcmp(tok[1], "PASSWORD") == 0) {
        REQUIRE_AUTH();
        if (n < 4) { err("USAGE: SET PASSWORD <old> <new>"); return; }
        if (strcmp(tok[2], s_password) != 0) { err("wrong old password"); return; }
        size_t nl = strlen(tok[3]);
        if (nl < 4U || nl > 15U) { err("new password: 4-15 chars"); return; }
        memcpy(s_password, tok[3], nl + 1U);
        ok("password changed");

    /* ------------------------------------------------------------------
     * SET TIME HH:MM:SS
     * (non richiede login — l'operatore può impostare l'ora)
     * ------------------------------------------------------------------ */
    } else if (strcmp(tok[1], "TIME") == 0) {
        if (n < 3) { err("USAGE: SET TIME HH:MM:SS"); return; }
        unsigned hh, mm, ss;
        /* str_upper non altera ':' né i digit */
        if (sscanf(tok[2], "%u:%u:%u", &hh, &mm, &ss) != 3 ||
            hh > 23U || mm > 59U || ss > 59U) {
            err("format: HH:MM:SS"); return;
        }
        RTC_TimeTypeDef t = {0};
        t.Hours   = (uint8_t)hh;
        t.Minutes = (uint8_t)mm;
        t.Seconds = (uint8_t)ss;
        HAL_RTC_SetTime(&hrtc, &t, RTC_FORMAT_BIN);
        snprintf(s_rsp_buf, sizeof(s_rsp_buf), "OK TIME:%02u:%02u:%02u\r\n", hh, mm, ss);
        send_response(s_rsp_buf);

    /* ------------------------------------------------------------------
     * SET DATE DD/MM/YYYY
     * (non richiede login)
     * ------------------------------------------------------------------ */
    } else if (strcmp(tok[1], "DATE") == 0) {
        if (n < 3) { err("USAGE: SET DATE DD/MM/YYYY"); return; }
        unsigned dd, mo, yy;
        if (sscanf(tok[2], "%u/%u/%u", &dd, &mo, &yy) != 3 ||
            dd < 1U || dd > 31U || mo < 1U || mo > 12U ||
            yy < 2000U || yy > 2099U) {
            err("format: DD/MM/YYYY (2000-2099)"); return;
        }
        RTC_DateTypeDef d = {0};
        d.Date  = (uint8_t)dd;
        d.Month = (uint8_t)mo;
        d.Year  = (uint8_t)(yy - 2000U);
        HAL_RTC_SetDate(&hrtc, &d, RTC_FORMAT_BIN);
        snprintf(s_rsp_buf, sizeof(s_rsp_buf), "OK DATE:%02u/%02u/%04u\r\n", dd, mo, yy);
        send_response(s_rsp_buf);

    /* ------------------------------------------------------------------
     * SET TERM ON|OFF
     * (non richiede login — vedi nEXT_485_TERMINATION_Pin, PA8, active LOW)
     *
     * Inserisce/rimuove la resistenza di terminazione del bus RS485.
     * Persistita subito in flash (Config_Save()): a differenza di altri
     * parametri di SET, questo pin è rilevante per l'integrità elettrica
     * del bus fin dal prossimo boot, quindi non si affida al comando
     * separato "SAVE CONFIG" (che richiede login).
     * ------------------------------------------------------------------ */
    } else if (strcmp(tok[1], "TERM") == 0) {
        if (n < 3) { err("USAGE: SET TERM ON|OFF"); return; }
        bool inserted;
        if      (strcmp(tok[2], "ON")  == 0) inserted = true;
        else if (strcmp(tok[2], "OFF") == 0) inserted = false;
        else { err("TERM: ON|OFF"); return; }

        g_config.termination = inserted ? 1U : 0U;
        BoardCtrl_SetTermination(inserted);

        Config_err_t e = Config_Save();
        if (e == CONFIG_OK) {
            snprintf(s_rsp_buf, sizeof(s_rsp_buf), "OK TERM:%s\r\n",
                     inserted ? "ON" : "OFF");
        } else {
            snprintf(s_rsp_buf, sizeof(s_rsp_buf),
                     "OK TERM:%s (WARN: save failed, code %d)\r\n",
                     inserted ? "ON" : "OFF", (int)e);
        }
        send_response(s_rsp_buf);

    /* ------------------------------------------------------------------
     * SET SETPOINTCOMP ON|OFF
     * (non richiede login — vedi HW_SETPOINT_SEL_Pin, PF14, active HIGH,
     * BoardCtrl_SetpointSel())
     *
     * Forza la selezione della sorgente del setpoint analogico verso LaseQ:
     *   OFF -> PF14=0: setpoint da EXT/utente (LPWR_SET_ISO), default sicuro
     *   ON  -> PF14=1: setpoint dal DAC di AMC (segnale corretto/linearizzato)
     * Applicato SUBITO sul pin (come "SET TERM ON|OFF") e persistito subito
     * in flash (g_config.hw_setpoint_sel, Config_Save()) — non in attesa di
     * "SAVE CONFIG". Aggiorna anche la copia live s_hw_setpoint_sel in fsm.c
     * (FSM_SetHwSetpointSel()) usata da action_enter_emission() e dal
     * heartbeat verso AMC (task_amc.c): senza questo secondo aggiornamento,
     * il pin scritto qui verrebbe silenziosamente sovrascritto al prossimo
     * ingresso in SYS_EMISSION con il vecchio valore letto da FSM_Init().
     *
     * NOTA: in FSM_MODE_SW puro (senza il toggle "SET SETPOINTHW ON",
     * "HYBRID2") action_enter_emission() forza comunque PF14=0 all'ingresso
     * in emissione, indipendentemente da questa impostazione — vedi
     * BoardCtrl_SetpointSel(s_setpoint_hw_enabled && (s_hw_setpoint_sel != 0))
     * in fsm.c. SETPOINTCOMP ha effetto pieno in FSM_MODE_ANALOG o in SW
     * con SETPOINTHW attivo.
     * ------------------------------------------------------------------ */
    } else if (strcmp(tok[1], "SETPOINTCOMP") == 0) {
        if (n < 3) { err("USAGE: SET SETPOINTCOMP ON|OFF"); return; }
        bool use_amc_dac;
        if      (strcmp(tok[2], "ON")  == 0) use_amc_dac = true;
        else if (strcmp(tok[2], "OFF") == 0) use_amc_dac = false;
        else { err("SETPOINTCOMP: ON|OFF"); return; }

        g_config.hw_setpoint_sel = use_amc_dac ? 1U : 0U;
        BoardCtrl_SetpointSel(use_amc_dac);
        FSM_SetHwSetpointSel(g_config.hw_setpoint_sel);

        Config_err_t e_spc = Config_Save();
        if (e_spc == CONFIG_OK) {
            snprintf(s_rsp_buf, sizeof(s_rsp_buf), "OK SETPOINTCOMP:%s\r\n",
                     use_amc_dac ? "ON" : "OFF");
        } else {
            snprintf(s_rsp_buf, sizeof(s_rsp_buf),
                     "OK SETPOINTCOMP:%s (WARN: save failed, code %d)\r\n",
                     use_amc_dac ? "ON" : "OFF", (int)e_spc);
        }
        send_response(s_rsp_buf);

    /* ------------------------------------------------------------------
     * SET NTC MAP <serigrafia 1-16> <SENSOR|OFF>  [PROTETTO]
     *
     * Associa la serigrafia PCB <serigrafia> (numero stampato sulla scheda,
     * 1-16 — vedi s_ntc_serigrafia_to_ch sopra per la conversione fissa
     * verso il canale AD7490 reale) al sensore indicato.
     * OFF (o 0xFF) disabilita il canale.
     * Sensori validi: WATER_IN WATER_OUT SPLICE DIODE1 DIODE2 AMBIENT
     * (più gli id generici NTC8..NTC13/NTC16 e NTC_GEN2, più PSU_TEMP/
     * PWR_EL_TEMP — NUOVO 2026-07-22 — per serigrafie senza uso funzionale
     * ancora assegnato).
     *
     * Esempio (mappatura di fabbrica dal 2026-07-15):
     *   SET NTC MAP 3 DIODE1     → serigrafia 3 (CH12) = temperatura diodo 1
     *   SET NTC MAP 4 DIODE2     → serigrafia 4 (CH13) = temperatura diodo 2
     *   SET NTC MAP 16 SPLICE    → serigrafia 16 (CH0) = temperatura splice
     *   SET NTC MAP 7 OFF        → serigrafia 7 disabilitata
     *
     * NOTA: prima del 2026-07-15 l'argomento era il canale AD7490 grezzo
     * (0-15); ora è il numero di serigrafia (1-16) leggibile sulla scheda —
     * comando NON retrocompatibile a livello di argomento numerico, solo il
     * formato del comando resta lo stesso.
     *
     * SET NTC BETA <serigrafia 1-16> <beta>  [PROTETTO], NUOVO 2026-07-22
     *
     * Coefficiente Beta del modello NTC (task_monitor.c,
     * update_ntc_readings(): T = 1/(1/298.15 + ln(R/R0)/beta)), ora
     * configurabile PER CANALE (g_config.ntc_beta[16], prima era uno
     * scalare unico condiviso da tutti gli NTC) — utile quando serigrafie
     * diverse montano componenti NTC fisicamente diversi (beta tipico
     * 2000-6000K). Come per MAP, l'argomento è il numero di serigrafia
     * (1-16): il mapping verso il canale AD7490 (s_ntc_serigrafia_to_ch) è
     * implicito, non serve conoscere il canale interno. R0/Rseries
     * (g_config.ntc_r0_ohm/ntc_rseries_ohm) restano invece scalari globali,
     * non per-canale.
     * ------------------------------------------------------------------ */
    } else if (strcmp(tok[1], "NTC") == 0) {
        REQUIRE_AUTH();
        if (n < 5 || (strcmp(tok[2], "MAP") != 0 && strcmp(tok[2], "BETA") != 0)) {
            err("USAGE: SET NTC MAP <serigrafia 1-16> <sensor|OFF> | SET NTC BETA <serigrafia 1-16> <beta>");
            return;
        }
        int serigrafia = atoi(tok[3]);
        if (serigrafia < 1 || serigrafia > (int)AD7490_NUM_CHANNELS) {
            err("serigrafia: 1-16"); return;
        }
        uint8_t ch = s_ntc_serigrafia_to_ch[serigrafia - 1];

        if (strcmp(tok[2], "MAP") == 0) {
            uint8_t sid = 0xFFU;
            if (strcmp(tok[4], "OFF") != 0) {
                bool found = false;
                for (uint8_t s = 0U; s < NTC_NUM_SENSORS; s++) {
                    if (strcmp(tok[4], s_ntc_sensor_names[s]) == 0) {
                        sid = s; found = true; break;
                    }
                }
                if (!found) {
                    err("sensor: WATER_IN|WATER_OUT|SPLICE|DIODE1|DIODE2|AMBIENT|PSU_TEMP|PWR_EL_TEMP|NTC8..NTC13|NTC16|NTC_GEN2|OFF");
                    return;
                }
            }

            g_config.ntc_ch_map[ch] = sid;
            const char *name = (sid < NTC_NUM_SENSORS) ? s_ntc_sensor_names[sid] : "OFF";
            snprintf(s_rsp_buf, sizeof(s_rsp_buf),
                     "OK NTC%02d(CH%02u) -> %s\r\n", serigrafia, (unsigned)ch, name);
            send_response(s_rsp_buf);

        } else { /* BETA */
            int beta = atoi(tok[4]);
            if (beta < 2000 || beta > 6000) { err("beta: 2000-6000 K"); return; }

            g_config.ntc_beta[ch] = (uint16_t)beta;
            snprintf(s_rsp_buf, sizeof(s_rsp_buf),
                     "OK NTC%02d(CH%02u) BETA:%d (SAVE CONFIG per persistere)\r\n",
                     serigrafia, (unsigned)ch, beta);
            send_response(s_rsp_buf);
        }

    } else {
        err("SET subcmd: MODE|GATEHW|SETPOINTHW|SETPOINTCOMP|SETPOINT|SETPOINT RANGE|CURRENT|LUT|ERR|WARN|FAULT|TEMP|NTC|FLOW|PD|HUM|DEW|DELAY|PSU|VCOMP|CONTACTOR|EFUSE|FREQ|DUTY|QCW|PASSWORD|TIME|DATE|TERM");
    }
}

/* ========================================================================== */
/* GESTORI COMANDI — GET                                                       */
/* ========================================================================== */

static void cmd_get(char **tok, int n)
{
    if (n < 2) { err("USAGE: GET STATE|MODE|REGIME|STATUS|POWER|ERR|WARN|FAULT|ALARM|FAULTS|LQERR|LQ|SAB|MODULES|DEW|TEMP|NTC|FLOW|CONFIG|THRESHOLDS|LUT|TIME|DATE|TERM|SETPOINTCOMP|SETPOINT RANGE|FW"); return; }

    if (strcmp(tok[1], "STATE") == 0) {
        snprintf(s_rsp_buf, sizeof(s_rsp_buf), "OK STATE:%s\r\n",
                 Rs485Cmd_StateStr(FSM_GetState()));
        send_response(s_rsp_buf);

    } else if (strcmp(tok[1], "MODE") == 0) {
        /* Canale di controllo attivo (s_mode, fsm.c) — SW o ANALOG, già
         * riportato anche dentro "GET STATUS" ma qui isolato per una
         * lettura rapida (stesso valore rispecchiato su nCMD_RDY_iso: 1=SW,
         * 0=ANALOG — vedi sync_cmd_rdy_status(), fsm.c). */
        snprintf(s_rsp_buf, sizeof(s_rsp_buf), "OK MODE:%s\r\n",
                 (FSM_GetMode() == FSM_MODE_ANALOG) ? "ANALOG" : "SW");
        send_response(s_rsp_buf);

    } else if (strcmp(tok[1], "REGIME") == 0) {
        /* Regime di emissione CONFIGURATO (g_config.qcw_enabled, "SET QCW
         * ON|OFF") — QCW se impulsato, CW altrimenti. Non richiede che
         * un'emissione sia in corso (a differenza di QCW_IsActive(), che
         * riflette anche lo stato live SYS_EMISSION — vedi task_amc.c). */
        snprintf(s_rsp_buf, sizeof(s_rsp_buf), "OK REGIME:%s\r\n",
                 g_config.qcw_enabled ? "QCW" : "CW");
        send_response(s_rsp_buf);

    } else if (strcmp(tok[1], "STATUS") == 0) {
        /* FSM_Mode_t non ha più un valore HYBRID dedicato (vedi fsm.h): SW e
         * ANALOG restano le uniche due modalità, i toggle indipendenti
         * GATEHW/SETPOINTHW ("HYBRID1"/"HYBRID2", validi solo in SW) sono
         * riportati a parte per non perdere visibilità sullo stato reale. */
        const char *mode_lut[] = {"SW", "ANALOG"};
        FSM_Mode_t m = FSM_GetMode();
        /*
         * NUOVO 2026-08-01: in ANALOG, oppure in SW con SETPOINT HW
         * ("HYBRID2") attivo — FSM_GetLaserModeWire() != 0, stessa condizione
         * usata per il byte laser_mode verso AMC — il setpoint realmente in
         * vigore è quello hardware (pin PC5/LPWR_SET_ISO, EXT_interface.c),
         * non il setpoint software. Lo riportiamo qui convertito in %
         * tramite la LUT di calibrazione dedicata (LUT_ConvertHwPowerToPct(),
         * lut_manager.h) invece di Setpoint_GetPct()/GetCurrentMa() — CUR è
         * il corrispondente in mA tramite la LUT SETPOINT esistente, solo
         * per coerenza di visualizzazione (LaseQ non riceve questo valore da
         * qui: resta pilotato dal segnale analogico diretto, percorso di
         * controllo invariato). Il ramo "DIR"/setpoint software sotto resta
         * IDENTICO per tutti gli altri casi (SW senza SETPOINT HW).
         */
        if (FSM_GetLaserModeWire() != 0U) {
            uint8_t  hw_pct = LUT_ConvertHwPowerToPct(EXT_GetPowerSetpointRaw());
            uint32_t hw_ma  = Setpoint_PctToCurrentMa(hw_pct);
            snprintf(s_rsp_buf, sizeof(s_rsp_buf),
                     "OK STATE:%s MODE:%s GATEHW:%d SETPHW:%d SP:%u%% CUR:%lumA\r\n",
                     Rs485Cmd_StateStr(FSM_GetState()),
                     ((unsigned)m < 2U) ? mode_lut[m] : "?",
                     (int)FSM_GetGateHwEnabled(), (int)FSM_GetSetpointHwEnabled(),
                     (unsigned)hw_pct, (unsigned long)hw_ma);

        /* Setpoint_GetPct() non riflette la corrente erogata quando il
         * setpoint è stato impostato via SET CURRENT (bypass LUT, vedi
         * Setpoint_IsDirectMode()): mostriamo "DIR" invece di un valore
         * percentuale stantio/fuorviante. */
        } else if (Setpoint_IsDirectMode()) {
            snprintf(s_rsp_buf, sizeof(s_rsp_buf),
                     "OK STATE:%s MODE:%s GATEHW:%d SETPHW:%d SP:DIR CUR:%lumA\r\n",
                     Rs485Cmd_StateStr(FSM_GetState()),
                     ((unsigned)m < 2U) ? mode_lut[m] : "?",
                     (int)FSM_GetGateHwEnabled(), (int)FSM_GetSetpointHwEnabled(),
                     (unsigned long)Setpoint_GetCurrentMa());
        } else {
            snprintf(s_rsp_buf, sizeof(s_rsp_buf),
                     "OK STATE:%s MODE:%s GATEHW:%d SETPHW:%d SP:%u%% CUR:%lumA\r\n",
                     Rs485Cmd_StateStr(FSM_GetState()),
                     ((unsigned)m < 2U) ? mode_lut[m] : "?",
                     (int)FSM_GetGateHwEnabled(), (int)FSM_GetSetpointHwEnabled(),
                     (unsigned)Setpoint_GetPct(),
                     (unsigned long)Setpoint_GetCurrentMa());
        }
        send_response(s_rsp_buf);

    } else if (strcmp(tok[1], "POWER") == 0) {
        uint16_t pw[4] = {0U};
        TaskAmc_GetPDPower(pw);
        snprintf(s_rsp_buf, sizeof(s_rsp_buf),
                 "OK PD0:%u PD1:%u PD2:%u PD3:%u\r\n",
                 (unsigned)pw[0], (unsigned)pw[1],
                 (unsigned)pw[2], (unsigned)pw[3]);
        send_response(s_rsp_buf);

    } else if (strcmp(tok[1], "ERR") == 0) {
        /*
         * BUGFIX (2026-07-14): prima "GET ERR MASK" veniva silenziosamente
         * interpretato come "GET ERR" (tok[2] ignorato) — rispondeva con gli
         * errori ATTUALMENTE ATTIVI (TaskMonitor_GetErrors()) invece della
         * maschera configurata (g_config.error_mask), fuorviante per chi
         * vuole verificare cosa ha appena impostato con "SET ERR MASK".
         * Aggiunta variante esplicita, stesso schema di "GET NTC MASK".
         */
        if (n >= 3 && strcmp(tok[2], "MASK") == 0) {
            char vs[36];
            format_bitmask(g_config.error_mask, s_err_fmt, vs, sizeof(vs));
            snprintf(s_rsp_buf, sizeof(s_rsp_buf), "OK ERR MASK:%s\r\n", vs);
            send_response(s_rsp_buf);
        } else {
            char vs[36];
            format_bitmask(TaskMonitor_GetErrors(), s_err_fmt, vs, sizeof(vs));
            snprintf(s_rsp_buf, sizeof(s_rsp_buf), "OK ERR:%s\r\n", vs);
            send_response(s_rsp_buf);
        }

    } else if (strcmp(tok[1], "WARN") == 0) {
        /* BUGFIX (2026-07-14): vedi "GET ERR MASK" sopra, stesso problema. */
        if (n >= 3 && strcmp(tok[2], "MASK") == 0) {
            char vs[36];
            format_bitmask(g_config.warning_mask, s_warn_fmt, vs, sizeof(vs));
            snprintf(s_rsp_buf, sizeof(s_rsp_buf), "OK WARN MASK:%s\r\n", vs);
            send_response(s_rsp_buf);
        } else {
            char vs[36];
            format_bitmask(TaskMonitor_GetWarnings(), s_warn_fmt, vs, sizeof(vs));
            snprintf(s_rsp_buf, sizeof(s_rsp_buf), "OK WARN:%s\r\n", vs);
            send_response(s_rsp_buf);
        }

    } else if (strcmp(tok[1], "FAULT") == 0) {
        /*
         * BUGFIX (2026-07-14): stesso problema di "GET ERR MASK"/"GET WARN
         * MASK" sopra — "GET FAULT MASK" veniva interpretato come "GET
         * FAULT" (tok[2] ignorato), restituendo l'ultimo evento di fault
         * invece della maschera configurata (g_config.fault_mask).
         */
        if (n >= 3 && strcmp(tok[2], "MASK") == 0) {
            char vs[36];
            format_bitmask(g_config.fault_mask, s_fault_fmt, vs, sizeof(vs));
            snprintf(s_rsp_buf, sizeof(s_rsp_buf), "OK FAULT MASK:%s\r\n", vs);
            send_response(s_rsp_buf);
        } else if (n >= 3 && strcmp(tok[2], "LATCHMASK") == 0) {
            /*
             * Sottoinsieme di FAULT_BIT_* configurato per persistere al
             * power-cycle (g_config.fault_latch_mask) — vedi "SET FAULT
             * LATCHMASK" sopra e banner "FAULT LATCH" in task_monitor.h.
             */
            char vs[36];
            format_bitmask(g_config.fault_latch_mask, s_fault_fmt, vs, sizeof(vs));
            snprintf(s_rsp_buf, sizeof(s_rsp_buf), "OK FAULT LATCHMASK:%s\r\n", vs);
            send_response(s_rsp_buf);
        } else if (n >= 3 && strcmp(tok[2], "LATCH") == 0) {
            /*
             * Bitmask dei fault ATTUALMENTE latchati (g_config.fault_latch_active
             * & g_config.fault_latch_mask) — non vuota finché non arriva un
             * "FRST" (vedi cmd_frst()). Riflette anche cosa ha eventualmente
             * forzato INIT → FAULT diretto in FSM_Init() dopo un riavvio.
             */
            char vs[36];
            format_bitmask(g_config.fault_latch_active & g_config.fault_latch_mask,
                            s_fault_fmt, vs, sizeof(vs));
            snprintf(s_rsp_buf, sizeof(s_rsp_buf), "OK FAULT LATCH:%s\r\n", vs);
            send_response(s_rsp_buf);
        } else {
            /*
             * Ultimo evento che ha causato SYS_FAULT (SOLO non recuperabile —
             * vedi "GET ALARM" sotto per SYS_ERROR), letto a posteriori (a
             * differenza di "LOG ERR ON", che notifica solo nel momento in
             * cui l'evento si verifica). Resta l'ultima causa nota finché non
             * se ne verifica una nuova. "NONE" se nessun fault dall'avvio.
             *
             * BUGFIX (2026-07-14): prima "GET FAULT" poteva rispondere anche
             * con cause di SYS_ERROR (es. "KEY", recuperabile via CERR),
             * fuorviante dato il nome del comando — separato in due registri
             * indipendenti, vedi FSM_GetLastFaultEvent()/GetLastErrorEvent()
             * in fsm.c.
             */
            snprintf(s_rsp_buf, sizeof(s_rsp_buf), "OK FAULT:%s\r\n",
                     FSM_GetLastFaultName());
            send_response(s_rsp_buf);
        }

    } else if (strcmp(tok[1], "ALARM") == 0) {
        /*
         * Ultimo evento che ha causato SYS_ERROR (RECUPERABILE via CERR —
         * es. KEY_A/KEY_B, SAB_TIMEOUT, SAB_INTLCK, FLOW, TEMP, LASEQ_ERROR).
         * Controparte di "GET FAULT" (solo SYS_FAULT, non recuperabile) —
         * nuovo comando dal 2026-07-14, vedi bugfix sopra.
         *
         * DISAMBIGUAZIONE SORGENTE (dal 2026-07-15): alcune cause sono
         * generiche di per sé (un solo evento FSM condiviso da più sorgenti
         * fisiche indipendenti — stesso limite già noto per
         * SYS_LASEQ_FAULT_EVENT/GET LQERR). FSM_GetLastFaultName()/
         * GetLastErrorName() ora restituiscono, quando possibile, la
         * sorgente specifica invece del nome generico:
         *   KEY          -> KEY_A | KEY_B | KEY_A|KEY_B
         *   LID          -> LID1 | LID2 | LID1|LID2 (dal 2026-07-15)
         *   FLOOD        -> FLOOD1 | FLOOD2 | FLOOD1|FLOOD2
         *   PSU          -> PSU1 | PSU2 | PSU1|PSU2
         *   SAB_INTLCK   -> SAB_INTLCK_A | SAB_INTLCK_B | SAB_INTLCK_A|SAB_INTLCK_B
         *                   (dal 2026-07-15 — i due canali sono ridondanza
         *                   dello stesso circuito di sicurezza, non varianti
         *                   di prodotto: mascherare un canale reale va
         *                   riservato al banco prova, vedi task_monitor.h)
         *   SAB_TEST     -> SAB_TEST_A | SAB_TEST_B | SAB_TEST_A|SAB_TEST_B
         *                   (dal 2026-07-15, stesso principio di SAB_INTLCK sopra)
         *   TEMP         -> WATER_IN | WATER_OUT | SPLICE | DIODE1 | DIODE2
         *                   | AMBIENT | LQ_DRIVER | LQ_AMBIENT (uno o più,
         *                   separati da "|" se contemporanei)
         * Vedi event_detail_name() in fsm.c. Ricostruita interrogando lo
         * stato live dei sottosistemi nell'istante in cui l'evento viene
         * processato (action_fault()/action_enter_error()) — se nel
         * frattempo la condizione originale è già rientrata (raro), si
         * ricade sul nome generico di sempre. "NONE" se nessun
         * errore dall'avvio.
         */
        snprintf(s_rsp_buf, sizeof(s_rsp_buf), "OK ALARM:%s\r\n",
                 FSM_GetLastErrorName());
        send_response(s_rsp_buf);

    } else if (strcmp(tok[1], "SAB") == 0) {
        /*
         * Diagnostica RAW del modulo SAB (dal 2026-07-15), aggiunta per
         * investigare bug segnalato: ERR_BIT_SAB_INTLCK_A/B abilitati in
         * error_mask ma nessuna transizione a SYS_ERROR quando i pin
         * nSAB_INTLCK_A/B_STATUS vanno HIGH (aperto) da SYS_ENABLED.
         *
         * A differenza di "GET ALARM" (ultima causa STORICA di SYS_ERROR,
         * aggiornata solo al momento della transizione), questo comando
         * espone lo stato LIVE, in tempo reale, di ogni anello della catena
         * SAB -> FSM, permettendo di isolare a banco dove si interrompe:
         *   STATE      - stato FSM di sistema corrente (per verificare che
         *                sia davvero SYS_ENABLED e non es. ancora SYS_ON se
         *                l'armo SAB non e' mai arrivato ad ARMED)
         *   SABSTATE   - stato interno del driver SAB (DISABLED/GUARD/
         *                ENABLING/ARMED/FAULT). Se resta ARMED con INTLCK
         *                aperto, il problema e' nella callback/mascheramento
         *                sotto, non nel driver. Se e' gia' FAULT, il driver
         *                ha rilevato l'apertura ma l'evento verso la FSM
         *                potrebbe essere stato scartato dalla maschera o
         *                dalla coda.
         *   INTLCK_A/B - stato RAW (non mascherato) dei due canali interlock
         *                (OPEN/CLOSED), da SAB_GetInterlockStatus().
         *   TEST_A/B   - stato RAW dei due canali diagnostica (OK/FAULT).
         *   MASK       - bit ERR_BIT_SAB_INTLCK_A/B e ERR_BIT_SAB_TEST_A/B
         *                correnti in g_config.error_mask (1=abilitato).
         */
        static const char * const s_sab_state_names[] = {
            "DISABLED", "GUARD", "ENABLING", "ARMED", "FAULT"
        };
        SAB_State_t sab_st = SAB_GetState();
        const char *sab_st_name = ((unsigned)sab_st < 5U) ? s_sab_state_names[sab_st] : "?";

        bool intlck_a_open, intlck_b_open, test_a_fault, test_b_fault;
        SAB_GetInterlockStatus(&intlck_a_open, &intlck_b_open);
        SAB_GetTestStatus(&test_a_fault, &test_b_fault);

        snprintf(s_rsp_buf, sizeof(s_rsp_buf),
                 "OK SAB STATE:%s SABSTATE:%s INTLCK_A:%s INTLCK_B:%s "
                 "TEST_A:%s TEST_B:%s MASK_INTLCK_A:%u MASK_INTLCK_B:%u "
                 "MASK_TEST_A:%u MASK_TEST_B:%u\r\n",
                 Rs485Cmd_StateStr(FSM_GetState()),
                 sab_st_name,
                 intlck_a_open ? "OPEN" : "CLOSED",
                 intlck_b_open ? "OPEN" : "CLOSED",
                 test_a_fault  ? "FAULT" : "OK",
                 test_b_fault  ? "FAULT" : "OK",
                 (unsigned)((g_config.error_mask & ERR_BIT_SAB_INTLCK_A) != 0U),
                 (unsigned)((g_config.error_mask & ERR_BIT_SAB_INTLCK_B) != 0U),
                 (unsigned)((g_config.error_mask & ERR_BIT_SAB_TEST_A)   != 0U),
                 (unsigned)((g_config.error_mask & ERR_BIT_SAB_TEST_B)   != 0U));
        send_response(s_rsp_buf);

    } else if (strcmp(tok[1], "FAULTS") == 0) {
        /*
         * Bitmask dei fault NON RECUPERABILI ATTUALMENTE attivi
         * (FAULT_BIT_LASEQ/AMC/COM/FLOOD/PSU, vedi task_monitor.h — FLOOD e
         * PSU migrati da error_mask il 2026-07-14, redesign per severità) —
         * a differenza di "GET FAULT" (singolare, ultima causa nota anche
         * se rientrata), questo riflette lo stato corrente in tempo reale.
         */
        char vs[36];
        format_bitmask(TaskMonitor_GetFaults(), s_fault_fmt, vs, sizeof(vs));
        snprintf(s_rsp_buf, sizeof(s_rsp_buf), "OK FAULTS:%s\r\n", vs);
        send_response(s_rsp_buf);

    } else if (strcmp(tok[1], "LQERR") == 0) {
        /*
         * Ultimo error_code grezzo (8 bit) ricevuto da LaseQ (MSG_STATUS
         * byte 1, FSM_ErrorCode_t in fsm.h), decodificato in nomi leggibili
         * — vedi FSM_FormatErrorCode()/task_comms.c. Diverso da "GET ERR"/
         * "GET FAULTS": quelli riflettono le maschere MMC (error_mask/
         * fault_mask), questo espone direttamente la diagnostica interna
         * riportata da LaseQ, indipendentemente da come le due maschere
         * sono configurate.
         */
        uint8_t lq_ec = TaskMonitor_GetLaseQErrorCode();
        char decoded[96];
        FSM_FormatErrorCode(lq_ec, decoded, sizeof(decoded));

        /*
         * Se è attivo il bit generico FAULT_HW, aggiunge anche il dettaglio
         * hw_fault_source (FSM_HwFaultSource_t) per distinguere PWR_OK da
         * interlock-non-confermato-a-enable da interlock-aperto-a-runtime
         * (vedi fsm.h, banner "DETTAGLIO DI FSM_FAULT_HW").
         */
        if ((lq_ec & FSM_FAULT_HW) != 0U) {
            LaseQ_status_vars_t st_hw = GetLaseQStatus();
            char hwdecoded[64];
            FSM_FormatHwFaultSource(st_hw.hw_fault_source, hwdecoded, sizeof(hwdecoded));
            snprintf(s_rsp_buf, sizeof(s_rsp_buf), "OK LQERR:0x%02X %s HWFLT:0x%02X %s\r\n",
                     (unsigned)lq_ec, decoded, (unsigned)st_hw.hw_fault_source, hwdecoded);
        } else {
            snprintf(s_rsp_buf, sizeof(s_rsp_buf), "OK LQERR:0x%02X %s\r\n",
                     (unsigned)lq_ec, decoded);
        }
        send_response(s_rsp_buf);

    } else if (strcmp(tok[1], "LQ") == 0) {
        /*
         * Parametri operativi dettagliati di LaseQ — protetto da login:
         * dati di servizio/diagnostica, non necessari per l'operatività
         * normale (a differenza di GET LQERR/GET TEMP LQ, pubblici).
         *
         * Combina:
         *   - GetLaseQStatus()  (autoritativo: ultimo STATUS ricevuto dallo
         *     slave — enable/gate/interlock EFFETTIVI, correnti, Vanode,
         *     setpoint ECHEGGIATO indietro, error_code, fsm_state LaseQ).
         *   - GetLaseQControl() (quello che MMC sta COMANDANDO in questo
         *     momento — serve soprattutto per "quali canali sono attivi",
         *     ch_enable: campo presente solo in MSG_CONTROL, non echeggiato
         *     indietro in MSG_STATUS, vedi laseq_protocol.h).
         *
         * SP_CMD = setpoint comandato da MMC ora; SP_ECHO = setpoint che
         * LaseQ conferma di aver applicato nell'ultimo STATUS (possono
         * differire per un ciclo RS485, ~20ms, durante una rampa).
         * Correnti/setpoint in mA, Vanode in mV (stesse unità delle API
         * LaseQSetCurrent()/LaseQ_status_vars_t, non convertite altrove).
         */
        REQUIRE_AUTH();

        LaseQ_status_vars_t  st = GetLaseQStatus();
        LaseQ_control_vars_t ct = GetLaseQControl();

        char errname[96];
        FSM_FormatErrorCode(st.error_code, errname, sizeof(errname));

        /*
         * hw_fault_source: dettaglio del bit generico ERR bit7 (FSM_FAULT_HW)
         * — distingue PWR_OK da interlock-non-confermato-a-enable da
         * interlock-aperto-a-runtime (fsm.h, banner "DETTAGLIO DI
         * FSM_FAULT_HW"; protocollo v0x0003, 2026-07-16). Sempre mostrato
         * (non solo se FAULT_HW attivo) per coerenza col resto del comando,
         * che espone sempre tutti i campi indipendentemente dallo stato.
         */
        char hwname[64];
        FSM_FormatHwFaultSource(st.hw_fault_source, hwname, sizeof(hwname));

        /* Un campo per riga dal 2026-07-15 (era una riga sola) — vedi send_line(). */
        send_line("OK LQ:");
        send_line("EN:%u GATE:%u CH:0x%02X MODE:%s",
                  (unsigned)st.enable, (unsigned)st.gate_status, (unsigned)ct.ch_enable,
                  ct.analog_mode ? "ANALOG" : "SW");
        send_line("ILK:%u OPM:%u OPD:%u",
                  (unsigned)st.interlock_status, (unsigned)st.OPM, (unsigned)st.OPD);
        send_line("SP_CMD:%lumA SP_ECHO:%lumA VANODE:%umV",
                  (unsigned long)ct.sw_current_setpoint,
                  (unsigned long)st.sw_current_setpoint,
                  (unsigned)st.v_anode);
        send_line("I0:%lu I1:%lu I2:%lu I3:%lumA",
                  (unsigned long)st.current_output[0], (unsigned long)st.current_output[1],
                  (unsigned long)st.current_output[2], (unsigned long)st.current_output[3]);
        send_line("ERR:0x%02X %s FSM:%u",
                  (unsigned)st.error_code, errname, (unsigned)st.fsm_state);
        send_line("HWFLT:0x%02X %s",
                  (unsigned)st.hw_fault_source, hwname);

    } else if (strcmp(tok[1], "MODULES") == 0) {
        /*
         * Stato di alimentazione dei 4 moduli interni sorvegliati da eFuse
         * (protetto da login: dati di servizio/diagnostica hardware).
         * EN riflette g_config.efuse_enabled_mask applicata al boot da
         * EFuse_Init() (vedi SET EFUSE MASK) — se EN:0, GOOD/FLT/I restano
         * comunque letti (il pin fisico non mente: un modulo mascherato ha
         * GOOD:0, I:0 per costruzione, dato che non è alimentato).
         * I in mA (vedi eFuse.h per il modello di conversione TPS16630).
         */
        REQUIRE_AUTH();

        static const char * const s_efuse_names[EFUSE_COUNT] = {
            "MAIN", "SAB", "COM", "LASEQ"
        };
        send_line("OK MODULES:");
        for (uint8_t id = 0U; id < EFUSE_COUNT; id++) {
            EFuse_status_t est = EFuse_GetStatus((EFuse_id_t)id);
            send_line("%s EN:%u GOOD:%u FLT:%u I:%umA",
                      s_efuse_names[id],
                      (unsigned)est.enabled, (unsigned)est.power_good,
                      (unsigned)est.fault, (unsigned)est.current_ma);
        }

    } else if (strcmp(tok[1], "DEW") == 0) {
        /*
         * Punto di condensa (Td) e margine (T_water_in - Td) dell'ultimo
         * calcolo valido, MMC e LaseQ separati (due arie/due bit distinti,
         * vedi WARN_BIT_DEW_MMC/LASEQ e ERR_BIT_DEW_MMC/LASEQ). STATE riflette
         * lo stesso stato usato per warning_mask/error_mask (OK/WARN/ERR);
         * N-A se il valore non è ancora significativo (cooling fermo o NTC
         * water_in non valido — vedi TaskMonitor_GetDewMMC()/GetDewLaseQ()).
         */
        float td, margin;
        bool  valid;
        uint32_t warns = TaskMonitor_GetWarnings();
        uint32_t errs  = TaskMonitor_GetErrors();

        /*
         * 48 byte: ampio margine per il contenuto reale (temperature
         * plausibili, poche decine di gradi). Il warning -Wformat-truncation
         * persiste comunque perché GCC valuta staticamente il caso peggiore
         * teorico di "%d" (int a 11 cifre con segno, INT_MIN) anche se
         * td10/mg10 qui sono in pratica sempre a 3-4 cifre — silenziato
         * sotto con un pragma mirato invece di sovradimensionare il buffer
         * per un caso che non si verifica mai.
         */
        char s_mmc[48], s_lq[48];

        valid = TaskMonitor_GetDewMMC(&td, &margin);
        if (!valid) {
            snprintf(s_mmc, sizeof(s_mmc), "MMC TD:N-A MARGIN:N-A STATE:N-A");
        } else {
            int td10 = (int)(td * 10.0f), mg10 = (int)(margin * 10.0f);
            const char *st = (errs & ERR_BIT_DEW_MMC) ? "ERR"
                            : (warns & WARN_BIT_DEW_MMC) ? "WARN" : "OK";
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wformat-truncation"
            snprintf(s_mmc, sizeof(s_mmc), "MMC TD:%d.%01d MARGIN:%d.%01d STATE:%s",
                     td10 / 10, (td10 < 0 ? -td10 : td10) % 10,
                     mg10 / 10, (mg10 < 0 ? -mg10 : mg10) % 10, st);
#pragma GCC diagnostic pop
        }

        valid = TaskMonitor_GetDewLaseQ(&td, &margin);
        if (!valid) {
            snprintf(s_lq, sizeof(s_lq), "LQ TD:N-A MARGIN:N-A STATE:N-A");
        } else {
            int td10 = (int)(td * 10.0f), mg10 = (int)(margin * 10.0f);
            const char *st = (errs & ERR_BIT_DEW_LASEQ) ? "ERR"
                            : (warns & WARN_BIT_DEW_LASEQ) ? "WARN" : "OK";
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wformat-truncation"
            snprintf(s_lq, sizeof(s_lq), "LQ TD:%d.%01d MARGIN:%d.%01d STATE:%s",
                     td10 / 10, (td10 < 0 ? -td10 : td10) % 10,
                     mg10 / 10, (mg10 < 0 ? -mg10 : mg10) % 10, st);
#pragma GCC diagnostic pop
        }

        /* Una riga per MMC, una per LaseQ (dal 2026-07-15, prima entrambe
         * sulla stessa riga) — vedi send_line(). */
        send_line("OK DEW:");
        send_line("%s", s_mmc);
        send_line("%s", s_lq);

    } else if (strcmp(tok[1], "TEMP") == 0) {
        /*
         * GET TEMP            → SHT35 (ambiente + umidità) + tutti gli NTC
         *                       attivi + temperature/umidità LaseQ (tutte
         *                       le temperature di sistema in un comando solo)
         * GET TEMP NTC        → solo NTC
         * GET TEMP SHT        → solo SHT35 MMC
         * GET TEMP LQ         → solo temperature/umidità LaseQ
         * GET TEMP MASK       → maschera device-enable dei sensori NTC
         *                       "funzionali" (g_config.temp_sensor_enabled_mask,
         *                       vedi SET TEMP MASK) — NON è la stessa cosa di
         *                       GET NTC MASK (quella è il mapping canale
         *                       AD7490 attivo/non attivo, questa è quale
         *                       sensore MAPPATO viene anche effettivamente
         *                       controllato da check_ntc_sensor() per
         *                       warning/fault). Aggiunto dopo che un sensore
         *                       mappato in GET NTC MAP (es. WATER_OUT) è
         *                       risultato non monitorato perché il suo bit
         *                       qui era rimasto a 0 dal default di fabbrica.
         */
        if (n >= 3 && strcmp(tok[2], "MASK") == 0) {
            snprintf(s_rsp_buf, sizeof(s_rsp_buf),
                     "OK TEMP MASK:0x%03X (bit0=WATER_IN bit1=WATER_OUT bit2=DRIVER "
                     "bit3=SPLICE bit4=DIODE1 bit5=AMBIENT bit6=DIODE2 bit7=PSU_TEMP bit8=PWR_EL_TEMP)\r\n",
                     (unsigned)g_config.temp_sensor_enabled_mask);
            send_response(s_rsp_buf);
            return;
        }

        bool show_sht = true;
        bool show_ntc = true;
        bool show_lq  = true;
        if (n >= 3) {
            if (strcmp(tok[2], "NTC") == 0) { show_sht = false; show_lq = false; }
            if (strcmp(tok[2], "SHT") == 0) { show_ntc = false; show_lq = false; }
            if (strcmp(tok[2], "LQ")  == 0) { show_sht = false; show_ntc = false; }
        }

        /* Una riga per elemento dal 2026-07-15 (era una riga sola) — vedi send_line(). */
        send_line("OK TEMP:");

        if (show_sht) {
            SHT35_Data_t d = SHT35_GetData();
            int t_cdeg = (int)d.temperature_cdeg;
            int t_i = t_cdeg / 100, t_f = (t_cdeg < 0 ? -t_cdeg : t_cdeg) % 100;
            /* MB_TEMP (ex "TAMB", rinominato 2026-07-22 — vedi s_temp_sensors[]
             * sopra: non è "ambient", è la temperatura dell'elettronica MMC
             * misurata dall'SHT35 locale, soglie proprie in temp_mb_*). */
            send_line("MB_TEMP:%d.%02d HUM:%d%%", t_i, t_f, (int)SHT35_GetHumidityPct());
        }

        if (show_ntc) {
            for (uint8_t s = 0U; s < NTC_NUM_SENSORS; s++) {
                int16_t t_c10 = 0;
                if (!TaskMonitor_GetNTCTempC10((NTC_SensorId_t)s, &t_c10)) continue;
                int ti = t_c10 / 10, tf = (t_c10 < 0 ? -t_c10 : t_c10) % 10;
                send_line("%s:%d.%01d", s_ntc_sensor_names[s], ti, (unsigned)tf);
            }
        }

        if (show_lq) {
            /*
             * Temperature/umidità LaseQ (MSG_STATUS, cache in task_monitor.c
             * aggiornata da task_comms.c ogni ciclo RS485 riuscito). Come per
             * gli NTC sopra, valori "ultimi noti": se LaseQ è disconnesso
             * restano quelli dell'ultima risposta valida — vedi GET FAULTS
             * (FAULT_BIT_LASEQ) per lo stato del link.
             * LQ_DRV0..3: i 4 elementi di potenza (vedi WARN_BIT_TEMP_DRIVER).
             * LQ_AMB/LQ_HUM: SHT35 di LaseQ.
             */
            uint8_t lq_drv[4];
            int8_t  lq_amb;
            uint8_t lq_hum;
            TaskMonitor_GetLaseQTemps(lq_drv, &lq_amb, &lq_hum);

            for (uint8_t i = 0U; i < 4U; i++) {
                send_line("LQ_DRV%u:%d", (unsigned)i, (int)lq_drv[i]);
            }
            send_line("LQ_AMB:%d LQ_HUM:%u%%", (int)lq_amb, (unsigned)lq_hum);
        }

    } else if (strcmp(tok[1], "FLOW") == 0) {
        float f1 = FlowMeter_GetFlowRate(FLOW_METER_1);
        float f2 = FlowMeter_GetFlowRate(FLOW_METER_2);
        int f1i = (int)(f1 * 10.0f), f2i = (int)(f2 * 10.0f);
        snprintf(s_rsp_buf, sizeof(s_rsp_buf),
                 "OK F1:%d.%01u F2:%d.%01u L/min\r\n",
                 f1i / 10, (unsigned)(f1i < 0 ? -f1i : f1i) % 10U,
                 f2i / 10, (unsigned)(f2i < 0 ? -f2i : f2i) % 10U);
        send_response(s_rsp_buf);

    } else if (strcmp(tok[1], "CONFIG") == 0) {
        /* Un campo per riga dal 2026-07-15 (era una riga sola) — vedi send_line(). */
        const char *fmt_str[] = {"HEX", "DEC", "BIN"};
        char emask[36], wmask[36];
        format_bitmask(g_config.error_mask,   NUM_FMT_HEX, emask, sizeof(emask));
        format_bitmask(g_config.warning_mask, NUM_FMT_HEX, wmask, sizeof(wmask));

        send_line("OK ERR_FMT:%s WARN_FMT:%s", fmt_str[s_err_fmt], fmt_str[s_warn_fmt]);
        send_line("ERR_MASK:%s", emask);
        send_line("WARN_MASK:%s", wmask);
        send_line("DEW_WARN:%d", (int)g_config.dew_warn_delta_c);
        send_line("DEW_ERR:%d", (int)g_config.dew_err_delta_c);
        send_line("HUM_MMC:%u", (unsigned)g_config.humidity_max_warn_pct);
        send_line("HUM_LQ:%u", (unsigned)g_config.laseq_humidity_max_warn_pct);
        send_line("DLY_PSU:%u", (unsigned)g_config.psu_dc_ok_delay_ms);
        send_line("DLY_SAB:%u", (unsigned)g_config.sab_interlock_timeout_ms);
        send_line("DLY_CONT:%u", (unsigned)g_config.contactor_psu_delay_ms);
        send_line("VCOMP_EN:%u DELAY:%ums RAMP:%ums RAMPUP:%ums (NUOVO 2026-07-22, vedi SET VCOMP)",
                   (unsigned)g_config.voltage_comp_enabled,
                   (unsigned)g_config.comp_stabilize_delay_ms,
                   (unsigned)g_config.comp_ramp_duration_ms,
                   (unsigned)g_config.comp_ramp_up_duration_ms);
        send_line("AUTH:%s", s_authenticated ? "YES" : "NO");

    } else if (strcmp(tok[1], "THRESHOLDS") == 0) {
        /*
         * Tutte le soglie configurabili in un comando solo (dal 2026-07-15
         * — prima serviva interrogare ogni sensore/parametro separatamente,
         * nessun comando le riassumeva tutte). Sola lettura, nessun login
         * richiesto (stesso livello di GET TEMP/GET FLOW/GET DEW, che già
         * espongono valori equivalenti singolarmente). Una riga per
         * sensore/parametro — vedi send_line().
         *
         * Per modificare: SET TEMP/SET FLOW/SET DEW/SET HUM (vedi HELP).
         */
        send_line("OK THRESHOLDS:");
        send_line("TEMP WATER_IN  MIN_ERR:%d MIN_WARN:%d MAX_WARN:%d MAX_ERR:%d",
                   (int)g_config.temp_water_in_min_err,  (int)g_config.temp_water_in_min_warn,
                   (int)g_config.temp_water_in_max_warn, (int)g_config.temp_water_in_max_err);
        send_line("TEMP WATER_OUT MIN_ERR:%d MIN_WARN:%d MAX_WARN:%d MAX_ERR:%d",
                   (int)g_config.temp_water_out_min_err,  (int)g_config.temp_water_out_min_warn,
                   (int)g_config.temp_water_out_max_warn, (int)g_config.temp_water_out_max_err);
        send_line("TEMP DRIVER    MIN_ERR:%d MIN_WARN:%d MAX_WARN:%d MAX_ERR:%d",
                   (int)g_config.temp_driver_min_err,  (int)g_config.temp_driver_min_warn,
                   (int)g_config.temp_driver_max_warn, (int)g_config.temp_driver_max_err);
        send_line("TEMP SPLICE    MIN_ERR:%d MIN_WARN:%d MAX_WARN:%d MAX_ERR:%d",
                   (int)g_config.temp_splice_min_err,  (int)g_config.temp_splice_min_warn,
                   (int)g_config.temp_splice_max_warn, (int)g_config.temp_splice_max_err);
        send_line("TEMP DIODE1    MIN_ERR:%d MIN_WARN:%d MAX_WARN:%d MAX_ERR:%d",
                   (int)g_config.temp_diode_min_err,  (int)g_config.temp_diode_min_warn,
                   (int)g_config.temp_diode_max_warn, (int)g_config.temp_diode_max_err);
        send_line("TEMP DIODE2    MIN_ERR:%d MIN_WARN:%d MAX_WARN:%d MAX_ERR:%d",
                   (int)g_config.temp_diode2_min_err,  (int)g_config.temp_diode2_min_warn,
                   (int)g_config.temp_diode2_max_warn, (int)g_config.temp_diode2_max_err);
        send_line("TEMP AMBIENT   MIN_ERR:%d MIN_WARN:%d MAX_WARN:%d MAX_ERR:%d",
                   (int)g_config.temp_ambient_min_err,  (int)g_config.temp_ambient_min_warn,
                   (int)g_config.temp_ambient_max_warn, (int)g_config.temp_ambient_max_err);
        send_line("TEMP LQ_AMBIENT MIN_ERR:%d MIN_WARN:%d MAX_WARN:%d MAX_ERR:%d",
                   (int)g_config.temp_lq_ambient_min_err,  (int)g_config.temp_lq_ambient_min_warn,
                   (int)g_config.temp_lq_ambient_max_warn, (int)g_config.temp_lq_ambient_max_err);
        /* PSU_TEMP/PWR_EL_TEMP/MB_TEMP: NUOVO 2026-07-22, vedi s_temp_sensors[] */
        send_line("TEMP PSU_TEMP  MIN_ERR:%d MIN_WARN:%d MAX_WARN:%d MAX_ERR:%d",
                   (int)g_config.temp_psu_min_err,  (int)g_config.temp_psu_min_warn,
                   (int)g_config.temp_psu_max_warn, (int)g_config.temp_psu_max_err);
        send_line("TEMP PWR_EL_TEMP MIN_ERR:%d MIN_WARN:%d MAX_WARN:%d MAX_ERR:%d",
                   (int)g_config.temp_pwr_el_min_err,  (int)g_config.temp_pwr_el_min_warn,
                   (int)g_config.temp_pwr_el_max_warn, (int)g_config.temp_pwr_el_max_err);
        send_line("TEMP MB_TEMP   MIN_ERR:%d MIN_WARN:%d MAX_WARN:%d MAX_ERR:%d",
                   (int)g_config.temp_mb_min_err,  (int)g_config.temp_mb_min_warn,
                   (int)g_config.temp_mb_max_warn, (int)g_config.temp_mb_max_err);
        send_line("FLOW           MIN_ERR:%u.%u MIN_WARN:%u.%u MAX_WARN:%u.%u MAX_ERR:%u.%u Lmin",
                   (unsigned)(g_config.flow_min_err_lpm_x10 / 10U),  (unsigned)(g_config.flow_min_err_lpm_x10 % 10U),
                   (unsigned)(g_config.flow_min_warn_lpm_x10 / 10U), (unsigned)(g_config.flow_min_warn_lpm_x10 % 10U),
                   (unsigned)(g_config.flow_max_warn_lpm_x10 / 10U), (unsigned)(g_config.flow_max_warn_lpm_x10 % 10U),
                   (unsigned)(g_config.flow_max_err_lpm_x10 / 10U),  (unsigned)(g_config.flow_max_err_lpm_x10 % 10U));
        send_line("HUM  MMC:%u%% LASEQ:%u%% (soglia warning unica, nessun MIN/MAX/ERR separato)",
                   (unsigned)g_config.humidity_max_warn_pct,
                   (unsigned)g_config.laseq_humidity_max_warn_pct);
        send_line("DEW  WARN_DELTA:%d ERR_DELTA:%d degC (applicato indipendentemente a MMC e LaseQ)",
                   (int)g_config.dew_warn_delta_c, (int)g_config.dew_err_delta_c);

    } else if (strcmp(tok[1], "NTC") == 0) {
        /*
         * GET NTC MAP   → mostra mappatura serigrafia (1-16) → nome sensore
         *                 (dal 2026-07-15, vedi SET NTC MAP sopra)
         * GET NTC MASK  → maschera hex 16-bit canali attivi
         * GET NTC BETA  → coefficiente Beta per serigrafia (NUOVO 2026-07-22,
         *                 vedi SET NTC BETA sopra)
         */
        if (n < 3) { err("USAGE: GET NTC MAP|MASK|BETA"); return; }

        if (strcmp(tok[2], "MAP") == 0) {
            /*
             * Formato: OK NTC01:WATER_IN NTC02:- NTC03:DIODE1 ...
             * Elencato per NUMERO DI SERIGRAFIA (1-16, leggibile sulla
             * scheda) dal 2026-07-15, non più per canale AD7490 grezzo —
             * vedi s_ntc_serigrafia_to_ch sopra. Il canale fisico resta
             * comunque consultabile tramite quella tabella se serve.
             */
            send_line("OK NTC MAP:");
            for (uint8_t serigrafia = 1U; serigrafia <= AD7490_NUM_CHANNELS; serigrafia++) {
                uint8_t ch  = s_ntc_serigrafia_to_ch[serigrafia - 1U];
                uint8_t sid = g_config.ntc_ch_map[ch];
                const char *name = (sid < NTC_NUM_SENSORS)
                                   ? s_ntc_sensor_names[sid] : "-";
                send_line("NTC%02u:%s", (unsigned)serigrafia, name);
            }

        } else if (strcmp(tok[2], "MASK") == 0) {
            snprintf(s_rsp_buf, sizeof(s_rsp_buf),
                     "OK NTC_MASK:0x%04X\r\n",
                     (unsigned)TaskMonitor_GetNTCActiveMask());
            send_response(s_rsp_buf);

        } else if (strcmp(tok[2], "BETA") == 0) {
            /* Formato: OK NTC BETA: / NTC01:3950 NTC02:3950 ... — indicizzato
             * per serigrafia (1-16), stesso schema di GET NTC MAP. */
            send_line("OK NTC BETA:");
            for (uint8_t serigrafia = 1U; serigrafia <= AD7490_NUM_CHANNELS; serigrafia++) {
                uint8_t ch = s_ntc_serigrafia_to_ch[serigrafia - 1U];
                send_line("NTC%02u:%u", (unsigned)serigrafia, (unsigned)g_config.ntc_beta[ch]);
            }

        } else {
            err("GET NTC: MAP|MASK|BETA");
        }

    } else if (strcmp(tok[1], "LUT") == 0) {
        /*
         * GET LUT VCOMP           → dump LUT compensazione tensione PSU (mA->mV)
         * GET LUT GAIN            → soglie finestre guadagno (SW/HW)
         * GET LUT VALID pd SW|HW  → LUT validazione PD (4 entry)
         * GET LUT POWER pd win    → LUT conversione ADC->Watt (5 entry)
         * GET LUT SETPOINT        → LUT power%->mA (g_config.setpoint_lut_ma[],
         *                           NON fa parte di LUT_Store_t/lut_manager.c:
         *                           vive in config.c, persistita subito da
         *                           Config_Save() — utile come "controllo"
         *                           per isolare un eventuale problema di
         *                           persistenza specifico del modulo LUT
         *                           (lut_manager.c) da uno più generale del
         *                           settore flash condiviso: se questa
         *                           sopravvive a un riavvio/reflash e le
         *                           altre no, il problema è nel modulo LUT,
         *                           non nella flash/nel settore condiviso.
         *
         * Aggiunti il 2026-07-27 (prima solo VCOMP era leggibile) per poter
         * confrontare tutte le sotto-tabelle di LUT_Store_t fra loro — se
         * dopo un riavvio TUTTE tornano ai valori di default precompilati
         * (facilmente riconoscibili, vedi banner "VALORI DI DEFAULT" in
         * lut_manager.c — dal 2026-08-01 anche VCOMP ha un default non
         * vuoto, curva reale, non più 0 entry), vuol dire che
         * flash_read()/LUT_Init() non sta rileggendo affatto la LUT dalla
         * flash (fallback a load_defaults() per l'intera struttura, che è
         * scritta/letta come un unico blocco con un solo CRC): non
         * sarebbe quindi un problema isolato a una sola sotto-tabella.
         */
        if (n < 3) {
            err("USAGE: GET LUT VCOMP|GAIN|VALID|POWER|SETPOINT|HWPWR"); return;
        }

        if (strcmp(tok[2], "VCOMP") == 0) {
            const LUT_Store_t *lut = LUT_Get();
            send_line("OK LUT VCOMP: (%u/%u entry)",
                      (unsigned)lut->voltage_comp_size, (unsigned)LUT_VCOMP_SIZE);
            for (uint8_t e = 0U; e < lut->voltage_comp_size; e++) {
                send_line("[%u] %umA->%umV", (unsigned)e,
                           (unsigned)lut->voltage_comp[e].current_ma,
                           (unsigned)lut->voltage_comp[e].voltage_mv);
            }

        } else if (strcmp(tok[2], "GAIN") == 0) {
            /* Default di fabbrica (load_defaults(), lut_manager.c): SW =
             * 2625/5250/7875/65535, HW = 1024/2048/3072/65535 — se dopo un
             * riavvio/reflash vedi ESATTAMENTE questi valori pur avendo
             * salvato soglie diverse con SET LUT GAIN + SAVE LUT, la LUT non
             * viene ricaricata dalla flash (stesso sintomo di VCOMP, non
             * isolato). */
            const LUT_Store_t *lut = LUT_Get();
            send_line("OK LUT GAIN:");
            send_line("SW:%u %u %u %u",
                       (unsigned)lut->gain_threshold_sw[0], (unsigned)lut->gain_threshold_sw[1],
                       (unsigned)lut->gain_threshold_sw[2], (unsigned)lut->gain_threshold_sw[3]);
            send_line("HW:%u %u %u %u",
                       (unsigned)lut->gain_threshold_hw[0], (unsigned)lut->gain_threshold_hw[1],
                       (unsigned)lut->gain_threshold_hw[2], (unsigned)lut->gain_threshold_hw[3]);

        } else if (strcmp(tok[2], "VALID") == 0) {
            if (n < 5) { err("USAGE: GET LUT VALID pd SW|HW"); return; }
            int pd = atoi(tok[3]);
            if (pd < 0 || pd > 3) { err("pd: 0-3"); return; }
            uint8_t mode;
            if      (strcmp(tok[4], "SW") == 0) mode = 0U;
            else if (strcmp(tok[4], "HW") == 0) mode = 1U;
            else { err("VALID mode: SW|HW"); return; }
            const LUT_Store_t *lut = LUT_Get();
            send_line("OK LUT VALID PD%d %s: (%u/%u entry)",
                       pd, tok[4], (unsigned)lut->pd_valid_size, (unsigned)LUT_PD_VALID_SIZE);
            for (uint8_t e = 0U; e < lut->pd_valid_size; e++) {
                const LUT_PDValidEntry_t *ent = &lut->pd_valid[mode][pd][e];
                send_line("[%u] sp=%u min=%u max=%u", (unsigned)e,
                           (unsigned)ent->setpoint, (unsigned)ent->pd_min, (unsigned)ent->pd_max);
            }

        } else if (strcmp(tok[2], "POWER") == 0) {
            if (n < 5) { err("USAGE: GET LUT POWER pd win"); return; }
            int pd = atoi(tok[3]), win = atoi(tok[4]);
            if (pd < 0 || pd > 3)   { err("pd: 0-3");  return; }
            if (win < 0 || win > 3) { err("win: 0-3"); return; }
            const LUT_Store_t *lut = LUT_Get();
            send_line("OK LUT POWER PD%d WIN%d: (%u/%u entry)",
                       pd, win, (unsigned)lut->pd_power_size, (unsigned)LUT_PD_POWER_SIZE);
            for (uint8_t e = 0U; e < lut->pd_power_size; e++) {
                const LUT_PDPowerEntry_t *ent = &lut->pd_power[pd][win][e];
                send_line("[%u] adc=%u watt=%u", (unsigned)e,
                           (unsigned)ent->adc, (unsigned)ent->power_w);
            }

        } else if (strcmp(tok[2], "SETPOINT") == 0) {
            /* g_config.setpoint_lut_ma[]: NON fa parte di LUT_Store_t, vive
             * in Config_t e persiste tramite lo stesso journal di config.c
             * già confermato sopravvivere al reflash — utile come "gruppo di
             * controllo". */
            send_line("OK LUT SETPOINT: (%u entry, 5%% x entry)", (unsigned)SETPOINT_LUT_SIZE);
            for (uint8_t i = 0U; i < SETPOINT_LUT_SIZE; i++) {
                send_line("[%u%%]=%umA", (unsigned)(i * 5U), (unsigned)g_config.setpoint_lut_ma[i]);
            }

        } else if (strcmp(tok[2], "HWPWR") == 0) {
            /* LUT setpoint HW (RAW ADC PC5 -> % potenza, NUOVO 2026-08-01) —
             * vedi lut_manager.h/LUT_HwPowerEntry_t. */
            const LUT_Store_t *lut = LUT_Get();
            send_line("OK LUT HWPWR: (%u/%u entry)",
                       (unsigned)lut->hw_power_size, (unsigned)LUT_HWPWR_SIZE);
            for (uint8_t e = 0U; e < lut->hw_power_size; e++) {
                send_line("[%u] adc=%u pct=%u%%", (unsigned)e,
                           (unsigned)lut->hw_power[e].adc_raw,
                           (unsigned)lut->hw_power[e].power_pct);
            }

        } else {
            err("LUT subcmd: VCOMP|GAIN|VALID|POWER|SETPOINT|HWPWR");
        }

    } else if (strcmp(tok[1], "TIME") == 0) {
        char ts[10], ds[12];
        rtc_get_strings(ts, sizeof(ts), ds, sizeof(ds));
        snprintf(s_rsp_buf, sizeof(s_rsp_buf), "OK TIME:%s\r\n", ts);
        send_response(s_rsp_buf);

    } else if (strcmp(tok[1], "DATE") == 0) {
        char ts[10], ds[12];
        rtc_get_strings(ts, sizeof(ts), ds, sizeof(ds));
        snprintf(s_rsp_buf, sizeof(s_rsp_buf), "OK DATE:%s\r\n", ds);
        send_response(s_rsp_buf);

    } else if (strcmp(tok[1], "TERM") == 0) {
        snprintf(s_rsp_buf, sizeof(s_rsp_buf), "OK TERM:%s\r\n",
                 g_config.termination ? "ON" : "OFF");
        send_response(s_rsp_buf);

    } else if (strcmp(tok[1], "SETPOINTCOMP") == 0) {
        snprintf(s_rsp_buf, sizeof(s_rsp_buf), "OK SETPOINTCOMP:%s\r\n",
                 g_config.hw_setpoint_sel ? "ON" : "OFF");
        send_response(s_rsp_buf);

    } else if (strcmp(tok[1], "SETPOINT") == 0) {
        if (n < 3 || strcmp(tok[2], "RANGE") != 0) {
            err("USAGE: GET SETPOINT RANGE"); return;
        }
        /*
         * Riporta lo stato REALE del pin nREDUCED_SETPOINT_RANGE
         * (EXT_GetReducedSetpointRange(), read-back GPIO), non la sola
         * configurazione salvata: normalmente i due sono allineati (SET
         * SETPOINT RANGE applica sempre entrambi insieme), ma il pin resta
         * la verità elettrica effettiva.
         */
        snprintf(s_rsp_buf, sizeof(s_rsp_buf), "OK SETPOINT RANGE:%s\r\n",
                 EXT_GetReducedSetpointRange() ? "6V" : "10V");
        send_response(s_rsp_buf);

    } else if (strcmp(tok[1], "FW") == 0) {
        /*
         * Versione + revisione + slot fisico del firmware applicativo
         * REALMENTE in esecuzione (vedi fw_version.h/fw_slot.h) -
         * puramente informativo, nessun REQUIRE_AUTH(), stesso schema del
         * comando "VER" del bootloader (MMC_Bootloader/App/OTA/ota_ascii.c).
         *
         * SLOT: rilevato a runtime dall'indirizzo del proprio vector table
         * (FwSlot_GetActiveChar(), vedi fw_slot.h) - NON dal journal
         * metadati del bootloader (che questo comando non consulta affatto:
         * e' proprio il punto, sapere cosa sta girando DAVVERO senza dover
         * entrare nel bootloader e chiedere STATUS, utile in particolare
         * dopo un flash diretto via CubeProgrammer che puo' aver lasciato
         * il journal disallineato).
         *
         * AMC/LASEQ (NUOVO 2026-09-04, AMC_protocol.h v0x0006 /
         * laseq_protocol.h v0x0004): versione (+ revisione di build per
         * LaseQ, che ha spazio nel payload per riportarla) letta
         * dall'ULTIMO CONFIG_ACK ricevuto da ciascuna scheda
         * (AMC_GetFwVersionStr(), GetLaseQStatus().fw_version/
         * fw_build_date) - non interrogata sul momento: il giro di
         * CONFIG_SET/CONFIG_ACK avviene solo all'avvio o su richiesta
         * esplicita di resend (vedi AMC.c/LaseQ.c), quindi il valore
         * riflette l'ultima config accettata O rifiutata (il campo e'
         * popolato da AMC/LaseQ su entrambi gli esiti). "UNKNOWN" se non
         * e' ancora arrivato nessun CONFIG_ACK in questa sessione, o se la
         * scheda sta girando un firmware piu' vecchio che non popola
         * ancora questo campo - non e' un errore, solo un dato non
         * disponibile al momento.
         */
        snprintf(s_rsp_buf, sizeof(s_rsp_buf), "OK FW:%s REV:%s SLOT:%c\r\n",
                 FW_VERSION_STR, FW_BUILD_DATE_STR, FwSlot_GetActiveChar());
        send_response(s_rsp_buf);

        {
            const char *amc_fw = AMC_GetFwVersionStr();
            snprintf(s_rsp_buf, sizeof(s_rsp_buf), "OK AMC:%s\r\n",
                     (amc_fw[0] != '\0') ? amc_fw : "UNKNOWN");
            send_response(s_rsp_buf);
        }

        {
            LaseQ_status_vars_t lq_status = GetLaseQStatus();
            snprintf(s_rsp_buf, sizeof(s_rsp_buf), "OK LASEQ:%s REV:%s\r\n",
                     (lq_status.fw_version[0] != '\0') ? lq_status.fw_version : "UNKNOWN",
                     (lq_status.fw_build_date[0] != '\0') ? lq_status.fw_build_date : "UNKNOWN");
            send_response(s_rsp_buf);
        }

    } else {
        err("GET subcmd: STATE|MODE|REGIME|STATUS|POWER|ERR|WARN|FAULT|ALARM|FAULTS|LQERR|LQ|MODULES|DEW|TEMP|NTC|FLOW|CONFIG|THRESHOLDS|LUT|TIME|DATE|TERM|SETPOINTCOMP|SETPOINT RANGE|FW");
    }
}

/* ========================================================================== */
/* GESTORI COMANDI — LOG                                                       */
/* ========================================================================== */

/**
 * LOG ON [interval_s]  — avvia stream periodico CSV (default 3s, vedi
 *                         send_log_header()/send_log_row()): invia subito
 *                         "OK", poi l'header CSV una volta sola, poi una
 *                         riga dati (prima immediata, poi ogni interval_s).
 * LOG OFF              — interrompe stream
 * LOG ERR ON|OFF       — notifica ALARM/FAULT non sollecitata (default OFF)
 */
static void cmd_log(char **tok, int n)
{
    if (n < 2) { err("USAGE: LOG <ON [interval_s]|OFF|ERR ON|OFF>"); return; }

    /*
     * LOG ERR ON|OFF: notifica non sollecitata ad ogni SYS_ERROR/SYS_FAULT
     * (vedi Rs485Cmd_NotifyFault()). Distinto dallo stream periodico
     * "LOG ON/OFF" sopra: flag di sessione, non persistito, sempre OFF
     * all'avvio — va riattivato esplicitamente ad ogni power-cycle.
     */
    if (strcmp(tok[1], "ERR") == 0) {
        if (n < 3) { err("USAGE: LOG ERR ON|OFF"); return; }

        if (strcmp(tok[2], "ON") == 0) {
            s_log_err_enabled = true;
            ok("LOG ERR ON");
        } else if (strcmp(tok[2], "OFF") == 0) {
            s_log_err_enabled = false;
            ok("LOG ERR OFF");
        } else {
            err("LOG ERR: ON|OFF");
        }
        return;
    }

    if (strcmp(tok[1], "ON") == 0) {
        uint32_t interval_s = 3U;
        if (n >= 3) {
            int v = atoi(tok[2]);
            if (v < 1 || v > 3600) { err("interval_s: 1-3600"); return; }
            interval_s = (uint32_t)v;
        }
        s_log_interval_ms = interval_s * 1000U;
        s_log_active      = true;
        snprintf(s_rsp_buf, sizeof(s_rsp_buf), "OK LOG ON interval:%lus\r\n",
                 (unsigned long)interval_s);
        send_response(s_rsp_buf);
        send_log_header();  /* intestazione CSV, una volta sola */
        send_log_row();     /* prima riga dati immediata */

    } else if (strcmp(tok[1], "OFF") == 0) {
        s_log_active = false;
        ok("LOG OFF");

    } else {
        err("LOG: ON [interval_s]|OFF|ERR ON|OFF");
    }
}

/* ========================================================================== */
/* GESTORI COMANDI — FLASH                                                     */
/* ========================================================================== */

static void cmd_save_lut(void)
{
    REQUIRE_AUTH();
    if (LUT_Save()) { ok("LUT saved to flash"); }
    else            { err("flash write failed"); }
}

static void cmd_reset_lut(void)
{
    REQUIRE_AUTH();
    LUT_ResetDefaults();
    ok("LUT reset to defaults");
}

static void cmd_save_config(void)
{
    REQUIRE_AUTH();
    Config_err_t e = Config_Save();
    if (e == CONFIG_OK) {
        ok("config saved to flash");
    } else {
        snprintf(s_rsp_buf, sizeof(s_rsp_buf),
                 "ERR config save failed (code %d)\r\n", (int)e);
        send_response(s_rsp_buf);
    }
}

/*
 * @brief  "FW UPDATE": innesca l'ingresso in modalita' update del
 *         bootloader MMC via RS485 al prossimo boot (vedi
 *         ota_update_trigger.h per il meccanismo completo - registro di
 *         backup RTC letto/consumato dal bootloader in ota_boot.c, che apre
 *         una finestra di ascolto RS485 molto piu' ampia della normale,
 *         pensata apposta per un reset innescato da remoto invece che da un
 *         power-on fisico con operatore gia' pronto al tool).
 *
 *         NON richiede LOGIN (aperto su richiesta esplicita, 2026-07-31 -
 *         prima richiedeva autenticazione come le altre operazioni
 *         sensibili di questo file). Richiede comunque stato SYS_IDLE:
 *         NVIC_SystemReset() interrompe immediatamente qualunque controllo
 *         software in corso (setpoint, interlock, regolazione PSU) -
 *         accettabile solo a banco/macchina fermo, mai durante
 *         un'emissione o una sequenza di accensione in corso (stesso
 *         principio delle altre operazioni "solo stato IDLE" in questo
 *         file, es. SET PSU/SET CONTACTOR MASK - quel controllo resta
 *         invariato, e' solo il LOGIN a essere stato rimosso). L'accesso
 *         al bootloader stesso resta comunque protetto dalla propria
 *         password OTA dedicata (LOGIN sul CLI ASCII del bootloader,
 *         vedi ota_ascii.c) - questo comando si limita ad aprire la
 *         finestra di ascolto, non bypassa l'autenticazione del
 *         trasferimento vero e proprio.
 *
 *         Dopo questo comando: il modulo si resetta, il bootloader entra in
 *         ascolto RS485 esteso (OTA_FORCED_UPDATE_WINDOW_MS, 30s) sullo
 *         stesso filo/baudrate - da quel momento va usato rs485_ota_update.py
 *         esattamente come per un aggiornamento innescato a power-on
 *         (trigger binario "OTAU" o CLI ASCII LOGIN/UPDATE, vedi ota_ascii.c/
 *         ota_rs485.c nel progetto bootloader). Se nessun aggiornamento
 *         arriva entro la finestra, il bootloader prosegue con il boot
 *         normale (nessun blocco indefinito).
 */
static void cmd_fw_update(void)
{
    if (FSM_GetState() != SYS_IDLE) { err_state("FW UPDATE", "IDLE"); return; }

    ok("entering bootloader update mode, resetting");

    /* Margine extra oltre al wait interno gia' fatto da send_response()
     * (osThreadFlagsWait su RS485_TX_FLAG, dentro ok() sopra): stessa
     * logica/durata usate dal bootloader stesso prima del proprio
     * NVIC_SystemReset() a fine transfer riuscito - vedi
     * Ota_Rs485_RunTransfer() in ota_rs485.c ("lascia il tempo alla UART
     * di svuotare il TX FIFO prima del reset"). */
    HAL_Delay(50U);

    HAL_RTCEx_BKUPWrite(&hrtc, OTA_TRIGGER_BKP_REGISTER, OTA_TRIGGER_BKP_MAGIC);
    NVIC_SystemReset(); /* non ritorna */
}

/* ========================================================================== */
/* HELP                                                                        */
/* ========================================================================== */

static void cmd_help(void)
{
    /* Inviamo in due blocchi per non eccedere s_rsp_buf */
    static const char help_a[] =
        "OK -- Sequenza FSM (nessun login) --\r\n"
        "  START  STOP  SON  SOFF  SEN  SDIS  PON  POFF  CERR\r\n"
        "-- Setpoint (nessun login) --\r\n"
        "  SET MODE SW|HW|ANALOG\r\n"
        "  SET SETPOINT <0-100>\r\n"
        "  SET SETPOINT RANGE 10V|6V   GET SETPOINT RANGE\r\n"
        "    (nREDUCED_SETPOINT_RANGE: applicata subito, persistita subito - come TERM)\r\n"
        "-- Monitor (nessun login) --\r\n"
        "  GET STATE|MODE|REGIME|STATUS|POWER|ERR|WARN|FAULT|ALARM|FAULTS|LQERR|DEW|TEMP[NTC|SHT|LQ|MASK]|NTC MAP|MASK|FLOW|CONFIG|THRESHOLDS|TIME|DATE|TERM|FW\r\n"
        "    (FW: versione + revisione/build-date + slot fisico A/B del firmware in esecuzione,\r\n"
        "     rilevato a runtime dal vector table - non richiede il bootloader)\r\n"
        "    (FAULT: ultimo evento causa di SYS_FAULT - NON recuperabile)\r\n"
        "    (ALARM: ultimo evento causa di SYS_ERROR - recuperabile via CERR)\r\n"
        "    (FAULT/ALARM: sorgente specifica quando distinguibile, es. KEY_A,\r\n"
        "     FLOOD2, PSU1, DIODE1|DIODE2 - non solo il nome generico)\r\n"
        "    (FAULTS: bitmask fault NON recuperabili ATTIVI ora - LASEQ/AMC/COM/FLOOD1/FLOOD2/PSU)\r\n"
        "    (GET FAULT LATCH: sottoinsieme di FAULTS latchato - persiste al power-cycle, vedi FRST)\r\n"
        "    (LQERR: error_code grezzo riportato da LaseQ, decodificato per nome bit)\r\n"
        "    (DEW: punto di condensa/margine MMC+LaseQ, N-A se non ancora significativo)\r\n"
        "    (TEMP senza argomenti: TUTTE le temperature di sistema - SHT35+NTC MMC e LaseQ)\r\n"
        "    (THRESHOLDS: TUTTE le soglie configurabili in un comando solo - TEMP\r\n"
        "     per sensore, FLOW, HUM, DEW; per modificarle vedi SET TEMP/FLOW/HUM/DEW sotto)\r\n"
        "    (GET NTC MAP/MODULES/TEMP/DEW/LQ/CONFIG/THRESHOLDS: dal 2026-07-15\r\n"
        "     restituiscono una riga per elemento, non piu' una riga sola)\r\n"
        "-- Formato bitmask (nessun login) --\r\n"
        "  SET ERR   HEX|DEC|BIN\r\n"
        "  SET WARN  HEX|DEC|BIN\r\n"
        "  SET FAULT HEX|DEC|BIN\r\n"
        "-- Log stream CSV (nessun login) --\r\n"
        "  LOG ON [interval_s]   LOG OFF   (default 3s, sep ';', header CSV + righe dati)\r\n"
        "  LOG ERR ON|OFF        (notifica ALARM/FAULT, default OFF a ogni avvio)\r\n"
        "-- RTC (nessun login) --\r\n"
        "  SET TIME HH:MM:SS\r\n"
        "  SET DATE DD/MM/YYYY\r\n"
        "-- Terminazione bus RS485 (nessun login) --\r\n"
        "  SET TERM ON|OFF   GET TERM\r\n"
        "-- Selezione sorgente setpoint HW verso LaseQ (nessun login) --\r\n"
        "  SET SETPOINTCOMP ON|OFF   GET SETPOINTCOMP\r\n"
        "    (PF14: ON=DAC AMC, OFF=EXT/utente - applicato/persistito subito, come TERM)\r\n";

    static const char help_b[] =
        "-- [LOGIN required] Autenticazione --\r\n"
        "  LOGIN <pw>   LOGOUT   SET PASSWORD <old> <new>\r\n"
        "-- [LOGIN required] Parametri dettagliati LaseQ --\r\n"
        "  GET LQ    (EN/GATE/CH/MODE/ILK/OPM/OPD/setpoint/Vanode/correnti/ERR/FSM)\r\n"
        "  GET SAB   (STATE FSM, SABSTATE driver, INTLCK_A/B, TEST_A/B, MASK_* - diagnostica RAW, dal 2026-07-15)\r\n"
        "-- [LOGIN required] Maschere errori/warning/fault --\r\n"
        "  SET ERR   MASK <num>    SET WARN MASK <num>    SET FAULT MASK <num>\r\n"
        "  GET ERR   MASK          GET WARN MASK          GET FAULT MASK\r\n"
        "    (GET ...MASK legge la maschera CONFIGURATA; GET ERR/WARN/FAULT\r\n"
        "     senza MASK leggono lo stato ATTUALE - vedi anche GET FAULTS)\r\n"
        "    (num: 0x0F2 = 0X0f2  b101 = B101  123 = dec)\r\n"
        "    (FAULT MASK: eventi -> SYS_FAULT non recuperabile - COM/LASEQ/AMC/FLOOD1/FLOOD2/PSU)\r\n"
        "    (ERR MASK: eventi -> SYS_ERROR recuperabile via CERR - FLOW/TEMP/SAB/KEY/DEW)\r\n"
        "-- [LOGIN required] Fault reset + latch persistente (power-cycle) --\r\n"
        "  FRST                                (resetta SYS_FAULT -> IDLE, azzera anche il latch persistente)\r\n"
        "  SET FAULT LATCHMASK <num>           GET FAULT LATCHMASK\r\n"
        "  GET FAULT LATCH                     (bit FAULT_BIT_* attualmente latchati)\r\n"
        "    (LATCHMASK: sottoinsieme di FAULT MASK che persiste al power-cycle -\r\n"
        "     forza INIT -> FAULT diretto al riavvio finche' non si invia FRST;\r\n"
        "     default: solo FLOOD1/FLOOD2)\r\n"
        "-- [LOGIN required] Soglie temperatura + device-enable NTC --\r\n"
        "  SET TEMP <sensor> <WARN|ERR> <MIN|MAX> <degC>\r\n"
        "    sensor: WATER_IN WATER_OUT DRIVER SPLICE DIODE1 DIODE2 AMBIENT LQ_AMBIENT\r\n"
        "            PSU_TEMP PWR_EL_TEMP MB_TEMP (NUOVO 2026-07-22)\r\n"
        "    (diodo laser: 2 sensori NTC indipendenti, DIODE1+DIODE2, dal 2026-07-15)\r\n"
        "    (LQ_AMBIENT: ambiente interno LaseQ (SHT35), soglie separate da AMBIENT\r\n"
        "     -piu' calde per componenti elettronici vicini- dal 2026-07-15)\r\n"
        "    (MB_TEMP: SHT35 su scheda MMC, ex \"ambient\"; PSU_TEMP/PWR_EL_TEMP: NTC\r\n"
        "     non ancora cablati - vedi SET NTC MAP - soglie NUOVO 2026-07-22)\r\n"
        "  SET TEMP MASK <0-0x1FF>  GET TEMP MASK  (bit0=WATER_IN..bit6=DIODE2,bit7=PSU_TEMP,bit8=PWR_EL_TEMP)\r\n"
        "-- [LOGIN required] Mappatura + calibrazione sensori NTC (AD7490) --\r\n"
        "  SET NTC MAP <serigrafia 1-16> <WATER_IN|WATER_OUT|SPLICE|DIODE1|DIODE2|AMBIENT|PSU_TEMP|PWR_EL_TEMP|OFF>\r\n"
        "    (argomento = numero di serigrafia stampato sulla scheda, non canale AD7490 grezzo)\r\n"
        "  SET NTC BETA <serigrafia 1-16> <beta 2000-6000>   (NUOVO 2026-07-22, per-canale)\r\n"
        "  GET NTC MAP   GET NTC BETA   (elenco per serigrafia 1-16)\r\n"
        "-- [LOGIN required] Soglie flusso + device-enable --\r\n"
        "  SET FLOW <WARN|ERR> <MIN|MAX> <val_x10 L/min>\r\n"
        "    (es. 15 = 1.5 L/min)\r\n"
        "  SET FLOW MASK <0-3>      (bit0=FLOW_METER_1 bit1=FLOW_METER_2)\r\n"
        "-- [LOGIN required] Soglie umidita' --\r\n"
        "  SET HUM <MMC|LASEQ> <val_pct>\r\n"
        "-- [LOGIN required] Delta dew point --\r\n"
        "  SET DEW <WARN|ERR> DELTA <degC>\r\n"
        "-- [LOGIN required] Ritardi di sistema --\r\n"
        "  SET DELAY <PSU|SAB|CONTACTOR> <ms>\r\n"
        "-- [LOGIN required] Limiti/maschera PSU (solo stato IDLE) --\r\n"
        "  SET PSU <VOLTAGE|CURRENT> <val>   (mV / mA, inviati subito ad AMC)\r\n"
        "  SET PSU MASK <0-3>                (bit0=PSU1 bit1=PSU2, locale)\r\n"
        "    (SAVE CONFIG per persistere)\r\n"
        "-- [LOGIN required] Compensazione dinamica tensione PSU (solo stato IDLE, NUOVO 2026-07-22) --\r\n"
        "  SET VCOMP ON|OFF                  (abilita/disabilita, solo SW non-QCW)\r\n"
        "  SET VCOMP DELAY  <ms 10-50>       (assestamento dopo accensione laser)\r\n"
        "  SET VCOMP RAMP   <ms 50-100>      (rampa discesa verso target LUT)\r\n"
        "  SET VCOMP RAMPUP <ms 1-5>         (rampa salita, prima del nuovo setpoint a LaseQ)\r\n"
        "    (flag/timing inviati subito ad AMC, SAVE CONFIG per persistere)\r\n"
        "-- [LOGIN required] Maschera contattori (solo stato IDLE) --\r\n"
        "  SET CONTACTOR MASK <0-3>          (bit0=CONTACTOR1 bit1=CONTACTOR2)\r\n"
        "    (SAVE CONFIG per persistere)\r\n"
        "-- [LOGIN required] Maschera alimentazione eFuse --\r\n"
        "  SET EFUSE MASK <0-15>             (bit0=MAIN bit1=SAB bit2=COM bit3=LASEQ)\r\n"
        "    (SAVE CONFIG + power-cycle: applicata SOLO al boot da EFuse_Init())\r\n"
        "  GET MODULES                       (stato ON/OFF + corrente di ognuno)\r\n"
        "-- [LOGIN required] Maschera fotodiodi (solo stato IDLE) --\r\n"
        "  SET PD MASK <0-15>                (bit0=PD1..bit3=PD4)\r\n"
        "    (SAVE CONFIG per persistere, riavvio per applicare lato AMC)\r\n"
        "-- [LOGIN required] Setpoint diretto (bypassa LUT, solo FSM_MODE_SW) --\r\n"
        "  SET CURRENT <mA 0-12000>\r\n"
        "-- Modalita' CW/QCW (solo FSM_MODE_SW, emissione software, nessun login) --\r\n"
        "  SET FREQ <hz 1-50000>   SET DUTY <pct 1-99>   SET QCW ON|OFF   GET REGIME\r\n"
        "    (QCW ON/OFF applicato subito se gia' in EMISSION; PE12 impulsato via TIM16)\r\n"
        "    (duty clampato in automatico se necessario a garantire Ton>=10us: es.\r\n"
        "     a 50000Hz il duty minimo effettivo e' 50%, non 1%)\r\n"
        "-- [LOGIN required] LUT calibrazione --\r\n"
        "  SET LUT GAIN SW|HW t0 t1 t2 t3\r\n"
        "  SET LUT VALID pd SW|HW entry sp min max\r\n"
        "  SET LUT POWER pd win entry adc watt\r\n"
        "  SET LUT SETPOINT idx current_ma   (idx 0-20, 5% x entry; persiste subito)\r\n"
        "  SET LUT VCOMP entry current_ma voltage_mv   (entry 0-9, solo stato IDLE,\r\n"
        "    NUOVO 2026-07-22: LUT compensazione tensione PSU, inviata subito ad AMC;\r\n"
        "    voltage_mv clampato a Vmax/SET PSU VOLTAGE, 0 = entry non valida)\r\n"
        "  SET LUT HWPWR entry adc_raw power_pct   (entry 0-10, RAW ADC PC5->% potenza,\r\n"
        "    NUOVO 2026-08-01: usata solo per reporting GET STATUS/COM STATUS quando\r\n"
        "    ANALOG o SW+SETPOINTHW, nessun invio ad AMC)\r\n"
        "  GET LUT VCOMP|GAIN|VALID pd SW|HW|POWER pd win|SETPOINT|HWPWR   (dump per confronto/diagnostica)\r\n"
        "  SAVE LUT   RESET LUT   SAVE CONFIG\r\n"
        "-- Ingresso in modalita' update via RS485 (solo stato IDLE, nessun LOGIN richiesto) --\r\n"
        "  FW UPDATE    (resetta ed entra nel bootloader in ascolto RS485 per 30s -\r\n"
        "                usare poi rs485_ota_update.py, trigger \"OTAU\")\r\n"
        "  HELP\r\n";

    send_response(help_a);
    send_response(help_b);
}

/* ========================================================================== */
/* PARSE & DISPATCH                                                            */
/* ========================================================================== */

static void parse_and_execute(char *line, uint16_t len)
{
    /* Rimuovi \r\n / null finali */
    while (len > 0U && (line[len-1U] == '\r' || line[len-1U] == '\n' ||
                        line[len-1U] == '\0')) {
        line[--len] = '\0';
    }
    if (len == 0U) return;

    /* str_upper prima della tokenizzazione — i numeri hex tipo 0x0f2
     * diventano 0X0F2 (gestito da parse_number via toupper interno) */
    str_upper(line);

    char *tok[12] = {0};
    int   n       = tokenize(line, tok, 12);
    if (n == 0) return;

    /* Controllo timeout sessione ad ogni comando */
    check_session_timeout();

    /* Dispatch */
    if      (strcmp(tok[0], "START")    == 0) { cmd_start(); }
    else if (strcmp(tok[0], "STOP")     == 0) { cmd_stop();  }
    else if (strcmp(tok[0], "SON")      == 0) { cmd_son();   }
    else if (strcmp(tok[0], "SOFF")     == 0) { cmd_soff();  }
    else if (strcmp(tok[0], "SEN")      == 0) { cmd_sen();   }
    else if (strcmp(tok[0], "SDIS")     == 0) { cmd_sdis();  }
    else if (strcmp(tok[0], "PON")      == 0) { cmd_pon();   }
    else if (strcmp(tok[0], "POFF")     == 0) { cmd_poff();  }
    else if (strcmp(tok[0], "CERR")     == 0) { cmd_cerr();  }
    else if (strcmp(tok[0], "FRST")     == 0) { cmd_frst();  }
    else if (strcmp(tok[0], "LOGIN")    == 0) { cmd_login(tok, n);  }
    else if (strcmp(tok[0], "LOGOUT")   == 0) { cmd_logout();       }
    else if (strcmp(tok[0], "SET")      == 0) { cmd_set(tok, n);    }
    else if (strcmp(tok[0], "GET")      == 0) { cmd_get(tok, n);    }
    else if (strcmp(tok[0], "LOG")      == 0) { cmd_log(tok, n);    }
    else if (strcmp(tok[0], "SAVE")  == 0 && n >= 2 && strcmp(tok[1], "LUT")    == 0) { cmd_save_lut();    }
    else if (strcmp(tok[0], "SAVE")  == 0 && n >= 2 && strcmp(tok[1], "CONFIG") == 0) { cmd_save_config(); }
    else if (strcmp(tok[0], "RESET") == 0 && n >= 2 && strcmp(tok[1], "LUT")    == 0) { cmd_reset_lut();   }
    else if (strcmp(tok[0], "FW")    == 0 && n >= 2 && strcmp(tok[1], "UPDATE") == 0) { cmd_fw_update();   }
    else if (strcmp(tok[0], "HELP")     == 0) { cmd_help(); }
    else { err("unknown command - type HELP"); }
}

/* ========================================================================== */
/* API PUBBLICA                                                                */
/* ========================================================================== */

void Rs485Cmd_Init(void)
{
    s_task_handle = osThreadGetId();
    s_rx_size  = 0U;

    static const osMutexAttr_t uart_mutex_attr = {
        .name      = "uart1_mtx",
        .attr_bits = osMutexPrioInherit,
    };
    s_uart_mutex = osMutexNew(&uart_mutex_attr);

    /*
     * Cancella flag di errore UART accumulati durante il boot
     * (ORE può essere già settato se il master ha inviato dati prima che
     * il task partisse) e svuota RDR residuo.
     */
    __HAL_UART_CLEAR_FLAG(&huart1, UART_CLEAR_OREF | UART_CLEAR_FEF | UART_CLEAR_NEF);
    (void)READ_REG(huart1.Instance->RDR);

    /* Avvia ricezione DMA+IDLE. HT disabilitato: per frame brevi
     * vogliamo solo l'evento IDLE, non la callback a metà buffer. */
    HAL_UARTEx_ReceiveToIdle_DMA(&huart1, s_dma_buf, sizeof(s_dma_buf));
    __HAL_DMA_DISABLE_IT(huart1.hdmarx, DMA_IT_HT);
}

/*
 * Chiamata da HAL_UARTEx_RxEventCallback in ISR per ogni evento IDLE/TC su USART1.
 * Size = numero di byte ricevuti nel buffer DMA dall'ultimo avvio.
 * Segnala il task; la D-cache viene invalidata nel task prima della lettura.
 */
void Rs485Cmd_OnRxEvent(uint16_t size)
{
    if (size == 0U) { return; }
    s_rx_size = size;
    if (s_task_handle != NULL) {
        osThreadFlagsSet(s_task_handle, RS485_RX_FLAG);
    }
}

/*
 * Chiamata da HAL_UART_ErrorCallback (ISR) quando UART segnala ORE/FE/NE.
 * Cancella i flag e riavvia DMA per non bloccare la ricezione.
 */
void Rs485Cmd_OnError(void)
{
    __HAL_UART_CLEAR_FLAG(&huart1, UART_CLEAR_OREF | UART_CLEAR_FEF | UART_CLEAR_NEF);
    HAL_UARTEx_ReceiveToIdle_DMA(&huart1, s_dma_buf, sizeof(s_dma_buf));
    __HAL_DMA_DISABLE_IT(huart1.hdmarx, DMA_IT_HT);
}

/*
 * Chiamata da HAL_UART_TxCpltCallback (ISR) per USART1.
 * Segnala al task RS485 che il DMA TX ha completato la trasmissione;
 * send_response() è in attesa su RS485_TX_FLAG con osThreadFlagsWait().
 */
void Rs485Cmd_OnTxCplt(void)
{
    if (s_task_handle != NULL) {
        osThreadFlagsSet(s_task_handle, RS485_TX_FLAG);
    }
}

void Rs485Cmd_Update(void)
{
    /*
     * Quando il log è attivo usiamo un timeout sul wait:
     *   - se arriva un frame prima dello scadere → esegui il comando
     *   - allo scadere del timeout → invia la riga di log
     * Quando il log è inattivo, il wait è comunque limitato (non più
     * osWaitForever): senza traffico RS485 per lungo tempo è normale, non
     * un blocco, ma Task_RS485 deve poter richiamare Watchdog_Heartbeat()
     * periodicamente. Lo scadere del timeout qui sotto non fa nulla di
     * diverso da prima (nessun log, nessun comando) tranne permettere
     * al chiamante di rilevare che il task è ancora vivo.
     */
    uint32_t timeout = s_log_active ? s_log_interval_ms : RS485_IDLE_HEARTBEAT_MS;
    uint32_t flags   = osThreadFlagsWait(RS485_RX_FLAG, osFlagsWaitAny, timeout);

    /* Controllo scadenza sessione (anche senza comandi, se log attivo) */
    check_session_timeout();

    if ((flags & osFlagsError) != 0U) {
        /*
         * osFlagsErrorTimeout: scaduto il timeout senza frame ricevuto.
         * Invia la riga di log periodica.
         */
        if (s_log_active) {
            send_log_row();
        }
        return;
    }

    /* Frame ricevuto → invalida cache, copia, rilancia DMA, parsa */
    uint16_t sz = s_rx_size;
    if (sz > RS485_CMD_BUF_SIZE - 1U) sz = RS485_CMD_BUF_SIZE - 1U;

    /*
     * H7 D-cache: il DMA ha scritto in RAM bypassando la cache.
     * Invalidiamo le linee interessate (32 byte allineati) prima di leggere,
     * altrimenti la CPU vedrebbe dati stantii dalla cache.
     */
    SCB_InvalidateDCache_by_Addr((volatile void *)s_dma_buf, (int32_t)sizeof(s_dma_buf));

    memcpy(s_cmd_buf, s_dma_buf, sz);
    s_cmd_buf[sz] = '\0';

    restart_rx();
    parse_and_execute(s_cmd_buf, sz);
}

/* ========================================================================== */
/* NOTIFICA FAULT NON SOLLECITATA                                              */
/* ========================================================================== */

void Rs485Cmd_NotifyFault(SysEvent_t ev, bool is_alarm)
{
    (void)ev;   /* Il nome/dettaglio viene ora da FSM_GetLastFaultName()/
                 * GetLastErrorName() sotto, non da una tabella locale — vedi
                 * commento subito sotto. */

    /*
     * Log errori RS485 standard (LOG ERR ON/OFF): se non attivo, nessuna
     * notifica per NESSUNA causa di SYS_ERROR/SYS_FAULT — comportamento
     * uniforme per tutti gli eventi, non più limitato a FLOW/TEMP/fault
     * hardware. Default OFF ad ogni avvio (s_log_err_enabled è un flag di
     * sessione, mai in config/flash).
     *
     * Esempi:
     *   ALARM FLOW         flusso fuori range
     *   ALARM WATER_IN|DIODE1   temperatura NTC / LaseQ fuori range (sorgenti specifiche, dal 2026-07-15)
     *   ALARM KEY_A        chiave A richiesta mancante/rimossa
     *   ALARM SAB_INTLCK   interlock SAB aperto
     *   FAULT FLOOD1       sensore allagamento 1 attivo
     *   FAULT PSU2         anomalia alimentatore 2
     *   FAULT INIT_CHECK   check hardware boot fallito
     *
     * Il nome/dettaglio viene da FSM_GetLastFaultName()/GetLastErrorName()
     * (dal 2026-07-15): stessa stringa mostrata da "GET FAULT"/"GET ALARM",
     * già disambiguata per sorgente quando possibile (KEY_A/KEY_B, LID1/LID2,
     * FLOOD1/FLOOD2, PSU1/PSU2, TEMP:<sensore>...) — vedi event_detail_name()
     * in fsm.c. s_last_fault_event/s_last_error_event (e i relativi dettagli)
     * sono già stati aggiornati da action_fault()/action_enter_error() PRIMA
     * di chiamare questa funzione, quindi la lettura qui è sempre coerente
     * con "ev".
     */
    if (!s_log_err_enabled) {
        return;
    }

    char buf[96];
    snprintf(buf, sizeof(buf), "%s %s\r\n",
             is_alarm ? "ALARM" : "FAULT",
             is_alarm ? FSM_GetLastErrorName() : FSM_GetLastFaultName());
    uint16_t len = (uint16_t)strlen(buf);

    /*
     * Esclusivo su huart1 (vedi s_uart_mutex): questa funzione gira nel task
     * FSM, non nel task RS485, quindi può contendere la periferica con
     * send_response(). Timeout breve e limitato: le azioni di sicurezza in
     * action_fault()/action_enter_error() sono già state eseguite PRIMA di
     * questa chiamata (è l'ultimo passo), quindi se il mutex non si libera
     * in tempo si rinuncia alla notifica RS485 invece di bloccare il task
     * FSM — la sicurezza non deve mai dipendere dalla riuscita di questo invio.
     */
    if (osMutexAcquire(s_uart_mutex, 50U) != osOK) {
        return;
    }

    /* Trasmissione bloccante breve: non richiede task RS485 attivo.
     * Timeout 20ms sufficiente per <32 byte a 115200 baud. */
    HAL_UART_Transmit(&huart1, (const uint8_t *)buf, len, 20U);

    osMutexRelease(s_uart_mutex);
}
