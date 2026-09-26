/*
 * sys_log.c
 *
 * Logger di sistema su scheda microSD. Vedere sys_log.h per documentazione.
 *
 * ARCHITETTURA:
 *   - FatFS su SPI3 via sd_spi driver.
 *   - Un file per giorno: LOG_YYYYMMDD.TXT (append).
 *   - Mutex FreeRTOS per accesso da task multipli.
 *   - Buffer statico locale per snprintf (no allocazione dinamica).
 *   - File aperto e chiuso ad ogni scrittura: garantisce consistenza in caso
 *     di reset/power-off improvviso (overhead accettabile per log asincroni).
 *
 * DIPENDENZE:
 *   - FatFS (CubeMX FATFS middleware, ff.h)
 *   - sd_spi.h (driver fisico SD)
 *   - hal_handles.h (hrtc)
 *   - fsm.h (SysState_t per SysLog_StateChange)
 *   - cmsis_os.h (mutex FreeRTOS)
 */

#include "sys_log.h"
#include "sd_spi.h"
#include "hal_handles.h"
#include "SPI3Bus.h"     /* mutex condiviso SPI3 (SD vs COM interface) */
#include "ff.h"          /* FatFS */
#include "cmsis_os.h"
#include <stdio.h>
#include <string.h>
#include <stdarg.h>

/* ============================================================================
 * ============================================================================
 * MODIFICA TEMPORANEA DI DEBUG (2026-07-29) — DA RIMUOVERE A TEST CONCLUSO.
 *
 * Per isolare un lockup di hspi3 lato COM interface (SPI3 condivisa con la
 * SD, vedi SPI3Bus.h/COM_interface.c) si vuole verificare se il blocco
 * sparisce escludendo del tutto la SD dal bus, cosi' SPI3 resta dedicata
 * SOLO a COM interface per la durata del test.
 *
 * Con SYSLOG_SD_DISABLED_FOR_COM_DEBUG a 1, SysLog_Init() NON chiama piu'
 * SD_Init()/f_mount(): la SD non viene mai toccata (ne' inizializzata ne'
 * montata), quindi ne' sd_spi.c ne' lo strato FatFS/user_diskio.c arrivano
 * mai a usare hspi3 — l'unico utente rimasto del bus e' COM_interface.c.
 * Il logger si comporta come se la SD non fosse presente: s_ready resta
 * false, ogni SysLog_Event()/SysLog_StateChange() diventa un no-op sicuro
 * (vedi il check "if (!s_ready ...) return;" sotto — nessun altro modulo
 * dipende dalla SD su questa board, la ricerca cross-progetto conferma che
 * f_mount()/SD_Init() sono chiamati SOLO da qui).
 *
 * RIPRISTINO: rimettere questa macro a 0 (o eliminare il blocco #if) per
 * tornare al comportamento normale.
 * ============================================================================
 * ============================================================================ */
#define SYSLOG_SD_DISABLED_FOR_COM_DEBUG   1

/* ============================================================================
 * COSTANTI
 * ============================================================================ */

/* Directory log nella root della SD (vuota = root) */
#define LOG_DIR          ""

/* Lunghezza riga completa: [YYYY-MM-DD HH:MM:SS] [LEVEL] <msg>\r\n + null */
#define LOG_LINE_BUF     (32U + SYSLOG_MSG_MAX_LEN + 4U)

#define LOG_MUTEX_TIMEOUT_MS   500U

/* ============================================================================
 * STATO INTERNO
 * ============================================================================ */

static osMutexId_t  s_mutex    = NULL;
static bool         s_ready    = false;   /* SD inizializzata + FatFS montato */
static FATFS        s_fatfs;              /* Oggetto filesystem FatFS */

/* ============================================================================
 * UTILITY: STATO / LIVELLO → STRINGA
 * ============================================================================ */

static const char *level_str(SysLog_Level_t level)
{
    switch (level) {
        case LOG_DEBUG: return "DEBUG";
        case LOG_INFO:  return "INFO ";
        case LOG_WARN:  return "WARN ";
        case LOG_ERROR: return "ERROR";
        default:        return "?????";
    }
}

static const char *state_str(SysState_t s)
{
    switch (s) {
        case SYS_INIT:     return "INIT";
        case SYS_IDLE:     return "IDLE";
        case SYS_ACTIVE:   return "ACTIVE";
        case SYS_ON:       return "ON";
        case SYS_ENABLED:  return "ENABLED";
        case SYS_EMISSION: return "EMISSION";
        case SYS_ERROR:    return "ERROR";
        case SYS_FAULT:    return "FAULT";
        default:           return "UNKNOWN";
    }
}

/* ============================================================================
 * UTILITY: TIMESTAMP + NOME FILE
 * ============================================================================ */

/*
 * Legge RTC e riempie i campi out_* (tutti in valori binari, non BCD).
 * Restituisce false se RTC non disponibile (usa valori di default).
 */
static bool rtc_get(uint16_t *year, uint8_t *month, uint8_t *day,
                    uint8_t *hour, uint8_t *min,   uint8_t *sec)
{
    RTC_TimeTypeDef t = {0};
    RTC_DateTypeDef d = {0};

    if (HAL_RTC_GetTime(&hrtc, &t, RTC_FORMAT_BIN) != HAL_OK ||
        HAL_RTC_GetDate(&hrtc, &d, RTC_FORMAT_BIN) != HAL_OK) {
        *year = 2000; *month = 1; *day = 1;
        *hour = 0;    *min   = 0; *sec = 0;
        return false;
    }

    *year  = (uint16_t)(d.Year + 2000U);
    *month = d.Month;
    *day   = d.Date;
    *hour  = t.Hours;
    *min   = t.Minutes;
    *sec   = t.Seconds;
    return true;
}

/*
 * Costruisce il nome file: "LOG_YYYYMMDD.TXT"
 */
static void make_filename(char *buf, size_t buf_len,
                          uint16_t year, uint8_t month, uint8_t day)
{
    snprintf(buf, buf_len, "LOG_%04u%02u%02u.TXT", year, month, day);
}

/* ============================================================================
 * SCRITTURA EFFETTIVA
 * ============================================================================ */

/*
 * Apre il file (append), scrive la riga, chiude. Thread-safe tramite mutex.
 * Chiamata solo dall'interno del mutex.
 */
static void write_line(const char *line)
{
    if (!s_ready) return;

    uint16_t year; uint8_t month, day, hour, min, sec;
    rtc_get(&year, &month, &day, &hour, &min, &sec);

    char fname[20];
    make_filename(fname, sizeof(fname), year, month, day);

    FIL f;
    FRESULT fr = f_open(&f, fname, FA_OPEN_APPEND | FA_WRITE);
    if (fr != FR_OK) return;

    UINT written;
    f_write(&f, line, strlen(line), &written);
    f_close(&f);
}

/* ============================================================================
 * API PUBBLICA
 * ============================================================================ */

void SysLog_Init(void)
{
    /* Crea mutex solo alla prima chiamata */
    if (s_mutex == NULL) {
        s_mutex = osMutexNew(NULL);
    }

    s_ready = false;

#if SYSLOG_SD_DISABLED_FOR_COM_DEBUG
    /* Vedi banner "MODIFICA TEMPORANEA DI DEBUG" in cima al file: SD
     * volutamente non toccata, SPI3 lasciata libera per il solo COM
     * interface durante il test del lockup hspi3. */
    return;
#else
    /* Inizializza SD — SPI3Bus_GetMutex(): mutex condiviso SPI3 (vedi SPI3Bus.h) */
    SD_Result_t sd_r = SD_Init(&hspi3, SPI3Bus_GetMutex());
    if (sd_r != SD_OK) return;

    /* Monta FatFS */
    FRESULT fr = f_mount(&s_fatfs, "", 1);  /* "": default drive, 1: mount now */
    if (fr != FR_OK) return;

    s_ready = true;

    /* Testa accesso scrivendo una riga vuota */
    uint16_t year; uint8_t month, day, hour, min, sec;
    rtc_get(&year, &month, &day, &hour, &min, &sec);
    char fname[20];
    make_filename(fname, sizeof(fname), year, month, day);

    FIL f;
    fr = f_open(&f, fname, FA_OPEN_APPEND | FA_WRITE);
    if (fr != FR_OK) {
        s_ready = false;
        return;
    }
    f_close(&f);
#endif /* SYSLOG_SD_DISABLED_FOR_COM_DEBUG */
}

void SysLog_Event(SysLog_Level_t level, const char *fmt, ...)
{
    if (level < SYSLOG_MIN_LEVEL) return;
    if (!s_ready || s_mutex == NULL) return;

    if (osMutexAcquire(s_mutex, LOG_MUTEX_TIMEOUT_MS) != osOK) return;

    uint16_t year; uint8_t month, day, hour, min, sec;
    rtc_get(&year, &month, &day, &hour, &min, &sec);

    /* Formatta il messaggio utente */
    char msg_buf[SYSLOG_MSG_MAX_LEN + 1U];
    va_list args;
    va_start(args, fmt);
    vsnprintf(msg_buf, sizeof(msg_buf), fmt, args);
    va_end(args);

    /* Componi riga completa */
    char line[LOG_LINE_BUF];
    snprintf(line, sizeof(line),
             "[%04u-%02u-%02u %02u:%02u:%02u] [%s] %s\r\n",
             year, month, day, hour, min, sec,
             level_str(level), msg_buf);

    write_line(line);

    osMutexRelease(s_mutex);
}

void SysLog_StateChange(SysState_t from, SysState_t to)
{
    SysLog_Event(LOG_INFO, "FSM: %s -> %s", state_str(from), state_str(to));
}

void SysLog_Boot(void)
{
    if (!s_ready || s_mutex == NULL) return;

    if (osMutexAcquire(s_mutex, LOG_MUTEX_TIMEOUT_MS) != osOK) return;

    const char *sep =
        "------------------------------------------------------------\r\n";
    write_line(sep);

    osMutexRelease(s_mutex);

    SysLog_Event(LOG_INFO, "=== System boot ===");
}
