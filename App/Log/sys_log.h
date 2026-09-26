/*
 * sys_log.h
 *
 * Logger di sistema su scheda microSD (FatFS).
 *
 * SCOPO:
 *   Salvare su SD gli eventi rilevanti della macchina: transizioni di stato FSM,
 *   errori, warning, eventi SAB, fault eFuse. Ogni entry è una riga ASCII con
 *   timestamp ISO-8601 e livello di severità.
 *
 * FORMATO LOG:
 *   [YYYY-MM-DD HH:MM:SS] [LEVEL] MESSAGE\r\n
 *   Esempio:
 *   [2024-06-15 14:23:01] [INFO ] FSM: IDLE -> ACTIVE
 *   [2024-06-15 14:23:05] [WARN ] eFuse LASEQ: fault asserted
 *   [2024-06-15 14:23:07] [ERROR] SAB: interlock open during EMISSION
 *
 * NOME FILE:
 *   Un file per giorno: LOG_YYYYMMDD.TXT nella root della SD.
 *   Se la SD non è presente o non è inizializzata, le chiamate sono no-op
 *   (il logger è completamente tollerante ai guasti della SD).
 *
 * THREAD SAFETY:
 *   Le funzioni SysLog_* sono thread-safe: usano un mutex FreeRTOS interno.
 *   Possono essere chiamate da qualsiasi task dopo SysLog_Init().
 *   NON chiamare da ISR.
 *
 * UTILIZZO:
 *   // Da freertos.c / MX_FREERTOS_Init():
 *   SysLog_Init();
 *
 *   // Da FSM action:
 *   SysLog_StateChange(prev_state, new_state);
 *
 *   // Da sab_event_cb:
 *   SysLog_Event(LOG_ERROR, "SAB: interlock open");
 *
 *   // Da task_monitor:
 *   SysLog_Event(LOG_WARN, "eFuse LASEQ: fault");
 */

#ifndef APP_LOG_SYS_LOG_H_
#define APP_LOG_SYS_LOG_H_

#include "fsm.h"   /* SysState_t */
#include <stdint.h>

/* ========================================================================== */
/* --- LIVELLI DI LOG --- */
/* ========================================================================== */

typedef enum {
    LOG_DEBUG = 0,   /**< Dettagli diagnostici (normalmente disabilitati) */
    LOG_INFO,        /**< Transizioni di stato, eventi normali            */
    LOG_WARN,        /**< Anomalie non critiche (eFuse fault, warning)    */
    LOG_ERROR,       /**< Errori che causano SYS_ERROR o SYS_FAULT        */
} SysLog_Level_t;

/* ========================================================================== */
/* --- CONFIGURAZIONE --- */
/* ========================================================================== */

/**
 * Livello minimo di log scritto su SD.
 * Cambiare in LOG_DEBUG per massimo dettaglio (aumenta I/O sulla SD).
 */
#define SYSLOG_MIN_LEVEL    LOG_INFO

/**
 * Lunghezza massima del messaggio (escluso timestamp e livello).
 * L'intera riga (timestamp + livello + messaggio) non supera ~120 caratteri.
 */
#define SYSLOG_MSG_MAX_LEN  80U

/* ========================================================================== */
/* --- API --- */
/* ========================================================================== */

/**
 * @brief  Inizializza il logger.
 *         Monta il filesystem FatFS, crea/apre il file di log del giorno,
 *         crea il mutex interno. Sicura da chiamare più volte.
 *         Da richiamare da un task FreeRTOS dopo l'avvio dello scheduler.
 *
 * @note   Se SD non presente o errore FatFS: tutte le chiamate successive
 *         sono silenziosamente ignorati (no-op).
 */
void SysLog_Init(void);

/**
 * @brief  Scrive un evento generico nel log.
 *
 * @param  level  Severità dell'evento.
 * @param  fmt    Formato printf. Lunghezza risultante ≤ SYSLOG_MSG_MAX_LEN.
 *
 * @example
 *   SysLog_Event(LOG_WARN, "eFuse %d: fault asserted", EFUSE_LASEQ);
 *   SysLog_Event(LOG_ERROR, "SAB timeout after %ums", elapsed);
 */
void SysLog_Event(SysLog_Level_t level, const char *fmt, ...);

/**
 * @brief  Scrive una transizione di stato FSM nel log.
 *         Usa livello LOG_INFO. Stampa i nomi testuali degli stati.
 *
 * @param  from  Stato precedente.
 * @param  to    Stato successivo.
 *
 * @example
 *   SysLog_StateChange(SYS_IDLE, SYS_ACTIVE);
 *   // → [2024-06-15 14:23:05] [INFO ] FSM: IDLE -> ACTIVE
 */
void SysLog_StateChange(SysState_t from, SysState_t to);

/**
 * @brief  Scrive una riga separatrice + header di avvio nel log.
 *         Da chiamare all'avvio del sistema (dopo SysLog_Init) per
 *         delimitare sessioni distinte nello stesso file giornaliero.
 */
void SysLog_Boot(void);

#endif /* APP_LOG_SYS_LOG_H_ */
