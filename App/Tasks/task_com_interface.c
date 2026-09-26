/*
 * task_com_interface.c
 *
 * Vedere task_com_interface.h per la documentazione (perche' un task
 * separato da task_comms.c, note sul watchdog).
 */

#include "FreeRTOS.h"
#include "task.h"
#include "task_com_interface.h"
#include "COM_interface.h"
#include "COM_interface_app.h"  /* COM_App_HandleStatus/Control/Config() */
#include "hal_handles.h"   /* hspi3 */
#include "sys_log.h"
#include <stdbool.h>
#include <string.h>

/* ========================================================================== */
/* --- ALLOCAZIONE STATICA --- */
/* ========================================================================== */

static StaticTask_t taskComInterface_tcb;
/*
 * 256 word (1KB) erano sufficienti finche' questo task faceva solo l'echo
 * di COM_MSG_TEST. Da quando gestisce STATUS/CONTROL/CONFIG (vedi
 * COM_interface_app.c) la profondita' di chiamata e' cresciuta parecchio:
 * FSM/Config/TaskMonitor/Setpoint/QCW/SHT35/FlowMeter/TaskAmc/LaseQ/eFuse/
 * SAB/RTC, con in mezzo Config_Save() (App/Config/config.c) che da solo usa
 * un locale ConfigRecord_t di 352 byte per costruire il record da scrivere
 * in flash. Con 1KB totale il rischio concreto e' uno stack overflow
 * silenzioso: configCHECK_FOR_STACK_OVERFLOW e' 0 in FreeRTOSConfig.h (il
 * vApplicationStackOverflowHook() in freertos.c non scatta mai), quindi un
 * overflow qui non fermerebbe nulla in modo visibile — corromperebbe
 * silenziosamente la RAM adiacente (altro task o stato di COM_interface.c,
 * es. s_busy), lasciando il task bloccato a meta' scambio: nessuna risposta
 * a COM, EndExchange() mai chiamata, e da quel momento in poi ogni fronte
 * successivo su nCOM_INT_IN viene ignorato (s_busy resta true per sempre).
 * Portato a 768 word (3KB), in linea con gli altri task che fanno math in
 * float e chiamate annidate (task_monitor: 640, task_amc/task_comms: 512).
 */
static StackType_t  taskComInterface_stack[768];

const osThreadAttr_t taskComInterface_attr = {
    .name       = "ComInterface",
    .stack_mem  = &taskComInterface_stack[0],
    .stack_size = sizeof(taskComInterface_stack),
    .priority   = osPriorityNormal2,
    .cb_mem     = &taskComInterface_tcb,
    .cb_size    = sizeof(taskComInterface_tcb),
};

/* ========================================================================== */
/* --- SEMAFORI PRIVATI --- */
/* ========================================================================== */

/* Dato da COM_Interface_MsgReady_EXTI_Callback() (ISR), atteso qui: sveglia
 * il task quando la COM interface segnala un messaggio pronto. */
static osSemaphoreId_t sem_msg_pending = NULL;

/* Dato da COM_Interface_SPI_Cplt_Callback()/_Error_Callback() (ISR DMA),
 * atteso dopo ogni singola transazione SPI (ce ne sono due per scambio:
 * richiesta poi risposta — vedi COM_interface.h). */
static osSemaphoreId_t sem_xfer_cplt = NULL;

static void OnMsgPending(void) { osSemaphoreRelease(sem_msg_pending); }
static void OnXferCplt(void)   { osSemaphoreRelease(sem_xfer_cplt); }

/* ========================================================================== */
/* --- BUFFER RISPOSTA APPLICATIVA --- */
/* ========================================================================== */

/* Buffer statico (non sullo stack, vedi taskComInterface_stack sopra: solo
 * 1KB) per il payload di risposta STATUS/CONTROL/CONFIG costruito da
 * COM_interface_app.c prima di COM_Interface_SendResponse(). Un solo scambio
 * alla volta e' in corso (task singolo, s_busy in COM_interface.c), nessuna
 * sincronizzazione aggiuntiva necessaria. */
static uint8_t s_resp_payload[COM_PAYLOAD_SIZE];

/* ========================================================================== */
/* --- COSTANTI --- */
/* ========================================================================== */

/*
 * Timeout attesa completamento di UNA transazione DMA SPI3. Valore
 * PROVVISORIO (largo margine): a COM_SPI_PRESCALER attuale (/64, vedi
 * COM_interface.c) un frame di 256 byte richiede circa 0.7ms — 50ms copre
 * ampiamente eventuale jitter di scheduling. Da rivedere insieme al
 * prescaler quando la velocita' di collegamento sara' definita.
 */
#define COM_XFER_TIMEOUT_MS   50U

/*
 * Delay minimo tra la fine della prima transazione (richiesta, catturata in
 * COM_Interface_GetRequestFrame()) e l'avvio della seconda (risposta,
 * COM_Interface_SendResponse()). Richiesto dal team COM interface: la loro
 * callback di fine TX (HAL_SPI_TxCpltCallback, vedi ALITE_COM/com_spi.c)
 * riarma HAL_SPI_Receive_DMA() in modo asincrono rispetto a questo task;
 * senza un minimo margine il MMC potrebbe generare il clock della seconda
 * transazione prima che la COM interface abbia finito di riarmare la
 * ricezione. 1 tick e' ampio margine (la COM interface impiega tipicamente
 * pochi microsecondi) senza impattare il periodo di 10s del test. In
 * futuro si potra' rimuovere riusando nCOM_INT_OUT come segnale esplicito
 * "COM pronta alla ricezione" (vedi COM_interface.h).
 */
#define COM_RESPONSE_DELAY_TICKS   1U

/* ========================================================================== */
/* --- TASK --- */
/* ========================================================================== */

void TaskComInterface_Run(void *argument)
{
    (void)argument;

    for (;;) {
        /* Attesa indefinita: la COM interface scrive solo quando ha un
         * messaggio nuovo, nessun traffico periodico atteso (vedi banner
         * WATCHDOG in task_com_interface.h — questo task NON chiama
         * Watchdog_Heartbeat() di proposito). */
        osSemaphoreAcquire(sem_msg_pending, osWaitForever);

        /*
         * Drain difensivo: se una transazione precedente e' finita in
         * errore/timeout lasciando un token orfano su sem_xfer_cplt (stesso
         * schema di laseq_transact() in task_comms.c), consumalo qui prima
         * di avviare la nuova richiesta, altrimenti la prima Acquire sotto lo
         * consumerebbe subito senza attendere il vero completamento DMA.
         */
        (void)osSemaphoreAcquire(sem_xfer_cplt, 0U);

        if (!COM_Interface_BeginRequest()) {
            /* Errore HAL immediato: cleanup gia' fatto internamente
             * (mutex rilasciato, busy azzerato) — si riparte dal prossimo
             * fronte nCOM_INT_IN. */
            SysLog_Event(LOG_WARN, "COM interface: avvio transazione richiesta fallito");
            continue;
        }

        if (osSemaphoreAcquire(sem_xfer_cplt, COM_XFER_TIMEOUT_MS) != osOK) {
            SysLog_Event(LOG_WARN, "COM interface: timeout transazione richiesta");
            COM_Interface_AbortAndEnd();
            continue;
        }

        const uint8_t *request = COM_Interface_GetRequestFrame();
        if (request == NULL) {
            /* Transazione DMA fallita, oppure frame non valido (START/STOP/
             * CRC) — vedi COM_Interface_GetRequestFrame(). Nessuna risposta
             * da inviare in questo giro: la COM interface e' tenuta a
             * ripresentare la richiesta se necessario (stesso principio del
             * fronte ignorato mentre busy, vedi COM_interface.h). */
            SysLog_Event(LOG_WARN, "COM interface: richiesta assente/non valida");
            COM_Interface_EndExchange();
            continue;
        }

        /*
         * COM_MSG_TEST (test SPI periodico, vedi ALITE_COM/com_dispatch.c
         * COM_build_test_request()/COM_parse_reply()): la COM interface si
         * aspetta indietro lo STESSO payload ricevuto (payload[i] = i, ma
         * qui si fa un echo puro byte-per-byte, senza assumere il pattern,
         * cosi' funziona qualunque contenuto la COM interface invii).
         *
         * STATUS/CONTROL/CONFIG (protocollo applicativo, vedi
         * COM_interface_app.h/.c): 'request' resta valido solo fino alla
         * prossima COM_Interface_BeginRequest(), quindi il payload va
         * interpretato/copiato ORA, prima di costruire ed inviare la
         * risposta in s_resp_payload (buffer statico, non sullo stack: il
         * task ha solo 1KB di stack, vedi taskComInterface_stack sopra).
         */
        const uint8_t req_msg_type = request[COM_OFF_MSG_TYPE];

        switch ((COM_MsgType_t)req_msg_type) {
        case COM_MSG_STATUS:
            COM_App_HandleStatus(&request[COM_OFF_PAYLOAD], s_resp_payload);
            break;
        case COM_MSG_CONTROL:
            COM_App_HandleControl(&request[COM_OFF_PAYLOAD], s_resp_payload);
            break;
        case COM_MSG_CONFIG:
            COM_App_HandleConfig(&request[COM_OFF_PAYLOAD], s_resp_payload);
            break;
        default:
            /* COM_MSG_TEST gestito sotto con l'echo dedicato; qualunque
             * altro codice inatteso -> risposta EMPTY a payload zero. */
            memset(s_resp_payload, 0, COM_PAYLOAD_SIZE);
            break;
        }

        /* Vedi COM_RESPONSE_DELAY_TICKS sopra: da' tempo alla COM interface
         * di riarmare HAL_SPI_Receive_DMA() prima della seconda transazione. */
        osDelay(COM_RESPONSE_DELAY_TICKS);

        bool send_ok;
        if (req_msg_type == (uint8_t)COM_MSG_TEST) {
            send_ok = COM_Interface_SendResponse(COM_MSG_TEST, &request[COM_OFF_PAYLOAD]);
        } else if (req_msg_type == (uint8_t)COM_MSG_STATUS
                   || req_msg_type == (uint8_t)COM_MSG_CONTROL
                   || req_msg_type == (uint8_t)COM_MSG_CONFIG) {
            send_ok = COM_Interface_SendResponse((COM_MsgType_t)req_msg_type, s_resp_payload);
        } else {
            send_ok = COM_Interface_SendResponse(COM_MSG_EMPTY, NULL);
        }

        if (!send_ok) {
            SysLog_Event(LOG_WARN, "COM interface: avvio transazione risposta fallito");
            COM_Interface_EndExchange();
            continue;
        }

        if (osSemaphoreAcquire(sem_xfer_cplt, COM_XFER_TIMEOUT_MS) != osOK) {
            SysLog_Event(LOG_WARN, "COM interface: timeout transazione risposta");
            COM_Interface_AbortAndEnd();
            continue;
        }

        if (!COM_Interface_LastXferOk()) {
            SysLog_Event(LOG_WARN, "COM interface: invio risposta fallito");
        }

        COM_Interface_EndExchange();
    }
}

/* ========================================================================== */
/* --- INIZIALIZZAZIONE --- */
/* ========================================================================== */

void TaskComInterface_Init(void)
{
    sem_msg_pending = osSemaphoreNew(1, 0, NULL);
    sem_xfer_cplt   = osSemaphoreNew(1, 0, NULL);

    COM_Interface_Init(&hspi3, OnMsgPending, OnXferCplt);
}
