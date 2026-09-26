/*
 * COM_interface.c
 *
 * Implementazione del driver COM interface: sorveglianza alimentazione +
 * trasporto SPI3 a lunghezza fissa. Vedere COM_interface.h per la
 * documentazione completa (sequenza di sincronismo, uso da
 * task_com_interface.c).
 */

#include "COM_interface.h"
#include "main.h"          /* Pin defines: COM_SPI_nCS_*, nCOM_PWR_FLT_* */
#include "cmsis_os.h"      /* osWaitForever (usato con SPI3Bus_Acquire) */
#include "SPI3Bus.h"        /* Mutex condiviso SPI3 (SD vs COM interface) */
#include "queues.h"        /* Queue_PostEvent() */
#include "fsm_events.h"    /* SYS_COM_FAULT_EVENT */
#include "config.h"        /* g_config.fault_mask */
#include "task_monitor.h"  /* FAULT_BIT_COM, TaskMonitor_SetFaultBit() */
#include <string.h>

/* ========================================================================== */
/* --- BUFFER DMA --- */
/* ========================================================================== */

/*
 * H7 D-Cache (abilitata in main.c): il DMA legge/scrive la RAM fisica
 * bypassando la D-Cache, serve manutenzione esplicita (stesso schema di
 * LaseQ.c/rs485_cmd.c):
 *   - TX: SCB_CleanDCache_by_Addr() dopo aver scritto il frame, PRIMA di
 *     avviare HAL_SPI_TransmitReceive_DMA().
 *   - RX: SCB_InvalidateDCache_by_Addr() prima che la CPU legga il buffer
 *     riempito dal DMA.
 *
 * SCB_*Cache_by_Addr() operano per linee cache intere (32 byte su
 * Cortex-M7): indirizzo e dimensione devono essere multipli di 32, altrimenti
 * l'operazione si estende a variabili adiacenti. COM_FRAME_SIZE (fisso a 256,
 * vedi COM_interface_protocol.h — allineato alla COM interface) e' gia'
 * multiplo di 32, ma COM_DMA_BUF_SIZE resta calcolato in automatico dal
 * COM_FRAME_SIZE corrente per sicurezza, nessun numero da toccare a mano qui.
 */
#define COM_DMA_BUF_SIZE   (((COM_FRAME_SIZE + 31U) / 32U) * 32U)

static uint8_t s_tx_buf[COM_DMA_BUF_SIZE] __attribute__((aligned(32)));
static uint8_t s_rx_buf[COM_DMA_BUF_SIZE] __attribute__((aligned(32)));

/* ========================================================================== */
/* --- CONFIGURAZIONE SPI3 DEDICATA COM INTERFACE --- */
/* ========================================================================== */

/*
 * Prescaler SPI3 per la COM interface. NOTA IMPORTANTE: com_spi_reconfigure()
 * (sotto) riscrive esplicitamente Init.BaudRatePrescaler da QUESTA macro ad
 * ogni transazione COM, per difendersi dal fatto che sd_spi.c altera lo
 * stesso hspi3 condiviso — quindi il valore di SPI3.BaudRatePrescaler nel
 * .ioc/MX_SPI3_Init() (main.c) NON ha alcun effetto sul link COM interface:
 * cambiare il prescaler in CubeMX senza aggiornare QUESTA macro non cambia
 * la velocita' reale del bus verso la COM interface.
 *
 * 2026-07-23: allineato a SPI_BAUDRATEPRESCALER_64 (~2.86MHz con kernel
 * clock SPI3 = PLL1Q ~183.33MHz), per margine di robustezza sul cavo flat
 * da 10cm verso la COM interface (vedi discussione in chat: a /2, ~91.7MHz,
 * il link e' considerato troppo aggressivo per un cavo non controllato in
 * impedenza). Se in futuro si valida un valore piu' alto via oscilloscopio,
 * aggiornare QUESTA macro (il .ioc da solo non basta).
 */
#define COM_SPI_PRESCALER   SPI_BAUDRATEPRESCALER_64

/* ========================================================================== */
/* --- STATO INTERNO --- */
/* ========================================================================== */

static SPI_HandleTypeDef *s_hspi3           = NULL;
static void (*s_on_msg_pending)(void)       = NULL;
static void (*s_on_xfer_cplt)(void)         = NULL;

/* true tra COM_Interface_MsgReady_EXTI_Callback() e COM_Interface_EndExchange()/
 * COM_Interface_AbortAndEnd(): un nuovo fronte su nCOM_INT_IN viene ignorato
 * finche' resta true (vedi banner "SEQUENZA DI SINCRONISMO" in COM_interface.h). */
static volatile bool s_busy            = false;

/* Esito dell'ultima transazione DMA, impostato dalle callback SPI ISR. */
static volatile bool s_last_xfer_ok    = false;

/* Impostato dall'ISR su nCOM_PWR_FLT (rising): fault immediato, letto e
 * consumato dal prossimo COM_Interface_Update(). */
static volatile bool s_pwr_fault_latched = false;

/* "Gia' segnalato" — stesso pattern lq_fault_posted/amc_fault_posted di
 * task_comms.c: evita di floodare la coda eventi con post ripetuti finche'
 * la condizione di fault resta presente. */
static bool s_fault_posted = false;

/* ========================================================================== */
/* --- HELPER: CS SOFTWARE --- */
/* ========================================================================== */

static inline void cs_low(void)
{
    HAL_GPIO_WritePin(COM_SPI_nCS_GPIO_Port, COM_SPI_nCS_Pin, GPIO_PIN_RESET);
}

static inline void cs_high(void)
{
    HAL_GPIO_WritePin(COM_SPI_nCS_GPIO_Port, COM_SPI_nCS_Pin, GPIO_PIN_SET);
}

/*
 * Riconfigura esplicitamente hspi3 con i parametri richiesti dalla COM
 * interface. Necessario perche' sd_spi.c (driver SD, stesso hspi3 condiviso)
 * altera a runtime DataSize/NSSPMode/prescaler ad ogni transazione — senza
 * questa riconfigurazione difensiva, uno scambio COM interface che segue
 * un accesso SD (log su errore, vedi sys_log.c) troverebbe il periferico
 * nello stato lasciato dalla SD invece che in quello atteso qui.
 */
static void com_spi_reconfigure(void)
{
    __HAL_SPI_DISABLE(s_hspi3);

    s_hspi3->Init.DataSize          = SPI_DATASIZE_8BIT;
    s_hspi3->Init.CLKPolarity       = SPI_POLARITY_LOW;
    s_hspi3->Init.CLKPhase          = SPI_PHASE_1EDGE;
    s_hspi3->Init.NSS               = SPI_NSS_SOFT;
    s_hspi3->Init.NSSPMode          = SPI_NSS_PULSE_DISABLE;
    s_hspi3->Init.BaudRatePrescaler = COM_SPI_PRESCALER;
    s_hspi3->Init.FifoThreshold     = SPI_FIFO_THRESHOLD_01DATA;

    HAL_SPI_Init(s_hspi3);
}

/* ========================================================================== */
/* --- API INIZIALIZZAZIONE --- */
/* ========================================================================== */

void COM_Interface_Init(SPI_HandleTypeDef *hspi3,
                         void (*on_msg_pending)(void),
                         void (*on_xfer_cplt)(void))
{
    s_hspi3          = hspi3;
    s_on_msg_pending = on_msg_pending;
    s_on_xfer_cplt   = on_xfer_cplt;

    s_busy              = false;
    s_last_xfer_ok       = false;
    s_pwr_fault_latched = false;
    s_fault_posted      = false;

    cs_high(); /* stato di riposo: CS deasserted */

    TaskMonitor_SetFaultBit(FAULT_BIT_COM, false);
}

/* ========================================================================== */
/* --- CALLBACK ISR --- */
/* ========================================================================== */

void COM_Interface_MsgReady_EXTI_Callback(void)
{
    /* Contesto ISR: nessuna chiamata bloccante. Se e' gia' in corso uno
     * scambio, il fronte va perso di proposito — vedi banner in
     * COM_interface.h ("SEQUENZA DI SINCRONISMO", punto 2). */
    if (s_busy) {
        return;
    }
    s_busy = true;

    if (s_on_msg_pending != NULL) {
        s_on_msg_pending();
    }
}

void COM_Interface_PwrFlt_EXTI_Callback(void)
{
    /* Contesto ISR: latch del fault, processato da COM_Interface_Update().
     * nCOM_PWR_FLT e' attivo alto (vedi main.c, GPIO_MODE_IT_RISING). */
    s_pwr_fault_latched = true;
}

void COM_Interface_SPI_Cplt_Callback(void)
{
    s_last_xfer_ok = true;
    if (s_on_xfer_cplt != NULL) {
        s_on_xfer_cplt();
    }
}

void COM_Interface_SPI_Error_Callback(void)
{
    s_last_xfer_ok = false;
    if (s_on_xfer_cplt != NULL) {
        s_on_xfer_cplt();
    }
}

/* ========================================================================== */
/* --- SCAMBIO DATI --- */
/* ========================================================================== */

bool COM_Interface_BeginRequest(void)
{
    if (!SPI3Bus_Acquire(osWaitForever)) {
        /* Non dovrebbe mai accadere con osWaitForever, ma per robustezza:
         * nessuna transazione avviata, nessun mutex da rilasciare. */
        s_busy = false;
        return false;
    }

    com_spi_reconfigure();

    /* Frame "vuoto": il contenuto non ha significato applicativo, serve
     * solo a generare il clock SPI (vedi banner in COM_interface.h). */
    memset(s_tx_buf, 0x00, sizeof(s_tx_buf));
    s_tx_buf[COM_OFF_START]    = COM_START_BYTE;
    s_tx_buf[COM_OFF_MSG_TYPE] = (uint8_t)COM_MSG_EMPTY;
    COM_FrameFinalize(s_tx_buf);

    /* CPU ha scritto s_tx_buf: flush verso la RAM fisica prima che il DMA
     * lo legga. */
    SCB_CleanDCache_by_Addr((uint32_t *)s_tx_buf, (int32_t)sizeof(s_tx_buf));
    /* s_rx_buf verra' invalidato in COM_Interface_GetRequestFrame(), dopo il
     * completamento DMA: nessuna manutenzione cache necessaria qui. */

    cs_low();

    if (HAL_SPI_TransmitReceive_DMA(s_hspi3,s_tx_buf, s_rx_buf,COM_FRAME_SIZE) != HAL_OK) {
        cs_high();
        SPI3Bus_Release();
        s_busy = false;
        return false;
    }

    return true;
}

const uint8_t *COM_Interface_GetRequestFrame(void)
{
    cs_high();

    if (!s_last_xfer_ok) {
        return NULL;
    }

    /* Il DMA ha scritto s_rx_buf in RAM bypassando la D-Cache: invalida
     * prima che la CPU la legga. */
    SCB_InvalidateDCache_by_Addr((uint32_t *)s_rx_buf, (int32_t)sizeof(s_rx_buf));

    if (!COM_FrameValidate(s_rx_buf)) {
        return NULL;
    }

    return s_rx_buf;
}

bool COM_Interface_SendResponse(COM_MsgType_t msg_type, const uint8_t *payload)
{
    memset(s_tx_buf, 0x00, sizeof(s_tx_buf));
    s_tx_buf[COM_OFF_START]    = COM_START_BYTE;
    s_tx_buf[COM_OFF_MSG_TYPE] = (uint8_t)msg_type;
    if (payload != NULL) {
        memcpy(&s_tx_buf[COM_OFF_PAYLOAD], payload, COM_PAYLOAD_SIZE);
    }
    COM_FrameFinalize(s_tx_buf);

    SCB_CleanDCache_by_Addr((uint32_t *)s_tx_buf, (int32_t)sizeof(s_tx_buf));

    cs_low();

    /* RX di questa transazione scartata (nessun uso applicativo oggi): resta
     * comunque su s_rx_buf, sovrascritto dalla prossima BeginRequest(). */
    if (HAL_SPI_TransmitReceive_DMA(s_hspi3, s_tx_buf, s_rx_buf, COM_FRAME_SIZE) != HAL_OK) {
        cs_high();
        return false;
    }

    return true;
}

void COM_Interface_EndExchange(void)
{
    cs_high();
    SPI3Bus_Release();
    s_busy = false;
}

void COM_Interface_AbortAndEnd(void)
{
    (void)HAL_SPI_Abort(s_hspi3);
    COM_Interface_EndExchange();
}

bool COM_Interface_LastXferOk(void)
{
    return s_last_xfer_ok;
}

/* ========================================================================== */
/* --- SORVEGLIANZA ALIMENTAZIONE --- */
/* ========================================================================== */

void COM_Interface_Update(void)
{
    /* Livello corrente di nCOM_PWR_FLT: attivo alto. Letto anche in
     * polling (non solo tramite il latch da EXTI) per coprire il caso in
     * cui il fault fosse gia' presente prima che COM_Interface_Init() venga
     * chiamata (nessun fronte di salita da rilevare in quel caso). */
    bool pwr_fault_now = (HAL_GPIO_ReadPin(nCOM_PWR_FLT_GPIO_Port, nCOM_PWR_FLT_Pin) == GPIO_PIN_SET)
                          || s_pwr_fault_latched;

    /*
     * SOSPESO TEMPORANEAMENTE (2026-07-31, richiesta esplicita): il bit
     * FAULT_BIT_COM veniva accumulato in s_active_faults (visibile in
     * "GET FAULTS") ad ogni ciclo, ma senza produrre mai un vero passaggio
     * a SYS_FAULT (l'escalation qui sotto resta comunque gated da
     * g_config.fault_mask, indipendente da questa riga). Risultato:
     * un flag che compariva accumulato senza alcun effetto reale — per ora
     * non riportarlo affatto in s_active_faults. Il resto della logica
     * (rilevamento pwr_fault_now, eventuale posting di SYS_COM_FAULT_EVENT
     * sotto) resta invariato e pronto: per riattivare l'accumulo, basta
     * de-commentare la riga sottostante.
     */
    // TaskMonitor_SetFaultBit(FAULT_BIT_COM, pwr_fault_now);

    if (pwr_fault_now) {
        /* FAULT_BIT_COM (g_config.fault_mask): categoria fault comunicazione,
         * separata da warning_mask/error_mask — vedi task_monitor.h. */
        if (!s_fault_posted && (g_config.fault_mask & FAULT_BIT_COM)) {
            Queue_PostEvent(SYS_COM_FAULT_EVENT);
            s_fault_posted = true;
        }
    } else {
        s_fault_posted      = false;
        s_pwr_fault_latched = false;
    }
}

bool COM_Interface_IsFaultActive(void)
{
    return s_pwr_fault_latched
        || (HAL_GPIO_ReadPin(nCOM_PWR_FLT_GPIO_Port, nCOM_PWR_FLT_Pin) == GPIO_PIN_SET);
}
