/*
 * AMC.c
 *
 * Driver comunicazione MMC → AMC (Analog Module Controller).
 * Vedere AMC.h per la documentazione dell'API.
 *
 * Comunicazione asincrona (IT + semaforo): il task chiamante (TaskComms)
 * si sospende sul semaforo invece di restare bloccato dentro l'HAL, cedendo
 * la CPU allo scheduler durante l'attesa. Timeout (AMC_RESPONSE_TIMEOUT_MS)
 * garantisce che il task non resti sospeso a tempo indeterminato in caso di
 * AMC assente. Vedi nota "COMUNICAZIONE NON BLOCCANTE" in AMC.h.
 */

#include "AMC.h"
#include "cmsis_os.h"
#include <string.h>

/* ============================================================================
 * STATO INTERNO
 * ============================================================================ */

static UART_HandleTypeDef *s_huart   = NULL;
static bool                s_enabled = false;
static uint16_t            s_seq     = 0;
static AMC_PayloadStatus_t s_last_status;
static bool                s_alive   = false;
/* Versione FW di AMC letta dall'ultimo CONFIG_ACK ricevuto (NUOVO
 * 2026-09-04, AMC_protocol.h v0x0006) - vedi AMC_GetFwVersionStr(). Stringa
 * vuota se AMC non ha ancora risposto a un CONFIG_SET, o se sta girando un
 * firmware piu' vecchio che non popola ancora questo campo (reserved a
 * zero lato AMC = stringa vuota qui, trattata come "sconosciuta"). */
static char s_fw_version[10] = {0};

/* Buffer fisso per TX/RX */
static uint8_t s_tx_buf[AMC_FRAME_SIZE];
static uint8_t s_rx_buf[AMC_FRAME_SIZE];

/* Semafori binari di sincronizzazione IT (segnalati da AMC_ITTxCallback()/
 * AMC_ITRxCallback(), dispatch in stm32h7xx_it.c) */
static osSemaphoreId_t s_sem_tx = NULL;
static osSemaphoreId_t s_sem_rx = NULL;

/* ============================================================================
 * INIT
 * ============================================================================ */

void AMC_Init(UART_HandleTypeDef *huart)
{
    s_huart   = huart;
    s_enabled = (huart != NULL);
    s_seq     = 0;
    s_alive   = false;
    memset(&s_last_status, 0, sizeof(s_last_status));

    /* Creati una sola volta: AMC_Init() puo' essere richiamato più volte
     * (es. reinizializzazione), ma un oggetto CMSIS-RTOS2 va creato una
     * sola volta. Chiamato da MX_FREERTOS_Init() prima dell'avvio dello
     * scheduler: osSemaphoreNew() è ammesso anche pre-kernel-start (stesso
     * schema già usato per il mutex flash in Config_CreateFlashMutex()). */
    if (s_sem_tx == NULL) {
        s_sem_tx = osSemaphoreNew(1, 0, NULL);
    }
    if (s_sem_rx == NULL) {
        s_sem_rx = osSemaphoreNew(1, 0, NULL);
    }
}

/* ============================================================================
 * TRANSAZIONE
 * ============================================================================ */

/**
 * @brief  Trasmette s_tx_buf e riceve s_rx_buf in modalità IT, sincronizzato
 *         tramite s_sem_tx/s_sem_rx. Non bloccante per la CPU: il chiamante
 *         cede lo scheduler durante l'attesa (osSemaphoreAcquire).
 * @param  timeout_ms  Timeout per ciascuna delle due attese (TX e RX).
 */
static AMC_err_t amc_transceive(uint32_t timeout_ms)
{
    /*
     * Drain difensivo: se la transazione precedente è finita in errore a
     * metà (vedi AMC_ITErrorCallback), può restare un token "orfano" su
     * uno dei due semafori (rilasciato ma mai consumato). Senza questo
     * drain, il prossimo osSemaphoreAcquire lo consumerebbe subito senza
     * attendere il vero completamento del nuovo trasferimento IT,
     * disallineando permanentemente TX/RX. Acquire con timeout 0 = non
     * bloccante: se non c'è alcun token pendente ritorna subito.
     */
    (void)osSemaphoreAcquire(s_sem_tx, 0U);
    (void)osSemaphoreAcquire(s_sem_rx, 0U);

    if (HAL_UART_Transmit_IT(s_huart, s_tx_buf, AMC_FRAME_SIZE) != HAL_OK) {
        return AMC_ERR_UART;
    }
    if (osSemaphoreAcquire(s_sem_tx, timeout_ms) != osOK) {
        return AMC_ERR_TIMEOUT;
    }

    memset(s_rx_buf, 0, AMC_FRAME_SIZE);
    if (HAL_UART_Receive_IT(s_huart, s_rx_buf, AMC_FRAME_SIZE) != HAL_OK) {
        return AMC_ERR_UART;
    }
    if (osSemaphoreAcquire(s_sem_rx, timeout_ms) != osOK) {
        return AMC_ERR_TIMEOUT;
    }

    return AMC_OK;
}

static uint16_t s_current_setpoint_ma = 0U;

/*
 * Flag one-shot "riavvio a caldo" per la compensazione tensione PSU (NUOVO
 * 2026-07-27, protocollo v0.0005 — vedi AMC_protocol.h). volatile perche'
 * scritta da task_comms.c (via AMC_RequestVoltageRestart()) e consumata da
 * task_amc.c (via AMC_Transact()): due task diversi, stesso schema RAM-only
 * gia' in uso per s_vcomp_lut_resend_pending/s_psu_cfg_resend_pending in
 * task_amc.c — non serve un mutex per un singolo bool con un solo scrittore
 * e un solo lettore.
 */
static volatile bool s_voltage_restart_request = false;

void AMC_RequestVoltageRestart(void)
{
    s_voltage_restart_request = true;
}

AMC_err_t AMC_Transact(uint8_t laser_mode, uint8_t hw_setpoint_sel, uint8_t qcw_active,
                       uint8_t setpoint_hw_enabled)
{
    if (!s_enabled || s_huart == NULL) {
        return AMC_ERR_DISABLED;
    }

    /* --- Build HEARTBEAT frame --- */
    memset(s_tx_buf, 0x00U, AMC_FRAME_SIZE);
    s_tx_buf[AMC_OFF_START]    = AMC_START_BYTE;
    s_tx_buf[AMC_OFF_MSG_TYPE] = (uint8_t)AMC_MSG_HEARTBEAT;

    /* One-shot: consumato qui (letto e subito azzerato) indipendentemente
     * dall'esito della transazione sotto — se il frame va perso per un
     * errore di comunicazione, si perde anche la richiesta di restart, ma
     * non blocchiamo il chiamante ne' teniamo lo stato in sospeso: stesso
     * livello di garanzia "best effort" gia' accettato per s_current_setpoint_ma
     * (nessun ACK previsto per l'HEARTBEAT). */
    bool voltage_restart = s_voltage_restart_request;
    s_voltage_restart_request = false;

    AMC_PayloadHeartbeat_t *hb =
        (AMC_PayloadHeartbeat_t *)&s_tx_buf[AMC_OFF_PAYLOAD];
    hb->seq                      = s_seq++;
    hb->laser_mode                = laser_mode;
    hb->hw_setpoint_sel           = hw_setpoint_sel;
    hb->protocol_version          = AMC_PROTOCOL_VERSION;
    hb->current_setpoint_ma       = s_current_setpoint_ma;  /* 0 in HW/ANALOG mode */
    hb->qcw_active                = qcw_active;              /* NUOVO 2026-07-22 */
    hb->setpoint_hw_enabled       = setpoint_hw_enabled;      /* NUOVO 2026-07-22 */
    hb->voltage_restart_request   = voltage_restart ? 1U : 0U; /* NUOVO 2026-07-27 */

    AMC_FrameFinalize(s_tx_buf);

    /* --- Trasmissione + ricezione (IT, non bloccante per la CPU) --- */
    AMC_err_t xfer_err = amc_transceive(AMC_RESPONSE_TIMEOUT_MS);
    if (xfer_err != AMC_OK) {
        s_alive = false;
        return xfer_err;
    }

    /* --- Validazione --- */
    if (!AMC_FrameValidate(s_rx_buf) ||
        s_rx_buf[AMC_OFF_MSG_TYPE] != (uint8_t)AMC_MSG_STATUS) {
        s_alive = false;
        return AMC_ERR_INVALID_RSP;
    }

    /* --- Deserializzazione --- */
    const AMC_PayloadStatus_t *st =
        (const AMC_PayloadStatus_t *)&s_rx_buf[AMC_OFF_PAYLOAD];
    memcpy(&s_last_status, st, sizeof(AMC_PayloadStatus_t));
    /* Conversione LUT (pd_raw → W) avviene nel layer applicativo (task_amc, dal 2026-07-22) */

    s_alive = true;
    return AMC_OK;
}

/* ============================================================================
 * LETTURA STATO
 * ============================================================================ */

AMC_PayloadStatus_t AMC_GetLastStatus(void)
{
    return s_last_status;
}

bool AMC_IsAlive(void)
{
    return s_alive;
}

/* ============================================================================
 * VERSIONE FW AMC (NUOVO 2026-09-04)
 * ============================================================================ */

const char *AMC_GetFwVersionStr(void)
{
    return s_fw_version;
}

/* ============================================================================
 * INVIO CONFIGURAZIONE PSU (una tantum)
 * ============================================================================ */

AMC_err_t AMC_SendConfig(uint16_t psu_voltage_limit_mv, uint16_t psu_current_limit_ma,
                         uint8_t voltage_comp_enabled, uint8_t comp_stabilize_delay_ms,
                         uint8_t comp_ramp_duration_ms, uint8_t comp_ramp_up_duration_ms)
{
    if (!s_enabled || s_huart == NULL) {
        return AMC_ERR_DISABLED;
    }

    /* --- Build CONFIG_SET frame --- */
    memset(s_tx_buf, 0x00U, AMC_FRAME_SIZE);
    s_tx_buf[AMC_OFF_START]    = AMC_START_BYTE;
    s_tx_buf[AMC_OFF_MSG_TYPE] = (uint8_t)AMC_MSG_CONFIG_SET;

    AMC_PayloadConfigSet_t *cfg =
        (AMC_PayloadConfigSet_t *)&s_tx_buf[AMC_OFF_PAYLOAD];
    cfg->psu_voltage_limit_mv      = psu_voltage_limit_mv;
    cfg->psu_current_limit_ma      = psu_current_limit_ma;
    cfg->protocol_version          = AMC_PROTOCOL_VERSION;
    cfg->voltage_comp_enabled      = voltage_comp_enabled;       /* NUOVO 2026-07-22 */
    cfg->comp_stabilize_delay_ms   = comp_stabilize_delay_ms;    /* NUOVO 2026-07-22 */
    cfg->comp_ramp_duration_ms     = comp_ramp_duration_ms;      /* NUOVO 2026-07-22 */
    cfg->comp_ramp_up_duration_ms  = comp_ramp_up_duration_ms;   /* NUOVO 2026-07-22 */

    AMC_FrameFinalize(s_tx_buf);

    /* --- TX + RX CONFIG_ACK (IT, non bloccante per la CPU) --- */
    AMC_err_t xfer_err = amc_transceive(AMC_RESPONSE_TIMEOUT_MS);
    if (xfer_err != AMC_OK) {
        return xfer_err;
    }

    if (!AMC_FrameValidate(s_rx_buf) ||
        s_rx_buf[AMC_OFF_MSG_TYPE] != (uint8_t)AMC_MSG_CONFIG_ACK) {
        return AMC_ERR_INVALID_RSP;
    }

    const AMC_PayloadConfigAck_t *ack =
        (const AMC_PayloadConfigAck_t *)&s_rx_buf[AMC_OFF_PAYLOAD];

    /* fw_version (NUOVO 2026-09-04, AMC_protocol.h v0x0006): copiato a
     * prescindere da accepted, AMC lo popola su OGNI CONFIG_ACK (vedi
     * handle_config_set() lato AMC). memcpy + azzeramento esplicito
     * dell'ultimo byte invece di fidarsi ciecamente del NUL ricevuto via
     * RS485 (difesa in profondita' contro un campo non NUL-terminated,
     * es. un AMC con firmware non ancora aggiornato che lascia il vecchio
     * "reserved" a un valore imprevisto). */
    memcpy(s_fw_version, ack->fw_version, sizeof(s_fw_version));
    s_fw_version[sizeof(s_fw_version) - 1U] = '\0';

    return (ack->accepted != 0U) ? AMC_OK : AMC_ERR_INVALID_RSP;
}

/* ============================================================================
 * INVIO CONFIGURAZIONE FOTODIODI
 * ============================================================================ */

AMC_err_t AMC_SendPDConfig(uint8_t pd_mask, uint8_t gain_windows,
                            uint16_t gain_settle_ms,
                            uint8_t stability_samples,
                            uint8_t stability_thresh)
{
    if (!s_enabled || s_huart == NULL) return AMC_ERR_DISABLED;

    memset(s_tx_buf, 0x00U, AMC_FRAME_SIZE);
    s_tx_buf[AMC_OFF_START]    = AMC_START_BYTE;
    s_tx_buf[AMC_OFF_MSG_TYPE] = (uint8_t)AMC_MSG_CONFIG_PD;

    AMC_PayloadConfigPD_t *pd = (AMC_PayloadConfigPD_t *)&s_tx_buf[AMC_OFF_PAYLOAD];
    pd->pd_mask             = pd_mask;
    pd->gain_windows        = gain_windows;
    pd->gain_settle_ms      = gain_settle_ms;
    pd->stability_samples   = stability_samples;
    pd->stability_threshold = stability_thresh;

    AMC_FrameFinalize(s_tx_buf);

    {
        AMC_err_t xfer_err = amc_transceive(AMC_RESPONSE_TIMEOUT_MS);
        if (xfer_err != AMC_OK) return xfer_err;
    }
    if (!AMC_FrameValidate(s_rx_buf) ||
        s_rx_buf[AMC_OFF_MSG_TYPE] != (uint8_t)AMC_MSG_CONFIG_PD_ACK)
        return AMC_ERR_INVALID_RSP;

    const AMC_PayloadConfigPDAck_t *ack =
        (const AMC_PayloadConfigPDAck_t *)&s_rx_buf[AMC_OFF_PAYLOAD];
    return (ack->accepted != 0U) ? AMC_OK : AMC_ERR_INVALID_RSP;
}


/* ============================================================================
 * LETTURA FAULT PIN (PE10, polling)
 * ============================================================================ */

bool AMC_ReadFaultPin(void)
{
#if AMC_FAULT_GPIO_READY
    return (HAL_GPIO_ReadPin(AMC_FAULT_N_GPIO_Port, AMC_FAULT_N_Pin) == GPIO_PIN_RESET);
#else
    /* Pin non ancora configurato in CubeMX: restituisce false (nessun fault) */
    return false;
#endif
}

/* ============================================================================
 * API v0.0003 — LUT GAIN
 * ============================================================================ */

/* Setpoint SW corrente (aggiornato da AMC_SetCurrentSetpoint, usato in Transact) */

void AMC_SetCurrentSetpoint(uint16_t current_ma)
{
    s_current_setpoint_ma = current_ma;
}

AMC_err_t AMC_SendGainLUT(uint8_t mode, const uint16_t threshold[4])
{
    if (!s_enabled || s_huart == NULL) return AMC_ERR_DISABLED;

    memset(s_tx_buf, 0x00U, AMC_FRAME_SIZE);
    s_tx_buf[AMC_OFF_START]    = AMC_START_BYTE;
    s_tx_buf[AMC_OFF_MSG_TYPE] = (uint8_t)AMC_MSG_CONFIG_GAIN_LUT;

    AMC_PayloadConfigGainLUT_t *p =
        (AMC_PayloadConfigGainLUT_t *)&s_tx_buf[AMC_OFF_PAYLOAD];
    p->mode         = mode;
    p->num_windows  = 4U;
    p->threshold[0] = threshold[0];
    p->threshold[1] = threshold[1];
    p->threshold[2] = threshold[2];
    p->threshold[3] = threshold[3];

    AMC_FrameFinalize(s_tx_buf);

    {
        AMC_err_t xfer_err = amc_transceive(AMC_RESPONSE_TIMEOUT_MS);
        if (xfer_err != AMC_OK) return xfer_err;
    }
    if (!AMC_FrameValidate(s_rx_buf) ||
        s_rx_buf[AMC_OFF_MSG_TYPE] != (uint8_t)AMC_MSG_CONFIG_GAIN_LUT_ACK)
        return AMC_ERR_INVALID_RSP;

    const AMC_PayloadConfigGainLUTAck_t *ack =
        (const AMC_PayloadConfigGainLUTAck_t *)&s_rx_buf[AMC_OFF_PAYLOAD];
    return (ack->accepted != 0U) ? AMC_OK : AMC_ERR_INVALID_RSP;
}

AMC_err_t AMC_SendPDValidEntry(uint8_t pd_idx, uint8_t mode,
                               uint8_t entry_idx, uint8_t total,
                               uint16_t setpoint, uint16_t pd_min, uint16_t pd_max)
{
    if (!s_enabled || s_huart == NULL) return AMC_ERR_DISABLED;

    memset(s_tx_buf, 0x00U, AMC_FRAME_SIZE);
    s_tx_buf[AMC_OFF_START]    = AMC_START_BYTE;
    s_tx_buf[AMC_OFF_MSG_TYPE] = (uint8_t)AMC_MSG_CONFIG_PD_VALID;

    AMC_PayloadConfigPDValid_t *p =
        (AMC_PayloadConfigPDValid_t *)&s_tx_buf[AMC_OFF_PAYLOAD];
    p->pd_idx        = pd_idx;
    p->mode          = mode;
    p->entry_idx     = entry_idx;
    p->total_entries = total;
    p->setpoint      = setpoint;
    p->pd_min        = pd_min;
    p->pd_max        = pd_max;

    AMC_FrameFinalize(s_tx_buf);

    {
        AMC_err_t xfer_err = amc_transceive(AMC_RESPONSE_TIMEOUT_MS);
        if (xfer_err != AMC_OK) return xfer_err;
    }
    if (!AMC_FrameValidate(s_rx_buf) ||
        s_rx_buf[AMC_OFF_MSG_TYPE] != (uint8_t)AMC_MSG_CONFIG_PD_VALID_ACK)
        return AMC_ERR_INVALID_RSP;

    const AMC_PayloadConfigPDValidAck_t *ack =
        (const AMC_PayloadConfigPDValidAck_t *)&s_rx_buf[AMC_OFF_PAYLOAD];
    return (ack->accepted != 0U) ? AMC_OK : AMC_ERR_INVALID_RSP;
}

/* ============================================================================
 * API v0.0004 — LUT COMPENSAZIONE TENSIONE PSU (NUOVO 2026-07-22)
 * ============================================================================ */

AMC_err_t AMC_SendVoltageCompLUTEntry(uint8_t entry_idx, uint8_t total,
                                      uint16_t current_ma, uint16_t voltage_mv)
{
    if (!s_enabled || s_huart == NULL) return AMC_ERR_DISABLED;

    memset(s_tx_buf, 0x00U, AMC_FRAME_SIZE);
    s_tx_buf[AMC_OFF_START]    = AMC_START_BYTE;
    s_tx_buf[AMC_OFF_MSG_TYPE] = (uint8_t)AMC_MSG_CONFIG_VOLTAGE_LUT;

    AMC_PayloadConfigVoltageLUT_t *p =
        (AMC_PayloadConfigVoltageLUT_t *)&s_tx_buf[AMC_OFF_PAYLOAD];
    p->entry_idx     = entry_idx;
    p->total_entries = total;
    p->current_ma    = current_ma;
    p->voltage_mv    = voltage_mv;

    AMC_FrameFinalize(s_tx_buf);

    {
        AMC_err_t xfer_err = amc_transceive(AMC_RESPONSE_TIMEOUT_MS);
        if (xfer_err != AMC_OK) return xfer_err;
    }
    if (!AMC_FrameValidate(s_rx_buf) ||
        s_rx_buf[AMC_OFF_MSG_TYPE] != (uint8_t)AMC_MSG_CONFIG_VOLTAGE_LUT_ACK)
        return AMC_ERR_INVALID_RSP;

    const AMC_PayloadConfigVoltageLUTAck_t *ack =
        (const AMC_PayloadConfigVoltageLUTAck_t *)&s_rx_buf[AMC_OFF_PAYLOAD];
    return (ack->accepted != 0U) ? AMC_OK : AMC_ERR_INVALID_RSP;
}

/* ============================================================================
 * ACCESSO DATI GREZZI FOTODIODI
 * La conversione ADC→W (LUT) avviene nel layer applicativo (task_comms.c).
 * ============================================================================ */

void AMC_GetPDRaw(uint16_t out_raw[4])
{
    for (uint8_t p = 0U; p < 4U; p++) {
        out_raw[p] = s_last_status.pd_raw[p];
    }
}

uint8_t AMC_GetGainWindow(void)
{
    return s_last_status.gain_window;
}

bool AMC_IsPDDataStale(void)
{
    /* pd_status_mask bit6 = pd_stale (NUOVO 2026-07-28, vedi AMC.h/AMC_protocol.h) */
    return s_alive && ((s_last_status.pd_status_mask & 0x40U) != 0U);
}

/* ============================================================================
 * CALLBACK ISR (dispatch da stm32h7xx_it.c per huart6)
 * ============================================================================ */

void AMC_ITTxCallback(UART_HandleTypeDef *huart)
{
    if (huart != s_huart) {
        return;
    }
    if (s_sem_tx != NULL) {
        osSemaphoreRelease(s_sem_tx);
    }
}

void AMC_ITRxCallback(UART_HandleTypeDef *huart)
{
    if (huart != s_huart) {
        return;
    }
    if (s_sem_rx != NULL) {
        osSemaphoreRelease(s_sem_rx);
    }
}

void AMC_ITErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart != s_huart) {
        return;
    }
    /*
     * ORE/FE/NE/PE: l'HAL abortisce già il transfer in corso (UART_IRQHandler
     * chiama UART_EndTxTransfer()/UART_EndRxTransfer() prima di invocare
     * questa callback) prima di rimettere huart in stato Ready. Rilasciamo
     * entrambi i semafori per sbloccare subito l'eventuale attesa pendente
     * in amc_transceive(), invece di aspettare il timeout pieno.
     * Quello dei due NON atteso in quel momento resta come token orfano
     * (rilasciato ma non consumato) — viene ripulito dal drain difensivo
     * a inizio di amc_transceive() alla transazione successiva, altrimenti
     * disallineerebbe silenziosamente le attese TX/RX del ciclo dopo.
     */
    if (s_sem_tx != NULL) {
        osSemaphoreRelease(s_sem_tx);
    }
    if (s_sem_rx != NULL) {
        osSemaphoreRelease(s_sem_rx);
    }
}
