/*
 * LaseQ.c
 *
 * Driver RS485 master STM32H723 per comunicazione con slave LaseQ4 (STM32G484).
 * Vedere LaseQ.h per la documentazione dell'API pubblica.
 *
 * Il funzionamento concettuale è identico alla versione originale:
 *   - BuildTxBuffer() serializza le variabili di controllo nel buffer raw
 *   - LaseQ_Transmit() alza DE, avvia DMA TX
 *   - TxCpltCallback() abbassa DE, notifica task tramite callback
 *   - LaseQ_Receive() avvia DMA RX
 *   - RxCpltCallback() notifica task tramite callback
 *   - LaseQ_ParseResponse() valida frame e deserializza payload
 *
 * Rispetto alla versione originale:
 *   - Frame fisso LASEQ_FRAME_SIZE (43 byte) con header START/SENDER/RECEIVER/TYPE
 *   - CRC16 CCITT su tutto il frame (esclusi CRC e STOP)
 *   - Supporto MSG_CONTROL, MSG_CONFIG_SET, MSG_PING
 *   - ParseResponse valida CRC, sender e MSG_TYPE atteso
 */

#include "LaseQ.h"
#include "main.h"
#include <string.h>

/* ============================================================================
 * MACRO GPIO RS485 (invariate rispetto all'originale)
 * ============================================================================ */
#define RS485_TX_EN()  HAL_GPIO_WritePin(LASE_Q_485_DIR_GPIO_Port, \
                                         LASE_Q_485_DIR_Pin, GPIO_PIN_SET)
#define RS485_RX_EN()  HAL_GPIO_WritePin(LASE_Q_485_DIR_GPIO_Port, \
                                         LASE_Q_485_DIR_Pin, GPIO_PIN_RESET)

/* ============================================================================
 * STATO INTERNO
 * ============================================================================ */
static LaseQ_control_vars_t s_ctrl  = {0};
static LaseQ_status_vars_t  s_status = {0};

static UART_HandleTypeDef *lq_huart      = NULL;
static void (*lq_on_tx_cplt)(void)       = NULL;
static void (*lq_on_rx_cplt)(void)       = NULL;
static uint8_t s_slave_addr              = 0x02U;

/*
 * Buffer DMA fissi: dimensione uguale per TX e RX (frame simmetrico).
 *
 * H7 D-Cache (abilitata in main.c dal 2026-07-02): il DMA legge/scrive la
 * RAM fisica bypassando la D-cache, quindi CPU e DMA possono vedere copie
 * disallineate dello stesso buffer se non si fa manutenzione cache
 * esplicita (stesso schema già usato in rs485_cmd.c per USART1):
 *   - TX: SCB_CleanDCache_by_Addr() dopo aver scritto il frame, PRIMA di
 *     HAL_UART_Transmit_DMA(), altrimenti il DMA legge dati stantii (il
 *     caso limite, con RAM .bss mai scritta, sono tutti zero — esattamente
 *     il sintomo osservato con la sonda logica).
 *   - RX: SCB_InvalidateDCache_by_Addr() prima che la CPU legga s_rx_buf in
 *     LaseQ_ParseResponse(), altrimenti si rischia di leggere una copia
 *     cache stantia invece del frame appena scritto dal DMA.
 *
 * SCB_*Cache_by_Addr() operano per linee cache intere (32 byte su
 * Cortex-M7): indirizzo e dimensione DEVONO essere multipli di 32 byte,
 * altrimenti l'operazione si estende a variabili adiacenti nella stessa
 * cache line (rischio concreto con Invalidate: scarta dati validi non
 * ancora flushati). LASEQ_FRAME_SIZE (43) non è multiplo di 32 → i buffer
 * sono dimensionati a LASEQ_DMA_BUF_SIZE (64, prossimo multiplo di 32) e
 * allineati con aligned(32). Il DMA usa comunque solo i primi
 * LASEQ_FRAME_SIZE byte (vedi HAL_UART_Transmit_DMA/Receive_DMA sotto);
 * il padding 43..63 non è mai letto né trasmesso.
 */
#define LASEQ_DMA_BUF_SIZE  64U

static uint8_t s_tx_buf[LASEQ_DMA_BUF_SIZE] __attribute__((aligned(32)));
static uint8_t s_rx_buf[LASEQ_DMA_BUF_SIZE] __attribute__((aligned(32)));

/* ============================================================================
 * FUNZIONI PRIVATE
 * ============================================================================ */

/**
 * @brief  Scrive header comune e avvia il buffer TX.
 *         Chiamata da tutte le Build* prima di riempire il payload.
 */
static void frame_init_tx(LaseQ_MsgType_t msg_type)
{
    memset(s_tx_buf, 0x00U, LASEQ_FRAME_SIZE);
    s_tx_buf[LASEQ_OFF_START]    = LASEQ_START_BYTE;
    s_tx_buf[LASEQ_OFF_SENDER]   = LASEQ_ADDR_MASTER;
    s_tx_buf[LASEQ_OFF_RECEIVER] = s_slave_addr;
    s_tx_buf[LASEQ_OFF_MSG_TYPE] = (uint8_t)msg_type;
}

/**
 * @brief  Serializza le variabili di controllo nel buffer TX (MSG_CONTROL).
 *         Mantiene lo stesso ordine di campo del BuildTxBuffer() originale.
 */
static void build_control_frame(void)
{
    frame_init_tx(LASEQ_MSG_CONTROL);

    LaseQ_PayloadControl_t *p =
        (LaseQ_PayloadControl_t *)&s_tx_buf[LASEQ_OFF_PAYLOAD];

    p->enable           = s_ctrl.enable;
    p->ch_enable        = s_ctrl.ch_enable;
    p->gate             = s_ctrl.gate;
    p->sw_control       = s_ctrl.sw_control;
    p->analog_mode      = s_ctrl.analog_mode;
    p->control_state    = s_ctrl.control_state;
    p->interlock_status = s_ctrl.interlock_status;
    p->clear_error      = s_ctrl.clear_error;
    p->OPM              = s_ctrl.OPM;
    p->OPD              = s_ctrl.OPD;

    p->sw_current_setpoint = s_ctrl.sw_current_setpoint;

    /* reserved già a zero per memset */

    LaseQ_FrameFinalize(s_tx_buf);
}

/**
 * @brief  Serializza un payload di configurazione nel buffer TX (MSG_CONFIG_SET).
 */
static void build_config_frame(const LaseQ_PayloadConfigSet_t *cfg)
{
    frame_init_tx(LASEQ_MSG_CONFIG_SET);
    memcpy(&s_tx_buf[LASEQ_OFF_PAYLOAD], cfg, sizeof(LaseQ_PayloadConfigSet_t));
    LaseQ_FrameFinalize(s_tx_buf);
}

/**
 * @brief  Serializza un frame MSG_PING.
 */
static void build_ping_frame(uint16_t seq)
{
    frame_init_tx(LASEQ_MSG_PING);

    LaseQ_PayloadPing_t *p =
        (LaseQ_PayloadPing_t *)&s_tx_buf[LASEQ_OFF_PAYLOAD];
    p->seq = seq;   /* Endianness non critica per il ping */

    LaseQ_FrameFinalize(s_tx_buf);
}

/* ============================================================================
 * INIT
 * ============================================================================ */
void LaseQ_Init(UART_HandleTypeDef *huart,
                uint8_t             slave_addr,
                void (*on_tx_cplt)(void),
                void (*on_rx_cplt)(void))
{
    lq_huart      = huart;
    s_slave_addr  = slave_addr;
    lq_on_tx_cplt = on_tx_cplt;
    lq_on_rx_cplt = on_rx_cplt;

    LaseQInterlockSet(1);
    HWEnableSet(0);
    HWGateSet(0);
    LaseQSupplySet(1);
    RS485_RX_EN();
}

/* ============================================================================
 * API CONTROLLO LASER (invariate rispetto all'originale)
 * ============================================================================ */
LaseQ_status_vars_t GetLaseQStatus(void)
{
    return s_status;
}

LaseQ_control_vars_t GetLaseQControl(void)
{
    return s_ctrl;
}

void EnableLaseQ(void)
{
    HWEnableSet(1);
    s_ctrl.enable = 1;

    if (s_ctrl.analog_mode) {
        HWEnableSet(1);
    }
}

void DisableLaseQ(void)
{
    if (s_ctrl.sw_control) {
        s_ctrl.enable = 0;
    }
    if (s_ctrl.analog_mode) {
        HWEnableSet(0);
    }
}

void LaseQGate(bool status)
{
    /*
     * FIX (2026-07-20): s_ctrl.gate (MSG_CONTROL.gate, RS485) resta SEMPRE
     * 0, indipendentemente da 'status'. Il gate reale in FSM_MODE_SW (CW e
     * QCW) è il pin fisico nGATE_MC — pilotato da BoardCtrl_GateMC_Open()/
     * Close() e da QCW_Start()/Stop()/QCW_TimerCallback() — collegato
     * DIRETTAMENTE al regolatore di LaseQ, non il comando software via
     * RS485. Vantaggio: allo spegnimento il gate è immediato (livello GPIO,
     * propagazione elettrica), non deve aspettare l'invio/elaborazione di
     * un frame MSG_CONTROL (fino a LQ_TRANSMIT_WINDOW = 20ms, vedi
     * task_comms.c) — critico anche per QCW, che pulsa il pin fino a 50kHz,
     * ben oltre quanto il ciclo RS485 potrebbe mai seguire.
     *
     * Lato LaseQ questo campo era già trattato come no-op per convenzione
     * documentale (vedi Lase-Q4_v2.1/App/Tasks/rs485_handler.c, banner
     * GATE: "MMC manda sempre p->gate == 0, quindi SetGate(p->gate) è di
     * fatto un no-op") — qui lo si rende esplicito e strutturale anche lato
     * MMC, non lasciato alla sola disciplina del chiamante.
     *
     * 'status' resta usato SOLO per il path hardware ANALOG/HYBRID
     * (HWGateSet() sotto, scrittura diretta sullo stesso pin nGATE_MC) —
     * invariato.
     */
    if (s_ctrl.sw_control) {
        s_ctrl.gate = 0U;
    }
    if (s_ctrl.analog_mode) {
        HWGateSet(status);
    }
}

void LaseQClearErr(void)
{
    s_ctrl.clear_error = 1;
}

void LaseQSetMode(LaseQ_mode_t mode)
{
    /* sw_control deve essere sempre 1, indipendentemente dal mode:
     * il protocollo RS485 verso LaseQ resta sempre attivo, è analog_mode
     * a determinare se il setpoint/gate sono guidati anche dall'ingresso
     * analogico (ANALOG, HYBRID) oppure no (SW). */
    s_ctrl.sw_control = 1;

    switch (mode) {
        case SW:
            s_ctrl.analog_mode = 0;
            break;
        case ANALOG:
        case HYBRID:
            s_ctrl.analog_mode = 1;
            break;
        default:
            break;
    }
}

void LaseQSetCurrent(uint32_t current_ma)
{
    s_ctrl.sw_current_setpoint = current_ma;
}

void LaseQSetChannels(uint8_t ch_mask)
{
    s_ctrl.ch_enable = ch_mask;
}

/* ============================================================================
 * TRANSAZIONE RS485
 * ============================================================================ */
LaseQ_err_t LaseQ_Transmit(void)
{
    build_control_frame();

    /* CPU ha scritto il frame in s_tx_buf (RAM cacheable): flush delle
     * linee dirty verso la RAM fisica prima che il DMA la legga (vedi
     * commento su LASEQ_DMA_BUF_SIZE sopra). */
    SCB_CleanDCache_by_Addr((uint32_t *)s_tx_buf, (int32_t)sizeof(s_tx_buf));

    RS485_TX_EN();

    if (HAL_UART_Transmit_DMA(lq_huart, s_tx_buf, LASEQ_FRAME_SIZE) != HAL_OK) {
        RS485_RX_EN();
        return LASEQ_ERR_UART;
    }
    return LASEQ_OK;
}

LaseQ_err_t LaseQ_TransmitConfig(const LaseQ_PayloadConfigSet_t *cfg)
{
    build_config_frame(cfg);

    SCB_CleanDCache_by_Addr((uint32_t *)s_tx_buf, (int32_t)sizeof(s_tx_buf));

    RS485_TX_EN();

    if (HAL_UART_Transmit_DMA(lq_huart, s_tx_buf, LASEQ_FRAME_SIZE) != HAL_OK) {
        RS485_RX_EN();
        return LASEQ_ERR_UART;
    }
    return LASEQ_OK;
}

LaseQ_err_t LaseQ_TransmitPing(uint16_t seq)
{
    build_ping_frame(seq);

    SCB_CleanDCache_by_Addr((uint32_t *)s_tx_buf, (int32_t)sizeof(s_tx_buf));

    RS485_TX_EN();

    if (HAL_UART_Transmit_DMA(lq_huart, s_tx_buf, LASEQ_FRAME_SIZE) != HAL_OK) {
        RS485_RX_EN();
        return LASEQ_ERR_UART;
    }
    return LASEQ_OK;
}

LaseQ_err_t LaseQ_Receive(void)
{
    if (HAL_UART_Receive_DMA(lq_huart, s_rx_buf, LASEQ_FRAME_SIZE) != HAL_OK) {
        return LASEQ_ERR_UART;
    }
    return LASEQ_OK;
}

LaseQ_err_t LaseQ_ParseResponse(LaseQ_MsgType_t expected_type)
{
    /*
     * Il DMA RX ha scritto s_rx_buf direttamente in RAM, bypassando la
     * D-cache: invalida le linee interessate prima che la CPU le legga,
     * altrimenti si rischia di leggere una copia cache stantia (es. il
     * frame del ciclo precedente) invece dei dati appena arrivati.
     */
    SCB_InvalidateDCache_by_Addr((uint32_t *)s_rx_buf, (int32_t)sizeof(s_rx_buf));

    /* --- Validazione frame (START, STOP, CRC) --- */
    if (!LaseQ_FrameValidate(s_rx_buf)) {
        return LASEQ_ERR_INVALID_RSP;
    }

    /* --- Verifica mittente --- */
    if (s_rx_buf[LASEQ_OFF_SENDER] != s_slave_addr) {
        return LASEQ_ERR_WRONG_SENDER;
    }

    /* --- Verifica tipo messaggio --- */
    if (s_rx_buf[LASEQ_OFF_MSG_TYPE] != (uint8_t)expected_type) {
        return LASEQ_ERR_INVALID_RSP;
    }

    /* --- Deserializzazione payload in base al tipo --- */
    switch (expected_type) {

        case LASEQ_MSG_STATUS: {
            const LaseQ_PayloadStatus_t *p =
                (const LaseQ_PayloadStatus_t *)&s_rx_buf[LASEQ_OFF_PAYLOAD];

            s_status.driver_address   = p->driver_address;
            s_status.error_code       = p->error_code;

            s_status.temperature[0]   = p->temperature[0];
            s_status.temperature[1]   = p->temperature[1];
            s_status.temperature[2]   = p->temperature[2];
            s_status.temperature[3]   = p->temperature[3];
            s_status.temp_ambient_c   = p->temp_ambient_c;

            s_status.humidity         = p->humidity;

            s_status.current_output[0] = p->current_output[0];
            s_status.current_output[1] = p->current_output[1];
            s_status.current_output[2] = p->current_output[2];
            s_status.current_output[3] = p->current_output[3];

            s_status.v_anode          = p->v_anode;
            s_status.enable           = p->enable;
            s_status.interlock_status = p->interlock_status;
            s_status.gate_status      = p->gate_status;
            s_status.OPM              = p->OPM;
            s_status.OPD              = p->OPD;
            s_status.sw_current_setpoint = p->sw_current_setpoint;
            s_status.hw_fault_source  = p->hw_fault_source;
            s_status.fsm_state        = p->fsm_state;
            break;
        }

        case LASEQ_MSG_CONFIG_ACK: {
            const LaseQ_PayloadConfigAck_t *p =
                (const LaseQ_PayloadConfigAck_t *)&s_rx_buf[LASEQ_OFF_PAYLOAD];

            /* fw_version/fw_build_date (NUOVO 0x0004, 2026-09-04): copiati
             * SEMPRE, anche su NAK (p->result != 0), cosi' "GET FW" puo'
             * riportare la versione di LaseQ anche quando l'ultima config
             * e' stata rifiutata. memcpy + azzeramento esplicito
             * dell'ultimo byte invece di fidarsi ciecamente del NUL
             * ricevuto via RS485 (difesa in profondita', stesso schema
             * gia' usato in AMC_SendConfig() per fw_version di AMC). */
            memcpy(s_status.fw_version, p->fw_version, sizeof(s_status.fw_version));
            s_status.fw_version[sizeof(s_status.fw_version) - 1U] = '\0';
            memcpy(s_status.fw_build_date, p->fw_build_date, sizeof(s_status.fw_build_date));
            s_status.fw_build_date[sizeof(s_status.fw_build_date) - 1U] = '\0';

            if (p->result != 0x00U) {
                return LASEQ_ERR_CONFIG_NAK;
            }
            break;
        }

        case LASEQ_MSG_PONG:
            /* Nessun dato da estrarre: la ricezione stessa conferma il link */
            break;

        default:
            return LASEQ_ERR_INVALID_RSP;
    }

    /* Dopo una risposta valida, azzera clear_error per non ritrametterlo */
    s_ctrl.clear_error = 0;

    return LASEQ_OK;
}

/* ============================================================================
 * CALLBACK ISR
 * ============================================================================ */
void LaseQ_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart != lq_huart) {
        return;
    }

    /*
     * CRITICITÀ DIREZIONE RS485:
     * Questo evento scatta solo quando l'ultimo stop bit è uscito
     * dallo shift register. Solo ora è sicuro abbassare DE.
     */
    RS485_RX_EN();

    if (lq_on_tx_cplt != NULL) {
        lq_on_tx_cplt();
    }
}

void LaseQ_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart != lq_huart) {
        return;
    }

    if (lq_on_rx_cplt != NULL) {
        lq_on_rx_cplt();
    }
}

void LaseQ_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart != lq_huart) {
        return;
    }

    /*
     * Su errore RS485 (rumore/framing/overrun) l'HAL ha già abortito il
     * transfer DMA in corso e riportato huart in stato Ready. Il DE resta
     * comunque in RX (RS485_RX_EN già impostato da TxCpltCallback o mai
     * spostato da RX se l'errore è avvenuto in ricezione) quindi non serve
     * toccare RS485_TX_EN/RX_EN qui.
     *
     * Rilasciamo entrambi i callback (tx/rx) per sbloccare subito
     * l'eventuale osSemaphoreAcquire pendente in laseq_transact(), invece
     * di aspettare il timeout pieno. Quello dei due non atteso in quel
     * momento resta come token "orfano": va ripulito con un drain
     * difensivo a inizio della transazione successiva (stesso schema già
     * usato in AMC.c / amc_transceive()).
     */
    if (lq_on_tx_cplt != NULL) {
        lq_on_tx_cplt();
    }
    if (lq_on_rx_cplt != NULL) {
        lq_on_rx_cplt();
    }
}

/* ============================================================================
 * GPIO HARDWARE (invariate rispetto all'originale)
 * ============================================================================ */
void LaseQSupplySet(bool status)
{
    HAL_GPIO_WritePin(nLASE_Q_PWR_SHDN_GPIO_Port,
                      nLASE_Q_PWR_SHDN_Pin,
                      (GPIO_PinState)status);
}

void HWGateSet(bool status)
{
    HAL_GPIO_WritePin(nGATE_MC_GPIO_Port,
                      nGATE_MC_Pin,
                      (GPIO_PinState)(!status));
}

void HWEnableSet(bool status)
{
    HAL_GPIO_WritePin(nLASE_Q_EN_GPIO_Port,
                      nLASE_Q_EN_Pin,
                      (GPIO_PinState)(!status));
}

void LaseQInterlockSet(bool status)
{
    HAL_GPIO_WritePin(nLASE_Q_INTLCK_GPIO_Port,
                      nLASE_Q_INTLCK_Pin,
                      (GPIO_PinState)(!status));
}
