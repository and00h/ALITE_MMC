/*
 * laseq_protocol.h
 *
 * ============================================================================
 * PROTOCOLLO DI COMUNICAZIONE RS485 - LaseQ
 * ============================================================================
 *
 * File condiviso tra master (STM32H723) e slave (STM32G484).
 * NON include header FreeRTOS, HAL o applicativi: dipende solo da stdint.h.
 *
 * STRUTTURA FRAME (lunghezza fissa: LASEQ_FRAME_SIZE = 47 byte):
 *
 *  Byte  0      : START_BYTE   (0xAA)
 *  Byte  1      : SENDER       (indirizzo mittente)
 *  Byte  2      : RECEIVER     (indirizzo destinatario)
 *  Byte  3      : MSG_TYPE     (vedi LaseQ_MsgType_t)
 *  Byte  4..43  : PAYLOAD      (LASEQ_PAYLOAD_SIZE = 40 byte fissi)
 *  Byte  44     : CRC16 high byte
 *  Byte  45     : CRC16 low byte
 *  Byte  46     : STOP_BYTE    (0x55)
 *
 * CRC16 CCITT calcolato su byte 0..43 (START + HEADER + PAYLOAD).
 *
 * AGGIUNTA CAMPI:
 *   Usare i byte reserved[] in fondo alla struct payload corrispondente,
 *   decrementandone la dimensione del numero di byte aggiunti.
 *   Incrementare LASEQ_PROTOCOL_VERSION ad ogni modifica.
 *   I _Static_assert garantiscono che il build fallisca se le dimensioni
 *   non tornano, prima ancora di andare sul target.
 *
 * CHANGELOG:
 *   0x0002 -> 0x0003 (Luca, 2026-07-16):
 *     LaseQ_PayloadStatus_t: aggiunto hw_fault_source (uint8_t, 1 byte),
 *     preso da _reserved_sht[3] -> _reserved_sht[2]. Prima di questo campo,
 *     error_code bit7 (FSM_FAULT_HW) accorpava tre cause hardware distinte
 *     lato LaseQ (PWR_OK basso, interlock non confermato all'abilitazione,
 *     interlock aperto durante ENABLED oltre la finestra di grazia) senza
 *     modo di distinguerle da RS485 — solo error_code=0x80 "fault hardware
 *     generico". hw_fault_source e' una bitmask OR-accumulata (stesso
 *     schema di error_code), vedi FSM_HwFaultSource_t in fsm.h/FSM.h per i
 *     bit. Non sostituisce error_code (bit7=FSM_FAULT_HW resta invariato):
 *     lo dettaglia. Decodificato lato MMC in "GET LQ" (rs485_cmd.c).
 *   0x0003 -> 0x0004 (Luca, 2026-09-04):
 *     LaseQ_PayloadConfigAck_t: aggiunti fw_version[16] e fw_build_date[18]
 *     (char, ASCII NUL-terminated), presi da reserved[37] -> reserved[3].
 *     Permettono a MMC di leggere versione e data/ora di build del
 *     firmware applicativo REALMENTE in esecuzione su LaseQ (stesso schema
 *     di FW_VERSION_STR/FW_BUILD_DATE_STR lato MMC, vedi
 *     MMC/App/Config/fw_version.h), senza un canale separato. Popolati da
 *     LaseQ in send_config_ack() (App/Tasks/rs485_handler.c) su OGNI
 *     risposta CONFIG_ACK (esito OK o NAK), cosi' la versione resta
 *     leggibile anche quando la config viene rifiutata. Letti lato MMC in
 *     LaseQ_ParseResponse() (LaseQ.c), esposti in LaseQ_status_vars_t
 *     (LaseQ.h), riportati dal comando RS485 "GET FW" (rs485_cmd.c). Un
 *     LaseQ con firmware PIU' VECCHIO (che non popola ancora questi campi)
 *     risponde con reserved a zero: MMC tratta stringhe vuote come
 *     "sconosciute" - nessun impatto sulla logica esistente (il controllo
 *     di versione resta su protocol_version_h/l, invariato).
 * ============================================================================
 */

#ifndef LASEQ_PROTOCOL_H_
#define LASEQ_PROTOCOL_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* ============================================================================
 * VERSIONE PROTOCOLLO
 * ============================================================================ */
#define LASEQ_PROTOCOL_VERSION      0x0004U

/* ============================================================================
 * COSTANTI FRAME
 * ============================================================================ */
#define LASEQ_START_BYTE            0xAAU
#define LASEQ_STOP_BYTE             0x55U

#define LASEQ_PAYLOAD_SIZE          40U
#define LASEQ_HEADER_SIZE           4U      /* START + SENDER + RECEIVER + MSG_TYPE */
#define LASEQ_CRC_SIZE              2U
#define LASEQ_FOOTER_SIZE           1U      /* STOP */
#define LASEQ_FRAME_SIZE            (LASEQ_HEADER_SIZE   \
                                   + LASEQ_PAYLOAD_SIZE  \
                                   + LASEQ_CRC_SIZE      \
                                   + LASEQ_FOOTER_SIZE)  /* = 47 */

/* Offset nel buffer raw */
#define LASEQ_OFF_START             0U
#define LASEQ_OFF_SENDER            1U
#define LASEQ_OFF_RECEIVER          2U
#define LASEQ_OFF_MSG_TYPE          3U
#define LASEQ_OFF_PAYLOAD           4U
#define LASEQ_OFF_CRC_H             (LASEQ_OFF_PAYLOAD + LASEQ_PAYLOAD_SIZE)
#define LASEQ_OFF_CRC_L             (LASEQ_OFF_CRC_H + 1U)
#define LASEQ_OFF_STOP              (LASEQ_OFF_CRC_L + 1U)

/* ============================================================================
 * INDIRIZZI
 * ============================================================================ */
#define LASEQ_ADDR_MASTER           0x01U
#define LASEQ_ADDR_BROADCAST        0xFFU

/* ============================================================================
 * TIPI DI MESSAGGIO
 * ============================================================================ */
typedef enum {
    LASEQ_MSG_CONTROL       = 0x10U,  /* Master -> Slave: comandi operativi   */
    LASEQ_MSG_STATUS        = 0x11U,  /* Slave  -> Master: telemetria e stato */
    LASEQ_MSG_CONFIG_SET    = 0x20U,  /* Master -> Slave: scrittura parametri */
    LASEQ_MSG_CONFIG_ACK    = 0x21U,  /* Slave  -> Master: conferma config    */
    LASEQ_MSG_PING          = 0x30U,  /* Master -> Slave: keepalive           */
    LASEQ_MSG_PONG          = 0x31U,  /* Slave  -> Master: risposta keepalive */
} LaseQ_MsgType_t;

/* ============================================================================
 * MODALITA' OPERATIVE (condiviso tra master e slave)
 * ============================================================================ */
typedef enum {
    SW      = 0,
    ANALOG  = 1,
    HYBRID  = 2,
} LaseQ_mode_t;

/* ============================================================================
 * PAYLOAD: MSG_CONTROL  (Master -> Slave, 40 byte)
 * ============================================================================ */
typedef struct __attribute__((packed)) {
    uint8_t     enable;             /*  1 */
    uint8_t     ch_enable;          /*  1 */
    uint8_t     gate;               /*  1 */
    uint8_t     sw_control;         /*  1 */
    uint8_t     analog_mode;        /*  1 */
    uint8_t     control_state;      /*  1 */
    uint8_t     interlock_status;   /*  1 */
    uint8_t     clear_error;        /*  1 */
    uint8_t     OPM;                /*  1 */
    uint8_t     OPD;                /*  1 */ /* subtot: 10 */
    uint32_t    sw_current_setpoint;   /*  4 */ /* subtot: 14 */
    uint8_t     reserved[26];       /* 26 */ /* tot:    40 */
} LaseQ_PayloadControl_t;

/* ============================================================================
 * PAYLOAD: MSG_STATUS  (Slave -> Master, 40 byte)
 * ============================================================================ */
typedef struct __attribute__((packed)) {
    uint8_t     driver_address;     /*  1 */
    uint8_t     error_code;         /*  1 */ /* subtot:  2 */
    uint8_t     temperature[4];     /*  4 */ /* subtot:  6 */  /* 4 temp elementi attivi [°C] */
    uint16_t    humidity;           /*  2 */ /* subtot:  8 */  /* SHT35: 0.01 %RH (es. 5000 = 50.00 %RH) */
    uint32_t    current_output[4];  /* 16 */ /* subtot: 24 */
    uint16_t    v_anode;            /*  2 */ /* subtot: 26 */
    uint8_t     enable;             /*  1 */
    uint8_t     interlock_status;   /*  1 */
    uint8_t     gate_status;        /*  1 */
    uint8_t     OPM;                /*  1 */
    uint8_t     OPD;                /*  1 */ /* subtot: 31 */
    uint32_t    sw_current_setpoint;/*  4 */ /* subtot: 35 */
    int8_t      temp_ambient_c;     /*  1 */ /* subtot: 36 */  /* SHT35 temperatura ambiente [°C] */
    uint8_t     hw_fault_source;    /*  1 */ /* subtot: 37 */  /* bitmask FSM_HwFaultSource_t (fsm.h), dettaglio di error_code bit7 */
    uint8_t     _reserved_sht[2];   /*  2 */ /* subtot: 39 */
    uint8_t     fsm_state;          /*  1 */ /* subtot: 40 */
} LaseQ_PayloadStatus_t;

/* ============================================================================
 * PAYLOAD: MSG_CONFIG_SET  (Master -> Slave, 40 byte)
 * ============================================================================ */
typedef struct __attribute__((packed)) {
    uint16_t    Vanode_min_err;         /*  2 */
    uint16_t    Vanode_max_err;         /*  2 */
    uint16_t    CurrentSet_min;         /*  2 */
    uint16_t    CurrentSet_max;         /*  2 */
    uint16_t    CurrentMonitor_max_err; /*  2 */
    uint16_t    SaturationThr;          /*  2 */ /* subtot: 12 */
    int16_t     temp_driver_min_err;    /*  2 */
    int16_t     temp_driver_max_err;    /*  2 */
    int16_t     temp_ambient_min_err;   /*  2 */
    int16_t     temp_ambient_max_err;   /*  2 */ /* subtot: 20 */
    uint8_t     laser_mode;             /*  1 */
    uint8_t     ch_enable;              /*  1 */
    uint16_t    startup_delay_ms;       /*  2 */ /* subtot: 24 */
    uint8_t     reserved[16];           /* 16 */ /* tot:    40 */
} LaseQ_PayloadConfigSet_t;

/* ============================================================================
 * PAYLOAD: MSG_CONFIG_ACK  (Slave -> Master, 40 byte)
 * ============================================================================ */
typedef struct __attribute__((packed)) {
    uint8_t     result;                 /*  1: 0=OK, altri=errore */
    uint8_t     protocol_version_h;     /*  1 */
    uint8_t     protocol_version_l;     /*  1 */ /* subtot:  3 */
    char        fw_version[16];         /* 16: FW_VERSION_STR di LaseQ, ASCII
                                          *     NUL-terminated, troncata a 15
                                          *     char + NUL (NUOVO 0x0004,
                                          *     2026-09-04) */ /* subtot: 19 */
    char        fw_build_date[18];      /* 18: data/ora di build
                                          *     (__DATE__ " " __TIME__),
                                          *     troncata a 17 char + NUL
                                          *     (NUOVO 0x0004) */ /* subtot: 37 */
    uint8_t     reserved[3];            /*  3 (era 37 prima di 0x0004) */
                                        /* tot:    40 */
} LaseQ_PayloadConfigAck_t;

/* ============================================================================
 * PAYLOAD: MSG_PING / MSG_PONG  (40 byte)
 * ============================================================================ */
typedef struct __attribute__((packed)) {
    uint16_t    seq;                    /*  2 */
    uint8_t     reserved[38];           /* 38 */ /* tot: 40 */
} LaseQ_PayloadPing_t;

/* ============================================================================
 * FRAME COMPLETO
 * ============================================================================ */
typedef struct __attribute__((packed)) {
    uint8_t     start;
    uint8_t     sender;
    uint8_t     receiver;
    uint8_t     msg_type;
    union {
        uint8_t                  raw[LASEQ_PAYLOAD_SIZE];
        LaseQ_PayloadControl_t   control;
        LaseQ_PayloadStatus_t    status;
        LaseQ_PayloadConfigSet_t config_set;
        LaseQ_PayloadConfigAck_t config_ack;
        LaseQ_PayloadPing_t      ping;
    } payload;
    uint8_t     crc_h;
    uint8_t     crc_l;
    uint8_t     stop;
} LaseQ_Frame_t;

/* ============================================================================
 * VERIFICA DIMENSIONI A COMPILE TIME
 * ============================================================================ */
_Static_assert(sizeof(LaseQ_PayloadControl_t)   == LASEQ_PAYLOAD_SIZE,
               "LaseQ_PayloadControl_t size mismatch");
_Static_assert(sizeof(LaseQ_PayloadStatus_t)    == LASEQ_PAYLOAD_SIZE,
               "LaseQ_PayloadStatus_t size mismatch");
_Static_assert(sizeof(LaseQ_PayloadConfigSet_t) == LASEQ_PAYLOAD_SIZE,
               "LaseQ_PayloadConfigSet_t size mismatch");
_Static_assert(sizeof(LaseQ_PayloadConfigAck_t) == LASEQ_PAYLOAD_SIZE,
               "LaseQ_PayloadConfigAck_t size mismatch");
_Static_assert(sizeof(LaseQ_PayloadPing_t)      == LASEQ_PAYLOAD_SIZE,
               "LaseQ_PayloadPing_t size mismatch");
_Static_assert(sizeof(LaseQ_Frame_t)            == LASEQ_FRAME_SIZE,
               "LaseQ_Frame_t size mismatch");

/* ============================================================================
 * CRC16 CCITT (poly 0x1021, init 0xFFFF)
 * static inline: nessuna translation unit separata, funziona su entrambi
 * i target senza problemi di linking.
 * ============================================================================ */
static inline uint16_t LaseQ_CRC16(const uint8_t *data, uint16_t len)
{
    uint16_t crc = 0xFFFFU;
    for (uint16_t i = 0U; i < len; i++) {
        crc ^= ((uint16_t)data[i] << 8U);
        for (uint8_t b = 0U; b < 8U; b++) {
            crc = (crc & 0x8000U)
                ? (uint16_t)((crc << 1U) ^ 0x1021U)
                : (uint16_t)(crc << 1U);
        }
    }
    return crc;
}

/* ============================================================================
 * HELPERS FRAME (inline, nessuna dipendenza esterna)
 * ============================================================================ */

/**
 * @brief  Scrive CRC e STOP nel buffer raw dopo che header e payload
 *         sono stati riempiti.
 */
static inline void LaseQ_FrameFinalize(uint8_t *buf)
{
    uint16_t crc = LaseQ_CRC16(buf, LASEQ_OFF_CRC_H);
    buf[LASEQ_OFF_CRC_H] = (uint8_t)(crc >> 8U);
    buf[LASEQ_OFF_CRC_L] = (uint8_t)(crc & 0xFFU);
    buf[LASEQ_OFF_STOP]  = LASEQ_STOP_BYTE;
}

/**
 * @brief  Valida START, STOP e CRC di un frame raw ricevuto.
 * @retval 1 se valido, 0 altrimenti.
 */
static inline int LaseQ_FrameValidate(const uint8_t *buf)
{
    if (buf[LASEQ_OFF_START] != LASEQ_START_BYTE) return 0;
    if (buf[LASEQ_OFF_STOP]  != LASEQ_STOP_BYTE)  return 0;
    uint16_t crc_calc = LaseQ_CRC16(buf, LASEQ_OFF_CRC_H);
    uint16_t crc_rx   = ((uint16_t)buf[LASEQ_OFF_CRC_H] << 8U)
                       |  (uint16_t)buf[LASEQ_OFF_CRC_L];
    return (crc_calc == crc_rx) ? 1 : 0;
}

#ifdef __cplusplus
}
#endif

#endif /* LASEQ_PROTOCOL_H_ */
