/*
 * COM_interface_protocol.h
 *
 * ============================================================================
 * PROTOCOLLO DI COMUNICAZIONE SPI3 - MMC (master) <-> COM interface (slave)
 * ============================================================================
 *
 * File di solo framing/trasporto, senza dipendenze HAL/FreeRTOS (solo
 * stdint.h) — stesso stile di AMC_protocol.h e laseq_protocol.h.
 *
 * AGGIORNAMENTO (2026-07-23): il framing sotto e' stato riallineato BIT PER
 * BIT all'implementazione gia' presente lato COM interface (ALITE_COM,
 * Core/Inc/ALITE/com/com.h + com_protocol.c/h), per poter eseguire il test
 * SPI periodico (COM_MSG_TEST, ogni 10s) descritto dal team COM. ALITE_COM
 * NON e' stato toccato: questo file adatta solo il lato MMC.
 *
 * Il protocollo APPLICATIVO (contenuto dei payload STATUS/CONTROL/CONFIG)
 * resta da definire — vedi COM_interface.h. Il solo tipo TEST e' oggi
 * pienamente gestito (echo del payload ricevuto, vedi task_com_interface.c).
 *
 * SEQUENZA DI SINCRONISMO (invariata, vedi COM_interface.h):
 *   1. nCOM_INT_IN (fronte di DISCESA, PD1 idle-high / assert-low lato COM,
 *      vedi COM_assert_int_out() in com_spi.c) -> la COM interface segnala
 *      che ha un messaggio pronto per il MMC.
 *   2. Il MMC (master SPI) invia un frame "vuoto" (COM_MSG_EMPTY, contenuto
 *      non significativo): serve ESCLUSIVAMENTE a generare il clock SPI
 *      necessario perche' la COM interface, che ha gia' armato in slave-TX
 *      il proprio frame reale, lo metta sul bus. Il MMC cattura quel frame
 *      in RX nella stessa transazione full-duplex.
 *   3. Il MMC elabora la richiesta ricevuta e risponde con un secondo frame:
 *      per COM_MSG_TEST, il payload ricevuto viene ripetuto identico
 *      (richiesto dal protocollo: la COM interface verifica che i byte
 *      payload[0..251] siano 0..251 nella risposta). Per gli altri tipi
 *      (STATUS/CONTROL/CONFIG), protocollo applicativo non ancora definito:
 *      si risponde con un frame COM_MSG_EMPTY placeholder.
 *
 * STRUTTURA FRAME (lunghezza fissa a 256 byte, come da specifica COM):
 *   Byte 0       : START_BYTE  (0xAA)
 *   Byte 1       : MSG_TYPE    (COM_MsgType_t)
 *   Byte 2..253   : PAYLOAD     (COM_PAYLOAD_SIZE = 252 byte)
 *   Byte 254      : CRC8        (poly 0x07, init 0x00 — vedi COM_FrameCRC)
 *   Byte 255      : END_BYTE    (0x5A)
 *
 * A differenza della revisione precedente di questo file, COM_FRAME_SIZE e'
 * ora il valore FISSO che arriva dalla specifica COM (256): e' COM_PAYLOAD_SIZE
 * a essere derivato da esso (COM_FRAME_SIZE - 4), non il contrario.
 *
 * VERSIONE HISTORY:
 *   0x0001 - framing provvisorio locale (32B payload, CRC16, 0xC0/0xC1) —
 *            SUPERATO, mai stato compatibile con la COM interface reale.
 *   0x0002 - framing riallineato a COM interface (256B, 0xAA/0x5A, CRC8),
 *            COM_MSG_TEST gestito con echo (2026-07-23).
 * ============================================================================
 */

#ifndef DRIVERS_COM_INTERFACE_COM_INTERFACE_PROTOCOL_H_
#define DRIVERS_COM_INTERFACE_COM_INTERFACE_PROTOCOL_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stddef.h>   /* offsetof(), per _Static_assert su COM_StatusTable_t */

/* ============================================================================
 * VERSIONE PROTOCOLLO
 * ============================================================================ */
#define COM_PROTOCOL_VERSION        0x0002U

/* ============================================================================
 * COSTANTI FRAME — allineate a COM_FRAME_SIZE/COM_START_BYTE/COM_END_BYTE di
 * ALITE_COM (Core/Inc/ALITE/com/com.h). COM_FRAME_SIZE e' l'ancora fissa: NON
 * derivarla da COM_PAYLOAD_SIZE, e' il contrario (come lato COM).
 * ============================================================================ */
#define COM_FRAME_SIZE               256U
#define COM_START_BYTE               0xAAU
#define COM_END_BYTE                 0x5AU

#define COM_OFF_START                0U
#define COM_OFF_MSG_TYPE             1U
#define COM_OFF_PAYLOAD              2U
#define COM_PAYLOAD_SIZE             (COM_FRAME_SIZE - 4U)   /* 252 byte */
#define COM_OFF_CRC                  (COM_FRAME_SIZE - 2U)   /* 254 */
#define COM_OFF_END                  (COM_FRAME_SIZE - 1U)   /* 255 */

/* ============================================================================
 * TIPI DI MESSAGGIO — stessi valori di COM_msg_code_e (ALITE_COM, com.h).
 * ============================================================================ */
typedef enum {
    COM_MSG_STATUS  = 0x00U,
    COM_MSG_CONTROL = 0x01U,
    COM_MSG_CONFIG  = 0x02U,
    COM_MSG_TEST    = 0x55U,
} COM_MsgType_t;

/* ============================================================================
 * PROTOCOLLO APPLICATIVO — allineato a ALITE_COM (Core/Inc/ALITE/com/com.h),
 * aggiornamento 2026-07-28. Vedi COM_interface_app.h/.c per l'interpretazione
 * lato MMC di questi payload (STATUS/CONTROL/CONFIG). Tenere sincronizzato
 * MANUALMENTE con com.h/com_dispatch.h lato ALITE_COM: nessun header condiviso
 * tra le due board, stesso schema gia' in uso per il resto di questo file.
 * ============================================================================ */

/* --- CONTROL: opcode (ALITE_COM com.h, COM_control_opcode_e) --------------- */
typedef enum {
    COM_CTRL_OP_START           = 1,
    COM_CTRL_OP_STOP            = 2,
    COM_CTRL_OP_SON             = 3,
    COM_CTRL_OP_SOFF            = 4,
    COM_CTRL_OP_SEN             = 5,
    COM_CTRL_OP_SDIS            = 6,
    COM_CTRL_OP_PON             = 7,
    COM_CTRL_OP_POFF            = 8,
    COM_CTRL_OP_CERR            = 9,
    COM_CTRL_OP_FRST            = 10,
    COM_CTRL_OP_SAVE_CONF       = 11,
    COM_CTRL_OP_SAVE_LUT        = 12,
    COM_CTRL_OP_RESET_LUT       = 13,
    COM_CTRL_OP_LOGIN           = 14,
    COM_CTRL_OP_LOGOUT          = 15,
    COM_CTRL_OP_SETMODE         = 16,
    COM_CTRL_OP_SETGATEHW       = 17,
    COM_CTRL_OP_SETSETPOINTHW   = 18,
    COM_CTRL_OP_SETSETPOINT     = 19,
    COM_CTRL_OP_SETCURRENT      = 20,
    COM_CTRL_OP_SETLUTGAIN      = 32,
    COM_CTRL_OP_SETLUTVALID     = 33,
    COM_CTRL_OP_SETLUTPOWER     = 34,
    COM_CTRL_OP_SETLUTSETPOINT  = 35,
    COM_CTRL_OP_SETERRMASK      = 48,
    COM_CTRL_OP_SETWARNMASK     = 49,
    COM_CTRL_OP_SETFAULTMASK    = 50,
    COM_CTRL_OP_SETFAULTLATCH   = 51,
    COM_CTRL_OP_SET_TEMP_MASK   = 52,
    COM_CTRL_OP_SET_FLOW_MASK   = 53,
    COM_CTRL_OP_SET_PSU_MASK    = 54,
    COM_CTRL_OP_SET_CONTACTOR_MASK = 55,
    COM_CTRL_OP_SET_EFUSE_MASK  = 56,
    COM_CTRL_OP_SET_PD_MASK     = 57,
    COM_CTRL_OP_SET_TEMP_THR    = 64,
    COM_CTRL_OP_SET_FLOW_THR    = 65,
    COM_CTRL_OP_SET_HUM_THR     = 66,
    COM_CTRL_OP_SET_DEW_THR     = 67,
    COM_CTRL_OP_SET_DELAY       = 68,
    COM_CTRL_OP_SET_PSU_V       = 69,
    COM_CTRL_OP_SET_NTC_MAP     = 70,
    COM_CTRL_OP_SET_FREQ        = 80,
    COM_CTRL_OP_SET_DUTY        = 81,
    COM_CTRL_OP_SET_QCW         = 82,
    COM_CTRL_OP_SET_PASS        = 96,
    COM_CTRL_OP_SETTIME         = 112,
    COM_CTRL_OP_SETDATE         = 113,
    COM_CTRL_OP_SETTERM         = 114,
} COM_ControlOpcode_t;

/* --- CONTROL: layout payload (ALITE_COM com_dispatch.h, COM_CTRL_PAYLOAD_*) --
 *
 * Byte 0 e' sempre l'opcode. Gli opcode "storici" (1..20 e 80..82) portano i
 * campi fissi sotto nei byte 1..10; tutti gli altri portano fino a 32 byte di
 * argomenti a partire da COM_CTRL_PAYLOAD_ARGS (byte 1..32), con layout
 * specifico per comando (COM_CTRL_*_THR_* sotto, offset relativi a ARGS). */
#define COM_CTRL_PAYLOAD_OPCODE                 0U
#define COM_CTRL_PAYLOAD_FSM                    1U
#define COM_CTRL_PAYLOAD_SW_POWER_SETPOINT      2U
#define COM_CTRL_PAYLOAD_QCW_ERR                3U
#define COM_CTRL_PAYLOAD_QCW_FREQ_HZ            4U   /* uint32 LE, byte 4..7 */
#define COM_CTRL_PAYLOAD_QCW_DC                 8U
#define COM_CTRL_PAYLOAD_MODE                   9U
#define COM_CTRL_PAYLOAD_GATE_HW_SETPOINT_HW    10U

#define COM_CTRL_PAYLOAD_ARGS                   1U
#define COM_CTRL_PAYLOAD_ARGS_SIZE              32U

#define COM_CTRL_PAYLOAD_SIZE                   (COM_CTRL_PAYLOAD_ARGS + COM_CTRL_PAYLOAD_ARGS_SIZE)   /* 33 */

/* SET_TEMP_THR (64): sensore, livello, limite, valore [°C] int16 LE */
#define COM_CTRL_TEMP_THR_SENSOR_ID             0U
#define COM_CTRL_TEMP_THR_LEVEL                 1U
#define COM_CTRL_TEMP_THR_LIMIT                 2U
#define COM_CTRL_TEMP_THR_VALUE_C               3U

/* SET_FLOW_THR (65): livello, limite, valore [L/min x10] uint16 LE */
#define COM_CTRL_FLOW_THR_LEVEL                 0U
#define COM_CTRL_FLOW_THR_LIMIT                 1U
#define COM_CTRL_FLOW_VALUE_LPM_X10             2U

/* SET_HUM_THR (66): target, valore [%RH] */
#define COM_CTRL_HUM_THR_TARGET                 0U
#define COM_CTRL_HUM_THR_VALUE_PCT              1U

/* SET_DEW_THR (67): livello, margine [°C] */
#define COM_CTRL_DEW_THR_LEVEL                  0U
#define COM_CTRL_DEW_THR_VALUE_C                1U

/* Codifiche degli argomenti soglia (ALITE_COM alite_system.h,
 * ALITE_threshold_level_e/_limit_e/_hum_target_e/_sensor_e). */
#define COM_CTRL_THR_LEVEL_WARN                 0U
#define COM_CTRL_THR_LEVEL_ERR                  1U
#define COM_CTRL_THR_LIMIT_MIN                  0U
#define COM_CTRL_THR_LIMIT_MAX                  1U
#define COM_CTRL_THR_HUM_MMC                    0U
#define COM_CTRL_THR_HUM_LASEQ                  1U

/* sensor_id di SET_TEMP_THR: stesso ordine dei blocchi THR_* della tabella
 * di stato (COM_STAT_OP_THR_WATER_IN_* .. COM_STAT_OP_THR_LQ_AMBIENT_*). */
typedef enum {
    COM_THR_SENSOR_WATER_IN   = 0,
    COM_THR_SENSOR_WATER_OUT  = 1,
    COM_THR_SENSOR_DRIVER     = 2,
    COM_THR_SENSOR_SPLICE     = 3,
    COM_THR_SENSOR_DIODE1     = 4,
    COM_THR_SENSOR_DIODE2     = 5,
    COM_THR_SENSOR_AMBIENT    = 6,
    COM_THR_SENSOR_LQ_AMBIENT = 7,
    COM_THR_SENSOR_COUNT
} COM_ThrSensorId_t;

/* fsm_flags (byte 1) — bit order aggiornato 2026-07-28 (PON e' il bit piu'
 * significativo, START il meno significativo: "PON | SEN | SON | START"). */
#define COM_CTRL_FLAG_PON    (1U << 3)   /* power on            */
#define COM_CTRL_FLAG_SEN    (1U << 2)   /* system enable       */
#define COM_CTRL_FLAG_SON    (1U << 1)   /* system on           */
#define COM_CTRL_FLAG_START  (1U << 0)   /* start               */

/* qcw_ctrl (byte 3) */
#define COM_CTRL_QCW_ENABLE     (1U << 0)
#define COM_CTRL_CLEAR_ERROR    (1U << 1)

/* hw_ctrl (byte 10) */
#define COM_CTRL_GATE_HW_ENABLE       (1U << 0)
#define COM_CTRL_SETPOINT_HW_ENABLE   (1U << 1)

/* mode (byte 9) — convenzione non ancora formalizzata lato ALITE_COM (il
 * campo e' un uint8_t grezzo): allineata qui alla numerazione gia' in uso
 * per g_config.laser_mode/LaseQ_mode_t (0=SW, 1=ANALOG, 2=HYBRID). Da
 * confermare col team COM se la codifica coincide. */
#define COM_CTRL_MODE_SW       0U
#define COM_CTRL_MODE_ANALOG   1U
#define COM_CTRL_MODE_HYBRID   2U

/* --- CONTROL: risposta (ALITE_COM com.h, COM_resp_status_e / COM_control_resp_t) */
typedef enum {
    COM_RESP_OK   = 0,
    COM_RESP_AUTH = 1,
    COM_RESP_ERR  = 0xFF,
} COM_RespStatus_t;

/* --- STATUS: opcode (ALITE_COM com.h, COM_status_opcode_e) ----------------- */
typedef enum {
    COM_STAT_OP_SYS_STATE               = 0,
    COM_STAT_OP_SAB_STATE               = 1,
    COM_STAT_OP_SETPOINT_PCT            = 2,
    COM_STAT_OP_SETPOINT_CUR_MA         = 3,
    COM_STAT_OP_SYS_FLAGS_0             = 5,
    COM_STAT_OP_SYS_FLAGS_1             = 6,
    COM_STAT_OP_SYS_FLAGS_2             = 7,
    COM_STAT_OP_SYS_FLAGS_3             = 8,
    COM_STAT_OP_SYS_FLAGS_4             = 9,
    COM_STAT_OP_PD0_POWER_W             = 10,
    COM_STAT_OP_PD1_POWER_W             = 12,
    COM_STAT_OP_PD2_POWER_W             = 14,
    COM_STAT_OP_PD3_POWER_W             = 16,
    COM_STAT_OP_ERR_ACTIVE              = 18,
    COM_STAT_OP_ERR_MASK                = 22,
    COM_STAT_OP_WARN_ACTIVE             = 26,
    COM_STAT_OP_WARN_MASK               = 30,
    COM_STAT_OP_FAULT_ACTIVE            = 34,
    COM_STAT_OP_FAULT_MASK              = 38,
    COM_STAT_OP_FAULT_LATCH_MASK        = 42,
    COM_STAT_OP_FAULT_LATCH_ACTIVE      = 46,
    COM_STAT_OP_TEMP_FAULT_SENSOR_MASK  = 50,
    COM_STAT_OP_LAST_FAULT_EVENT_ID     = 53,
    COM_STAT_OP_LAST_ERROR_EVENT_ID     = 54,
    COM_STAT_OP_LQ_ERROR_CODE           = 55,
    COM_STAT_OP_LQ_CH_MASK              = 56,
    COM_STAT_OP_LQ_ILK                  = 57,
    COM_STAT_OP_LQ_OPM                  = 58,
    COM_STAT_OP_LQ_OPD                  = 59,
    COM_STAT_OP_LQ_SP_CMD_MA            = 60,
    COM_STAT_OP_LQ_SP_ECHO_MA           = 62,
    COM_STAT_OP_LQ_VANODE_MV            = 64,
    COM_STAT_OP_LQ_I0_MA                = 66,
    COM_STAT_OP_LQ_I1_MA                = 68,
    COM_STAT_OP_LQ_I2_MA                = 70,
    COM_STAT_OP_LQ_I3_MA                = 72,
    COM_STAT_OP_LQ_FSM_STATE            = 74,
    COM_STAT_OP_MOD_MAIN_CURRENT_MA     = 75,
    COM_STAT_OP_MOD_SAB_CURRENT_MA      = 77,
    COM_STAT_OP_MODE_COM_CURRENT_MA     = 79,
    COM_STAT_OP_MODE_LASEQ_CURRENT_MA   = 81,
    COM_STAT_OP_DEW_MMC_TD_C10          = 83,
    COM_STAT_OP_DEW_MMC_MARGIN_C10      = 85,
    COM_STAT_OP_DEW_LQ_TD_C10           = 87,
    COM_STAT_OP_DEW_LQ_MARGIN_C10       = 89,
    COM_STAT_OP_TEMP_SHT35_MMC_C100     = 91,
    COM_STAT_OP_HUM_SHT35_MMC_PCT       = 93,
    COM_STAT_OP_NTC_TEMP_C10_WATER_IN   = 94,
    COM_STAT_OP_NTC_TEMP_C10_WATER_OUT  = 96,
    COM_STAT_OP_NTC_TEMP_C10_GEN2       = 98,
    COM_STAT_OP_NTC_TEMP_C10_SPLICE     = 100,
    COM_STAT_OP_NTC_TEMP_C10_DIODE1     = 102,
    COM_STAT_OP_NTC_TEMP_C10_AMBIENT    = 104,
    COM_STAT_OP_NTC_TEMP_C10_DIODE2     = 106,
    COM_STAT_OP_NTC_TEMP_C10_NTC8       = 108,
    COM_STAT_OP_NTC_TEMP_C10_NTC9       = 110,
    COM_STAT_OP_NTC_TEMP_C10_NTC10      = 112,
    COM_STAT_OP_NTC_TEMP_C10_NTC11      = 114,
    COM_STAT_OP_NTC_TEMP_C10_NTC12      = 116,
    COM_STAT_OP_NTC_TEMP_C10_NTC13      = 118,
    COM_STAT_OP_NTC_TEMP_C10_NTC14      = 120,
    COM_STAT_OP_NTC_TEMP_C10_NTC15      = 122,
    COM_STAT_OP_NTC_TEMP_C10_NTC16      = 124,
    COM_STAT_OP_LQ_DRIVER_TEMP_C_0      = 126,
    COM_STAT_OP_LQ_DRIVER_TEMP_C_1      = 127,
    COM_STAT_OP_LQ_DRIVER_TEMP_C_2      = 128,
    COM_STAT_OP_LQ_DRIVER_TEMP_C_3      = 129,
    COM_STAT_OP_LQ_AMBIENT_TEMP_C       = 130,
    COM_STAT_OP_LQ_HUMIDITY_PCT         = 131,
    COM_STAT_OP_FLOW1_LPM_X10           = 132,
    COM_STAT_OP_FLOW2_LPM_X10           = 134,
    COM_STAT_OP_NTC_MAP_SERIGRAFIA_01   = 136,
    COM_STAT_OP_NTC_MAP_SERIGRAFIA_02   = 137,
    COM_STAT_OP_NTC_MAP_SERIGRAFIA_03   = 138,
    COM_STAT_OP_NTC_MAP_SERIGRAFIA_04   = 139,
    COM_STAT_OP_NTC_MAP_SERIGRAFIA_05   = 140,
    COM_STAT_OP_NTC_MAP_SERIGRAFIA_06   = 141,
    COM_STAT_OP_NTC_MAP_SERIGRAFIA_07   = 142,
    COM_STAT_OP_NTC_MAP_SERIGRAFIA_08   = 143,
    COM_STAT_OP_NTC_MAP_SERIGRAFIA_09   = 144,
    COM_STAT_OP_NTC_MAP_SERIGRAFIA_10   = 145,
    COM_STAT_OP_NTC_MAP_SERIGRAFIA_11   = 146,
    COM_STAT_OP_NTC_MAP_SERIGRAFIA_12   = 147,
    COM_STAT_OP_NTC_MAP_SERIGRAFIA_13   = 148,
    COM_STAT_OP_NTC_MAP_SERIGRAFIA_14   = 149,
    COM_STAT_OP_NTC_MAP_SERIGRAFIA_15   = 150,
    COM_STAT_OP_NTC_MAP_SERIGRAFIA_16   = 151,
    COM_STAT_OP_NTC_ACTIVE_MASK         = 152,
    COM_STAT_OP_THR_WATER_IN_MIN_ERR_C  = 154,
    COM_STAT_OP_THR_WATER_IN_MIN_WARN_C = 156,
    COM_STAT_OP_THR_WATER_IN_MAX_WARN_C = 158,
    COM_STAT_OP_THR_WATER_IN_MAX_ERR_C  = 160,
    COM_STAT_OP_THR_WATER_OUT_MIN_ERR_C = 162,
    COM_STAT_OP_THR_WATER_OUT_MIN_WARN_C = 164,
    COM_STAT_OP_THR_WATER_OUT_MAX_WARN_C = 166,
    COM_STAT_OP_THR_WATER_OUT_MAX_ERR_C = 168,
    COM_STAT_OP_THR_DRIVER_MIN_ERR_C    = 170,
    COM_STAT_OP_THR_DRIVER_MIN_WARN_C   = 172,
    COM_STAT_OP_THR_DRIVER_MAX_WARN_C   = 174,
    COM_STAT_OP_THR_DRIVER_MAX_ERR_C    = 176,
    COM_STAT_OP_THR_SPLICE_MIN_ERR_C    = 178,
    COM_STAT_OP_THR_SPLICE_MIN_WARN_C   = 180,
    COM_STAT_OP_THR_SPLICE_MAX_WARN_C   = 182,
    COM_STAT_OP_THR_SPLICE_MAX_ERR_C    = 184,
    COM_STAT_OP_THR_DIODE1_MIN_ERR_C    = 186,
    COM_STAT_OP_THR_DIODE1_MIN_WARN_C   = 188,
    COM_STAT_OP_THR_DIODE1_MAX_WARN_C   = 190,
    COM_STAT_OP_THR_DIODE1_MAX_ERR_C    = 192,
    COM_STAT_OP_THR_DIODE2_MIN_ERR_C    = 194,
    COM_STAT_OP_THR_DIODE2_MIN_WARN_C   = 196,
    COM_STAT_OP_THR_DIODE2_MAX_WARN_C   = 198,
    COM_STAT_OP_THR_DIODE2_MAX_ERR_C    = 200,
    COM_STAT_OP_THR_AMBIENT_MIN_ERR_C   = 202,
    COM_STAT_OP_THR_AMBIENT_MIN_WARN_C  = 204,
    COM_STAT_OP_THR_AMBIENT_MAX_WARN_C  = 206,
    COM_STAT_OP_THR_AMBIENT_MAX_ERR_C   = 208,
    COM_STAT_OP_THR_LQ_AMBIENT_MIN_ERR_C = 210,
    COM_STAT_OP_THR_LQ_AMBIENT_MIN_WARN_C = 212,
    COM_STAT_OP_THR_LQ_AMBIENT_MAX_WARN_C = 214,
    COM_STAT_OP_THR_LQ_AMBIENT_MAX_ERR_C = 216,
    COM_STAT_OP_THR_FLOW_MIN_ERR_LPMX10 = 218,
    COM_STAT_OP_THR_FLOW_MIN_WARN_LPMX10 = 220,
    COM_STAT_OP_THR_FLOW_MAX_WARN_LPMX10 = 222,
    COM_STAT_OP_THR_FLOW_MAX_ERR_LPMX10 = 224,
    COM_STAT_OP_THR_HUM_MMC_MAX_WARN_PCT = 226,
    COM_STAT_OP_THR_HUM_LASEQ_MAX_WARN_PCT = 227,
    COM_STAT_OP_THR_DEW_WARN_DELTA_C    = 228,
    COM_STAT_OP_THR_DEW_ERR_DELTA_C     = 229,
    COM_STAT_OP_CFG_DLY_PSU_MS          = 230,
    COM_STAT_OP_CFG_DLY_SAB_MS          = 232,
    COM_STAT_OP_CFG_DLY_CONTACTOR_MS    = 234,
    COM_STAT_OP_RTC_HOUR                = 236,
    COM_STAT_OP_RTC_MIN                 = 237,
    COM_STAT_OP_RTC_SEC                 = 238,
    COM_STAT_OP_RTC_YEAR                = 239,
    COM_STAT_OP_RTC_MONTH               = 241,
    COM_STAT_OP_RTC_DAY                 = 242,
} COM_StatusOpcode_t;

/* --- STATUS: formato risposta (AGGIORNAMENTO 2026-07-30) -------------------
 *
 * ALITE_COM ha cambiato il formato della risposta STATUS rispetto alla
 * revisione precedente di questo file. Non e' piu' "1 opcode richiesto -> 4
 * byte di risposta a posizione fissa": la richiesta ora porta una LISTA di
 * opcode (com.h, COM_status_req_t: n_requested + requested[]), e la risposta
 * e' un'UNICA TABELLA a lunghezza fissa dove ogni valore vive all'offset
 * ASSOLUTO indicato dal proprio COM_StatusOpcode_t, indipendentemente da
 * quanti/quali opcode erano stati richiesti. Prova diretta nel codice
 * ALITE_COM (Core/Src/ALITE/alite_system.c):
 *
 *   ALITE_system_update_status_full() legge
 *     resp.status_resp.data[COM_STAT_OP_SYS_STATE]
 *     *(uint32_t*)&resp.status_resp.data[COM_STAT_OP_ERR_ACTIVE]
 *     *(uint32_t*)&resp.status_resp.data[COM_STAT_OP_ERR_MASK]
 *   cioe' indicizza SEMPRE con il valore enum come offset di byte in data[],
 *   MAI in base all'ordine/posizione della richiesta.
 *
 * Di conseguenza il MMC deve rispondere con la tabella COMPLETA ad ogni
 * richiesta STATUS, non solo con gli opcode elencati in requested[] — anche
 * perche' ALITE_get_system_err_flag() (stesso file) costruisce la richiesta
 * con un bug (n_requested viene impostato a s_full_status_opcodes[0], cioe'
 * al VALORE del primo opcode invece che alla lunghezza dell'array) che rende
 * requested[]/n_requested inaffidabile lato ALITE_COM: rispondere sempre con
 * la tabella intera e' l'unico comportamento robusto a entrambi gli usi
 * osservati. req_payload (n_requested/requested[]) e' quindi ignorato da
 * COM_App_HandleStatus() — vedi COM_interface_app.c.
 *
 * Il layout sotto rispecchia ESATTAMENTE gli offset/larghezze impliciti nella
 * spaziatura di COM_StatusOpcode_t sopra (larghezza di ogni campo = offset
 * del campo successivo - offset del campo corrente): verificato campo per
 * campo con _Static_assert(offsetof(...) == COM_STAT_OP_...) sotto, cosi' un
 * futuro disallineamento con com.h fa fallire la build invece di corrompere
 * silenziosamente la risposta.
 * ---------------------------------------------------------------------- */
typedef struct __attribute__((packed)) {
    uint8_t  sys_state;                    /* 0   */
    uint8_t  sab_state;                    /* 1   */
    uint8_t  setpoint_pct;                 /* 2   */
    uint16_t setpoint_cur_ma;              /* 3   */
    uint8_t  sys_flags_0;                  /* 5   */
    uint8_t  sys_flags_1;                  /* 6   */
    uint8_t  sys_flags_2;                  /* 7   */
    uint8_t  sys_flags_3;                  /* 8   */
    uint8_t  sys_flags_4;                  /* 9   */
    uint16_t pd0_power_w;                  /* 10  */
    uint16_t pd1_power_w;                  /* 12  */
    uint16_t pd2_power_w;                  /* 14  */
    uint16_t pd3_power_w;                  /* 16  */
    uint32_t err_active;                   /* 18  */
    uint32_t err_mask;                     /* 22  */
    uint32_t warn_active;                  /* 26  */
    uint32_t warn_mask;                    /* 30  */
    uint32_t fault_active;                 /* 34  */
    uint32_t fault_mask;                   /* 38  */
    uint32_t fault_latch_mask;             /* 42  */
    uint32_t fault_latch_active;           /* 46  */
    uint8_t  temp_fault_sensor_mask[3];    /* 50..52 (24 bit, LE: source e' uint16_t) */
    uint8_t  last_fault_event_id;          /* 53  */
    uint8_t  last_error_event_id;          /* 54  */
    uint8_t  lq_error_code;                /* 55  */
    uint8_t  lq_ch_mask;                   /* 56  */
    uint8_t  lq_ilk;                       /* 57  */
    uint8_t  lq_opm;                       /* 58  */
    uint8_t  lq_opd;                       /* 59  */
    uint16_t lq_sp_cmd_ma;                 /* 60  */
    uint16_t lq_sp_echo_ma;                /* 62  */
    uint16_t lq_vanode_mv;                 /* 64  */
    uint16_t lq_i0_ma;                     /* 66  */
    uint16_t lq_i1_ma;                     /* 68  */
    uint16_t lq_i2_ma;                     /* 70  */
    uint16_t lq_i3_ma;                     /* 72  */
    uint8_t  lq_fsm_state;                 /* 74  */
    uint16_t mod_main_current_ma;          /* 75  */
    uint16_t mod_sab_current_ma;           /* 77  */
    uint16_t mode_com_current_ma;          /* 79  */
    uint16_t mode_laseq_current_ma;        /* 81  */
    int16_t  dew_mmc_td_c10;               /* 83  */
    int16_t  dew_mmc_margin_c10;           /* 85  */
    int16_t  dew_lq_td_c10;                /* 87  */
    int16_t  dew_lq_margin_c10;            /* 89  */
    int16_t  temp_sht35_mmc_c100;          /* 91  */
    uint8_t  hum_sht35_mmc_pct;            /* 93  */
    int16_t  ntc_temp_c10[16];             /* 94..125 (WATER_IN..NTC16) */
    uint8_t  lq_driver_temp_c[4];          /* 126..129 */
    int8_t   lq_ambient_temp_c;            /* 130 */
    uint8_t  lq_humidity_pct;              /* 131 */
    int16_t  flow1_lpm_x10;                /* 132 */
    int16_t  flow2_lpm_x10;                /* 134 */
    uint8_t  ntc_map_serigrafia[16];       /* 136..151 */
    uint16_t ntc_active_mask;              /* 152 */
    int16_t  thr_water_in_min_err_c;       /* 154 */
    int16_t  thr_water_in_min_warn_c;      /* 156 */
    int16_t  thr_water_in_max_warn_c;      /* 158 */
    int16_t  thr_water_in_max_err_c;       /* 160 */
    int16_t  thr_water_out_min_err_c;      /* 162 */
    int16_t  thr_water_out_min_warn_c;     /* 164 */
    int16_t  thr_water_out_max_warn_c;     /* 166 */
    int16_t  thr_water_out_max_err_c;      /* 168 */
    int16_t  thr_driver_min_err_c;         /* 170 */
    int16_t  thr_driver_min_warn_c;        /* 172 */
    int16_t  thr_driver_max_warn_c;        /* 174 */
    int16_t  thr_driver_max_err_c;         /* 176 */
    int16_t  thr_splice_min_err_c;         /* 178 */
    int16_t  thr_splice_min_warn_c;        /* 180 */
    int16_t  thr_splice_max_warn_c;        /* 182 */
    int16_t  thr_splice_max_err_c;         /* 184 */
    int16_t  thr_diode1_min_err_c;         /* 186 */
    int16_t  thr_diode1_min_warn_c;        /* 188 */
    int16_t  thr_diode1_max_warn_c;        /* 190 */
    int16_t  thr_diode1_max_err_c;         /* 192 */
    int16_t  thr_diode2_min_err_c;         /* 194 */
    int16_t  thr_diode2_min_warn_c;        /* 196 */
    int16_t  thr_diode2_max_warn_c;        /* 198 */
    int16_t  thr_diode2_max_err_c;         /* 200 */
    int16_t  thr_ambient_min_err_c;        /* 202 */
    int16_t  thr_ambient_min_warn_c;       /* 204 */
    int16_t  thr_ambient_max_warn_c;       /* 206 */
    int16_t  thr_ambient_max_err_c;        /* 208 */
    int16_t  thr_lq_ambient_min_err_c;     /* 210 */
    int16_t  thr_lq_ambient_min_warn_c;    /* 212 */
    int16_t  thr_lq_ambient_max_warn_c;    /* 214 */
    int16_t  thr_lq_ambient_max_err_c;     /* 216 */
    uint16_t thr_flow_min_err_lpmx10;      /* 218 */
    uint16_t thr_flow_min_warn_lpmx10;     /* 220 */
    uint16_t thr_flow_max_warn_lpmx10;     /* 222 */
    uint16_t thr_flow_max_err_lpmx10;      /* 224 */
    uint8_t  thr_hum_mmc_max_warn_pct;     /* 226 */
    uint8_t  thr_hum_laseq_max_warn_pct;   /* 227 */
    uint8_t  thr_dew_warn_delta_c;         /* 228 */
    uint8_t  thr_dew_err_delta_c;          /* 229 */
    uint16_t cfg_dly_psu_ms;               /* 230 */
    uint16_t cfg_dly_sab_ms;               /* 232 */
    uint16_t cfg_dly_contactor_ms;         /* 234 */
    uint8_t  rtc_hour;                     /* 236 */
    uint8_t  rtc_min;                      /* 237 */
    uint8_t  rtc_sec;                      /* 238 */
    uint16_t rtc_year;                     /* 239 */
    uint8_t  rtc_month;                    /* 241 */
    uint8_t  rtc_day;                      /* 242 */
} COM_StatusTable_t;

/* Checkpoint principali (non ogni singolo campo, sarebbe ridondante per gli
 * array): bastano i punti di giunzione tra blocchi per catturare qualunque
 * disallineamento di padding/ordine rispetto a COM_StatusOpcode_t. */
_Static_assert(offsetof(COM_StatusTable_t, sys_state)               == COM_STAT_OP_SYS_STATE, "status_tbl");
_Static_assert(offsetof(COM_StatusTable_t, setpoint_cur_ma)         == COM_STAT_OP_SETPOINT_CUR_MA, "status_tbl");
_Static_assert(offsetof(COM_StatusTable_t, pd0_power_w)             == COM_STAT_OP_PD0_POWER_W, "status_tbl");
_Static_assert(offsetof(COM_StatusTable_t, err_active)              == COM_STAT_OP_ERR_ACTIVE, "status_tbl");
_Static_assert(offsetof(COM_StatusTable_t, fault_latch_active)      == COM_STAT_OP_FAULT_LATCH_ACTIVE, "status_tbl");
_Static_assert(offsetof(COM_StatusTable_t, temp_fault_sensor_mask)  == COM_STAT_OP_TEMP_FAULT_SENSOR_MASK, "status_tbl");
_Static_assert(offsetof(COM_StatusTable_t, last_fault_event_id)     == COM_STAT_OP_LAST_FAULT_EVENT_ID, "status_tbl");
_Static_assert(offsetof(COM_StatusTable_t, lq_error_code)           == COM_STAT_OP_LQ_ERROR_CODE, "status_tbl");
_Static_assert(offsetof(COM_StatusTable_t, lq_sp_cmd_ma)            == COM_STAT_OP_LQ_SP_CMD_MA, "status_tbl");
_Static_assert(offsetof(COM_StatusTable_t, lq_fsm_state)            == COM_STAT_OP_LQ_FSM_STATE, "status_tbl");
_Static_assert(offsetof(COM_StatusTable_t, mod_main_current_ma)     == COM_STAT_OP_MOD_MAIN_CURRENT_MA, "status_tbl");
_Static_assert(offsetof(COM_StatusTable_t, dew_mmc_td_c10)          == COM_STAT_OP_DEW_MMC_TD_C10, "status_tbl");
_Static_assert(offsetof(COM_StatusTable_t, temp_sht35_mmc_c100)     == COM_STAT_OP_TEMP_SHT35_MMC_C100, "status_tbl");
_Static_assert(offsetof(COM_StatusTable_t, hum_sht35_mmc_pct)       == COM_STAT_OP_HUM_SHT35_MMC_PCT, "status_tbl");
_Static_assert(offsetof(COM_StatusTable_t, ntc_temp_c10)            == COM_STAT_OP_NTC_TEMP_C10_WATER_IN, "status_tbl");
_Static_assert(offsetof(COM_StatusTable_t, lq_driver_temp_c)        == COM_STAT_OP_LQ_DRIVER_TEMP_C_0, "status_tbl");
_Static_assert(offsetof(COM_StatusTable_t, lq_ambient_temp_c)       == COM_STAT_OP_LQ_AMBIENT_TEMP_C, "status_tbl");
_Static_assert(offsetof(COM_StatusTable_t, lq_humidity_pct)         == COM_STAT_OP_LQ_HUMIDITY_PCT, "status_tbl");
_Static_assert(offsetof(COM_StatusTable_t, flow1_lpm_x10)           == COM_STAT_OP_FLOW1_LPM_X10, "status_tbl");
_Static_assert(offsetof(COM_StatusTable_t, ntc_map_serigrafia)      == COM_STAT_OP_NTC_MAP_SERIGRAFIA_01, "status_tbl");
_Static_assert(offsetof(COM_StatusTable_t, ntc_active_mask)         == COM_STAT_OP_NTC_ACTIVE_MASK, "status_tbl");
_Static_assert(offsetof(COM_StatusTable_t, thr_water_in_min_err_c)  == COM_STAT_OP_THR_WATER_IN_MIN_ERR_C, "status_tbl");
_Static_assert(offsetof(COM_StatusTable_t, thr_lq_ambient_max_err_c) == COM_STAT_OP_THR_LQ_AMBIENT_MAX_ERR_C, "status_tbl");
_Static_assert(offsetof(COM_StatusTable_t, thr_flow_min_err_lpmx10) == COM_STAT_OP_THR_FLOW_MIN_ERR_LPMX10, "status_tbl");
_Static_assert(offsetof(COM_StatusTable_t, thr_hum_mmc_max_warn_pct) == COM_STAT_OP_THR_HUM_MMC_MAX_WARN_PCT, "status_tbl");
_Static_assert(offsetof(COM_StatusTable_t, thr_dew_warn_delta_c)    == COM_STAT_OP_THR_DEW_WARN_DELTA_C, "status_tbl");
_Static_assert(offsetof(COM_StatusTable_t, cfg_dly_psu_ms)          == COM_STAT_OP_CFG_DLY_PSU_MS, "status_tbl");
_Static_assert(offsetof(COM_StatusTable_t, rtc_hour)                == COM_STAT_OP_RTC_HOUR, "status_tbl");
_Static_assert(offsetof(COM_StatusTable_t, rtc_year)                == COM_STAT_OP_RTC_YEAR, "status_tbl");
_Static_assert(offsetof(COM_StatusTable_t, rtc_day)                 == COM_STAT_OP_RTC_DAY, "status_tbl");
_Static_assert(sizeof(COM_StatusTable_t) == (COM_STAT_OP_RTC_DAY + 1U), "status_tbl_size");
_Static_assert(sizeof(COM_StatusTable_t) < (COM_PAYLOAD_SIZE - 1U), "status_tbl_fits_payload");

/* --- CONFIG: payload (ALITE_COM com.h, COM_config_payload_t) --------------- */
typedef struct __attribute__((packed)) {
    uint32_t error_mask;
    uint32_t warn_mask;
    uint32_t fault_mask;
    uint8_t  psu_mask;
    uint8_t  flow_mask;
    uint8_t  temp_mask;
    uint8_t  pd_mask;
    uint32_t psu_voltage;
    uint32_t psu_current;
    uint8_t  contactor_mask;
} COM_ConfigPayload_t;

_Static_assert(sizeof(COM_ConfigPayload_t) == 25U, "com_config_payload");

/* Frame "vuoto"/dummy usato SOLO per generare il clock nel primo passo dello
 * scambio (vedi COM_Interface_BeginRequest() in COM_interface.c): il
 * contenuto non viene mai interpretato dalla COM interface, che in quella
 * fase ha armato solo il proprio DMA in trasmissione (nessuna RX attiva) —
 * vedi banner "SEQUENZA DI SINCRONISMO" sopra. Alias su COM_MSG_STATUS per
 * restare nel set di codici validi, nessun significato applicativo. */
#define COM_MSG_EMPTY   COM_MSG_STATUS

/* ============================================================================
 * FRAME COMPLETO
 * ============================================================================ */
typedef struct __attribute__((packed)) {
    uint8_t start;
    uint8_t msg_type;
    uint8_t payload[COM_PAYLOAD_SIZE];
    uint8_t crc;
    uint8_t end;
} COM_Frame_t;

/* ============================================================================
 * VERIFICA DIMENSIONI A COMPILE TIME
 * ============================================================================ */
_Static_assert(sizeof(COM_Frame_t) == COM_FRAME_SIZE, "com_frame");

/* ============================================================================
 * CRC8 — stesso algoritmo/stessa struttura a due passi di COM_frame_crc()
 * in ALITE_COM (Core/Src/ALITE/com/com_protocol.c): poly 0x07, init 0x00,
 * calcolato su START..PAYLOAD (COM_OFF_CRC byte), poi esteso sul byte END
 * (che va quindi scritto PRIMA di calcolare il CRC, non dopo). Il CRC NON
 * copre se' stesso.
 * ============================================================================ */
static inline uint8_t COM_crc8_update(uint8_t crc, const uint8_t *data, uint16_t len)
{
    for (uint16_t i = 0U; i < len; i++) {
        crc ^= data[i];
        for (uint8_t b = 0U; b < 8U; b++) {
            crc = (crc & 0x80U)
                ? (uint8_t)((crc << 1U) ^ 0x07U)
                : (uint8_t)(crc << 1U);
        }
    }
    return crc;
}

static inline uint8_t COM_FrameCRC(const uint8_t *frame)
{
    uint8_t crc = COM_crc8_update(0x00U, frame, COM_OFF_CRC);       /* START..payload */
    crc = COM_crc8_update(crc, &frame[COM_OFF_END], 1U);             /* + END byte */
    return crc;
}

static inline void COM_FrameFinalize(uint8_t *buf)
{
    buf[COM_OFF_END] = COM_END_BYTE;   /* scritto PRIMA del CRC: il CRC lo copre */
    buf[COM_OFF_CRC] = COM_FrameCRC(buf);
}

static inline int COM_FrameValidate(const uint8_t *buf)
{
    if (buf[COM_OFF_START] != COM_START_BYTE) return 0;
    if (buf[COM_OFF_END]   != COM_END_BYTE)   return 0;
    if (buf[COM_OFF_CRC]   != COM_FrameCRC(buf)) return 0;
    return 1;
}

#ifdef __cplusplus
}
#endif

#endif /* DRIVERS_COM_INTERFACE_COM_INTERFACE_PROTOCOL_H_ */
