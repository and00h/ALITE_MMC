/*
 * AMC_protocol.h
 *
 * ============================================================================
 * PROTOCOLLO DI COMUNICAZIONE UART - MMC (master) <-> AMC (slave)
 * ============================================================================
 *
 * File condiviso tra MMC (STM32H723) e AMC (STM32G473).
 * Non dipende da header HAL, FreeRTOS o applicativi: solo stdint.h.
 *
 * STRUTTURA FRAME (lunghezza fissa: AMC_FRAME_SIZE = 21 byte):
 *   Byte  0      : START_BYTE   (0xA5)
 *   Byte  1      : MSG_TYPE     (AMC_MsgType_t)
 *   Byte  2..17  : PAYLOAD      (AMC_PAYLOAD_SIZE = 16 byte)
 *   Byte  18..19 : CRC16 CCITT (poly 0x1021, init 0xFFFF, byte 0..17)
 *   Byte  20     : STOP_BYTE    (0x5A)
 *
 * SEQUENZA DI AVVIO (MMC → AMC):
 *   1. MSG_CONFIG_SET           → CONFIG_ACK
 *   2. MSG_CONFIG_PD            → CONFIG_PD_ACK
 *   3. MSG_CONFIG_GAIN_LUT x2   → CONFIG_GAIN_LUT_ACK  (mode=SW poi mode=HW)
 *   4. MSG_CONFIG_PD_VALID x N  → CONFIG_PD_VALID_ACK  (4 PD x M entry x 2 modi)
 *   5. MSG_CONFIG_VOLTAGE_LUT x N -> CONFIG_VOLTAGE_LUT_ACK (N entry, ordine crescente di current_ma)
 *   6. Loop: MSG_HEARTBEAT      → MSG_STATUS (ogni AMC_HEARTBEAT_PERIOD_MS)
 *
 * VERSIONE HISTORY:
 *   0x0001 - schema originale (CONFIG_SET, HEARTBEAT, STATUS)
 *   0x0002 - aggiunta CONFIG_PD, CONFIG_LUT, estensione STATUS
 *   0x0003 - HEARTBEAT + current_setpoint_ma; STATUS + pd_raw[4];
 *            CONFIG_GAIN_LUT (soglie gain mode-aware);
 *            CONFIG_PD_VALID (LUT validazione con interpolazione lineare);
 *            CONFIG_PD semplificato (rimossa gain_threshold)
 *   0x0004 - (2026-07-22) HEARTBEAT + qcw_active + setpoint_hw_enabled
 *            (2 byte, reserved 8->6);
 *            CONFIG_SET + voltage_comp_enabled/comp_stabilize_delay_ms/
 *            comp_ramp_duration_ms/comp_ramp_up_duration_ms (4 byte,
 *            reserved 6->2), per la compensazione dinamica tensione PSU;
 *            nuovo CONFIG_VOLTAGE_LUT/_ACK (LUT mA->mV, una entry/msg,
 *            stesso schema di CONFIG_PD_VALID)
 *   0x0005 - (2026-07-27) HEARTBEAT + voltage_restart_request (1 byte,
 *            reserved 6->5). Flag one-shot (MMC lo alza per UN heartbeat,
 *            poi lo riabbassa da solo): richiede ad AMC di trattare il
 *            PROSSIMO aumento di current_setpoint_ma come se il laser
 *            venisse riacceso da zero ai fini della compensazione dinamica
 *            di tensione PSU, cioe' di rieseguire la stessa sequenza
 *            "Vmax -> stabilizza -> rampa in discesa verso la tensione
 *            compensata" gia' usata alla vera accensione (psu_voltage_comp.c
 *            lato AMC), invece di interpolare direttamente verso la
 *            tensione target del nuovo setpoint. Necessario perche' un
 *            aumento di corrente a laser gia' acceso produce uno spunto
 *            (drop di tensione) che va assorbito con lo stesso margine di
 *            Vmax riservato all'accensione, altrimenti la tensione e' gia'
 *            al valore basso compensato per il VECCHIO setpoint quando
 *            arriva la richiesta di corrente piu' alta. Lato MMC: vedi
 *            AMC_RequestVoltageRestart() (AMC.h/.c) e task_comms.c
 *            (VCOMP_SETPOINT_HOLD_MS, alzato a 2000ms lo stesso giorno per
 *            coprire l'assestamento elettrico reale, circa 1.5s) — il
 *            nuovo current_setpoint_ma viene comunque inviato ad AMC
 *            SUBITO (AMC_SetCurrentSetpoint(), non gated), cosi' che AMC
 *            possa iniziare a puntare alla tensione compensata del NUOVO
 *            target non appena la sua rampa in discesa si avvia; e' verso
 *            LaseQ (non verso AMC) che l'inoltro del nuovo valore resta
 *            trattenuto per VCOMP_SETPOINT_HOLD_MS.
 *            IMPORTANTE per il firmware AMC (STM32G473, fuori da questo
 *            repo): comp_stabilize_delay_ms/comp_ramp_duration_ms restano
 *            pensati per il debounce alla vera accensione (10-50ms +
 *            50-100ms, complessivamente molto piu' brevi di 2s) — se AMC
 *            riusa questi stessi tempi anche per la sequenza di "riavvio a
 *            caldo" innescata da voltage_restart_request, rischia di
 *            ripiombare sulla tensione compensata (bassa, per il VECCHIO
 *            setpoint) ben PRIMA che MMC inoltri realmente il nuovo
 *            setpoint a LaseQ (che arriva solo dopo i 2s): va verificato/
 *            allineato lato AMC, non deducibile da questo repo.
 *   0x0006 - (2026-09-04) AMC_PayloadConfigAck_t + fw_version[10] (char,
 *            ASCII NUL-terminated, reserved 12->2): permette a MMC di
 *            leggere la versione del firmware applicativo REALMENTE in
 *            esecuzione su AMC (stesso schema di FW_VERSION_STR lato MMC,
 *            vedi App/Config/fw_version.h) senza un canale separato.
 *            Popolato da AMC in handle_config_set() (App/Comms/
 *            uart_comms.c) su OGNI risposta CONFIG_ACK (accepted o meno),
 *            cosi' la versione resta leggibile anche quando la config
 *            viene rifiutata. Letto lato MMC in AMC_SendConfig() (AMC.c),
 *            esposto da AMC_GetFwVersionStr() (AMC.h), riportato dal
 *            comando RS485 "GET FW" (rs485_cmd.c). Un AMC con firmware PIU'
 *            VECCHIO (che non popola ancora questo campo) risponde con
 *            fw_version a zero: MMC tratta una stringa vuota come
 *            "sconosciuta" - nessun impatto sulla logica esistente (il
 *            controllo di versione resta su protocol_version, invariato).
 * ============================================================================
 */

#ifndef AMC_PROTOCOL_H_
#define AMC_PROTOCOL_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* ============================================================================
 * VERSIONE PROTOCOLLO
 * ============================================================================ */
#define AMC_PROTOCOL_VERSION        0x0006U

/* ============================================================================
 * COSTANTI FRAME
 * ============================================================================ */
#define AMC_START_BYTE              0xA5U
#define AMC_STOP_BYTE               0x5AU
#define AMC_PAYLOAD_SIZE            16U
#define AMC_FRAME_SIZE              (1U + 1U + AMC_PAYLOAD_SIZE + 2U + 1U)  /* = 21 */

#define AMC_OFF_START               0U
#define AMC_OFF_MSG_TYPE            1U
#define AMC_OFF_PAYLOAD             2U
#define AMC_OFF_CRC_H               (AMC_OFF_PAYLOAD + AMC_PAYLOAD_SIZE)
#define AMC_OFF_CRC_L               (AMC_OFF_CRC_H + 1U)
#define AMC_OFF_STOP                (AMC_OFF_CRC_L + 1U)

/* ============================================================================
 * TIMING
 *
 * AMC_HEARTBEAT_PERIOD_MS (2026-07-22): ridotto da 200 a 20ms. Fino a questa
 * data l'heartbeat MMC->AMC era servito ogni 10 cicli LaseQ (AMC_LOOP_DIVISOR,
 * task_comms.c) da un contatore software. Il gating del setpoint ascendente
 * verso LaseQ (vedi task_comms.c/task_amc.c, rampa 1-5ms su
 * comp_ramp_up_duration_ms) richiede che AMC sia raggiungibile con una
 * cadenza comparabile a LaseQ: il servicing AMC ora vive in un task
 * dedicato (task_amc.c) con periodo = LQ_TRANSMIT_WINDOW (20ms), invece del
 * vecchio meccanismo a divisore. AMC_MAX_MISS_COUNT scalato di conseguenza
 * (5 -> 50) per mantenere invariata la finestra di ~1s prima del fault
 * comunicazione (5 x 200ms = 50 x 20ms = 1000ms).
 * ============================================================================ */
#define AMC_HEARTBEAT_PERIOD_MS     20U
#define AMC_RESPONSE_TIMEOUT_MS     100U
#define AMC_MAX_MISS_COUNT          50U

/* ============================================================================
 * TIPI DI MESSAGGIO
 * ============================================================================ */
typedef enum {
    /* Ciclo operativo */
    AMC_MSG_HEARTBEAT            = 0x10U,
    AMC_MSG_STATUS               = 0x11U,

    /* Configurazione PSU (una tantum) */
    AMC_MSG_CONFIG_SET           = 0x20U,
    AMC_MSG_CONFIG_ACK           = 0x21U,

    /* Configurazione fotodiodi */
    AMC_MSG_CONFIG_PD            = 0x22U,
    AMC_MSG_CONFIG_PD_ACK        = 0x23U,

    /* LUT soglie selezione finestra guadagno MUX (SW e HW mode separati) */
    AMC_MSG_CONFIG_GAIN_LUT      = 0x24U,
    AMC_MSG_CONFIG_GAIN_LUT_ACK  = 0x25U,

    /* LUT validazione fotodiodi con interpolazione lineare (una entry/msg) */
    AMC_MSG_CONFIG_PD_VALID      = 0x26U,
    AMC_MSG_CONFIG_PD_VALID_ACK  = 0x27U,

    /* LUT compensazione tensione PSU, mA->mV (una entry/msg, NUOVO 2026-07-22) */
    AMC_MSG_CONFIG_VOLTAGE_LUT     = 0x28U,
    AMC_MSG_CONFIG_VOLTAGE_LUT_ACK = 0x29U,
} AMC_MsgType_t;

/* ============================================================================
 * PAYLOAD: AMC_MSG_HEARTBEAT  (MMC -> AMC, 16 byte)
 *
 * current_setpoint_ma: corrente comandata a LaseQ in modalita' SW [mA].
 *   In modalita' HW/ANALOG vale 0 (AMC usa PA0 per la selezione LUT).
 *   AMC usa questo campo per selezionare la finestra gain e determinare
 *   il range atteso dei fotodiodi in modalita' SW.
 * qcw_active (NUOVO 2026-07-22): 1 se la modalita' QCW e' correntemente
 *   attiva lato MMC (g_config.qcw_enabled ed emissione SW in corso), 0
 *   altrimenti. AMC non ha altro modo di saperlo (QCW e' un gate fisico
 *   pilotato direttamente da MMC, nGATE_MC, mai passato via RS485 prima
 *   d'ora): serve alla compensazione dinamica tensione PSU
 *   (psu_voltage_comp.c) per restare disattiva durante QCW, dove
 *   LASE_Q_OUT_5V si accende/spegne al ritmo dell'impulsazione (fino a
 *   50kHz) e andrebbe interpretato in modo errato come accensioni/
 *   spegnimenti del laser.
 * setpoint_hw_enabled (NUOVO 2026-07-22): 1 se il toggle "SETPOINT HW"
 *   lato MMC e' attivo (FSM_GetSetpointHwEnabled(), detto anche "HYBRID2":
 *   in FSM_MODE_SW instrada il riferimento di corrente per LaseQ dal pin
 *   analogico esterno invece che da RS485/SW), 0 altrimenti. NON e' lo
 *   stesso valore di hw_setpoint_sel sopra: quest'ultimo e' un instradamento
 *   hardware statico (quale sorgente pilota il DAC di AMC, praticamente
 *   sempre 0 finche' l'AMC DAC non e' implementato), mentre
 *   setpoint_hw_enabled e' lo stato LIVE del toggle RS485 "SET SETPOINTHW
 *   ON|OFF". Serve alla compensazione dinamica tensione PSU
 *   (psu_voltage_comp.c) per restare disattiva quando il riferimento di
 *   corrente e' instradato dal pin analogico: i cambi li' sono istantanei/
 *   imprevedibili, non gestibili con una rampa temporizzata.
 * voltage_restart_request (NUOVO 2026-07-27): vedi banner "VERSIONE HISTORY"
 *   sopra, voce 0x0005. Flag one-shot: 1 SOLO nell'heartbeat immediatamente
 *   successivo a un aumento del setpoint di corrente con compensazione
 *   attiva, poi torna a 0 da solo (vedi AMC_RequestVoltageRestart(), AMC.c).
 * ============================================================================ */
typedef struct __attribute__((packed)) {
    uint16_t    seq;                    /*  2: numero sequenza */
    uint8_t     laser_mode;             /*  1: FSM_Mode_t cast (0=SW,1=HYBRID,2=ANALOG) */
    uint8_t     hw_setpoint_sel;        /*  1: 0=PA0 grezzo, 1=DAC AMC */
    uint16_t    protocol_version;       /*  2: AMC_PROTOCOL_VERSION */
    uint16_t    current_setpoint_ma;    /*  2: corrente LaseQ in modalita' SW [mA] */
    uint8_t     qcw_active;             /*  1: 1=QCW attivo, 0=CW/non-SW (NUOVO 2026-07-22) */
    uint8_t     setpoint_hw_enabled;    /*  1: 1=toggle "SETPOINT HW" attivo (NUOVO 2026-07-22) */
    uint8_t     voltage_restart_request;/*  1: 1=tratta il prossimo aumento di corrente come una
                                          *     riaccensione ai fini VCOMP (NUOVO 2026-07-27) */
    uint8_t     reserved[5];            /*  5: riserva (era 6 prima dell'aggiunta di
                                          *     voltage_restart_request) */
                                        /* tot: 16 */
} AMC_PayloadHeartbeat_t;

/* ============================================================================
 * PAYLOAD: AMC_MSG_STATUS  (AMC -> MMC, 16 byte)
 *
 * pd_raw[4]: letture ADC dei 4 fotodiodi (12 bit, 0-4095).
 *   Indice 0..3 = PD1..PD4. PD non attivi restituiscono 0.
 *   La conversione ADC->Watt avviene su MMC tramite la propria LUT.
 * ============================================================================ */
typedef struct __attribute__((packed)) {
    uint16_t    seq;                    /*  2: eco seq ricevuto */
    uint8_t     amc_state;              /*  1: AMC_State_t */
    uint8_t     error_code;             /*  1: AMC_ErrorCode_t */
    uint8_t     gain_window;            /*  1: finestra guadagno corrente (0-3) */
    uint8_t     pd_status_mask;         /*  1: bit[0..3]=PD[0..3] valid; bit4=stable; bit5=laser_on;
                                          *     bit6=pd_stale (NUOVO 2026-07-28, vedi
                                          *     Photodiode_IsStale()/PD_STALE_TIMEOUT_MS in
                                          *     photodiode.h lato AMC: 1=i valori pd_raw[] non
                                          *     sono stati aggiornati da oltre 500ms, es. per uno
                                          *     stallo dell'acquisizione ADC1 — rete di sicurezza
                                          *     aggiunta dopo il bugfix DMAContinuousRequests in
                                          *     MX_ADC1_Init/MX_ADC2_Init, main.c lato AMC. Lato
                                          *     MMC: AMC_IsPDDataStale(), colonna PD_STALE in
                                          *     "LOG ON" — rs485_cmd.c) */
    uint16_t    hw_setpoint_raw;        /*  2: ADC2 PA0 [0-4095] */
    uint16_t    pd_raw[4];              /*  8: letture ADC fotodiodi [0-4095] */
                                        /* tot: 16 */
} AMC_PayloadStatus_t;

/* ============================================================================
 * PAYLOAD: AMC_MSG_CONFIG_SET  (MMC -> AMC, 16 byte)
 *
 * Campi di compensazione tensione PSU (NUOVO 2026-07-22, ricavati dai 6 byte
 * reserved rimasti 2): psu_voltage_limit_mv sopra resta il valore MASSIMO
 * (Vmax, "SET PSU VOLTAGE") a cui la tensione torna sempre quando la
 * compensazione non è attiva o il laser è spento — vedi psu_voltage_comp.h.
 * ============================================================================ */
typedef struct __attribute__((packed)) {
    uint16_t    psu_voltage_limit_mv;   /*  2: tensione massima PSU [mV] (Vmax) */
    uint16_t    psu_current_limit_ma;   /*  2: corrente massima PSU [mA] */
    uint16_t    protocol_version;       /*  2: versione attesa */
    uint16_t    hw_sp_scale_num;        /*  2: numeratore scaling HW setpoint */
    uint16_t    hw_sp_scale_den;        /*  2: denominatore scaling HW setpoint */
    uint8_t     voltage_comp_enabled;   /*  1: 1=compensazione attiva (NUOVO 2026-07-22) */
    uint8_t     comp_stabilize_delay_ms;/*  1: debounce dopo accensione laser [10-50ms] */
    uint8_t     comp_ramp_duration_ms;  /*  1: durata rampa lenta in discesa [50-100ms] */
    uint8_t     comp_ramp_up_duration_ms; /* 1: durata rampa veloce in salita [1-5ms] */
    uint8_t     reserved[2];            /*  2 (era 6 prima del 2026-07-22) */
                                        /* tot: 16 */
} AMC_PayloadConfigSet_t;

/* ============================================================================
 * PAYLOAD: AMC_MSG_CONFIG_ACK  (AMC -> MMC, 16 byte)
 * ============================================================================ */
typedef struct __attribute__((packed)) {
    uint8_t     accepted;               /*  1: 1=OK */
    uint8_t     error_code;             /*  1 */
    uint16_t    protocol_version;       /*  2: versione AMC */
    char        fw_version[10];         /* 10: FW_VERSION_STR di AMC, ASCII
                                          *     NUL-terminated, troncata a 9
                                          *     char + NUL (NUOVO 0x0006,
                                          *     2026-09-04 - "GET FW" lato
                                          *     MMC) */
    uint8_t     reserved[2];            /*  2 (era 12 prima di 0x0006) */
                                        /* tot: 16 */
} AMC_PayloadConfigAck_t;

/* ============================================================================
 * PAYLOAD: AMC_MSG_CONFIG_PD  (MMC -> AMC, 16 byte)
 *
 * v0.0003: le soglie gain sono rimosse da qui, arrivano via CONFIG_GAIN_LUT.
 * ============================================================================ */
typedef struct __attribute__((packed)) {
    uint8_t     pd_mask;                /*  1: bitmask PD attivi (bit0=PD1..bit3=PD4) */
    uint8_t     gain_windows;           /*  1: numero finestre guadagno (normalmente 4) */
    uint16_t    gain_settle_ms;         /*  2: OTA settling time [ms] */
    uint8_t     stability_samples;      /*  1: campioni per analisi stabilita' (2-32) */
    uint8_t     stability_threshold;    /*  1: soglia variazione ADC2 [raw counts] */
    uint8_t     reserved[10];           /* 10 */
                                        /* tot: 16 */
} AMC_PayloadConfigPD_t;

/* ============================================================================
 * PAYLOAD: AMC_MSG_CONFIG_PD_ACK  (AMC -> MMC, 16 byte)
 * ============================================================================ */
typedef struct __attribute__((packed)) {
    uint8_t     accepted;               /*  1 */
    uint8_t     error_code;             /*  1 */
    uint8_t     reserved[14];           /* 14 */
                                        /* tot: 16 */
} AMC_PayloadConfigPDAck_t;

/* ============================================================================
 * PAYLOAD: AMC_MSG_CONFIG_GAIN_LUT  (MMC -> AMC, 16 byte)
 *
 * Imposta le soglie per la selezione della finestra guadagno MUX.
 * Inviare due volte: mode=0 (SW) e mode=1 (HW).
 *
 * SW mode: confronta current_setpoint_ma (campo HEARTBEAT).
 * HW mode: confronta hw_setpoint_raw (ADC2 PA0).
 *
 * Logica finestra (4 finestre):
 *   val < threshold[0]  -> finestra 0  (guadagno piu' basso)
 *   val < threshold[1]  -> finestra 1
 *   val < threshold[2]  -> finestra 2
 *   val >= threshold[2] -> finestra 3  (guadagno piu' alto)
 *   threshold[3] = 0xFFFF (non usato come limite superiore)
 * ============================================================================ */
typedef struct __attribute__((packed)) {
    uint8_t     mode;                   /*  1: 0=SW (current_ma), 1=HW (pa0_adc) */
    uint8_t     num_windows;            /*  1: numero finestre (normalmente 4) */
    uint16_t    threshold[4];           /*  8: soglie di separazione [raw] */
    uint8_t     reserved[6];            /*  6 */
                                        /* tot: 16 */
} AMC_PayloadConfigGainLUT_t;

/* ============================================================================
 * PAYLOAD: AMC_MSG_CONFIG_GAIN_LUT_ACK  (AMC -> MMC, 16 byte)
 * ============================================================================ */
typedef struct __attribute__((packed)) {
    uint8_t     accepted;               /*  1 */
    uint8_t     error_code;             /*  1 */
    uint8_t     mode;                   /*  1: eco */
    uint8_t     reserved[13];           /* 13 */
                                        /* tot: 16 */
} AMC_PayloadConfigGainLUTAck_t;

/* ============================================================================
 * PAYLOAD: AMC_MSG_CONFIG_PD_VALID  (MMC -> AMC, 16 byte)
 *
 * Imposta una entry della LUT di validazione per un fotodiodo.
 * AMC interpola linearmente tra le entry per determinare il range
 * atteso al setpoint corrente.
 *
 * Le entry DEVONO arrivare in ordine crescente di setpoint.
 * Per setpoint fuori range AMC clampa alla entry piu' vicina.
 *
 * SW mode: setpoint = current_setpoint_ma (da HEARTBEAT)
 * HW mode: setpoint = hw_setpoint_raw (ADC2 PA0)
 *
 * Sequenza invio: pd_idx=0..3, mode=0..1, entry_idx=0..total_entries-1.
 * ============================================================================ */
typedef struct __attribute__((packed)) {
    uint8_t     pd_idx;                 /*  1: indice fotodiodo (0-3) */
    uint8_t     mode;                   /*  1: 0=SW, 1=HW */
    uint8_t     entry_idx;              /*  1: indice entry (0-based) */
    uint8_t     total_entries;          /*  1: totale entry per questo PD+mode */
    uint16_t    setpoint;               /*  2: valore setpoint [mA o raw] */
    uint16_t    pd_min;                 /*  2: ADC minimo atteso [0-4095] */
    uint16_t    pd_max;                 /*  2: ADC massimo atteso [0-4095] */
    uint8_t     reserved[6];            /*  6 */
                                        /* tot: 16 */
} AMC_PayloadConfigPDValid_t;

/* ============================================================================
 * PAYLOAD: AMC_MSG_CONFIG_PD_VALID_ACK  (AMC -> MMC, 16 byte)
 * ============================================================================ */
typedef struct __attribute__((packed)) {
    uint8_t     accepted;               /*  1 */
    uint8_t     error_code;             /*  1 */
    uint8_t     pd_idx;                 /*  1: eco */
    uint8_t     mode;                   /*  1: eco */
    uint8_t     entry_idx;              /*  1: eco */
    uint8_t     reserved[11];           /* 11 */
                                        /* tot: 16 */
} AMC_PayloadConfigPDValidAck_t;

/* ============================================================================
 * PAYLOAD: AMC_MSG_CONFIG_VOLTAGE_LUT  (MMC -> AMC, 16 byte, NUOVO 2026-07-22)
 *
 * Imposta una entry della LUT di compensazione tensione PSU (mA -> mV).
 * AMC interpola linearmente tra le entry per determinare la tensione target
 * al setpoint di corrente corrente — vedi psu_voltage_comp.h. Le entry
 * DEVONO arrivare in ordine crescente di current_ma. voltage_mv == 0 è una
 * entry non configurata: AMC ricade sul valore massimo psu_voltage_limit_mv
 * (CONFIG_SET) invece di scrivere 0V.
 *
 * Sequenza invio: entry_idx=0..total_entries-1 (stesso schema di
 * AMC_MSG_CONFIG_PD_VALID sopra, ma senza indice pd/modo: una sola LUT).
 * ============================================================================ */
typedef struct __attribute__((packed)) {
    uint8_t     entry_idx;              /*  1: indice entry (0-based) */
    uint8_t     total_entries;          /*  1: totale entry di questa LUT */
    uint16_t    current_ma;             /*  2: setpoint di corrente LaseQ [mA] */
    uint16_t    voltage_mv;             /*  2: tensione target PSU [mV], 0=non valida */
    uint8_t     reserved[10];           /* 10 */
                                        /* tot: 16 */
} AMC_PayloadConfigVoltageLUT_t;

/* ============================================================================
 * PAYLOAD: AMC_MSG_CONFIG_VOLTAGE_LUT_ACK  (AMC -> MMC, 16 byte, NUOVO 2026-07-22)
 * ============================================================================ */
typedef struct __attribute__((packed)) {
    uint8_t     accepted;               /*  1 */
    uint8_t     error_code;             /*  1 */
    uint8_t     entry_idx;              /*  1: eco */
    uint8_t     reserved[13];           /* 13 */
                                        /* tot: 16 */
} AMC_PayloadConfigVoltageLUTAck_t;

/* ============================================================================
 * STATI AMC
 * ============================================================================ */
typedef enum {
    AMC_STATE_INIT   = 0,   /* Attende CONFIG_SET */
    AMC_STATE_CFG    = 1,   /* CONFIG_SET ok, attende CONFIG_PD + LUT */
    AMC_STATE_ACTIVE = 2,   /* Operativo */
    AMC_STATE_ERROR  = 3,   /* Fault interno */
} AMC_State_t;

/* ============================================================================
 * CODICI ERRORE AMC
 * ============================================================================ */
typedef enum {
    AMC_EC_OK        = 0,
    AMC_EC_VERSION   = 1,
    AMC_EC_RANGE     = 2,
    AMC_EC_DAC_FAULT = 3,
    AMC_EC_ADC_FAULT = 4,
} AMC_ErrorCode_t;

/* ============================================================================
 * FRAME COMPLETO
 * ============================================================================ */
typedef struct __attribute__((packed)) {
    uint8_t  start;
    uint8_t  msg_type;
    union {
        uint8_t                        raw[AMC_PAYLOAD_SIZE];
        AMC_PayloadHeartbeat_t         heartbeat;
        AMC_PayloadStatus_t            status;
        AMC_PayloadConfigSet_t         config_set;
        AMC_PayloadConfigAck_t         config_ack;
        AMC_PayloadConfigPD_t          config_pd;
        AMC_PayloadConfigPDAck_t       config_pd_ack;
        AMC_PayloadConfigGainLUT_t     config_gain_lut;
        AMC_PayloadConfigGainLUTAck_t  config_gain_lut_ack;
        AMC_PayloadConfigPDValid_t     config_pd_valid;
        AMC_PayloadConfigPDValidAck_t  config_pd_valid_ack;
        AMC_PayloadConfigVoltageLUT_t     config_voltage_lut;
        AMC_PayloadConfigVoltageLUTAck_t  config_voltage_lut_ack;
    } payload;
    uint8_t  crc_h;
    uint8_t  crc_l;
    uint8_t  stop;
} AMC_Frame_t;

/* ============================================================================
 * VERIFICA DIMENSIONI A COMPILE TIME
 * ============================================================================ */
_Static_assert(sizeof(AMC_PayloadHeartbeat_t)        == AMC_PAYLOAD_SIZE, "heartbeat");
_Static_assert(sizeof(AMC_PayloadStatus_t)           == AMC_PAYLOAD_SIZE, "status");
_Static_assert(sizeof(AMC_PayloadConfigSet_t)        == AMC_PAYLOAD_SIZE, "config_set");
_Static_assert(sizeof(AMC_PayloadConfigAck_t)        == AMC_PAYLOAD_SIZE, "config_ack");
_Static_assert(sizeof(AMC_PayloadConfigPD_t)         == AMC_PAYLOAD_SIZE, "config_pd");
_Static_assert(sizeof(AMC_PayloadConfigPDAck_t)      == AMC_PAYLOAD_SIZE, "config_pd_ack");
_Static_assert(sizeof(AMC_PayloadConfigGainLUT_t)    == AMC_PAYLOAD_SIZE, "config_gain_lut");
_Static_assert(sizeof(AMC_PayloadConfigGainLUTAck_t) == AMC_PAYLOAD_SIZE, "config_gain_lut_ack");
_Static_assert(sizeof(AMC_PayloadConfigPDValid_t)    == AMC_PAYLOAD_SIZE, "config_pd_valid");
_Static_assert(sizeof(AMC_PayloadConfigPDValidAck_t) == AMC_PAYLOAD_SIZE, "config_pd_valid_ack");
_Static_assert(sizeof(AMC_PayloadConfigVoltageLUT_t)    == AMC_PAYLOAD_SIZE, "config_voltage_lut");
_Static_assert(sizeof(AMC_PayloadConfigVoltageLUTAck_t) == AMC_PAYLOAD_SIZE, "config_voltage_lut_ack");
_Static_assert(sizeof(AMC_Frame_t)                   == AMC_FRAME_SIZE,   "frame");

/* ============================================================================
 * CRC16 CCITT (poly 0x1021, init 0xFFFF)
 * ============================================================================ */
static inline uint16_t AMC_CRC16(const uint8_t *data, uint16_t len)
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

static inline void AMC_FrameFinalize(uint8_t *buf)
{
    uint16_t crc = AMC_CRC16(buf, AMC_OFF_CRC_H);
    buf[AMC_OFF_CRC_H] = (uint8_t)(crc >> 8U);
    buf[AMC_OFF_CRC_L] = (uint8_t)(crc & 0xFFU);
    buf[AMC_OFF_STOP]  = AMC_STOP_BYTE;
}

static inline int AMC_FrameValidate(const uint8_t *buf)
{
    if (buf[AMC_OFF_START] != AMC_START_BYTE) return 0;
    if (buf[AMC_OFF_STOP]  != AMC_STOP_BYTE)  return 0;
    uint16_t calc = AMC_CRC16(buf, AMC_OFF_CRC_H);
    uint16_t rx   = ((uint16_t)buf[AMC_OFF_CRC_H] << 8U) | buf[AMC_OFF_CRC_L];
    return (calc == rx) ? 1 : 0;
}

#ifdef __cplusplus
}
#endif

#endif /* AMC_PROTOCOL_H_ */
