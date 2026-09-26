/*
 * config.h
 *
 * Modulo di configurazione persistente del modulo laser MMC.
 *
 * LAYOUT Config_t (320 byte = 8 × 40, con #pragma pack(push,4)):
 *
 *   off   0: RS485(4)
 *   off   4: PSU(8)
 *   off  12: LaseQ(4) [laser_mode+pad]
 *   off  16: Temps 6×4×int16_t = 48 byte
 *   off  64: Flow 4×uint16_t = 8 byte
 *   off  72: OpParams: interlock(4)+lid(4)+delay(2)+reduced(1)+pad(1) = 12
 *   off  84: fw_version(4)
 *   off  88: Sequence: psu_delay(2)+sab_timeout(2) = 4
 *   off  92: Devices: sab(1)+psu_mask(1)+contactor_mask(1) = 3
 *   off  95: Ambient: humidity_mmc(1)+humidity_laseq(1)+_amb_pad[3] = 5 → off 100
 *   off 100: Masks: warning(4)+error(4) = 8
 *   off 108: HwSetpoint: sel(1)+_hw_sp_pad[3] = 4
 *   off 112: NTC: map[16]+r0(4)+beta[16](32)+rseries(2)+_ntc_pad[2] = 56
 *            (2026-07-22: ntc_beta da scalare globale ad array per canale
 *            AD7490, per poter associare un coefficiente Beta diverso a
 *            ciascun NTC — indicizzato come ntc_ch_map, non per serigrafia)
 *   off 168: SetpointLUT: 21×uint16_t = 42 byte
 *   off 210: Photodiode: pd_mask(1)+gain_windows(1)+gain_threshold[4]×2=8+gain_settle_ms(2)+stab_samples(1)+stab_thresh(1) = 14
 *   off 224: FaultMask: fault_mask(4)                                         = 4
 *   off 228: DeviceMasks2: flow_sensor_enabled_mask(1)+_dev2_pad0(1)+temp_sensor_enabled_mask(2, uint16_t) = 4
 *            (2026-07-22: temp_sensor_enabled_mask da uint8_t a uint16_t,
 *            servivano più bit per PSU_TEMP/PWR_EL_TEMP)
 *   off 232: QCW: qcw_freq_hz(2)+qcw_duty_pct(1)+qcw_enabled(1)                = 4
 *   off 236: efuse_enabled_mask(4)                                            = 4
 *   off 240: FaultLatch: fault_latch_mask(4)+fault_latch_active(4)           = 8
 *   off 248: Temp diodo2: min_err(2)+min_warn(2)+max_warn(2)+max_err(2)       = 8
 *   off 256: Temp LQ ambient: min_err(2)+min_warn(2)+max_warn(2)+max_err(2)   = 8
 *   off 264: Temp PSU_TEMP (NUOVO 2026-07-22): min_err/min_warn/max_warn/max_err = 8
 *   off 272: Temp PWR_EL_TEMP (NUOVO 2026-07-22): min_err/min_warn/max_warn/max_err = 8
 *   off 280: Temp MB_TEMP (NUOVO 2026-07-22, SHT35-MMC): min_err/min_warn/max_warn/max_err = 8
 *   off 288: Voltage compensation (NUOVO 2026-07-22):
 *            voltage_comp_enabled(1)+comp_stabilize_delay_ms(1)+
 *            comp_ramp_duration_ms(1)+comp_ramp_up_duration_ms(1) = 4
 *   off 292: reserved[28] (espansione futura)                                = 28
 *   = 320 byte ✓ (8 × 40) — 28 byte liberi in _reserved
 */

#ifndef APP_CONFIG_CONFIG_H_
#define APP_CONFIG_CONFIG_H_

#include <stdint.h>
#include <stdbool.h>
#include "flash_map.h"
#include <assert.h>
#include "stm32h7xx_hal.h"

/* ========================================================================== */
/* --- VERSIONE DELLO SCHEMA ---
 *
 * RESET 2026-07-15: il contatore CONFIG_MAGIC è stato riportato a 0xA55A0001
 * (era arrivato a 0xA55A0009 dopo 8 evoluzioni incrementali dello schema —
 * fault_mask, efuse_enabled_mask, ridesign warning/error/fault mask, DIODE2,
 * soglie ambiente LaseQ dedicate, split SAB_INTLCK/TEST per canale A/B).
 * Il reset è stato deciso ED ESEGUITO insieme a un ERASE COMPLETO del
 * settore flash del journal Config (settore 7, vedi flash_map.h) su tutte
 * le unità che ricevono questo firmware, per non doversi portare dietro la
 * numerazione incrementale pregressa.
 *
 * ATTENZIONE — QUESTO VALORE ERA GIÀ STATO USATO in passato per lo schema
 * ORIGINALE (160 byte, molto più piccolo dell'attuale Config_t da 256
 * byte). Config_Load() considera valido QUALSIASI record il cui campo
 * magic combaci con CONFIG_MAGIC corrente (vedi sotto), indipendentemente
 * da quando è stato scritto. Riutilizzare 0xA55A0001 è sicuro SOLO perché
 * l'erase di settore garantisce che nessun record — vecchio o nuovo — sia
 * rimasto in flash con quel magic. Se una qualsiasi unità dovesse ricevere
 * questo firmware SENZA l'erase di settore (es. aggiornamento OTA/JTAG che
 * salta il passaggio manuale), un eventuale record residuo dello schema
 * originale a 160 byte con lo stesso magic 0xA55A0001 verrebbe accettato
 * come valido e copiato in g_config con memcpy(..., sizeof(Config_t)) —
 * cioè 256 byte letti da un record che ne conteneva realmente solo 160,
 * interpretando byte di flash NON scritti da quel vecchio schema (quindi
 * di contenuto indeterminato) come se fossero soglie/maschere/mask attuali
 * valide. Verificare SEMPRE l'erase prima di aggiornare un'unità con
 * questo firmware.
 *
 * D'ora in poi riprendere l'incremento normale (prossima evoluzione dello
 * schema → 0xA55A0002, ecc.): NON riutilizzare 0xA55A0001 una seconda
 * volta a meno di un nuovo erase di settore altrettanto esplicito e
 * verificato su tutte le unità.
 *
 * EVOLUZIONE 2026-07-22 (0xA55A0001 → 0xA55A0002): Config_t cresce da 256
 * a 320 byte (ntc_beta da scalare a array[16], nuove soglie PSU_TEMP/
 * PWR_EL_TEMP/MB_TEMP, nuovi campi compensazione tensione PSU, mask
 * temp_sensor_enabled_mask da 8 a 16 bit). Come sopra: ERASE COMPLETO del
 * settore 7 richiesto su tutte le unità in campo prima di questo firmware.
 */
/* ========================================================================== */

#define CONFIG_MAGIC   0xA55A0002UL

/* ========================================================================== */
/* --- LUT SETPOINT --- */
/* ========================================================================== */

/*
 * La LUT mappa una percentuale di setpoint (0-100%) a un valore di corrente [mA]
 * per LaseQ. È definita con gradini da 5%, quindi 21 entry (0, 5, 10, ..., 100%).
 * L'interpolazione lineare a tratti fornisce valori intermedi.
 *
 * Default fabbrica: retta 0W/0mA @ 0% → 550W/10500mA @ 100%.
 * Modificabile in service mode via Config_SetLUTEntry() / Config_SetLUT().
 */
#define SETPOINT_LUT_SIZE      21U    /**< Numero di entry: 0%, 5%, ..., 100%  */
#define SETPOINT_LUT_STEP_PCT   5U    /**< Passo LUT [punti percentuali]       */
#define SETPOINT_MAX_PCT      100U    /**< Valore massimo setpoint [%]         */

/* ========================================================================== */
/* --- PAYLOAD: PARAMETRI DI MACCHINA --- */
/* ========================================================================== */

#pragma pack(push, 4)
typedef struct {

    /* --- RS485 (offset 0, 4 byte) --- */
    uint8_t     slave_address;
    uint8_t     termination;
    uint8_t     _rs485_pad[2];

    /* --- PSU (offset 4, 8 byte) --- */
    int32_t     psu_voltage_mv;
    int32_t     psu_current_ma;

    /* --- LaseQ (offset 12, 4 byte) --- */
    uint8_t     laser_mode;             /* LaseQ_mode_t: 0=SW, 1=ANALOG, 2=HYBRID */
    uint8_t     _laser_pad[3];

    /* --- Temperature [°C con segno] (offset 16, 48 byte) --- */
    int16_t     temp_water_in_min_err;
    int16_t     temp_water_in_min_warn;
    int16_t     temp_water_in_max_err;
    int16_t     temp_water_in_max_warn;

    int16_t     temp_water_out_min_err;
    int16_t     temp_water_out_min_warn;
    int16_t     temp_water_out_max_err;
    int16_t     temp_water_out_max_warn;

    int16_t     temp_driver_min_err;
    int16_t     temp_driver_min_warn;
    int16_t     temp_driver_max_err;
    int16_t     temp_driver_max_warn;

    int16_t     temp_splice_min_err;
    int16_t     temp_splice_min_warn;
    int16_t     temp_splice_max_err;
    int16_t     temp_splice_max_warn;

    int16_t     temp_diode_min_err;
    int16_t     temp_diode_min_warn;
    int16_t     temp_diode_max_err;
    int16_t     temp_diode_max_warn;

    int16_t     temp_ambient_min_err;
    int16_t     temp_ambient_min_warn;
    int16_t     temp_ambient_max_err;
    int16_t     temp_ambient_max_warn;

    /* --- Flusso [L/min × 10] (offset 64, 8 byte) --- */
    uint16_t    flow_min_err_lpm_x10;
    uint16_t    flow_min_warn_lpm_x10;
    uint16_t    flow_max_err_lpm_x10;
    uint16_t    flow_max_warn_lpm_x10;

    /* --- Parametri operativi (offset 72, 12 byte) --- */
    uint32_t    interlock_mask;
    uint32_t    lid_mask;
    uint16_t    startup_delay_ms;
    uint8_t     reduced_setpoint_range;
    uint8_t     _op_pad[1];

    /* --- Metadata (offset 84, 4 byte) --- */
    uint32_t    fw_version;

    /* --- Sequenza accensione (offset 88, 4 byte) --- */
    uint16_t    contactor_psu_delay_ms;
    uint16_t    sab_interlock_timeout_ms;

    /* --- Dispositivi abilitati (offset 92, 3 byte) --- */
    uint8_t     sab_enabled;
    uint8_t     psu_enabled_mask;
    uint8_t     contactor_enabled_mask;

    /* --- Soglie umidità e dew point (offset 95, 5 byte totali) ---
     *
     * humidity_max_warn_pct / laseq_humidity_max_warn_pct:
     *   Soglie umidità assoluta. LaseQ opera in ambiente più caldo,
     *   quindi la sua soglia può essere più alta.
     *
     * dew_warn_delta_c / dew_err_delta_c:
     *   Margine rispetto al punto di rugiada (calcolato con formula di Magnus).
     *   Warning se (T_amb - T_dew) < dew_warn_delta_c.
     *   Error   se (T_amb - T_dew) < dew_err_delta_c.
     *   Applicati INDIPENDENTEMENTE per MMC (SHT35 on-board) e LaseQ (SHT35 interno).
     *   Default: ERR_BIT_DEW_MMC mascherato, ERR_BIT_DEW_LASEQ attivo.
     */
    uint8_t     humidity_max_warn_pct;       /* MMC/SHT35 [%RH], default 80   */
    uint8_t     laseq_humidity_max_warn_pct; /* LaseQ ambient [%RH], default 90 */
    int8_t      dew_warn_delta_c;            /* Margine warn dew point [°C], default 5 */
    int8_t      dew_err_delta_c;             /* Margine err  dew point [°C], default 3 */
    uint8_t     _amb_pad[1];                 /* → offset 100, allineato a 4   */

    /* --- Maschere warning/errori (offset 100, 8 byte) ---
     *
     * Bit assignments per warning_mask (vedi WARN_BIT_* in task_monitor.h).
     * Bit assignments per error_mask   (vedi ERR_BIT_*  in task_monitor.h).
     * Default: 0xFFFFFFFF = tutti abilitati.
     */
    uint32_t    warning_mask;
    uint32_t    error_mask;

    /* --- Setpoint hardware + ritardo monitoraggio PSU (offset 108, 4 byte) --- */
    uint8_t     hw_setpoint_sel;       /* 0=EXT analog, 1=AMC DAC */
    uint8_t     _hw_sp_pad[1];
    uint16_t    psu_dc_ok_delay_ms;    /* Inibizione check DC_OK dopo SON [ms], default 2000 */

    /* --- Tabella NTC (offset 112, 56 byte) ---
     *
     * ntc_ch_map[ch] = NTC_SensorId_t per il canale AD7490 ch (0-15).
     * 0xFF = canale non utilizzato.
     * Modello beta: T = 1/(1/T0 + ln(R/R0)/beta), T0=298.15K
     * Circuito partitore: VCC → R_series → NTC → GND
     *   R_ntc = R_series * raw / (4095 - raw)
     *
     * ntc_beta[ch] (dal 2026-07-22): Beta INDIVIDUALE per canale AD7490
     * fisico (0-15), stessa indicizzazione di ntc_ch_map — NON per
     * serigrafia e NON per NTC_SensorId_t logico. Il comando RS485
     * "SET NTC BETA <serigrafia 1-16> <beta>" traduce la serigrafia in
     * canale con la stessa tabella già usata da "SET NTC MAP"
     * (s_ntc_serigrafia_to_ch[], rs485_cmd.c), quindi per l'operatore
     * l'indicizzazione resta "per serigrafia" anche se qui sotto è per
     * canale. ntc_r0_ohm/ntc_rseries_ohm restano invece scalari globali
     * (stesso modello di partitore per tutti gli NTC, cambia solo il Beta
     * dichiarato per componente).
     */
    uint8_t     ntc_ch_map[16];
    uint32_t    ntc_r0_ohm;
    uint16_t    ntc_beta[16];
    uint16_t    ntc_rseries_ohm;
    uint8_t     _ntc_pad[2];              /* → offset 168, allineato a 4 */

    /* --- LUT Setpoint (offset 136, 42 byte) ---
     *
     * setpoint_lut_ma[i] = corrente [mA] per setpoint al i*5%.
     * Es: setpoint_lut_ma[0]=0mA (0%), setpoint_lut_ma[20]=10500mA (100%).
     * Aggiornabile in service mode tramite Config_SetLUTEntry() / Config_SetLUT().
     */
    uint16_t    setpoint_lut_ma[SETPOINT_LUT_SIZE];

    /* --- Fotodiodi AMC (offset 178, 14 byte) ---
     *
     * Parametri inviati ad AMC via MSG_CONFIG_PD all'avvio.
     * pd_mask: bitmask PD attivi (bit0=PD1..bit3=PD4).
     * pd_gain_threshold[4]: soglie ADC raw che dividono il range setpoint
     *   nelle finestre di guadagno (0=finestra min, 3=finestra max).
     * pd_gain_settle_ms: ms di attesa OTA dopo cambio guadagno.
     * pd_stability_samples: campioni per valutare stabilità setpoint.
     * pd_stability_threshold: variazione max [raw counts] per "stabile".
     */
    uint8_t     pd_mask;                  /* off 178, 1 byte */
    uint8_t     pd_gain_windows;          /* off 179, 1 byte (2 o 4) */
    uint16_t    pd_gain_threshold[4];     /* off 180, 8 byte */
    uint16_t    pd_gain_settle_ms;        /* off 188, 2 byte */
    uint8_t     pd_stability_samples;     /* off 190, 1 byte */
    uint8_t     pd_stability_threshold;   /* off 191, 1 byte */
                                          /* = 14 byte totali */

    /* --- Maschera fault comunicazione (offset 192, 4 byte) ---
     *
     * Bit assignments: FAULT_BIT_* in task_monitor.h. Categoria SEPARATA da
     * warning_mask/error_mask: qui rientrano ESCLUSIVAMENTE i fault di
     * comunicazione/timeout interni alla macchina (COM interface, LaseQ,
     * AMC) — mai errori operativi (temperature, flusso, umidità, dew point).
     * Default: 0xFFFFFFFF = tutti abilitati.
     */
    uint32_t    fault_mask;

    /* --- Device masks aggiuntivi (offset 196, 4 byte) ---
     *
     * Stesso pattern di psu_enabled_mask/contactor_enabled_mask (offset 92):
     * abilitano/disabilitano il monitoraggio del dispositivo fisico
     * corrispondente, indipendentemente da warning_mask/error_mask/fault_mask
     * (quelle mascherano solo la NOTIFICA di un evento già rilevato; queste
     * disabilitano la lettura/il check stesso, per HW non ancora cablato).
     *
     * flow_sensor_enabled_mask: bit0=FLOW_METER_1, bit1=FLOW_METER_2.
     * temp_sensor_enabled_mask: bit0=WATER_IN, bit1=WATER_OUT, bit2=DRIVER,
     *   bit3=SPLICE, bit4=DIODE1, bit5=AMBIENT, bit6=DIODE2 (dal 2026-07-15,
     *   diodo laser a doppio sensore), bit7=PSU_TEMP, bit8=PWR_EL_TEMP
     *   (dal 2026-07-22, nuovi slot NTC) — i sensori NTC "funzionali" di
     *   NTC_SensorId_t (le restanti serigrafie generiche restano gestite
     *   solo da ntc_ch_map). Esteso da uint8_t a uint16_t il 2026-07-22 per
     *   fare posto ai due nuovi bit. MB_TEMP (SHT35-MMC) non ha un bit qui:
     *   è un dispositivo I2C fisso on-board, non un canale NTC mappabile,
     *   mascherabile solo lato notifica (WARN_BIT_TEMP_MB/ERR_BIT_TEMP).
     *
     * "Photodiode mask" (richiesta insieme a queste due): riusa il campo
     * pd_mask già esistente sopra (offset 178→210).
     */
    uint8_t     flow_sensor_enabled_mask; /* off 228, 1 byte */
    uint8_t     _dev2_pad0;               /* off 229, 1 byte, allinea il campo seguente a 2 */
    uint16_t    temp_sensor_enabled_mask; /* off 230, 2 byte -> offset 232, allineato a 4 */

    /* --- QCW (Quasi-Continuous Wave), offset 200, 4 byte ---
     *
     * Parametri di modulazione impulsata di nGATE_MC (PE12) in modalità
     * software (FSM_MODE_SW), attivi durante SYS_EMISSION quando
     * qcw_enabled != 0 — vedi Drivers/QCW/QCW.h. In CW (qcw_enabled == 0,
     * default) il gate resta aperto in modo statico come da comportamento
     * originale (action_enter_emission()/BoardCtrl_GateMC_Open()).
     *
     * qcw_freq_hz: frequenza di impulsazione [Hz], range utile 1-50000Hz
     *   (vedi QCW_MIN_FREQ_HZ/QCW_MAX_FREQ_HZ in QCW.h — prescaler dinamico,
     *   limiti imposti dalla risoluzione a 16 bit del timer dedicato, non
     *   da TIM1/OS tick).
     * qcw_duty_pct: duty cycle [%], range nominale 1-99 — ma vincolato anche
     *   dal basso in funzione di qcw_freq_hz: il Ton (fase ON) effettivo non
     *   deve scendere sotto QCW_MIN_ON_TIME_US (10us), quindi a frequenze
     *   sopra 1000Hz il duty minimo REALE è più alto (es. 50% a 50000Hz),
     *   non 1% — vedi QCW_ClampDutyForMinOnTime()/QCW_MIN_ON_TIME_US in
     *   QCW.h, applicato ad ogni "SET FREQ"/"SET DUTY" (rs485_cmd.c e
     *   COM_interface_app.c) PRIMA di scrivere questo campo.
     * qcw_enabled: 0=CW (default, comportamento invariato), 1=QCW attivo.
     */
    uint16_t    qcw_freq_hz;              /* off 200, 2 byte */
    uint8_t     qcw_duty_pct;             /* off 202, 1 byte */
    uint8_t     qcw_enabled;              /* off 203, 1 byte */

    /* --- Maschera alimentazione eFuse (offset 204, 4 byte) ---
     *
     * Bit0=EFUSE_MAIN (scheda madre), bit1=EFUSE_SAB, bit2=EFUSE_COM,
     * bit3=EFUSE_LASEQ (stessa numerazione di EFuse_id_t, eFuse.h).
     *
     * Se il bit corrispondente è 0, quell'eFuse resta SEMPRE spento (SHDN
     * asserito nel livello che lo disabilita — attenzione: EFUSE_MAIN è
     * active HIGH, gli altri tre active LOW, vedi eFuse.h/eFuse.c — usare
     * sempre EFuse_Enable()/EFuse_Disable(), mai scrivere il pin a mano):
     * applicata una sola volta in EFuse_Init() (Sys_HwInit(), freertos.c),
     * PRIMA di qualunque tentativo di comunicazione/controllo verso quel
     * modulo. Diversa da fault_mask/error_mask (quelle mascherano solo la
     * NOTIFICA di un fault già rilevato): questa taglia l'alimentazione a
     * monte, stesso principio di flow_sensor_enabled_mask/
     * temp_sensor_enabled_mask per l'HW non ancora cablato/da escludere.
     *
     * Default: 0x0F = tutti e 4 i moduli alimentati (comportamento
     * originale, invariato per chi non la configura mai).
     */
    uint32_t    efuse_enabled_mask;       /* off 204, 4 byte */

    /* --- Fault latch (offset 208, 8 byte) ---
     *
     * Vedi banner "FAULT LATCH" in task_monitor.h. Stessa numerazione bit
     * di fault_mask (FAULT_BIT_*, task_monitor.h) — NON un nuovo set di
     * costanti.
     *
     * fault_latch_mask: quali FAULT_BIT_* devono "latched" (persistere in
     *   flash) al verificarsi, impedendo un riavvio automatico via
     *   power-cycle finché non arriva un comando esplicito "FRST" (RS485,
     *   protetto da login — vedi FSM_RequestFaultReset() in fsm.h).
     *   Default: FAULT_BIT_FLOOD1|FAULT_BIT_FLOOD2 (Config_LoadDefaults()).
     *
     * fault_latch_active: bitmask RUNTIME dei fault attualmente latched —
     *   scritta da action_fault() (fsm.c) al momento dell'ingresso in
     *   SYS_FAULT se (TaskMonitor_GetFaults() & fault_latch_mask) != 0,
     *   persistita IMMEDIATAMENTE in flash (non attende un "SAVE CONFIG"
     *   manuale). Controllata da FSM_Init() a ogni boot: se diversa da 0,
     *   la FSM transisce direttamente INIT -> FAULT, saltando il check
     *   hardware normale. Azzerata SOLO dal comando "FRST" (azione
     *   action_fault_reset(), fsm.c), che ripulisce anche i flag di
     *   debounce in task_monitor.c che altrimenti ri-triggererebbero subito
     *   lo stesso fault.
     */
    uint32_t    fault_latch_mask;         /* off 208, 4 byte */
    uint32_t    fault_latch_active;       /* off 212, 4 byte */

    /* --- Soglie temperatura diodo laser 2 (offset 216, 8 byte) ---
     *
     * Dal 2026-07-15: il diodo laser è monitorato da DUE sensori NTC fisici
     * indipendenti (serigrafia NTC3/CH12 = DIODE1, campi temp_diode_*
     * sopra; serigrafia NTC4/CH13 = DIODE2, campi qui) — vedi
     * NTC_SENSOR_DIODE1/DIODE2 e WARN_BIT_TEMP_DIODE1/DIODE2 in
     * task_monitor.h. Occupa gli ultimi 8 byte di _reserved (0 byte liberi
     * rimasti per espansione futura).
     */
    int16_t     temp_diode2_min_err;      /* off 216, 2 byte */
    int16_t     temp_diode2_min_warn;     /* off 218, 2 byte */
    int16_t     temp_diode2_max_warn;     /* off 220, 2 byte */
    int16_t     temp_diode2_max_err;      /* off 222, 2 byte */

    /* --- Soglie temperatura ambiente LaseQ (offset 224, 8 byte) ---
     *
     * Dal 2026-07-15: separate da temp_ambient_* (sensore ambiente MMC,
     * sopra) — l'ambiente interno di LaseQ è più caldo per la vicinanza dei
     * componenti elettronici di potenza, condividere la stessa soglia
     * causava falsi warning/fault. Vedi Monitor_CheckLaseQTelemetry() in
     * task_monitor.c. Default: min_err/min_warn uguali a temp_ambient
     * (10°C/12°C), max_warn/max_err portati a 43°C/45°C (più alti di
     * temp_ambient, per l'ambiente LaseQ fisiologicamente più caldo).
     */
    int16_t     temp_lq_ambient_min_err;  /* off 256, 2 byte */
    int16_t     temp_lq_ambient_min_warn; /* off 258, 2 byte */
    int16_t     temp_lq_ambient_max_warn; /* off 260, 2 byte */
    int16_t     temp_lq_ambient_max_err;  /* off 262, 2 byte */

    /* --- Soglie temperatura PSU (offset 264, 8 byte, NUOVO 2026-07-22) ---
     *
     * NTC_SENSOR_PSU_TEMP, di default mappato su serigrafia NTC14 (CH2) —
     * non ancora cablato in campo, slot pronto per quando la sonda verrà
     * installata (vedi ntc_ch_map/Config_LoadDefaults()). Default tarati
     * sul range operativo dichiarato per il PSU NSP-3200-48 Mean Well
     * (operativo fino a 70-85°C con derating): max_warn=50°C segnala che
     * il PSU è già in zona di derating, max_err=80°C resta sotto il limite
     * assoluto superiore. Modificabili via "SET TEMP PSU_TEMP ...".
     */
    int16_t     temp_psu_min_err;         /* off 264, 2 byte */
    int16_t     temp_psu_min_warn;        /* off 266, 2 byte */
    int16_t     temp_psu_max_warn;        /* off 268, 2 byte */
    int16_t     temp_psu_max_err;         /* off 270, 2 byte */

    /* --- Soglie temperatura elettronica di potenza (offset 272, 8 byte, NUOVO 2026-07-22) ---
     *
     * NTC_SENSOR_PWR_EL_TEMP, di default mappato su serigrafia NTC15 (CH1)
     * — non ancora cablato in campo. Default PLACEHOLDER (stessi valori di
     * PSU_TEMP, nessuna specifica fornita per questo sensore) — DA
     * CALIBRARE quando la sonda sarà installata e la zona termica nota.
     */
    int16_t     temp_pwr_el_min_err;      /* off 272, 2 byte */
    int16_t     temp_pwr_el_min_warn;     /* off 274, 2 byte */
    int16_t     temp_pwr_el_max_warn;     /* off 276, 2 byte */
    int16_t     temp_pwr_el_max_err;      /* off 278, 2 byte */

    /* --- Soglie temperatura scheda madre / SHT35-MMC (offset 280, 8 byte, NUOVO 2026-07-22) ---
     *
     * MB_TEMP: temperatura riportata dall'SHT35 già montato su MMC (I2C2),
     * finora usato SOLO per umidità/dew point (vedi humidity_max_warn_pct/
     * dew_*), MAI confrontato con soglie di temperatura proprie. Va
     * tenuto distinto da temp_ambient_* (quello è l'NTC fisico CH11,
     * misura la scocca/l'aria del rack — vedi commento sopra): MB_TEMP è
     * un sensore montato su un PCB, che in esercizio normale raggiunge
     * facilmente 35-45°C — confrontarlo con soglie da "temperatura
     * ambiente esterna" (20-27°C) sarebbe concettualmente errato. Default
     * allineati a temp_lq_ambient_* (stesso tipo di sensore, stessa
     * fisiologia termica: SHT35 su scheda elettronica).
     */
    int16_t     temp_mb_min_err;          /* off 280, 2 byte */
    int16_t     temp_mb_min_warn;         /* off 282, 2 byte */
    int16_t     temp_mb_max_warn;         /* off 284, 2 byte */
    int16_t     temp_mb_max_err;          /* off 286, 2 byte */

    /* --- Compensazione dinamica tensione PSU (offset 288, 4 byte, NUOVO 2026-07-22) ---
     *
     * Attiva, solo in modalità SW non-QCW e con SETPOINT HW non attivo
     * (hw_sel letto localmente da AMC, vedi AMC_protocol.h), un abbassamento
     * temporaneo della tensione PSU durante l'emissione per ridurre la
     * dissipazione sui mosfet dei driver lineari — vedi banner in
     * AMC/App/Setpoint/psu_voltage_comp.h per la macchina a stati completa.
     *
     * voltage_comp_enabled:     0=disattivo (comportamento invariato,
     *                           tensione sempre a psu_voltage_mv), 1=attivo.
     * comp_stabilize_delay_ms:  Debounce dopo l'accensione laser (LASE_Q_OUT_5V
     *                           alto) prima di iniziare la discesa [10-50ms].
     * comp_ramp_duration_ms:    Durata della rampa LENTA in discesa verso la
     *                           tensione target dalla LUT mA→mV [50-100ms].
     * comp_ramp_up_duration_ms: Durata della rampa VELOCE in salita, quando il
     *                           setpoint di corrente aumenta durante
     *                           l'emissione [1-5ms] — usata sia da AMC (per
     *                           eseguirla) sia da MMC (per sapere quanto
     *                           attendere prima di inoltrare il nuovo
     *                           setpoint a LaseQ, vedi task_amc.c).
     * Tutti uint8_t: i range sopra stanno comodamente in un byte, utile per
     * restare dentro i 6 byte reserved di AMC_PayloadConfigSet_t.
     * Trasmessi ad AMC nello stesso messaggio CONFIG_SET del voltage/current
     * limit (all'accensione e in IDLE dopo modifica via RS485).
     */
    uint8_t     voltage_comp_enabled;      /* off 288, 1 byte */
    uint8_t     comp_stabilize_delay_ms;   /* off 289, 1 byte */
    uint8_t     comp_ramp_duration_ms;     /* off 290, 1 byte */
    uint8_t     comp_ramp_up_duration_ms;  /* off 291, 1 byte */

    /* --- Riservato per espansione futura (offset 292, 28 byte) --- */
    uint8_t     _reserved[28];

} Config_t;
#pragma pack(pop)

_Static_assert(
    sizeof(Config_t) == 320U,
    "Config_t must be exactly 320 bytes (8 x 40, H7 flash granularity)"
);

_Static_assert(
    (sizeof(Config_t) % FLASH_WRITE_GRANULARITY) == 0,
    "Config_t size must be a multiple of 32 bytes"
);

/* ========================================================================== */
/* --- RECORD DI JOURNALING --- */
/* ========================================================================== */

typedef enum __attribute__((packed)) {
    RECORD_FREE     = 0xFF,
    RECORD_VALID    = 0xAA,
    RECORD_OBSOLETE = 0x00,
} RecordStatus_t;

typedef struct __attribute__((packed, aligned(32))) {
    RecordStatus_t  status;
    uint8_t         _pad[3];
    uint32_t        magic;
    uint32_t        crc32;
    uint8_t         _hdr_pad[20];
    Config_t        data;
} ConfigRecord_t;

_Static_assert(
    (sizeof(ConfigRecord_t) % FLASH_WRITE_GRANULARITY) == 0,
    "ConfigRecord_t size must be a multiple of 32 bytes"
);

/*
 * IMPORTANTE: usa CONFIG_JOURNAL_SIZE (non CONFIG_FLASH_SIZE) — il settore 7
 * è condiviso con LUT_Store_t (lut_manager.c), che occupa una riserva fissa
 * in fondo al settore (vedi flash_map.h, LUT_RESERVED_SIZE). Il journal non
 * deve mai scrivere oltre CONFIG_JOURNAL_SIZE, altrimenti sovrascrive i
 * byte della LUT (bug osservato il 2026-07-02, vedi commento in flash_map.h).
 */
#define CONFIG_MAX_RECORDS  (CONFIG_JOURNAL_SIZE / sizeof(ConfigRecord_t))

/* ========================================================================== */
/* --- CODICI DI RITORNO --- */
/* ========================================================================== */

typedef enum {
    CONFIG_OK               =  0,
    CONFIG_ERR_NOT_FOUND    = -1,
    CONFIG_ERR_CRC          = -2,
    CONFIG_ERR_MAGIC        = -3,
    CONFIG_ERR_FLASH_FULL   = -4,
    CONFIG_ERR_HAL          = -5,
} Config_err_t;

/* ========================================================================== */
/* --- ISTANZA GLOBALE IN RAM --- */
/* ========================================================================== */

extern Config_t g_config;

/* ========================================================================== */
/* --- API PUBBLICA --- */
/* ========================================================================== */

void         Config_Init(CRC_HandleTypeDef *hcrc);
Config_err_t Config_Load(void);
Config_err_t Config_Save(void);
void         Config_LoadDefaults(void);

/**
 * @brief  Crea il mutex condiviso che serializza ogni accesso al settore
 *         flash 7 (journal Config + LUT_Store_t, vedi flash_map.h).
 *         Da chiamare UNA VOLTA da MX_FREERTOS_Init() (freertos.c), DOPO
 *         osKernelInitialize() — NON da Config_Init(), che gira prima
 *         dell'inizializzazione del kernel e non può creare oggetti RTOS.
 *         Idempotente (no-op se già creato).
 */
void Config_CreateFlashMutex(void);

/**
 * @brief  Acquisisce il mutex flash condiviso. Usata anche da lut_manager.c.
 *         Prima del kernel avviato (mutex non ancora creato) ritorna sempre
 *         true senza attendere: in quella finestra l'esecuzione è single-thread.
 * @param  timeout_ms  osWaitForever o un timeout in ms (stile CMSIS-RTOS2).
 * @retval true se acquisito, false su timeout/errore.
 */
bool Config_FlashMutexAcquire(uint32_t timeout_ms);

/**
 * @brief  Rilascia il mutex flash condiviso preso con Config_FlashMutexAcquire().
 */
void Config_FlashMutexRelease(void);
uint32_t     Config_CalcCRC(const Config_t *cfg);

/* ========================================================================== */
/* --- API SERVICE MODE: AGGIORNAMENTO LUT SETPOINT --- */
/* ========================================================================== */

/**
 * @brief  Aggiorna una singola entry della LUT setpoint e persiste in flash.
 *
 * Da chiamare dal gestore RS485/COM in modalità service/administrator.
 * Aggiorna g_config.setpoint_lut_ma[idx] e chiama Config_Save().
 *
 * @param  idx         Indice LUT [0 .. SETPOINT_LUT_SIZE-1]
 * @param  current_ma  Valore di corrente [mA] da assegnare
 * @retval CONFIG_OK, CONFIG_ERR_HAL, o CONFIG_ERR_NOT_FOUND se idx fuori range.
 */
Config_err_t Config_SetLUTEntry(uint8_t idx, uint16_t current_ma);

/**
 * @brief  Sovrascrive l'intera LUT setpoint e persiste in flash.
 *
 * @param  lut_ma  Array di SETPOINT_LUT_SIZE valori uint16_t [mA].
 * @param  count   Numero di entry fornite (deve essere == SETPOINT_LUT_SIZE).
 * @retval CONFIG_OK, CONFIG_ERR_NOT_FOUND se count errato, CONFIG_ERR_HAL.
 */
Config_err_t Config_SetLUT(const uint16_t *lut_ma, uint8_t count);

#endif /* APP_CONFIG_CONFIG_H_ */
