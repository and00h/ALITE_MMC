/*
 * COM_interface_app.c
 *
 * Vedere COM_interface_app.h per la documentazione. Ogni entry dei tre
 * switch sotto e' commentata con la sorgente dati reale usata (stessa API
 * gia' impiegata da rs485_cmd.c per i comandi RS485 equivalenti) oppure con
 * un TODO esplicito quando il payload fisso attuale (vedi le costanti
 * COM_CTRL_PAYLOAD_xxx e l'enum COM_StatusOpcode_t in
 * COM_interface_protocol.h) non porta ancora i dati necessari.
 */

#include "COM_interface_app.h"

#include <string.h>
#include <stdint.h>
#include <stdbool.h>

#include "hal_handles.h"       /* hrtc */
#include "fsm.h"                /* FSM_GetState/GetMode/Get*HwEnabled/... */
#include "fsm_events.h"          /* SysEvent_t, Queue_PostEvent() targets */
#include "queues.h"              /* Queue_PostEvent() */
#include "config.h"              /* g_config, Config_Save() */
#include "task_monitor.h"        /* TaskMonitor_Get*(), NTC_SensorId_t */
#include "setpoint.h"            /* Setpoint_SetPct()/GetPct()/GetCurrentMa()/PctToCurrentMa() */
#include "QCW.h"                 /* QCW_SetParams()/Start()/Stop()/IsActive() */
#include "BoardCtrl.h"           /* BoardCtrl_GateMC_Open() */
#include "SHT35.h"               /* SHT35_GetData()/GetHumidityPct() */
#include "FlowMeter.h"           /* FlowMeter_GetFlowRate() */
#include "task_amc.h"            /* TaskAmc_GetPDPower() */
#include "LaseQ.h"               /* GetLaseQStatus()/GetLaseQControl() */
#include "eFuse.h"               /* EFuse_GetStatus() */
#include "SAB.h"                 /* SAB_GetState() */
#include "EXT_interface.h"       /* EXT_GetPowerSetpointRaw(): setpoint HW (PC5) */
#include "lut_manager.h"         /* LUT_ConvertHwPowerToPct(): NUOVO 2026-08-01 */
#include "rs485_cmd.h"           /* Rs485Cmd_CheckPassword()/ChangePassword()/NtcSerigrafiaToCh() */

/* ========================================================================== */
/* --- HELPER --- */
/* ========================================================================== */

/* Usata solo da COM_App_HandleConfig() (risposta a 4 byte, placeholder
 * lato ALITE_COM): COM_App_HandleStatus() ora scrive direttamente nei campi
 * tipizzati di COM_StatusTable_t, vedi sotto. */
static void put_u32(uint8_t *resp_payload, uint32_t v)
{
    memcpy(resp_payload, &v, sizeof v);
}

/* °C (float) -> decimi di grado (int16 esteso a int32, coerente con gli
 * opcode "_C10"). Arrotondamento simmetrico, nessuna dipendenza da lroundf. */
static int32_t c_to_c10(float c)
{
    return (int32_t)(c * 10.0f + ((c >= 0.0f) ? 0.5f : -0.5f));
}

/*
 * COM_STAT_OP_SYS_STATE: ALITE_COM non tratta questo byte come l'ordinale
 * lineare di SysState_t (MMC, fsm.h: SYS_INIT=0/IDLE=1/ACTIVE=2/ON=3/
 * ENABLED=4/EMISSION=5/ERROR=6/FAULT=7). Lo store' direttamente in
 * ALITE_system_status_e (alite_system.h), che e' un BITMASK CUMULATIVO:
 * Idle=0b0000, Active=0b0001 (bit START), On=0b0011 (START|SON),
 * Enabled=0b0111 (START|SON|SEN), PowerOn=0b1111 (START|SON|SEN|PON) — stessi
 * bit di COM_CTRL_FLAG_PON/SEN/SON/START. Prova diretta: com_dispatch.c
 * (ALITE_COM) ha fsm_flags_from_status() che genera esattamente questi bit a
 * partire da Idle/Active/On/Enabled/PowerOn, e alite_system.c confronta
 * mb_status con questi stessi simboli (mb_status != Idle, == Active, ecc.)
 * per le guardie di ALITE_system_start()/_set()/_set_enabled()/
 * _set_poweron(). Inviare l'ordinale SysState_t grezzo (es. SYS_ON = 3, che
 * per puro caso numerico coincide con On = 0b0011 = 3, ma SYS_IDLE = 1 !=
 * Idle = 0, SYS_ACTIVE = 2 != Active = 1, SYS_ENABLED = 4 != Enabled = 7,
 * SYS_EMISSION = 5 != PowerOn = 15) romperebbe silenziosamente le guardie di
 * stato lato ALITE_COM. Mappatura 1:1 verificata sulla progressione delle
 * guardie (SYS_IDLE->SYS_ACTIVE via START, SYS_ACTIVE->SYS_ON via SON,
 * SYS_ON->SYS_ENABLED via SEN, SYS_ENABLED->SYS_EMISSION via PON — stessa
 * progressione a 5 livelli di Idle/Active/On/Enabled/PowerOn).
 *
 * SYS_INIT/SYS_ERROR/SYS_FAULT non hanno un corrispondente in
 * ALITE_system_status_e (che ha solo Idle/Active/On/Enabled/PowerOn per il
 * percorso normale, piu' i sentinel locali Error/Unconnected/Uninitialized a
 * 32 bit non rappresentabili in un singolo byte 0..255): qui si riporta Idle
 * (0) come "nessun livello operativo attivo" — err_active/fault_active
 * (COM_STAT_OP_ERR_ACTIVE/FAULT_ACTIVE, gia' inviati separatamente)
 * restano la fonte primaria per capire che c'e' un errore/fault. DA
 * CONFERMARE col team COM che questo non induca l'interfaccia a mostrare la
 * macchina come genuinamente "Idle" durante un errore/fault.
 */
static uint8_t alite_status_bits_from_fsm(SysState_t st)
{
    switch (st) {
    case SYS_ACTIVE:   return COM_CTRL_FLAG_START;
    case SYS_ON:        return (uint8_t)(COM_CTRL_FLAG_START | COM_CTRL_FLAG_SON);
    case SYS_ENABLED:   return (uint8_t)(COM_CTRL_FLAG_START | COM_CTRL_FLAG_SON | COM_CTRL_FLAG_SEN);
    case SYS_EMISSION:  return (uint8_t)(COM_CTRL_FLAG_START | COM_CTRL_FLAG_SON | COM_CTRL_FLAG_SEN | COM_CTRL_FLAG_PON);
    case SYS_IDLE:
    case SYS_INIT:
    case SYS_ERROR:
    case SYS_FAULT:
    default:
        return 0x0U;
    }
}

static void read_rtc(RTC_TimeTypeDef *t, RTC_DateTypeDef *d)
{
    /* HAL_RTC_GetDate() dopo GetTime() e' richiesto per sbloccare gli shadow
     * register (stesso schema di rs485_cmd.c, "GET TIME"/"GET DATE"). */
    HAL_RTC_GetTime(&hrtc, t, RTC_FORMAT_BIN);
    HAL_RTC_GetDate(&hrtc, d, RTC_FORMAT_BIN);
}

/* ========================================================================== */
/* --- STATUS --- */
/* ========================================================================== */

void COM_App_HandleStatus(const uint8_t *req_payload, uint8_t *resp_payload)
{
    /*
     * AGGIORNAMENTO 2026-07-30 — ALITE_COM ha cambiato il formato della
     * risposta STATUS: non piu' "1 opcode -> 4 byte a posizione fissa", ma
     * un'unica tabella completa (COM_StatusTable_t, COM_interface_protocol.h)
     * dove ogni campo vive all'offset ASSOLUTO del proprio COM_StatusOpcode_t.
     * req_payload (n_requested + lista opcode, COM_status_req_t lato
     * ALITE_COM) e' quindi IGNORATO qui di proposito: si risponde sempre con
     * la tabella intera, indipendentemente da cosa/quanto era stato elencato
     * nella richiesta — vedi il banner sopra COM_StatusTable_t per le prove
     * nel codice ALITE_COM che giustificano questa scelta (indicizzazione ad
     * offset assoluto in ALITE_system_update_status_full(), e il bug di
     * n_requested in ALITE_get_system_err_flag() che rende la lista
     * inaffidabile).
     */
    (void)req_payload;

    memset(resp_payload, 0, COM_PAYLOAD_SIZE);
    resp_payload[0] = (uint8_t)COM_RESP_OK;

    COM_StatusTable_t tbl;
    memset(&tbl, 0, sizeof tbl);

    /* Vedi banner sopra alite_status_bits_from_fsm(): NON e' l'ordinale
     * grezzo di FSM_GetState(), e' il bitmask che ALITE_system_status_e
     * (ALITE_COM) si aspetta. */
    tbl.sys_state      = alite_status_bits_from_fsm(FSM_GetState());
    /* SAB_STATE: nessun consumo osservato lato ALITE_COM (ne' in
     * alite_system.c ne' altrove nel sorgente attuale) — nessuna codifica da
     * rispettare per ora, si riporta l'ordinale SAB_State_t nativo. Se in
     * futuro ALITE_COM inizia a interpretarlo come proprio enum, andra'
     * riverificato come sopra per sys_state. */
    tbl.sab_state      = (uint8_t)SAB_GetState();

    /*
     * NUOVO 2026-08-01: in ANALOG, oppure in SW con SETPOINT HW ("HYBRID2")
     * attivo — FSM_GetLaserModeWire() != 0, stessa condizione usata per il
     * byte laser_mode verso AMC e per "GET STATUS" (rs485_cmd.c) — il
     * setpoint realmente in vigore è quello hardware (pin PC5/LPWR_SET_ISO,
     * EXT_interface.c), non il setpoint software: lo riportiamo qui
     * convertito in % tramite la LUT di calibrazione dedicata
     * (LUT_ConvertHwPowerToPct(), lut_manager.h) al posto di
     * Setpoint_GetPct()/GetCurrentMa(). setpoint_cur_ma è il corrispondente
     * in mA tramite la LUT SETPOINT esistente, solo per coerenza — nessun
     * impatto sul percorso di controllo reale verso LaseQ/AMC (invariato).
     * In tutti gli altri casi (SW senza SETPOINT HW) comportamento
     * identico a prima.
     */
    if (FSM_GetLaserModeWire() != 0U) {
        uint8_t hw_pct = LUT_ConvertHwPowerToPct(EXT_GetPowerSetpointRaw());
        tbl.setpoint_pct    = hw_pct;
        tbl.setpoint_cur_ma = (uint16_t)Setpoint_PctToCurrentMa(hw_pct);
    } else {
        tbl.setpoint_pct    = Setpoint_GetPct();
        tbl.setpoint_cur_ma = (uint16_t)Setpoint_GetCurrentMa();
    }

    /*
     * SYS_FLAGS_0..4 (5 byte, 40 bit): la codifica bit-per-bit non e' ancora
     * stata concordata col team COM (vedi meeting notes: "codifica da
     * definire" per errori/warning, stato valvola, stato contattori/
     * coperchi, enable/gate laser...). Fino a spec formale esponiamo SOLO
     * SYS_FLAGS_0 con un mapping PROVVISORIO dei pochi flag booleani gia'
     * disponibili come singole funzioni (modalita' e toggle HYBRID1/2 +
     * QCW enable); SYS_FLAGS_1..4 restano a 0 (TODO: da definire con COM).
     */
    {
        uint8_t flags = 0;
        if (FSM_GetMode() == FSM_MODE_ANALOG)  flags |= (1U << 0);
        if (FSM_GetGateHwEnabled())             flags |= (1U << 1);
        if (FSM_GetSetpointHwEnabled())         flags |= (1U << 2);
        if (g_config.qcw_enabled)               flags |= (1U << 3);
        if (QCW_IsActive())                     flags |= (1U << 4);
        tbl.sys_flags_0 = flags;
    }
    /* sys_flags_1..4 restano 0 (memset sopra). */

    {
        uint16_t pw[4];
        TaskAmc_GetPDPower(pw);
        tbl.pd0_power_w = pw[0];
        tbl.pd1_power_w = pw[1];
        tbl.pd2_power_w = pw[2];
        tbl.pd3_power_w = pw[3];
    }

    /*
     * ERR_ACTIVE (2026-07-31, richiesta esplicita team COM): TaskMonitor_
     * GetErrors() riporta i bit "errore recuperabile" anche quando la FSM
     * non e' (piu') in SYS_ERROR — es. un errore gia' rientrato ma il cui
     * bit resta attivo per un giro finche' Monitor_CheckTemperatures()/altri
     * check non lo azzerano, o un bit relativo a una condizione osservata
     * ma non (ancora/piu') sufficiente a tenere la FSM in SYS_ERROR. Inviarlo
     * sempre creava ambiguita' sull'interfaccia grafica ALITE_COM (poteva
     * mostrare "errore attivo" mentre la macchina e' gia' tornata operativa
     * in uno stato successivo). Da qui in poi: valore reale SOLO quando
     * FSM_GetState() == SYS_ERROR, altrimenti sempre 0 — nessuna ambiguita'
     * possibile lato UI. err_mask (soglia di quali bit contano come errore)
     * resta invariata, e' solo ERR_ACTIVE ad essere azzerato fuori da
     * SYS_ERROR. FAULT_ACTIVE/WARN_ACTIVE NON sono toccati: la richiesta
     * riguardava esplicitamente il flag di errore, non fault/warning.
     */
    tbl.err_active = (FSM_GetState() == SYS_ERROR) ? TaskMonitor_GetErrors() : 0U;
    tbl.err_mask            = g_config.error_mask;
    tbl.warn_active          = TaskMonitor_GetWarnings();
    tbl.warn_mask            = g_config.warning_mask;
    tbl.fault_active         = TaskMonitor_GetFaults();
    tbl.fault_mask           = g_config.fault_mask;
    tbl.fault_latch_mask     = g_config.fault_latch_mask;
    tbl.fault_latch_active   = g_config.fault_latch_active;

    /* temp_fault_sensor_mask e' 24 bit (3 byte) nel wire, sorgente reale
     * uint16_t (TaskMonitor_GetTempFaultSensors()): nessuna perdita di dati,
     * il byte alto scritto qui resta a 0. */
    {
        uint16_t m = TaskMonitor_GetTempFaultSensors();
        tbl.temp_fault_sensor_mask[0] = (uint8_t)(m & 0xFFU);
        tbl.temp_fault_sensor_mask[1] = (uint8_t)((m >> 8) & 0xFFU);
        tbl.temp_fault_sensor_mask[2] = 0U;
    }
    tbl.last_fault_event_id = (uint8_t)FSM_GetLastFaultEvent();
    tbl.last_error_event_id = (uint8_t)FSM_GetLastErrorEvent();

    /* --- LaseQ (RS485, telemetria gia' cache-ata da task_comms.c) --------- */
    {
        LaseQ_status_vars_t  lq_st  = GetLaseQStatus();
        LaseQ_control_vars_t lq_ctl = GetLaseQControl();

        tbl.lq_error_code  = lq_st.error_code;
        /* Canali comandati da MMC (non necessariamente ancora confermati
         * dallo slave) — stessa fonte di "GET LQ" (rs485_cmd.c). */
        tbl.lq_ch_mask     = lq_ctl.ch_enable;
        tbl.lq_ilk         = lq_st.interlock_status;
        tbl.lq_opm         = lq_st.OPM;
        tbl.lq_opd         = lq_st.OPD;
        /* Setpoint comandato da MMC (GetLaseQControl), distinto dall'echo
         * confermato dallo slave (GetLaseQStatus) — vedi banner in
         * rs485_cmd.c "GET LQ". Sorgenti uint32_t, wire a 2 byte: valori
         * attesi in mA di corrente diodo, ben entro 65535 nell'uso reale. */
        tbl.lq_sp_cmd_ma   = (uint16_t)lq_ctl.sw_current_setpoint;
        tbl.lq_sp_echo_ma  = (uint16_t)lq_st.sw_current_setpoint;
        tbl.lq_vanode_mv   = lq_st.v_anode;
        tbl.lq_i0_ma       = (uint16_t)lq_st.current_output[0];
        tbl.lq_i1_ma       = (uint16_t)lq_st.current_output[1];
        tbl.lq_i2_ma       = (uint16_t)lq_st.current_output[2];
        tbl.lq_i3_ma       = (uint16_t)lq_st.current_output[3];
        tbl.lq_fsm_state   = lq_st.fsm_state;

        tbl.lq_driver_temp_c[0] = lq_st.temperature[0];
        tbl.lq_driver_temp_c[1] = lq_st.temperature[1];
        tbl.lq_driver_temp_c[2] = lq_st.temperature[2];
        tbl.lq_driver_temp_c[3] = lq_st.temperature[3];
        tbl.lq_ambient_temp_c   = lq_st.temp_ambient_c;
        /* LaseQ_status_vars_t.humidity e' in 0.01 %RH (es. 5000 = 50.00%),
         * il campo wire e' "_PCT" (intero) -> conversione /100. */
        tbl.lq_humidity_pct     = (uint8_t)(lq_st.humidity / 100U);
    }

    /* --- Correnti moduli (eFuse) ------------------------------------------ */
    tbl.mod_main_current_ma   = EFuse_GetStatus(EFUSE_MAIN).current_ma;
    tbl.mod_sab_current_ma    = EFuse_GetStatus(EFUSE_SAB).current_ma;
    tbl.mode_com_current_ma   = EFuse_GetStatus(EFUSE_COM).current_ma;
    tbl.mode_laseq_current_ma = EFuse_GetStatus(EFUSE_LASEQ).current_ma;

    /* --- Dew point -----------------------------------------------------------
     * TaskMonitor_GetDew{MMC,LaseQ}() ritornano false quando il valore non e'
     * "attuale" (cooling fermo/NTC non valido): i puntatori di uscita restano
     * comunque scritti con l'ultimo valore noto (vedi task_monitor.h), che e'
     * quanto riportiamo qui — nessun modo di segnalare "stale" nel formato
     * dati attuale. */
    {
        float td, margin;
        TaskMonitor_GetDewMMC(&td, &margin);
        tbl.dew_mmc_td_c10     = (int16_t)c_to_c10(td);
        tbl.dew_mmc_margin_c10 = (int16_t)c_to_c10(margin);
        TaskMonitor_GetDewLaseQ(&td, &margin);
        tbl.dew_lq_td_c10      = (int16_t)c_to_c10(td);
        tbl.dew_lq_margin_c10  = (int16_t)c_to_c10(margin);
    }

    tbl.temp_sht35_mmc_c100 = SHT35_GetData().temperature_cdeg;
    tbl.hum_sht35_mmc_pct   = (uint8_t)SHT35_GetHumidityPct();

    /* --- NTC (MMC, AD7490) -------------------------------------------------
     * NTC_TEMP_C10_GEN2 usa NTC_SENSOR_DRIVER (id logico 2): dal 2026-07-14
     * quel canale non e' piu' associato alla temperatura "driver" (sostituita
     * dalle 4 temperature LaseQ, vedi task_monitor.h) ma resta un id generico
     * valido — ipotesi di mapping con "GEN2", da confermare col team COM.
     * NTC_SENSOR_NTC14/NTC15 non esistono piu' come id generici: dal
     * 2026-07-22 sono stati formalizzati in NTC_SENSOR_PSU_TEMP (ex "NTC14")
     * e NTC_SENSOR_PWR_EL_TEMP (ex "NTC15") — vedi task_monitor.h. */
    {
        static const NTC_SensorId_t map[16] = {
            NTC_SENSOR_WATER_IN, NTC_SENSOR_WATER_OUT, NTC_SENSOR_DRIVER,
            NTC_SENSOR_SPLICE, NTC_SENSOR_DIODE1, NTC_SENSOR_AMBIENT,
            NTC_SENSOR_DIODE2, NTC_SENSOR_NTC8, NTC_SENSOR_NTC9,
            NTC_SENSOR_NTC10, NTC_SENSOR_NTC11, NTC_SENSOR_NTC12,
            NTC_SENSOR_NTC13, NTC_SENSOR_PSU_TEMP, NTC_SENSOR_PWR_EL_TEMP,
            NTC_SENSOR_NTC16,
        };
        for (unsigned i = 0U; i < 16U; i++) {
            int16_t c10 = 0;
            TaskMonitor_GetNTCTempC10(map[i], &c10);
            tbl.ntc_temp_c10[i] = c10;
        }
    }

    tbl.flow1_lpm_x10 = (int16_t)(FlowMeter_GetFlowRate(FLOW_METER_1) * 10.0f);
    tbl.flow2_lpm_x10 = (int16_t)(FlowMeter_GetFlowRate(FLOW_METER_2) * 10.0f);

    /* ntc_map_serigrafia[16]: sensore (NTC_SensorId_t, 0xFF = OFF) di ogni
     * serigrafia 1-16 — g_config.ntc_ch_map e' indicizzato per CANALE AD7490,
     * da cui la traduzione (stessa di "GET NTC MAP"). */
    for (uint8_t s = 1U; s <= 16U; s++) {
        tbl.ntc_map_serigrafia[s - 1U] = g_config.ntc_ch_map[Rs485Cmd_NtcSerigrafiaToCh(s)];
    }

    tbl.ntc_active_mask = TaskMonitor_GetNTCActiveMask();

    /* --- Soglie (g_config, gia' nell'unita' richiesta dall'opcode) -------- */
    tbl.thr_water_in_min_err_c  = g_config.temp_water_in_min_err;
    tbl.thr_water_in_min_warn_c = g_config.temp_water_in_min_warn;
    tbl.thr_water_in_max_warn_c = g_config.temp_water_in_max_warn;
    tbl.thr_water_in_max_err_c  = g_config.temp_water_in_max_err;
    tbl.thr_water_out_min_err_c  = g_config.temp_water_out_min_err;
    tbl.thr_water_out_min_warn_c = g_config.temp_water_out_min_warn;
    tbl.thr_water_out_max_warn_c = g_config.temp_water_out_max_warn;
    tbl.thr_water_out_max_err_c  = g_config.temp_water_out_max_err;
    tbl.thr_driver_min_err_c  = g_config.temp_driver_min_err;
    tbl.thr_driver_min_warn_c = g_config.temp_driver_min_warn;
    tbl.thr_driver_max_warn_c = g_config.temp_driver_max_warn;
    tbl.thr_driver_max_err_c  = g_config.temp_driver_max_err;
    tbl.thr_splice_min_err_c  = g_config.temp_splice_min_err;
    tbl.thr_splice_min_warn_c = g_config.temp_splice_min_warn;
    tbl.thr_splice_max_warn_c = g_config.temp_splice_max_warn;
    tbl.thr_splice_max_err_c  = g_config.temp_splice_max_err;
    tbl.thr_diode1_min_err_c  = g_config.temp_diode_min_err;
    tbl.thr_diode1_min_warn_c = g_config.temp_diode_min_warn;
    tbl.thr_diode1_max_warn_c = g_config.temp_diode_max_warn;
    tbl.thr_diode1_max_err_c  = g_config.temp_diode_max_err;
    tbl.thr_diode2_min_err_c  = g_config.temp_diode2_min_err;
    tbl.thr_diode2_min_warn_c = g_config.temp_diode2_min_warn;
    tbl.thr_diode2_max_warn_c = g_config.temp_diode2_max_warn;
    tbl.thr_diode2_max_err_c  = g_config.temp_diode2_max_err;
    tbl.thr_ambient_min_err_c  = g_config.temp_ambient_min_err;
    tbl.thr_ambient_min_warn_c = g_config.temp_ambient_min_warn;
    tbl.thr_ambient_max_warn_c = g_config.temp_ambient_max_warn;
    tbl.thr_ambient_max_err_c  = g_config.temp_ambient_max_err;
    tbl.thr_lq_ambient_min_err_c  = g_config.temp_lq_ambient_min_err;
    tbl.thr_lq_ambient_min_warn_c = g_config.temp_lq_ambient_min_warn;
    tbl.thr_lq_ambient_max_warn_c = g_config.temp_lq_ambient_max_warn;
    tbl.thr_lq_ambient_max_err_c  = g_config.temp_lq_ambient_max_err;
    tbl.thr_flow_min_err_lpmx10  = g_config.flow_min_err_lpm_x10;
    tbl.thr_flow_min_warn_lpmx10 = g_config.flow_min_warn_lpm_x10;
    tbl.thr_flow_max_warn_lpmx10 = g_config.flow_max_warn_lpm_x10;
    tbl.thr_flow_max_err_lpmx10  = g_config.flow_max_err_lpm_x10;
    tbl.thr_hum_mmc_max_warn_pct    = g_config.humidity_max_warn_pct;
    tbl.thr_hum_laseq_max_warn_pct  = g_config.laseq_humidity_max_warn_pct;
    tbl.thr_dew_warn_delta_c = (uint8_t)g_config.dew_warn_delta_c;
    tbl.thr_dew_err_delta_c  = (uint8_t)g_config.dew_err_delta_c;

    /*
     * CFG_DLY_PSU/SAB/CONTACTOR_MS: mapping per nome piu' vicino disponibile
     * in g_config (nessun campo "psu_delay_ms" a se stante) — da confermare
     * col team COM:
     *   PSU        -> psu_dc_ok_delay_ms (inibizione check DC_OK dopo SON)
     *   SAB        -> sab_interlock_timeout_ms
     *   CONTACTOR  -> contactor_psu_delay_ms
     */
    tbl.cfg_dly_psu_ms        = g_config.psu_dc_ok_delay_ms;
    tbl.cfg_dly_sab_ms        = g_config.sab_interlock_timeout_ms;
    tbl.cfg_dly_contactor_ms  = g_config.contactor_psu_delay_ms;

    {
        RTC_TimeTypeDef t; RTC_DateTypeDef d;
        read_rtc(&t, &d);
        tbl.rtc_hour  = t.Hours;
        tbl.rtc_min   = t.Minutes;
        tbl.rtc_sec   = t.Seconds;
        tbl.rtc_year  = (uint16_t)(2000U + d.Year);
        tbl.rtc_month = d.Month;
        tbl.rtc_day   = d.Date;
    }

    memcpy(&resp_payload[1], &tbl, sizeof tbl);
}

/* ========================================================================== */
/* --- SOGLIE (SET_TEMP/FLOW/HUM/DEW_THR) --- */
/* ========================================================================== */

/*
 * Stessi limiti di "SET TEMP/FLOW/HUM/DEW" in rs485_cmd.c. Il login richiesto
 * su RS485 non si applica al canale COM. Nessun Config_Save() qui: come su
 * RS485 il valore resta in RAM finche' non arriva un SAVE CONFIG.
 */
#define THR_TEMP_MIN_C          (-50)
#define THR_TEMP_MAX_C          200
#define THR_FLOW_MAX_LPM_X10    600U
#define THR_HUM_MAX_PCT         100U
#define THR_DEW_MAX_DELTA_C     20U

/* Puntatori alle 4 soglie g_config di un sensore, indicizzati per
 * COM_ThrSensorId_t — stessa corrispondenza di s_temp_sensors[] in
 * rs485_cmd.c (privata a quel file), incluso DIODE1 -> campo storico "diode". */
typedef struct {
    int16_t *min_err;
    int16_t *min_warn;
    int16_t *max_warn;
    int16_t *max_err;
} ThrTempFields_t;

#define THR_TEMP_FIELDS(pfx) \
    { &g_config.temp_##pfx##_min_err,  &g_config.temp_##pfx##_min_warn, \
      &g_config.temp_##pfx##_max_warn, &g_config.temp_##pfx##_max_err }

static const ThrTempFields_t s_thr_temp_fields[COM_THR_SENSOR_COUNT] = {
    [COM_THR_SENSOR_WATER_IN]   = THR_TEMP_FIELDS(water_in),
    [COM_THR_SENSOR_WATER_OUT]  = THR_TEMP_FIELDS(water_out),
    [COM_THR_SENSOR_DRIVER]     = THR_TEMP_FIELDS(driver),
    [COM_THR_SENSOR_SPLICE]     = THR_TEMP_FIELDS(splice),
    [COM_THR_SENSOR_DIODE1]     = THR_TEMP_FIELDS(diode),
    [COM_THR_SENSOR_DIODE2]     = THR_TEMP_FIELDS(diode2),
    [COM_THR_SENSOR_AMBIENT]    = THR_TEMP_FIELDS(ambient),
    [COM_THR_SENSOR_LQ_AMBIENT] = THR_TEMP_FIELDS(lq_ambient),
};

static bool thr_level_limit_valid(uint8_t level, uint8_t limit)
{
    return (level == COM_CTRL_THR_LEVEL_WARN || level == COM_CTRL_THR_LEVEL_ERR)
        && (limit == COM_CTRL_THR_LIMIT_MIN || limit == COM_CTRL_THR_LIMIT_MAX);
}

static COM_RespStatus_t set_temp_thr(const uint8_t *args)
{
    uint8_t sensor = args[COM_CTRL_TEMP_THR_SENSOR_ID];
    uint8_t level  = args[COM_CTRL_TEMP_THR_LEVEL];
    uint8_t limit  = args[COM_CTRL_TEMP_THR_LIMIT];
    int16_t value;
    memcpy(&value, &args[COM_CTRL_TEMP_THR_VALUE_C], sizeof value);

    if (sensor >= COM_THR_SENSOR_COUNT || !thr_level_limit_valid(level, limit)
        || value < THR_TEMP_MIN_C || value > THR_TEMP_MAX_C) { return COM_RESP_ERR; }

    const ThrTempFields_t *f = &s_thr_temp_fields[sensor];
    bool is_err = (level == COM_CTRL_THR_LEVEL_ERR);
    int16_t *p = (limit == COM_CTRL_THR_LIMIT_MIN) ? (is_err ? f->min_err : f->min_warn)
                                                   : (is_err ? f->max_err : f->max_warn);
    *p = value;
    return COM_RESP_OK;
}

static COM_RespStatus_t set_flow_thr(const uint8_t *args)
{
    uint8_t  level = args[COM_CTRL_FLOW_THR_LEVEL];
    uint8_t  limit = args[COM_CTRL_FLOW_THR_LIMIT];
    uint16_t value;
    memcpy(&value, &args[COM_CTRL_FLOW_VALUE_LPM_X10], sizeof value);

    if (!thr_level_limit_valid(level, limit) || value > THR_FLOW_MAX_LPM_X10) { return COM_RESP_ERR; }

    bool is_err = (level == COM_CTRL_THR_LEVEL_ERR);
    uint16_t *p = (limit == COM_CTRL_THR_LIMIT_MIN)
                ? (is_err ? &g_config.flow_min_err_lpm_x10 : &g_config.flow_min_warn_lpm_x10)
                : (is_err ? &g_config.flow_max_err_lpm_x10 : &g_config.flow_max_warn_lpm_x10);
    *p = value;
    return COM_RESP_OK;
}

static COM_RespStatus_t set_hum_thr(const uint8_t *args)
{
    uint8_t target = args[COM_CTRL_HUM_THR_TARGET];
    uint8_t value  = args[COM_CTRL_HUM_THR_VALUE_PCT];

    if (value > THR_HUM_MAX_PCT) { return COM_RESP_ERR; }

    if      (target == COM_CTRL_THR_HUM_MMC)   { g_config.humidity_max_warn_pct       = value; }
    else if (target == COM_CTRL_THR_HUM_LASEQ) { g_config.laseq_humidity_max_warn_pct = value; }
    else { return COM_RESP_ERR; }
    return COM_RESP_OK;
}

static COM_RespStatus_t set_dew_thr(const uint8_t *args)
{
    uint8_t level = args[COM_CTRL_DEW_THR_LEVEL];
    uint8_t value = args[COM_CTRL_DEW_THR_VALUE_C];

    if (value > THR_DEW_MAX_DELTA_C) { return COM_RESP_ERR; }

    if      (level == COM_CTRL_THR_LEVEL_WARN) { g_config.dew_warn_delta_c = (int8_t)value; }
    else if (level == COM_CTRL_THR_LEVEL_ERR)  { g_config.dew_err_delta_c  = (int8_t)value; }
    else { return COM_RESP_ERR; }
    return COM_RESP_OK;
}

/* ========================================================================== */
/* --- LUT, MASCHERE, RITARDI, MAPPA NTC, PASSWORD --- */
/* ========================================================================== */

/*
 * Stessi range e stesse guardie di stato dei comandi RS485 equivalenti
 * ("SET LUT ...", "SET ... MASK", "SET DELAY", "SET NTC MAP", "LOGIN",
 * "SET PASSWORD" in rs485_cmd.c). Come per le soglie, il login RS485 non si
 * applica al canale COM: e' la COM interface a chiedere il login (LOGIN qui
 * sotto) e a tenere la sessione dei propri client prima di inoltrare questi
 * comandi. LUT gain/valid/power restano in RAM fino a SAVE_LUT, maschere/
 * ritardi/mappa NTC fino a SAVE_CONF; la LUT setpoint la persiste subito
 * Config_SetLUTEntry(), come "SET LUT SETPOINT".
 */
#define COM_LUT_GAIN_THRESHOLDS   4U      /* t0..t3, come "SET LUT GAIN" */
#define COM_DELAY_MAX_MS          10000U  /* come "SET DELAY" */

static uint16_t get_u16(const uint8_t *p)
{
    uint16_t v;
    memcpy(&v, p, sizeof v);
    return v;
}

static uint32_t get_u32(const uint8_t *p)
{
    uint32_t v;
    memcpy(&v, p, sizeof v);
    return v;
}

static bool lut_mode_valid(uint8_t mode)
{
    return (mode == LUT_MODE_SW) || (mode == LUT_MODE_HW);
}

static COM_RespStatus_t set_lut_gain(const uint8_t *args)
{
    uint8_t mode = args[COM_CTRL_LUT_GAIN_MODE];
    if (!lut_mode_valid(mode)) { return COM_RESP_ERR; }

    for (uint8_t i = 0U; i < COM_LUT_GAIN_THRESHOLDS; i++) {
        LUT_SetGainThreshold(mode, i, get_u16(&args[COM_CTRL_LUT_GAIN_T0 + (2U * i)]));
    }
    return COM_RESP_OK;
}

static COM_RespStatus_t set_lut_valid(const uint8_t *args)
{
    uint8_t pd    = args[COM_CTRL_LUT_VALID_PD];
    uint8_t mode  = args[COM_CTRL_LUT_VALID_MODE];
    uint8_t entry = args[COM_CTRL_LUT_VALID_ENTRY];
    if (pd >= LUT_PD_MAX || !lut_mode_valid(mode) || entry >= LUT_PD_VALID_SIZE) { return COM_RESP_ERR; }

    LUT_SetPDValidEntry(mode, pd, entry, get_u16(&args[COM_CTRL_LUT_VALID_SP]),
                        get_u16(&args[COM_CTRL_LUT_VALID_MIN]), get_u16(&args[COM_CTRL_LUT_VALID_MAX]));
    return COM_RESP_OK;
}

static COM_RespStatus_t set_lut_power(const uint8_t *args)
{
    uint8_t pd    = args[COM_CTRL_LUT_POWER_PD];
    uint8_t win   = args[COM_CTRL_LUT_POWER_WIN];
    uint8_t entry = args[COM_CTRL_LUT_POWER_ENTRY];
    if (pd >= LUT_PD_MAX || win >= LUT_GAIN_WINDOWS || entry >= LUT_PD_POWER_SIZE) { return COM_RESP_ERR; }

    LUT_SetPDPowerEntry(pd, win, entry, get_u16(&args[COM_CTRL_LUT_POWER_ADC]),
                        get_u16(&args[COM_CTRL_LUT_POWER_WATT]));
    return COM_RESP_OK;
}

static COM_RespStatus_t set_lut_setpoint(const uint8_t *args)
{
    uint8_t idx = args[COM_CTRL_LUT_SETPOINT_IDX];
    if (idx >= SETPOINT_LUT_SIZE) { return COM_RESP_ERR; }

    return (Config_SetLUTEntry(idx, get_u16(&args[COM_CTRL_LUT_SETPOINT_CURRENT_MA])) == CONFIG_OK)
         ? COM_RESP_OK : COM_RESP_ERR;
}

static COM_RespStatus_t set_mask(COM_ControlOpcode_t opcode, const uint8_t *args)
{
    uint32_t v = get_u32(&args[COM_CTRL_MASK_VALUE]);
    /* PSU, CONTACTOR e PD solo in IDLE, come su RS485. */
    bool idle = (FSM_GetState() == SYS_IDLE);

    switch (opcode) {
    case COM_CTRL_OP_SETERRMASK:    g_config.error_mask       = v; break;
    case COM_CTRL_OP_SETWARNMASK:   g_config.warning_mask     = v; break;
    case COM_CTRL_OP_SETFAULTMASK:  g_config.fault_mask       = v; break;
    case COM_CTRL_OP_SETFAULTLATCH: g_config.fault_latch_mask = v; break;
    case COM_CTRL_OP_SET_TEMP_MASK:
        if (v > 0x1FFU) { return COM_RESP_ERR; }
        g_config.temp_sensor_enabled_mask = (uint16_t)v;
        break;
    case COM_CTRL_OP_SET_FLOW_MASK:
        if (v > 0x03U) { return COM_RESP_ERR; }
        g_config.flow_sensor_enabled_mask = (uint8_t)v;
        break;
    case COM_CTRL_OP_SET_PSU_MASK:
        if (!idle || v > 0x03U) { return COM_RESP_ERR; }
        g_config.psu_enabled_mask = (uint8_t)v;
        break;
    case COM_CTRL_OP_SET_CONTACTOR_MASK:
        if (!idle || v > 0x03U) { return COM_RESP_ERR; }
        g_config.contactor_enabled_mask = (uint8_t)v;
        break;
    case COM_CTRL_OP_SET_EFUSE_MASK:
        /* Applicata solo al prossimo boot (EFuse_Init()), come "SET EFUSE MASK". */
        if (v > 0x0FU) { return COM_RESP_ERR; }
        g_config.efuse_enabled_mask = v;
        break;
    case COM_CTRL_OP_SET_PD_MASK:
        if (!idle || v > 0x0FU) { return COM_RESP_ERR; }
        g_config.pd_mask = (uint8_t)v;
        break;
    default:
        return COM_RESP_ERR;
    }
    return COM_RESP_OK;
}

static COM_RespStatus_t set_delay(const uint8_t *args)
{
    uint16_t v = get_u16(&args[COM_CTRL_DELAY_VALUE_MS]);
    if (v > COM_DELAY_MAX_MS) { return COM_RESP_ERR; }

    /* Stessa corrispondenza dei campi CFG_DLY_* della tabella di stato. */
    switch (args[COM_CTRL_DELAY_TARGET]) {
    case COM_CTRL_DELAY_TGT_PSU:       g_config.psu_dc_ok_delay_ms       = v; break;
    case COM_CTRL_DELAY_TGT_SAB:       g_config.sab_interlock_timeout_ms = v; break;
    case COM_CTRL_DELAY_TGT_CONTACTOR: g_config.contactor_psu_delay_ms   = v; break;
    default: return COM_RESP_ERR;
    }
    return COM_RESP_OK;
}

static COM_RespStatus_t set_ntc_map(const uint8_t *args)
{
    uint8_t ch  = Rs485Cmd_NtcSerigrafiaToCh(args[COM_CTRL_NTC_MAP_SERIGRAPHY]);
    uint8_t sid = args[COM_CTRL_NTC_MAP_SENSOR];
    if (ch == 0xFFU || (sid >= NTC_NUM_SENSORS && sid != COM_CTRL_NTC_MAP_OFF)) { return COM_RESP_ERR; }

    g_config.ntc_ch_map[ch] = sid;
    return COM_RESP_OK;
}

/* Campo password (COM_CTRL_PASS_SIZE byte) -> stringa; false se il campo non
 * contiene il terminatore. */
static bool pass_from_field(const uint8_t *field, char out[COM_CTRL_PASS_SIZE])
{
    memcpy(out, field, COM_CTRL_PASS_SIZE);
    return memchr(out, '\0', COM_CTRL_PASS_SIZE) != NULL;
}

static COM_RespStatus_t login(const uint8_t *args)
{
    char pw[COM_CTRL_PASS_SIZE];
    if (!pass_from_field(&args[COM_CTRL_LOGIN_PASS], pw) || pw[0] == '\0') { return COM_RESP_ERR; }
    return Rs485Cmd_CheckPassword(pw) ? COM_RESP_OK : COM_RESP_AUTH;
}

static COM_RespStatus_t set_pass(const uint8_t *args)
{
    char old_pw[COM_CTRL_PASS_SIZE];
    char new_pw[COM_CTRL_PASS_SIZE];
    if (!pass_from_field(&args[COM_CTRL_SET_PASS_OLD], old_pw)
        || !pass_from_field(&args[COM_CTRL_SET_PASS_NEW], new_pw)) { return COM_RESP_ERR; }

    switch (Rs485Cmd_ChangePassword(old_pw, new_pw)) {
    case RS485_PW_OK:        return COM_RESP_OK;
    case RS485_PW_WRONG_OLD: return COM_RESP_AUTH;
    default:                 return COM_RESP_ERR;
    }
}

/* ========================================================================== */
/* --- CONTROL --- */
/* ========================================================================== */

void COM_App_HandleControl(const uint8_t *req_payload, uint8_t *resp_payload)
{
    memset(resp_payload, 0, COM_PAYLOAD_SIZE);

    uint8_t  fsm_flags = req_payload[COM_CTRL_PAYLOAD_FSM];
    (void)fsm_flags; /* stato desiderato "a specchio": le transizioni reali
                       * restano guidate dall'opcode sotto, non da fsm_flags
                       * direttamente (stesso principio di rs485_cmd.c: un
                       * comando alla volta, non uno snapshot di stato). */
    const uint8_t *args = &req_payload[COM_CTRL_PAYLOAD_ARGS];
    uint8_t  power_pct = args[COM_CTRL_PAYLOAD_SW_POWER_SETPOINT];
    uint8_t  qcw_ctrl   = args[COM_CTRL_PAYLOAD_QCW_ERR];
    uint32_t qcw_freq_hz;
    memcpy(&qcw_freq_hz, &args[COM_CTRL_PAYLOAD_QCW_FREQ_HZ], sizeof qcw_freq_hz);
    uint8_t  qcw_duty = args[COM_CTRL_PAYLOAD_QCW_DC];
    uint8_t  mode     = args[COM_CTRL_PAYLOAD_MODE];
    uint8_t  hw_ctrl  = args[COM_CTRL_PAYLOAD_GATE_HW_SETPOINT_HW];
    /* Stessi byte dei campi sopra, letti come argomenti dagli opcode che non
     * usano i campi fissi (vedi COM_CTRL_PAYLOAD_ARGS). */
    COM_ControlOpcode_t opcode = (COM_ControlOpcode_t)req_payload[COM_CTRL_PAYLOAD_OPCODE];

    COM_RespStatus_t status = COM_RESP_OK;

    switch (opcode) {

    /* --- Sequenza FSM (stesse guardie di stato/modalita' di rs485_cmd.c) -- */
    case COM_CTRL_OP_START:
        if (FSM_GetMode() == FSM_MODE_ANALOG || FSM_GetState() != SYS_IDLE) { status = COM_RESP_ERR; break; }
        Queue_PostEvent(SYS_START_EVENT);
        break;
    case COM_CTRL_OP_STOP:
        if (FSM_GetMode() == FSM_MODE_ANALOG || FSM_GetState() != SYS_ACTIVE) { status = COM_RESP_ERR; break; }
        Queue_PostEvent(SYS_STOP_EVENT);
        break;
    case COM_CTRL_OP_SON:
        if (FSM_GetMode() == FSM_MODE_ANALOG || FSM_GetState() != SYS_ACTIVE) { status = COM_RESP_ERR; break; }
        Queue_PostEvent(SYS_ON_EVENT);
        break;
    case COM_CTRL_OP_SOFF: {
        SysState_t st = FSM_GetState();
        if (FSM_GetMode() == FSM_MODE_ANALOG
            || (st != SYS_ON && st != SYS_ENABLED && st != SYS_EMISSION)) { status = COM_RESP_ERR; break; }
        Queue_PostEvent(SYS_OFF_EVENT);
        break;
    }
    case COM_CTRL_OP_SEN:
        if (FSM_GetMode() == FSM_MODE_ANALOG || FSM_GetState() != SYS_ON
            || FSM_IsSabRearmBlocked()) { status = COM_RESP_ERR; break; }
        Queue_PostEvent(SYS_ENABLE_EVENT);
        break;
    case COM_CTRL_OP_SDIS: {
        SysState_t st = FSM_GetState();
        if (FSM_GetMode() == FSM_MODE_ANALOG
            || (st != SYS_ENABLED && st != SYS_EMISSION)) { status = COM_RESP_ERR; break; }
        Queue_PostEvent(SYS_DISABLE_EVENT);
        break;
    }
    case COM_CTRL_OP_PON:
        if (FSM_GetMode() == FSM_MODE_ANALOG || FSM_GetState() != SYS_ENABLED) { status = COM_RESP_ERR; break; }
        Queue_PostEvent(SYS_LASERON_EVENT);
        break;
    case COM_CTRL_OP_POFF:
        if (FSM_GetMode() == FSM_MODE_ANALOG || FSM_GetState() != SYS_EMISSION) { status = COM_RESP_ERR; break; }
        Queue_PostEvent(SYS_LASEROFF_EVENT);
        break;
    case COM_CTRL_OP_CERR:
        /* CERR e' valido in qualunque modalita' (stessa eccezione di rs485_cmd.c). */
        if (FSM_GetState() != SYS_ERROR) { status = COM_RESP_ERR; break; }
        Queue_PostEvent(SYS_CLR_ERR_EVENT);
        break;

    /* --- Modalita' / toggle HW --------------------------------------------- */
    case COM_CTRL_OP_SETMODE: {
        SysEvent_t ev;
        if      (mode == COM_CTRL_MODE_SW)     ev = SYS_SET_MODE_SW_EVENT;
        else if (mode == COM_CTRL_MODE_ANALOG) ev = SYS_SET_MODE_ANALOG_EVENT;
        else if (mode == COM_CTRL_MODE_HYBRID) ev = SYS_SET_MODE_HYBRID_EVENT;
        else { status = COM_RESP_ERR; break; }
        Queue_PostEvent(ev);
        break;
    }
    case COM_CTRL_OP_SETGATEHW:
        Queue_PostEvent((hw_ctrl & COM_CTRL_GATE_HW_ENABLE) ? SYS_SET_GATE_HW_ON_EVENT
                                                             : SYS_SET_GATE_HW_OFF_EVENT);
        break;
    case COM_CTRL_OP_SETSETPOINTHW:
        Queue_PostEvent((hw_ctrl & COM_CTRL_SETPOINT_HW_ENABLE) ? SYS_SET_SETPOINT_HW_ON_EVENT
                                                                 : SYS_SET_SETPOINT_HW_OFF_EVENT);
        break;

    /* --- Setpoint / QCW ----------------------------------------------------- */
    case COM_CTRL_OP_SETSETPOINT:
        if (FSM_GetMode() != FSM_MODE_SW || power_pct > 100U) { status = COM_RESP_ERR; break; }
        Setpoint_SetPct(power_pct);
        break;
    case COM_CTRL_OP_SET_FREQ:
        if (qcw_freq_hz < QCW_MIN_FREQ_HZ || qcw_freq_hz > QCW_MAX_FREQ_HZ) { status = COM_RESP_ERR; break; }
        g_config.qcw_freq_hz = (uint16_t)qcw_freq_hz;
        /* Vincolo Ton >= QCW_MIN_ON_TIME_US (vedi QCW.h e rs485_cmd.c "SET
         * FREQ", stessa logica): il duty gia' configurato potrebbe non
         * garantire piu' un Ton sufficiente alla nuova frequenza. Clamp
         * automatico del duty, freq_hz non toccata. */
        g_config.qcw_duty_pct = QCW_ClampDutyForMinOnTime(g_config.qcw_freq_hz, g_config.qcw_duty_pct);
        if (QCW_IsActive()) { QCW_SetParams(g_config.qcw_freq_hz, g_config.qcw_duty_pct); }
        break;
    case COM_CTRL_OP_SET_DUTY:
        if (qcw_duty < QCW_MIN_DUTY_PCT || qcw_duty > QCW_MAX_DUTY_PCT) { status = COM_RESP_ERR; break; }
        /* Vincolo Ton >= QCW_MIN_ON_TIME_US (vedi QCW.h e rs485_cmd.c "SET
         * DUTY", stessa logica): il pct richiesto e' nominalmente valido ma
         * alla frequenza gia' configurata potrebbe dare un Ton troppo corto. */
        g_config.qcw_duty_pct = QCW_ClampDutyForMinOnTime(g_config.qcw_freq_hz, qcw_duty);
        if (QCW_IsActive()) { QCW_SetParams(g_config.qcw_freq_hz, g_config.qcw_duty_pct); }
        break;
    case COM_CTRL_OP_SET_QCW: {
        bool want_on = (qcw_ctrl & COM_CTRL_QCW_ENABLE) != 0U;
        g_config.qcw_enabled = want_on ? 1U : 0U;
        bool live = (FSM_GetState() == SYS_EMISSION) && (FSM_GetMode() == FSM_MODE_SW);
        if (live) {
            if (want_on) {
                QCW_SetParams(g_config.qcw_freq_hz, g_config.qcw_duty_pct);
                QCW_Start();
            } else {
                QCW_Stop();
                BoardCtrl_GateMC_Open();
            }
        }
        break;
    }

    /*
     * Comandi protetti da login su RS485 (REQUIRE_AUTH()) che la COM
     * interface non ha ancora motivo di inviare -> rifiutati esplicitamente
     * con AUTH invece di essere eseguiti senza controllo.
     */
    case COM_CTRL_OP_FRST:
    case COM_CTRL_OP_SETCURRENT:
        status = COM_RESP_AUTH;
        break;

    /* --- Login e persistenza (vedi il banner sopra set_lut_gain()) --------- */
    case COM_CTRL_OP_LOGIN:
        status = login(args);
        break;
    case COM_CTRL_OP_LOGOUT:
        /* Nessuna sessione lato MMC per il canale COM: niente da chiudere. */
        break;
    case COM_CTRL_OP_SET_PASS:
        status = set_pass(args);
        break;
    case COM_CTRL_OP_SAVE_CONF:
        status = (Config_Save() == CONFIG_OK) ? COM_RESP_OK : COM_RESP_ERR;
        break;
    case COM_CTRL_OP_SAVE_LUT:
        status = LUT_Save() ? COM_RESP_OK : COM_RESP_ERR;
        break;
    case COM_CTRL_OP_RESET_LUT:
        LUT_ResetDefaults();
        break;

    /* --- LUT ---------------------------------------------------------------- */
    case COM_CTRL_OP_SETLUTGAIN:
        status = set_lut_gain(args);
        break;
    case COM_CTRL_OP_SETLUTVALID:
        status = set_lut_valid(args);
        break;
    case COM_CTRL_OP_SETLUTPOWER:
        status = set_lut_power(args);
        break;
    case COM_CTRL_OP_SETLUTSETPOINT:
        status = set_lut_setpoint(args);
        break;

    /* --- Maschere ----------------------------------------------------------- */
    case COM_CTRL_OP_SETERRMASK:
    case COM_CTRL_OP_SETWARNMASK:
    case COM_CTRL_OP_SETFAULTMASK:
    case COM_CTRL_OP_SETFAULTLATCH:
    case COM_CTRL_OP_SET_TEMP_MASK:
    case COM_CTRL_OP_SET_FLOW_MASK:
    case COM_CTRL_OP_SET_PSU_MASK:
    case COM_CTRL_OP_SET_CONTACTOR_MASK:
    case COM_CTRL_OP_SET_EFUSE_MASK:
    case COM_CTRL_OP_SET_PD_MASK:
        status = set_mask(opcode, args);
        break;

    /* --- Ritardi e mappa NTC ------------------------------------------------ */
    case COM_CTRL_OP_SET_DELAY:
        status = set_delay(args);
        break;
    case COM_CTRL_OP_SET_NTC_MAP:
        status = set_ntc_map(args);
        break;

    /* --- Soglie (argomenti da COM_CTRL_PAYLOAD_ARGS) ------------------------ */
    case COM_CTRL_OP_SET_TEMP_THR:
        status = set_temp_thr(args);
        break;
    case COM_CTRL_OP_SET_FLOW_THR:
        status = set_flow_thr(args);
        break;
    case COM_CTRL_OP_SET_HUM_THR:
        status = set_hum_thr(args);
        break;
    case COM_CTRL_OP_SET_DEW_THR:
        status = set_dew_thr(args);
        break;

    /*
     * TODO: comandi ad argomenti (tensione/corrente PSU, timestamp, RS485
     * term...) non ancora implementati lato MMC — il payload li veicola gia'
     * da COM_CTRL_PAYLOAD_ARGS (vedi COM_interface_protocol.h): rifiutati
     * esplicitamente.
     */
    case COM_CTRL_OP_SET_PSU_V:
    case COM_CTRL_OP_SETTIME:
    case COM_CTRL_OP_SETDATE:
    case COM_CTRL_OP_SETTERM:
        status = COM_RESP_ERR;
        break;

    default:
        status = COM_RESP_ERR;
        break;
    }

    resp_payload[0] = (uint8_t)status;
}

/* ========================================================================== */
/* --- CONFIG --- */
/* ========================================================================== */

void COM_App_HandleConfig(const uint8_t *req_payload, uint8_t *resp_payload)
{
    memset(resp_payload, 0, COM_PAYLOAD_SIZE);

    COM_ConfigPayload_t cfg;
    memcpy(&cfg, req_payload, sizeof cfg);

    /* Applicazione diretta a g_config — stesso set di campi di "SET ERR/WARN/
     * FAULT MASK", "SET PSU/FLOW/TEMP/PD MASK", "SET PSU VOLTAGE/CURRENT",
     * "SET CONTACTOR MASK" in rs485_cmd.c, in un solo colpo invece che comando
     * per comando (coerente con "messaggio di controllo/config solo se
     * modificato" delle note di riunione). */
    g_config.error_mask             = cfg.error_mask;
    g_config.warning_mask           = cfg.warn_mask;
    g_config.fault_mask             = cfg.fault_mask;
    g_config.psu_enabled_mask       = cfg.psu_mask;
    g_config.flow_sensor_enabled_mask = cfg.flow_mask;
    g_config.temp_sensor_enabled_mask = cfg.temp_mask;
    g_config.pd_mask                = cfg.pd_mask;
    g_config.psu_voltage_mv         = (int32_t)cfg.psu_voltage;
    g_config.psu_current_ma         = (int32_t)cfg.psu_current;
    g_config.contactor_enabled_mask = cfg.contactor_mask;

    /*
     * Persistenza immediata in flash (stesso schema di "SET TERM"/"SET
     * SETPOINT RANGE": un frame CONFIG rappresenta gia' una decisione esplicita
     * di service, non un valore provvisorio in attesa di "SAVE CONFIG").
     * COM_config_resp_t e' ancora un placeholder lato ALITE_COM ("TODO: define
     * response type", com.h): qui restituiamo 0 = OK, altrimenti il codice
     * Config_err_t (negativo) esteso a uint32.
     */
    Config_err_t err = Config_Save();
    put_u32(resp_payload, (err == CONFIG_OK) ? 0U : (uint32_t)err);
}
