/*
 * task_amc.c
 *
 * Task dedicato al servicing di AMC (Analog Module Controller, STM32G473)
 * via USART6. NUOVO 2026-07-22 — vedi task_amc.h per la motivazione
 * (scorporato da task_comms.c per rendere il servicing AMC comparabile in
 * frequenza a quello di LaseQ, requisito della compensazione dinamica di
 * tensione PSU / gating del setpoint ascendente).
 *
 * STARTUP SEQUENCE (immutata rispetto alla vecchia posizione in
 * task_comms.c, spostata qui as-is + nuovo punto 2e):
 *   1. Configurazione AMC (CONFIG_SET: limiti PSU + flag/timing compensazione)
 *   2. Configurazione fotodiodi (CONFIG_PD)
 *   3. LUT soglie guadagno (CONFIG_GAIN_LUT x2: SW poi HW)
 *   4. LUT validazione PD (CONFIG_PD_VALID: 4 PD x N entry x 2 modi)
 *   5. LUT compensazione tensione (CONFIG_VOLTAGE_LUT x N, NUOVO 2026-07-22)
 *   6. Loop ciclico: heartbeat/status ogni LQ_TRANSMIT_WINDOW ms
 *
 * A differenza di task_comms.c (LaseQ), qui non c'e' un delay di
 * assestamento power-on dedicato: AMC e' un MCU sulla stessa scheda MMC,
 * non richiede un tempo di boot separato comparabile a quello di LaseQ
 * (alimentato via RS485/cavo esterno). I retry sulle singole configurazioni
 * tollerano comunque un AMC non ancora pronto al primissimo tentativo.
 */

#include "FreeRTOS.h"
#include "task.h"
#include "task_amc.h"
#include "queues.h"
#include "fsm.h"
#include "AMC.h"
#include "LaseQ.h"          /* LQ_TRANSMIT_WINDOW: periodo del loop di questo task */
#include "task_monitor.h"
#include "config.h"
#include <stdbool.h>
#include "setpoint.h"
#include "lut_manager.h"
#include "Watchdog.h"
#include "sys_log.h"
#include "QCW.h"

/* ========================================================================== */
/* --- ALLOCAZIONE STATICA --- */
/* ========================================================================== */

static StaticTask_t taskAmc_tcb;
static StackType_t  taskAmc_stack[512];

const osThreadAttr_t taskAmc_attr = {
    .name       = "AMC",
    .stack_mem  = &taskAmc_stack[0],
    .stack_size = sizeof(taskAmc_stack),
    .priority   = osPriorityNormal2,
    .cb_mem     = &taskAmc_tcb,
    .cb_size    = sizeof(taskAmc_tcb),
};

/*
 * Il loop di questo task e' scandito da LQ_TRANSMIT_WINDOW (20ms, LaseQ.h),
 * NON da AMC_HEARTBEAT_PERIOD_MS: quest'ultima resta nel protocollo
 * condiviso (AMC_protocol.h) solo per documentazione. Il vincolo di
 * sequenza MMC->AMC->LaseQ (vedi task_amc.h) richiede che i due periodi
 * restino allineati: se in futuro dovessero divergere, questo assert lo
 * segnala a compile-time invece che con un comportamento silenzioso.
 */
_Static_assert(AMC_HEARTBEAT_PERIOD_MS == LQ_TRANSMIT_WINDOW,
               "AMC_HEARTBEAT_PERIOD_MS deve restare allineato a LQ_TRANSMIT_WINDOW "
               "(task AMC dedicato, vedi task_amc.h)");

/* ========================================================================== */
/* --- POTENZA FOTODIODI (layer applicativo: LUT conversion) --- */
/* ========================================================================== */

/*
 * Calcolata da questo task dopo ogni AMC_Transact() riuscito.
 * Accessibile agli altri moduli tramite TaskAmc_GetPDPower().
 */
static uint16_t s_pd_power_w[4] = {0U, 0U, 0U, 0U};

void TaskAmc_GetPDPower(uint16_t out_w[4])
{
    for (uint8_t p = 0U; p < 4U; p++) {
        out_w[p] = s_pd_power_w[p];
    }
}

/* ========================================================================== */
/* --- RICHIESTE RE-INVIO CONFIG/LUT AD AMC (da rs485_cmd.c) --- */
/* ========================================================================== */

static volatile bool s_psu_cfg_resend_pending   = false;
static volatile bool s_vcomp_lut_resend_pending  = false;

void TaskAmc_RequestPSUConfigResend(void)
{
    s_psu_cfg_resend_pending = true;
}

void TaskAmc_RequestVoltageCompLUTResend(void)
{
    s_vcomp_lut_resend_pending = true;
}

/* ========================================================================== */
/* --- TASK --- */
/* ========================================================================== */

void TaskAmc_Run(void *arg)
{
    (void)arg;

    /* ------------------------------------------------------------------ */
    /* 1. Configurazione AMC (una tantum, limiti PSU + compensazione)     */
    /* ------------------------------------------------------------------ */
    /*
     * g_config letto qui una sola volta per estrarre i limiti PSU e i
     * parametri di compensazione (parametri di calibrazione, non cambiano
     * durante l'esecuzione normale se non tramite TaskAmc_RequestPSUConfigResend(),
     * gestita nel loop sotto). psu_voltage_mv/psu_current_ma sono int32_t
     * in config (possono essere negativi solo per errori di calibrazione),
     * clampati a uint16_t.
     */
    {
        uint16_t v_mv = (g_config.psu_voltage_mv > 0 && g_config.psu_voltage_mv <= 0xFFFFU)
                        ? (uint16_t)g_config.psu_voltage_mv : 0U;
        uint16_t i_ma = (g_config.psu_current_ma > 0 && g_config.psu_current_ma <= 0xFFFFU)
                        ? (uint16_t)g_config.psu_current_ma : 0U;

        /* Fino a 3 tentativi — AMC potrebbe non essere ancora pronto */
        uint8_t cfg_retries = 3;
        AMC_err_t cfg_err   = AMC_ERR_TIMEOUT;
        while (cfg_retries-- > 0) {
            cfg_err = AMC_SendConfig(v_mv, i_ma,
                                      g_config.voltage_comp_enabled,
                                      g_config.comp_stabilize_delay_ms,
                                      g_config.comp_ramp_duration_ms,
                                      g_config.comp_ramp_up_duration_ms);
            if (cfg_err == AMC_OK || cfg_err == AMC_ERR_DISABLED) break;
            osDelay(100);
        }
        /*
         * Se AMC non risponde alla config, non blocchiamo l'avvio: AMC_FAULT
         * sara' rilevato nel loop dal miss count heartbeat. (AMC_ERR_DISABLED
         * = AMC non configurato in hw = nessun fault)
         */
    }

    /* ------------------------------------------------------------------ */
    /* 2. Configurazione fotodiodi AMC (CONFIG_PD + LUT)                  */
    /* ------------------------------------------------------------------ */
    /*
     * Inviato solo se CONFIG_SET e' andato a buon fine (AMC in stato CFG).
     * Le LUT di riferimento sono a default [0..4095] (accetta tutto): in
     * futuro potranno essere configurate da GUI tramite procedura service.
     * Fino ad allora AMC validera' sempre OK qualunque lettura PD.
     *
     * NOTA: pd_gain_threshold e' un array uint16_t[4] in g_config.
     * Il cast al puntatore const uint16_t* e' legale grazie a #pragma pack.
     */
    {
        uint8_t pd_retries = 3;
        AMC_err_t pd_err   = AMC_ERR_TIMEOUT;
        while (pd_retries-- > 0) {
            pd_err = AMC_SendPDConfig(
                g_config.pd_mask,
                g_config.pd_gain_windows,
                g_config.pd_gain_settle_ms,
                g_config.pd_stability_samples,
                g_config.pd_stability_threshold
            );
            if (pd_err == AMC_OK || pd_err == AMC_ERR_DISABLED) break;
            osDelay(100);
        }

        /* -------------------------------------------------------------- */
        /* 2b. LUT soglie guadagno (CONFIG_GAIN_LUT x2: SW + HW)          */
        /* -------------------------------------------------------------- */
        {
            const LUT_Store_t *lut = LUT_Get();

            /* SW mode: soglie basate su current_ma */
            for (uint8_t r = 0; r < 3; r++) {
                AMC_err_t e = AMC_SendGainLUT(0U, lut->gain_threshold_sw);
                if (e == AMC_OK || e == AMC_ERR_DISABLED) break;
                osDelay(50);
            }

            /* HW mode: soglie basate su pa0_adc */
            for (uint8_t r = 0; r < 3; r++) {
                AMC_err_t e = AMC_SendGainLUT(1U, lut->gain_threshold_hw);
                if (e == AMC_OK || e == AMC_ERR_DISABLED) break;
                osDelay(50);
            }
        }

        /* -------------------------------------------------------------- */
        /* 2c. LUT validazione PD (CONFIG_PD_VALID: 4 PD x N entry x 2 modi) */
        /* -------------------------------------------------------------- */
        {
            const LUT_Store_t *lut = LUT_Get();
            uint8_t n = lut->pd_valid_size;

            for (uint8_t mode = 0; mode < 2U; mode++) {
                for (uint8_t pd = 0; pd < LUT_PD_MAX; pd++) {
                    for (uint8_t e = 0; e < n; e++) {
                        const LUT_PDValidEntry_t *entry =
                            &lut->pd_valid[mode][pd][e];
                        uint8_t retries = 3;
                        while (retries-- > 0) {
                            AMC_err_t err = AMC_SendPDValidEntry(
                                pd, mode, e, n,
                                entry->setpoint,
                                entry->pd_min,
                                entry->pd_max
                            );
                            if (err == AMC_OK || err == AMC_ERR_DISABLED) break;
                            osDelay(20);
                        }
                    }
                }
            }
        }

        /* -------------------------------------------------------------- */
        /* 2d. LUT compensazione tensione PSU (CONFIG_VOLTAGE_LUT,        */
        /*     NUOVO 2026-07-22): mA -> mV, entry in ordine crescente di  */
        /*     current_ma — vedi lut_manager.h/LUT_VoltageCompEntry_t.    */
        /* -------------------------------------------------------------- */
        {
            const LUT_Store_t *lut = LUT_Get();
            uint8_t n = lut->voltage_comp_size;

            for (uint8_t e = 0; e < n; e++) {
                const LUT_VoltageCompEntry_t *entry = &lut->voltage_comp[e];
                uint8_t retries = 3;
                while (retries-- > 0) {
                    AMC_err_t err = AMC_SendVoltageCompLUTEntry(
                        e, n, entry->current_ma, entry->voltage_mv);
                    if (err == AMC_OK || err == AMC_ERR_DISABLED) break;
                    osDelay(20);
                }
            }
        }
    }

    /* ------------------------------------------------------------------ */
    /* 3. Loop principale — ogni LQ_TRANSMIT_WINDOW ms (20ms)             */
    /* ------------------------------------------------------------------ */
    TickType_t xLastWakeTime = xTaskGetTickCount();

    uint8_t amc_miss_count   = 0;
    bool    amc_fault_posted = false;

    for (;;)
    {
        Watchdog_Heartbeat(WDG_TASK_AMC);

        /* -------------------------------------------------------------- */
        /* 3a. Polling PE10 (AMC_FAULT_N, open-drain attivo basso)        */
        /* -------------------------------------------------------------- */
        if (AMC_ReadFaultPin() && !amc_fault_posted &&
            (g_config.fault_mask & FAULT_BIT_AMC)) {
            Queue_PostEvent(SYS_AMC_FAULT_EVENT);
            amc_fault_posted = true;
            TaskMonitor_SetFaultBit(FAULT_BIT_AMC, true);
        }

        /* -------------------------------------------------------------- */
        /* 3b. Re-invio limiti PSU + flag/timing compensazione a richiesta */
        /* -------------------------------------------------------------- */
        /*
         * Vedi TaskAmc_RequestPSUConfigResend() in task_amc.h. Il gate
         * sullo stato FSM (PSU spento) e' gia' stato fatto dal chiamante
         * in rs485_cmd.c; qui rilettura difensiva: se nel frattempo il
         * sistema e' uscito da IDLE, si salta e si ritenta al prossimo
         * ciclo (il flag resta pending).
         */
        if (s_psu_cfg_resend_pending) {
            if (FSM_GetState() == SYS_IDLE) {
                s_psu_cfg_resend_pending = false;

                uint16_t cfg_v_mv = (g_config.psu_voltage_mv > 0 && g_config.psu_voltage_mv <= 0xFFFFU)
                                    ? (uint16_t)g_config.psu_voltage_mv : 0U;
                uint16_t cfg_i_ma = (g_config.psu_current_ma > 0 && g_config.psu_current_ma <= 0xFFFFU)
                                    ? (uint16_t)g_config.psu_current_ma : 0U;

                AMC_err_t cfg_resend_err = AMC_SendConfig(
                    cfg_v_mv, cfg_i_ma,
                    g_config.voltage_comp_enabled,
                    g_config.comp_stabilize_delay_ms,
                    g_config.comp_ramp_duration_ms,
                    g_config.comp_ramp_up_duration_ms);
                if (cfg_resend_err == AMC_OK || cfg_resend_err == AMC_ERR_DISABLED) {
                    SysLog_Event(LOG_INFO,
                                 "AMC: limiti PSU/comp aggiornati (V=%umV I=%umA comp=%u)",
                                 cfg_v_mv, cfg_i_ma, (unsigned)g_config.voltage_comp_enabled);
                } else {
                    SysLog_Event(LOG_WARN, "AMC: re-invio limiti PSU fallito (err=%d)",
                                 (int)cfg_resend_err);
                }
            }
            /* Se non IDLE: flag resta true, si ritenta al prossimo ciclo. */
        }

        /* -------------------------------------------------------------- */
        /* 3c. Re-invio LUT compensazione tensione a richiesta            */
        /* -------------------------------------------------------------- */
        /* Vedi TaskAmc_RequestVoltageCompLUTResend() in task_amc.h — stesso
         * gate SYS_IDLE del punto 3b. */
        if (s_vcomp_lut_resend_pending) {
            if (FSM_GetState() == SYS_IDLE) {
                s_vcomp_lut_resend_pending = false;

                const LUT_Store_t *lut = LUT_Get();
                uint8_t n = lut->voltage_comp_size;
                bool    all_ok = true;

                for (uint8_t e = 0; e < n; e++) {
                    const LUT_VoltageCompEntry_t *entry = &lut->voltage_comp[e];
                    AMC_err_t err = AMC_SendVoltageCompLUTEntry(
                        e, n, entry->current_ma, entry->voltage_mv);
                    if (err != AMC_OK && err != AMC_ERR_DISABLED) {
                        all_ok = false;
                    }
                }

                if (all_ok) {
                    SysLog_Event(LOG_INFO,
                                 "AMC: LUT compensazione tensione aggiornata (%u entry)", n);
                } else {
                    SysLog_Event(LOG_WARN,
                                 "AMC: re-invio LUT compensazione tensione parzialmente fallito");
                }
            }
        }

        /* -------------------------------------------------------------- */
        /* 3d. Heartbeat/status                                            */
        /* -------------------------------------------------------------- */
        FSM_Mode_t mode   = FSM_GetMode();
        /*
         * hw_sel indica ad AMC se HW_SETPOINT_SEL e' alto (= AMC DAC attivo).
         * Deriva da s_hw_setpoint_sel in fsm.c (letto da g_config all'init,
         * poi aggiornabile a runtime via comando RS485 "SET SETPOINTCOMP
         * ON|OFF", vedi FSM_SetHwSetpointSel()), NON dalla modalita' FSM.
         * Default fabbrica 0 (AMC DAC non ancora impiegato in produzione).
         */
        uint8_t hw_sel = FSM_GetHwSetpointSel();

        /* Aggiorna setpoint corrente in AMC_Transact (SW mode: current_ma; HW: 0) */
        if (mode == FSM_MODE_SW) {
            AMC_SetCurrentSetpoint((uint16_t)Setpoint_GetCurrentMa());
        } else {
            AMC_SetCurrentSetpoint(0U);
        }

        /*
         * qcw_active: vedi banner AMC_PayloadHeartbeat_t (AMC_protocol.h).
         * QCW_IsActive() riflette g_config.qcw_enabled + emissione SW in
         * corso (impostato da QCW_Start()/azzerato da QCW_Stop(), guidati
         * dalla FSM in SYS_EMISSION) — serve alla compensazione dinamica
         * lato AMC (psu_voltage_comp.c) per restare disattiva durante QCW,
         * dove LASE_Q_OUT_5V si accende/spegne al ritmo dell'impulsazione.
         */
        uint8_t qcw_active = QCW_IsActive() ? 1U : 0U;

        /*
         * setpoint_hw_enabled: stato LIVE del toggle RS485 "SET SETPOINTHW
         * ON|OFF" (FSM_GetSetpointHwEnabled(), "HYBRID2") — DA NON
         * CONFONDERE con hw_sel/FSM_GetHwSetpointSel() sopra, che è un
         * instradamento hardware statico diverso (vedi banner
         * AMC_PayloadHeartbeat_t in AMC_protocol.h). Serve alla
         * compensazione dinamica lato AMC (psu_voltage_comp.c) per restare
         * disattiva quando il riferimento di corrente è instradato dal pin
         * analogico esterno (cambi istantanei/imprevedibili).
         */
        uint8_t setpoint_hw_enabled = FSM_GetSetpointHwEnabled() ? 1U : 0U;

        /*
         * NON castare "mode" direttamente: FSM_Mode_t non ha piu' un valore
         * HYBRID dedicato (vedi fsm.h, "MODALITA' OPERATIVA"), ma il
         * protocollo AMC (AMC_protocol.h, condiviso con la board AMC)
         * documenta ancora 0=SW/1=HYBRID/2=ANALOG per il campo laser_mode.
         * FSM_GetLaserModeWire() mantiene questa codifica storica
         * indipendentemente dalla numerazione interna di FSM_Mode_t (1 se
         * FSM_MODE_SW con toggle "HYBRID2"/setpoint HW attivo — vedi
         * FSM_GetSetpointHwEnabled()).
         */
        AMC_err_t amc_err = AMC_Transact(FSM_GetLaserModeWire(), hw_sel, qcw_active,
                                         setpoint_hw_enabled);

        if (amc_err == AMC_OK) {
            /* Conversione ADC->W tramite LUT (layer applicativo) */
            uint16_t raw[4];
            AMC_GetPDRaw(raw);
            uint8_t win = AMC_GetGainWindow();
            for (uint8_t p = 0U; p < 4U; p++) {
                /*
                 * pd_mask (g_config, bit0=PD1..bit3=PD4): device-enable
                 * dedicato ("photodiode mask", stesso pattern di
                 * psu_enabled_mask/contactor_enabled_mask) — un PD
                 * disabilitato viene riportato a 0W invece di propagare
                 * una lettura di un canale non installato/non calibrato.
                 * Il mask resta comunque inviato ad AMC via MSG_CONFIG_PD
                 * all'avvio (vedi punto 2 sopra) e aggiornabile a runtime
                 * con "SET PD MASK" (rs485_cmd.c, richiede reset AMC per
                 * essere ri-applicato lato AMC — solo il lato MMC/display
                 * e' immediato).
                 */
                s_pd_power_w[p] = (g_config.pd_mask & (1U << p))
                                   ? LUT_ConvertPDToWatt(p, win, raw[p])
                                   : 0U;
            }
            amc_miss_count   = 0;
            amc_fault_posted = false;
            TaskMonitor_SetFaultBit(FAULT_BIT_AMC, false);
        } else if (amc_err != AMC_ERR_DISABLED) {
            amc_miss_count++;
            /* 50 miss × 20ms = 1s prima del fault (era 5 x 200ms prima del
             * 2026-07-22, stessa finestra temporale — vedi AMC_protocol.h).
             * Mascherabile con FAULT_BIT_AMC (g_config.fault_mask). */
            if (amc_miss_count >= AMC_MAX_MISS_COUNT && !amc_fault_posted &&
                (g_config.fault_mask & FAULT_BIT_AMC)) {
                Queue_PostEvent(SYS_AMC_FAULT_EVENT);
                amc_fault_posted = true;
            }
            TaskMonitor_SetFaultBit(FAULT_BIT_AMC, amc_miss_count >= AMC_MAX_MISS_COUNT);
        }

        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(LQ_TRANSMIT_WINDOW));
    }
}
