/*
 * BoardCtrl.h
 *
 * Driver per i segnali di controllo interni della scheda MMC.
 *
 * RESPONSABILITÀ:
 *   Astrae i GPIO di controllo del percorso laser che non appartengono
 *   a nessun altro driver periferico (PSU, LaseQ, SAB, ecc.).
 *   L'application layer (FSM) usa esclusivamente queste API senza
 *   mai chiamare HAL_GPIO_WritePin direttamente.
 *
 * SEGNALI GESTITI:
 *
 *   nGATE_MC       (PE12, active LOW, GPIO_Output)
 *     Gate software lato MCU. Quando LOW il path del gate SW è attivo
 *     e il segnale si propaga verso LaseQ.
 *     La FSM lo apre in SYS_EMISSION (modalità SW/HYBRID).
 *
 *   nGATE_HW_EN    (PE13, active LOW, GPIO_Output)
 *     Abilita il path hardware del gate.
 *     Quando LOW, il segnale nGATE_IN_iso (fino a 50kHz) si propaga
 *     direttamente a LaseQ bypassando la FSM (modalità ANALOG).
 *     Default: HIGH (disabilitato).
 *
 *   HW_SETPOINT_SEL  (PF14, active HIGH, GPIO_Output)
 *     Seleziona la sorgente del setpoint analogico verso LaseQ:
 *       LOW  (0) = setpoint da utente via EXT interface (LPWR_SET_ISO)
 *                  → usato in FSM_MODE_SW (irrilevante, LaseQ usa RS485)
 *                    e in FSM_MODE_ANALOG (setpoint hardware da EXT)
 *       HIGH (1) = setpoint generato dal DAC del modulo AMC
 *                  → usato in FSM_MODE_HYBRID
 *     Preferenza persistita in g_config.hw_setpoint_sel, modificabile a
 *     runtime via comando RS485 "SET SETPOINTCOMP ON|OFF" (senza login,
 *     applicato subito + persistito subito — vedi rs485_cmd.c), stesso
 *     schema di nEXT_485_TERMINATION sotto. Il comando aggiorna anche la
 *     copia live in fsm.c (FSM_SetHwSetpointSel()) riletta da
 *     action_enter_emission() e dall'heartbeat verso AMC (task_amc.c).
 *
 *   nHW_SETPOINT_EN  (PE11, GPIO_Output → GPIO_Input di AMC)
 *     Linea di notifica hardware diretta MMC→AMC (precauzione PCB).
 *     Attualmente non utilizzata: nessuna funzione applicativa assegnata.
 *     Non guidata da questo driver — pin lasciato nello stato di init CubeMX.
 *
 *   AMC_FAULT_N  (PE10, GPIO_Input, open-drain da AMC)
 *     Gestita da AMC.h / task_comms.c (polling ogni 200ms).
 *     Non di competenza di questo driver.
 *
 *   nEXT_485_TERMINATION  (PA8, active LOW, GPIO_Output)
 *     Inserisce/rimuove la resistenza di terminazione del bus RS485.
 *     LOW (0) = terminazione inserita. HIGH (1) = terminazione rimossa.
 *     Stato applicato in BoardCtrl_Init() a partire da g_config.termination
 *     (persistito in flash, modificabile a runtime via comando RS485
 *     "SET TERM ON|OFF", senza login — vedi rs485_cmd.c).
 *
 * INIT:
 *   BoardCtrl_Init() va chiamato in MX_FREERTOS_Init() prima dello scheduler.
 *   Porta tutti i segnali allo stato di reset sicuro e applica la
 *   preferenza di terminazione RS485 salvata in configurazione.
 */

#ifndef DRIVERS_BOARDCTRL_BOARDCTRL_H_
#define DRIVERS_BOARDCTRL_BOARDCTRL_H_

#include <stdbool.h>
#include <stdint.h>

/* ========================================================================== */
/* --- INIT --- */
/* ========================================================================== */

/**
 * @brief  Inizializza BoardCtrl: porta tutti i segnali allo stato sicuro.
 *           nGATE_MC        = HIGH (gate chiuso)
 *           nGATE_HW_EN     = HIGH (HW gate path disabilitato)
 *           HW_SETPOINT_SEL = LOW  (EXT setpoint, default sicuro)
 *         Da chiamare in MX_FREERTOS_Init() prima di avviare i task.
 */
void BoardCtrl_Init(void);

/* ========================================================================== */
/* --- GATE MC (software path) --- */
/* ========================================================================== */

/**
 * @brief  Chiude il gate software: nGATE_MC = HIGH.
 *         L'emissione laser via path SW è bloccata.
 */
void BoardCtrl_GateMC_Close(void);

/**
 * @brief  Apre il gate software: nGATE_MC = LOW.
 *         Abilita il path SW verso LaseQ.
 *         Usare solo in SYS_EMISSION (modalità SW o HYBRID).
 */
void BoardCtrl_GateMC_Open(void);

/* ========================================================================== */
/* --- GATE HW PATH (bypass analogico a 50kHz) --- */
/* ========================================================================== */

/**
 * @brief  Abilita il path hardware del gate: nGATE_HW_EN = LOW.
 *         nGATE_IN_iso si propaga direttamente a LaseQ (ANALOG mode).
 *         La FSM NON deve aprire/chiudere il gate MC mentre HW è abilitato.
 */
void BoardCtrl_GateHW_Enable(void);

/**
 * @brief  Disabilita il path hardware del gate: nGATE_HW_EN = HIGH.
 *         Blocca la propagazione del segnale analogico esterno.
 */
void BoardCtrl_GateHW_Disable(void);

/* ========================================================================== */
/* --- SETPOINT SELECTION --- */
/* ========================================================================== */

/**
 * @brief  Seleziona la sorgente del setpoint analogico verso LaseQ.
 *
 * @param  use_amc_dac
 *           false → HW_SETPOINT_SEL = LOW  (0): setpoint da EXT/utente (LPWR_SET_ISO)
 *                   Usare in FSM_MODE_SW (irrilevante) e FSM_MODE_ANALOG.
 *           true  → HW_SETPOINT_SEL = HIGH (1): setpoint dal DAC di AMC.
 *                   Usare in FSM_MODE_HYBRID.
 */
void BoardCtrl_SetpointSel(bool use_amc_dac);

/* ========================================================================== */
/* --- TERMINAZIONE BUS RS485 --- */
/* ========================================================================== */

/**
 * @brief  Inserisce o rimuove la terminazione del bus RS485.
 *
 * @param  inserted
 *           true  → nEXT_485_TERMINATION = LOW  (0): terminazione inserita.
 *           false → nEXT_485_TERMINATION = HIGH (1): terminazione rimossa.
 */
void BoardCtrl_SetTermination(bool inserted);

#endif /* DRIVERS_BOARDCTRL_BOARDCTRL_H_ */
