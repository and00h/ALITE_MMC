/*
 * BoardCtrl.c
 *
 * Implementazione del driver per i segnali di controllo interni della scheda MMC.
 * Vedere BoardCtrl.h per la documentazione completa.
 */

#include "BoardCtrl.h"
#include "main.h"   /* Pin definitions: nGATE_MC_*, nGATE_HW_EN_*, HW_SETPOINT_SEL_*, nHW_SETPOINT_EN_*, nEXT_485_TERMINATION_* */
#include "config.h" /* g_config.termination */

/* ========================================================================== */
/* --- INIT --- */
/* ========================================================================== */

void BoardCtrl_Init(void)
{
    /*
     * Stato sicuro all'avvio:
     *   - Gate SW chiuso (nessuna emissione accidentale)
     *   - Gate HW path disabilitato
     *   - Selezione setpoint: EXT/utente (default; irrilevante in SW mode)
     *   - Terminazione RS485: applicata secondo la preferenza persistita
     *     in configurazione (g_config caricata da Config_Init(), chiamata
     *     in main.c prima di MX_FREERTOS_Init()/BoardCtrl_Init()).
     */
    BoardCtrl_GateMC_Close();
    BoardCtrl_GateHW_Disable();
    BoardCtrl_SetpointSel(false);
    BoardCtrl_SetTermination(g_config.termination != 0U);
}

/* ========================================================================== */
/* --- GATE MC --- */
/* ========================================================================== */

void BoardCtrl_GateMC_Close(void)
{
    /* nGATE_MC active LOW: HIGH = gate chiuso */
    HAL_GPIO_WritePin(nGATE_MC_GPIO_Port, nGATE_MC_Pin, GPIO_PIN_SET);
}

void BoardCtrl_GateMC_Open(void)
{
    /* nGATE_MC active LOW: LOW = gate aperto → emissione abilitata */
    HAL_GPIO_WritePin(nGATE_MC_GPIO_Port, nGATE_MC_Pin, GPIO_PIN_RESET);
}

/* ========================================================================== */
/* --- GATE HW PATH --- */
/* ========================================================================== */

void BoardCtrl_GateHW_Enable(void)
{
    /* nGATE_HW_EN active LOW: LOW = path HW abilitato */
    HAL_GPIO_WritePin(nGATE_HW_EN_GPIO_Port, nGATE_HW_EN_Pin, GPIO_PIN_RESET);
}

void BoardCtrl_GateHW_Disable(void)
{
    /* nGATE_HW_EN active LOW: HIGH = path HW disabilitato */
    HAL_GPIO_WritePin(nGATE_HW_EN_GPIO_Port, nGATE_HW_EN_Pin, GPIO_PIN_SET);
}

/* ========================================================================== */
/* --- SETPOINT SELECTION --- */
/* ========================================================================== */

void BoardCtrl_SetpointSel(bool use_amc_dac)
{
    /*
     * HW_SETPOINT_SEL active HIGH:
     *   LOW  (false) = setpoint da EXT/utente (LPWR_SET_ISO) → usato in SW e ANALOG
     *   HIGH (true)  = setpoint da DAC di AMC                → usato in HYBRID
     */
    HAL_GPIO_WritePin(HW_SETPOINT_SEL_GPIO_Port, HW_SETPOINT_SEL_Pin,
                      use_amc_dac ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

/* ========================================================================== */
/* --- TERMINAZIONE BUS RS485 --- */
/* ========================================================================== */

void BoardCtrl_SetTermination(bool inserted)
{
    /* nEXT_485_TERMINATION active LOW: LOW = terminazione inserita */
    HAL_GPIO_WritePin(nEXT_485_TERMINATION_GPIO_Port, nEXT_485_TERMINATION_Pin,
                      inserted ? GPIO_PIN_RESET : GPIO_PIN_SET);
}


