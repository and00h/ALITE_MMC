/*
 * EXT_interface.c
 *
 * Created on: 10 giu 2026
 * Author: lucamaggiotanasi
 */

#include "EXT_interface.h"
#include "main.h"

/* ========================================================================== */
/*--- VARIABILI PRIVATE ---*/
/* ========================================================================== */

EXT_interface_t EXT_interface = {0};

static ADC_HandleTypeDef *EXT_adc = NULL;

/* ========================================================================== */
/*--- FUNZIONI DI INIZIALIZZAZIONE ---*/
/* ========================================================================== */

void EXT_Interface_Init(ADC_HandleTypeDef *hadc, bool reduced_current_setpoint)
{
	//Associazione ADC
    EXT_adc = hadc;

    //Impostazione del current setpoint range secondo la preferenza persistita
    //in configurazione (g_config.reduced_setpoint_range, caricata da
    //Config_Init() prima di questa chiamata in main.c).
    EXT_SetReducedSetpointRange(reduced_current_setpoint);
}

/* ========================================================================== */
/*--- PARAMETRI ---*/
/* ========================================================================== */

void EXT_SetReducedSetpointRange(bool reduced)
{
    /* nREDUCED_SETPOINT_RANGE active LOW: LOW = range 6V, HIGH = range 10V */
    HAL_GPIO_WritePin(nREDUCED_SETPOINT_RANGE_GPIO_Port, nREDUCED_SETPOINT_RANGE_Pin, !reduced);
    EXT_interface.reduced_current_setpoint = reduced;
}

bool EXT_GetReducedSetpointRange(void)
{
    /* Letto back dal pin reale (stesso schema di WaterValve_IsOpen()), non
     * dalla sola variabile in RAM: riflette lo stato elettrico EFFETTIVO,
     * normalmente allineato a g_config.reduced_setpoint_range perché
     * EXT_SetReducedSetpointRange() applica sempre i due insieme (vedi
     * "SET SETPOINT RANGE", rs485_cmd.c). */
    return (HAL_GPIO_ReadPin(nREDUCED_SETPOINT_RANGE_GPIO_Port, nREDUCED_SETPOINT_RANGE_Pin) == GPIO_PIN_RESET);
}

/* ========================================================================== */
/*--- ACQUISIZIONE INGRESSI ---*/
/* ========================================================================== */

void EXT_AcquireIN(uint16_t GPIO_Pin)
{
    (void)GPIO_Pin;

    /* Acquisizione ingressi digitali */

    EXT_interface.clr_err = !HAL_GPIO_ReadPin(nCLR_ERR_iso_GPIO_Port, nCLR_ERR_iso_Pin);

    /* AIMING: non implementato in questo branch — il pin rimane nello stato
     * impostato da GPIO_Init (pull-up/pull-down, mai scritto attivamente).
     * Il campo è forzato a false per evitare comportamenti inattesi se
     * qualcuno controllasse il valore senza guardare AIMING_ENABLE. */
    EXT_interface.aiming = false;   /* AIMING_ENABLE = 0 */

    EXT_interface.enable = !HAL_GPIO_ReadPin(nENABLE_IN_iso_GPIO_Port, nENABLE_IN_iso_Pin);

    EXT_interface.system_on = !HAL_GPIO_ReadPin(nSYS_ON_iso_GPIO_Port, nSYS_ON_iso_Pin);

    EXT_interface.ext_control = !HAL_GPIO_ReadPin(nEXT_CTL_iso_GPIO_Port, nEXT_CTL_iso_Pin);

    EXT_interface.vext_present = !HAL_GPIO_ReadPin(nVEXT_GOOD_GPIO_Port, nVEXT_GOOD_Pin);

    /* Acquisizione ingressi analogici */

    HAL_ADC_Start(EXT_adc);

    if (HAL_ADC_PollForConversion(EXT_adc, 8) == HAL_OK)
    {
        uint32_t raw_value = HAL_ADC_GetValue(EXT_adc);

        uint32_t current_value = raw_value;
        uint32_t power_value   = raw_value;

        EXT_interface.current_setpoint = current_value;
        EXT_interface.power_setpoint   = power_value;
    }

    HAL_ADC_Stop(EXT_adc);
}

bool EXT_IsVextGood(void)
{
    return EXT_interface.vext_present;
}

uint16_t EXT_GetPowerSetpointRaw(void)
{
    /* power_setpoint e' un uint32_t solo per uniformita' col resto della
     * struct, ma HAL_ADC_GetValue() su ADC2 (risoluzione 16 bit) non
     * eccede mai 0xFFFF: cast diretto, nessun clamp necessario. */
    return (uint16_t)EXT_interface.power_setpoint;
}

/* ========================================================================== */
/*--- CONTROLLO USCITE ---*/
/* ========================================================================== */

void EXT_SetValveStatus(bool status)
{
    HAL_GPIO_WritePin(nWATER_VALVE_STATUS_GPIO_Port,nWATER_VALVE_STATUS_Pin,!status);
    EXT_interface.valve_status = status;
}

void EXT_SetSysStatus(bool status)
{
    HAL_GPIO_WritePin(nREADY_iso_GPIO_Port,nREADY_iso_Pin,!status);
    EXT_interface.sys_ready = status;
}

void EXT_SetCmdStatus(bool status)
{
    HAL_GPIO_WritePin(nCMD_RDY_iso_GPIO_Port,nCMD_RDY_iso_Pin,!status);
    EXT_interface.cmd_ready = status;
}

void EXT_EmissionRdyStatus(bool status)
{
    HAL_GPIO_WritePin(nEMISSION_RDY_iso_GPIO_Port,nEMISSION_RDY_iso_Pin,!status);
    EXT_interface.emission_ready = status;
}

void EXT_SetError(bool status)
{
    HAL_GPIO_WritePin(nERROR_iso_GPIO_Port,nERROR_iso_Pin,!status);
    EXT_interface.error = status;
}

/* ========================================================================== */
/*--- CALLBACK HARDWARE ---*/
/* ========================================================================== */

void EXT_Interface_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == nGATE_IN_iso_Pin)
    {
        EXT_interface.gate =
                !HAL_GPIO_ReadPin(
                        nGATE_IN_iso_GPIO_Port,
                        nGATE_IN_iso_Pin);
    }
}
