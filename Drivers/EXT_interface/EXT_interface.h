/*
 * EXT_interface.h
 *
 * Created on: 10 giu 2026
 * Author: lucamaggiotanasi
 *
 * DESCRIZIONE COMPONENTE:
 * Driver di gestione dell'interfaccia EXT.
 * Acquisisce gli ingressi digitali e analogici provenienti dal connettore
 * esterno e gestisce le uscite di stato verso il sistema remoto.
 */

#ifndef EXT_INTERFACE_EXT_INTERFACE_H_
#define EXT_INTERFACE_EXT_INTERFACE_H_

#include <stdbool.h>
#include <stdint.h>
#include "stm32h7xx_hal.h"

/**
 * @brief Struttura contenente lo stato completo dell'interfaccia EXT.
 */
typedef struct {

    /* ====================================================================== */
    /* INPUTS                                                                  */
    /* ====================================================================== */

    bool clr_err;
    bool aiming;
    bool enable;
    bool system_on;
    bool ext_control;
    bool vext_present;
    bool gate;

    uint32_t current_setpoint;
    uint32_t power_setpoint;

    /* ====================================================================== */
    /* OUTPUTS                                                                 */
    /* ====================================================================== */

    bool error;
    bool valve_status;
    bool emission_ready;
    bool aiming_on;
    bool cmd_ready;
    bool sys_ready;

    /* ====================================================================== */
    /* PARAMETERS                                                                 */
    /* ====================================================================== */
    bool reduced_current_setpoint;

} EXT_interface_t;

/* ========================================================================== */
/*--- FUNZIONI DI INIZIALIZZAZIONE ---*/
/* ========================================================================== */

/**
 * @brief Associa la periferica ADC utilizzata per l'acquisizione degli ingressi analogici.
 *
 * @param hadc Puntatore alla periferica ADC configurata in CubeMX.
 */
void EXT_Interface_Init(ADC_HandleTypeDef *hadc, bool reduced_current_setpoint);

/* ========================================================================== */
/*--- PARAMETRI ---*/
/* ========================================================================== */

/**
 * @brief Imposta a runtime il range del setpoint esterno (nREDUCED_SETPOINT_RANGE,
 *        active LOW) e aggiorna EXT_interface.reduced_current_setpoint.
 *
 * @param reduced
 *          false -> pin HIGH: range 10V (default di fabbrica)
 *          true  -> pin LOW : range 6V
 *
 *        Non persiste in flash: il chiamante (rs485_cmd.c, comando
 *        "SET SETPOINT RANGE 10V|6V") è responsabile di aggiornare
 *        g_config.reduced_setpoint_range e chiamare Config_Save(),
 *        stesso schema di BoardCtrl_SetTermination()/g_config.termination.
 */
void EXT_SetReducedSetpointRange(bool reduced);

/**
 * @brief Legge back lo stato REALE del pin nREDUCED_SETPOINT_RANGE (non la
 *        sola variabile in RAM) — stesso schema di WaterValve_IsOpen().
 *        Usato da "GET SETPOINT RANGE" (rs485_cmd.c) per riportare lo stato
 *        elettrico effettivo, normalmente allineato a
 *        g_config.reduced_setpoint_range.
 *
 * @retval true  = pin LOW:  range 6V (reduced)
 * @retval false = pin HIGH: range 10V (default di fabbrica)
 */
bool EXT_GetReducedSetpointRange(void);

/* ========================================================================== */
/*--- ACQUISIZIONE INGRESSI ---*/
/* ========================================================================== */

/**
 * @brief Aggiorna lo stato degli ingressi digitali e analogici.
 *
 * La funzione acquisisce:
 * - ingressi digitali isolati;
 * - presenza alimentazione esterna;
 * - setpoint analogici tramite ADC.
 *
 * @param GPIO_Pin Parametro riservato (attualmente non utilizzato).
 */
void EXT_AcquireIN(uint16_t GPIO_Pin);

/**
 * @brief Stato del segnale nVEXT_GOOD (alimentazione dell'interfaccia EXT),
 *        aggiornato ogni 5ms da EXT_AcquireIN() (task_inputs.c).
 *
 *        Usato da fsm.c per condizionare il passaggio in modalità ANALOG
 *        (SW->ANALOG, sia da pin nEXT_CTL_iso che da comando RS485
 *        "SET MODE ANALOG"): consentito SOLO se questa funzione ritorna
 *        true. SETPOINT HW / GATE HW ("HYBRID1"/"HYBRID2") NON sono
 *        soggetti a questo vincolo.
 *
 * @retval true  = nVEXT_GOOD basso (0): alimentazione EXT presente/corretta.
 * @retval false = nVEXT_GOOD alto (1): alimentazione EXT assente/non corretta.
 */
bool EXT_IsVextGood(void);

/**
 * @brief  Valore ADC grezzo del setpoint HW letto su PC5 (LPWR_SET_ISO,
 *         ADC2_INP8), aggiornato ogni 5ms da EXT_AcquireIN() (task_inputs.c)
 *         — stesso valore campionato in EXT_interface.power_setpoint.
 *
 *         Usato da rs485_cmd.c ("GET STATUS") e COM_interface_app.c
 *         (risposta STATUS) per convertire il setpoint HW in percentuale
 *         tramite la LUT di calibrazione dedicata (LUT_ConvertHwPowerToPct(),
 *         lut_manager.h) quando FSM_GetLaserModeWire() != 0 — vedi banner su
 *         LUT_HwPowerEntry_t.
 *
 * @retval Valore ADC2 grezzo [0-65535] (risoluzione 16 bit, vedi
 *         MX_ADC2_Init(), main.c).
 */
uint16_t EXT_GetPowerSetpointRaw(void);

/* ========================================================================== */
/*--- CONTROLLO USCITE ---*/
/* ========================================================================== */

/**
 * @brief Aggiorna lo stato della valvola acqua.
 */
void EXT_SetValveStatus(bool status);

/**
 * @brief Aggiorna il segnale di sistema pronto.
 */
void EXT_SetSysStatus(bool status);

/**
 * @brief Aggiorna il segnale di comando pronto.
 */
void EXT_SetCmdStatus(bool status);

/**
 * @brief Aggiorna il segnale di emissione pronta.
 */
void EXT_EmissionRdyStatus(bool status);

/**
 * @brief Aggiorna il segnale di errore.
 */
void EXT_SetError(bool status);

/* ========================================================================== */
/*--- CALLBACK HARDWARE ---*/
/* ========================================================================== */

/**
 * @brief Da inserire in HAL_GPIO_EXTI_Callback().
 *
 * Aggiorna i segnali dell'interfaccia EXT gestiti tramite interrupt.
 *
 * @param GPIO_Pin Pin che ha generato l'interrupt EXTI.
 */
void EXT_Interface_Callback(uint16_t GPIO_Pin);

#endif /* EXT_INTERFACE_EXT_INTERFACE_H_ */
