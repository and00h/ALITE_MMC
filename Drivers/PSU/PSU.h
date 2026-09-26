/*
 * PSU.h
 *
 * Created on: 28 mag 2026
 * Author: lucamaggiotanasi
 */

#ifndef PSU_PSU_H_
#define PSU_PSU_H_

#include "stm32h7xx_hal.h"
#include <stdbool.h>

/**
 * @brief Enumerazione per l'identificazione degli alimentatori.
 */
typedef enum{
	PSU1,               /**< Istanza Alimentatore 1 */
	PSU2                /**< Istanza Alimentatore 2 */
}PSU_id_t;

/**
 * @brief Enumerazione dei codici di errore del driver.
 */
typedef enum{
	PSU_OK,             /**< Nessun errore riscontrato */
	PSU_ID_ERR,         /**< Errore: ID Alimentatore fuori range */
	PSU_ALARM_ERR,      /**< Errore: Stato di allarme hardware attivo */
}PSU_error_t;

/**
 * @brief Struttura dati che rappresenta lo stato di un singolo alimentatore.
 */
typedef struct{
	PSU_id_t id;			//PSU identificator
    uint32_t bus_voltage;   //Voltage measured

    int32_t voltage_prog;   //Voltage programming value
    int32_t current_prog;   //Current programming value

    bool dc_ok;    			//DC OK flag
    bool alarm;        		//ALARM flag

    bool status;			//PSU power flag
    PSU_error_t error;          // Stato di errore
} PSU_t;

/* --- PROTOTIPI DELLE FUNZIONI INTERNE ED ESTERNE --- */

/**
 * @brief Inizializza l'alimentatore specificato spegnendolo e azzerando i riferimenti.
 */
PSU_t PSUInit(PSU_id_t psu_id);

/**
 * @brief Imposta globalmente il valore di tensione (PV) per entrambi gli alimentatori.
 */
void PSUVoltageSet(uint32_t value);

/**
 * @brief Imposta globalmente il limite di corrente (PC) per entrambi gli alimentatori.
 */
void PSUCurrentSet(uint32_t value);

/**
 * @brief Attiva l'uscita dell'alimentatore selezionato agendo sul GPIO di ON/OFF.
 */
void PSUTurnOn(PSU_id_t psu_id);

/**
 * @brief Disattiva l'uscita dell'alimentatore selezionato agendo sul GPIO di ON/OFF.
 */
void PSUTurnOff(PSU_id_t psu_id);

/**
 * @brief Restituisce una copia dello stato dell'istanza dell'alimentatore selezionato.
 */
PSU_t PSUGetStatus(PSU_id_t psu_id);

/**
 * @brief Aggiorna automaticamente i flag di alarm delle entità alimentatore.
 */
void PSU_Interface_Callback(uint16_t GPIO_Pin);


#endif /* PSU_PSU_H_ */
