/*
 * Contactors.h
 *
 *  Created on: 10 giu 2026
 *      Author: lucamaggiotanasi
 */

#ifndef CONTACTORS_CONTACTORS_H_
#define CONTACTORS_CONTACTORS_H_

/*
 * PSU.h
 *
 * Created on: 28 mag 2026
 * Author: lucamaggiotanasi
 */

#include "stm32h7xx_hal.h"
#include <stdbool.h>

/**
 * @brief Enumerazione per l'identificazione dei contattori.
 */
typedef enum{
	CONTACTOR1,               /**< Istanza Contattore 1 */
	CONTACTOR2                /**< Istanza Contattore 2 */
}CONTACTOR_id_t;

/**
 * @brief Struttura dati che rappresenta lo stato di un singolo contattore.
 */
typedef struct{
	CONTACTOR_id_t id;
	bool control;	//Segnale di controllo
    bool status;	//Stato del contatto ausiliare
} CONTACTOR_t;

/* --- PROTOTIPI DELLE FUNZIONI INTERNE ED ESTERNE --- */

/**
 * @brief Attiva il contattore selezionato agendo sul GPIO di ON/OFF.
 *
 * @note  No-op sul GPIO se il contattore è mascherato come "non collegato"
 *        in g_config.contactor_enabled_mask (bit0=CONTACTOR1, bit1=CONTACTOR2,
 *        modificabile via "SET CONTACTOR MASK" su RS485) — a prescindere dal
 *        chiamante, non solo da contactors_on_masked() in fsm.c.
 */
void ContactorTurnOn(CONTACTOR_id_t contactor_id);

/**
 * @brief Disattiva il contattore selezionato agendo sul GPIO di ON/OFF.
 */
void ContactorTurnOff(CONTACTOR_id_t contactor_id);

/**
 * @brief Restituisce una copia dello stato dell'istanza del contattore selezionato.
 *
 * @note  Se il contattore è mascherato come "non collegato" in
 *        g_config.contactor_enabled_mask, il pin di stato ausiliario NON
 *        viene letto (verosimilmente floating/non cablato): status torna
 *        sempre false invece di propagare un valore indefinito.
 */
CONTACTOR_t ContactorGetStatus(CONTACTOR_id_t contactor_id);


#endif /* CONTACTORS_CONTACTORS_H_ */
