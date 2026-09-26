/*
 * Contactors.c
 *
 *  Created on: 10 giu 2026
 *      Author: lucamaggiotanasi
 */

/**
 * @brief Array statico privato per l'incapsulamento dei dati dei due contattori.
 */
#include "Contactors.h"
#include "main.h"
#include "config.h"   /* g_config.contactor_enabled_mask */

static CONTACTOR_t CONTACTOR[2] = {
    {
        .id = CONTACTOR1,
		.control = 0,
        .status = 0,
    },
    {
        .id = CONTACTOR2,
		.control = 0,
        .status = 0,
    }
};

/*
 * @brief Contattore "collegato" secondo g_config.contactor_enabled_mask
 *        (bit0=CONTACTOR1, bit1=CONTACTOR2 — SET CONTACTOR MASK su RS485).
 *        Un contattore NON collegato non va mai acceso (GPIO ON non cablato
 *        o non presente) e il suo pin di stato ausiliario non va mai letto
 *        (verosimilmente floating).
 */
static inline bool contactor_is_connected(CONTACTOR_id_t contactor_id)
{
    return (g_config.contactor_enabled_mask & (1U << (uint8_t)contactor_id)) != 0U;
}


/**
 * @brief Accende l'alimentatore specificato tramite logica GPIO.
 */
void ContactorTurnOn(CONTACTOR_id_t contactor_id) {
    // Controllo di sicurezza per evitare Out-of-Bounds nell'array
    if (contactor_id > CONTACTOR2) return;

    // Non collegato (mascherato): non tocca il GPIO, resta/rimane spento
    if (!contactor_is_connected(contactor_id)) {
        CONTACTOR[contactor_id].status = false;
        return;
    }

    // Gestione hardware dei pin di attivazione indipendenti per PSU1 e PSU2
    if (contactor_id == CONTACTOR1) {
        HAL_GPIO_WritePin(CONTACTOR_1_ON_GPIO_Port,CONTACTOR_1_ON_Pin,GPIO_PIN_SET);
    } else if (contactor_id == CONTACTOR2){
    	HAL_GPIO_WritePin(CONTACTOR_2_ON_GPIO_Port,CONTACTOR_2_ON_Pin,GPIO_PIN_SET);
    }

    // Aggiorna lo stato logico di accensione nella struttura dati interna
    CONTACTOR[contactor_id].status = true;
}

/**
 * @brief Spegne l'alimentatore specificato tramite logica GPIO.
 * @note  Sempre eseguita, anche se il contattore è mascherato come "non
 *        collegato": lo spegnimento non va mai negato per non rischiare di
 *        lasciare energizzato un contattore per un mascheramento errato
 *        (fail-safe, stesso principio di psu_turn_off_all()/contactors_off_all()
 *        in fsm.c, che spengono sempre entrambi a prescindere dalla maschera).
 */
void ContactorTurnOff(CONTACTOR_id_t contactor_id) {
    // Controllo di sicurezza per evitare Out-of-Bounds nell'array
    if (contactor_id > CONTACTOR2) return;

    // Gestione hardware dei pin di attivazione indipendenti per PSU1 e PSU2
    if (contactor_id == CONTACTOR1) {
        HAL_GPIO_WritePin(CONTACTOR_1_ON_GPIO_Port,CONTACTOR_1_ON_Pin,GPIO_PIN_RESET);
    } else if (contactor_id == CONTACTOR2){
    	HAL_GPIO_WritePin(CONTACTOR_2_ON_GPIO_Port,CONTACTOR_2_ON_Pin,GPIO_PIN_RESET);
    }

    // Aggiorna lo stato logico di accensione nella struttura dati interna
    CONTACTOR[contactor_id].status = false;
}

CONTACTOR_t ContactorGetStatus(CONTACTOR_id_t contactor_id) {
	CONTACTOR_t contactor={0};

    // Se l'ID è fuori range, inizializza una struttura vuota contrassegnata dall'errore
    if (contactor_id > CONTACTOR2) return contactor;

    /*
     * Non collegato (mascherato): ignora la lettura del pin di stato
     * ausiliario (verosimilmente floating/non cablato) — status torna
     * sempre false invece di propagare un valore indefinito.
     */
    if (!contactor_is_connected(contactor_id)) {
        CONTACTOR[contactor_id].status = false;
        return CONTACTOR[contactor_id];
    }

    //Leggo il valore di DC OK della PSU specifica per aggiornare lo stato prima di ritornare
    if(contactor_id==CONTACTOR1){
    	CONTACTOR[contactor_id].status=HAL_GPIO_ReadPin(CONTACTOR_1_STATUS_GPIO_Port,CONTACTOR_1_STATUS_Pin);
    }
    else if(contactor_id==CONTACTOR2){
    	CONTACTOR[contactor_id].status=HAL_GPIO_ReadPin(CONTACTOR_2_STATUS_GPIO_Port,CONTACTOR_2_STATUS_Pin);
    }

    // Ritorna la struttura dati reale memorizzata nell'array statico
    return CONTACTOR[contactor_id];
}
