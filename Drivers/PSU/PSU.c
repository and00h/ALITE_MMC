/*
 * PSU.c
 *
 * Created on: 28 mag 2026
 * Author: lucamaggiotanasi
 */
#include "PSU.h"
#include "main.h"

/**
 * @brief Array statico privato per l'incapsulamento dei dati dei due alimentatori.
 */
static PSU_t PSU[2] = {
    {
        .id = PSU1,
        .bus_voltage = 0,
        .voltage_prog = 0,
        .current_prog = 0,
        .dc_ok = 0,
        .alarm = 0,
        .status = 0,
		.error = 0
    },
    {
        .id = PSU2,
        .bus_voltage = 0,
        .voltage_prog = 0,
        .current_prog = 0,
        .dc_ok = 0,
        .alarm = 0,
        .status = 0,
		.error = 0
    }
};

/**
 * @brief Esegue la sequenza di inizializzazione per una singola PSU.
 */
PSU_t PSUInit(PSU_id_t psu_id){
    // Se l'ID è fuori range, inizializza una struttura vuota contrassegnata dall'errore
    if (psu_id > PSU2){
    	PSU_t psu={0};
    	psu.error=PSU_ID_ERR;

    	return psu;
    }
	// Spegne l'alimentatore per garantire uno stato iniziale sicuro
	PSUTurnOff(psu_id);

	uint32_t psu_voltage_value=0; //Valori da leggere in memoria quando sarà implementata
	uint32_t psu_current_value=0;

	// Inizializza i riferimenti globali portandoli a zero
	PSUVoltageSet(psu_voltage_value);
	PSUCurrentSet(psu_current_value);

	// Restituisce lo stato corrente dell'alimentatore appena inizializzato
	return PSUGetStatus(psu_id);
}

/**
 * @brief Accende l'alimentatore specificato tramite logica GPIO (Attiva Bassa).
 */
void PSUTurnOn(PSU_id_t psu_id) {
    // Controllo di sicurezza per evitare Out-of-Bounds nell'array
    if (psu_id > PSU2) return;

    // Gestione hardware dei pin di attivazione indipendenti per PSU1 e PSU2
    if (psu_id == PSU1) {
        HAL_GPIO_WritePin(nPSU1_ON_GPIO_Port,nPSU1_ON_Pin,GPIO_PIN_RESET);
    } else if (psu_id == PSU2){
    	HAL_GPIO_WritePin(nPSU2_ON_GPIO_Port,nPSU2_ON_Pin,GPIO_PIN_RESET);
    }

    // Aggiorna lo stato logico di accensione nella struttura dati interna
    PSU[psu_id].status = true;
}

/**
 * @brief Spegne l'alimentatore microfilmato disattivando il pin GPIO (Logica Alta).
 */
void PSUTurnOff(PSU_id_t psu_id) {
    // Controllo di sicurezza per evitare Out-of-Bounds nell'array
    if (psu_id > PSU2) return;

    // Disattivazione hardware indipendente per PSU1 e PSU2
    if (psu_id == PSU1) {
        HAL_GPIO_WritePin(nPSU1_ON_GPIO_Port,nPSU1_ON_Pin,GPIO_PIN_SET);
    } else if (psu_id == PSU2){
    	HAL_GPIO_WritePin(nPSU2_ON_GPIO_Port,nPSU2_ON_Pin,GPIO_PIN_SET);
    }

    // Aggiorna lo stato logico di spegnimento nella struttura dati interna
    PSU[psu_id].status = false;
}

/**
 * @brief Recupera i dati di stato della PSU validando la richiesta.
 */
PSU_t PSUGetStatus(PSU_id_t psu_id) {
	PSU_t psu={0};

    // Se l'ID è fuori range, inizializza una struttura vuota contrassegnata dall'errore
    if (psu_id > PSU2){
    	psu.error=PSU_ID_ERR;

    	return psu;
    }

    //Leggo il valore di DC OK della PSU specifica per aggiornare lo stato prima di ritornare
    //Polarità: pin ALTO = DC OK asserito (nessun fault), pin BASSO = DC non ok.
    if(psu_id==PSU1){
    	PSU[psu_id].dc_ok=HAL_GPIO_ReadPin(nPSU1_DC_OK_GPIO_Port,nPSU1_DC_OK_Pin);
    }
    else if(psu_id==PSU2){
    	PSU[psu_id].dc_ok=HAL_GPIO_ReadPin(nPSU2_DC_OK_GPIO_Port,nPSU2_DC_OK_Pin);
    }

    // Ritorna la struttura dati reale memorizzata nell'array statico
    return PSU[psu_id];
}

/**
 * @brief Converte e applica il valore di tensione comune a entrambi gli alimentatori.
 */
void PSUVoltageSet(uint32_t value){
	//conversione tra valore in mV e DAC value

	// Essendo il segnale PV unico, aggiorna il valore programmato in entrambe le istanze
	PSU[PSU1].voltage_prog=value;
	PSU[PSU2].voltage_prog=value;
}

/**
 * @brief Converte e applica il valore di corrente limite comune a entrambi gli alimentatori.
 */
void PSUCurrentSet(uint32_t value){

	// Essendo il segnale PC unico, aggiorna il valore programmato in entrambe le istanze
	PSU[PSU1].current_prog=value;
	PSU[PSU2].current_prog=value;
}

/* ========================================================================== */
/*--- CALLBACK HARDWARE ---*/
/* ========================================================================== */

void PSU_Interface_Callback(uint16_t GPIO_Pin)
{
    //Polarità: pin ALTO = nessun allarme, pin BASSO = ALARM attivo (fault).
    if (GPIO_Pin == PSU1_ALARM_Pin)
    {
    	PSU[PSU1].alarm=!HAL_GPIO_ReadPin(PSU1_ALARM_GPIO_Port,PSU1_ALARM_Pin);
    }
    if (GPIO_Pin == PSU2_ALARM_Pin)
    {
    	PSU[PSU2].alarm=!HAL_GPIO_ReadPin(PSU2_ALARM_GPIO_Port,PSU2_ALARM_Pin);
    }
}

