/*
 * Lid.h
 *
 * Driver per i sensori di apertura coperchio/sportello (dal 2026-07-15).
 *
 * HARDWARE:
 *   LID1_OPEN  (PE2, GPIO_Input, NOPULL)
 *   LID2_OPEN  (PE3, GPIO_Input, NOPULL)
 *
 *   ASSUNZIONE DI POLARITA' (da verificare su schematico/banco prova):
 *   a differenza dei segnali "n*" (attivi LOW) usati altrove in questo
 *   progetto, questi pin NON hanno il prefisso "n" — il nome "*_OPEN"
 *   suggerisce quindi pin HIGH = coperchio APERTO (stato non sicuro),
 *   pin LOW = coperchio chiuso. Se il banco prova mostra il comportamento
 *   opposto, invertire la lettura in Lid_Process() (un solo punto).
 *
 *   Due sensori fisicamente e logicamente indipendenti (stesso principio
 *   di Key.h per le chiavi A/B): spesso in produzione potrebbe essere
 *   montato/cablato un solo sensore, o nessuno durante il commissioning.
 *
 * UTILIZZO:
 *   Chiamare Lid_Process(tick_ms) ogni tick_ms millisecondi da task_inputs.
 *   Leggere lo stato con Lid_GetStatus() o Lid_InterlockSatisfied().
 */

#ifndef DRIVERS_LID_LID_H_
#define DRIVERS_LID_LID_H_

#include <stdbool.h>
#include <stdint.h>

/* Tempo di debounce [ms] — stesso valore di KEY_DEBOUNCE_MS */
#define LID_DEBOUNCE_MS     20U

typedef enum {
    LID_1 = 0,   /* LID1_OPEN (PE2) */
    LID_2,       /* LID2_OPEN (PE3) */
    LID_COUNT
} Lid_id_t;

/**
 * @brief  Aggiorna il debounce dei sensori coperchio.
 * @param  tick_ms  Millisecondi trascorsi dall'ultima chiamata (periodo del task chiamante).
 * @note   Chiamare ogni tick_ms ms da task_inputs.
 */
void Lid_Process(uint32_t tick_ms);

/**
 * @brief  Stato debounced del sensore coperchio.
 * @return true se APERTO (stato non sicuro).
 */
bool Lid_GetStatus(Lid_id_t id);

/**
 * @brief  Interlock coperchi soddisfatto, tenendo conto di g_config.error_mask.
 *
 *         Ogni sensore (LID1, LID2) è richiesto SOLO se il suo bit
 *         (ERR_BIT_LID1 / ERR_BIT_LID2, vedi task_monitor.h) è abilitato
 *         in error_mask. Un sensore mascherato (bit a 0) viene ignorato:
 *         non serve che sia chiuso, non blocca l'accensione, non genera
 *         errore se aperto. Stesso principio di Key_InterlockSatisfied().
 *
 * @return true se ogni sensore NON mascherato risulta chiuso
 *         (un sensore mascherato conta sempre come soddisfatto).
 */
bool Lid_InterlockSatisfied(void);

#endif /* DRIVERS_LID_LID_H_ */
