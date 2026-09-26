/*
 * Key.h
 *
 * Driver per i contatti chiave di attivazione del pannello frontale.
 *
 * HARDWARE:
 *   nKEY_STATUS_A  (PC13, GPIO_Input, active LOW)
 *   nKEY_STATUS_B  (PC14, GPIO_Input, active LOW)
 *
 *   Entrambi sono contatti a 2 posizioni stabili (NO / NC).
 *   true = contatto chiuso (chiave inserita, pin LOW).
 *
 * UTILIZZO:
 *   Chiamare Key_Process(tick_ms) ogni tick_ms millisecondi da task_inputs.
 *   Leggere lo stato con Key_GetKeyStatus() o Key_BothKeysInserted().
 */

#ifndef DRIVERS_KEY_KEY_H_
#define DRIVERS_KEY_KEY_H_

#include <stdbool.h>
#include <stdint.h>

/* Tempo di debounce [ms] */
#define KEY_DEBOUNCE_MS     20U

typedef enum {
    KEY_A = 0,   /* nKEY_STATUS_A (PC13) */
    KEY_B,       /* nKEY_STATUS_B (PC14) */
    KEY_COUNT
} Key_id_t;

/**
 * @brief  Aggiorna il debounce dei contatti chiave.
 * @param  tick_ms  Millisecondi trascorsi dall'ultima chiamata (periodo del task chiamante).
 * @note   Chiamare ogni tick_ms ms da task_inputs.
 */
void Key_Process(uint32_t tick_ms);

/**
 * @brief  Stato debounced del contatto chiave.
 * @return true se chiuso (chiave inserita).
 */
bool Key_GetKeyStatus(Key_id_t id);

/**
 * @brief  true se entrambi i contatti sono chiusi.
 *         NOTA: non usare per il gate di accensione se in produzione può
 *         essere montato un solo contatto — vedi Key_InterlockSatisfied().
 */
bool Key_BothKeysInserted(void);

/**
 * @brief  Interlock chiave soddisfatto, tenendo conto di g_config.error_mask.
 *
 *         Ogni contatto (A, B) è richiesto SOLO se il suo bit
 *         (ERR_BIT_KEY_A / ERR_BIT_KEY_B, vedi task_monitor.h) è abilitato
 *         in error_mask. Un contatto mascherato (bit a 0) viene ignorato:
 *         non serve che sia inserito, non blocca l'accensione, non genera
 *         fault se rimosso. Permette di:
 *           - montare un solo contatto in produzione (maschera l'altro bit
 *             una volta per tutte nella config di fabbrica);
 *           - ignorare temporaneamente entrambi i contatti per debug/banco
 *             prova (maschera entrambi i bit).
 *
 * @return true se ogni contatto NON mascherato risulta inserito
 *         (un contatto mascherato conta sempre come soddisfatto).
 */
bool Key_InterlockSatisfied(void);

#endif /* DRIVERS_KEY_KEY_H_ */
