/*
 * setpoint.h
 *
 * Gestione setpoint laser in percentuale con conversione via LUT.
 *
 * ============================================================================
 * ARCHITETTURA
 * ============================================================================
 *
 * L'utente (RS485/COM interface) fornisce il setpoint come percentuale [0-100%].
 * Questo modulo converte la percentuale in corrente [mA] tramite la LUT
 * g_config.setpoint_lut_ma[], con interpolazione lineare a tratti tra entry
 * adiacenti (risoluzione interna: 1%).
 *
 * La LUT è memorizzata in g_config (21 entry, gradini da 5%, persistente in
 * flash) e aggiornabile via service mode tramite Config_SetLUTEntry() /
 * Config_SetLUT().
 *
 * Il valore convertito viene scritto in LaseQ tramite LaseQSetCurrent() alla
 * prossima trasmissione RS485 (gestita da task_comms quando FSM_GetMode()
 * == FSM_MODE_SW, indipendentemente dal toggle "HYBRID2"/setpoint HW —
 * vedi fsm.h — che lascia il comportamento di task_comms invariato).
 * In FSM_MODE_ANALOG il setpoint è gestito dall'hardware (LPWR_SET_ISO /
 * AMC DAC) e task_comms non chiama LaseQSetCurrent().
 *
 * ============================================================================
 * UTILIZZO TIPICO
 * ============================================================================
 *
 *   // Da handler RS485/COM quando arriva comando di setpoint:
 *   Setpoint_SetPct(user_pct);
 *
 *   // In task_comms.c prima di LaseQ_Transmit() (solo FSM_MODE_SW):
 *   LaseQSetCurrent(Setpoint_GetCurrentMa());
 *
 * ============================================================================
 * SETPOINT DIRETTO IN CORRENTE (bypass LUT)
 * ============================================================================
 *
 * Setpoint_SetCurrentMa() permette di impostare direttamente la corrente
 * [mA] senza passare per la conversione percentuale/LUT — utile in service
 * mode per bench test o verifica calibrazione senza dover ricostruire la
 * percentuale equivalente. Attiva un override interno: Setpoint_GetCurrentMa()
 * restituisce il valore diretto finché non arriva una nuova Setpoint_SetPct()
 * (che disattiva l'override, tornando alla conversione normale via LUT) o
 * Setpoint_Reset(). Setpoint_GetPct() continua a riportare l'ultima
 * percentuale impostata esplicitamente e NON riflette il valore diretto
 * attivo: usare Setpoint_IsDirectMode() per distinguere i due casi in UI/log.
 *
 * ============================================================================
 */

#ifndef APP_SETPOINT_SETPOINT_H_
#define APP_SETPOINT_SETPOINT_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

/* ========================================================================== */
/* --- API SETPOINT --- */
/* ========================================================================== */

/**
 * @brief  Imposta il setpoint operativo in percentuale.
 *
 * Memorizza il valore in RAM. Non scrive su flash né invia a LaseQ.
 * Chiamare dal gestore RS485/COM quando riceve un comando di setpoint.
 *
 * @param  pct  Percentuale setpoint [0-100]. Valori > 100 vengono clampati a 100.
 */
void Setpoint_SetPct(uint8_t pct);

/**
 * @brief  Restituisce la percentuale setpoint corrente.
 * @retval Percentuale [0-100].
 */
uint8_t Setpoint_GetPct(void);

/**
 * @brief  Converte la percentuale setpoint corrente in corrente [mA]
 *         tramite la LUT g_config con interpolazione lineare a tratti.
 * @retval Corrente [mA] corrispondente al setpoint attivo.
 */
uint32_t Setpoint_GetCurrentMa(void);

/**
 * @brief  Azzera il setpoint (sicurezza: chiamare in IDLE/FAULT/ACTIVE).
 *         Equivalente a Setpoint_SetPct(0).
 */
void Setpoint_Reset(void);

/**
 * @brief  Conversione pura: percentuale → corrente [mA] tramite LUT + interpolazione.
 *
 * Non modifica lo stato interno. Utile per preview/verifica senza applicare il valore.
 *
 * @param  pct   Percentuale [0-100]. Valori > 100 vengono clampati a 100.
 * @retval Corrente [mA] corrispondente.
 */
uint32_t Setpoint_PctToCurrentMa(uint8_t pct);

/**
 * @brief  Imposta il setpoint operativo direttamente in corrente [mA],
 *         bypassando la conversione percentuale/LUT (vedi banner sopra).
 *
 * Memorizza il valore in RAM e attiva l'override diretto. Non scrive su
 * flash né invia a LaseQ/AMC direttamente: come Setpoint_SetPct(), la
 * trasmissione avviene al prossimo ciclo di task_comms.
 * Chiamare dal gestore RS485/COM in modalità service (comando protetto).
 *
 * @param  current_ma  Corrente diretta [mA]. Clampata a 65535 (limite
 *                      uint16_t del canale LaseQSetCurrent()/AMC_SetCurrentSetpoint()).
 */
void Setpoint_SetCurrentMa(uint32_t current_ma);

/**
 * @brief  true se il setpoint attivo è stato impostato in modo diretto
 *         (Setpoint_SetCurrentMa()) invece che tramite percentuale.
 *         In tal caso Setpoint_GetPct() non riflette la corrente erogata.
 */
bool Setpoint_IsDirectMode(void);

#ifdef __cplusplus
}
#endif

#endif /* APP_SETPOINT_SETPOINT_H_ */
