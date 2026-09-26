/*
 * COM_interface_app.h
 *
 * Livello APPLICATIVO del protocollo COM interface (lato MMC): interpreta i
 * payload STATUS/CONTROL/CONFIG ricevuti via SPI3 (vedi COM_interface.h/.c
 * per il trasporto) e li traduce in chiamate reali alla FSM/config/telemetria
 * di macchina, cosi' come task_com_interface.c li traduce in chiamate a
 * questo modulo. Separazione analoga a com_dispatch.c/com.c lato ALITE_COM.
 *
 * STATO (2026-07-28): implementa gli opcode CONTROL e gli opcode STATUS per
 * cui esiste gia' un accessor pubblico nel firmware MMC (FSM, g_config,
 * task_monitor, Setpoint, QCW, SHT35, FlowMeter, TaskAmc, LaseQ, eFuse, SAB,
 * RTC), piu' le soglie SET_TEMP/FLOW/HUM/DEW_THR (argomenti nei byte
 * COM_CTRL_PAYLOAD_ARGS..). Gli altri opcode ad argomenti (password, mask,
 * indice LUT, ecc. — vedi COM_control_opcode_e/COM_status_opcode_e in
 * COM_interface_protocol.h) non sono ancora implementati lato MMC e
 * rispondono esplicitamente con errore/AUTH invece di essere ignorati in
 * silenzio: cercare "TODO" in COM_interface_app.c per l'elenco preciso.
 */

#ifndef DRIVERS_COM_INTERFACE_COM_INTERFACE_APP_H_
#define DRIVERS_COM_INTERFACE_COM_INTERFACE_APP_H_

#include "COM_interface_protocol.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief  Gestisce una richiesta STATUS. AGGIORNAMENTO 2026-07-30: il
 *         formato risposta ALITE_COM non e' piu' "1 opcode -> 4 byte a
 *         posizione fissa" ma un'unica tabella completa (COM_StatusTable_t,
 *         COM_interface_protocol.h) con ogni campo all'offset assoluto del
 *         proprio COM_StatusOpcode_t — vedi il banner sopra COM_StatusTable_t
 *         per le prove nel codice ALITE_COM. req_payload (n_requested +
 *         lista opcode richiesti, COM_status_req_t lato ALITE_COM) e' quindi
 *         IGNORATO: si risponde sempre con la tabella intera.
 * @param  req_payload   Payload ricevuto, non utilizzato (vedi sopra).
 * @param  resp_payload  Buffer di uscita, COM_PAYLOAD_SIZE byte: azzerato
 *                        internamente, poi byte 0 = COM_RespStatus_t (sempre
 *                        OK), byte 1.. = COM_StatusTable_t completa —
 *                        stesso formato letto da COM_parse_status_resp()/
 *                        ALITE_system_update_status_full() lato ALITE_COM
 *                        (com_dispatch.c/alite_system.c).
 */
void COM_App_HandleStatus(const uint8_t *req_payload, uint8_t *resp_payload);

/**
 * @brief  Gestisce una richiesta CONTROL ricevuta (COM_CTRL_PAYLOAD_SIZE
 *         byte: opcode al byte 0, poi i campi fissi fsm_flags,
 *         power_setpoint, qcw_ctrl, qcw_freq_hz, qcw_duty, mode, hw_ctrl per
 *         gli opcode storici, oppure fino a 32 byte di argomenti per gli
 *         altri — vedi COM_CTRL_PAYLOAD_* in COM_interface_protocol.h) e
 *         applica l'azione richiesta.
 * @param  req_payload   Payload ricevuto (almeno COM_CTRL_PAYLOAD_SIZE byte).
 * @param  resp_payload  Buffer di uscita, COM_PAYLOAD_SIZE byte: azzerato
 *                        internamente, poi byte 0 = COM_RespStatus_t — stesso
 *                        formato letto da COM_parse_control_resp() lato
 *                        ALITE_COM.
 */
void COM_App_HandleControl(const uint8_t *req_payload, uint8_t *resp_payload);

/**
 * @brief  Gestisce una richiesta CONFIG ricevuta (COM_ConfigPayload_t, 25
 *         byte) e applica la configurazione a g_config.
 * @param  req_payload   Payload ricevuto (almeno sizeof(COM_ConfigPayload_t)
 *                        byte).
 * @param  resp_payload  Buffer di uscita, COM_PAYLOAD_SIZE byte: azzerato
 *                        internamente, poi byte 0..3 = uint32 (0 = OK,
 *                        diverso da 0 = errore) — stesso formato letto da
 *                        COM_parse_config_resp() lato ALITE_COM (placeholder,
 *                        marcato "TODO: define response type" anche li').
 */
void COM_App_HandleConfig(const uint8_t *req_payload, uint8_t *resp_payload);

#ifdef __cplusplus
}
#endif

#endif /* DRIVERS_COM_INTERFACE_COM_INTERFACE_APP_H_ */
