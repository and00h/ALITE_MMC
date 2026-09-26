/*
 * rs485_cmd.h  --  MMC
 *
 * Interfaccia di comando ASCII su RS485 (USART1, DE hardware nativo).
 *
 * FORMATO COMANDI (dal master esterno verso MMC):
 *   <COMANDO> [ARG1 ARG2 ...]\r\n
 *
 * FORMATO RISPOSTE (MMC verso master):
 *   OK [dati]\r\n        — comando eseguito
 *   ERR <motivo>\r\n     — comando rifiutato o parametri errati
 *
 * COMANDI SUPPORTATI:
 * ─────────────────────────────────────────────────────────────────────
 *  LASER ON                    — abilita emissione laser
 *  LASER OFF                   — disabilita emissione laser
 *
 *  SET MODE SW|HW|ANALOG       — imposta modalita' operativa
 *  SET SETPOINT <0-100>        — imposta potenza % (solo SW mode)
 *
 *  GET POWER                   — legge potenza stimata [W] per ogni PD
 *                                → OK PD0:<W> PD1:<W> PD2:<W> PD3:<W>
 *  GET STATUS                  — legge stato sistema
 *                                → OK MODE:<x> SP:<x>% AMC:<x> LQ:<x>
 *  GET CONFIG                  — legge parametri principali di config
 *
 *  SET LUT GAIN SW|HW <t0> <t1> <t2> <t3>
 *      — imposta soglie guadagno per il modo specificato [mA o ADC raw]
 *
 *  SET LUT VALID <pd> SW|HW <entry> <setpoint> <min> <max>
 *      — imposta entry LUT validazione fotodiodo (pd=0-3, entry=0-3)
 *
 *  SET LUT POWER <pd> <win> <entry> <adc> <watt>
 *      — imposta entry LUT conversione ADC->W (pd=0-3, win=0-3, entry=0-4)
 *
 *  SAVE LUT                    — salva LUT corrente in flash
 *  RESET LUT                   — ripristina LUT ai valori di default
 *
 *  HELP                        — elenca i comandi disponibili
 * ─────────────────────────────────────────────────────────────────────
 *
 * UTILIZZO:
 *   Rs485Cmd_Init();           // una volta in MX_FREERTOS_Init o task init
 *   Rs485Cmd_Update();         // polling nel task RS485 (non blocca)
 */

#ifndef APP_RS485_RS485_CMD_H_
#define APP_RS485_RS485_CMD_H_

#include <stdint.h>
#include <stdbool.h>
#include "fsm.h"   /* SysState_t */

/* Lunghezza massima riga di comando (byte) */
#define RS485_CMD_BUF_SIZE   128U
/* Lunghezza massima risposta (byte) */
#define RS485_RSP_BUF_SIZE   256U

/* ============================================================================
 * API
 * ============================================================================ */

/**
 * @brief  Inizializza il modulo: salva il task handle e avvia DMA su USART1.
 *         Chiamare all'inizio di Task_RS485(), dopo MX_USART1_UART_Init().
 */
void Rs485Cmd_Init(void);

/**
 * @brief  Loop bloccante: attende un frame DMA (osThreadFlagsWait), lo parsa
 *         ed esegue il comando. Da chiamare in loop infinito nel task RS485.
 */
void Rs485Cmd_Update(void);

/**
 * @brief  Da richiamare in HAL_UARTEx_RxEventCallback() per USART1.
 *         Size = byte ricevuti nel buffer DMA (evento IDLE o TC). Contesto ISR.
 */
void Rs485Cmd_OnRxEvent(uint16_t size);

/**
 * @brief  Da richiamare in HAL_UART_ErrorCallback() per USART1.
 *         Cancella flag ORE/FE/NE e riavvia DMA. Contesto ISR.
 */
void Rs485Cmd_OnError(void);

/**
 * @brief  Da richiamare in HAL_UART_TxCpltCallback() per USART1.
 *         Segnala al task RS485 che il DMA TX ha completato. Contesto ISR.
 */
void Rs485Cmd_OnTxCplt(void);

/**
 * @brief  Stringa leggibile per uno SysState_t (es. "IDLE", "EMISSION").
 */
const char *Rs485Cmd_StateStr(SysState_t st);

/**
 * @brief  Invia una notifica di fault/allarme non sollecitata sul bus RS485.
 *         Formato: "ALARM <event_name>\r\n" oppure "FAULT <event_name>\r\n".
 *         Da chiamare da action_fault()/action_enter_error() nel task FSM,
 *         per QUALSIASI causa di SYS_ERROR/SYS_FAULT (standardizzato: prima
 *         solo FLOW/TEMP/le cause di action_fault() notificavano).
 *         Thread-safe: usa HAL_UART_Transmit (bloccante, breve).
 *
 *         No-op se il log errori RS485 non è attivo (vedi "LOG ERR ON/OFF"
 *         in rs485_cmd.c) — flag di sessione, non persistito, sempre OFF
 *         all'avvio.
 *
 * @param  ev        Evento FSM che ha causato la transizione.
 * @param  is_alarm  true = prefisso "ALARM" (errore recuperabile, → SYS_ERROR).
 *                   false = prefisso "FAULT" (non recuperabile, → SYS_FAULT).
 */
void Rs485Cmd_NotifyFault(SysEvent_t ev, bool is_alarm);

#endif /* APP_RS485_RS485_CMD_H_ */
