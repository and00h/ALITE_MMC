/*
 * task_rs485.h  --  MMC
 *
 * Task FreeRTOS per l'interfaccia RS485 ASCII (USART6).
 *
 * PRIORITÀ SUGGERITA: osPriorityLow (inferiore a task_comms e task_fsm).
 * STACK SUGGERITO:    512 word (2 kB) — sufficiente per snprintf + parser.
 *
 * Il task inizializza Rs485Cmd e chiama Rs485Cmd_Update() in polling
 * con un ritardo di 5 ms per non saturare la CPU.
 * La ricezione reale è guidata dagli eventi DMA Idle (~ 1ms dopo CR+LF),
 * quindi il ritardo di 5ms introduce latenza di risposta trascurabile.
 */

#ifndef APP_RS485_TASK_RS485_H_
#define APP_RS485_TASK_RS485_H_

#include "cmsis_os.h"

/** Attributi del task (stack statico, priorità bassa). */
extern const osThreadAttr_t taskRS485_attr;

/**
 * @brief Entry-point del task RS485.
 *        Da passare a osThreadNew() in MX_FREERTOS_Init().
 *        @param argument  non utilizzato (passare NULL).
 */
void Task_RS485(void *argument);

#endif /* APP_RS485_TASK_RS485_H_ */
