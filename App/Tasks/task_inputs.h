/*
 * task_inputs.h
 *
 * Task di acquisizione e processing di tutti gli ingressi del sistema (5ms).
 *
 * RESPONSABILITÀ:
 *   - EXT_AcquireIN()   : acquisisce ingressi digitali e setpoint analogici EXT
 *   - Key_Process()     : debounce polling chiavi fisiche (nKEY_STATUS_A/B)
 *   - FPButton_Process(): debounce e state machine pulsante frontale (da EXTI)
 *   - Polling nEXT_CTL_iso  (PD14): edge detection → SYS_EXT_CTRL/OFF_EVENT + mode
 *   - Polling nSYS_ON_iso   (PD15): edge detection → SYS_EXT_SYSON/SYSOFF_EVENT
 *   - Polling nENABLE_IN_iso (PG2): edge detection → SYS_EXT_ENABLE/DISABLE_EVENT
 *   - Key removed check (ON/ENABLED/EMISSION): → SYS_KEY_REMOVED_EVENT
 *   - Lid_Process()     : debounce polling coperchi (LID1_OPEN/LID2_OPEN)
 *   - Lid open check (ON/ENABLED/EMISSION): → SYS_LID_OPEN_EVENT
 *   - SAB_Process()     : gestione timeout interlock e monitoraggio armo SAB
 *
 * PRIORITÀ: osPriorityNormal2 (superiore a Outputs, inferiore a Monitor)
 * PERIODICITÀ: 5ms con vTaskDelayUntil (nessun drift)
 *
 * NOTA:
 *   Gli ingressi EXT (nEXT_CTL_iso, nSYS_ON_iso, nENABLE_IN_iso) sono
 *   segnali isolati otticamente, livello logico attivo LOW (invertito).
 *   La lettura diretta del pin GPIO dà HIGH = inattivo, LOW = attivo.
 */

#ifndef APP_TASKS_TASK_INPUTS_H_
#define APP_TASKS_TASK_INPUTS_H_

#include "cmsis_os.h"

/* Attributi del task - usati in freertos.c */
extern const osThreadAttr_t taskInputs_attr;

/**
 * @brief Entry point del task inputs.
 *        Da passare a osThreadNew() in MX_FREERTOS_Init().
 */
void TaskInputs_Run(void *arg);

#endif /* APP_TASKS_TASK_INPUTS_H_ */
