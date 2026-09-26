/*
 * task_outputs.c
 *
 * Implementazione del task di aggiornamento dei LED del pannello frontale.
 */

#include "FreeRTOS.h"
#include "task.h"
#include "task_outputs.h"
#include "LED.h"
#include "Watchdog.h"

/* ========================================================================== */
/* --- ALLOCAZIONE STATICA --- */
/* ========================================================================== */

static StaticTask_t taskOutputs_tcb;
static StackType_t  taskOutputs_stack[256];

const osThreadAttr_t taskOutputs_attr = {
    .name       = "Outputs",
    .stack_mem  = &taskOutputs_stack[0],
    .stack_size = sizeof(taskOutputs_stack),
    .priority   = osPriorityNormal1,
    .cb_mem     = &taskOutputs_tcb,
    .cb_size    = sizeof(taskOutputs_tcb),
};

/* ========================================================================== */
/* --- TASK --- */
/* ========================================================================== */

void TaskOutputs_Run(void *arg)
{
    (void)arg;

    TickType_t xLastWakeTime = xTaskGetTickCount();

    for (;;)
    {
        Watchdog_Heartbeat(WDG_TASK_OUTPUTS);

        FPLED_Process();

        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(FPLED_PROCESS_PERIOD_MS));
    }
}
