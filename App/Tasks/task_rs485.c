/*
 * task_rs485.c  --  MMC
 *
 * Task FreeRTOS: interfaccia RS485 ASCII su USART1.
 *
 * RESPONSABILITÀ:
 *   - Inizializza il modulo Rs485Cmd (avvia DMA, salva task handle).
 *   - Chiama Rs485Cmd_Update() in loop; ogni chiamata blocca su
 *     osThreadFlagsWait finché non arriva un frame completo dall'ISR.
 *     Il task consuma CPU solo durante la elaborazione del comando.
 */

#include "task_rs485.h"
#include "rs485_cmd.h"
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"
#include "Watchdog.h"

/* Attributi del task — accessibili da freertos.c tramite extern */
static StaticTask_t s_tcb;
/*
 * 1024 word = 4 kB (raddoppiata da 2kB il 2026-07-02, vedi indagine
 * HardFault su SET TERM ON / Config_Save()). Con MPU debug su flash
 * NOT_BUFFERABLE il fault resta comunque IMPRECISERR con BFAR non valido:
 * indizio che la scrittura incriminata NON è nella flash ma in RAM,
 * compatibile con un overflow di questo stack durante la catena
 * Task_RS485→Rs485Cmd_Update→parse_and_execute→cmd_set→Config_Save()→
 * flash_write_record()→HAL_FLASH_Program() (new_rec da 224 byte allocato
 * sullo stack di Config_Save() + snprintf/tokenize + HAL sottostante).
 * g_rs485StackHwm (sotto) resta lo strumento per verificarlo.
 */
static uint32_t     s_stack[1024U];

const osThreadAttr_t taskRS485_attr = {
    .name       = "RS485",
    .stack_mem  = &s_stack[0],
    .stack_size = sizeof(s_stack),
    .priority   = osPriorityLow1,
    .cb_mem     = &s_tcb,
    .cb_size    = sizeof(s_tcb),
};

/*
 * Minimo storico dello spazio libero di stack rilevato per Task_RS485, in
 * word da 4 byte (vedi uxTaskGetStackHighWaterMark). Aggiornato ad ogni
 * ciclo in Task_RS485(): tiene il valore più basso mai osservato, quindi
 * rappresenta il "punto di massima occupazione" raggiunto dallo stack da
 * 512 word (2kB) allocato sopra. Ispezionabile nel debugger (Expressions:
 * g_rs485StackHwm). Se si avvicina a 0 lo stack è insufficiente per la
 * catena Task_RS485→Rs485Cmd_Update→parse_and_execute→cmd_*→ok/err→
 * send_response (+ HAL UART/DMA sottostanti) — vedi indagine HardFault
 * RS485 del 2026-07-02.
 */
volatile UBaseType_t g_rs485StackHwm = (UBaseType_t)-1;

void Task_RS485(void *argument)
{
    (void)argument;

    /* Inizializza il modulo RS485 (avvia DMA su USART6) */
    Rs485Cmd_Init();

    for (;;)
    {
        /* Rs485Cmd_Update() blocca su osThreadFlagsWait (con timeout limitato
         * per heartbeat, vedi rs485_cmd.c) finché non arriva un frame; il
         * task non consuma CPU apprezzabile tra un comando e il successivo. */
        Rs485Cmd_Update();
        Watchdog_Heartbeat(WDG_TASK_RS485);

        UBaseType_t hwm = uxTaskGetStackHighWaterMark(NULL);
        if (hwm < g_rs485StackHwm) {
            g_rs485StackHwm = hwm;
        }
    }
}
