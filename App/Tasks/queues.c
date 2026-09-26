/*
 * queues.c
 *
 * Creazione e gestione centralizzata delle code FreeRTOS di sistema.
 */

#include "queues.h"

/* ========================================================================== */
/* --- DEFINIZIONE DEGLI HANDLE --- */
/* ========================================================================== */

osMessageQueueId_t eventQueueHandle = NULL;

/* ========================================================================== */
/* --- API --- */
/* ========================================================================== */

void Queues_Init(void)
{
    eventQueueHandle = osMessageQueueNew(
        16,                 /* Numero massimo di elementi */
        sizeof(SysEvent_t), /* Dimensione di un elemento */
        NULL                /* Attributi default */
    );

    /* In un sistema di produzione: gestire il caso NULL con un assert o log */
}

void Queue_PostEvent(SysEvent_t event)
{
    if (eventQueueHandle == NULL) return;

    /*
     * Timeout 0: non bloccante.
     * Se la coda è piena l'evento viene scartato. Il task_monitor
     * rileverà nuovamente la condizione al prossimo ciclo di monitoraggio.
     */
    osMessageQueuePut(eventQueueHandle, &event, 0, 0);
}
