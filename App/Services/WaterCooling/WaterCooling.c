/*
 * WaterCooling.c
 *
 *  Created on: 9 giu 2026
 *      Author: lucamaggiotanasi
 */


#include "WaterCooling.h"
#include "WaterValve.h"
#include "FlowMeter.h"
// #include "TempSensor.h"   // quando sarà disponibile

// Stato interno — privato, non esposto nell'header
static WaterCooling_State_t wc_state = WC_STATE_OFF;

void WaterCooling_Enable(void)
{
    WaterValveOpen();
    // FlowMeter non necessita di "start" esplicito se usa IC capture continuo,
    // ma se hai un enable hardware aggiungilo qui
    wc_state = WC_STATE_RUNNING;
}

void WaterCooling_Disable(void)
{
    WaterValveClose();
    wc_state = WC_STATE_OFF;
}

void WaterCooling_Emergency(void)
{
	/* NON necessario chiudere la valvola per errori di flusso*/
    //WaterValveClose();
    wc_state = WC_STATE_ERROR;
    // nessuna sequenza, chiusura immediata
}

WaterCooling_State_t WaterCooling_GetState(void)
{
    return wc_state;
}

bool WaterCooling_IsReady(void)
{
    if (wc_state != WC_STATE_RUNNING) return false;
    return (FlowMeter_GetFlowRate(FLOW_METER_1) >= WC_FLOW_MIN_LPM);
}
