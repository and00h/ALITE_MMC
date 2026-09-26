/*
 * Lid.c
 *
 * Driver per i sensori di apertura coperchio (LID1_OPEN/LID2_OPEN).
 * Vedere Lid.h per documentazione e nota sulla polarità assunta.
 */

#include "Lid.h"
#include "main.h"          /* LID1_OPEN/LID2_OPEN_GPIO_Port, _Pin */
#include "config.h"        /* g_config.error_mask */
#include "task_monitor.h"  /* ERR_BIT_LID1 / ERR_BIT_LID2 */

typedef struct {
    bool     debounced;
    uint16_t debounce_cnt;
} Lid_state_t;

static Lid_state_t s_lids[LID_COUNT];

static void update_lid(Lid_id_t id, bool raw, uint32_t tick_ms)
{
    Lid_state_t *l = &s_lids[id];

    if (raw == l->debounced) {
        l->debounce_cnt = 0;
    } else {
        l->debounce_cnt += tick_ms;
        if (l->debounce_cnt >= LID_DEBOUNCE_MS) {
            l->debounced    = raw;
            l->debounce_cnt = 0;
        }
    }
}

void Lid_Process(uint32_t tick_ms)
{
    /* raw = true significa "aperto" (vedi nota polarità in Lid.h) */
    update_lid(LID_1,
        HAL_GPIO_ReadPin(LID1_OPEN_GPIO_Port, LID1_OPEN_Pin) == GPIO_PIN_SET,
        tick_ms);
    update_lid(LID_2,
        HAL_GPIO_ReadPin(LID2_OPEN_GPIO_Port, LID2_OPEN_Pin) == GPIO_PIN_SET,
        tick_ms);
}

bool Lid_GetStatus(Lid_id_t id)
{
    if (id >= LID_COUNT) return false;
    return s_lids[id].debounced;
}

bool Lid_InterlockSatisfied(void)
{
    /*
     * Bit SET in error_mask = controllo attivo (stesso significato usato in
     * Key_InterlockSatisfied() e in task_monitor.c per FLOOD/FLOW/TEMP/PSU/
     * DEW). Sensore mascherato (bit a 0) → non richiesto, conta come chiuso
     * a prescindere dallo stato reale del pin (utile se non montato/floating
     * in produzione o durante il commissioning).
     */
    bool lid1_ok = !(g_config.error_mask & ERR_BIT_LID1) || !s_lids[LID_1].debounced;
    bool lid2_ok = !(g_config.error_mask & ERR_BIT_LID2) || !s_lids[LID_2].debounced;

    return lid1_ok && lid2_ok;
}
