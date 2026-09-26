/*
 * Key.c
 *
 * Driver per i contatti chiave di attivazione (nKEY_STATUS_A/B).
 * Vedere Key.h per documentazione.
 */

#include "Key.h"
#include "main.h"          /* nKEY_STATUS_A/B_GPIO_Port, nKEY_STATUS_A/B_Pin */
#include "config.h"        /* g_config.error_mask */
#include "task_monitor.h"  /* ERR_BIT_KEY_A / ERR_BIT_KEY_B */

typedef struct {
    bool     debounced;
    uint16_t debounce_cnt;
} Key_state_t;

static Key_state_t s_keys[KEY_COUNT];

static void update_key(Key_id_t id, bool raw, uint32_t tick_ms)
{
    Key_state_t *k = &s_keys[id];

    if (raw == k->debounced) {
        k->debounce_cnt = 0;
    } else {
        k->debounce_cnt += tick_ms;
        if (k->debounce_cnt >= KEY_DEBOUNCE_MS) {
            k->debounced    = raw;
            k->debounce_cnt = 0;
        }
    }
}

void Key_Process(uint32_t tick_ms)
{
    update_key(KEY_A,
        HAL_GPIO_ReadPin(nKEY_STATUS_A_GPIO_Port, nKEY_STATUS_A_Pin) == GPIO_PIN_RESET,
        tick_ms);
    update_key(KEY_B,
        HAL_GPIO_ReadPin(nKEY_STATUS_B_GPIO_Port, nKEY_STATUS_B_Pin) == GPIO_PIN_RESET,
        tick_ms);
}

bool Key_GetKeyStatus(Key_id_t id)
{
    if (id >= KEY_COUNT) return false;
    return s_keys[id].debounced;
}

bool Key_BothKeysInserted(void)
{
    return s_keys[KEY_A].debounced && s_keys[KEY_B].debounced;
}

bool Key_InterlockSatisfied(void)
{
    /*
     * Bit SET in error_mask = controllo attivo (stesso significato usato in
     * task_monitor.c per FLOOD/FLOW/TEMP/PSU/DEW). Contatto mascherato
     * (bit a 0) → non richiesto, conta come soddisfatto a prescindere dallo
     * stato reale del pin (utile se scollegato/floating in produzione).
     */
    bool a_ok = !(g_config.error_mask & ERR_BIT_KEY_A) || s_keys[KEY_A].debounced;
    bool b_ok = !(g_config.error_mask & ERR_BIT_KEY_B) || s_keys[KEY_B].debounced;

    return a_ok && b_ok;
}
