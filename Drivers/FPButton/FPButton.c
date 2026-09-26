/*
 * FPButton.c
 *
 * Driver per il pulsante momentaneo del pannello frontale (nFP_BUTTON, EXTI15).
 * Vedere FPButton.h per documentazione.
 *
 * ARCHITETTURA:
 *   FPButton_EXTI_Callback() — ISR: registra stato pin e fronte pendente.
 *   FPButton_Process(tick_ms) — task: debounce + state machine con
 *                              contatori ms. Nessun HAL_GetTick().
 *
 * MACCHINA A STATI:
 *
 *   IDLE
 *     ──(fronte discesa confermato)──> PRESSED
 *
 *   PRESSED   [hold_cnt++]
 *     ──(fronte salita + hold < LONG)──> WAIT_DOUBLE
 *     ──(hold_cnt >= LONG)─────────────> LONG_HOLD
 *
 *   LONG_HOLD
 *     ──(fronte salita confermato)──> emit LONG_PRESS → IDLE
 *
 *   WAIT_DOUBLE   [wait_cnt++]
 *     ──(fronte discesa)──────────────> emit DOUBLE → PRESSED
 *     ──(wait_cnt >= DOUBLE_MS)───────> emit PRESS  → IDLE
 *
 * Il debounce avviene all'interno di ogni transizione: un fronte opposto
 * ricevuto durante il periodo di debounce annulla il fronte precedente.
 */

#include "FPButton.h"
#include "main.h"       /* nFP_BUTTON_GPIO_Port, nFP_BUTTON_Pin */
#include "FreeRTOS.h"
#include "task.h"

/* ========================================================================== */
/* --- CALLBACK --- */
/* ========================================================================== */

static FPButton_EventCb_t s_event_cb = NULL;

void FPButton_RegisterCallback(FPButton_EventCb_t cb)
{
    s_event_cb = cb;
}

/* ========================================================================== */
/* --- STATO VOLATILE (scritto da ISR) --- */
/* ========================================================================== */

static volatile struct {
    bool pressed;    /* true = pulsante premuto (pin LOW) */
    bool new_edge;   /* fronte non ancora consumato       */
} s_hw;

/* ========================================================================== */
/* --- STATO STATE MACHINE (usato solo da FPButton_Process) --- */
/* ========================================================================== */

typedef enum {
    BTN_IDLE = 0,
    BTN_DEBOUNCE_PRESS,
    BTN_PRESSED,
    BTN_DEBOUNCE_REL,
    BTN_LONG_HOLD,
    BTN_WAIT_DOUBLE,
} BtnState_t;

static struct {
    BtnState_t state;
    bool       pin_at_edge;
    uint16_t   debounce_cnt;
    uint32_t   hold_cnt;
    uint32_t   wait_cnt;
} s_sm;

/* ========================================================================== */
/* --- API PUBBLICA --- */
/* ========================================================================== */

void FPButton_EXTI_Callback(void)
{
    s_hw.pressed  = (HAL_GPIO_ReadPin(nFP_BUTTON_GPIO_Port, nFP_BUTTON_Pin) == GPIO_PIN_RESET);
    s_hw.new_edge = true;
}

void FPButton_Process(uint32_t tick_ms)
{
    /* Preleva fronte pendente con sezione critica FreeRTOS */
    bool edge_available = false;
    bool edge_pressed   = false;

    taskENTER_CRITICAL();
    if (s_hw.new_edge) {
        edge_available = true;
        edge_pressed   = s_hw.pressed;
        s_hw.new_edge  = false;
    }
    taskEXIT_CRITICAL();

    if (edge_available) {
        s_sm.pin_at_edge  = edge_pressed;
        s_sm.debounce_cnt = 0;
    }

    switch (s_sm.state) {

        case BTN_IDLE:
            if (edge_available && edge_pressed) {
                s_sm.state = BTN_DEBOUNCE_PRESS;
            }
            break;

        case BTN_DEBOUNCE_PRESS:
            if (edge_available && !edge_pressed) {
                /* bounce: fronte opposto durante debounce */
                s_sm.state = BTN_IDLE;
                break;
            }
            s_sm.debounce_cnt += tick_ms;
            if (s_sm.debounce_cnt >= FPBTN_DEBOUNCE_MS) {
                s_sm.state    = BTN_PRESSED;
                s_sm.hold_cnt = 0;
            }
            break;

        case BTN_PRESSED:
            if (edge_available && !edge_pressed) {
                s_sm.state        = BTN_DEBOUNCE_REL;
                s_sm.debounce_cnt = 0;
                break;
            }
            s_sm.hold_cnt += tick_ms;
            if (s_sm.hold_cnt >= FPBTN_LONG_PRESS_MS) {
                s_sm.state = BTN_LONG_HOLD;
            }
            break;

        case BTN_DEBOUNCE_REL:
            if (edge_available && edge_pressed) {
                /* bounce: ancora premuto */
                s_sm.state    = BTN_PRESSED;
                s_sm.hold_cnt = 0;
                break;
            }
            s_sm.debounce_cnt += tick_ms;
            if (s_sm.debounce_cnt >= FPBTN_DEBOUNCE_MS) {
                if (s_sm.hold_cnt >= FPBTN_LONG_PRESS_MS) {
                    if (s_event_cb) { s_event_cb(FPBTN_EVT_LONG_PRESS); }
                    s_sm.state = BTN_IDLE;
                } else {
                    s_sm.state    = BTN_WAIT_DOUBLE;
                    s_sm.wait_cnt = 0;
                }
            }
            break;

        case BTN_LONG_HOLD:
            if (edge_available && !edge_pressed) {
                s_sm.state        = BTN_DEBOUNCE_REL;
                s_sm.debounce_cnt = 0;
                s_sm.hold_cnt     = FPBTN_LONG_PRESS_MS; /* garantisce branch long */
            }
            break;

        case BTN_WAIT_DOUBLE:
            if (edge_available && edge_pressed) {
                if (s_event_cb) { s_event_cb(FPBTN_EVT_DOUBLE_PRESS); }
                s_sm.state        = BTN_DEBOUNCE_PRESS;
                s_sm.debounce_cnt = 0;
                break;
            }
            s_sm.wait_cnt += tick_ms;
            if (s_sm.wait_cnt >= FPBTN_DOUBLE_PRESS_MS) {
                if (s_event_cb) { s_event_cb(FPBTN_EVT_PRESS); }
                s_sm.state = BTN_IDLE;
            }
            break;
    }
}
