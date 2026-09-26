/*
 * QCW.c
 *
 * Implementazione della modulazione impulsata QCW su TIM16 + PE12.
 * Vedere QCW.h per la documentazione completa.
 *
 * TIM16 GESTITO DA CUBEMX (dal 2026-07-13):
 *   Il progetto è stato rigenerato con TIM16 + il suo global interrupt
 *   attivati nel .ioc. L'handle `htim16`, `MX_TIM16_Init()` (Core/Src/main.c)
 *   e `HAL_TIM_Base_MspInit()`/`MspDeInit` (Core/Src/stm32h7xx_hal_msp.c —
 *   clock enable + NVIC priority/enable per TIM16) sono ora generati da
 *   CubeMX: questo modulo NON deve più duplicarli (lo faceva in una prima
 *   versione, prima della rigenerazione — vedi cronologia). `TIM16_IRQHandler()`
 *   in Core/Src/stm32h7xx_it.c è anch'esso generato da CubeMX e chiama
 *   HAL_TIM_IRQHandler(&htim16) direttamente: la callback
 *   HAL_TIM_PeriodElapsedCallback() (Core/Src/main.c, USER CODE Callback 1)
 *   smista su htim->Instance == TIM16 verso QCW_TimerCallback() sotto —
 *   nessun handler dedicato necessario da questo file.
 *
 *   QCW_Init() si limita a portare `htim16` (extern, vedi hal_handles.h) in
 *   uno stato di riposo con HAL_TIM_Base_Init(), SENZA avviarlo: i
 *   valori "veri" di prescaler/ARR sono ricalcolati dinamicamente da
 *   QCW_SetParams() (vedi sotto) e scritti direttamente sui registri da
 *   QCW_Start()/QCW_TimerCallback(), non da qui.
 *
 * PRESCALER DINAMICO (esteso 2026-07-20, range 1Hz-50kHz):
 *   Il range originario (20-2000Hz) stava comodamente in un contatore a
 *   1MHz fisso (ARR max 65535 tick = 65.535ms, fase più lunga possibile
 *   ~49.5ms). Con QCW_MIN_FREQ_HZ=1 un prescaler fisso non basta più: a 1Hz
 *   un periodo intero sarebbe 1'000'000 tick anche a 1MHz, ben oltre il
 *   limite a 16 bit; a 50kHz, viceversa, un prescaler tarato per le basse
 *   frequenze darebbe solo poche decine di tick per periodo (risoluzione
 *   del duty cycle grossolana, a scatti del 5% o peggio).
 *
 *   QCW_SetParams() calcola quindi, ad ogni chiamata, il prescaler PIÙ
 *   PICCOLO (= risoluzione più fine possibile) tale che il PERIODO INTERO
 *   alla frequenza richiesta stia entro il limite a 16 bit di TIM16:
 *
 *     psc_plus_1 = ceil(tim_clk_hz / (freq_hz * 65535))   (>= 1)
 *     scaled_clk = tim_clk_hz / psc_plus_1
 *     period_ticks = scaled_clk / freq_hz                  (<= 65535)
 *
 *   tim_clk_hz è il clock di TIM16 (APB2, stesso raddoppio x2 quando il
 *   prescaler APB2 != 1 usato da stm32h7xx_hal_timebase_tim.c per TIM1 —
 *   vedi HAL_RCC_GetPCLK2Freq()/RM0468). Esempi (tim_clk_hz ~275MHz sul
 *   clock tree di progetto): a 1Hz risoluzione ~15µs/tick; a 50kHz clock
 *   quasi pieno, ~3.6ns/tick (period_ticks ~5500, duty a step dello 0.02%).
 *
 *   Prescaler e durate ON/OFF (in tick, alla risoluzione così scelta) sono
 *   applicati SEMPRE INSIEME, mai disallineati, ad ogni cambio fase
 *   (QCW_Start()/QCW_TimerCallback(), vedi apply_phase() sotto): il
 *   registro PSC è bufferizzato in hardware (si carica solo al prossimo
 *   evento di update, indipendentemente da ARPE), quindi si forza un evento
 *   di update software (UG) subito dopo averlo scritto, così la fase che
 *   sta per partire usa sempre la coppia PSC/ARR corretta.
 *
 *   LIMITI FISICI (non solo di calcolo, vedi anche banner RANGE in QCW.h):
 *   verso i 50kHz l'interrupt di TIM16 scatta fino a 100'000 volte/s
 *   (~5-10% CPU stimato); a duty molto sbilanciati (vicino a 1% o 99%) a
 *   50kHz la fase più corta dura solo poche decine di ns, paragonabile alla
 *   latenza di ingresso ISR — jitter percentuale non trascurabile in quel
 *   caso limite.
 */

#include "QCW.h"
#include "stm32h7xx_hal.h"
#include "hal_handles.h" /* extern htim16 (generato da CubeMX, main.c) */
#include "main.h"        /* nGATE_MC_Pin/_GPIO_Port */
#include "BoardCtrl.h"   /* BoardCtrl_GateMC_Close() */

/* ========================================================================== */
/* --- STATO INTERNO --- */
/* ========================================================================== */

static bool     s_active       = false;
static bool     s_gate_is_on   = false;   /* fase corrente: true=ON (PE12 LOW) */
static uint32_t s_psc          = 0U;      /* prescaler (registro PSC, = divisore-1) per la frequenza/risoluzione correnti */
static uint32_t s_on_ticks     = 500U;    /* durata fasi in tick, alla risoluzione data da s_psc */
static uint32_t s_off_ticks    = 500U;

/* ========================================================================== */
/* --- HELPER GPIO --- */
/* ========================================================================== */

static inline void gate_pulse_on(void)
{
    /* nGATE_MC active LOW: LOW = laser acceso (vedi BoardCtrl.c) */
    HAL_GPIO_WritePin(nGATE_MC_GPIO_Port, nGATE_MC_Pin, GPIO_PIN_RESET);
}

static inline void gate_pulse_off(void)
{
    HAL_GPIO_WritePin(nGATE_MC_GPIO_Port, nGATE_MC_Pin, GPIO_PIN_SET);
}

/* ========================================================================== */
/* --- INIT --- */
/* ========================================================================== */

void QCW_Init(void)
{
    /*
     * Stato di riposo sicuro: prescaler/ARR "veri" sono ricalcolati da
     * QCW_SetParams() e scritti direttamente sui registri da QCW_Start()/
     * QCW_TimerCallback() (vedi apply_phase() sotto) — i valori qui sono
     * solo un placeholder, sempre sovrascritti prima dell'avvio.
     *
     * htim16.Instance è già TIM16 (impostato da MX_TIM16_Init(), main.c).
     * Clock enable e NVIC (priorità 5, vedi stm32h7xx_hal_msp.c) sono già
     * stati eseguiti da HAL_TIM_Base_MspInit() alla prima HAL_TIM_Base_Init()
     * in main() — NON li ripetiamo qui.
     */
    htim16.Init.Prescaler         = 0U;
    htim16.Init.CounterMode       = TIM_COUNTERMODE_UP;
    htim16.Init.Period            = s_on_ticks - 1U;   /* placeholder, riarmato da QCW_Start() */
    htim16.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    htim16.Init.RepetitionCounter = 0U;
    htim16.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

    HAL_TIM_Base_Init(&htim16);
}

/* ========================================================================== */
/* --- PARAMETRI --- */
/* ========================================================================== */

bool QCW_SetParams(uint16_t freq_hz, uint8_t duty_pct)
{
    bool clamped = false;

    if (freq_hz < QCW_MIN_FREQ_HZ)  { freq_hz = QCW_MIN_FREQ_HZ;  clamped = true; }
    if (freq_hz > QCW_MAX_FREQ_HZ)  { freq_hz = QCW_MAX_FREQ_HZ;  clamped = true; }
    if (duty_pct < QCW_MIN_DUTY_PCT) { duty_pct = QCW_MIN_DUTY_PCT; clamped = true; }
    if (duty_pct > QCW_MAX_DUTY_PCT) { duty_pct = QCW_MAX_DUTY_PCT; clamped = true; }

    /*
     * Prescaler dinamico (vedi banner in cima al file): il PIÙ PICCOLO
     * (massima risoluzione) tale che il periodo intero, alla frequenza
     * richiesta, stia entro il limite a 16 bit di TIM16 (ARR max 65535).
     * TIM16 è su APB2 come TIM1/TIM15 (stesso fattore x2 quando il
     * prescaler APB2 != 1, vedi stm32h7xx_hal_timebase_tim.c/RM0468).
     *
     * Aritmetica in uint32_t semplice (niente uint64_t): con
     * freq_hz <= QCW_MAX_FREQ_HZ (50000), freq_hz*65535 <= ~3.28e9 e
     * tim_clk_hz (~qualche centinaio di MHz) + quel prodotto restano
     * entrambi ben dentro il range di un uint32_t (max ~4.29e9).
     */
    uint32_t tim_clk_hz = 2U * HAL_RCC_GetPCLK2Freq();

    uint32_t denom      = (uint32_t)freq_hz * 65535UL;
    uint32_t psc_plus_1 = (tim_clk_hz + denom - 1U) / denom;   /* ceiling division */
    if (psc_plus_1 < 1U)     { psc_plus_1 = 1U; }
    if (psc_plus_1 > 65536U) { psc_plus_1 = 65536U; }          /* PSC e' a 16 bit: divisore max 65536 */

    uint32_t scaled_clk_hz = tim_clk_hz / psc_plus_1;
    uint32_t period_ticks  = scaled_clk_hz / (uint32_t)freq_hz;
    if (period_ticks < 2U)     { period_ticks = 2U; }          /* almeno 1 tick per fase */
    if (period_ticks > 65535U) { period_ticks = 65535U; }      /* sicurezza, non atteso dato il calcolo sopra */

    uint32_t on_ticks  = (period_ticks * (uint32_t)duty_pct) / 100UL;
    if (on_ticks == 0U) { on_ticks = 1U; }
    if (on_ticks >= period_ticks) { on_ticks = period_ticks - 1U; }
    uint32_t off_ticks = period_ticks - on_ticks;
    if (off_ticks == 0U) { off_ticks = 1U; }

    /* Sezione critica breve: le nuove durate/prescaler diventano visibili a
     * QCW_TimerCallback() (ISR) solo al prossimo riarmo di fase, mai a
     * metà lettura (scrittura atomica su ARM per uint32_t allineati, ma il
     * lock esplicito documenta l'intento ed è comunque economico). */
    __disable_irq();
    s_psc       = psc_plus_1 - 1U;
    s_on_ticks  = on_ticks;
    s_off_ticks = off_ticks;
    __enable_irq();

    return !clamped;
}

/* ========================================================================== */
/* --- VINCOLO Ton MINIMO (vedi QCW_MIN_ON_TIME_US in QCW.h) --- */
/* ========================================================================== */

uint8_t QCW_ClampDutyForMinOnTime(uint16_t freq_hz, uint8_t duty_pct)
{
    /*
     * Ton_us = duty_pct/100 * period_us = duty_pct/100 * (1e6/freq_hz)
     *        = duty_pct * 10000 / freq_hz
     * Vincolo: Ton_us >= QCW_MIN_ON_TIME_US
     *   =>  duty_pct >= QCW_MIN_ON_TIME_US * freq_hz / 10000
     * Ceiling division per garantire il vincolo anche in presenza di
     * arrotondamento (un duty_pct ottenuto per troncamento potrebbe dare
     * Ton_us leggermente sotto soglia).
     *
     * Aritmetica in uint32_t: QCW_MIN_ON_TIME_US=10, freq_hz <= 50000
     * (QCW_MAX_FREQ_HZ) => numeratore max 10*50000+9999 ~ 5.1e5, ben dentro
     * il range di un uint32_t.
     */
    uint32_t min_duty = ((uint32_t)QCW_MIN_ON_TIME_US * (uint32_t)freq_hz + 9999UL) / 10000UL;

    /* Sicurezza: non atteso dato il calcolo sopra (a QCW_MAX_FREQ_HZ=50000Hz
     * min_duty vale 50%), ma evita comunque di clampare oltre il massimo
     * nominale se le costanti in QCW.h dovessero cambiare in futuro. */
    if (min_duty > (uint32_t)QCW_MAX_DUTY_PCT) { min_duty = (uint32_t)QCW_MAX_DUTY_PCT; }

    if ((uint32_t)duty_pct < min_duty) {
        duty_pct = (uint8_t)min_duty;
    }
    return duty_pct;
}

/* ========================================================================== */
/* --- START/STOP --- */
/* ========================================================================== */

/*
 * Applica la fase (prescaler + durata) in modo atomico e SEMPRE consistente:
 * PSC è un registro bufferizzato in hardware (si carica solo al prossimo
 * evento di update, indipendentemente da ARPE), quindi si forza un evento
 * di update software (UG) subito dopo averlo scritto insieme all'ARR, così
 * la fase che sta per iniziare usa sempre la coppia PSC/ARR corretta (mai
 * un prescaler "vecchio" abbinato a un ARR "nuovo" o viceversa). L'update
 * forzato genera anche un UIF/interrupt "fantasma": va ripulito subito dopo
 * per non rientrare spuriamente in QCW_TimerCallback().
 */
static inline void apply_phase(uint32_t ticks)
{
    __HAL_TIM_SET_PRESCALER(&htim16, s_psc);
    __HAL_TIM_SET_AUTORELOAD(&htim16, (ticks > 0U ? ticks - 1U : 0U));
    __HAL_TIM_SET_COUNTER(&htim16, 0U);
    HAL_TIM_GenerateEvent(&htim16, TIM_EVENTSOURCE_UPDATE);
    __HAL_TIM_CLEAR_FLAG(&htim16, TIM_FLAG_UPDATE);
}

void QCW_Start(void)
{
    if (s_active) {
        return; /* già in corso */
    }

    s_active     = true;
    s_gate_is_on = true;
    gate_pulse_on();

    apply_phase(s_on_ticks);

    HAL_TIM_Base_Start_IT(&htim16);
}

void QCW_Stop(void)
{
    if (!s_active) {
        /* Anche se non attivo, garantisce lo stato sicuro (idempotente,
         * innocuo se chiamato da action_enter_error()/action_fault() che
         * chiudono comunque il gate indipendentemente da QCW). */
        BoardCtrl_GateMC_Close();
        return;
    }

    HAL_TIM_Base_Stop_IT(&htim16);
    s_active     = false;
    s_gate_is_on = false;

    /* Stato sicuro: gate chiuso via BoardCtrl (stessa API usata da CW) */
    BoardCtrl_GateMC_Close();
}

bool QCW_IsActive(void)
{
    return s_active;
}

/* ========================================================================== */
/* --- CALLBACK (chiamata da HAL_TIM_PeriodElapsedCallback, main.c) --- */
/* ========================================================================== */

void QCW_TimerCallback(void)
{
    if (!s_active) {
        return; /* interrupt residuo dopo uno Stop concorrente: ignora */
    }

    if (s_gate_is_on) {
        gate_pulse_off();
        s_gate_is_on = false;
        apply_phase(s_off_ticks);
    } else {
        gate_pulse_on();
        s_gate_is_on = true;
        apply_phase(s_on_ticks);
    }
}
