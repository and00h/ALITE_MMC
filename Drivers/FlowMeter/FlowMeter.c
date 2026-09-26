/*
 * flow_meter.c
 */

#include "FlowMeter.h"
#include <stddef.h>

#define FLOW_METER_K_FACTOR     11.0f  /* K-factor da datasheet sensore: F[Hz] = 11 * Q[L/min] (+-3%) */
#define FLOW_TIMEOUT_MS         500

/*
 * LINEARIZZAZIONE (taratura vs sensore da banco, 2026-07-20):
 * il K-factor di datasheet (+-3%) porta il valore letto da OPAL vicino al
 * reale ma con uno scostamento sistematico non trascurabile agli estremi del
 * campo misurato. Corretto con una regressione lineare Q_reale = a*Q_opal + b
 * sui 5 punti di confronto OPAL / sensore da banco (L/min):
 *   OPAL   Banco
 *   5.2    5.5
 *   4.6    5.0
 *   3.6    4.0
 *   2.5    3.0
 *   5.3    5.7
 * Minimi quadrati: a = 0.9519, b = 0.6040 (R^2 ~ 0.999, residui < 0.06 L/min
 * su tutti i punti). Validata SOLO nel range campionato (~2.5-5.3 L/min letti
 * da OPAL): fuori da quel range (vicino allo zero o a portate elevate) e'
 * un'estrapolazione, non garantita — da ricontrollare se si tarano nuovi
 * punti a portate diverse.
 */
#define FLOW_CAL_GAIN           0.9519f
#define FLOW_CAL_OFFSET         0.6040f /* L/min */

/*
 * MEDIA MOBILE (FLOW_AVG_SAMPLES): il conteggio a bordo capture e' un
 * campionamento "a periodo singolo" (1 impulso = 1 stima di frequenza),
 * quindi intrinsecamente rumoroso (turbolenza, jitter meccanico della
 * girante). Per smussarlo senza introdurre troppa latenza si fa una media
 * mobile sugli ultimi N impulsi.
 *
 * N e' scelto in funzione della frequenza di acquisizione attesa: a portata
 * nominale (chiller 5.5 L/min, K-factor 11 -> F = 11*5.5 = 60.5 Hz, periodo
 * ~16.5ms) si hanno circa 6 impulsi per ciclo di task_monitor (100ms,
 * MONITOR_PERIOD_MS in task_monitor.h). N=8 (potenza di 2, indicizzazione a
 * maschera) copre quella finestra con un margine, restando comunque
 * responsivo (aggiornamento pieno del buffer in ~130ms alla portata
 * nominale) e degradando in modo grazioso a portate piu' basse (il buffer
 * semplicemente impiega piu' tempo a riempirsi, la media e' fatta sui soli
 * campioni validi tramite sample_count).
 */
#define FLOW_AVG_SAMPLES         8U
#define FLOW_AVG_MASK            (FLOW_AVG_SAMPLES - 1U)

/* Struttura interna (nascosta all'applicazione) */
typedef struct {
    TIM_HandleTypeDef *htim;
    uint32_t           channel;
    uint32_t           tim_clk_hz;
    uint32_t           last_capture;
    uint32_t           last_overflow_count;
    uint8_t            is_first_capture;
    volatile float     frequency_hz;
    volatile float     flow_rate_lpm;
    volatile uint32_t  last_interrupt_time;
    uint8_t            is_configured;
    /* Media mobile (buffer circolare a somma incrementale) */
    float              freq_samples[FLOW_AVG_SAMPLES];
    uint8_t            sample_idx;
    uint8_t            sample_count;
    float              freq_sum;
} Internal_FlowMeter_t;

/* ARRAY PRIVATO delle istanze: accessibile solo in questo file */
static Internal_FlowMeter_t g_meters[FLOW_METER_COUNT] = {0};

/*
 * Contatore condiviso degli overflow (evento Update) del timer. TIM4 e'
 * condiviso da FLOW_METER_1 (CH1) e FLOW_METER_2 (CH2): essendo lo stesso
 * contatore hardware a 16 bit, il numero di overflow e' identico per
 * entrambi i canali, un solo contatore basta. Incrementato da
 * FlowMeter_PeriodElapsedCallback() (agganciata da HAL_TIM_PeriodElapsedCallback,
 * main.c). Necessario perche' a piena velocita' (nessun prescaler, ARR=65535)
 * il timer va in overflow molte volte tra due impulsi consecutivi del
 * flussometro: senza contarli il calcolo del periodo si "aliasa" sul modulo
 * 65536, dando frequenze/portate del tutto errate e incoerenti da una
 * lettura all'altra (bug osservato con GET FLOW).
 */
static volatile uint32_t s_tim4_overflow_count = 0;
static uint8_t           s_tim4_base_started = 0;

/* Funzione statica di utilità per gestire il timeout di stallo (0 Hz) */
static void Check_Timeout(Internal_FlowMeter_t *p_meter) {
    if (p_meter->is_configured && ((HAL_GetTick() - p_meter->last_interrupt_time) > FLOW_TIMEOUT_MS)) {
        p_meter->frequency_hz = 0.0f;
        p_meter->flow_rate_lpm = 0.0f;
        p_meter->is_first_capture = 1;
        /* Ripulisce la media mobile: al ritorno del segnale si riparte da
         * campioni freschi invece di mediare con valori pre-stallo stantii. */
        for (uint8_t i = 0; i < FLOW_AVG_SAMPLES; i++) {
            p_meter->freq_samples[i] = 0.0f;
        }
        p_meter->sample_idx   = 0;
        p_meter->sample_count = 0;
        p_meter->freq_sum     = 0.0f;
    }
}

/* Inserisce un nuovo campione di frequenza (grezzo, 1 impulso) nella media
 * mobile e restituisce la media aggiornata. */
static float Push_FreqSample(Internal_FlowMeter_t *p_meter, float new_freq) {
    p_meter->freq_sum -= p_meter->freq_samples[p_meter->sample_idx];
    p_meter->freq_samples[p_meter->sample_idx] = new_freq;
    p_meter->freq_sum += new_freq;
    p_meter->sample_idx = (uint8_t)((p_meter->sample_idx + 1U) & FLOW_AVG_MASK);

    if (p_meter->sample_count < FLOW_AVG_SAMPLES) {
        p_meter->sample_count++;
    }

    return p_meter->freq_sum / (float)p_meter->sample_count;
}

uint32_t TIM_GetClock(TIM_HandleTypeDef *htim)
{
    uint32_t pclk;
    uint32_t timclk;

    /* TIM1 and TIM8 are on APB2 */
    if (htim->Instance == TIM1  ||
        htim->Instance == TIM8  ||
        htim->Instance == TIM15 ||
        htim->Instance == TIM16 ||
        htim->Instance == TIM17)
    {
        pclk = HAL_RCC_GetPCLK2Freq();

        /* On STM32H7: timer clock = PCLKx * 2 if APBx prescaler != 1 */
        if (HAL_RCC_GetHCLKFreq() != HAL_RCC_GetPCLK2Freq())
            timclk = pclk * 2;
        else
            timclk = pclk;
    }
    else
    {
        /* APB1 timers */
        pclk = HAL_RCC_GetPCLK1Freq();

        if (HAL_RCC_GetHCLKFreq() != HAL_RCC_GetPCLK1Freq())
            timclk = pclk * 2;
        else
            timclk = pclk;
    }

    return timclk;
}

void FlowMeter_Init(FlowMeter_ID_t id, TIM_HandleTypeDef *htim, uint32_t channel)
{
    if (id >= FLOW_METER_COUNT || htim == NULL) return;

    Internal_FlowMeter_t *p_meter = &g_meters[id];

    p_meter->htim = htim;
    p_meter->channel = channel;
    p_meter->tim_clk_hz = TIM_GetClock(htim);
    p_meter->last_capture = 0;
    p_meter->last_overflow_count = s_tim4_overflow_count;
    p_meter->is_first_capture = 1;
    p_meter->frequency_hz = 0.0f;
    p_meter->flow_rate_lpm = 0.0f;
    p_meter->last_interrupt_time = HAL_GetTick();
    p_meter->is_configured = 1;
    p_meter->sample_idx   = 0;
    p_meter->sample_count = 0;
    p_meter->freq_sum     = 0.0f;
    for (uint8_t i = 0; i < FLOW_AVG_SAMPLES; i++) {
        p_meter->freq_samples[i] = 0.0f;
    }

    /* Avvia l'hardware */
    HAL_TIM_IC_Start_IT(p_meter->htim, p_meter->channel);

    /* Avvia l'interrupt di Update (overflow) sullo stesso timer, una sola
     * volta (condiviso tra FLOW_METER_1/2): serve a FlowMeter_PeriodElapsedCallback()
     * per contare gli overflow tra due capture, vedi banner su
     * s_tim4_overflow_count sopra. HAL_TIM_IC_Start_IT() da solo abilita solo
     * CCxIE, non UIE. */
    if (!s_tim4_base_started) {
        HAL_TIM_Base_Start_IT(p_meter->htim);
        s_tim4_base_started = 1;
    }
}

/* CALLBACK GLOBALE: da agganciare da HAL_TIM_PeriodElapsedCallback() (main.c)
 * per l'evento di Update (overflow) del timer condiviso dai flussometri. */
void FlowMeter_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim == NULL) return;

    /* Unico contatore condiviso: incrementa per qualunque flussometro
     * configurato su questo htim (vedi banner sopra). */
    for (int i = 0; i < FLOW_METER_COUNT; i++) {
        if (g_meters[i].is_configured && g_meters[i].htim->Instance == htim->Instance) {
            s_tim4_overflow_count++;
            break;
        }
    }
}

/* CALLBACK GLOBALE: Viene chiamata passando solo il puntatore dell'HAL.
 * Cerca autonomamente a quale istanza interna appartiene l'interrupt. */
void FlowMeter_CaptureCallback(TIM_HandleTypeDef *htim)
{
    if (htim == NULL) return;

    uint32_t active_ch = htim->Channel;

    /* Cicla su tutte le istanze configurate nel driver */
    for (int i = 0; i < FLOW_METER_COUNT; i++)
    {
        Internal_FlowMeter_t *p_meter = &g_meters[i];

        if (!p_meter->is_configured) continue;

        /* Verifica corrispondenza del Timer Fisico (es. TIM4 == TIM4) e del canale attivo */
        if (htim->Instance == p_meter->htim->Instance)
        {
            if ((p_meter->channel == TIM_CHANNEL_1 && active_ch == HAL_TIM_ACTIVE_CHANNEL_1) ||
                (p_meter->channel == TIM_CHANNEL_2 && active_ch == HAL_TIM_ACTIVE_CHANNEL_2))
            {
                /* Trovato il flussometro corretto! Esegui il calcolo */
                uint32_t current_capture = HAL_TIM_ReadCapturedValue(htim, p_meter->channel);
                uint32_t period_ticks = 0;

                /* Snapshot del contatore overflow condiviso: eventuale
                 * overflow "in corsa" tra la lettura qui e l'incremento in
                 * FlowMeter_PeriodElapsedCallback() introduce al massimo un
                 * errore di 1 ciclo (65536 tick, ~238us a 275MHz) su periodi
                 * dell'ordine dei ms/decine di ms — trascurabile rispetto
                 * alla tolleranza +-3% del sensore. */
                uint32_t overflow_now = s_tim4_overflow_count;

                if (!p_meter->is_first_capture)
                {
                    uint32_t ovf_delta = overflow_now - p_meter->last_overflow_count;

                    /* Ticks totali trascorsi = overflow completi (ovf_delta *
                     * 65536) + delta grezzo tra le due catture. Sostituisce
                     * la vecchia correzione "a singolo wrap" (insufficiente:
                     * a piena velocità del timer, senza prescaler, tra due
                     * impulsi del flussometro il contatore a 16 bit va in
                     * overflow decine di volte, non una sola — da qui le
                     * letture GET FLOW incoerenti/enormi osservate). */
                    period_ticks = (ovf_delta << 16) + current_capture - p_meter->last_capture;

                    if (period_ticks > 0) {
                        float raw_freq = (float)p_meter->tim_clk_hz / (float)period_ticks;
                        float avg_freq = Push_FreqSample(p_meter, raw_freq);
                        float raw_lpm  = avg_freq / FLOW_METER_K_FACTOR;

                        p_meter->frequency_hz = avg_freq;
                        /* Linearizzazione vs sensore da banco — vedi banner
                         * FLOW_CAL_GAIN/FLOW_CAL_OFFSET sopra. */
                        p_meter->flow_rate_lpm = (FLOW_CAL_GAIN * raw_lpm) + FLOW_CAL_OFFSET;
                    }
                } else {
                    p_meter->is_first_capture = 0;
                }

                p_meter->last_capture = current_capture;
                p_meter->last_overflow_count = overflow_now;
                p_meter->last_interrupt_time = HAL_GetTick();

                break; /* Esci dal ciclo for (l'evento è stato gestito) */
            }
        }
    }
}

float FlowMeter_GetFrequency(FlowMeter_ID_t id)
{
    if (id >= FLOW_METER_COUNT) return 0.0f;
    Check_Timeout(&g_meters[id]);
    return g_meters[id].frequency_hz;
}

float FlowMeter_GetFlowRate(FlowMeter_ID_t id)
{
    if (id >= FLOW_METER_COUNT) return 0.0f;
    Check_Timeout(&g_meters[id]);
    return g_meters[id].flow_rate_lpm;
}
