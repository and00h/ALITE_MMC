/*
 * eFuse.c
 *
 * Implementazione del driver eFuse.
 * Vedere eFuse.h per documentazione API e architettura.
 */

#include "eFuse.h"
#include "main.h"   /* definizioni pin CubeMX */
#include "config.h" /* g_config.efuse_enabled_mask */

/* ========================================================================== */
/* --- MAPPA HARDWARE --- */
/* ========================================================================== */

/*
 * Descrizione fisica di ogni eFuse: pin GPIO e canale ADC per la corrente.
 *
 * SHDN polarity:
 *   EFUSE_MAIN  → MAIN_PWR_SHDN (active HIGH): GPIO_PIN_SET  = eFuse OFF
 *   altri       → nXXX_PWR_SHDN (active LOW):  GPIO_PIN_RESET = eFuse OFF
 */
typedef struct {
    /* SHDN */
    GPIO_TypeDef *shdn_port;
    uint16_t      shdn_pin;
    GPIO_PinState shdn_off_level;   /**< Livello da scrivere per SPEGNERE l'eFuse */

    /* GOOD (input, active HIGH) */
    GPIO_TypeDef *good_port;
    uint16_t      good_pin;

    /* FLT (input, active LOW) */
    GPIO_TypeDef *flt_port;
    uint16_t      flt_pin;

    /* ADC per corrente */
    uint8_t       adc_instance;     /**< 1 = ADC1, 3 = ADC3 */
    uint32_t      adc_channel;      /**< ADC_CHANNEL_x da stm32h7xx_hal_adc.h */

    /*
     * Denominatore per conversione ADC→mA (TPS16630, Vref=3.3V, ADC 12-bit):
     *   I_mA = raw * 3300 / imon_denom
     *
     *   Gain TPS16630: 27.9 µA/A  → resistenza sensing R [Ω]
     *   denom = round(4095 × 27.9e-6 × R_ohm × 1e3)
     *         = round(4095 × 0.0279 × R_kΩ)
     *
     *   R=36kΩ  (MAIN):   denom = round(4095 × 0.0279 × 36)  = 4113
     *   R=107kΩ (periph): denom = round(4095 × 0.0279 × 107) = 12225
     */
    uint16_t      imon_denom;
} EFuse_hw_t;

/* Denominatori precalcolati */
#define EFUSE_IMON_DENOM_MAIN    4113U    /* R=36kΩ  → ~0.803 mA/LSB, max ~3285mA */
#define EFUSE_IMON_DENOM_PERIPH 12225U    /* R=107kΩ → ~0.270 mA/LSB, max ~1106mA */

static const EFuse_hw_t hw_map[EFUSE_COUNT] = {

    [EFUSE_MAIN] = {
        .shdn_port      = MAIN_PWR_SHDN_GPIO_Port,
        .shdn_pin       = MAIN_PWR_SHDN_Pin,
        .shdn_off_level = GPIO_PIN_SET,          /* active HIGH: SET = spento */
        .good_port      = MAIN_PWR_GOOD_GPIO_Port,
        .good_pin       = MAIN_PWR_GOOD_Pin,
        .flt_port       = nMAIN_PWR_FLT_5V_GPIO_Port,
        .flt_pin        = nMAIN_PWR_FLT_5V_Pin,
        .adc_instance   = 1,
        .adc_channel    = ADC_CHANNEL_5,         /* PB1 = ADC1_IN5 */
        .imon_denom     = EFUSE_IMON_DENOM_MAIN,
    },

    [EFUSE_SAB] = {
        .shdn_port      = nSAB_PWR_SHDN_GPIO_Port,
        .shdn_pin       = nSAB_PWR_SHDN_Pin,
        .shdn_off_level = GPIO_PIN_RESET,        /* active LOW: RESET = spento */
        .good_port      = SAB_PWR_GOOD_GPIO_Port,
        .good_pin       = SAB_PWR_GOOD_Pin,
        .flt_port       = nSAB_PWR_FLT_GPIO_Port,
        .flt_pin        = nSAB_PWR_FLT_Pin,
        .adc_instance   = 1,
        .adc_channel    = ADC_CHANNEL_15,        /* PA3 = ADC1_IN15 */
        .imon_denom     = EFUSE_IMON_DENOM_PERIPH,
    },

    [EFUSE_COM] = {
        .shdn_port      = nCOM_PWR_SHDN_GPIO_Port,
        .shdn_pin       = nCOM_PWR_SHDN_Pin,
        .shdn_off_level = GPIO_PIN_RESET,
        .good_port      = COM_PWR_GOOD_GPIO_Port,
        .good_pin       = COM_PWR_GOOD_Pin,
        .flt_port       = nCOM_PWR_FLT_GPIO_Port,
        .flt_pin        = nCOM_PWR_FLT_Pin,
        .adc_instance   = 3,
        .adc_channel    = ADC_CHANNEL_5,         /* PF3 = ADC3_IN5 */
        .imon_denom     = EFUSE_IMON_DENOM_PERIPH,
    },

    [EFUSE_LASEQ] = {
        .shdn_port      = nLASE_Q_PWR_SHDN_GPIO_Port,
        .shdn_pin       = nLASE_Q_PWR_SHDN_Pin,
        .shdn_off_level = GPIO_PIN_RESET,
        .good_port      = LASE_Q_PWR_GOOD_GPIO_Port,
        .good_pin       = LASE_Q_PWR_GOOD_Pin,
        .flt_port       = nLASE_Q_PWR_FLT_GPIO_Port,
        .flt_pin        = nLASE_Q_PWR_FLT_Pin,
        .adc_instance   = 1,
        .adc_channel    = ADC_CHANNEL_4,         /* PC4 = ADC1_IN4 */
        .imon_denom     = EFUSE_IMON_DENOM_PERIPH,
    },
};

/* ========================================================================== */
/* --- STATO INTERNO --- */
/* ========================================================================== */

static EFuse_status_t s_status[EFUSE_COUNT];

static ADC_HandleTypeDef *s_hadc1 = NULL;
static ADC_HandleTypeDef *s_hadc3 = NULL;

/* ========================================================================== */
/* --- FUNZIONI PRIVATE --- */
/* ========================================================================== */

/*
 * @brief  Legge un canale ADC in modalità polling (singola conversione).
 *         Riconfigura il canale a runtime — sicuro in modalità non-DMA.
 *
 * @param  hadc     Handle ADC già inizializzato da CubeMX.
 * @param  channel  ADC_CHANNEL_x da leggere.
 * @return Valore grezzo [0..4095], oppure 0xFFFF in caso di errore HAL.
 */
static uint16_t adc_read_channel(ADC_HandleTypeDef *hadc, uint32_t channel)
{
    ADC_ChannelConfTypeDef cfg = {
        .Channel      = channel,
        .Rank         = ADC_REGULAR_RANK_1,
        .SamplingTime = ADC_SAMPLETIME_64CYCLES_5,  /* ~2µs @170MHz, sufficiente per I_MON */
        .SingleDiff   = ADC_SINGLE_ENDED,
        .OffsetNumber = ADC_OFFSET_NONE,
        .Offset       = 0,
    };

    if (HAL_ADC_ConfigChannel(hadc, &cfg) != HAL_OK) return 0xFFFFU;
    if (HAL_ADC_Start(hadc)               != HAL_OK) return 0xFFFFU;
    if (HAL_ADC_PollForConversion(hadc, 5) != HAL_OK) {
        HAL_ADC_Stop(hadc);
        return 0xFFFFU;
    }
    uint16_t val = (uint16_t)HAL_ADC_GetValue(hadc);
    HAL_ADC_Stop(hadc);
    return val;
}

/*
 * @brief  Legge il pin FLT (active LOW): restituisce true se il fault è attivo.
 */
static inline bool read_fault(const EFuse_hw_t *hw)
{
    return (HAL_GPIO_ReadPin(hw->flt_port, hw->flt_pin) == GPIO_PIN_RESET);
}

/*
 * @brief  Legge il pin GOOD (active HIGH): restituisce true se la tensione è ok.
 */
static inline bool read_good(const EFuse_hw_t *hw)
{
    return (HAL_GPIO_ReadPin(hw->good_port, hw->good_pin) == GPIO_PIN_SET);
}

/* ========================================================================== */
/* --- API PUBBLICA --- */
/* ========================================================================== */

void EFuse_Init(ADC_HandleTypeDef *hadc1, ADC_HandleTypeDef *hadc3)
{
    s_hadc1 = hadc1;
    s_hadc3 = hadc3;

    /*
     * Stato iniziale: ogni eFuse abilitato o tenuto spento in base al bit
     * corrispondente in g_config.efuse_enabled_mask (bit0=MAIN, bit1=SAB,
     * bit2=COM, bit3=LASEQ — vedi config.h). Applicata UNA SOLA VOLTA qui,
     * al boot, prima che qualunque altro driver tenti di comunicare con un
     * modulo eventualmente mascherato (COM_Interface_Init/LaseQ_Init
     * girano comunque, ma senza alimentazione a valle il link resta
     * semplicemente assente — gestito come un normale fault di
     * comunicazione da FAULT_BIT_COM/FAULT_BIT_LASEQ, task_comms.c).
     *
     * Usa sempre EFuse_Enable()/EFuse_Disable() (mai scrivere il pin SHDN
     * direttamente): la polarità non è uniforme, EFUSE_MAIN è active HIGH
     * (SHDN=HIGH per spegnere) mentre SAB/COM/LASEQ sono active LOW
     * (SHDN=LOW per spegnere) — vedi hw_map[] sopra.
     */
    for (EFuse_id_t id = 0; id < EFUSE_COUNT; id++) {
        bool masked_on = (g_config.efuse_enabled_mask & (1UL << (uint32_t)id)) != 0UL;

        if (masked_on) {
            EFuse_Enable(id);
        } else {
            EFuse_Disable(id);
        }
        s_status[id].power_good  = false;  /* verrà aggiornato alla prima UpdateAll */
        s_status[id].fault       = false;
        s_status[id].current_raw = 0U;
        s_status[id].current_ma  = 0U;
    }
}

void EFuse_Enable(EFuse_id_t id)
{
    if (id >= EFUSE_COUNT) return;
    const EFuse_hw_t *hw = &hw_map[id];

    /* Livello opposto a shdn_off_level = eFuse ON */
    GPIO_PinState on_level = (hw->shdn_off_level == GPIO_PIN_SET)
                             ? GPIO_PIN_RESET
                             : GPIO_PIN_SET;
    HAL_GPIO_WritePin(hw->shdn_port, hw->shdn_pin, on_level);
    s_status[id].enabled = true;
}

void EFuse_Disable(EFuse_id_t id)
{
    if (id >= EFUSE_COUNT) return;
    const EFuse_hw_t *hw = &hw_map[id];

    HAL_GPIO_WritePin(hw->shdn_port, hw->shdn_pin, hw->shdn_off_level);
    s_status[id].enabled = false;
}

void EFuse_DisableAll(void)
{
    for (EFuse_id_t id = 0; id < EFUSE_COUNT; id++) {
        EFuse_Disable(id);
    }
}

void EFuse_UpdateAll(void)
{
    for (EFuse_id_t id = 0; id < EFUSE_COUNT; id++) {
        const EFuse_hw_t *hw = &hw_map[id];

        /* GPIO */
        s_status[id].power_good = read_good(hw);
        s_status[id].fault      = read_fault(hw);

        /* ADC corrente: lettura grezza + conversione in mA */
        uint16_t raw = 0U;
        if (hw->adc_instance == 1 && s_hadc1 != NULL) {
            raw = adc_read_channel(s_hadc1, hw->adc_channel);
        } else if (hw->adc_instance == 3 && s_hadc3 != NULL) {
            raw = adc_read_channel(s_hadc3, hw->adc_channel);
        }

        if (raw == 0xFFFFU) {
            /* Errore ADC: mantieni ultimi valori validi */
            continue;
        }

        s_status[id].current_raw = raw;

        /*
         * Conversione: I_mA = raw * Vref_mV / (4095 * gain * R_ohm)
         *                    = raw * 3300 / imon_denom
         * Overflow check: 4095 * 3300 = 13,513,500 < 2^32 ✓
         */
        s_status[id].current_ma = (uint16_t)((uint32_t)raw * 3300U / hw->imon_denom);
    }
}

EFuse_status_t EFuse_GetStatus(EFuse_id_t id)
{
    if (id >= EFUSE_COUNT) {
        EFuse_status_t empty = {0};
        return empty;
    }
    return s_status[id];
}

bool EFuse_IsOk(EFuse_id_t id)
{
    if (id >= EFUSE_COUNT) return false;
    const EFuse_status_t *st = &s_status[id];
    return st->enabled && st->power_good && !st->fault;
}
