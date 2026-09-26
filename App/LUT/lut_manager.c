/*
 * lut_manager.c  --  MMC
 *
 * Implementazione del modulo LUT.
 *
 * LAYOUT FLASH (settore 7, condiviso col journal di config.c — vedi
 * flash_map.h per la spiegazione completa e la storia del bug del
 * 2026-07-02, quando LUT_FLASH_ADDR era erroneamente fissato a
 * CONFIG_FLASH_BASE_ADDR+256 e collideva col secondo record del journal):
 *   [CONFIG_FLASH_BASE_ADDR .. +CONFIG_JOURNAL_SIZE) : journal ConfigRecord_t (config.c)
 *   [LUT_FLASH_BASE_ADDR    .. +LUT_RESERVED_SIZE)   : LUT_Store_t, blocco fisso (qui)
 *
 * L'erase del settore 7 (sotto, in LUT_Save()) cancella FISICAMENTE anche
 * il journal di config.c: subito dopo aver riscritto la LUT, richiamiamo
 * Config_Save() per ripristinare il record Config corrente nel journal
 * ora vuoto, altrimenti ogni SAVE LUT/RESET LUT perderebbe silenziosamente
 * tutta la configurazione persistita.
 *
 * CRC32: algoritmo standard (poly 0xEDB88320, init 0xFFFFFFFF, reflected).
 * Calcolato su tutti i byte di LUT_Store_t prima del campo crc32.
 *
 * VALORI DI DEFAULT (laser 550W, corrente nominale 10500mA / lifecycle 12000mA):
 *   Tutti i range PD sono PLACEHOLDER — calibrare durante il commissioning.
 *
 *   gain_threshold_sw (current_ma → finestra guadagno MUX):
 *     win0: < 2625mA  (≈ 25% I_max)
 *     win1: < 5250mA  (≈ 50% I_max)
 *     win2: < 7875mA  (≈ 75% I_max)
 *     win3: >= 7875mA
 *
 *   gain_threshold_hw (pa0_adc → finestra guadagno MUX):
 *     win0: < 1024    win1: < 2048    win2: < 3072    win3: >= 3072
 *
 *   pd_valid SW — placeholder uniformi per tutti i PD (calibrare):
 *     @    0mA: [0, 50]         (buio)
 *     @ 3500mA: [500, 900]      (~33% potenza)
 *     @ 7000mA: [1200, 1800]    (~67% potenza)
 *     @10500mA: [2000, 2800]    (100% potenza)
 *
 *   pd_valid HW — placeholder:
 *     @ pa0=0:    [0, 50]
 *     @ pa0=1024: [500, 900]
 *     @ pa0=2048: [1200, 1800]
 *     @ pa0=3072: [2000, 2800]
 *
 *   pd_power (ADC → Watt, 550W full scale — identica per tutti i PD):
 *     @ ADC=0:      0 W
 *     @ ADC=1024: 137 W
 *     @ ADC=2048: 275 W
 *     @ ADC=3072: 412 W
 *     @ ADC=4095: 550 W
 *
 *   hw_power (RAW ADC2 a 16 bit, pin PC5/LPWR_SET_ISO → % potenza HW,
 *   NUOVO 2026-08-01, PLACEHOLDER lineare — calibrare):
 *     @ ADC=0:       0%      @ ADC=26214:  40%     @ ADC=52428:  80%
 *     @ ADC=6554:   10%      @ ADC=32768:  50%     @ ADC=58982:  90%
 *     @ ADC=13107:  20%      @ ADC=39321:  60%     @ ADC=65535: 100%
 *     @ ADC=19661:  30%      @ ADC=45875:  70%
 *
 *   voltage_comp (current_ma → voltage_mv, NUOVO 2026-07-22, valori REALI
 *   inseriti 2026-08-01 — non un placeholder, curva di calibrazione
 *   fornita dal team, a differenza delle altre sotto-tabelle sopra):
 *     @  1000mA: 31500mV   @  5000mA: 34800mV   @  9000mA: 37200mV
 *     @  2000mA: 32500mV   @  6000mA: 35400mV   @ 10500mA: 37850mV
 *     @  3000mA: 33500mV   @  7000mA: 36000mV
 *     @  4000mA: 34000mV   @  8000mA: 36800mV
 *   Sotto il minimo (1000mA): AMC non estrapola linearmente, restituisce
 *   31500mV costante — vedi psu_voltage_comp.c (progetto AMC).
 */

#include "lut_manager.h"
#include "stm32h7xx_hal.h"
#include <string.h>
#include <stddef.h>
#include "cmsis_os.h"         /* osWaitForever, Config_FlashMutexAcquire/Release */
#include "config.h"          /* Config_Save(), g_config — ripristino dopo erase condiviso */
#include "hal_handles.h"     /* hiwdg1 */

/* ============================================================================
 * INDIRIZZO FLASH LUT
 * ============================================================================ */

/* Blocco fisso riservato in fondo al settore 7, separato dal journal Config
 * (vedi flash_map.h per il layout completo e la storia del bug che questo
 * static_assert previene per il futuro). */
#define LUT_FLASH_ADDR   LUT_FLASH_BASE_ADDR

_Static_assert(
    sizeof(LUT_Store_t) <= LUT_RESERVED_SIZE,
    "LUT_Store_t supera lo spazio riservato in fondo al settore 7 (vedi "
    "flash_map.h, LUT_RESERVED_SIZE): aumentare la riserva prima di continuare."
);

/* ============================================================================
 * STATO INTERNO
 * ============================================================================ */

static LUT_Store_t s_lut;

/* ============================================================================
 * CRC32 (standard IEEE 802.3, poly riflesso 0xEDB88320)
 * ============================================================================ */

static uint32_t crc32_compute(const uint8_t *data, uint32_t len)
{
    uint32_t crc = 0xFFFFFFFFUL;
    for (uint32_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (uint8_t b = 0; b < 8U; b++) {
            crc = (crc & 1U) ? ((crc >> 1U) ^ 0xEDB88320UL) : (crc >> 1U);
        }
    }
    return crc ^ 0xFFFFFFFFUL;
}

static uint32_t lut_crc(const LUT_Store_t *s)
{
    /* CRC calcolato su tutti i campi tranne crc32 stesso */
    return crc32_compute((const uint8_t *)s,
                         (uint32_t)offsetof(LUT_Store_t, crc32));
}

/* ============================================================================
 * VALORI DI DEFAULT
 * ============================================================================ */

static void load_defaults(void)
{
    memset(&s_lut, 0, sizeof(s_lut));
    s_lut.magic = LUT_MAGIC;

    /* --- Soglie guadagno SW (basate su current_ma) --- */
    s_lut.gain_threshold_sw[0] = 2625U;   /* win0: < 2625mA  (~25% di 10500mA) */
    s_lut.gain_threshold_sw[1] = 5250U;   /* win1: < 5250mA  (~50%) */
    s_lut.gain_threshold_sw[2] = 7875U;   /* win2: < 7875mA  (~75%) */
    s_lut.gain_threshold_sw[3] = 0xFFFFU; /* win3: open-ended */

    /* --- Soglie guadagno HW (basate su pa0_adc) --- */
    s_lut.gain_threshold_hw[0] = 1024U;
    s_lut.gain_threshold_hw[1] = 2048U;
    s_lut.gain_threshold_hw[2] = 3072U;
    s_lut.gain_threshold_hw[3] = 0xFFFFU;

    /* --- LUT validazione PD: 4 entry per PD per modo --- */
    s_lut.pd_valid_size = LUT_PD_VALID_SIZE;

    /*
     * Valori dummy identici per tutti i PD -- calibrare per singolo canale
     * durante il commissioning tramite LUT_SetPDValidEntry() + LUT_Save().
     */
    static const LUT_PDValidEntry_t sw_default[LUT_PD_VALID_SIZE] = {
        {     0U,    0U,   50U },  /* buio — da calibrare */
        {  3500U,  500U,  900U },  /* ~33% I_max — placeholder */
        {  7000U, 1200U, 1800U },  /* ~67% I_max — placeholder */
        { 10500U, 2000U, 2800U },  /* 100% I_max (10.5A) — placeholder */
    };
    static const LUT_PDValidEntry_t hw_default[LUT_PD_VALID_SIZE] = {
        {    0U,    0U,   50U },  /* buio — placeholder */
        { 1024U,  500U,  900U },  /* ~25% ADC range — placeholder */
        { 2048U, 1200U, 1800U },  /* ~50% — placeholder */
        { 3072U, 2000U, 2800U },  /* ~75% — placeholder */
    };

    for (uint8_t p = 0; p < LUT_PD_MAX; p++) {
        memcpy(s_lut.pd_valid[LUT_MODE_SW][p], sw_default,
               sizeof(sw_default));
        memcpy(s_lut.pd_valid[LUT_MODE_HW][p], hw_default,
               sizeof(hw_default));
    }

    /* --- LUT ADC -> Watt (550W full scale, per PD x finestra guadagno — placeholder) ---
     *
     * La potenza corrispondente a un dato ADC_raw dipende dal guadagno attivo.
     * Esempio con 4 finestre (fattori guadagno puramente illustrativi — calibrare):
     *   Win0 (gain x1):  ADC 4095 ≈ 550W  → piena scala
     *   Win1 (gain x2):  ADC 4095 ≈ 275W  → metà scala ADC = potenza piena
     *   Win2 (gain x4):  ADC 4095 ≈ 137W
     *   Win3 (gain x8):  ADC 4095 ≈  69W
     * I valori qui sotto sono PLACEHOLDER lineari. Sostituire con misure reali.
     */
    s_lut.pd_power_size = LUT_PD_POWER_SIZE;

    /* [win0] guadagno minimo: range pieno 0-550W */
    static const LUT_PDPowerEntry_t pw0[LUT_PD_POWER_SIZE] = {
        {    0U,   0U }, { 1024U, 137U }, { 2048U, 275U }, { 3072U, 412U }, { 4095U, 550U },
    };
    /* [win1] ~x2: full scale ADC ≈ 275W */
    static const LUT_PDPowerEntry_t pw1[LUT_PD_POWER_SIZE] = {
        {    0U,   0U }, { 1024U,  68U }, { 2048U, 137U }, { 3072U, 206U }, { 4095U, 275U },
    };
    /* [win2] ~x4: full scale ADC ≈ 137W */
    static const LUT_PDPowerEntry_t pw2[LUT_PD_POWER_SIZE] = {
        {    0U,   0U }, { 1024U,  34U }, { 2048U,  68U }, { 3072U, 103U }, { 4095U, 137U },
    };
    /* [win3] ~x8: full scale ADC ≈ 68W */
    static const LUT_PDPowerEntry_t pw3[LUT_PD_POWER_SIZE] = {
        {    0U,   0U }, { 1024U,  17U }, { 2048U,  34U }, { 3072U,  51U }, { 4095U,  68U },
    };

    for (uint8_t p = 0; p < LUT_PD_MAX; p++) {
        memcpy(&s_lut.pd_power[p][0][0], pw0, sizeof(pw0));
        memcpy(&s_lut.pd_power[p][1][0], pw1, sizeof(pw1));
        memcpy(&s_lut.pd_power[p][2][0], pw2, sizeof(pw2));
        memcpy(&s_lut.pd_power[p][3][0], pw3, sizeof(pw3));
    }

    /*
     * --- LUT compensazione tensione PSU (NUOVO 2026-07-22, valori reali
     *     inseriti 2026-08-01) ---
     *
     * Curva di calibrazione reale (non più placeholder vuoto): 10 entry
     * current_ma -> voltage_mv, ordine crescente di current_ma (vincolo
     * richiesto da AMC/psu_voltage_comp.c). Sotto il minimo (1000mA) AMC
     * NON estrapola linearmente all'indietro: restituisce il voltage_mv
     * della prima entry (31500mV) come valore costante — vedi
     * psu_voltage_comp.c, PSUComp_LutLookup() (progetto AMC).
     */
    s_lut.voltage_comp_size = LUT_VCOMP_SIZE;
    static const LUT_VoltageCompEntry_t vcomp_default[LUT_VCOMP_SIZE] = {
        {  1000U, 31500U }, {  2000U, 32500U }, {  3000U, 33500U }, {  4000U, 34000U },
        {  5000U, 34800U }, {  6000U, 35400U }, {  7000U, 36000U }, {  8000U, 36800U },
        {  9000U, 37200U }, { 10500U, 37850U },
    };
    memcpy(s_lut.voltage_comp, vcomp_default, sizeof(vcomp_default));

    /*
     * --- LUT setpoint HW: RAW ADC (PC5/LPWR_SET_ISO) -> % potenza (NUOVO 2026-08-01) ---
     *
     * Default: retta lineare 0->0% ... 65535->100% (risoluzione ADC2 a 16
     * bit, vedi MX_ADC2_Init(), main.c), 11 entry a passo 10%. PLACEHOLDER,
     * come pd_power sopra: il partitore/isolatore analogico a monte di PC5
     * puo' non essere perfettamente lineare — calibrare durante il
     * commissioning con "SET LUT HWPWR" + "SAVE LUT".
     */
    s_lut.hw_power_size = LUT_HWPWR_SIZE;
    static const LUT_HwPowerEntry_t hwpwr_default[LUT_HWPWR_SIZE] = {
        {     0U,   0U }, {  6554U,  10U }, { 13107U,  20U }, { 19661U,  30U },
        { 26214U,  40U }, { 32768U,  50U }, { 39321U,  60U }, { 45875U,  70U },
        { 52428U,  80U }, { 58982U,  90U }, { 65535U, 100U },
    };
    memcpy(s_lut.hw_power, hwpwr_default, sizeof(hwpwr_default));

    s_lut.crc32 = lut_crc(&s_lut);
}

/* ============================================================================
 * FLASH READ / WRITE
 * ============================================================================ */

static bool flash_read(void)
{
    const LUT_Store_t *flash_ptr = (const LUT_Store_t *)LUT_FLASH_ADDR;

    if (flash_ptr->magic != LUT_MAGIC) return false;

    /* Copia in RAM e verifica CRC */
    memcpy(&s_lut, flash_ptr, sizeof(LUT_Store_t));
    uint32_t expected = lut_crc(&s_lut);
    return (s_lut.crc32 == expected);
}

/*
 * Scrive s_lut nel blocco flash riservato (LUT_FLASH_ADDR), in blocchi da
 * FLASH_WRITE_GRANULARITY byte. NON cancella il settore: il chiamante deve
 * aver già eseguito l'erase (sia esso LUT_Save() stessa, sia — dal
 * 2026-07-27 — Config_Save() nel proprio ramo di overflow del journal, che
 * condivide fisicamente lo stesso settore 7, vedi LUT_RestoreAfterSectorErase()
 * in lut_manager.h). Gestisce da sola acquisizione/rilascio del mutex flash
 * e lock/unlock HAL, cosi' da poter essere richiamata sia da LUT_Save() sia
 * dall'esterno senza che il chiamante debba conoscerne i dettagli.
 */
static bool lut_write_to_flash(void)
{
    s_lut.crc32 = lut_crc(&s_lut);

    /*
     * Mutex condiviso col journal Config (vedi config.c, s_flash_mutex):
     * NON usare taskENTER_CRITICAL() qui — mascherare gli interrupt ferma
     * anche SysTick, disabilitando il timeout software di HAL_FLASH_Program()/
     * HAL_FLASHEx_Erase() e trasformando un eventuale errore hardware in un
     * hang che dipende dal reset IWDG per uscirne (bug osservato il
     * 2026-07-02 sul journal Config, stesso principio qui). Il mutex dà la
     * mutua esclusione necessaria (contro Config_Save() concorrente sullo
     * stesso settore fisico) senza questo effetto collaterale.
     */
    if (!Config_FlashMutexAcquire(osWaitForever)) {
        return false;
    }

    HAL_FLASH_Unlock();
    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_ALL_ERRORS_BANK1);

    /* Scrittura LUT in blocchi da 32 byte (FLASH_WRITE_GRANULARITY), nel
     * blocco riservato LUT_FLASH_ADDR (== LUT_FLASH_BASE_ADDR, in fondo al
     * settore, separato dal journal Config — vedi flash_map.h). */
    const uint8_t *src = (const uint8_t *)&s_lut;
    uint32_t dest = LUT_FLASH_ADDR;
    uint32_t remaining = sizeof(LUT_Store_t);

    /* Pad all'ultimo blocco con 0xFF */
    uint8_t block[FLASH_WRITE_GRANULARITY];

    while (remaining > 0) {
        uint32_t chunk = (remaining >= FLASH_WRITE_GRANULARITY)
                         ? FLASH_WRITE_GRANULARITY : remaining;

        memset(block, 0xFF, FLASH_WRITE_GRANULARITY);
        memcpy(block, src, chunk);

        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_FLASHWORD, dest,
                              (uint32_t)block) != HAL_OK) {
            HAL_FLASH_Lock();
            Config_FlashMutexRelease();
            return false;
        }

        src       += chunk;
        dest      += FLASH_WRITE_GRANULARITY;
        remaining  = (remaining > chunk) ? (remaining - chunk) : 0U;
    }

    HAL_FLASH_Lock();
    Config_FlashMutexRelease();

    SCB_InvalidateDCache_by_Addr((uint32_t *)CONFIG_FLASH_BASE_ADDR, (int32_t)CONFIG_FLASH_SIZE);
    __ISB();
    __DSB();

    return true;
}

/*
 * Scrittura in flash: la granularita' minima dell'H7 e' 32 byte.
 * LUT_Store_t (~300 byte) deve essere scritta in blocchi da 32 byte.
 * Il settore viene cancellato prima della scrittura.
 */
bool LUT_Save(void)
{
    /*
     * Mutex + erase gestiti qui (non in lut_write_to_flash(), che presuppone
     * un settore già cancellato): NON usare taskENTER_CRITICAL() qui —
     * mascherare gli interrupt ferma anche SysTick, disabilitando il
     * timeout software di HAL_FLASHEx_Erase() e trasformando un eventuale
     * errore hardware in un hang che dipende dal reset IWDG per uscirne
     * (bug osservato il 2026-07-02 sul journal Config, stesso principio
     * qui). Il mutex dà la mutua esclusione necessaria (contro Config_Save()
     * concorrente sullo stesso settore fisico) senza questo effetto
     * collaterale.
     */
    if (!Config_FlashMutexAcquire(osWaitForever)) {
        return false;
    }

    /* Vedi commento in config.c/erase_config_sector(): sull'H7 single-bank
     * un erase blocca il fetch di codice dall'intero bank per la durata
     * dell'operazione, indipendentemente dal mascheramento interrupt.
     * L'IWDG (contatore hardware indipendente) continua a contare durante
     * lo stallo: la riarmiamo appena prima per massimizzare il margine. */
    HAL_IWDG_Refresh(&hiwdg1);

    HAL_FLASH_Unlock();
    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_ALL_ERRORS_BANK1);

    /* Cancellazione settore 7 — cancella FISICAMENTE anche il journal di
     * config.c, che condivide lo stesso settore (vedi flash_map.h). */
    FLASH_EraseInitTypeDef erase = {
        .TypeErase    = FLASH_TYPEERASE_SECTORS,
        .Banks        = FLASH_BANK_1,
        .Sector       = CONFIG_FLASH_SECTOR,
        .NbSectors    = 1,
        .VoltageRange = FLASH_VOLTAGE_RANGE_3,
    };
    uint32_t sector_error = 0;
    if (HAL_FLASHEx_Erase(&erase, &sector_error) != HAL_OK) {
        HAL_FLASH_Lock();
        Config_FlashMutexRelease();
        return false;
    }

    HAL_FLASH_Lock();
    Config_FlashMutexRelease();

    if (!lut_write_to_flash()) {
        return false;
    }

    /*
     * L'erase sopra ha cancellato anche il journal Config. Ripristiniamo
     * subito il record corrente (g_config, in RAM) nel journal ora vuoto,
     * cosi' SAVE LUT / RESET LUT non fanno perdere silenziosamente la
     * configurazione persistita. Un fallimento qui non invalida comunque
     * la LUT appena scritta (gia' valida in flash); g_config resta corretta
     * in RAM ma tornerebbe ai default al prossimo riavvio — non ideale ma
     * non distruttivo.
     */
    (void)Config_Save();

    return true;
}

bool LUT_RestoreAfterSectorErase(void)
{
    /* Vedi doc in lut_manager.h: il chiamante (Config_Save(), ramo di
     * overflow del journal) ha già cancellato l'intero settore 7 per conto
     * proprio — qui ci limitiamo a riscrivere s_lut (RAM) nel blocco
     * riservato, senza un secondo erase (che sarebbe ridondante, dato che il
     * settore è già vuoto) e senza richiamare Config_Save() (che è proprio
     * il chiamante: causerebbe una ricorsione infinita). */
    return lut_write_to_flash();
}

/* ============================================================================
 * API PUBBLICA
 * ============================================================================ */

void LUT_Init(void)
{
    if (!flash_read()) {
        load_defaults();
        /* Tentativo di persistenza dei default; non bloccante se fallisce */
        LUT_Save();
    }
}

const LUT_Store_t *LUT_Get(void)
{
    return &s_lut;
}

void LUT_ResetDefaults(void)
{
    load_defaults();
    LUT_Save();
}

void LUT_SetGainThreshold(uint8_t mode, uint8_t index, uint16_t value)
{
    if (mode > LUT_MODE_HW || index >= LUT_GAIN_WINDOWS) return;
    if (mode == LUT_MODE_SW) {
        s_lut.gain_threshold_sw[index] = value;
    } else {
        s_lut.gain_threshold_hw[index] = value;
    }
}

void LUT_SetPDValidEntry(uint8_t mode, uint8_t pd_idx, uint8_t entry_idx,
                         uint16_t setpoint, uint16_t pd_min, uint16_t pd_max)
{
    if (mode > LUT_MODE_HW || pd_idx >= LUT_PD_MAX ||
        entry_idx >= LUT_PD_VALID_SIZE) return;

    s_lut.pd_valid[mode][pd_idx][entry_idx].setpoint = setpoint;
    s_lut.pd_valid[mode][pd_idx][entry_idx].pd_min   = pd_min;
    s_lut.pd_valid[mode][pd_idx][entry_idx].pd_max   = pd_max;
}

void LUT_SetPDPowerEntry(uint8_t pd_idx, uint8_t gain_win, uint8_t entry_idx,
                         uint16_t adc, uint16_t power_w)
{
    if (pd_idx >= LUT_PD_MAX || gain_win >= LUT_GAIN_WINDOWS ||
        entry_idx >= LUT_PD_POWER_SIZE) return;
    s_lut.pd_power[pd_idx][gain_win][entry_idx].adc     = adc;
    s_lut.pd_power[pd_idx][gain_win][entry_idx].power_w = power_w;
}

uint16_t LUT_ConvertPDToWatt(uint8_t pd_idx, uint8_t gain_win, uint16_t adc_raw)
{
    if (pd_idx >= LUT_PD_MAX || gain_win >= LUT_GAIN_WINDOWS ||
        s_lut.pd_power_size == 0) return 0U;

    const LUT_PDPowerEntry_t *tbl = &s_lut.pd_power[pd_idx][gain_win][0];
    uint8_t n = s_lut.pd_power_size;

    /* Clamp inferiore */
    if (adc_raw <= tbl[0].adc) return tbl[0].power_w;
    /* Clamp superiore */
    if (adc_raw >= tbl[n - 1U].adc) return tbl[n - 1U].power_w;

    /* Interpolazione lineare */
    for (uint8_t i = 0U; i < (n - 1U); i++) {
        if (adc_raw >= tbl[i].adc && adc_raw < tbl[i + 1U].adc) {
            uint16_t span   = tbl[i + 1U].adc     - tbl[i].adc;
            uint16_t delta  = adc_raw              - tbl[i].adc;
            int32_t  d_pow  = (int32_t)tbl[i + 1U].power_w - (int32_t)tbl[i].power_w;
            return (uint16_t)((int32_t)tbl[i].power_w
                              + (d_pow * (int32_t)delta) / (int32_t)span);
        }
    }
    return 0U;
}

void LUT_SetVoltageCompEntry(uint8_t entry_idx, uint16_t current_ma, uint16_t voltage_mv)
{
    if (entry_idx >= LUT_VCOMP_SIZE) return;

    s_lut.voltage_comp[entry_idx].current_ma = current_ma;
    s_lut.voltage_comp[entry_idx].voltage_mv = voltage_mv;

    /* Crescita incrementale del conteggio entry, stesso principio "in ordine
     * 0,1,2..." documentato in lut_manager.h. Non decresce mai qui: per
     * accorciare la LUT si passa da LUT_ResetDefaults() o si sovrascrivono
     * le entry in coda con voltage_mv=0 (trattate comunque come "non
     * valide" da AMC in fase di interpolazione). */
    if ((uint8_t)(entry_idx + 1U) > s_lut.voltage_comp_size) {
        s_lut.voltage_comp_size = (uint8_t)(entry_idx + 1U);
    }
}

void LUT_SetHwPowerEntry(uint8_t entry_idx, uint16_t adc_raw, uint8_t power_pct)
{
    if (entry_idx >= LUT_HWPWR_SIZE) return;
    if (power_pct > 100U) power_pct = 100U;

    s_lut.hw_power[entry_idx].adc_raw   = adc_raw;
    s_lut.hw_power[entry_idx].power_pct = power_pct;
}

uint8_t LUT_ConvertHwPowerToPct(uint16_t adc_raw)
{
    if (s_lut.hw_power_size == 0U) return 0U;

    const LUT_HwPowerEntry_t *tbl = &s_lut.hw_power[0];
    uint8_t n = s_lut.hw_power_size;

    /* Clamp inferiore/superiore */
    if (adc_raw <= tbl[0].adc_raw)          return tbl[0].power_pct;
    if (adc_raw >= tbl[n - 1U].adc_raw)     return tbl[n - 1U].power_pct;

    /* Interpolazione lineare (stesso schema di LUT_ConvertPDToWatt()) */
    for (uint8_t i = 0U; i < (n - 1U); i++) {
        if (adc_raw >= tbl[i].adc_raw && adc_raw < tbl[i + 1U].adc_raw) {
            uint16_t span = tbl[i + 1U].adc_raw - tbl[i].adc_raw;
            if (span == 0U) return tbl[i].power_pct; /* entry non monotona: evita div/0 */

            uint16_t delta = adc_raw - tbl[i].adc_raw;
            int32_t d_pct  = (int32_t)tbl[i + 1U].power_pct - (int32_t)tbl[i].power_pct;
            int32_t result = (int32_t)tbl[i].power_pct + (d_pct * (int32_t)delta) / (int32_t)span;

            if (result < 0)   result = 0;
            if (result > 100) result = 100;
            return (uint8_t)result;
        }
    }
    return 0U;
}
