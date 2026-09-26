/*
 * lut_manager.h  --  MMC
 *
 * Gestione delle LUT di sistema: storage in flash, defaults precompilati,
 * API di modifica puntuale e invio alle periferiche (AMC).
 *
 * STRUTTURA LUT:
 *
 *   1. gain_threshold_sw[4]   : soglie current_ma per finestre guadagno AMC (SW mode)
 *   2. gain_threshold_hw[4]   : soglie pa0_adc per finestre guadagno AMC (HW mode)
 *   3. pd_valid_sw[4][N]      : LUT validazione PD in SW mode (current_ma -> [min,max])
 *   4. pd_valid_hw[4][N]      : LUT validazione PD in HW mode (pa0_adc  -> [min,max])
 *   5. pd_power[4][4][N]      : LUT conversione ADC -> Potenza [W] per PD × finestra guadagno.
 *                               Lo stesso valore ADC letto in Win0 e Win2 corrisponde a
 *                               potenze diverse: la LUT deve essere indicizzata per guadagno.
 *                               MMC riceve gain_window nel campo STATUS, quindi ha entrambi.
 *   6. hw_power[N]            : LUT conversione RAW ADC (PC5/LPWR_SET_ISO, EXT_interface.c)
 *                               -> % potenza HW (NUOVO 2026-08-01). Usata solo per reporting
 *                               (GET STATUS RS485 / STATUS COM interface) quando
 *                               FSM_GetLaserModeWire() != 0 — vedi banner su LUT_HwPowerEntry_t.
 *
 * La LUT power%->current_ma e' gia' in g_config.setpoint_lut (21 entry, passo 5%).
 *
 * FLASH STORAGE:
 *   La struttura LUT_Store_t e' salvata nel settore 7 della flash,
 *   a LUT_FLASH_BASE_ADDR (dopo il blocco Config_t).
 *   Scrittura: cancella settore, riscrive intero blocco.
 *   Lettura: memcpy a RAM all'avvio.
 *
 * VALORI DI DEFAULT (precompilati per riferimento fotodiodi dummy):
 *   Laser 3kW ipotetico. I valori sono placeholder per il commissioning.
 *   Corrente massima LaseQ: 8000 mA.
 */

#ifndef APP_LUT_LUT_MANAGER_H_
#define APP_LUT_LUT_MANAGER_H_

#include <stdint.h>
#include <stdbool.h>
#include "flash_map.h"

/* ============================================================================
 * DIMENSIONI LUT
 * ============================================================================ */
#define LUT_PD_MAX              4U   /* Numero fotodiodi */
#define LUT_PD_VALID_SIZE       4U   /* Entry per PD per modo (interpolazione lineare) */
#define LUT_PD_POWER_SIZE       5U   /* Entry per PD per conversione ADC->W */
#define LUT_GAIN_WINDOWS        4U   /* Finestre guadagno MUX */
#define LUT_VCOMP_SIZE         10U   /* Entry LUT compensazione tensione PSU (mA->mV), NUOVO 2026-07-22 */
#define LUT_HWPWR_SIZE         11U   /* Entry LUT setpoint HW (RAW ADC->% potenza), NUOVO 2026-08-01 */

/* Mode index usato nelle LUT PD */
#define LUT_MODE_SW             0U   /* SW: setpoint = current_ma */
#define LUT_MODE_HW             1U   /* HW: setpoint = pa0_adc */

/* ============================================================================
 * TIPI
 * ============================================================================ */

/*
 * Entry LUT validazione PD.
 * Definisce per un dato setpoint il range ADC atteso per un fotodiodo.
 * AMC interpola linearmente tra entry consecutive.
 */
typedef struct {
    uint16_t setpoint;  /* current_ma (SW) o pa0_adc (HW) */
    uint16_t pd_min;    /* ADC minimo atteso [0-4095] */
    uint16_t pd_max;    /* ADC massimo atteso [0-4095] */
} LUT_PDValidEntry_t;

/*
 * Entry LUT conversione ADC -> Potenza.
 * Usata da MMC dopo aver ricevuto pd_raw[] + gain_window dal STATUS di AMC.
 * Indice: pd_power[pd_idx][gain_window][entry]
 * Lo stesso ADC_raw corrisponde a potenze diverse a seconda del guadagno attivo.
 * Interpolazione lineare tra entry consecutive.
 */
typedef struct {
    uint16_t adc;       /* Valore ADC [0-4095] */
    uint16_t power_w;   /* Potenza corrispondente [W] */
} LUT_PDPowerEntry_t;

/*
 * Entry LUT compensazione tensione PSU (NUOVO 2026-07-22).
 * Correla un setpoint di corrente LaseQ [mA] a una tensione target PSU
 * [mV] più bassa, usata da AMC (psu_voltage_comp.c) per ridurre la
 * dissipazione sui mosfet dei driver lineari durante l'emissione in
 * modalità SW non-QCW. AMC interpola linearmente tra entry consecutive
 * (stesso schema di LUT_PDValidEntry_t/AMC_MSG_CONFIG_PD_VALID) — le
 * entry DEVONO essere impostate in ordine crescente di current_ma.
 *
 * voltage_mv == 0 è un valore SENTINELLA: se un'entry non è stata
 * configurata (o il lookup per il setpoint corrente restituisce 0),
 * AMC NON scrive mai 0V sul PSU — usa invece il valore massimo
 * configurato con SET PSU VOLTAGE (psu_voltage_limit_mv), come se la
 * compensazione fosse disattivata per quel punto. Vedi banner in
 * AMC/App/Setpoint/psu_voltage_comp.h.
 */
typedef struct {
    uint16_t current_ma;  /* Setpoint di corrente LaseQ [mA] */
    uint16_t voltage_mv;  /* Tensione target PSU [mV], 0 = non configurata */
} LUT_VoltageCompEntry_t;

/*
 * Entry LUT conversione RAW ADC -> percentuale di potenza HW (NUOVO
 * 2026-08-01).
 *
 * Converte la lettura ADC2 grezza del pin PC5 (LPWR_SET_ISO, EXT_interface.c
 * — EXT_AcquireIN() la acquisisce ogni 5ms in task_inputs.c) nella
 * percentuale di potenza corrispondente [0-100%], per poter riportare il
 * setpoint HARDWARE (impostato dall'operatore sul connettore EXT analogico)
 * con lo stesso significato del setpoint software [0-100%] mostrato in
 * condizioni normali.
 *
 * Usata SOLO per REPORTING (comando RS485 "GET STATUS" e risposta STATUS
 * della COM interface verso ALITE_COM), esclusivamente quando
 * FSM_GetLaserModeWire() != 0 (FSM_MODE_ANALOG, oppure FSM_MODE_SW con
 * SETPOINT HW/"HYBRID2" attivo — vedi fsm.h): in tutti gli altri casi
 * continua a essere riportato il setpoint software esistente
 * (Setpoint_GetPct()/GetCurrentMa()), comportamento invariato. NON influisce
 * in alcun modo sul percorso di controllo reale verso LaseQ/AMC (quello
 * resta gestito da hw_setpoint_sel/setpoint_hw_enabled, invariato).
 *
 * Interpolazione lineare tra entry consecutive, stesso schema di
 * LUT_PDPowerEntry_t sopra. Le entry vanno impostate in ordine CRESCENTE di
 * adc_raw (ADC2 risoluzione 16 bit, range 0-65535 — vedi MX_ADC2_Init(),
 * main.c).
 */
typedef struct {
    uint16_t adc_raw;    /* Valore ADC2 grezzo [0-65535] */
    uint8_t  power_pct;  /* Potenza HW corrispondente [0-100%] */
} LUT_HwPowerEntry_t;

/*
 * Struttura completa salvata in flash.
 * Dimensione: circa 300 byte -> fits well within flash sector.
 */
#define LUT_MAGIC   0xABCD0003UL   /* bump 2026-08-01: aggiunta hw_power (RAW->% setpoint HW) */

typedef struct {
    uint32_t magic;

    /* Soglie finestre guadagno per AMC */
    uint16_t gain_threshold_sw[LUT_GAIN_WINDOWS];  /* SW: basate su current_ma */
    uint16_t gain_threshold_hw[LUT_GAIN_WINDOWS];  /* HW: basate su pa0_adc */

    /* LUT validazione PD (per modo e per PD) */
    uint8_t          pd_valid_size;                /* Entry usate (1..LUT_PD_VALID_SIZE) */
    LUT_PDValidEntry_t pd_valid[2][LUT_PD_MAX][LUT_PD_VALID_SIZE]; /* [modo][pd][entry] */

    /* LUT conversione ADC -> W (per PD) */
    uint8_t          pd_power_size;                /* Entry usate (1..LUT_PD_POWER_SIZE) */
    LUT_PDPowerEntry_t pd_power[LUT_PD_MAX][LUT_GAIN_WINDOWS][LUT_PD_POWER_SIZE]; /* [pd][win][entry] */

    /* LUT compensazione tensione PSU (NUOVO 2026-07-22) */
    uint8_t                voltage_comp_size;              /* Entry configurate (0..LUT_VCOMP_SIZE) */
    LUT_VoltageCompEntry_t voltage_comp[LUT_VCOMP_SIZE];    /* mA -> mV, ordine crescente di current_ma */

    /* LUT conversione RAW ADC (PC5/LPWR_SET_ISO) -> % potenza HW (NUOVO 2026-08-01) */
    uint8_t            hw_power_size;              /* Entry valide (0..LUT_HWPWR_SIZE) */
    LUT_HwPowerEntry_t hw_power[LUT_HWPWR_SIZE];    /* adc_raw -> power_pct, ordine crescente di adc_raw */

    uint32_t crc32;    /* CRC32 di tutti i campi precedenti */
} LUT_Store_t;

/* ============================================================================
 * API
 * ============================================================================ */

/**
 * @brief  Inizializza il modulo LUT.
 *         Tenta di caricare la LUT dalla flash; se non valida, carica i default.
 */
void LUT_Init(void);

/**
 * @brief  Restituisce un puntatore alla struttura LUT corrente (RAM).
 */
const LUT_Store_t *LUT_Get(void);

/**
 * @brief  Salva la LUT corrente in flash (cancella settore e riscrive).
 * @retval true se la scrittura ha avuto successo.
 */
bool LUT_Save(void);

/**
 * @brief  Riscrive la LUT corrente (RAM, s_lut) nel blocco flash riservato
 *         SENZA cancellare il settore — presuppone che il settore sia stato
 *         GIA' cancellato dal chiamante (vedi config.c, Config_Save(), ramo
 *         di overflow del journal: quel ramo cancella l'intero settore 7,
 *         che ospita anche il blocco LUT_Store_t, e senza questa chiamata
 *         la LUT salvata in precedenza andrebbe persa silenziosamente al
 *         prossimo LUT_Init() — bug osservato il 2026-07-27: "SAVE LUT"
 *         eseguito con successo, ma dopo molti "SAVE CONFIG" successivi
 *         [overflow del journal] la LUT risultava tornata a 0/10 entry).
 *         Stesso principio simmetrico di LUT_Save(), che dopo il PROPRIO
 *         erase richiama Config_Save() per ripristinare il journal.
 * @retval true se la scrittura ha avuto successo.
 */
bool LUT_RestoreAfterSectorErase(void);

/**
 * @brief  Ripristina i valori di default e salva in flash.
 */
void LUT_ResetDefaults(void);

/* --- Modifica puntuale --- */

/**
 * @brief  Aggiorna una soglia della LUT gain per un dato modo.
 * @param  mode    LUT_MODE_SW o LUT_MODE_HW
 * @param  index   0-3
 * @param  value   Nuova soglia [current_ma o pa0_adc]
 */
void LUT_SetGainThreshold(uint8_t mode, uint8_t index, uint16_t value);

/**
 * @brief  Aggiorna una entry della LUT validazione PD.
 * @param  mode       LUT_MODE_SW o LUT_MODE_HW
 * @param  pd_idx     0-3
 * @param  entry_idx  0..(LUT_PD_VALID_SIZE-1)
 * @param  setpoint   Valore setpoint [mA o raw]
 * @param  pd_min     ADC minimo atteso
 * @param  pd_max     ADC massimo atteso
 */
void LUT_SetPDValidEntry(uint8_t mode, uint8_t pd_idx, uint8_t entry_idx,
                         uint16_t setpoint, uint16_t pd_min, uint16_t pd_max);

/**
 * @brief  Aggiorna una entry della LUT ADC->W per un PD e una finestra guadagno.
 * @param  pd_idx      0-3
 * @param  gain_win    0-3 (finestra guadagno MUX)
 * @param  entry_idx   0..(LUT_PD_POWER_SIZE-1)
 * @param  adc         Valore ADC [0-4095]
 * @param  power_w     Potenza [W]
 */
void LUT_SetPDPowerEntry(uint8_t pd_idx, uint8_t gain_win, uint8_t entry_idx,
                         uint16_t adc, uint16_t power_w);

/**
 * @brief  Converte una lettura ADC di un fotodiodo in potenza [W].
 *         Usa la LUT specifica per il gain_window attivo al momento della misura.
 *         Interpolazione lineare tra le entry della LUT.
 * @param  pd_idx     0-3
 * @param  gain_win   0-3 (campo gain_window dal STATUS AMC)
 * @param  adc_raw    Valore ADC grezzo [0-4095]
 * @retval Potenza stimata [W]; 0 se LUT vuota.
 */
uint16_t LUT_ConvertPDToWatt(uint8_t pd_idx, uint8_t gain_win, uint16_t adc_raw);

/**
 * @brief  Aggiorna una entry della LUT compensazione tensione PSU (mA->mV).
 *         Aggiorna anche voltage_comp_size se entry_idx+1 supera il valore
 *         corrente (crescita incrementale, stesso comportamento "naturale"
 *         di un operatore che imposta le entry in ordine 0,1,2...).
 *         RAM-only: serve "SAVE LUT" per persistere in flash, e va poi
 *         ritrasmessa ad AMC (vedi TaskAmc_RequestPSUConfigResend() /
 *         AMC_SendVoltageCompLUTEntry()) — lo stesso comando RS485 "SET LUT
 *         VCOMP" marca già il resend pending.
 *
 * @param  entry_idx   Indice entry [0 .. LUT_VCOMP_SIZE-1]
 * @param  current_ma  Setpoint di corrente LaseQ [mA]
 * @param  voltage_mv  Tensione target PSU [mV] (0 = entry non valida, vedi
 *                     banner su LUT_VoltageCompEntry_t)
 */
void LUT_SetVoltageCompEntry(uint8_t entry_idx, uint16_t current_ma, uint16_t voltage_mv);

/**
 * @brief  Aggiorna una entry della LUT setpoint HW (RAW ADC -> % potenza).
 *         RAM-only: serve "SAVE LUT" per persistere in flash (stesso schema
 *         di GAIN/VALID/POWER — nessun invio ad AMC, uso esclusivamente
 *         locale per reporting).
 *
 * @param  entry_idx   Indice entry [0 .. LUT_HWPWR_SIZE-1]
 * @param  adc_raw     Valore ADC2 grezzo [0-65535]
 * @param  power_pct   Percentuale di potenza corrispondente [0-100], clampata a 100.
 */
void LUT_SetHwPowerEntry(uint8_t entry_idx, uint16_t adc_raw, uint8_t power_pct);

/**
 * @brief  Converte una lettura ADC2 grezza del setpoint HW (PC5/LPWR_SET_ISO)
 *         in percentuale di potenza [0-100%]. Interpolazione lineare tra le
 *         entry della LUT, clampata agli estremi.
 *
 * @param  adc_raw  Valore ADC2 grezzo [0-65535] (vedi EXT_GetPowerSetpointRaw(),
 *                  EXT_interface.h).
 * @retval Percentuale stimata [0-100]; 0 se LUT vuota (hw_power_size == 0).
 */
uint8_t LUT_ConvertHwPowerToPct(uint16_t adc_raw);

#endif /* APP_LUT_LUT_MANAGER_H_ */
