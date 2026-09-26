/*
 * SHT35.c
 *
 * Implementazione del driver SHT35.
 * Vedere SHT35.h per la documentazione dell'API.
 *
 * CHANGELOG:
 * - SHT35_Init(): rilascia nTH_SENS_RESET (attivo basso) prima di qualunque
 *   accesso I2C. MX_GPIO_Init() (main.c) porta il pin a GPIO_PIN_RESET
 *   all'avvio (reset asserito, sensore disabilitato) e non lo rilascia mai
 *   — causa confermata del sintomo "SHT35 non risponde mai" (SHT35_Read()
 *   fallisce ad ogni ciclo, TAMB/HUM restano ai valori sentinella
 *   SHT35_INVALID_TEMP/HUM). Stesso pin, stesso bug, stessa causa già
 *   individuata e corretta sul firmware LaseQ (Drivers/SHT-35/SHT-35.c,
 *   stesso sensore, stesso schema di reset) — fix qui portato in linea con
 *   quello.
 */

#include "SHT35.h"
#include "main.h"   /* nTH_SENS_RESET_Pin, nTH_SENS_RESET_GPIO_Port */
#include <string.h>

/* ========================================================================== */
/* --- COMANDI SHT35 --- */
/* ========================================================================== */

/*
 * Single shot - high repeatability, clock stretching disabled
 * Datasheet SHT3x: command 0x2400
 */
#define SHT35_CMD_MEAS_HIGHREP   0x2400U

/* ========================================================================== */
/* --- STATO INTERNO --- */
/* ========================================================================== */

static I2C_HandleTypeDef *s_hi2c   = NULL;
static SHT35_Data_t       s_data   = { SHT35_INVALID_TEMP, SHT35_INVALID_HUM, false };

/*
 * Sincronizzazione IT delegata al layer applicativo (nessuna dipendenza
 * FreeRTOS/CMSIS-RTOS in questo driver, vedi nota in SHT35.h). Trigger e Read
 * non sono mai concorrenti: entrambi girano nel task_monitor.
 * SHT35_ITTxCallback()/SHT35_ITRxCallback() (successo) e
 * SHT35_ITErrorCallback() (errore) chiamano s_signal_fn dal contesto ISR di
 * I2C2_EV/ER_IRQHandler; la funzione chiamante chiama s_wait_fn invece di
 * fare polling bloccante come con HAL_I2C_Master_*().
 */
static SHT35_WaitFn_t   s_wait_fn      = NULL;
static SHT35_SignalFn_t s_signal_fn    = NULL;
static volatile bool    s_i2c_it_error = false;

/* ========================================================================== */
/* --- FUNZIONI PRIVATE --- */
/* ========================================================================== */

/*
 * CRC-8 con polinomio 0x31, valore iniziale 0xFF.
 * Usato da Sensirion per verificare l'integrità dei dati.
 */
static uint8_t crc8(const uint8_t *data, uint8_t len)
{
    uint8_t crc = 0xFF;
    for (uint8_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (uint8_t b = 0; b < 8; b++) {
            if (crc & 0x80) {
                crc = (uint8_t)((crc << 1) ^ 0x31);
            } else {
                crc <<= 1;
            }
        }
    }
    return crc;
}

/* ========================================================================== */
/* --- API PUBBLICA --- */
/* ========================================================================== */

void SHT35_Init(I2C_HandleTypeDef *hi2c, SHT35_WaitFn_t wait_fn, SHT35_SignalFn_t signal_fn)
{
    /*
     * Rilascio hardware reset (nTH_SENS_RESET, attivo basso) — vedi
     * CHANGELOG in testa al file. MX_GPIO_Init() lo porta a GPIO_PIN_RESET
     * all'avvio (reset asserito, chip disabilitato) e non lo rilascia mai
     * autonomamente: senza questo il sensore non risponde MAI su I2C
     * (nessun ACK), ogni SHT35_Trigger()/SHT35_Read() fallisce in timeout o
     * errore, s_data.valid resta false e TaskMonitor_/SHT35_GetData()
     * restituisce sempre i valori sentinella (TAMB=-327.68, HUM=-1%).
     *
     * Va fatto qui, al primo utilizzo del driver, PRIMA di qualunque
     * comando I2C verso il sensore — SHT35_Init() è chiamata una sola
     * volta in TaskMonitor_Run(), dopo osKernelStart() (vedi task_monitor.c),
     * quindi HAL_Delay(2) bloccante è accettabile: copre il tempo di
     * power-up del sensore dopo il rilascio del reset (datasheet Sensirion
     * SHT3x: max 1.5ms, margine doppio per sicurezza).
     */
    HAL_GPIO_WritePin(nTH_SENS_RESET_GPIO_Port, nTH_SENS_RESET_Pin, GPIO_PIN_SET);
    HAL_Delay(2U);

    s_hi2c      = hi2c;
    s_wait_fn   = wait_fn;
    s_signal_fn = signal_fn;

    memset(&s_data, 0, sizeof(s_data));
    s_data.temperature_cdeg = SHT35_INVALID_TEMP;
    s_data.humidity_cpct    = SHT35_INVALID_HUM;
    s_data.valid            = false;
}

bool SHT35_Trigger(void)
{
    if (s_hi2c == NULL) return false;

    uint8_t cmd[2] = {
        (uint8_t)(SHT35_CMD_MEAS_HIGHREP >> 8),
        (uint8_t)(SHT35_CMD_MEAS_HIGHREP & 0xFF)
    };

    /* Drain difensivo: vedi commento identico in AD7490.c/spi_transact()
     * e AMC.c/amc_transceive(). Un token residuo su s_sht35_sem da un
     * timeout precedente (Trigger o Read) andrebbe altrimenti a
     * disallineare silenziosamente l'attesa della prossima transazione. */
    if (s_wait_fn != NULL) { (void)s_wait_fn(0U); }

    s_i2c_it_error = false;
    HAL_StatusTypeDef ret = HAL_I2C_Master_Transmit_IT(
        s_hi2c,
        (uint16_t)(SHT35_I2C_ADDR << 1),
        cmd, 2
    );

    if (ret != HAL_OK) {
        return false;
    }

    /* Attende il completamento IT cedendo la CPU allo scheduler (a differenza
     * della vecchia HAL_I2C_Master_Transmit() bloccante). L'attesa vera e
     * propria è implementata dal layer applicativo e iniettata tramite
     * s_wait_fn (vedi SHT35_Init). */
    if (s_wait_fn == NULL || !s_wait_fn(SHT35_I2C_TIMEOUT)) {
        HAL_I2C_Master_Abort_IT(s_hi2c, (uint16_t)(SHT35_I2C_ADDR << 1));
        return false;
    }

    return !s_i2c_it_error;
}

bool SHT35_Read(void)
{
    if (s_hi2c == NULL) return false;

    /*
     * Risposta SHT35 dopo misura single-shot: 6 byte
     *   [0][1] = temperatura raw (MSB first)
     *   [2]    = CRC temperatura
     *   [3][4] = umidità raw (MSB first)
     *   [5]    = CRC umidità
     */
    uint8_t buf[6] = {0};

    /* Drain difensivo (vedi commento in SHT35_Trigger()). */
    if (s_wait_fn != NULL) { (void)s_wait_fn(0U); }

    s_i2c_it_error = false;
    HAL_StatusTypeDef ret = HAL_I2C_Master_Receive_IT(
        s_hi2c,
        (uint16_t)(SHT35_I2C_ADDR << 1),
        buf, 6
    );

    if (ret == HAL_OK) {
        /* Attende il completamento IT cedendo la CPU allo scheduler (a
         * differenza della vecchia HAL_I2C_Master_Receive() bloccante). */
        if (s_wait_fn == NULL || !s_wait_fn(SHT35_I2C_TIMEOUT)) {
            HAL_I2C_Master_Abort_IT(s_hi2c, (uint16_t)(SHT35_I2C_ADDR << 1));
            ret = HAL_TIMEOUT;
        } else if (s_i2c_it_error) {
            ret = HAL_ERROR;
        }
    }

    if (ret != HAL_OK) {
        s_data.valid = false;
        return false;
    }

    /* Verifica CRC temperatura */
    if (crc8(&buf[0], 2) != buf[2]) {
        s_data.valid = false;
        return false;
    }

    /* Verifica CRC umidità */
    if (crc8(&buf[3], 2) != buf[5]) {
        s_data.valid = false;
        return false;
    }

    /* Conversione temperatura:
     *   T [°C] = -45 + 175 * raw / 65535
     *   T [centideg] = (-4500 + 17500 * raw / 65535)
     *   Usiamo aritmetica intera: 17500 * raw può overflow su 32 bit se raw=65535
     *   → cast a 64 bit per il prodotto
     */
    uint16_t raw_t = (uint16_t)((buf[0] << 8) | buf[1]);
    uint16_t raw_h = (uint16_t)((buf[3] << 8) | buf[4]);

    s_data.temperature_cdeg = (int16_t)(-4500 + (int32_t)(17500LL * raw_t / 65535LL));
    s_data.humidity_cpct    = (int16_t)(int32_t)(10000LL * raw_h / 65535LL);
    s_data.valid            = true;

    return true;
}

SHT35_Data_t SHT35_GetData(void)
{
    return s_data;
}

int8_t SHT35_GetHumidityPct(void)
{
    if (!s_data.valid) return -1;
    return (int8_t)(s_data.humidity_cpct / 100);
}

/* ========================================================================== */
/* --- CALLBACK IT --- */
/* ========================================================================== */

void SHT35_ITTxCallback(void)
{
    /* Contesto ISR (I2C2_EV_IRQHandler → HAL_I2C_EV_IRQHandler → qui via il
     * dispatcher in stm32h7xx_it.c). s_signal_fn (fornita dal layer
     * applicativo, tipicamente osSemaphoreRelease) gestisce il contesto IRQ. */
    if (s_signal_fn != NULL) {
        s_signal_fn();
    }
}

void SHT35_ITRxCallback(void)
{
    if (s_signal_fn != NULL) {
        s_signal_fn();
    }
}

void SHT35_ITErrorCallback(void)
{
    s_i2c_it_error = true;
    if (s_signal_fn != NULL) {
        s_signal_fn();
    }
}
