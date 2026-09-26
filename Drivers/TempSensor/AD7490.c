/*
 * AD7490.c
 *
 * Implementazione del driver AD7490WBCSZ.
 * Vedere AD7490.h per documentazione protocollo e API.
 */

#include "AD7490.h"
#include "main.h"   /* TEMP_ADC_nCS_GPIO_Port, TEMP_ADC_nCS_Pin */
#include <stdbool.h>

/* ========================================================================== */
/* --- COSTANTI PROTOCOLLO --- */
/* ========================================================================== */

/*
 * MAPPA BIT REALE DEL CONTROL REGISTER AD7490 (datasheet Analog Devices
 * Rev. E, Table 9 — verificata il 2026-07-06, vedi anche AD7490.h):
 *
 *   bit15  WRITE     bit14  SEQ       bit13..10  ADD3..ADD0
 *   bit9   PM1        bit8   PM0       bit7   SHADOW
 *   bit6   WEAK/TRI   bit5   RANGE     bit4   CODING
 *   bit3..0  don't care
 *
 * ATTENZIONE (2026-07-06): la versione precedente di queste macro
 * assumeva ADD[14:11]/PM[10:9]/SHADOW[8]/RANGE[7]/CODING[6] — cioè
 * ignorava completamente il bit SEQ (bit14), sfasando di una posizione
 * tutto il registro dopo WRITE. Effetto pratico (verificato contro il
 * datasheet): il vecchio "PM_NORMAL = 0x3<<9" scriveva PM1=1 (bit9,
 * corretto) ma il vero PM0 (bit8) non veniva MAI scritto (restava 0) →
 * PM1=1/PM0=0 = FULL SHUTDOWN a ogni transazione, invece di Normal
 * Operation. Inoltre il vecchio bit RANGE (7) cadeva sul vero SHADOW e
 * il vecchio bit CODING (6) sul vero WEAK/TRI, forzando entrambi a 1 in
 * modo non intenzionale, e il campo canale invadeva il bit SEQ per i
 * canali 8-15. Root cause del sintomo "DOUT sempre a 1, non risponde mai".
 *
 * PM[1:0] = 11  → Normal operation (no power-down).
 * RANGE   = 1   → Input range 0..Vref (unipolare).
 * CODING  = 1   → Straight binary output.
 * SEQ = SHADOW = WEAK/TRI = 0 → nessun sequencer, DOUT torna a
 *   three-state a fine transfer (comportamento "semplice" richiesto
 *   dal driver: ogni scrittura seleziona il canale successivo).
 */
#define AD7490_WRITE_BIT        (1U << 15)
#define AD7490_PM_NORMAL        (0x3U << 8)
#define AD7490_RANGE_UNIPOLAR   (1U << 5)
#define AD7490_CODING_BINARY    (1U << 4)

#define AD7490_CTRL_BASE        (AD7490_WRITE_BIT | AD7490_PM_NORMAL | \
                                 AD7490_RANGE_UNIPOLAR | AD7490_CODING_BINARY)

/*
 * Timeout in ms per l'attesa del completamento IT (wait_fn applicativa).
 * Rispetto alla vecchia versione bloccante (10ms, tempo di trasferimento
 * HW puro) qui includiamo anche la latenza di scheduling del task che
 * chiama spi_transact(): 20ms resta ampiamente sufficiente per 2 byte a
 * >8MHz e assorbe eventuali ritardi di context-switch senza mascherare
 * un guasto reale (mancata risposta della periferica/ISR).
 */
#define AD7490_SPI_TIMEOUT_MS   20U

/* Maschera risultato (bit 11..0) */
#define AD7490_RESULT_MASK      0x0FFFU

/* ========================================================================== */
/* --- STATO INTERNO --- */
/* ========================================================================== */

static SPI_HandleTypeDef *s_hspi = NULL;

/*
 * Sincronizzazione IT delegata al layer applicativo (nessuna dipendenza
 * FreeRTOS/CMSIS-RTOS in questo driver, vedi nota in AD7490.h).
 * AD7490_ITCallback() (successo) e AD7490_ITErrorCallback() (errore) chiamano
 * s_signal_fn dal contesto ISR di SPI5_IRQHandler; spi_transact() chiama
 * s_wait_fn (tipicamente osSemaphoreAcquire lato App) invece di fare polling
 * bloccante su HAL_SPI_TransmitReceive() — la CPU è ceduta allo scheduler
 * per tutta la durata del trasferimento.
 */
static AD7490_WaitFn_t   s_wait_fn      = NULL;
static AD7490_SignalFn_t s_signal_fn    = NULL;
static volatile bool     s_spi_it_error = false;

/* ========================================================================== */
/* --- FUNZIONI PRIVATE --- */
/* ========================================================================== */

static inline void cs_assert(void)
{
    HAL_GPIO_WritePin(TEMP_ADC_nCS_GPIO_Port, TEMP_ADC_nCS_Pin, GPIO_PIN_RESET);
}

static inline void cs_deassert(void)
{
    HAL_GPIO_WritePin(TEMP_ADC_nCS_GPIO_Port, TEMP_ADC_nCS_Pin, GPIO_PIN_SET);
}

/*
 * @brief  Esegue una singola transazione SPI da 16 bit (full-duplex).
 *         Gestisce CS manualmente (CS alto tra le transazioni, come richiesto
 *         dall'AD7490 per la sincronizzazione del framing).
 *
 * @param  tx_word   Parola di controllo da trasmettere (16 bit).
 * @param  rx_word   Puntatore dove scrivere la risposta (16 bit). Può essere NULL.
 * @return AD7490_OK o codice di errore.
 */
static AD7490_err_t spi_transact(uint16_t tx_word, uint16_t *rx_word)
{
    /*
     * HAL SPI opera su byte: spezziamo la parola in 2 byte MSB first.
     * L'AD7490 richiede MSB first e CPOL=0, CPHA=1 (SPI Mode 1).
     * La configurazione CubeMX deve rispecchiare questo.
     *
     * tx/rx sono variabili locali (stack): restano valide per tutta la
     * durata della transazione IT perché la funzione non ritorna finché
     * non ha atteso il completamento (s_wait_fn sotto), quindi non serve
     * renderle static.
     */
    uint8_t tx[2] = { (uint8_t)(tx_word >> 8), (uint8_t)(tx_word & 0xFF) };
    uint8_t rx[2] = { 0, 0 };

    /*
     * Drain difensivo: se la transazione precedente è finita in timeout
     * (sotto, HAL_SPI_Abort()), non è garantito che l'IT di completamento
     * non arrivi comunque poco dopo l'abort — lascerebbe un token residuo
     * su s_ad7490_sem. Senza questo drain, la PROSSIMA s_wait_fn() lo
     * consumerebbe subito senza attendere il vero completamento della
     * nuova transazione, disallineando silenziosamente tutte le
     * transazioni successive (stesso bug già trovato e corretto in
     * AMC.c/amc_transceive() sul lato UART verso AMC).
     */
    if (s_wait_fn != NULL) { (void)s_wait_fn(0U); }

    cs_assert();

    s_spi_it_error = false;
    HAL_StatusTypeDef status = HAL_SPI_TransmitReceive_IT(s_hspi, tx, rx, 2);

    if (status != HAL_OK) {
        cs_deassert();
        return (status == HAL_TIMEOUT) ? AD7490_ERR_TIMEOUT : AD7490_ERR_SPI;
    }

    /*
     * Attende il completamento IT cedendo la CPU allo scheduler (a differenza
     * della vecchia HAL_SPI_TransmitReceive() bloccante, che monopolizzava
     * il task chiamante per l'intera durata del trasferimento). L'attesa
     * vera e propria (tipicamente osSemaphoreAcquire) è implementata dal
     * layer applicativo e iniettata qui tramite s_wait_fn.
     */
    bool completed = (s_wait_fn != NULL) && s_wait_fn(AD7490_SPI_TIMEOUT_MS);
    cs_deassert();

    if (!completed) {
        /* Nessuna callback ricevuta in tempo: abort per resettare lo stato
         * interno della periferica prima del prossimo tentativo. */
        HAL_SPI_Abort(s_hspi);
        return AD7490_ERR_TIMEOUT;
    }

    if (s_spi_it_error) {
        return AD7490_ERR_SPI;
    }

    if (rx_word != NULL) {
        *rx_word = ((uint16_t)rx[0] << 8) | rx[1];
    }
    return AD7490_OK;
}

/*
 * @brief  Costruisce la control word per il canale specificato.
 *         ADD[3:0] = bit[13:10] (vedi mappa bit reale sopra).
 */
static inline uint16_t build_ctrl(uint8_t channel)
{
    return (uint16_t)(AD7490_CTRL_BASE | ((uint16_t)(channel & 0x0FU) << 10));
}

/* ========================================================================== */
/* --- API PUBBLICA --- */
/* ========================================================================== */

void AD7490_Init(SPI_HandleTypeDef *hspi, AD7490_WaitFn_t wait_fn, AD7490_SignalFn_t signal_fn)
{
    s_hspi      = hspi;
    s_wait_fn   = wait_fn;
    s_signal_fn = signal_fn;

    /* CS inattivo (HIGH) */
    cs_deassert();

    /*
     * Prima transazione di wake-up: il chip potrebbe essere in power-down
     * dopo reset. Inviamo una control word per CH0 in normal operation.
     * Il risultato viene scartato (garbage prima della prima conversione).
     */
    spi_transact(build_ctrl(0), NULL);
}

AD7490_err_t AD7490_ScanAll(uint16_t out[AD7490_NUM_CHANNELS])
{
    if (s_hspi == NULL) return AD7490_ERR_INIT;

    AD7490_err_t err;
    uint16_t rx_word;

    /*
     * Sequenza pipeline per 16 canali:
     *
     *   Transazione 0: CTRL(CH0) → scarta (il chip non ha ancora dati)
     *   Transazione 1: CTRL(CH1) → risultato CH0
     *   ...
     *   Transazione 15: CTRL(CH15) → risultato CH14
     *   Transazione 16: dummy(0)  → risultato CH15
     *
     * Totale: 17 transazioni SPI.
     */

    /* Prima transazione: avvia CH0, scarta risposta */
    err = spi_transact(build_ctrl(0), NULL);
    if (err != AD7490_OK) return err;

    /* Transazioni 1..15: avvia CH[i], legge risultato CH[i-1] */
    for (uint8_t ch = 1; ch < AD7490_NUM_CHANNELS; ch++) {
        err = spi_transact(build_ctrl(ch), &rx_word);
        if (err != AD7490_OK) return err;
        out[ch - 1] = rx_word & AD7490_RESULT_MASK;
    }

    /* Ultima transazione: dummy, legge risultato CH15 */
    err = spi_transact(0x0000U, &rx_word);
    if (err != AD7490_OK) return err;
    out[AD7490_NUM_CHANNELS - 1] = rx_word & AD7490_RESULT_MASK;

    return AD7490_OK;
}

AD7490_err_t AD7490_ReadChannel(uint8_t channel, uint16_t *out)
{
    if (s_hspi == NULL) return AD7490_ERR_INIT;
    if (channel >= AD7490_NUM_CHANNELS || out == NULL) return AD7490_ERR_SPI;

    AD7490_err_t err;
    uint16_t rx_word;

    /* Transazione 1: avvia conversione del canale target */
    err = spi_transact(build_ctrl(channel), NULL);
    if (err != AD7490_OK) return err;

    /* Transazione 2: legge il risultato (invia dummy) */
    err = spi_transact(0x0000U, &rx_word);
    if (err != AD7490_OK) return err;

    *out = rx_word & AD7490_RESULT_MASK;
    return AD7490_OK;
}

/* ========================================================================== */
/* --- CALLBACK IT --- */
/* ========================================================================== */

void AD7490_ITCallback(void)
{
    /* Contesto ISR (SPI5_IRQHandler → HAL_SPI_IRQHandler → qui via il
     * dispatcher in stm32h7xx_it.c). s_signal_fn (fornita dal layer
     * applicativo, tipicamente osSemaphoreRelease) gestisce il contesto IRQ. */
    if (s_signal_fn != NULL) {
        s_signal_fn();
    }
}

void AD7490_ITErrorCallback(void)
{
    s_spi_it_error = true;
    if (s_signal_fn != NULL) {
        s_signal_fn();
    }
}
