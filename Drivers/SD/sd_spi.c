/*
 * sd_spi.c
 *
 * Driver SPI per scheda microSD (protocollo SPI Mode 0).
 * Vedere sd_spi.h per documentazione API.
 *
 * PROTOCOLLO SD IN SPI MODE:
 *   1. Alimentazione + 74+ clock con CS HIGH (dummy clocks).
 *   2. CMD0  (GO_IDLE_STATE)  → R1 = 0x01 (idle).
 *   3. CMD8  (SEND_IF_COND)   → verifica tensione + pattern (SD v2 only).
 *   4. Loop: CMD55 + ACMD41   → init completata quando R1 = 0x00.
 *   5. CMD58 (READ_OCR)       → rileva SDHC/SDXC (OCR bit 30).
 *   6. CMD16 (SET_BLOCKLEN)   → forza 512B per SD v1/v2 non-HC.
 *   7. Aumento velocità SPI per trasferimenti dati.
 *
 * INDIRIZZI BLOCCO:
 *   SD v1/v2: indirizzo in byte → block_addr × 512.
 *   SDHC/SDXC: indirizzo diretto in blocchi.
 */

#include "sd_spi.h"
#include "main.h"    /* Pin defines: nSD_CS, nSD_PRESENT */

/* ============================================================================
 * TIMING E COSTANTI PROTOCOLLO
 * ============================================================================ */

#define SD_INIT_TIMEOUT_MS    2000U   /* Timeout ACMD41 durante init */
#define SD_READ_TIMEOUT_MS     500U   /* Timeout attesa token lettura */
#define SD_WRITE_TIMEOUT_MS    500U   /* Timeout attesa fine scrittura */

#define SD_TOKEN_START_BLOCK   0xFEU  /* Data token per CMD17/CMD18/CMD24 */
#define SD_TOKEN_STOP_TRAN     0xFDU  /* Stop token per CMD25 */

/* Maschere risposta data response (write) */
#define SD_DATA_RESP_MASK      0x1FU
#define SD_DATA_RESP_ACCEPTED  0x05U  /* Dati accettati */

/* Dummy byte per mantenere clock attivo durante attesa */
#define SD_DUMMY_BYTE          0xFFU

/*
 * Prescaler SPI per velocità init (≤400kHz) e dati (~8MHz).
 * STM32H7: SPI3 clock = PCLK1 (tipicamente 100–200MHz per H723).
 * Con PCLK1=100MHz: prescaler 256 → ~390kHz, prescaler 16 → ~6.25MHz.
 * Con PCLK1=200MHz: prescaler 512 non disponibile → usare 256 per init.
 * Adattare se necessario.
 */
#define SD_PRESCALER_SLOW   SPI_BAUDRATEPRESCALER_256   /* ≤400kHz init */
#define SD_PRESCALER_FAST   SPI_BAUDRATEPRESCALER_16    /* ~6-12MHz data */

/* ============================================================================
 * STATO INTERNO
 * ============================================================================ */

static SPI_HandleTypeDef *s_hspi      = NULL;
static osMutexId_t        s_spi_mutex = NULL;
static SD_CardType_t      s_card_type = SD_TYPE_NONE;
static bool               s_ready     = false;

/* ============================================================================
 * HELPER: CS e SPI
 * ============================================================================ */

static inline void cs_low(void)
{
    HAL_GPIO_WritePin(nSD_CS_GPIO_Port, nSD_CS_Pin, GPIO_PIN_RESET);
}

static inline void cs_high(void)
{
    HAL_GPIO_WritePin(nSD_CS_GPIO_Port, nSD_CS_Pin, GPIO_PIN_SET);
}

static inline void spi_lock(void)
{
    if (s_spi_mutex != NULL) { osMutexAcquire(s_spi_mutex, osWaitForever); }
}

static inline void spi_unlock(void)
{
    if (s_spi_mutex != NULL) { osMutexRelease(s_spi_mutex); }
}

/*
 * Riconfigura SPI3 (data size, CPOL/CPHA, NSS, prescaler) per l'uso da parte
 * della SD. Necessario perché SPI3 è condivisa con la COM interface
 * (Drivers/COM_interface/COM_interface.c), che la riconfigura a sua volta
 * (prescaler e NSSPMode diversi) prima di ogni propria transazione: senza
 * questa riconfigurazione esplicita ad ogni utilizzo, il driver che usa il
 * bus per secondo troverebbe l'impostazione lasciata dall'altro.
 * (Corretto 2026-07-17: il vecchio commento su un DataSize a 4-bit per la
 * COM interface era un refuso — SPI3 è sempre 8-bit, per entrambi i driver.)
 */
static void spi_reconfigure(uint32_t prescaler)
{
    /* Disabilita SPI prima di modificare i registri di configurazione */
    __HAL_SPI_DISABLE(s_hspi);

    s_hspi->Init.DataSize         = SPI_DATASIZE_8BIT;
    s_hspi->Init.BaudRatePrescaler = prescaler;
    s_hspi->Init.CLKPolarity      = SPI_POLARITY_LOW;
    s_hspi->Init.CLKPhase         = SPI_PHASE_1EDGE;
    s_hspi->Init.NSS              = SPI_NSS_SOFT;
    s_hspi->Init.NSSPMode         = SPI_NSS_PULSE_DISABLE;
    s_hspi->Init.FifoThreshold    = SPI_FIFO_THRESHOLD_01DATA;

    HAL_SPI_Init(s_hspi);
}

/*
 * Invia un singolo byte e riceve un byte in contemporanea (full-duplex).
 */
static uint8_t spi_xfer(uint8_t tx)
{
    uint8_t rx = 0;
    HAL_SPI_TransmitReceive(s_hspi, &tx, &rx, 1, 10);
    return rx;
}

/*
 * Invia N byte dummy (0xFF) per generare clock (CS può essere HIGH o LOW).
 */
static void spi_send_dummy(uint8_t n)
{
    while (n--) { spi_xfer(SD_DUMMY_BYTE); }
}

/* ============================================================================
 * HELPER: COMANDI SD
 * ============================================================================ */

/*
 * Invia un comando SD e restituisce R1.
 * arg: 32-bit argument; crc: CRC7 + stop bit (precompilato per CMD0 e CMD8).
 */
static uint8_t sd_send_cmd(uint8_t cmd, uint32_t arg, uint8_t crc)
{
    uint8_t buf[6] = {
        (uint8_t)(0x40U | cmd),
        (uint8_t)(arg >> 24),
        (uint8_t)(arg >> 16),
        (uint8_t)(arg >> 8),
        (uint8_t)(arg),
        crc,
    };

    /* Lascia un byte di gap prima del comando */
    spi_xfer(SD_DUMMY_BYTE);

    HAL_SPI_Transmit(s_hspi, buf, 6, 10);

    /* Attendi risposta R1 (max 8 byte con MISO=0xFF = busy) */
    uint8_t r1 = SD_DUMMY_BYTE;
    for (uint8_t i = 0; i < 8U; i++) {
        r1 = spi_xfer(SD_DUMMY_BYTE);
        if ((r1 & 0x80U) == 0U) break;   /* bit 7 = 0 → risposta valida */
    }
    return r1;
}

/*
 * Attende che la scheda non sia più occupata (MISO torna HIGH).
 * Restituisce true se la scheda è diventata libera entro timeout_ms.
 */
static bool sd_wait_not_busy(uint32_t timeout_ms)
{
    uint32_t t0 = HAL_GetTick();
    while ((HAL_GetTick() - t0) < timeout_ms) {
        if (spi_xfer(SD_DUMMY_BYTE) == SD_DUMMY_BYTE) return true;
    }
    return false;
}

/*
 * Attende il data token (0xFE) per la lettura.
 * Restituisce true se il token è arrivato, false su timeout o errore.
 */
static bool sd_wait_data_token(uint32_t timeout_ms)
{
    uint32_t t0 = HAL_GetTick();
    while ((HAL_GetTick() - t0) < timeout_ms) {
        uint8_t b = spi_xfer(SD_DUMMY_BYTE);
        if (b == SD_TOKEN_START_BLOCK) return true;
        if (b != SD_DUMMY_BYTE) return false;   /* error token */
    }
    return false;
}

/* ============================================================================
 * INIT
 * ============================================================================ */

SD_Result_t SD_Init(SPI_HandleTypeDef *hspi, osMutexId_t spi_mutex)
{
    if (hspi == NULL) return SD_ERR_PARAM;

    s_hspi      = hspi;
    s_spi_mutex = spi_mutex;
    s_card_type = SD_TYPE_NONE;
    s_ready     = false;

    /* Verifica presenza scheda */
    if (!SD_IsPresent()) return SD_ERR_NO_CARD;

    spi_lock();

    /* ---- Fase 1: Riconfigura SPI3 a 8-bit, velocità lenta (≤400kHz) ---- */
    spi_reconfigure(SD_PRESCALER_SLOW);

    /* ---- Fase 2: Power-up — CS HIGH, 80+ dummy clock (10 byte) ---- */
    cs_high();
    spi_send_dummy(10);

    /* ---- Fase 3: CMD0 — Reset scheda → SPI mode ---- */
    cs_low();
    uint8_t r1 = sd_send_cmd(0, 0x00000000UL, 0x95U);
    cs_high();
    spi_send_dummy(1);

    if (r1 != 0x01U) {
        spi_unlock();
        return SD_ERR_INIT_FAIL;
    }

    /* ---- Fase 4: CMD8 — Rileva SD v2 ---- */
    /* Argument: VHS=0x01 (2.7-3.6V), Check Pattern=0xAA */
    cs_low();
    r1 = sd_send_cmd(8, 0x000001AAUL, 0x87U);

    uint8_t  sd_version;
    uint32_t ocr = 0;

    if (r1 == 0x01U) {
        /* SD v2: leggi R7 (4 byte aggiuntivi) */
        uint8_t r7[4];
        for (uint8_t i = 0; i < 4U; i++) { r7[i] = spi_xfer(SD_DUMMY_BYTE); }
        cs_high();
        spi_send_dummy(1);

        if ((r7[2] == 0x01U) && (r7[3] == 0xAAU)) {
            sd_version = 2;
        } else {
            spi_unlock();
            return SD_ERR_INIT_FAIL;  /* Tensione non supportata */
        }
    } else if ((r1 & 0x04U) != 0U) {
        /* CMD8 non supportato → SD v1 */
        cs_high();
        spi_send_dummy(1);
        sd_version = 1;
    } else {
        cs_high();
        spi_send_dummy(1);
        spi_unlock();
        return SD_ERR_INIT_FAIL;
    }

    /* ---- Fase 5: ACMD41 — Avvio inizializzazione ---- */
    /* HCS=1 per SD v2 (supporta SDHC) */
    uint32_t acmd41_arg = (sd_version == 2) ? 0x40000000UL : 0x00000000UL;
    uint32_t t0 = HAL_GetTick();
    do {
        /* CMD55 (APP_CMD) + CMD41 (SEND_OP_COND) */
        cs_low();
        sd_send_cmd(55, 0x00000000UL, 0x65U);
        cs_high();
        spi_send_dummy(1);

        cs_low();
        r1 = sd_send_cmd(41, acmd41_arg, 0x77U);
        cs_high();
        spi_send_dummy(1);

        if ((HAL_GetTick() - t0) >= SD_INIT_TIMEOUT_MS) {
            spi_unlock();
            return SD_ERR_TIMEOUT;
        }

        HAL_Delay(1);
    } while (r1 == 0x01U);   /* 0x00 = init completata */

    if (r1 != 0x00U) {
        spi_unlock();
        return SD_ERR_INIT_FAIL;
    }

    /* ---- Fase 6: CMD58 — Leggi OCR per determinare SDHC ---- */
    if (sd_version == 2) {
        cs_low();
        r1 = sd_send_cmd(58, 0x00000000UL, 0xFDU);
        if (r1 == 0x00U) {
            uint8_t ocr_buf[4];
            for (uint8_t i = 0; i < 4U; i++) { ocr_buf[i] = spi_xfer(SD_DUMMY_BYTE); }
            ocr = ((uint32_t)ocr_buf[0] << 24) | ((uint32_t)ocr_buf[1] << 16)
                | ((uint32_t)ocr_buf[2] << 8)  | ocr_buf[3];
        }
        cs_high();
        spi_send_dummy(1);

        s_card_type = (ocr & 0x40000000UL) ? SD_TYPE_SDHC : SD_TYPE_V2;
    } else {
        s_card_type = SD_TYPE_V1;
    }

    /* ---- Fase 7: CMD16 — Imposta block size a 512B (SD v1/v2 non-HC) ---- */
    if (s_card_type != SD_TYPE_SDHC) {
        cs_low();
        r1 = sd_send_cmd(16, SD_BLOCK_SIZE, 0xFFU);
        cs_high();
        spi_send_dummy(1);
        if (r1 != 0x00U) {
            spi_unlock();
            return SD_ERR_INIT_FAIL;
        }
    }

    /* ---- Fase 8: Aumenta velocità SPI per trasferimenti dati ---- */
    spi_reconfigure(SD_PRESCALER_FAST);

    s_ready = true;
    spi_unlock();
    return SD_OK;
}

/* ============================================================================
 * READ / WRITE
 * ============================================================================ */

SD_Result_t SD_ReadBlock(uint32_t block_addr, uint8_t *buf)
{
    if (!s_ready || buf == NULL) return SD_ERR_PARAM;

    /* SDHC: indirizzo blocco; SD v1/v2: indirizzo byte */
    uint32_t addr = (s_card_type == SD_TYPE_SDHC) ? block_addr : (block_addr * SD_BLOCK_SIZE);

    spi_lock();
    cs_low();

    uint8_t r1 = sd_send_cmd(17, addr, 0xFFU);  /* CMD17: READ_SINGLE_BLOCK */
    if (r1 != 0x00U) {
        cs_high(); spi_send_dummy(1); spi_unlock();
        return SD_ERR_READ;
    }

    /* Attendi data token 0xFE */
    if (!sd_wait_data_token(SD_READ_TIMEOUT_MS)) {
        cs_high(); spi_send_dummy(1); spi_unlock();
        return SD_ERR_TIMEOUT;
    }

    /* Leggi 512 byte dati */
    for (uint16_t i = 0; i < SD_BLOCK_SIZE; i++) {
        buf[i] = spi_xfer(SD_DUMMY_BYTE);
    }

    /* Consuma 2 byte CRC (ignorati) */
    spi_xfer(SD_DUMMY_BYTE);
    spi_xfer(SD_DUMMY_BYTE);

    cs_high();
    spi_send_dummy(1);
    spi_unlock();
    return SD_OK;
}

SD_Result_t SD_WriteBlock(uint32_t block_addr, const uint8_t *buf)
{
    if (!s_ready || buf == NULL) return SD_ERR_PARAM;

    uint32_t addr = (s_card_type == SD_TYPE_SDHC) ? block_addr : (block_addr * SD_BLOCK_SIZE);

    spi_lock();

    /* Attendi che la scheda non sia occupata da scritture precedenti */
    if (!sd_wait_not_busy(SD_WRITE_TIMEOUT_MS)) {
        spi_unlock(); return SD_ERR_TIMEOUT;
    }

    cs_low();

    uint8_t r1 = sd_send_cmd(24, addr, 0xFFU);  /* CMD24: WRITE_SINGLE_BLOCK */
    if (r1 != 0x00U) {
        cs_high(); spi_send_dummy(1); spi_unlock();
        return SD_ERR_WRITE;
    }

    /* Invia data token */
    spi_xfer(SD_TOKEN_START_BLOCK);

    /* Invia 512 byte dati */
    for (uint16_t i = 0; i < SD_BLOCK_SIZE; i++) {
        spi_xfer(buf[i]);
    }

    /* Invia 2 byte CRC dummy */
    spi_xfer(SD_DUMMY_BYTE);
    spi_xfer(SD_DUMMY_BYTE);

    /* Leggi data response token */
    uint8_t resp = spi_xfer(SD_DUMMY_BYTE);
    if ((resp & SD_DATA_RESP_MASK) != SD_DATA_RESP_ACCEPTED) {
        cs_high(); spi_send_dummy(1); spi_unlock();
        return SD_ERR_WRITE;
    }

    /* Attendi fine scrittura (busy = MISO LOW) */
    if (!sd_wait_not_busy(SD_WRITE_TIMEOUT_MS)) {
        cs_high(); spi_send_dummy(1); spi_unlock();
        return SD_ERR_TIMEOUT;
    }

    cs_high();
    spi_send_dummy(1);
    spi_unlock();
    return SD_OK;
}

/* ============================================================================
 * GETTER
 * ============================================================================ */

SD_CardType_t SD_GetCardType(void)  { return s_card_type; }
bool          SD_IsReady(void)      { return s_ready; }

bool SD_IsPresent(void)
{
    /* nSD_PRESENT: active LOW = scheda presente */
    return (HAL_GPIO_ReadPin(nSD_PRESENT_GPIO_Port, nSD_PRESENT_Pin) == GPIO_PIN_RESET);
}
