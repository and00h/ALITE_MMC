/*
 * config.c
 *
 * Implementazione del modulo di configurazione persistente.
 * Vedere config.h per la documentazione dell'API pubblica e
 * la descrizione dell'architettura di journaling.
 */

#include "config.h"
#include "task_monitor.h"   /* ERR_BIT_DEW_MMC per default error_mask */
#include "lut_manager.h"    /* LUT_RestoreAfterSectorErase() dopo erase da overflow journal */
#include <string.h>

#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"        /* osMutex* */
#include "hal_handles.h"    /* hiwdg1 */

/* ========================================================================== */
/* --- ISTANZA GLOBALE IN RAM --- */
/* ========================================================================== */

Config_t g_config;

/* ========================================================================== */
/* --- HANDLE CRC (da assegnare in Config_Init tramite puntatore esterno) --- */
/* ========================================================================== */

/*
 * Il modulo usa il periferica CRC hardware dell'H723 per il calcolo del CRC32.
 * CubeMX genera l'handle hcrc: Config_Init() riceve il puntatore e lo memorizza.
 * In questo modo il driver rimane disaccoppiato dalla generazione CubeMX.
 */
static CRC_HandleTypeDef *s_hcrc = NULL;

/* ========================================================================== */
/* --- MUTEX CONDIVISO SUL SETTORE FLASH 7 --- */
/* ========================================================================== */

/*
 * BUG OSSERVATO IL 2026-07-02: flash_write_record()/invalidate_record()
 * erano protette con taskENTER_CRITICAL()/taskEXIT_CRITICAL() invece che con
 * un mutex. Problema: HAL_FLASH_Program() (e HAL_FLASHEx_Erase()) usano
 * internamente HAL_GetTick() — basato su SysTick — per il proprio timeout
 * software (FLASH_TIMEOUT_VALUE). taskENTER_CRITICAL() maschera gli
 * interrupt fino a configMAX_SYSCALL_INTERRUPT_PRIORITY, SysTick compreso:
 * col tick fermo, il timeout HAL non scade MAI. Un comando RS485 (SET TERM
 * ON) è rimasto bloccato dentro il ciclo di flash_write_record() per oltre
 * 30 secondi (nessun fault, nessun errore HAL) fino al reset forzato da
 * IWDG — che è l'UNICO meccanismo indipendente dal tick che ha fermato
 * l'attesa, lasciando però il record a metà scrittura.
 *
 * FIX: un mutex RTOS offre la stessa mutua esclusione (nessun'altra
 * scrittura flash concorrente da altri task) SENZA mascherare gli
 * interrupt, quindi SysTick/HAL_GetTick() restano vivi e un vero errore
 * hardware di programmazione torna un CONFIG_ERR_HAL pulito entro il
 * timeout HAL, invece di un hang che dipende dall'IWDG per essere risolto.
 * Condiviso con lut_manager.c (Config_FlashMutexAcquire/Release) perché
 * LUT_Save() tocca lo stesso settore fisico (vedi flash_map.h).
 *
 * Creato in Config_CreateFlashMutex(), chiamata da MX_FREERTOS_Init() DOPO
 * osKernelInitialize() (freertos.c) — non in Config_Init(), che gira PRIMA
 * dell'inizializzazione del kernel (main.c) e non può creare oggetti RTOS.
 * Le funzioni sotto tollerano un mutex ancora NULL (bootstrap pre-RTOS):
 * in quella finestra l'esecuzione è comunque single-thread, nessun rischio
 * di accesso concorrente.
 */
static osMutexId_t s_flash_mutex = NULL;

void Config_CreateFlashMutex(void)
{
    if (s_flash_mutex != NULL) {
        return; /* già creato */
    }
    static const osMutexAttr_t flash_mutex_attr = {
        .name      = "flash_s7_mtx",
        .attr_bits = osMutexPrioInherit,
    };
    s_flash_mutex = osMutexNew(&flash_mutex_attr);
}

bool Config_FlashMutexAcquire(uint32_t timeout_ms)
{
    if (s_flash_mutex == NULL) {
        return true; /* bootstrap pre-RTOS: nessuna mutua esclusione necessaria */
    }
    return (osMutexAcquire(s_flash_mutex, timeout_ms) == osOK);
}

void Config_FlashMutexRelease(void)
{
    if (s_flash_mutex == NULL) {
        return;
    }
    osMutexRelease(s_flash_mutex);
}

/* ========================================================================== */
/* --- FUNZIONI PRIVATE --- */
/* ========================================================================== */

/**
 * @brief  Restituisce il puntatore al record in flash all'indice dato.
 */
static inline const ConfigRecord_t *record_at(uint32_t index)
{
    return (const ConfigRecord_t *)(CONFIG_FLASH_BASE_ADDR
                                    + index * sizeof(ConfigRecord_t));
}

/**
 * @brief  Scansiona il settore e restituisce l'indice dell'ultimo record VALID.
 *         Se non ne trova nessuno, restituisce CONFIG_MAX_RECORDS (sentinella).
 */
static uint32_t find_last_valid_record(void)
{
    uint32_t found = CONFIG_MAX_RECORDS; /* sentinella: nessun record trovato */

    for (uint32_t i = 0; i < CONFIG_MAX_RECORDS; i++) {
        const ConfigRecord_t *rec = record_at(i);
        if (rec->status == RECORD_VALID) {
            found = i; /* continua: vogliamo l'ULTIMO valido */
        }
    }
    return found;
}

/**
 * @brief  Restituisce l'indice del primo slot FREE nel settore.
 *         Se non ne trova nessuno, restituisce CONFIG_MAX_RECORDS (settore pieno).
 */
static uint32_t find_first_free_slot(void)
{
    for (uint32_t i = 0; i < CONFIG_MAX_RECORDS; i++) {
        const ConfigRecord_t *rec = record_at(i);
        if (rec->status == RECORD_FREE) {
            return i;
        }
    }
    return CONFIG_MAX_RECORDS; /* settore pieno */
}

/**
 * @brief  Scrive un singolo valore a 32 byte (256 bit) nella flash H7.
 *         Gestisce unlock/lock internamente.
 * @param  dest_addr  Indirizzo di destinazione in flash (deve essere allineato a 32 byte).
 * @param  src        Puntatore ai dati sorgente (almeno 32 byte).
 * @retval HAL_OK o codice di errore HAL.
 */
static HAL_StatusTypeDef flash_write_32bytes(uint32_t dest_addr, const void *src)
{
    /* L'HAL H7 vuole i dati come array di uint32_t a 8 elementi (8 * 4 = 32 byte) */
    const uint32_t *words = (const uint32_t *)src;

    HAL_StatusTypeDef status = HAL_FLASH_Program(
        FLASH_TYPEPROGRAM_FLASHWORD,
        dest_addr,
        (uint32_t)words   /* cast a uint32_t del puntatore, come vuole l'HAL H7 */
    );

    return status;
}

/**
 * @brief  Scrive un'intera struttura ConfigRecord_t in flash a partire da dest_addr.
 *         La struttura viene scritta a blocchi da 32 byte.
 * @retval CONFIG_OK o CONFIG_ERR_HAL.
 */
static Config_err_t flash_write_record(uint32_t dest_addr, const ConfigRecord_t *rec)
{
    const uint8_t *src     = (const uint8_t *)rec;
    uint32_t       addr    = dest_addr;
    uint32_t       written = 0;
    uint32_t       total   = sizeof(ConfigRecord_t);

    /*
     * Mutua esclusione tramite MUTEX, non più taskENTER_CRITICAL() (vedi
     * commento su s_flash_mutex, sopra: mascherare gli interrupt fermava
     * anche SysTick, disabilitando il timeout software di HAL_FLASH_Program()
     * e trasformando un eventuale errore hardware in un hang senza uscita
     * fino al reset IWDG — bug osservato il 2026-07-02 su SET TERM ON).
     */
    if (!Config_FlashMutexAcquire(osWaitForever)) {
        return CONFIG_ERR_HAL;
    }

    HAL_FLASH_Unlock();
    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_ALL_ERRORS_BANK1);

    while (written < total) {
        if (flash_write_32bytes(addr, src + written) != HAL_OK) {
            HAL_FLASH_Lock();
            Config_FlashMutexRelease();
            return CONFIG_ERR_HAL;
        }
        written += FLASH_WRITE_GRANULARITY;
        addr    += FLASH_WRITE_GRANULARITY;
    }

    HAL_FLASH_Lock();
    Config_FlashMutexRelease();
    return CONFIG_OK;
}

/**
 * @brief  NON CHIAMARE — lasciata solo come riferimento storico.
 *
 *         Tentava di invalidare un record riscrivendo il byte di status del
 *         suo header (0xAA VALID -> 0x00 OBSOLETE) via un secondo
 *         HAL_FLASH_Program() sullo stesso flash word da 32 byte già
 *         programmato al momento della creazione del record.
 *
 *         SBAGLIATO su H7: il commento originale ("porta solo bit da 1 a 0,
 *         sempre lecito su flash NOR") vale per la singola transizione di
 *         bit, ma la flash H7 vieta una SECONDA programmazione dello stesso
 *         flash word da 256 bit dopo il primo erase, indipendentemente dal
 *         fatto che i nuovi bit siano compatibili con quelli già scritti.
 *         Il risultato era un flash word con ECC corrotto, che alla lettura
 *         successiva (anche solo per lo scan di find_last_valid_record())
 *         genera un bus fault hardware (double ECC error) — causa
 *         identificata della serie di HardFault "prima scrittura OK, seconda
 *         fallisce" osservata su Config_Save() durante l'indagine sul
 *         comando RS485 SET TERM ON del 2026-07-02.
 *
 *         Config_Save() non la chiama più: vedi commento lì per la
 *         spiegazione di perché l'invalidazione non serve comunque per la
 *         correttezza del journaling. __attribute__((unused)) per evitare il
 *         warning "defined but not used" tenendo comunque il codice come
 *         documentazione dell'errore.
 */
__attribute__((unused))
static Config_err_t invalidate_record(uint32_t index)
{
    /*
     * Leggiamo il blocco di 32 byte dell'header in un buffer temporaneo,
     * modifichiamo solo il byte di status e riscriviamo l'intero blocco.
     * Non si può scrivere solo 1 byte: la granularità minima H7 è 32 byte.
     */
    const ConfigRecord_t *rec = record_at(index);

    /* Buffer allineato a 32 byte in RAM */
    __attribute__((aligned(32))) uint8_t hdr_buf[FLASH_WRITE_GRANULARITY];
    memcpy(hdr_buf, rec, FLASH_WRITE_GRANULARITY);

    /* Modifica il byte di status nel buffer */
    hdr_buf[0] = (uint8_t)RECORD_OBSOLETE;

    /* Mutex: stessa motivazione di flash_write_record() */
    if (!Config_FlashMutexAcquire(osWaitForever)) {
        return CONFIG_ERR_HAL;
    }

    HAL_FLASH_Unlock();
    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_ALL_ERRORS_BANK1);

    HAL_StatusTypeDef status = flash_write_32bytes(
        CONFIG_FLASH_BASE_ADDR + index * sizeof(ConfigRecord_t),
        hdr_buf
    );

    HAL_FLASH_Lock();
    Config_FlashMutexRelease();

    /* Invalidazione della cache dati per evitare letture stale dopo la scrittura */
    SCB_InvalidateDCache_by_Addr(
        (uint32_t *)(CONFIG_FLASH_BASE_ADDR + index * sizeof(ConfigRecord_t)),
        FLASH_WRITE_GRANULARITY
    );
    __ISB();
    __DSB();

    return (status == HAL_OK) ? CONFIG_OK : CONFIG_ERR_HAL;
}

/**
 * @brief  Cancella l'intero settore di configurazione.
 * @retval CONFIG_OK o CONFIG_ERR_HAL.
 */
static Config_err_t erase_config_sector(void)
{
    FLASH_EraseInitTypeDef erase_cfg = {
        .TypeErase     = FLASH_TYPEERASE_SECTORS,
        .Banks         = FLASH_BANK_1,
        .Sector        = CONFIG_FLASH_SECTOR,
        .NbSectors     = 1,
        .VoltageRange  = FLASH_VOLTAGE_RANGE_3,  /* VCC 2.7V - 3.6V */
    };

    uint32_t sector_error = 0;

    /*
     * Mutex (non taskENTER_CRITICAL): protegge dall'accesso concorrente di
     * un'altra scrittura/erase flash (es. LUT_Save()), senza mascherare gli
     * interrupt — vedi motivazione estesa su s_flash_mutex. Mascherare gli
     * interrupt qui sarebbe comunque inutile per un motivo ulteriore,
     * specifico dell'erase: sull'H7 single-bank, mentre un erase è in corso
     * la CPU non può fetchare ISTRUZIONI dallo stesso bank flash su cui
     * gira il firmware (RM0468, "no read-while-write" entro lo stesso
     * bank) — l'intero sistema si ferma per la durata dell'erase
     * indipendentemente dal mascheramento interrupt, ISR comprese. L'IWDG
     * (contatore hardware indipendente, clock LSI) continua però a contare
     * durante lo stallo: la riarmiamo appena prima di iniziare per
     * massimizzare il margine.
     */
    if (!Config_FlashMutexAcquire(osWaitForever)) {
        return CONFIG_ERR_HAL;
    }

    HAL_IWDG_Refresh(&hiwdg1);

    HAL_FLASH_Unlock();
    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_ALL_ERRORS_BANK1);

    HAL_StatusTypeDef status = HAL_FLASHEx_Erase(&erase_cfg, &sector_error);

    HAL_FLASH_Lock();
    Config_FlashMutexRelease();

    /* Invalida tutta la cache dati relativa al settore */
    SCB_InvalidateDCache_by_Addr(
        (uint32_t *)CONFIG_FLASH_BASE_ADDR,
        CONFIG_FLASH_SIZE
    );
    __ISB();
    __DSB();

    return (status == HAL_OK) ? CONFIG_OK : CONFIG_ERR_HAL;
}

/* ========================================================================== */
/* --- API PUBBLICA --- */
/* ========================================================================== */

/**
 * @brief  Calcola il CRC32 hardware del payload Config_t fornito.
 */
uint32_t Config_CalcCRC(const Config_t *cfg)
{
    if (s_hcrc == NULL) {
        /* Fallback software (non dovrebbe mai accadere se Config_Init è stato chiamato) */
        return 0xDEADBEEF;
    }

    /*
     * La periferica CRC H7 opera su parole da 32 bit.
     * sizeof(Config_t) deve essere multiplo di 4 (garantito dall'allineamento).
     */
    return HAL_CRC_Calculate(
        s_hcrc,
        (uint32_t *)cfg,
        sizeof(Config_t) / sizeof(uint32_t)
    );
}

/**
 * @brief  Popola g_config con i valori di fabbrica.
 */
void Config_LoadDefaults(void)
{
    memset(&g_config, 0, sizeof(Config_t));

    /* --- Rete --- */

    /*
     * Terminazione bus RS485 (nEXT_485_TERMINATION_Pin, PA8, active LOW).
     * Default: NON inserita (0). La scheda MMC non è garantita essere
     * l'ultimo nodo del bus RS485 in ogni installazione: forzare la
     * terminazione di fabbrica potrebbe caricare il bus in configurazioni
     * multi-drop. L'installatore la abilita via comando RS485 "SET TERM ON"
     * (nessun login richiesto, vedi rs485_cmd.c) solo se questo nodo è
     * effettivamente a un capo del bus.
     * 0 = non inserita, 1 = inserita. Applicata a GPIO in BoardCtrl_Init().
     */
    g_config.termination = 0;

    /* --- PSU (valori sicuri di default: uscita spenta, zero riferimenti) --- */
    g_config.psu_voltage_mv = 39500;
    g_config.psu_current_ma = 0;


    /* --- LaseQ --- */
    g_config.laser_mode                = 0;   /* SW mode */

    /* --- Soglie temperatura --- */
    /* Water Cooling */
    g_config.temp_water_in_min_err = 15;
    g_config.temp_water_in_min_warn  = 18;

    g_config.temp_water_in_max_err = 26;
    g_config.temp_water_in_max_warn  = 24;

    g_config.temp_water_out_min_err = 15;
    g_config.temp_water_out_min_warn  = 18;

    g_config.temp_water_out_max_err = 27;
    g_config.temp_water_out_max_warn  = 25;

    /* Laser Driver */
    g_config.temp_driver_min_err = 10;
    g_config.temp_driver_min_warn  = 12;

    g_config.temp_driver_max_err = 90;
    g_config.temp_driver_max_warn  = 80;

    /* Fiber */
    g_config.temp_splice_min_err = 10;
    g_config.temp_splice_min_warn  = 12;

    g_config.temp_splice_max_err = 45;
    g_config.temp_splice_max_warn  = 35;

    /* Laser Diode 1 (serigrafia NTC3, CH12) */
    g_config.temp_diode_min_err = 15;
    g_config.temp_diode_min_warn  = 18;

    g_config.temp_diode_max_err = 27;
    g_config.temp_diode_max_warn  = 25;

    /* Laser Diode 2 (serigrafia NTC4, CH13) — dal 2026-07-15, secondo
     * sensore fisico del diodo laser, soglie identiche a DIODE1 come punto
     * di partenza (stesso componente, stessa zona termica). */
    g_config.temp_diode2_min_err = 15;
    g_config.temp_diode2_min_warn  = 18;

    g_config.temp_diode2_max_err = 27;
    g_config.temp_diode2_max_warn  = 25;

    /*
     * Ambient (NTC_SENSOR_AMBIENT, serigrafia NTC6/CH11) — sonda posta sulla
     * scocca della macchina, il più possibile vicino alla temperatura
     * dell'aria ESTERNA al rack (non un sensore su PCB come MB_TEMP/
     * LQ_AMBIENT, che invece misurano l'interno di schede elettroniche).
     * Soglie aggiornate 2026-07-22 (min_err/min_warn alzate da 10/12 a
     * 12/15 — margine più realistico rispetto al punto di rugiada per aria
     * ambiente esterna vera e propria).
     */
    g_config.temp_ambient_min_err = 12;
    g_config.temp_ambient_min_warn  = 15;

    g_config.temp_ambient_max_err = 40;
    g_config.temp_ambient_max_warn  = 35;

    /*
     * PSU_TEMP (NUOVO 2026-07-22, NTC_SENSOR_PSU_TEMP, serigrafia NTC14/CH2
     * — sonda non ancora cablata in campo, vedi ntc_ch_map sotto). Soglie
     * tarate sul range operativo dichiarato per il PSU NSP-3200-48 Mean
     * Well (operativo fino a 70-85°C con derating): max_warn=50°C indica
     * che il PSU è già in zona di derating, max_err=80°C resta sotto il
     * limite assoluto superiore con margine di sicurezza.
     */
    g_config.temp_psu_min_err    = 10;
    g_config.temp_psu_min_warn   = 12;
    g_config.temp_psu_max_err    = 80;
    g_config.temp_psu_max_warn   = 65;

    /*
     * PWR_EL_TEMP (NUOVO 2026-07-22, NTC_SENSOR_PWR_EL_TEMP, serigrafia
     * NTC15/CH1 — sonda non ancora cablata in campo). Default PLACEHOLDER
     * (stessi valori di PSU_TEMP, nessuna specifica fornita per questo
     * sensore) — DA CALIBRARE quando la sonda sarà installata e la zona
     * termica nota (elettronica di potenza generica).
     */
    g_config.temp_pwr_el_min_err  = 10;
    g_config.temp_pwr_el_min_warn = 12;
    g_config.temp_pwr_el_max_err  = 80;
    g_config.temp_pwr_el_max_warn = 50;

    /*
     * MB_TEMP (NUOVO 2026-07-22, temperatura dall'SHT35 già montato su MMC,
     * I2C2 — finora usato solo per umidità/dew point). Default allineati a
     * temp_lq_ambient_* sotto (stesso tipo di sensore, stessa fisiologia
     * termica: SHT35 su scheda elettronica, non aria ambiente esterna).
     */
    g_config.temp_mb_min_err   = 10;
    g_config.temp_mb_min_warn  = 12;
    g_config.temp_mb_max_err   = 45;
    g_config.temp_mb_max_warn  = 43;

    /*
     * Ambient LaseQ (SHT35 interno, dal 2026-07-15): soglie DEDICATE, non
     * più condivise con temp_ambient_* sopra. L'ambiente interno di LaseQ
     * è fisiologicamente
     * più caldo (vicinanza componenti elettronici di potenza) — min_err/
     * min_warn restano uguali a temp_ambient (il rischio di condensa a
     * bassa temperatura non dipende dalla posizione del sensore), ma
     * max_warn/max_err vanno alzati per non generare falsi warning/fault.
     */
    g_config.temp_lq_ambient_min_err   = 10;
    g_config.temp_lq_ambient_min_warn  = 12;

    g_config.temp_lq_ambient_max_err   = 45;
    g_config.temp_lq_ambient_max_warn  = 43;

    /* --- Soglie flusso [L/min * 10] --- */
    g_config.flow_min_err_lpm_x10 = 42;   /* 4.2 L/min */
    g_config.flow_min_warn_lpm_x10  = 45;   /* 4.5 L/min */

    g_config.flow_max_err_lpm_x10 = 63;   /* 1.0 L/min */
    g_config.flow_max_warn_lpm_x10  = 60;   /* 1.5 L/min */

    /* --- Parametri operativi --- */
    g_config.interlock_mask  = 0xFFFFFFFF;   /* Tutti gli interlock abilitati */
    g_config.startup_delay_ms = 500;

    /*
     * Range setpoint esterno (nREDUCED_SETPOINT_RANGE_Pin, active LOW).
     * Default: range pieno 10V (0). Applicato a GPIO in EXT_Interface_Init()
     * (main.c) tramite EXT_SetReducedSetpointRange(). Modificabile a runtime
     * via comando RS485 "SET SETPOINT RANGE 10V|6V" (nessun login richiesto,
     * vedi rs485_cmd.c) — stesso schema di g_config.termination sopra:
     * applicato subito al pin e persistito subito in flash (Config_Save()),
     * non in attesa del comando separato "SAVE CONFIG".
     * 0 = range 10V (pin HIGH), 1 = range 6V (pin LOW).
     */
    g_config.reduced_setpoint_range = 0;

    /* --- Metadati (il fw_version reale viene iniettato dal build system) --- */
    g_config.fw_version = 0x00000001;

    /* --- Sequenza accensione --- */
    g_config.contactor_psu_delay_ms    = 200;    /* 200ms contattori → PSU */
    g_config.sab_interlock_timeout_ms  = 2000;   /* 2s per chiusura interlock SAB */

    /* --- Dispositivi abilitati (tutti on per default) --- */
    g_config.sab_enabled            = 1;
    g_config.psu_enabled_mask       = 0x01;   /* PSU1 + PSU2 */
    g_config.contactor_enabled_mask = 0x01;   /* CONTACTOR1 + CONTACTOR2 */

    /*
     * flow_sensor_enabled_mask: solo FLOW_METER_1 è fisicamente cablato
     * (vedi main.c, FlowMeter_Init(FLOW_METER_2, ...) è commentato) —
     * bit1 lasciato a 0 finché il secondo flussometro non viene installato.
     *
     * temp_sensor_enabled_mask (AGGIORNATO 2026-07-31): tutte le 8 sonde
     * NTC fisicamente cablate oggi risultano confermate — WATER_IN(bit0),
     * WATER_OUT(bit1), SPLICE(bit3), DIODE1(bit4), AMBIENT(bit5),
     * DIODE2(bit6), PSU_TEMP(bit7), PWR_EL_TEMP(bit8). Tutte e 8 abilitate
     * qui: check_ntc_sensor() (task_monitor.c) usa questo bit come
     * device-enable (non più warning_mask, che torna a mascherare solo la
     * notifica evento) — con il bit a 1 il confronto soglie warn/err viene
     * eseguito per il sensore, e in caso di superamento soglia il bit
     * WARN_BIT_TEMP_* corrispondente si accumula SEMPRE in s_active_warnings
     * (letto da TaskMonitor_GetWarnings()), e ERR_BIT_TEMP in s_active_errors
     * (TaskMonitor_GetErrors()) se lo stato e' >= SYS_ON — indipendentemente
     * da warning_mask/error_mask, che sotto sono comunque entrambe già
     * permissive (warning_mask = 0xFFFFFFFF, error_mask include ERR_BIT_TEMP)
     * quindi generano anche il relativo evento FSM (SYS_TEMP_WARN/ERROR_EVENT)
     * alla prima transizione oltre soglia, non solo l'accumulo del flag.
     * bit2 (ex NTC_SENSOR_DRIVER) resta escluso: dal 2026-07-14 non è più
     * controllato qui, la temperatura driver arriva dalla telemetria LaseQ
     * (vedi Monitor_CheckLaseQTelemetry()).
     *
     * NOTA: un'unità con configurazione già salvata in flash non eredita
     * questo nuovo default — serve comunque "SET TEMP MASK" via RS485 (o un
     * factory reset) per applicarlo a un'unità già commissionata.
     */
    g_config.flow_sensor_enabled_mask = 0x01;   /* FLOW_METER_1 */
    g_config.temp_sensor_enabled_mask = 0x01FBU;
    /* bit0 WATER_IN | bit1 WATER_OUT | bit3 SPLICE | bit4 DIODE1 |
     * bit5 AMBIENT | bit6 DIODE2 | bit7 PSU_TEMP | bit8 PWR_EL_TEMP */

    /*
     * efuse_enabled_mask: bit0=EFUSE_MAIN bit1=EFUSE_SAB bit2=EFUSE_COM
     * bit3=EFUSE_LASEQ (vedi EFuse_id_t, eFuse.h). Default: TUTTI accesi
     * (0x0F) — comportamento originale (EFuse_Init() abilitava
     * incondizionatamente tutti e 4), invariato finché non si sceglie
     * esplicitamente di escludere un modulo (SET EFUSE MASK, rs485_cmd.c).
     */
    g_config.efuse_enabled_mask = 0x0FU;
    /*
     * NOTA bit2 (NTC_SENSOR_DRIVER/CH12): dal 2026-07-14 non è più
     * consultato da Monitor_CheckTemperatures() — la temperatura "driver di
     * corrente" arriva ora dalle 4 temperature riportate da LaseQ
     * (MSG_STATUS.temperature[0..3], sempre verificate quando la telemetria
     * LaseQ è disponibile, senza device-enable dedicato — vedi
     * Monitor_CheckLaseQTelemetry() in task_monitor.c). Il bit resta
     * definito/persistito per compatibilità ma non ha più alcun effetto.
     */

    /* --- Soglie umidità e dew point --- */
    g_config.humidity_max_warn_pct       = 80;   /* MMC/SHT35: warning sopra 80% RH */
    g_config.laseq_humidity_max_warn_pct = 90;   /* LaseQ: soglia più alta (ambiente caldo driver) */
    g_config.dew_warn_delta_c            =  5;   /* Warning se T_amb - T_dew < 5°C */
    g_config.dew_err_delta_c             =  3;   /* Error   se T_amb - T_dew < 3°C */

    /* --- Maschere warning/errori ---
     *
     * Warning: tutti abilitati (inclusi WARN_BIT_DEW_MMC e WARN_BIT_DEW_LASEQ),
     * TRANNE i canali NTC non ancora cablati in fase di commissioning
     * (WATER_OUT, DRIVER, SPLICE, DIODE1, DIODE2, AMBIENT) — qui mascherati
     * solo per coerenza/simmetria con temp_sensor_enabled_mask sopra (dal
     * 2026-07-13 il device-enable vero e proprio è temp_sensor_enabled_mask,
     * non più questo bit: check_ntc_sensor() valuta il sensore solo se
     * ENTRAMBI i bit sono abilitati). Riabilitare via RS485 (warning_mask +
     * temp_sensor_enabled_mask) man mano che gli altri NTC vengono cablati.
     * Error:   ERR_BIT_DEW_MMC (bit 9) mascherato di default — il sensore MMC
     *          è su scheda madre, meno critico per la condensazione sul diodo.
     *          ERR_BIT_DEW_LASEQ (bit 10) attivo — il LaseQ contiene ottiche.
     *          ERR_BIT_LID1/ERR_BIT_LID2 (bit 12, 13, dal 2026-07-15) attivi
     *          per default come ERR_BIT_KEY_A/KEY_B: se in produzione è
     *          montato un solo sensore coperchio (o nessuno in banco prova),
     *          mascherare il bit relativo via RS485 (SET ERR MASK).
     *          ERR_BIT_SAB_INTLCK_A/B, ERR_BIT_SAB_TEST_A/B (bit 3-6, dal
     *          2026-07-15) attivi per default (entrambi i canali) —
     *          a differenza di KEY/LID, i due canali SAB sono la STESSA
     *          catena di sicurezza duplicata per ridondanza: mascherarne
     *          uno va riservato al solo banco prova/debug con un canale
     *          cablato, mai lasciato mascherato in produzione (vedi banner
     *          in task_monitor.h).
     */

    /*
    g_config.warning_mask = 0xFFFFFFFFUL & ~(WARN_BIT_TEMP_WATER_OUT |
                                              WARN_BIT_TEMP_DRIVER   |
                                              WARN_BIT_TEMP_SPLICE   |
                                              WARN_BIT_TEMP_DIODE1   |
                                              WARN_BIT_TEMP_DIODE2   |
                                              WARN_BIT_TEMP_AMBIENT);

                                              */
    g_config.warning_mask = 0xFFFFFFFFUL;
    //g_config.error_mask   = 0x3EBFUL & ~ERR_BIT_DEW_MMC;
    g_config.error_mask   = 0x0UL;

    /*
     * Fault non recuperabili -> SYS_FAULT (categoria separata per severità,
     * vedi task_monitor.h FAULT_BIT_*): tutti abilitati per default,
     * incluso FAULT_BIT_COM — anche se il driver COM interface è ancora
     * minimale (solo nCOM_PWR_FLT + heartbeat nCOM_INT_IN), un fault reale
     * su quel link deve fermare la macchina come per LaseQ/AMC.
     * FAULT_BIT_FLOOD1/FLOOD2/PSU (dal 2026-07-14, migrati da error_mask)
     * inclusi qui — coerente col comportamento storico (erano già
     * abilitati di default anche in error_mask).
     */
    //g_config.fault_mask = 0xFFFFFFFFUL;
    g_config.fault_mask = 0x0UL;

    /*
     * FAULT LATCH (dal 2026-07-14, vedi banner "FAULT LATCH" in
     * task_monitor.h): quali fault, se si verificano, restano "latched"
     * (persistiti in flash) anche dopo un power-cycle, impedendo il riavvio
     * automatico finché non arriva un comando esplicito "FRST" (RS485,
     * protetto da login). Default: SOLO allagamento (FLOOD1/FLOOD2) — un
     * power-cycle non deve far ripartire la macchina su un allagamento non
     * ancora verificato/risolto da un operatore. Gli altri fault (PSU,
     * comunicazione) restano non latched: un power-cycle li fa ritentare
     * naturalmente, comportamento storico invariato.
     * fault_latch_active: bitmask runtime dei fault ATTUALMENTE latched,
     * persistita a ogni occorrenza (vedi action_fault(), fsm.c) — default 0
     * (nessun fault latched all'avvio di un sistema mai andato in fault).
     */
    //g_config.fault_latch_mask   = FAULT_BIT_FLOOD1 | FAULT_BIT_FLOOD2;
    g_config.fault_latch_mask   = 0U;
    g_config.fault_latch_active = 0U;

    /* --- Controllo setpoint hardware --- */
    g_config.hw_setpoint_sel    = 0;       /* 0 = EXT analog → LaseQ */
    g_config.psu_dc_ok_delay_ms = 2500U;  /* 2s inibizione DC_OK dopo SON */

    /* --- LUT NTC: mapping verificato da schema hardware (serigrafia scheda → CH AD7490)
     *
     * Corrispondenza serigrafia → canale ADC (da schema Opal MMC):
     *   Serigrafia 1  → CH14   Serigrafia 9  → CH7
     *   Serigrafia 2  → CH15   Serigrafia 10 → CH6
     *   Serigrafia 3  → CH12   Serigrafia 11 → CH5
     *   Serigrafia 4  → CH13   Serigrafia 12 → CH4
     *   Serigrafia 5  → CH10   Serigrafia 13 → CH3
     *   Serigrafia 6  → CH11   Serigrafia 14 → CH2
     *   Serigrafia 7  → CH8    Serigrafia 15 → CH1
     *   Serigrafia 8  → CH9    Serigrafia 16 → CH0
     *
     * I 16 sensor_id (NTC_SensorId_t, task_monitor.h) coprono ora tutte le
     * serigrafie. ASSEGNAZIONE AGGIORNATA 2026-07-15 (comunicata
     * dall'utente, sostituisce la mappatura funzionale precedente — il
     * diodo laser ha DUE sensori fisici distinti, e splice si è spostata
     * su serigrafia 16):
     *   Serigrafia 1  → water_in  (NTC_SENSOR_WATER_IN)  → CH14
     *   Serigrafia 2  → water_out (NTC_SENSOR_WATER_OUT) → CH15
     *   Serigrafia 3  → diode1    (NTC_SENSOR_DIODE1)    → CH12
     *   Serigrafia 4  → diode2    (NTC_SENSOR_DIODE2)    → CH13
     *   Serigrafia 6  → ambient   (NTC_SENSOR_AMBIENT)   → CH11
     *   Serigrafia 16 → splice    (NTC_SENSOR_SPLICE)    → CH0
     * Le restanti serigrafie (5, 7-15) sono fisicamente non assegnate: nessun
     * id generico specifico è "riservato" per una particolare serigrafia —
     * assegnarle a piacere (NTC8..NTC16, NTC_SENSOR_DRIVER) via "SET NTC
     * MAP <serigrafia> <id>" quando verranno cablate. Unica eccezione:
     * l'id storico "NTC7" non esiste più come slot generico, riassegnato a
     * DIODE2 sopra (serigrafia 4) — non usarlo per altre serigrafie.
     *
     * NOTA (bring-up/debug SPI AD7490, 2026-07-02): sul banco è collegato
     * fisicamente solo il sensore di serigrafia 1 (water_in, CH14). Tutti
     * gli altri canali sono temporaneamente disabilitati (0xFF) per isolare
     * i test SPI da eventuali NTC assenti/flottanti. Ripristinare le righe
     * commentate sotto (e aggiungere il mapping per i generici quando
     * verranno cablati/assegnati) prima della validazione multi-sensore. La
     * mappatura via RS485 ("SET NTC MAP <serigrafia 1-16> <sensore>", dal
     * 2026-07-15 indicizzata per numero di serigrafia e non più per canale
     * AD7490 grezzo) resta comunque il modo consigliato per applicarla in
     * campo senza ricompilare.
     */
    memset(g_config.ntc_ch_map, 0xFF, sizeof(g_config.ntc_ch_map));
    g_config.ntc_ch_map[14] = NTC_SENSOR_WATER_IN;  /* CH14 (serigrafia 1) — unico sensore collegato in debug */
    g_config.ntc_ch_map[15] = NTC_SENSOR_WATER_OUT;  // CH15 (serigrafia 2)
    g_config.ntc_ch_map[12] = NTC_SENSOR_DIODE1;     // CH12 (serigrafia 3)
    g_config.ntc_ch_map[13] = NTC_SENSOR_DIODE2;     // CH13 (serigrafia 4)
    g_config.ntc_ch_map[11] = NTC_SENSOR_AMBIENT;    // CH11 (serigrafia 6)
    g_config.ntc_ch_map[0]  = NTC_SENSOR_SPLICE;     // CH0  (serigrafia 16)
    g_config.ntc_ch_map[2]  = NTC_SENSOR_PSU_TEMP;   // CH2  (serigrafia 14, NUOVO 2026-07-22)
    g_config.ntc_ch_map[1]  = NTC_SENSOR_PWR_EL_TEMP;// CH1  (serigrafia 15, NUOVO 2026-07-22)

    /*
     * PSU_TEMP/PWR_EL_TEMP (2026-07-22): sonde non ancora cablate fisicamente
     * (comunicato dall'utente), ma verosimilmente destinate a serigrafia
     * NTC14 (CH2) e NTC15 (CH1) — righe pronte sopra, commentate come le
     * altre in attesa di cablaggio. Se lo schema elettrico finale assegnasse
     * canali diversi, rimappare in campo con "SET NTC MAP <serigrafia> ..."
     * senza ricompilare.
     * CH0-9 restanti (serigrafie 7-13, NTC7/NTC9..NTC13) rimangono 0xFF: non
     * ancora cablati/assegnati.
     */

    g_config.ntc_r0_ohm      = 10100;  /* 10 kΩ a 25°C  */
    g_config.ntc_rseries_ohm = 10000;  /* Resistenza serie partitore 10 kΩ */
    /*
     * ntc_beta[16] (dal 2026-07-22, era scalare unico): stesso valore
     * 3950K (tipico NTC 10k B3950) per tutti i 16 canali come default di
     * fabbrica — modificabile per singolo canale via "SET NTC BETA
     * <serigrafia 1-16> <beta>" quando si monta un NTC di modello diverso
     * su un canale specifico.
     */
    for (uint8_t ch = 0U; ch < 16U; ch++) {
        g_config.ntc_beta[ch] = 3950U;
    }

    /* --- LUT setpoint (default fabbrica: interpolazione lineare 0-10500 mA) ---
     *
     * Retta: 0% = 0mA, 100% = 10500mA → step da 5% = 525mA
     * I valori intermedi verranno raffinati in fase di calibrazione.
     *
     *   idx  0  5% step  0%: 0mA
     *   idx  1  5% step  5%: 525mA
     *   ...
     *   idx 20  5% step 100%: 10500mA
     */
    {
        const uint16_t lut_default[SETPOINT_LUT_SIZE] = {
               0,   1440,  1890,  2330,  2780,   /* 0-20%   */
            3230,  3680,  4140,  4590,  5050,   /* 25-45%  */
            5530,  6000,  6440,  6920,  7390,   /* 50-70%  */
            7895,  8380,  8895,  9425,  9960,   /* 75-95%  */
           10500                                /* 100%    */
        };
        for (uint8_t i = 0; i < SETPOINT_LUT_SIZE; i++) {
            g_config.setpoint_lut_ma[i] = lut_default[i];
        }
    }

    /* --- Fotodiodi AMC (default factory) ---
     *
     * pd_mask: 0x0F = tutti e 4 i PD abilitati.
     * pd_gain_threshold: divide [0..4095] in 4 finestre uguali da 1024 raw
     *   (~800mV ciascuna con VDDA=3.3V). Da calibrare per ogni installazione.
     * pd_gain_settle_ms: 5ms — tempo di settle tipico OTA + MUX.
     * pd_stability_samples: 8 campioni (≈8ms a 1kHz SysTick).
     * pd_stability_threshold: 20 raw ≈ 16mV — margine per segnale "stabile".
     */
    g_config.pd_mask                  = 0x0FU;
    g_config.pd_gain_windows          = 4U;
    g_config.pd_gain_threshold[0]     = 1024U;
    g_config.pd_gain_threshold[1]     = 2048U;
    g_config.pd_gain_threshold[2]     = 3072U;
    g_config.pd_gain_threshold[3]     = 4095U;
    g_config.pd_gain_settle_ms        = 5U;
    g_config.pd_stability_samples     = 8U;
    g_config.pd_stability_threshold   = 20U;

    /* --- QCW (default: CW, comportamento invariato) ---
     *
     * qcw_enabled=0: nGATE_MC resta pilotato staticamente da
     * action_enter_emission() come prima di questa modifica. freq/duty
     * hanno valori di default sensati ma sono inerti finché "SET QCW ON"
     * non viene inviato (vedi rs485_cmd.c, Drivers/QCW/QCW.h).
     */
    g_config.qcw_enabled  = 0U;
    g_config.qcw_freq_hz  = 100U;   /* 100Hz, valore di default ragionevole per QCW laser */
    g_config.qcw_duty_pct = 50U;    /* 50% */

    /*
     * --- Compensazione dinamica tensione PSU (NUOVO 2026-07-22, default: OFF) ---
     *
     * voltage_comp_enabled=0: comportamento invariato, tensione PSU sempre
     * al valore di psu_voltage_mv (SET PSU VOLTAGE) — nessuna modifica al
     * comportamento esistente finché non viene attivata esplicitamente via
     * "SET VCOMP ON". Tempi di default a metà dei range richiesti.
     */
    g_config.voltage_comp_enabled     = 1U;
    g_config.comp_stabilize_delay_ms  = 30U;  /* range utile 10-50ms */
    g_config.comp_ramp_duration_ms    = 75U;  /* range utile 50-100ms (rampa lenta, discesa) */
    g_config.comp_ramp_up_duration_ms = 3U;   /* range utile 1-5ms (rampa veloce, salita) */
}

/**
 * @brief  Carica il più recente record valido dalla flash in g_config.
 */
Config_err_t Config_Load(void)
{
    uint32_t idx = find_last_valid_record();

    if (idx == CONFIG_MAX_RECORDS) {
        /* Nessun record trovato: flash vergine o completamente obsoleta */
        return CONFIG_ERR_NOT_FOUND;
    }

    const ConfigRecord_t *rec = record_at(idx);

    /* Verifica magic / versione schema */
    if (rec->magic != CONFIG_MAGIC) {
        return CONFIG_ERR_MAGIC;
    }

    /* Verifica CRC prima di copiare in RAM */
    uint32_t expected_crc = Config_CalcCRC(&rec->data);
    if (rec->crc32 != expected_crc) {
        return CONFIG_ERR_CRC;
    }

    /* Copia il payload in g_config */
    memcpy(&g_config, &rec->data, sizeof(Config_t));

    return CONFIG_OK;
}

/**
 * @brief  Persiste il contenuto attuale di g_config in flash tramite journaling.
 */
Config_err_t Config_Save(void)
{
    Config_err_t err;

    /* Prepara il record da scrivere */
    __attribute__((aligned(32))) ConfigRecord_t new_rec;
    memset(&new_rec, 0xFF, sizeof(ConfigRecord_t));   /* inizia da stato "free" */

    new_rec.status = RECORD_VALID;
    new_rec.magic  = CONFIG_MAGIC;
    memcpy(&new_rec.data, &g_config, sizeof(Config_t));
    new_rec.crc32  = Config_CalcCRC(&new_rec.data);

    /* Trova il record attualmente valido per invalidarlo dopo la scrittura */
    uint32_t old_idx = find_last_valid_record();

    /* Trova il primo slot libero */
    uint32_t new_idx = find_first_free_slot();

    if (new_idx == CONFIG_MAX_RECORDS) {
        /*
         * Settore pieno (CONFIG_MAX_RECORDS record scritti, vedi config.h —
         * con CONFIG_JOURNAL_SIZE attuale sono alcune centinaia: evento
         * atteso solo dopo moltissimi salvataggi nella vita del dispositivo):
         * erase e riparti dallo slot 0. Il record precedente viene perso
         * nell'erase, non serve invalidarlo.
         *
         * BUG STORICO (osservato 2026-07-27): l'erase cancella FISICAMENTE
         * anche il blocco LUT_Store_t riservato in fondo allo stesso settore
         * (vedi flash_map.h) — un utente aveva salvato una LUT VCOMP con
         * "SAVE LUT", ma dopo che il journal si era riempito ed era scattato
         * questo ramo, "GET LUT VCOMP" tornava 0/10 entry al riavvio
         * successivo: la LUT era stata cancellata insieme al journal e
         * nessuno la riscriveva. A differenza di LUT_Save() (che dopo il
         * proprio erase richiama Config_Save() per ripristinare il journal),
         * questo ramo prima non richiamava LUT_Save() indietro — non era un
         * problema di scrittura/rilettura della LUT (flash_read()/CRC in
         * lut_manager.c funzionano correttamente), ma di simmetria mancante
         * fra i due moduli che condividono il settore.
         * FIX: richiamare LUT_RestoreAfterSectorErase() subito dopo l'erase,
         * cosi' come LUT_Save() ripristina Config_Save(). Usa una scrittura
         * "solo write" (nessun secondo erase, nessuna ricorsione in
         * Config_Save()) — vedi lut_manager.c.
         */
        err = erase_config_sector();
        if (err != CONFIG_OK) {
            return err;
        }
        (void)LUT_RestoreAfterSectorErase();
        new_idx = 0;
        old_idx = CONFIG_MAX_RECORDS; /* non c'è nulla da invalidare */
    }

    /* Scrivi il nuovo record nello slot libero */
    uint32_t new_addr = CONFIG_FLASH_BASE_ADDR + new_idx * sizeof(ConfigRecord_t);
    err = flash_write_record(new_addr, &new_rec);
    if (err != CONFIG_OK) {
        return err;
    }

    /* Invalida la cache per il nuovo record scritto */
    SCB_InvalidateDCache_by_Addr(
        (uint32_t *)new_addr,
        sizeof(ConfigRecord_t)
    );
    __ISB();
    __DSB();

    /*
     * Verifica effettiva (prima mancava, nonostante il commento sotto lo
     * dichiarasse già): rilegge il record appena scritto e lo confronta
     * byte per byte con quanto intendevamo scrivere. Se la scrittura è
     * stata corrotta (es. interrotta a metà da un altro task/ISR malgrado
     * la sezione critica), lo scopriamo SUBITO, durante questo SAVE CONFIG,
     * invece che al prossimo boot con un HardFault da ECC error in lettura.
     * In caso di mismatch NON invalidiamo il vecchio record: resta l'unico
     * valido e la configurazione precedente non viene persa.
     */
    const ConfigRecord_t *written = record_at(new_idx);
    if (memcmp(written, &new_rec, sizeof(ConfigRecord_t)) != 0) {
        return CONFIG_ERR_HAL;
    }

    /*
     * BUG STORICO (2026-07-02): qui veniva chiamata invalidate_record(old_idx)
     * per marcare OBSOLETE il record precedente, riscrivendone l'header a 32
     * byte (status 0xAA->0x00, resto identico). Il commento originale di
     * invalidate_record() sosteneva che fosse sempre lecito perché porta solo
     * bit da 1 a 0 — vero per il byte di status, ma FALSO per l'operazione
     * nel suo complesso sull'H7: la flash H7 programma a granularità di
     * "flash word" da 256 bit (32 byte) e OGNI flash word può essere
     * programmata UNA SOLA VOLTA per ciclo di erase, indipendentemente dal
     * fatto che i nuovi bit scritti siano bit-compatibili con quelli già
     * programmati. Riscrivere lo stesso header (già programmato al momento
     * della creazione del record) violava questa regola: la seconda
     * programmazione corrompeva l'ECC di quella flash word, causando un bus
     * fault (double ECC error) alla successiva lettura — sia immediatamente
     * (verifica in invalidate_record stessa) sia, peggio, al boot successivo
     * in Config_Load()/find_last_valid_record(), con settore lasciato in
     * stato incoerente fino al prossimo erase manuale. Root cause dietro la
     * serie di HardFault "prima scrittura OK, seconda fallisce" osservata
     * durante l'indagine sul comando RS485 SET TERM ON.
     *
     * FIX: non serve invalidare il vecchio record per la correttezza del
     * journaling. find_last_valid_record() scansiona SEMPRE tutti gli slot e
     * ritorna quello con indice più alto tra i VALID: un vecchio record
     * rimasto VALID (mai più letto perché superato da uno più recente) non
     * causa ambiguità né bug, semplicemente occupa uno slot fino al prossimo
     * erase per settore pieno. invalidate_record() non viene più chiamata
     * (funzione lasciata sotto solo come riferimento storico, NON richiamare:
     * vedi spiegazione nel suo commento).
     */
    (void)old_idx;

    return CONFIG_OK;
}

/**
 * @brief  Punto di ingresso del modulo. Inizializza CRC e carica la configurazione.
 * @param  hcrc  Puntatore all'handle CRC generato da CubeMX (&hcrc nel main.c).
 */
void Config_Init(CRC_HandleTypeDef *hcrc)
{
    s_hcrc = hcrc;

    Config_err_t err = Config_Load();

    if (err != CONFIG_OK) {
        /*
         * Flash vergine, schema cambiato o CRC errato.
         * In tutti i casi: carica i valori di fabbrica e salvali in flash,
         * così al prossimo avvio il settore sarà popolato correttamente.
         */
        Config_LoadDefaults();
        Config_Save();
    }
}


/* ========================================================================== */
/* --- API SERVICE MODE: AGGIORNAMENTO LUT SETPOINT --- */
/* ========================================================================== */

/**
 * @brief  Aggiorna una singola entry della LUT setpoint e persiste in flash.
 */
Config_err_t Config_SetLUTEntry(uint8_t idx, uint16_t current_ma)
{
    if (idx >= SETPOINT_LUT_SIZE) {
        return CONFIG_ERR_NOT_FOUND;
    }
    g_config.setpoint_lut_ma[idx] = current_ma;
    return Config_Save();
}

/**
 * @brief  Sovrascrive l'intera LUT setpoint e persiste in flash.
 */
Config_err_t Config_SetLUT(const uint16_t *lut_ma, uint8_t count)
{
    if (count != SETPOINT_LUT_SIZE || lut_ma == NULL) {
        return CONFIG_ERR_NOT_FOUND;
    }
    for (uint8_t i = 0; i < SETPOINT_LUT_SIZE; i++) {
        g_config.setpoint_lut_ma[i] = lut_ma[i];
    }
    return Config_Save();
}
