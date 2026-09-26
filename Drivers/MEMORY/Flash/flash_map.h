/*
 * flash_map.h
 *
 * Mappa dei settori della flash interna STM32H723ZGT6.
 * Questo file è l'unico punto in cui sono definiti gli indirizzi fisici;
 * nessun altro modulo deve contenere magic numbers di indirizzo.
 *
 * Layout H723 (1 MB, bank singolo):
 *   Sector 0 : 0x08000000 - 0x0801FFFF  (128 KB)  Firmware
 *   Sector 1 : 0x08020000 - 0x0803FFFF  (128 KB)  Firmware
 *   Sector 2 : 0x08040000 - 0x0805FFFF  (128 KB)  Firmware
 *   Sector 3 : 0x08060000 - 0x0807FFFF  (128 KB)  Firmware
 *   Sector 4 : 0x08080000 - 0x0809FFFF  (128 KB)  Firmware
 *   Sector 5 : 0x080A0000 - 0x080BFFFF  (128 KB)  Firmware
 *   Sector 6 : 0x080C0000 - 0x080DFFFF  (128 KB)  Firmware
 *   Sector 7 : 0x080E0000 - 0x080FFFFF  (128 KB)  CONFIG  <--
 *
 * Riferimento: RM0468 Rev4, Table 7. Flash memory sector address.
 * ATTENZIONE: H723ZGT6 ha 1MB Flash (0x08000000-0x080FFFFF), single-bank.
 * NON confondere con H743 (2MB dual-bank) dove il sector 7 di bank2 = 0x081E0000.
 */

#ifndef MEMORY_FLASH_FLASH_MAP_H_
#define MEMORY_FLASH_FLASH_MAP_H_

/* --- Settore riservato alla configurazione persistente --- */
#define CONFIG_FLASH_SECTOR       FLASH_SECTOR_7
#define CONFIG_FLASH_BASE_ADDR    0x080E0000UL
#define CONFIG_FLASH_SIZE         (128UL * 1024UL)   /* 131072 bytes */
#define CONFIG_FLASH_END_ADDR     (CONFIG_FLASH_BASE_ADDR + CONFIG_FLASH_SIZE)

/* Granularità minima di scrittura dell'H7: 256 bit = 32 byte.
 * Ogni struttura scritta in flash DEVE essere allineata e dimensionata
 * a multipli di questa costante. */
#define FLASH_WRITE_GRANULARITY   32U

/*
 * --- CONDIVISIONE DEL SETTORE 7 CON LUT_Store_t (lut_manager.c) ---
 *
 * L'H723ZGT6 e' single-bank, 1 MB, 8 settori da 128 KB: tutti gli altri
 * settori (0-6) sono occupati dal firmware, quindi il journal di
 * configurazione (config.c) e la LUT di calibrazione PD (lut_manager.c)
 * DEVONO condividere lo stesso, unico settore 7. L'erase minimo dell'H7 e'
 * per SETTORE INTERO, non per sotto-regione: i due blocchi non possono
 * scegliere offset indipendentemente, altrimenti il journal (che scrive
 * record incrementali via program, senza erase, finche' non e' pieno)
 * finisce per scrivere sopra byte gia' programmati da LUT_Save() (o
 * viceversa), producendo un programming error o dati corrotti.
 *
 * BUG STORICO (2026-07-02): LUT_FLASH_ADDR era fissato a
 * CONFIG_FLASH_BASE_ADDR+256, calcolato come "dopo i 192 byte di Config_t".
 * Errore: il journal non scrive Config_t nudo, ma ConfigRecord_t (header
 * 32 byte + Config_t = 224 byte), su PIU' slot successivi. +256 cadeva
 * quindi dentro il payload del SECONDO record (byte [224,448)), cioè
 * esattamente nel primo slot che il journal riusa dopo il primo salvataggio.
 * Il comando RS485 "SET TERM ON" (secondo Config_Save() mai eseguito su
 * quel dispositivo) ha scritto sopra i byte già programmati da una LUT
 * salvata in precedenza: blocco del micro (HAL_FLASH_Program bloccato in
 * attesa su un bit non cancellabile, con interrupt mascherati dalla
 * sezione critica) -> reset forzato da IWDG a metà operazione -> settore
 * lasciato in stato incoerente -> HardFault nei boot successivi in
 * Config_Init() (ECC error in lettura). Richiesto erase manuale del
 * settore 7 per ripristinare uno stato pulito.
 *
 * FIX: riserviamo un blocco FISSO in fondo al settore, esclusivamente per
 * LUT_Store_t, e riduciamo di conseguenza lo spazio utilizzabile dal
 * journal Config (CONFIG_MAX_RECORDS in config.h). I due blocchi non
 * potranno mai più sovrapporsi, qualunque sia il numero di record scritti.
 * lut_manager.c verifica a compile-time (_Static_assert) che
 * sizeof(LUT_Store_t) non superi questa riserva.
 *
 * Layout risultante del settore 7:
 *   [0                 .. CONFIG_JOURNAL_SIZE)  : journal ConfigRecord_t (config.c)
 *   [CONFIG_JOURNAL_SIZE .. CONFIG_FLASH_SIZE)   : LUT_Store_t fisso     (lut_manager.c)
 *
 * NOTA aggiuntiva: qualunque erase del settore 7 (da config.c quando il
 * journal si riempie, o da lut_manager.c ad ogni LUT_Save()) cancella
 * FISICAMENTE entrambi i blocchi. lut_manager.c richiama Config_Save()
 * subito dopo il proprio erase per ripristinare il record Config corrente;
 * il caso opposto (journal Config pieno, evento atteso solo dopo ~500+
 * salvataggi) rimane una limitazione nota e a bassissima probabilità.
 */
#define LUT_RESERVED_SIZE          (4UL * 1024UL)   /* >> sizeof(LUT_Store_t), margine per crescita futura */
#define CONFIG_JOURNAL_SIZE        (CONFIG_FLASH_SIZE - LUT_RESERVED_SIZE)
#define LUT_FLASH_BASE_ADDR        (CONFIG_FLASH_BASE_ADDR + CONFIG_JOURNAL_SIZE)

#endif /* MEMORY_FLASH_FLASH_MAP_H_ */
