/*
 * ota_update_trigger.h (MMC, applicativo)
 *
 * Le due costanti sotto DEVONO restare identiche a OTA_BKP_REGISTER/
 * OTA_BKP_MAGIC in MMC_Bootloader/App/OTA/ota_config.h - progetti CubeIDE
 * separati (nessuna libreria condivisa), quindi nessun modo di forzare la
 * coerenza a compile-time: se una delle due cambia, va cambiata anche
 * l'altra a mano.
 *
 * Meccanismo (vedi ota_strategy_analysis_2026-07-17.md §5.2 e §20.6):
 * il registro di backup RTC sopravvive a qualunque reset che non tocchi il
 * dominio backup (software reset incluso - esattamente il caso qui: NVIC_
 * SystemReset() chiamato subito dopo). L'applicativo scrive OTA_BKP_MAGIC
 * qui prima di resettarsi (comando RS485 "FW UPDATE", vedi rs485_cmd.c);
 * il bootloader, al boot successivo, lo legge e lo consuma (Ota_Backup
 * CheckAndClearUpdateFlag() in ota_backup.c) per decidere di aprire una
 * finestra di ascolto RS485 molto piu' ampia di quella normale, invece del
 * breve grace window pensato per un power-on fisico.
 */
#ifndef APP_CONFIG_OTA_UPDATE_TRIGGER_H_
#define APP_CONFIG_OTA_UPDATE_TRIGGER_H_

#define OTA_TRIGGER_BKP_REGISTER   RTC_BKP_DR0
#define OTA_TRIGGER_BKP_MAGIC      0x0A17DA7EUL

#endif /* APP_CONFIG_OTA_UPDATE_TRIGGER_H_ */
