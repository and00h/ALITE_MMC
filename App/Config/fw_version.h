/*
 * fw_version.h (MMC, applicativo)
 *
 * Versione e revisione del firmware applicativo, esposte via RS485 dal
 * comando "GET FW" (rs485_cmd.c) - stesso schema del comando "VER" del
 * bootloader (MMC_Bootloader/App/OTA/ota_ascii.c, OTA_BOOTLOADER_VERSION).
 */
#ifndef APP_CONFIG_FW_VERSION_H_
#define APP_CONFIG_FW_VERSION_H_

/* Versione applicativa: stringa libera, da incrementare A MANO a ogni
 * release significativa (nessuna injection automatica da build system -
 * vedi anche il commento su g_config.fw_version in config.c, che resta un
 * campo persistito separato, pensato per un meccanismo di injection futuro
 * mai completato, e quindi NON affidabile per riportare la versione del
 * binario realmente in esecuzione: un firmware piu' vecchio puo' aver
 * lasciato in flash un valore stantio). Questa stringa, al contrario, e'
 * sempre coerente con l'.elf/.bin realmente compilato e flashato. */
/*
 * 1.0.1 (2026-08-01): reporting del setpoint HW (pin PC5/LPWR_SET_ISO,
 * ADC2) su "GET STATUS" (RS485) e sulla risposta STATUS della COM
 * interface, convertito in % tramite nuova LUT di calibrazione dedicata
 * (LUT_ConvertHwPowerToPct(), lut_manager.h — comando "SET/GET LUT HWPWR").
 * Attivo solo quando FSM_GetLaserModeWire() != 0 (ANALOG, o SW con
 * SETPOINT HW attivo); comportamento invariato in tutti gli altri casi.
 * Nessuna modifica al percorso di controllo reale verso LaseQ/AMC.
 */
#define FW_VERSION_STR       "1.1.0"

/* Revisione di build: data/ora di compilazione, iniettata automaticamente
 * dal preprocessore C ad ogni build (nessun passo manuale) - utile per
 * distinguere due binari con la stessa FW_VERSION_STR ma compilati in
 * momenti diversi durante lo sviluppo, prima di un bump di versione
 * formale. */
#define FW_BUILD_DATE_STR    (__DATE__ " " __TIME__)

#endif /* APP_CONFIG_FW_VERSION_H_ */
