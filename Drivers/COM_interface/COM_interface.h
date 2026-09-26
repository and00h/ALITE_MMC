/*
 * COM_interface.h
 *
 * Driver di collegamento verso il modulo COM interface (scheda STM32H7
 * dedicata a Ethernet/WebUI, vedi istruzioni di progetto), via SPI3
 * condivisa con la SD card (vedi Drivers/SPI3Bus/SPI3Bus.h). Responsabilità
 * di QUESTO modulo (lato MMC):
 *   1. Sorvegliare l'alimentazione della COM interface (nCOM_PWR_FLT) e
 *      segnalare i fault alla FSM.
 *   2. Implementare il TRASPORTO a lunghezza fissa su SPI3 descritto sotto,
 *      riallineato (2026-07-23) al framing gia' usato da ALITE_COM (256B,
 *      0xAA/0x5A, CRC8 — vedi COM_interface_protocol.h) per poter eseguire
 *      il test SPI periodico (COM_MSG_TEST, echo del payload). Il CONTENUTO
 *      applicativo dei messaggi STATUS/CONTROL/CONFIG non e' invece ancora
 *      definito: per quei tipi si scambia ancora un frame placeholder
 *      (COM_MSG_EMPTY).
 *
 * HARDWARE (vedi main.h):
 *   COM_SPI_nCS      PD0   GPIO_Output — chip select SPI3 verso COM interface
 *                          (SPI3 e' configurata NSS software: il CS va
 *                          pilotato a mano attorno a ogni transazione, come
 *                          fa gia' sd_spi.c per la SD — vedi cs_low()/
 *                          cs_high() in COM_interface.c)
 *   nCOM_INT_IN      PD1   GPIO_IT_FALLING (vedi MX_GPIO_Init/main.c e
 *                          MMC.ioc: PD1 idle-high, la COM interface lo porta
 *                          basso per segnalare un messaggio pronto — vedi
 *                          COM_assert_int_out()/COM_clear_int_out() in
 *                          ALITE_COM Core/Src/ALITE/com/com_spi.c). NON e'
 *                          piu' un heartbeat/keepalive (revisione
 *                          2026-07-17): ogni fronte di discesa avvia la
 *                          sequenza di sincronismo sotto, nessun'altra
 *                          semantica. (Nota: i commenti precedenti di questo
 *                          file riportavano erroneamente "rising edge" — il
 *                          codice generato in main.c ha sempre usato
 *                          GPIO_MODE_IT_FALLING, corretto.)
 *   nCOM_INT_OUT     PD2   GPIO_Output — riservato, non ancora usato
 *   COM_PWR_GOOD     PD3   GPIO_Input  — alimentazione COM interface OK
 *   nCOM_PWR_FLT     PD4   GPIO_IT_RISING — fault alimentazione/eFuse COM interface (attivo alto)
 *   nCOM_PWR_SHDN    PD5   GPIO_Output — shutdown alimentazione COM interface (MMC -> COM)
 *   SPI3 (hspi3)     condivisa con la scheda SD — vedi Drivers/SPI3Bus/SPI3Bus.h.
 *                    Data size 8-bit, CPOL=0/CPHA=0, sempre — vedi banner in
 *                    SPI3Bus.h (revisionato 2026-07-17: NON esiste piu' una
 *                    modalita' 4-bit per la COM interface, era un refuso).
 *
 * SEQUENZA DI SINCRONISMO MASTER/SLAVE (2026-07-17, revisione completa):
 *   Il MMC e' master SPI: la COM interface non puo' avviare da sola una
 *   trasmissione, deve prima "chiedere il turno" al MMC tramite nCOM_INT_IN.
 *
 *   1. La COM interface abbassa nCOM_INT_IN (fronte di discesa, EXTI1)
 *      quando ha un messaggio pronto per il MMC.
 *   2. ISR (COM_Interface_MsgReady_EXTI_Callback, chiamata da
 *      HAL_GPIO_EXTI_Callback): se e' gia' in corso uno scambio (s_busy),
 *      il fronte viene IGNORATO (nessuna coda: la COM interface e' tenuta a
 *      ripresentare la richiesta se necessario). Altrimenti marca s_busy e
 *      sveglia (tramite callback registrata) il task dedicato
 *      (task_com_interface.c) — nessuna chiamata bloccante in ISR.
 *   3. Il task, fuori da contesto ISR, acquisisce il mutex SPI3
 *      (SPI3Bus_Acquire) e invia un frame "vuoto" (COM_MSG_EMPTY, tutto a
 *      zero): il contenuto trasmesso NON ha significato, serve solo a
 *      generare il clock SPI. Essendo SPI full-duplex, la COM interface
 *      mette contemporaneamente sul bus la sua richiesta reale, catturata
 *      in RX nella STESSA transazione DMA (COM_Interface_BeginRequest()).
 *   4. Il task legge/valida la richiesta (COM_Interface_GetRequestFrame())
 *      e invia una seconda transazione con la risposta reale
 *      (COM_Interface_SendResponse()): per COM_MSG_TEST il payload ricevuto
 *      viene ripetuto identico (vedi task_com_interface.c); per gli altri
 *      tipi, protocollo applicativo non ancora definito, si risponde
 *      COM_MSG_EMPTY. Prima di inviare la risposta il task attende un
 *      breve delay (vedi COM_RESPONSE_DELAY_TICKS in task_com_interface.c) per
 *      dare tempo alla COM interface di riarmare la propria
 *      HAL_SPI_Receive_DMA() lato slave (vedi TxCpltCallback in
 *      ALITE_COM/com_spi.c) prima che il MMC generi il clock della seconda
 *      transazione.
 *   5. Il task chiude lo scambio (COM_Interface_EndExchange()): rilascia il
 *      mutex SPI3 e azzera s_busy, pronto per il prossimo fronte.
 *
 *   NOTA TASK DEDICATO: la gestione di cui sopra vive in un task FreeRTOS
 *   SEPARATO (task_com_interface.c), NON nel loop di task_comms.c (che serve
 *   LaseQ/AMC con un proprio periodo fisso) — un'attesa bloccante su un
 *   semaforo alimentato da un evento asincrono e sporadico (la COM interface
 *   scrive solo quando ha qualcosa di nuovo, non periodicamente) non deve
 *   condividere il ciclo di un altro protocollo. Per lo stesso motivo questo
 *   task NON e' monitorato dal watchdog software (Watchdog.h): puo' restare
 *   bloccato in attesa per tempo indefinito senza che sia un sintomo di
 *   malfunzionamento.
 *
 * RILEVAMENTO FAULT ALIMENTAZIONE (invariato):
 *   nCOM_PWR_FLT (EXTI4, rising): fault immediato — COM_Interface_PwrFlt_EXTI_Callback()
 *   marca subito il fault; COM_Interface_Update() (chiamata periodicamente da
 *   task_comms, indipendentemente dal ciclo AMC) lo consuma e pubblica
 *   SYS_COM_FAULT_EVENT (gated su g_config.fault_mask), una sola volta finche'
 *   la condizione non rientra (stesso pattern lq_fault_posted/amc_fault_posted
 *   di task_comms.c). Dal 2026-07-17 questa e' l'UNICA sorgente di
 *   SYS_COM_FAULT_EVENT: non esiste piu' un fault da "heartbeat mancante" su
 *   nCOM_INT_IN, che ora e' solo il segnale "messaggio pronto" del punto sopra.
 *
 * USO:
 *   // Da task_com_interface.c (TaskComInterface_Init(), chiamata da
 *   // MX_FREERTOS_Init() prima dello scheduler, dopo SPI3Bus_CreateMutex()):
 *   COM_Interface_Init(&hspi3, OnMsgPending, OnXferCplt);
 *
 *   // Da HAL_GPIO_EXTI_Callback() (stm32h7xx_it.c):
 *   if (GPIO_Pin == nCOM_INT_IN_Pin)   { COM_Interface_MsgReady_EXTI_Callback(); }
 *   if (GPIO_Pin == nCOM_PWR_FLT_Pin)  { COM_Interface_PwrFlt_EXTI_Callback(); }
 *
 *   // Da HAL_SPI_TxRxCpltCallback()/HAL_SPI_ErrorCallback() (stm32h7xx_it.c),
 *   // ramo hspi->Instance == SPI3:
 *   COM_Interface_SPI_Cplt_Callback();   // in TxRxCpltCallback
 *   COM_Interface_SPI_Error_Callback();  // in ErrorCallback
 *
 *   // Da task_comms.c, periodicamente (solo fault alimentazione, non piu'
 *   // legato al ciclo AMC):
 *   COM_Interface_Update();
 */

#ifndef DRIVERS_COM_INTERFACE_COM_INTERFACE_H_
#define DRIVERS_COM_INTERFACE_COM_INTERFACE_H_

#include "stm32h7xx_hal.h"
#include "COM_interface_protocol.h"
#include <stdbool.h>
#include <stdint.h>

/* ========================================================================== */
/* --- API INIZIALIZZAZIONE --- */
/* ========================================================================== */

/**
 * @brief  Inizializza il driver: salva l'handle SPI3, registra le callback
 *         applicative e resetta stato/fault/busy. Non tocca ancora il bus
 *         SPI3 (nessuna transazione finche' non arriva un fronte su
 *         nCOM_INT_IN).
 * @param  hspi3         Handle SPI3 (da hal_handles.h), condiviso con la SD —
 *                       vedi Drivers/SPI3Bus/SPI3Bus.h per l'arbitraggio.
 * @param  on_msg_pending Callback chiamata da ISR (COM_Interface_MsgReady_EXTI_Callback)
 *                       quando arriva un fronte su nCOM_INT_IN e non e' gia'
 *                       in corso uno scambio. Deve limitarsi a svegliare il
 *                       task dedicato (es. osSemaphoreRelease), nessuna
 *                       chiamata bloccante. Puo' essere NULL solo in test.
 * @param  on_xfer_cplt  Callback chiamata da COM_Interface_SPI_Cplt_Callback()
 *                       (contesto ISR, sia su completamento OK che su
 *                       errore DMA) per sbloccare l'attesa del task sulla
 *                       singola transazione in corso. Stesso schema di
 *                       LaseQ_Init()/on_tx_cplt-on_rx_cplt.
 */
void COM_Interface_Init(SPI_HandleTypeDef *hspi3,
                         void (*on_msg_pending)(void),
                         void (*on_xfer_cplt)(void));

/* ========================================================================== */
/* --- CALLBACK ISR --- */
/* ========================================================================== */

/**
 * @brief  Da richiamare da HAL_GPIO_EXTI_Callback() su nCOM_INT_IN (EXTI1,
 *         rising edge). Contesto ISR. Se e' gia' in corso uno scambio
 *         (busy), il fronte viene ignorato (nessuna coda/ri-arm automatico:
 *         la COM interface deve ripresentare la richiesta). Altrimenti marca
 *         busy e invoca la callback on_msg_pending registrata in
 *         COM_Interface_Init(). Nessuna chiamata bloccante.
 */
void COM_Interface_MsgReady_EXTI_Callback(void);

/**
 * @brief  Da richiamare da HAL_GPIO_EXTI_Callback() su nCOM_PWR_FLT (EXTI4,
 *         rising edge). Contesto ISR: marca il fault immediato (letto e
 *         processato dal prossimo COM_Interface_Update()); nessuna
 *         Queue_PostEvent() diretta dall'ISR.
 */
void COM_Interface_PwrFlt_EXTI_Callback(void);

/**
 * @brief  Da richiamare da HAL_SPI_TxRxCpltCallback() (stm32h7xx_it.c) quando
 *         hspi->Instance == SPI3. Contesto ISR: marca l'ultima transazione
 *         come riuscita e invoca la callback on_xfer_cplt registrata in
 *         COM_Interface_Init() per sbloccare l'attesa del task dedicato.
 */
void COM_Interface_SPI_Cplt_Callback(void);

/**
 * @brief  Da richiamare da HAL_SPI_ErrorCallback() (stm32h7xx_it.c) quando
 *         hspi->Instance == SPI3. Contesto ISR: marca l'ultima transazione
 *         come fallita (vedi COM_Interface_LastXferOk()) e invoca comunque
 *         on_xfer_cplt per sbloccare l'attesa del task dedicato, che deve
 *         poi verificare l'esito prima di proseguire — stesso schema di
 *         LaseQ_ErrorCallback() in LaseQ.c.
 */
void COM_Interface_SPI_Error_Callback(void);

/* ========================================================================== */
/* --- SCAMBIO DATI (da chiamare SOLO da task_com_interface.c, in sequenza) --- */
/* ========================================================================== */

/**
 * @brief  Avvia il primo passo dello scambio: acquisisce il mutex SPI3
 *         (bloccante, osWaitForever — chiamare SOLO da contesto task),
 *         riconfigura esplicitamente hspi3 (8-bit/CPOL0/CPHA0/NSS software,
 *         prescaler dedicato) per difendersi da un'eventuale riconfigurazione
 *         lasciata dal driver SD (sd_spi.c altera DataSize/NSSPMode/prescaler
 *         a runtime sullo stesso hspi3 condiviso), abbassa COM_SPI_nCS e avvia
 *         via DMA l'invio di un frame COM_MSG_EMPTY (tutto a zero, solo
 *         "clock") mentre cattura in RX la richiesta reale della COM
 *         interface (full-duplex, singola transazione).
 *         Il chiamante deve poi attendere la callback di completamento
 *         (on_xfer_cplt) prima di leggere il risultato con
 *         COM_Interface_GetRequestFrame().
 * @retval true   DMA avviata correttamente.
 * @retval false  Errore HAL immediato: mutex gia' rilasciato e busy azzerato
 *                internamente, il chiamante puo' semplicemente ripartire
 *                dal prossimo fronte (nessun cleanup aggiuntivo richiesto).
 */
bool COM_Interface_BeginRequest(void);

/**
 * @brief  Da chiamare dopo che on_xfer_cplt ha sbloccato l'attesa relativa
 *         a COM_Interface_BeginRequest(). Invalida la D-Cache sul buffer RX
 *         e valida il frame ricevuto (START/STOP/CRC — COM_FrameValidate()).
 * @retval Puntatore al frame ricevuto (COM_FRAME_SIZE byte, layout
 *         COM_Frame_t) se valido, NULL se il frame non ha superato la
 *         validazione o se l'ultima transazione e' terminata in errore
 *         (vedi COM_Interface_LastXferOk()). Il puntatore resta valido solo
 *         fino alla prossima COM_Interface_BeginRequest().
 */
const uint8_t *COM_Interface_GetRequestFrame(void);

/**
 * @brief  Secondo passo dello scambio: costruisce un frame con msg_type e
 *         payload indicati, lo finalizza (CRC+STOP) e lo invia via DMA
 *         (RX di questa transazione scartata). Da chiamare SOLO dopo un
 *         COM_Interface_BeginRequest() andato a buon fine. Il chiamante deve
 *         attendere di nuovo on_xfer_cplt prima di chiamare
 *         COM_Interface_EndExchange().
 * @param  msg_type  Tipo di messaggio (COM_MsgType_t) — oggi solo COM_MSG_EMPTY.
 * @param  payload   COM_PAYLOAD_SIZE byte di payload, oppure NULL per un
 *                   payload a zero (caso COM_MSG_EMPTY).
 * @retval true   DMA avviata correttamente.
 * @retval false  Errore HAL immediato — il chiamante deve comunque chiamare
 *                COM_Interface_EndExchange() per rilasciare mutex/busy.
 */
bool COM_Interface_SendResponse(COM_MsgType_t msg_type, const uint8_t *payload);

/**
 * @brief  Chiude lo scambio: rilascia il mutex SPI3 e azzera il flag busy,
 *         pronto per il prossimo fronte su nCOM_INT_IN. Da chiamare SEMPRE
 *         a fine sequenza (successo, errore o timeout) tranne nel caso in
 *         cui COM_Interface_BeginRequest() abbia gia' restituito false (in
 *         quel caso il cleanup e' gia' stato fatto internamente).
 */
void COM_Interface_EndExchange(void);

/**
 * @brief  Abortisce una transazione DMA in corso (timeout lato task) e
 *         chiude lo scambio (equivalente a HAL_SPI_Abort() + EndExchange()).
 *         Da chiamare se l'attesa di on_xfer_cplt scade.
 */
void COM_Interface_AbortAndEnd(void);

/**
 * @brief  Esito dell'ultima transazione DMA (impostato dalla callback
 *         COM_Interface_SPI_Cplt_Callback()): true se completata senza
 *         errori HAL, false se HAL_SPI_ErrorCallback e' stata invocata.
 */
bool COM_Interface_LastXferOk(void);

/* ========================================================================== */
/* --- SORVEGLIANZA ALIMENTAZIONE (invariata) --- */
/* ========================================================================== */

/**
 * @brief  Avanzamento periodico (chiamare da task_comms.c). Controlla il
 *         fault immediato da nCOM_PWR_FLT, aggiorna
 *         TaskMonitor_SetFaultBit(FAULT_BIT_COM, ...) e pubblica
 *         SYS_COM_FAULT_EVENT (gated su g_config.fault_mask) finche' il
 *         fault resta attivo. Dal 2026-07-17 NON gestisce piu' alcun
 *         heartbeat/miss-count: nCOM_INT_IN e' ora solo il segnale
 *         "messaggio pronto" gestito dal task dedicato.
 */
void COM_Interface_Update(void);

/**
 * @brief  true se al momento e' attivo un fault di alimentazione COM interface
 *         (nCOM_PWR_FLT asserito). Utile per telemetria RS485.
 */
bool COM_Interface_IsFaultActive(void);

#endif /* DRIVERS_COM_INTERFACE_COM_INTERFACE_H_ */
