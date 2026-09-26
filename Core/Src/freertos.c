/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "queues.h"
#include "fsm_events.h"
#include "config.h"
#include "task_comms.h"
#include "task_amc.h"
#include "task_monitor.h"
#include "task_fsm.h"
#include "task_outputs.h"
#include "task_inputs.h"
#include "task_rs485.h"
#include "hal_handles.h"
#include "eFuse.h"
#include "SAB.h"
#include "BoardCtrl.h"
#include "AMC.h"
#include "sys_log.h"
#include "Watchdog.h"
#include "string.h"
#include "SPI3Bus.h"
#include "COM_interface.h"
#include "task_com_interface.h"
#include "QCW.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/*
 * Popolato da vApplicationStackOverflowHook() (sotto): nome del task e
 * relativo handle al momento dell'overflow rilevato. Ispezionabile nel
 * debugger (Expressions: g_ovfTaskName / g_ovfTaskHandle) quando si ferma
 * su BKPT. Vedi anche configCHECK_FOR_STACK_OVERFLOW in FreeRTOSConfig.h.
 */
volatile char        g_ovfTaskName[configMAX_TASK_NAME_LEN];
volatile TaskHandle_t g_ovfTaskHandle;

/* USER CODE END Variables */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* Callback SAB: traduce eventi driver in eventi FSM.
 *
 * SAB_EVT_ARMED non genera un evento FSM: action_enter_enabled() rileva
 * l'armo direttamente interrogando SAB_GetState() nel suo loop di polling.
 * Gli altri eventi (fault) sono postati alla coda e processati dalla FSM
 * quando action_enter_enabled() ritorna (stato già SYS_ENABLED a quel punto,
 * la FSM transita poi in SYS_ERROR).
 *
 * MASCHERABILITÀ (g_config.error_mask): il log su SD resta SEMPRE scritto
 * (traccia diagnostica indipendente dalla maschera); solo il post
 * dell'evento FSM è condizionato al bit ERR_BIT_SAB_x — a differenza di
 * FLOOD/FLOW/TEMP/PSU (gestiti in task_monitor.c), questi bit prima non
 * erano mai controllati da nessuna parte: la maschera non aveva alcun
 * effetto sui fault SAB.
 *
 * DISCRIMINAZIONE CANALE A/B (dal 2026-07-15): SAB_EVT_INTERLOCK_OPEN e
 * SAB_EVT_TEST_FAIL restano eventi generici (SAB.h/SAB.c non distinguono
 * il canale nella callback), ma qui il canale specifico si ricostruisce
 * con SAB_GetInterlockStatus()/SAB_GetTestStatus() e viene mascherato
 * INDIPENDENTEMENTE per canale (ERR_BIT_SAB_INTLCK_A/B,
 * ERR_BIT_SAB_TEST_A/B — vedi task_monitor.h).
 *
 * ATTENZIONE — INTLCK vs TEST (rivisto 2026-07-16): i due canali INTLCK
 * sono la STESSA catena di sicurezza duplicata per ridondanza (non varianti
 * di prodotto come KEY_A/B o LID1/2): mascherare un canale INTLCK REALMENTE
 * cablato sopprime solo la notifica verso la FSM di sistema, MAI la
 * risposta di sicurezza del modulo SAB (che va comunque in SAB_STATE_FAULT
 * su qualsiasi canale interlock, indipendentemente da error_mask) — usare
 * la maschera INTLCK solo per banco prova/debug a canale singolo. I canali
 * TEST invece, da questo bugfix, sono già filtrati a monte da
 * test_pins_ok() (SAB.c): un canale TEST mascherato non genera nemmeno più
 * SAB_EVT_TEST_FAIL per quel canale, quindi il controllo error_mask qui
 * sotto per TEST_A/B è ormai ridondante (ma innocuo) rispetto a quello
 * interno al driver — lasciato per simmetria col ramo INTLCK e perché
 * SAB_GetTestStatus() resta comunque una lettura RAW non mascherata.
 */
static void sab_event_cb(SAB_Evt_t evt)
{
    switch (evt) {
        case SAB_EVT_ARMED:
            /* Rilevato dal polling in action_enter_enabled(): nessuna azione */
            break;
        case SAB_EVT_TIMEOUT: {
            /*
             * BUGFIX (Luca, 2026-07-16): a differenza di SAB_EVT_INTERLOCK_OPEN
             * sotto, questo log non riportava MAI quale canale (A/B) non si
             * era chiuso entro il timeout — utile in banco per distinguere
             * subito "che interlock è aperto" senza dover interrogare anche
             * "GET SAB". Stesso dato (SAB_GetInterlockStatus()), stessa
             * causa fisica di SAB_EVT_INTERLOCK_OPEN, solo rilevata in fase
             * di armo (SAB_STATE_ENABLING) anziché dopo (SAB_STATE_ARMED).
             */
            bool a_open, b_open;
            SAB_GetInterlockStatus(&a_open, &b_open);
            SysLog_Event(LOG_ERROR,
                         "SAB: interlock timeout (A:%s B:%s)",
                         a_open ? "OPEN" : "CLOSED",
                         b_open ? "OPEN" : "CLOSED");
            if (g_config.error_mask & ERR_BIT_SAB_TIMEOUT) {
                Queue_PostEvent(SYS_SAB_TIMEOUT_EVENT);
            }
            break;
        }
        case SAB_EVT_INTERLOCK_OPEN: {
            bool a_open, b_open;
            SAB_GetInterlockStatus(&a_open, &b_open);

            if (a_open) {
                SysLog_Event(LOG_ERROR, "SAB: interlock open (channel A)");
                if (g_config.error_mask & ERR_BIT_SAB_INTLCK_A) {
                    Queue_PostEvent(SYS_SAB_INTERLOCK_OPEN_EVENT);
                }
            }
            if (b_open) {
                SysLog_Event(LOG_ERROR, "SAB: interlock open (channel B)");
                if (g_config.error_mask & ERR_BIT_SAB_INTLCK_B) {
                    Queue_PostEvent(SYS_SAB_INTERLOCK_OPEN_EVENT);
                }
            }
            break;
        }
        case SAB_EVT_TEST_FAIL: {
            bool a_fault, b_fault;
            SAB_GetTestStatus(&a_fault, &b_fault);

            if (a_fault) {
                SysLog_Event(LOG_ERROR, "SAB: test signal fault (channel A)");
                if (g_config.error_mask & ERR_BIT_SAB_TEST_A) {
                    Queue_PostEvent(SYS_SAB_TEST_FAIL_EVENT);
                }
            }
            if (b_fault) {
                SysLog_Event(LOG_ERROR, "SAB: test signal fault (channel B)");
                if (g_config.error_mask & ERR_BIT_SAB_TEST_B) {
                    Queue_PostEvent(SYS_SAB_TEST_FAIL_EVENT);
                }
            }
            break;
        }
        default:
            break;
    }
}

/*
 * Sys_HwInit — init hardware che usa HAL blocking (SPI, I2C, ADC, UART, SD).
 *
 * NON chiamare da MX_FREERTOS_Init (pre-scheduler): i timeout HAL (basati su
 * TIM1) non sono affidabili prima di osKernelStart().
 * Viene chiamata come prima istruzione di TaskMonitor_Run (priorità Normal5,
 * la più alta), garantendo che l'HW sia pronto prima che task a priorità
 * inferiore (FSM, Comms, RS485 …) ricevano il loro primo time-slice.
 */
void Sys_HwInit(void)
{
    /*
     * AD7490_Init()/SHT35_Init() NON vengono chiamate qui: richiedono i
     * function pointer wait/signal la cui implementazione (basata su
     * semafori CMSIS-RTOS) è di competenza del layer applicativo, non di
     * questo file di init HW. Vengono chiamate da TaskMonitor_Run() in
     * task_monitor.c, subito dopo Sys_HwInit() — stesso pattern già usato da
     * LaseQ_Init() in task_comms.c.
     */
    EFuse_Init(&hadc1, &hadc3);
    SAB_Init((uint16_t)g_config.sab_interlock_timeout_ms, sab_event_cb);
    AMC_Init(&huart6);      /* USART6 → AMC (STM32G473) */
    /* COM_Interface_Init() NON viene piu' chiamata qui dal 2026-07-17: lo fa
     * TaskComInterface_Init() (MX_FREERTOS_Init(), sotto), che le passa le
     * proprie callback OnMsgPending/OnXferCplt — stesso schema di
     * LaseQ_Init() chiamata da TaskComms_Init(). Nessuna chiamata HAL
     * bloccante in COM_Interface_Init(): sicura anche pre-scheduler. */
    SysLog_Init();          /* Mount FatFS, apre file di log del giorno */
    SysLog_Boot();          /* Prima riga "System boot" */

    /*
     * QCW_Init(): configura TIM16 a livello di registro (non gestito da
     * CubeMX/.ioc — periferica libera, vedi Drivers/QCW/QCW.h). Deve girare
     * DOPO osKernelStart() come gli altri init in Sys_HwInit() (HAL_GetTick()
     * affidabile solo a scheduler avviato); non avvia il timer (QCW_Start()
     * lo fa solo su ingresso in SYS_EMISSION con QCW abilitato).
     */
    QCW_Init();

    /*
     * Watchdog software (heartbeat per task) sopra IWDG1 hardware.
     * Watchdog_LogResetCause() DEVE girare dopo SysLog_Init()/SysLog_Boot()
     * (altrimenti il log è no-op) e legge/cancella i flag RCC PRIMA che
     * qualunque altro reset li sovrascriva — è il primo consumatore di quei
     * flag nel firmware. Watchdog_Init() abilita il refresh condizionato
     * fatto da Watchdog_Service() nel default task (main.c, USER CODE 5).
     */
    Watchdog_LogResetCause();
    Watchdog_Init(&hiwdg1);
}

void MX_FREERTOS_Init(void)
{
    /* Solo init sicuri pre-scheduler: RAM e oggetti RTOS */
    Config_CreateFlashMutex(); /* mutex condiviso settore flash 7 (config.c/lut_manager.c) */
    SPI3Bus_CreateMutex();   /* mutex condiviso SPI3 (SD vs COM interface, vedi SPI3Bus.h) */
    Queues_Init();           /* coda eventi FSM */
    BoardCtrl_Init();        /* GPIO in stato sicuro */
    TaskComms_Init();        /* semafori (safe dopo osKernelInitialize) */
    TaskComInterface_Init(); /* semafori + COM_Interface_Init() (idem) */

    osThreadNew(TaskMonitor_Run,     NULL, &taskMonitor_attr);
    osThreadNew(TaskFSM_Run,         NULL, &taskFSM_attr);
    osThreadNew(TaskComms_Run,       NULL, &taskComms_attr);
    osThreadNew(TaskAmc_Run,         NULL, &taskAmc_attr);
    osThreadNew(TaskComInterface_Run, NULL, &taskComInterface_attr);
    osThreadNew(TaskOutputs_Run,     NULL, &taskOutputs_attr);
    osThreadNew(TaskInputs_Run,      NULL, &taskInputs_attr);
    osThreadNew(Task_RS485,          NULL, &taskRS485_attr);
}

/*
 * @brief  Chiamata da FreeRTOS (vedi configCHECK_FOR_STACK_OVERFLOW in
 *         FreeRTOSConfig.h) quando rileva che lo stack di un task ha
 *         superato il limite allocato. Salva il nome/handle del task in
 *         variabili globali ispezionabili nel debugger e ferma l'esecuzione:
 *         a questo punto lo stack del task è già corrotto, non ha senso
 *         proseguire.
 */
void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName)
{
    g_ovfTaskHandle = xTask;
    strncpy((char *)g_ovfTaskName, pcTaskName, configMAX_TASK_NAME_LEN - 1);
    g_ovfTaskName[configMAX_TASK_NAME_LEN - 1] = '\0';

    __asm volatile ("BKPT #02");
    for (;;) { }
}
/* USER CODE END Application */

