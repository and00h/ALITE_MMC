/*
 * QCW.h
 *
 * Modulazione impulsata (Quasi-Continuous Wave) di nGATE_MC in modalità
 * software (FSM_MODE_SW). Alternativa a CW (gate aperto staticamente per
 * tutta la durata di SYS_EMISSION, comportamento originale/default).
 *
 * HARDWARE:
 *   nGATE_MC (PE12, active LOW: LOW = laser acceso) — vedi BoardCtrl.h.
 *   PE12 è mappabile via AF come TIM1CH3N, ma TIM1 è riservato all'HAL come
 *   timebase di sistema (1ms tick, vedi stm32h7xx_hal_timebase_tim.c) e non
 *   può essere riconfigurato come timer PWM senza rompere FreeRTOS/HAL_Delay.
 *   Questo modulo usa quindi TIM16 (dal 2026-07-13 configurato in CubeMX/.ioc
 *   con il suo global interrupt attivo — handle `htim16` generato in main.c,
 *   vedi hal_handles.h) SOLO come sorgente di interrupt periodici: PE12 resta
 *   configurato nativamente come GPIO_Output "normale" (BoardCtrl_Init(),
 *   invariato) e viene pilotato via HAL_GPIO_WritePin() dentro l'interrupt
 *   di TIM16, non da un canale PWM hardware. Questo evita qualunque remap
 *   dinamico della funzione alternata del pin. Clock enable e NVIC restano
 *   di competenza di CubeMX (MX_TIM16_Init()/HAL_TIM_Base_MspInit(), non
 *   duplicati qui).
 *
 * PRINCIPIO DI FUNZIONAMENTO:
 *   QCW_Start() apre il gate (laser ON) e arma TIM16 per scadere dopo la
 *   durata "ON" (freq/duty correnti). Ad ogni scadenza (QCW_TimerCallback,
 *   chiamata da HAL_TIM_PeriodElapsedCallback), il pin viene invertito e il
 *   timer riarmato con la durata della fase opposta (ON<->OFF) — classico
 *   bit-banging a interrupt, nessun canale PWM hardware coinvolto.
 *
 * RANGE E PRESCALER DINAMICO (esteso 2026-07-20, 1Hz-50kHz):
 *   Frequenza: QCW_MIN_FREQ_HZ .. QCW_MAX_FREQ_HZ.
 *   Duty cycle: QCW_MIN_DUTY_PCT .. QCW_MAX_DUTY_PCT.
 *   Un prescaler FISSO non copre più questo range (a 1Hz un periodo intero
 *   sarebbe ~1M tick anche a 1MHz, ben oltre il limite a 16 bit di TIM16
 *   (ARR max 65535); a 50kHz un prescaler tarato per le basse frequenze
 *   darebbe una risoluzione di sole poche decine di tick per periodo,
 *   quantizzando grossolanamente il duty cycle). QCW_SetParams() ricalcola
 *   quindi ad ogni chiamata il prescaler PIÙ PICCOLO (massima risoluzione
 *   possibile) tale che il periodo intero, alla frequenza richiesta, stia
 *   entro il limite a 16 bit — es. ~15µs/tick a 1Hz, clock quasi pieno
 *   (~3.6ns/tick) a 50kHz. Prescaler e durate delle due fasi (ON/OFF, in
 *   tick alla risoluzione così scelta) sono applicati insieme — mai
 *   disallineati — ad ogni cambio fase in QCW_Start()/QCW_TimerCallback(),
 *   forzando un evento di update (UG) per il caricamento immediato del
 *   registro PSC (sempre bufferizzato in hardware, indipendentemente da
 *   ARPE).
 *
 *   LIMITI FISICI DA TENERE PRESENTI (non solo di calcolo):
 *   - Verso i 50kHz l'interrupt di TIM16 scatta 2x per periodo, cioè fino a
 *     100'000 volte/s: un carico CPU non trascurabile (stimato ~5-10% a
 *     50kHz continui) da sommare agli altri task/interrupt del sistema.
 *   - Sempre verso i 50kHz, a duty cycle molto lontani dal 50% (vicino a
 *     QCW_MIN_DUTY_PCT o QCW_MAX_DUTY_PCT) la fase più corta può durare
 *     solo poche decine di ns: la latenza di ingresso ISR (tipicamente
 *     100-300ns su Cortex-M7 a questa priorità) diventa comparabile alla
 *     durata della fase stessa, con jitter percentuale rilevante sul duty
 *     cycle reale. Per applicazioni che richiedono duty accurato a 50kHz,
 *     preferire valori di duty vicini al 50%.
 *   - Verso l'1Hz nessun problema di questo tipo (fasi dell'ordine dei
 *     centinaia di ms, interrupt rate trascurabile).
 *
 * USO:
 *   QCW_Init();                          // una volta, Sys_HwInit()/MX_FREERTOS_Init()
 *   QCW_SetParams(freq_hz, duty_pct);     // da "SET FREQ"/"SET DUTY" (rs485_cmd.c)
 *   QCW_Start();                          // da action_enter_emission() se qcw_enabled
 *   QCW_Stop();                           // da action_enter_enabled()/error/fault, o "SET QCW OFF"
 *
 * THREAD SAFETY:
 *   QCW_Start()/QCW_Stop()/QCW_SetParams() vanno chiamate solo dal task FSM
 *   (stesso vincolo di BoardCtrl_GateMC_Open/Close, mai da ISR). Il riarmo
 *   del timer dentro QCW_TimerCallback() avviene in contesto ISR e non
 *   condivide stato con le funzioni di controllo se non tramite i registri
 *   TIM16 stessi (nessuna sezione critica aggiuntiva necessaria: le
 *   funzioni di controllo fermano sempre il timer con HAL_TIM_Base_Stop_IT()
 *   prima di modificarne la configurazione).
 */

#ifndef DRIVERS_QCW_QCW_H_
#define DRIVERS_QCW_QCW_H_

#include <stdint.h>
#include <stdbool.h>

/* ========================================================================== */
/* --- LIMITI --- */
/* ========================================================================== */

#define QCW_MIN_FREQ_HZ       1U    /**< Frequenza minima [Hz] */
#define QCW_MAX_FREQ_HZ   50000U    /**< Frequenza massima [Hz] */
#define QCW_MIN_DUTY_PCT      1U    /**< Duty cycle minimo [%] */
#define QCW_MAX_DUTY_PCT     99U    /**< Duty cycle massimo [%] */

/*
 * QCW_MIN_ON_TIME_US (dal 2026-08-02): vincolo FISICO aggiuntivo, ortogonale
 * ai range nominali di freq/duty sopra. A freq/duty nominalmente validi ma
 * combinati "male" (freq alta + duty basso) il Ton effettivo (fase ON,
 * laser acceso) puo' scendere a pochi tick — es. 1% duty a 50000Hz vale
 * solo 0.2us nominali, ben sotto la latenza di ingresso ISR (100-300ns,
 * vedi banner RANGE E PRESCALER DINAMICO sopra) e comunque troppo corto
 * perche' l'elettronica di gate abbia un pilotaggio affidabile.
 *
 * Il vincolo Ton_us >= QCW_MIN_ON_TIME_US equivale, per freq_hz > 1000Hz, a
 * duty_pct >= freq_hz/1000 (a 50000Hz: duty minimo utile 50%, non piu' 1%
 * come da QCW_MIN_DUTY_PCT nominale). Sotto 1000Hz il vincolo non incide
 * mai: anche a duty_pct = QCW_MIN_DUTY_PCT (1%) il Ton resta >= 10us.
 *
 * Applicato SOLO sul duty (non su freq_hz, mai toccata) tramite
 * QCW_ClampDutyForMinOnTime() sotto — vedi banner li' per il dettaglio dei
 * punti di chiamata. Vincolo asimmetrico: SOLO sul Ton (fase ON). Il Toff
 * (fase OFF) puo' analogamente scendere sotto i 10us a duty vicini al
 * massimo (es. 99% a 50000Hz -> Toff ~0.2us) ma questo NON e' coperto da
 * questa costante ne' dall'helper sotto — scelta intenzionale, il requisito
 * riguarda solo il Ton.
 */
#define QCW_MIN_ON_TIME_US    10U   /**< Ton (fase ON) minimo garantito [us] */

/* ========================================================================== */
/* --- API --- */
/* ========================================================================== */

/**
 * @brief  Inizializza `htim16` (generato da CubeMX, già pronto con clock/NVIC
 *         a posto da MX_TIM16_Init()) in uno stato di riposo sicuro, SENZA
 *         avviarlo. Da chiamare una sola volta, dopo l'avvio dello scheduler
 *         (stesso vincolo HW init di Sys_HwInit(), vedi freertos.c —
 *         HAL_TIM_Base_Init() qui non rieseguirà MspInit perché l'handle è
 *         già READY, solo la riscrittura dei registri PSC/ARR). Prescaler e
 *         ARR "veri" sono ricalcolati dinamicamente da QCW_SetParams() ad
 *         ogni chiamata (vedi banner RANGE E PRESCALER DINAMICO sopra) e
 *         applicati da QCW_Start()/QCW_TimerCallback(): i valori scritti qui
 *         sono solo un placeholder, sempre sovrascritti prima che il timer
 *         venga effettivamente avviato. Non tocca PE12 (resta gestito da
 *         BoardCtrl_Init(), GPIO_Output nativo).
 */
void QCW_Init(void);

/**
 * @brief  Imposta frequenza e duty cycle di impulsazione. Ricalcola anche il
 *         prescaler di TIM16 (risoluzione dinamica, vedi banner RANGE E
 *         PRESCALER DINAMICO in cima al file) in funzione della frequenza
 *         richiesta. Se QCW è attualmente in corso (QCW_IsActive()), i nuovi
 *         parametri hanno effetto dal ciclo successivo (nessun glitch: la
 *         fase in corso viene completata con i parametri precedenti).
 *
 * @param  freq_hz   Frequenza [Hz], clampata a [QCW_MIN_FREQ_HZ..QCW_MAX_FREQ_HZ].
 * @param  duty_pct  Duty cycle [%], clampato a [QCW_MIN_DUTY_PCT..QCW_MAX_DUTY_PCT].
 * @retval false se i parametri richiesti sono fuori range (comunque
 *         clampati e applicati), true altrimenti.
 */
bool QCW_SetParams(uint16_t freq_hz, uint8_t duty_pct);

/**
 * @brief  Clampa duty_pct al minimo necessario affinche' il Ton risultante
 *         (fase ON) alla frequenza freq_hz data non scenda sotto
 *         QCW_MIN_ON_TIME_US (vedi banner sopra). NON modifica/richiede
 *         freq_hz oltre alla lettura: il parametro che viene eventualmente
 *         corretto e' sempre duty_pct.
 *
 *         Va chiamata da OGNI punto che scrive g_config.qcw_duty_pct e/o
 *         g_config.qcw_freq_hz (oggi: "SET FREQ"/"SET DUTY" in rs485_cmd.c
 *         e COM_CTRL_OP_SET_FREQ/SET_DUTY in COM_interface_app.c) PRIMA di
 *         salvare il duty in g_config — non basta farlo dentro
 *         QCW_SetParams(), perche' quella viene invocata solo quando QCW e'
 *         gia' attivo (QCW_IsActive()); se non attivo i due comandi SET
 *         scrivono g_config direttamente, senza mai passare da li'.
 *
 *         Asimmetrico per design: vincola solo il Ton (fase ON), non il
 *         Toff (fase OFF) — vedi banner QCW_MIN_ON_TIME_US sopra.
 *
 * @param  freq_hz   Frequenza [Hz], gia' clampata a
 *                    [QCW_MIN_FREQ_HZ..QCW_MAX_FREQ_HZ] dal chiamante.
 * @param  duty_pct  Duty cycle [%] candidato, gia' clampato a
 *                    [QCW_MIN_DUTY_PCT..QCW_MAX_DUTY_PCT] dal chiamante.
 * @retval Duty cycle [%] da usare effettivamente: invariato se il Ton
 *         risultante era gia' >= QCW_MIN_ON_TIME_US, altrimenti alzato al
 *         minimo che rispetta il vincolo (mai oltre QCW_MAX_DUTY_PCT).
 */
uint8_t QCW_ClampDutyForMinOnTime(uint16_t freq_hz, uint8_t duty_pct);

/**
 * @brief  Avvia l'impulsazione: apre subito il gate (fase ON) e arma TIM16.
 *         Da chiamare da action_enter_emission() al posto di
 *         BoardCtrl_GateMC_Open() quando g_config.qcw_enabled è attivo e
 *         s_mode == FSM_MODE_SW. No-op se già attivo.
 */
void QCW_Start(void);

/**
 * @brief  Ferma l'impulsazione e chiude il gate in modo sicuro
 *         (BoardCtrl_GateMC_Close(), PE12 HIGH = laser spento).
 *         Da chiamare in USCITA da SYS_EMISSION (qualunque causa: comando,
 *         errore, fault) e su "SET QCW OFF" mentre attivo. No-op se non attivo.
 */
void QCW_Stop(void);

/**
 * @brief  true se l'impulsazione QCW è attualmente in corso.
 */
bool QCW_IsActive(void);

/**
 * @brief  Da richiamare da HAL_TIM_PeriodElapsedCallback() (main.c) quando
 *         htim->Instance == TIM16. Contesto ISR: inverte PE12 e riarma il
 *         timer con la durata della fase opposta.
 *
 *         Catena di chiamata (TIM16_IRQHandler generato da CubeMX in
 *         stm32h7xx_it.c): TIM16_IRQHandler() -> HAL_TIM_IRQHandler(&htim16)
 *         -> HAL_TIM_PeriodElapsedCallback(&htim16) -> (dispatch su Instance
 *         in main.c) -> QCW_TimerCallback(). Nessun handler IRQ dedicato
 *         necessario in questo modulo.
 */
void QCW_TimerCallback(void);

#endif /* DRIVERS_QCW_QCW_H_ */
