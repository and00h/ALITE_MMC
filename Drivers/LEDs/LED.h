/*
 * LED.h
 *
 * Created on: 26 mag 2026
 * Author: lucamaggiotanasi
 *
 * DESCRIZIONE COMPONENTE:
 * Driver software per la gestione dei LED di stato della scheda e dei LED PWM
 * del pannello frontale (Status LED ed Emission LED). Supporta modalità statiche,
 * lampeggi a frequenza variabile e dissolvenze (fade) lineari tramite correzione gamma.
 */

#ifndef LEDS_LED_H_
#define LEDS_LED_H_

#include "stm32h7xx_hal.h"

/* ========================================================================== */
/*--- COSTANTI LOGICHE DI PILOTAGGIO ---*/
/* ========================================================================== */
// Nota: I LED sulla scheda sono tipicamente configurati in logica invertita (Active Low)
#define LED_ON	0   // Livello logico basso (0V) -> LED Acceso
#define LED_OFF 1   // Livello logico alto (VCC) -> LED Spento

/* ========================================================================== */
/*--- CONFIGURAZIONE HARDWARE DEL TIMER (PWM) ---*/
/* ========================================================================== */
#define LED_TIM_ARR 999   // Auto-Reload Register: definisce la risoluzione del PWM (0-999 = 1000 step)
#define FPLED_PSC	274   // Prescaler del Timer: determina la frequenza base dell'onda PWM

/* ========================================================================== */
/*--- TEMPORIZZAZIONI E FREQUENZE DEGLI EFFETTI ---*/
/* ========================================================================== */
/* Frequenza stimata di chiamata della funzione FPLED_Process() espressa in millisecondi.
 * Modificare questo valore se si decide di richiamare il processo con un periodo diverso. */
#define FPLED_PROCESS_PERIOD_MS    1

// Espressione dei tempi di Blink in millisecondi (Assumendo processo a 1ms)
#define FAST_BLINK_CNT	(250 / FPLED_PROCESS_PERIOD_MS) // T_toggle = 250ms -> Periodo = 500ms (Frequenza = 2 Hz)
#define SLOW_BLINK_CNT	(500 / FPLED_PROCESS_PERIOD_MS) // T_toggle = 500ms -> Periodo = 1000ms (Frequenza = 1 Hz)

// Espressione del tempo di avanzamento (passo) per gli effetti di Fade
#define FAST_FADE_STEP_MS  (4  / FPLED_PROCESS_PERIOD_MS)  // Avanzamento rampa ogni 4ms  → ciclo completo ~512ms (~2 Hz)
#define SLOW_FADE_STEP_MS  (15 / FPLED_PROCESS_PERIOD_MS)  // Avanzamento rampa ogni 15ms → ciclo completo ~1920ms (~0.5 Hz)

// Temporizzazioni per DOUBLE_FADE (IDLE: due respiri + pausa)
#define DOUBLE_FADE_PAUSE_MS  (1000 / FPLED_PROCESS_PERIOD_MS) // Pausa tra i cicli di doppio fade

// Temporizzazioni per TRIPLE_BLINK (FAULT: 3 blink veloci + pausa)
#define TRIPLE_BLINK_ON_MS    FAST_BLINK_CNT                   // Fase ON  per ogni blink (250ms)
#define TRIPLE_BLINK_OFF_MS   FAST_BLINK_CNT                   // Fase OFF tra blink (250ms)
#define TRIPLE_BLINK_PAUSE_MS (1000 / FPLED_PROCESS_PERIOD_MS) // Pausa finale (1s)

/* ========================================================================== */
/*--- TIPI ENUMERATIVI (API PUBBLICHE) ---*/
/* ========================================================================== */

/**
 * @brief Colori disponibili per i LED RGB integrati sulla scheda (On-Board LED).
 * Pilotati direttamente in modalità GPIO standard (Digital Output On/Off).
 */
typedef enum {
	NONE,         // Tutti i canali spenti
    RED,          // Solo LED Rosso
    GREEN,        // Solo LED Verde
    BLUE,         // Solo LED Blu
    WHITE,        // Accensione contemporanea RGB (Luce bianca)
    YELLOW,       // Rosso + Verde
	PURPLE,       // Rosso + Blu
	LIGHT_BLUE    // Verde + Blu
} color_t;

/**
 * @brief Modalità operative per i LED del Pannello Frontale (FPLED) gestiti in PWM.
 */
typedef enum {
	OFF,          // LED completamente spento
	STATIC,       // LED acceso fisso alla luminosità massima impostata
	SLOW_FADE,    // Dissolvenza ciclica lenta (effetto "respiro" ~1.9s periodo)
	FAST_FADE,    // Dissolvenza ciclica rapida (effetto "respiro" ~512ms periodo)
	SLOW_BLINK,   // Lampeggio classico lento (1 Hz, Duty Cycle 50%)
	FAST_BLINK,   // Lampeggio classico rapido (2 Hz, Duty Cycle 50%)
	DOUBLE_FADE,  // Doppio respiro lento + pausa 1s (IDLE: fade-fade-stop)
	TRIPLE_BLINK, // Tre blink veloci (2Hz) + pausa 1s (FAULT)
} LED_mode_t;

/**
 * @brief Identificativi univoci dei LED del Pannello Frontale.
 * Utilizzati come indice (ID) per indirizzare i comandi alle funzioni Set/Get.
 */
typedef enum {
	STATUS_LED,   // Identifica il LED di stato (Mappato sul Canale 1 del Timer)
	EMISSION_LED  // Identifica il LED di emissione potenza/laser (Mappato sul Canale 2 del Timer)
} LED_id_t;

/* ========================================================================== */
/*--- STRUTTURE DATI (INCAPSULAMENTO PARAMETRI LED) ---*/
/* ========================================================================== */

/**
 * @brief Struttura di controllo contenente lo stato software e hardware di un singolo LED PWM.
 */
typedef struct{
    LED_mode_t mode;       // Modalità di funzionamento attuale del LED
    uint32_t ch;           // Canale hardware associato (Es: TIM_CHANNEL_1, TIM_CHANNEL_2)

    int16_t brightness;    // Indice corrente della rampa (punta alla cella della gamma_table, range 0-63)
    int16_t direction;     // Direzione della rampa di dissolvenza (1 = incremento, -1 = decremento)

    uint16_t blink_cnt;    // Contatore incrementale per la gestione dei millisecondi (Blink e Fade)
    uint8_t  state;        // Flag di stato logico interno per l'effetto Blink (0 = spento, 1 = acceso fisso)

    uint16_t max_intensity;// Valore di Duty Cycle massimo scalato in base all'ARR del Timer (0 - LED_TIM_ARR)
} FPLED_t;

/* ========================================================================== */
/*--- PROTOTIPI DELLE FUNZIONI PUBBLICHE (API) ---*/
/* ========================================================================== */

/**
 * @brief Inizializza il modulo driver dei LED del pannello frontale.
 * @note Questa funzione configura i parametri delle strutture interne, assegna i canali del timer
 * e avvia fisicamente le periferiche PWM hardware di STM32 in modalità Interrupt.
 * @param htim: Puntatore alla struttura di gestione del Timer generata da CubeMX (es: &htim1).
 */
void FPLED_Init(TIM_HandleTypeDef *htim);

/**
 * @brief Configura lo stato dei pin del LED RGB presente sulla scheda madre.
 * @param color: Colore desiderato estratto dall'omonimo enum color_t.
 */
void SetBoardLED(color_t color);

/**
 * @brief Imposta l'intensità luminosa massima per un determinato LED del pannello frontale.
 * @param led: Identificativo del LED da modificare (STATUS_led oppure EMISSION_LED).
 * @param intensity: Valore di luminosità espresso in percentuale (range: 0 - 100).
 */
void SetLEDIntensity(LED_id_t led, uint8_t intensity);

/**
 * @brief Modifica la modalità di funzionamento (effetto visivo) di un determinato LED del pannello frontale.
 * @param led: Identificativo del LED da modificare (STATUS_led oppure EMISSION_LED).
 * @param mode: Nuova modalità operativa da applicare (es: SLOW_BLINK, FAST_FADE, STATIC ecc.).
 */
void SetLEDMode(LED_id_t led, LED_mode_t mode);

/**
 * @brief Esegue l'elaborazione ciclica della macchina a stati dei LED del pannello frontale.
 * @critical Questa funzione DEVE essere chiamata tassativamente all'interno di un ciclo temporizzato
 * regolare (consigliato: Callback del SysTick o di un Timer di Base ogni 1ms o task FreeRTOS con vTaskDelayUntil).
 */
void FPLED_Process(void);


/* ========================================================================== */
/*--- FUNZIONI DI UTILITÀ AGGIUNTIVE (FUNZIONALIZZAZIONE ESTERNA) ---*/
/* ========================================================================== */

/**
 * @brief Restituisce la modalità attuale in cui si trova il LED specificato.
 * @note Molto utile nelle macchine a stati esterne per verificare se un LED sta già lampeggiando
 * (es. in caso di allarme attivo) prima di inviare un comando ridondante.
 * @param led: Identificativo del LED.
 * @return LED_mode_t: Modalità operativa corrente.
 */
LED_mode_t GetLEDMode(LED_id_t led);

/**
 * @brief Restituisce il valore di intensità di picco impostato sul LED.
 * @param led: Identificativo del LED.
 * @return uint8_t: Intensità corrente convertita in valore percentuale (0 - 100).
 */
uint8_t GetLEDIntensity(LED_id_t led);


#endif /* LEDS_LED_H_ */
