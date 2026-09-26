/*
 * LED.c
 *
 * Created on: 26 mag 2026
 * Author: lucamaggiotanasi
 */
#include "LED.h"
#include "main.h"
#include "stdbool.h"

// Puntatore al modulo Timer hardware (TIM) configurato in CubeMX
static TIM_HandleTypeDef   *fpled_tim = NULL;

// Istanze statiche dei LED del pannello frontale per preservare l'incapsulamento
static FPLED_t EmissionLED = {0};
static FPLED_t StatusLED = {0};

/* * ARRAYS DI PUNTATORI AI LED
 * Permette di mappare l'enum LED_id_t (es. STATUS=0, EMISSION=1) direttamente
 * agli indirizzi di memoria delle struct statiche. Fondamentale per ciclare in FPLED_Process.
 */
static FPLED_t *FPLEDs[2] = {&StatusLED, &EmissionLED};

/*
 * TABELLA DI CORREZIONE GAMMA (64 livelli)
 * Risolve il problema della percezione non lineare dell'occhio umano.
 * NOTA: Il valore massimo memorizzato è 6141. Se cambia l'ARR del timer,
 * la formula matematica nel FADE scalerà questo valore proporzionalmente.
 */
static const uint16_t gamma_table[64] = {
    0,     1,     2,     4,     7,     11,    16,    22,
    30,    39,    49,    61,    75,    91,    108,   128,
    150,   174,   201,   230,   262,   297,   335,   375,
    419,   466,   517,   571,   629,   691,   757,   827,
    901,   980,   1064,  1152,  1246,  1344,  1448,  1557,
    1672,  1792,  1918,  2050,  2188,  2332,  2483,  2640,
    2803,  2973,  3150,  3334,  3525,  3724,  3929,  4143,
    4364,  4593,  4830,  5075,  5328,  5591,  5861,  6141
};


void FPLED_Init(TIM_HandleTypeDef *htim){
    fpled_tim = htim;

    /* Inizializzazione strutture software */
    StatusLED.blink_cnt    = 0;
    StatusLED.brightness   = 0;
    StatusLED.ch           = TIM_CHANNEL_1;
    StatusLED.mode         = OFF;
    StatusLED.state        = 0;
    StatusLED.direction    = 1;
    StatusLED.max_intensity = LED_TIM_ARR;

    EmissionLED.blink_cnt    = 0;
    EmissionLED.brightness   = 0;
    EmissionLED.ch           = TIM_CHANNEL_2;
    EmissionLED.mode         = OFF;
    EmissionLED.state        = 0;
    EmissionLED.direction    = 1;
    EmissionLED.max_intensity = LED_TIM_ARR;

    /*
     * Fermaiamo il timer prima di toccare PSC e ARR.
     * CubeMX genera TIM15 con PSC=0, ARR=65535 in modalità Output Compare.
     * Qui sovrascriviamo con i valori corretti per i LED.
     *
     * Nota CubeMX: impostare TIM15 CH1/CH2 come "PWM Generation" e
     * Prescaler=274, Counter Period=999 per evitare questo override runtime.
     */
    __HAL_TIM_DISABLE(fpled_tim);

    __HAL_TIM_SET_PRESCALER(fpled_tim, FPLED_PSC);
    __HAL_TIM_SET_AUTORELOAD(fpled_tim, LED_TIM_ARR);

    /*
     * Forza un Update Event (UG bit) per applicare subito il prescaler.
     * Il PSC è preloaded su STM32: senza UEV rimarrebbe quello vecchio
     * fino al prossimo overflow del contatore.
     * Azzera anche il contatore e pulisce il flag UIF.
     */
    fpled_tim->Instance->EGR = TIM_EGR_UG;
    __HAL_TIM_CLEAR_FLAG(fpled_tim, TIM_FLAG_UPDATE);

    /*
     * PROBLEMA PRINCIPALE: CubeMX ha configurato TIM15 CH1/CH2 in modalità
     * Output Compare (CCMR registri = OC TIMING/FROZEN), non PWM.
     * HAL_TIM_PWM_Start() abilita il canale e il MOE ma NON cambia la modalità
     * del canale → nessun segnale PWM sull'uscita (pin bloccato).
     *
     * Soluzione: riconfigura esplicitamente i canali in PWM mode 1 tramite
     * HAL_TIM_PWM_ConfigChannel(), che scrive i registri CCMR correttamente.
     */
    TIM_OC_InitTypeDef oc = {0};
    oc.OCMode       = TIM_OCMODE_PWM1;
    oc.Pulse        = 0;                     /* duty cycle iniziale = 0 */
    oc.OCPolarity   = TIM_OCPOLARITY_HIGH;
    oc.OCFastMode   = TIM_OCFAST_DISABLE;
    oc.OCNPolarity  = TIM_OCNPOLARITY_HIGH;
    oc.OCIdleState  = TIM_OCIDLESTATE_RESET;
    oc.OCNIdleState = TIM_OCNIDLESTATE_RESET;

    HAL_TIM_PWM_ConfigChannel(fpled_tim, &oc, TIM_CHANNEL_1);
    HAL_TIM_PWM_ConfigChannel(fpled_tim, &oc, TIM_CHANNEL_2);

    /*
     * Avvio PWM:
     *   - Abilita CCx per il canale (CCER register)
     *   - Attiva MOE (Main Output Enable, richiesto per TIM15 advanced timer)
     *   - Avvia il contatore (CR1 |= CEN)
     * Duty cycle iniziale già a 0 dalla configurazione sopra → LED spenti.
     */
    HAL_TIM_PWM_Start(fpled_tim, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(fpled_tim, TIM_CHANNEL_2);
}

void SetBoardLED(color_t color){
	switch(color){
		case NONE:
			HAL_GPIO_WritePin(nLED_RED_GPIO_Port,nLED_RED_Pin,LED_OFF);
			HAL_GPIO_WritePin(nLED_GREEN_GPIO_Port,nLED_GREEN_Pin,LED_OFF);
			HAL_GPIO_WritePin(nLED_BLUE_GPIO_Port,nLED_BLUE_Pin,LED_OFF);
			break;
		case RED:
			HAL_GPIO_WritePin(nLED_RED_GPIO_Port,nLED_RED_Pin,LED_ON);
			HAL_GPIO_WritePin(nLED_GREEN_GPIO_Port,nLED_GREEN_Pin,LED_OFF);
			HAL_GPIO_WritePin(nLED_BLUE_GPIO_Port,nLED_BLUE_Pin,LED_OFF);
			break;
		case GREEN:
			HAL_GPIO_WritePin(nLED_RED_GPIO_Port,nLED_RED_Pin,LED_OFF);
			HAL_GPIO_WritePin(nLED_GREEN_GPIO_Port,nLED_GREEN_Pin,LED_ON);
			HAL_GPIO_WritePin(nLED_BLUE_GPIO_Port,nLED_BLUE_Pin,LED_OFF);
			break;
		case BLUE:
			HAL_GPIO_WritePin(nLED_RED_GPIO_Port,nLED_RED_Pin,LED_OFF);
			HAL_GPIO_WritePin(nLED_GREEN_GPIO_Port,nLED_GREEN_Pin,LED_OFF);
			HAL_GPIO_WritePin(nLED_BLUE_GPIO_Port,nLED_BLUE_Pin,LED_ON);
			break;
		case WHITE:
			HAL_GPIO_WritePin(nLED_RED_GPIO_Port,nLED_RED_Pin,LED_ON);
			HAL_GPIO_WritePin(nLED_GREEN_GPIO_Port,nLED_GREEN_Pin,LED_ON);
			HAL_GPIO_WritePin(nLED_BLUE_GPIO_Port,nLED_BLUE_Pin,LED_ON);
			break;
		case YELLOW:
			HAL_GPIO_WritePin(nLED_RED_GPIO_Port,nLED_RED_Pin,LED_ON);
			HAL_GPIO_WritePin(nLED_GREEN_GPIO_Port,nLED_GREEN_Pin,LED_ON);
			HAL_GPIO_WritePin(nLED_BLUE_GPIO_Port,nLED_BLUE_Pin,LED_OFF);
			break;
		case PURPLE:
			HAL_GPIO_WritePin(nLED_RED_GPIO_Port,nLED_RED_Pin,LED_ON);
			HAL_GPIO_WritePin(nLED_GREEN_GPIO_Port,nLED_GREEN_Pin,LED_OFF);
			HAL_GPIO_WritePin(nLED_BLUE_GPIO_Port,nLED_BLUE_Pin,LED_ON);
			break;
		case LIGHT_BLUE:
			HAL_GPIO_WritePin(nLED_RED_GPIO_Port,nLED_RED_Pin,LED_OFF);
			HAL_GPIO_WritePin(nLED_GREEN_GPIO_Port,nLED_GREEN_Pin,LED_ON);
			HAL_GPIO_WritePin(nLED_BLUE_GPIO_Port,nLED_BLUE_Pin,LED_ON);
			break;
		default:
            break;
		}
}


/*
 * IMPOSTAZIONE LUMINOSITÀ MASSIMA (0-100%)
 * Limita l'intensità massima del led specificato in percentuale 0-100%.
 */
void SetLEDIntensity(LED_id_t led, uint8_t intensity){
	if (intensity > 100)
    {
		intensity = 100;
	}
	FPLEDs[led]->max_intensity = ((uint32_t) intensity * LED_TIM_ARR) / 100;
}

// IMPOSTAZIONE MODALITÀ DI FUNZIONAMENTO
void SetLEDMode(LED_id_t led, LED_mode_t mode){
	FPLEDs[led]->mode      = mode;
    FPLEDs[led]->blink_cnt  = 0;   /* reset contatore temporale                     */
    FPLEDs[led]->state      = 0;   /* reset sub-stato (usato da BLINK e pattern)    */
    FPLEDs[led]->brightness = 0;   /* reset indice gamma (FADE parte da zero)       */
    FPLEDs[led]->direction  = 1;   /* fade parte sempre in salita                   */
}

// Iniezione diretta nel registro di comparazione (CCR) del rispettivo canale del timer
static void FPLED_SetCCR(uint32_t ch, uint16_t val)
{
    __HAL_TIM_SET_COMPARE(fpled_tim, ch, val);
}

/*
 * MACCHINA A STATI DI PROCESSO DEI LED
 * Questa funzione deve essere chiamata a intervalli regolari (es. ogni 1ms nel SysTick o in un timer di callback).
 * Gestisce l'avanzamento dei contatori temporali e calcola l'output PWM per ciascuno dei due LED.
 */
void FPLED_Process(void){
    for(int i = 0; i < 2; i++)
    {
        switch(FPLEDs[i]->mode)
        {
        case OFF:
            FPLED_SetCCR(FPLEDs[i]->ch, 0);
            break;

        case STATIC:
            // Corretto: usa il limite proporzionale calcolato, non più il valore fisso 999
            FPLED_SetCCR(FPLEDs[i]->ch, FPLEDs[i]->max_intensity);
            break;

        case SLOW_BLINK:
        	FPLEDs[i]->blink_cnt++;

            if(FPLEDs[i]->blink_cnt >= SLOW_BLINK_CNT)
            {
            	FPLEDs[i]->blink_cnt = 0;
            	FPLEDs[i]->state ^= 1; // Inversione logica dello stato (Toggle bit a bit)

            	if (FPLEDs[i]->state != 0)
                {
					FPLED_SetCCR(FPLEDs[i]->ch, FPLEDs[i]->max_intensity);
				}
				else
                {
					FPLED_SetCCR(FPLEDs[i]->ch, 0);
				}
            }
            break;

        case FAST_BLINK:
        	FPLEDs[i]->blink_cnt++;

            if(FPLEDs[i]->blink_cnt >= FAST_BLINK_CNT)
            {
            	FPLEDs[i]->blink_cnt = 0;
            	FPLEDs[i]->state ^= 1; // Inversione logica dello stato (Toggle bit a bit)

            	if (FPLEDs[i]->state != 0)
                {
					FPLED_SetCCR(FPLEDs[i]->ch, FPLEDs[i]->max_intensity);
				}
				else
                {
					FPLED_SetCCR(FPLEDs[i]->ch, 0);
				}
            }
            break;

        case SLOW_FADE:
			FPLEDs[i]->blink_cnt++;

			/* Rallentamento: avanza di uno step gamma ogni SLOW_FADE_STEP_MS ms.
             * Con 64 step e 15ms per step → rampa completa in ~960ms per direzione
             * → ciclo respiro completo ~1.92s (~0.52 Hz).                          */
			if (FPLEDs[i]->blink_cnt >= SLOW_FADE_STEP_MS)
			{
				FPLEDs[i]->blink_cnt = 0;
				FPLEDs[i]->brightness += FPLEDs[i]->direction;

				if(FPLEDs[i]->brightness >= 63)
				{
					FPLEDs[i]->brightness = 63;
					FPLEDs[i]->direction = -1;
				}

				if(FPLEDs[i]->brightness <= 0)
				{
					FPLEDs[i]->brightness = 0;
					FPLEDs[i]->direction = 1;
				}

				/* Scaling dinamico: gamma_table[63]=5848 → duty proporzionale a max_intensity */
				uint32_t duty_s = ((uint32_t)gamma_table[FPLEDs[i]->brightness] * FPLEDs[i]->max_intensity) / 5848;
				FPLED_SetCCR(FPLEDs[i]->ch, (uint16_t)duty_s);
			}
			break;

        case FAST_FADE:
			FPLEDs[i]->blink_cnt++;

			/* Rallentamento fast: avanza ogni FAST_FADE_STEP_MS ms.
             * Con 64 step e 4ms per step → rampa completa in ~256ms per direzione
             * → ciclo completo ~512ms (~2 Hz).                                      */
			if (FPLEDs[i]->blink_cnt >= FAST_FADE_STEP_MS)
			{
				FPLEDs[i]->blink_cnt = 0;
				FPLEDs[i]->brightness += FPLEDs[i]->direction;

				if(FPLEDs[i]->brightness >= 63)
				{
					FPLEDs[i]->brightness = 63;
					FPLEDs[i]->direction = -1;
				}

				if(FPLEDs[i]->brightness <= 0)
				{
					FPLEDs[i]->brightness = 0;
					FPLEDs[i]->direction = 1;
				}

				uint32_t duty_f = ((uint32_t)gamma_table[FPLEDs[i]->brightness] * FPLEDs[i]->max_intensity) / 5848;
				FPLED_SetCCR(FPLEDs[i]->ch, (uint16_t)duty_f);
			}
			break;

        case DOUBLE_FADE:
        {
            /*
             * Doppio respiro lento + pausa.
             * stati 0-3: fasi di fade (64 step × SLOW_FADE_STEP_MS ms ciascuno)
             *   stato 0: fade UP   (1° respiro)
             *   stato 1: fade DOWN (1° respiro)
             *   stato 2: fade UP   (2° respiro)
             *   stato 3: fade DOWN (2° respiro)
             * stato 4: pausa a LED spento per DOUBLE_FADE_PAUSE_MS ms, poi torna a 0.
             *
             * Timing totale (con SLOW_FADE_STEP_MS=15): 4 × 960ms + 1000ms ≈ 4.84s ciclo.
             */
            if (FPLEDs[i]->state < 4U)
            {
                /* Fasi di fade */
                FPLEDs[i]->blink_cnt++;
                if (FPLEDs[i]->blink_cnt >= FAST_FADE_STEP_MS)
                {
                    FPLEDs[i]->blink_cnt = 0;
                    FPLEDs[i]->brightness += FPLEDs[i]->direction;

                    /* Limite superiore: inversione */
                    if (FPLEDs[i]->brightness >= 63)
                    {
                        FPLEDs[i]->brightness = 63;
                        FPLEDs[i]->direction  = -1;
                        FPLEDs[i]->state++;   /* 0→1 oppure 2→3 */
                    }
                    /* Limite inferiore: inizio prossima fase o pausa */
                    else if (FPLEDs[i]->brightness <= 0)
                    {
                        FPLEDs[i]->brightness = 0;
                        if (FPLEDs[i]->state == 3U)
                        {
                            /* Fine 2° respiro: vai in pausa */
                            FPLEDs[i]->state     = 4U;
                            FPLEDs[i]->blink_cnt = 0;
                        }
                        else
                        {
                            FPLEDs[i]->direction = 1;
                            FPLEDs[i]->state++;   /* 1→2 */
                        }
                    }

                    uint32_t duty_df = ((uint32_t)gamma_table[FPLEDs[i]->brightness]
                                        * FPLEDs[i]->max_intensity) / 5848U;
                    FPLED_SetCCR(FPLEDs[i]->ch, (uint16_t)duty_df);
                }
            }
            else
            {
                /* Fase pausa: LED spento, attesa DOUBLE_FADE_PAUSE_MS poi riparti */
                FPLED_SetCCR(FPLEDs[i]->ch, 0U);
                FPLEDs[i]->blink_cnt++;
                if (FPLEDs[i]->blink_cnt >= DOUBLE_FADE_PAUSE_MS)
                {
                    FPLEDs[i]->blink_cnt = 0;
                    FPLEDs[i]->state     = 0U;
                    FPLEDs[i]->brightness = 0;
                    FPLEDs[i]->direction  = 1;
                }
            }
        }
        break;

        case TRIPLE_BLINK:
        {
            /*
             * Tre blink veloci (2 Hz) + pausa 1s.
             * stato 0: ON  250ms (1° blink)
             * stato 1: OFF 250ms
             * stato 2: ON  250ms (2° blink)
             * stato 3: OFF 250ms
             * stato 4: ON  250ms (3° blink)
             * stato 5: PAUSE 1000ms, poi torna a 0.
             *
             * Timing totale: 5×250ms + 1000ms = 2250ms ciclo.
             */
            bool tb_on = (FPLEDs[i]->state < 5U) && ((FPLEDs[i]->state % 2U) == 0U);

            FPLED_SetCCR(FPLEDs[i]->ch,
                         tb_on ? FPLEDs[i]->max_intensity : 0U);

            FPLEDs[i]->blink_cnt++;

            uint16_t tb_thr;
            if (FPLEDs[i]->state == 5U)
            {
                tb_thr = TRIPLE_BLINK_PAUSE_MS;
            }
            else
            {
                tb_thr = tb_on ? TRIPLE_BLINK_ON_MS : TRIPLE_BLINK_OFF_MS;
            }

            if (FPLEDs[i]->blink_cnt >= tb_thr)
            {
                FPLEDs[i]->blink_cnt = 0;
                FPLEDs[i]->state++;
                if (FPLEDs[i]->state > 5U)
                {
                    FPLEDs[i]->state = 0U;
                }
            }
        }
        break;

        default:
            break;
        }
    }
}

/* ========================================================================== */
/*--- FUNZIONI DI UTILITÀ AGGIUNTIVE (FUNZIONALIZZAZIONE ESTERNA) ---*/
/* ========================================================================== */

LED_mode_t GetLEDMode(LED_id_t led) {
    return FPLEDs[led]->mode;
}

uint8_t GetLEDIntensity(LED_id_t led) {
    // Rifacciamo il calcolo inverso per restituire la percentuale esatta (0-100) all'applicazione
    return (uint8_t)(((uint32_t)FPLEDs[led]->max_intensity * 100) / LED_TIM_ARR);
}
