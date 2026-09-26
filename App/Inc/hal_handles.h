/*
 * hal_handles.h
 *
 *  Created on: 11 giu 2026
 *      Author: lucamaggiotanasi
 */

#ifndef INC_HAL_HANDLES_H_
#define INC_HAL_HANDLES_H_


/*
 * hal_handles.h
 *
 * Dichiarazioni extern degli handle HAL generati da CubeMX in main.c.
 * CubeMX non genera usart.h/tim.h separati se l'opzione "split per periferica"
 * non è attiva — questo file colma quella lacuna senza toccare i file generati.
 *
 * REGOLA: aggiungere qui solo gli handle effettivamente usati dal codice
 * applicativo. Non modificare mai main.c o main.h manualmente.
 */

#include "stm32h7xx_hal.h"

extern UART_HandleTypeDef huart1;    /* USART1  - RS485 ASCII interface (HW DE nativo)              */
extern UART_HandleTypeDef huart6;    /* USART6  - AMC (STM32G473, full-duplex)                       */
extern UART_HandleTypeDef huart10;   /* USART10 - LaseQ (half-duplex, DIR via GPIO PF13, no HW ctrl) */
extern ADC_HandleTypeDef  hadc1;     /* Correnti eFuse + V PSU   */
extern ADC_HandleTypeDef  hadc3;     /* Corrente eFuse COM       */
extern SPI_HandleTypeDef  hspi3;     /* SPI3: COM interface + SD card (condivisa) */
extern SPI_HandleTypeDef  hspi5;     /* ADC esterno AD7490 temp  */
extern I2C_HandleTypeDef  hi2c2;     /* SHT35 umidità/temp amb.  */
extern RTC_HandleTypeDef  hrtc;      /* RTC interno (log + SET TIME/DATE) */
extern IWDG_HandleTypeDef hiwdg1;    /* Watchdog indipendente (vedi App/Watchdog) */
extern TIM_HandleTypeDef  htim16;    /* QCW: impulsazione nGATE_MC (vedi Drivers/QCW/QCW.h) */


#endif /* INC_HAL_HANDLES_H_ */
