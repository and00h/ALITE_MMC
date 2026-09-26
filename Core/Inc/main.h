/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32h7xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define LID1_OPEN_Pin GPIO_PIN_2
#define LID1_OPEN_GPIO_Port GPIOE
#define LID2_OPEN_Pin GPIO_PIN_3
#define LID2_OPEN_GPIO_Port GPIOE
#define AUX_LED_Pin GPIO_PIN_4
#define AUX_LED_GPIO_Port GPIOE
#define STATUS_LED_Pin GPIO_PIN_5
#define STATUS_LED_GPIO_Port GPIOE
#define EMISSION_LED_Pin GPIO_PIN_6
#define EMISSION_LED_GPIO_Port GPIOE
#define nKEY_STATUS_A_Pin GPIO_PIN_13
#define nKEY_STATUS_A_GPIO_Port GPIOC
#define nKEY_STATUS_B_Pin GPIO_PIN_14
#define nKEY_STATUS_B_GPIO_Port GPIOC
#define nFP_BUTTON_Pin GPIO_PIN_15
#define nFP_BUTTON_GPIO_Port GPIOC
#define nFP_BUTTON_EXTI_IRQn EXTI15_10_IRQn
#define SENS_SDA_Pin GPIO_PIN_0
#define SENS_SDA_GPIO_Port GPIOF
#define SENS_SCL_Pin GPIO_PIN_1
#define SENS_SCL_GPIO_Port GPIOF
#define nTH_SENS_RESET_Pin GPIO_PIN_2
#define nTH_SENS_RESET_GPIO_Port GPIOF
#define COM_PWR_I_MON_Pin GPIO_PIN_3
#define COM_PWR_I_MON_GPIO_Port GPIOF
#define nFLOOD_1_Pin GPIO_PIN_4
#define nFLOOD_1_GPIO_Port GPIOF
#define nFLOOD_2_Pin GPIO_PIN_5
#define nFLOOD_2_GPIO_Port GPIOF
#define TEMP_ADC_nCS_Pin GPIO_PIN_6
#define TEMP_ADC_nCS_GPIO_Port GPIOF
#define TEMP_ADC_SCK_Pin GPIO_PIN_7
#define TEMP_ADC_SCK_GPIO_Port GPIOF
#define TEMP_ADC_DOUT_Pin GPIO_PIN_8
#define TEMP_ADC_DOUT_GPIO_Port GPIOF
#define TEMP_ADC_DIN_Pin GPIO_PIN_9
#define TEMP_ADC_DIN_GPIO_Port GPIOF
#define SAB_PWR_GOOD_Pin GPIO_PIN_10
#define SAB_PWR_GOOD_GPIO_Port GPIOF
#define nSAB_PWR_FLT_Pin GPIO_PIN_0
#define nSAB_PWR_FLT_GPIO_Port GPIOC
#define nSAB_PWR_SHDN_Pin GPIO_PIN_1
#define nSAB_PWR_SHDN_GPIO_Port GPIOC
#define nSAB_INTLCK_B_STATUS_Pin GPIO_PIN_2
#define nSAB_INTLCK_B_STATUS_GPIO_Port GPIOC
#define nSAB_INTLCK_B_STATUS_EXTI_IRQn EXTI2_IRQn
#define nSAB_B_TEST_Pin GPIO_PIN_3
#define nSAB_B_TEST_GPIO_Port GPIOC
#define nSAB_INTLCK_A_STATUS_Pin GPIO_PIN_0
#define nSAB_INTLCK_A_STATUS_GPIO_Port GPIOA
#define nSAB_INTLCK_A_STATUS_EXTI_IRQn EXTI0_IRQn
#define nSAB_EN_Pin GPIO_PIN_1
#define nSAB_EN_GPIO_Port GPIOA
#define nSAB_A_TEST_Pin GPIO_PIN_2
#define nSAB_A_TEST_GPIO_Port GPIOA
#define SAB_PWR_I_MON_Pin GPIO_PIN_3
#define SAB_PWR_I_MON_GPIO_Port GPIOA
#define AIMING_DAC_Pin GPIO_PIN_5
#define AIMING_DAC_GPIO_Port GPIOA
#define AIMING_R_TEMP_Pin GPIO_PIN_6
#define AIMING_R_TEMP_GPIO_Port GPIOA
#define LASE_Q_PWR_I_MON_Pin GPIO_PIN_4
#define LASE_Q_PWR_I_MON_GPIO_Port GPIOC
#define LPWR_SET_ISO_Pin GPIO_PIN_5
#define LPWR_SET_ISO_GPIO_Port GPIOC
#define MAIN_PWR_I_MON_Pin GPIO_PIN_1
#define MAIN_PWR_I_MON_GPIO_Port GPIOB
#define PSU2_VMON_Pin GPIO_PIN_11
#define PSU2_VMON_GPIO_Port GPIOF
#define PSU1_VMON_Pin GPIO_PIN_12
#define PSU1_VMON_GPIO_Port GPIOF
#define LASE_Q_485_DIR_Pin GPIO_PIN_13
#define LASE_Q_485_DIR_GPIO_Port GPIOF
#define HW_SETPOINT_SEL_Pin GPIO_PIN_14
#define HW_SETPOINT_SEL_GPIO_Port GPIOF
#define nLASE_Q_INTLCK_Pin GPIO_PIN_15
#define nLASE_Q_INTLCK_GPIO_Port GPIOF
#define LASE_Q_OUT_Pin GPIO_PIN_0
#define LASE_Q_OUT_GPIO_Port GPIOG
#define AIMING_ENABLE_Pin GPIO_PIN_7
#define AIMING_ENABLE_GPIO_Port GPIOE
#define nMCU2_RST_Pin GPIO_PIN_8
#define nMCU2_RST_GPIO_Port GPIOE
#define nLASE_Q_EN_Pin GPIO_PIN_9
#define nLASE_Q_EN_GPIO_Port GPIOE
#define nHW_SETPOINT_FLT_Pin GPIO_PIN_10
#define nHW_SETPOINT_FLT_GPIO_Port GPIOE
#define nHW_SETPOINT_FLT_EXTI_IRQn EXTI15_10_IRQn
#define nHW_SETPOINT_EN_Pin GPIO_PIN_11
#define nHW_SETPOINT_EN_GPIO_Port GPIOE
#define nGATE_MC_Pin GPIO_PIN_12
#define nGATE_MC_GPIO_Port GPIOE
#define nGATE_HW_EN_Pin GPIO_PIN_13
#define nGATE_HW_EN_GPIO_Port GPIOE
#define nLASE_Q_PWR_SHDN_Pin GPIO_PIN_14
#define nLASE_Q_PWR_SHDN_GPIO_Port GPIOE
#define nLASE_Q_PWR_FLT_Pin GPIO_PIN_15
#define nLASE_Q_PWR_FLT_GPIO_Port GPIOE
#define LASE_Q_PWR_GOOD_Pin GPIO_PIN_10
#define LASE_Q_PWR_GOOD_GPIO_Port GPIOB
#define nPSU1_ON_Pin GPIO_PIN_11
#define nPSU1_ON_GPIO_Port GPIOB
#define nPSU2_ON_Pin GPIO_PIN_12
#define nPSU2_ON_GPIO_Port GPIOB
#define nPSU1_DC_OK_Pin GPIO_PIN_13
#define nPSU1_DC_OK_GPIO_Port GPIOB
#define PSU1_ALARM_Pin GPIO_PIN_14
#define PSU1_ALARM_GPIO_Port GPIOB
#define PSU1_ALARM_EXTI_IRQn EXTI15_10_IRQn
#define nPSU2_DC_OK_Pin GPIO_PIN_15
#define nPSU2_DC_OK_GPIO_Port GPIOB
#define PSU2_ALARM_Pin GPIO_PIN_8
#define PSU2_ALARM_GPIO_Port GPIOD
#define PSU2_ALARM_EXTI_IRQn EXTI9_5_IRQn
#define CONTACTOR_1_ON_Pin GPIO_PIN_9
#define CONTACTOR_1_ON_GPIO_Port GPIOD
#define MAIN_PWR_GOOD_Pin GPIO_PIN_10
#define MAIN_PWR_GOOD_GPIO_Port GPIOD
#define nMAIN_PWR_FLT_5V_Pin GPIO_PIN_11
#define nMAIN_PWR_FLT_5V_GPIO_Port GPIOD
#define nMAIN_PWR_FLT_5V_EXTI_IRQn EXTI15_10_IRQn
#define MAIN_PWR_SHDN_Pin GPIO_PIN_12
#define MAIN_PWR_SHDN_GPIO_Port GPIOD
#define nVEXT_GOOD_Pin GPIO_PIN_13
#define nVEXT_GOOD_GPIO_Port GPIOD
#define nEXT_CTL_iso_Pin GPIO_PIN_14
#define nEXT_CTL_iso_GPIO_Port GPIOD
#define nSYS_ON_iso_Pin GPIO_PIN_15
#define nSYS_ON_iso_GPIO_Port GPIOD
#define nENABLE_IN_iso_Pin GPIO_PIN_2
#define nENABLE_IN_iso_GPIO_Port GPIOG
#define nAIMING_iso_Pin GPIO_PIN_3
#define nAIMING_iso_GPIO_Port GPIOG
#define nCLR_ERR_iso_Pin GPIO_PIN_4
#define nCLR_ERR_iso_GPIO_Port GPIOG
#define nGATE_IN_iso_Pin GPIO_PIN_5
#define nGATE_IN_iso_GPIO_Port GPIOG
#define nGATE_IN_iso_EXTI_IRQn EXTI9_5_IRQn
#define nREDUCED_SETPOINT_RANGE_Pin GPIO_PIN_6
#define nREDUCED_SETPOINT_RANGE_GPIO_Port GPIOG
#define nCMD_RDY_iso_Pin GPIO_PIN_7
#define nCMD_RDY_iso_GPIO_Port GPIOG
#define nREADY_iso_Pin GPIO_PIN_8
#define nREADY_iso_GPIO_Port GPIOG
#define nEMISSION_RDY_iso_Pin GPIO_PIN_6
#define nEMISSION_RDY_iso_GPIO_Port GPIOC
#define nAIMING_ON_iso_Pin GPIO_PIN_7
#define nAIMING_ON_iso_GPIO_Port GPIOC
#define nERROR_iso_Pin GPIO_PIN_8
#define nERROR_iso_GPIO_Port GPIOC
#define nWATER_VALVE_STATUS_Pin GPIO_PIN_9
#define nWATER_VALVE_STATUS_GPIO_Port GPIOC
#define nEXT_485_TERMINATION_Pin GPIO_PIN_8
#define nEXT_485_TERMINATION_GPIO_Port GPIOA
#define EXT_485_Tx_Pin GPIO_PIN_9
#define EXT_485_Tx_GPIO_Port GPIOA
#define EXT_485_Rx_Pin GPIO_PIN_10
#define EXT_485_Rx_GPIO_Port GPIOA
#define CONTACTOR_2_ON_Pin GPIO_PIN_11
#define CONTACTOR_2_ON_GPIO_Port GPIOA
#define EXT_485_DIR_Pin GPIO_PIN_12
#define EXT_485_DIR_GPIO_Port GPIOA
#define COM_SPI_SCK_Pin GPIO_PIN_10
#define COM_SPI_SCK_GPIO_Port GPIOC
#define COM_SPI_MISO_Pin GPIO_PIN_11
#define COM_SPI_MISO_GPIO_Port GPIOC
#define COM_SPI_MOSI_Pin GPIO_PIN_12
#define COM_SPI_MOSI_GPIO_Port GPIOC
#define COM_SPI_nCS_Pin GPIO_PIN_0
#define COM_SPI_nCS_GPIO_Port GPIOD
#define nCOM_INT_IN_Pin GPIO_PIN_1
#define nCOM_INT_IN_GPIO_Port GPIOD
#define nCOM_INT_IN_EXTI_IRQn EXTI1_IRQn
#define nCOM_INT_OUT_Pin GPIO_PIN_2
#define nCOM_INT_OUT_GPIO_Port GPIOD
#define COM_PWR_GOOD_Pin GPIO_PIN_3
#define COM_PWR_GOOD_GPIO_Port GPIOD
#define nCOM_PWR_FLT_Pin GPIO_PIN_4
#define nCOM_PWR_FLT_GPIO_Port GPIOD
#define nCOM_PWR_FLT_EXTI_IRQn EXTI4_IRQn
#define nCOM_PWR_SHDN_Pin GPIO_PIN_5
#define nCOM_PWR_SHDN_GPIO_Port GPIOD
#define CONTACTOR_2_STATUS_Pin GPIO_PIN_6
#define CONTACTOR_2_STATUS_GPIO_Port GPIOD
#define CONTACTOR_1_STATUS_Pin GPIO_PIN_7
#define CONTACTOR_1_STATUS_GPIO_Port GPIOD
#define HW_SETPOINT_MC_Tx_Pin GPIO_PIN_9
#define HW_SETPOINT_MC_Tx_GPIO_Port GPIOG
#define LASE_Q_485_Rx_Pin GPIO_PIN_11
#define LASE_Q_485_Rx_GPIO_Port GPIOG
#define LASE_Q_485_Tx_Pin GPIO_PIN_12
#define LASE_Q_485_Tx_GPIO_Port GPIOG
#define HW_SETPOINT_MC_Rx_Pin GPIO_PIN_14
#define HW_SETPOINT_MC_Rx_GPIO_Port GPIOG
#define nSD_CS_Pin GPIO_PIN_15
#define nSD_CS_GPIO_Port GPIOG
#define nSD_PRESENT_Pin GPIO_PIN_5
#define nSD_PRESENT_GPIO_Port GPIOB
#define FLOW_METER_1_PULSE_Pin GPIO_PIN_6
#define FLOW_METER_1_PULSE_GPIO_Port GPIOB
#define FLOW_METER_2_PULSE_Pin GPIO_PIN_7
#define FLOW_METER_2_PULSE_GPIO_Port GPIOB
#define nLED_RED_Pin GPIO_PIN_8
#define nLED_RED_GPIO_Port GPIOB
#define nLED_GREEN_Pin GPIO_PIN_9
#define nLED_GREEN_GPIO_Port GPIOB
#define nLED_BLUE_Pin GPIO_PIN_0
#define nLED_BLUE_GPIO_Port GPIOE
#define nWATER_VALVE_EN_Pin GPIO_PIN_1
#define nWATER_VALVE_EN_GPIO_Port GPIOE

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
