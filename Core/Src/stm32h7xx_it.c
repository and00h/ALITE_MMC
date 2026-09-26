/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32h7xx_it.c
  * @brief   Interrupt Service Routines.
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
#include "main.h"
#include "stm32h7xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "FlowMeter.h"
#include "EXT_interface.h"
#include "PSU.h"
#include "LaseQ.h"
#include "FPButton.h"
#include "SAB.h"
#include "rs485_cmd.h"
#include "AD7490.h"
#include "SHT35.h"
#include "AMC.h"
#include "COM_interface.h"
/* QCW.h non incluso qui: TIM16_IRQHandler() e' generato da CubeMX (chiama
 * HAL_TIM_IRQHandler(&htim16) direttamente) — vedi QCW.h per la catena
 * di dispatch completa fino a QCW_TimerCallback() in main.c. */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/*
 * Snapshot del contesto al momento dell'HardFault, popolato da
 * HardFault_C_Handler() (vedi USER CODE 0 sotto). Ispezionabile a
 * runtime nel debugger (Expressions / Live Watch, variabile globale
 * g_faultInfo) oppure via BKPT quando si ferma in HardFault_C_Handler.
 *
 * cfsr: bit [7:0]  = MMFSR (MemManage)
 *       bit [15:8] = BFSR  (BusFault)  -> bit1 PRECISERR, bit2 IMPRECISERR,
 *                    bit7 BFARVALID (bfar valido solo se questo è 1)
 *       bit [31:16]= UFSR  (UsageFault)
 * hfsr: bit30 FORCED = fault escalato da altro handler (vedi cfsr per la causa reale)
 */
typedef struct {
    uint32_t r0, r1, r2, r3, r12, lr, pc, psr;
    uint32_t cfsr, hfsr, mmfar, bfar, afsr;
    uint32_t exc_return;
} HardFaultInfo_t;

volatile HardFaultInfo_t g_faultInfo;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

void HardFault_C_Handler(uint32_t *pFaultStack, uint32_t exc_return);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/*
 * @brief  Estrae dallo stack frame di eccezione (impilato in HW da MSP o PSP,
 *         a seconda di EXC_RETURN) i registri e i fault status register,
 *         li salva in g_faultInfo e ferma l'esecuzione su BKPT.
 *         Nel debugger: Window > Show View > Expressions, aggiungere
 *         "g_faultInfo", poi ispezionare cfsr/hfsr/bfar/mmfar/pc.
 *
 *         cfsr bit9 (IMPRECISERR) = 1 → il fault è asincrono: pc/lr in
 *         g_faultInfo NON sono l'istruzione che ha causato il fault (tipico
 *         su H7 con write-buffer, es. scritture DMA/periferiche pochi cicli
 *         prima). In quel caso guardare cosa viene chiamato subito prima nel
 *         codice sorgente, non il pc riportato.
 */
void HardFault_C_Handler(uint32_t *pFaultStack, uint32_t exc_return)
{
    g_faultInfo.r0  = pFaultStack[0];
    g_faultInfo.r1  = pFaultStack[1];
    g_faultInfo.r2  = pFaultStack[2];
    g_faultInfo.r3  = pFaultStack[3];
    g_faultInfo.r12 = pFaultStack[4];
    g_faultInfo.lr  = pFaultStack[5];   /* return address del codice interrotto */
    g_faultInfo.pc  = pFaultStack[6];   /* istruzione che stava eseguendo (vedi nota IMPRECISERR) */
    g_faultInfo.psr = pFaultStack[7];

    g_faultInfo.cfsr  = SCB->CFSR;
    g_faultInfo.hfsr  = SCB->HFSR;
    g_faultInfo.mmfar = SCB->MMFAR;
    g_faultInfo.bfar  = SCB->BFAR;
    g_faultInfo.afsr  = SCB->AFSR;
    g_faultInfo.exc_return = exc_return;

    __asm volatile ("BKPT #01");
    while (1) { }
}

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern I2C_HandleTypeDef hi2c2;
extern DMA_HandleTypeDef hdma_spi3_rx;
extern DMA_HandleTypeDef hdma_spi3_tx;
extern SPI_HandleTypeDef hspi3;
extern SPI_HandleTypeDef hspi5;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim16;
extern DMA_HandleTypeDef hdma_usart1_rx;
extern DMA_HandleTypeDef hdma_usart1_tx;
extern DMA_HandleTypeDef hdma_usart10_rx;
extern DMA_HandleTypeDef hdma_usart10_tx;
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart6;
extern UART_HandleTypeDef huart10;
extern TIM_HandleTypeDef htim1;

/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
   while (1)
  {
  }
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /*
   * Bit2 di LR (EXC_RETURN) indica se lo stack frame di eccezione è su
   * MSP o PSP. Nessuna chiamata a funzione prima di qui: LR contiene
   * ancora l'EXC_RETURN originale impostato dall'hardware all'ingresso
   * dell'eccezione. Passa puntatore allo stack frame + EXC_RETURN a
   * HardFault_C_Handler() (USER CODE 0 sopra) per l'analisi.
   */
  __asm volatile
  (
    " tst lr, #4                                                \n"
    " ite eq                                                    \n"
    " mrseq r0, msp                                             \n"
    " mrsne r0, psp                                             \n"
    " mov r1, lr                                                \n"
    " b HardFault_C_Handler                                     \n"
  );
  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Pre-fetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/******************************************************************************/
/* STM32H7xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32h7xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles EXTI line0 interrupt.
  */
void EXTI0_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI0_IRQn 0 */

  /* USER CODE END EXTI0_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(nSAB_INTLCK_A_STATUS_Pin);
  /* USER CODE BEGIN EXTI0_IRQn 1 */

  /* USER CODE END EXTI0_IRQn 1 */
}

/**
  * @brief This function handles EXTI line1 interrupt.
  */
void EXTI1_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI1_IRQn 0 */

  /* USER CODE END EXTI1_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(nCOM_INT_IN_Pin);
  /* USER CODE BEGIN EXTI1_IRQn 1 */

  /* USER CODE END EXTI1_IRQn 1 */
}

/**
  * @brief This function handles EXTI line2 interrupt.
  */
void EXTI2_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI2_IRQn 0 */

  /* USER CODE END EXTI2_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(nSAB_INTLCK_B_STATUS_Pin);
  /* USER CODE BEGIN EXTI2_IRQn 1 */

  /* USER CODE END EXTI2_IRQn 1 */
}

/**
  * @brief This function handles EXTI line4 interrupt.
  */
void EXTI4_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI4_IRQn 0 */

  /* USER CODE END EXTI4_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(nCOM_PWR_FLT_Pin);
  /* USER CODE BEGIN EXTI4_IRQn 1 */

  /* USER CODE END EXTI4_IRQn 1 */
}

/**
  * @brief This function handles DMA1 stream0 global interrupt.
  */
void DMA1_Stream0_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Stream0_IRQn 0 */

  /* USER CODE END DMA1_Stream0_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart1_rx);
  /* USER CODE BEGIN DMA1_Stream0_IRQn 1 */

  /* USER CODE END DMA1_Stream0_IRQn 1 */
}

/**
  * @brief This function handles DMA1 stream1 global interrupt.
  */
void DMA1_Stream1_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Stream1_IRQn 0 */

  /* USER CODE END DMA1_Stream1_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart1_tx);
  /* USER CODE BEGIN DMA1_Stream1_IRQn 1 */

  /* USER CODE END DMA1_Stream1_IRQn 1 */
}

/**
  * @brief This function handles DMA1 stream2 global interrupt.
  */
void DMA1_Stream2_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Stream2_IRQn 0 */

  /* USER CODE END DMA1_Stream2_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart10_rx);
  /* USER CODE BEGIN DMA1_Stream2_IRQn 1 */

  /* USER CODE END DMA1_Stream2_IRQn 1 */
}

/**
  * @brief This function handles DMA1 stream3 global interrupt.
  */
void DMA1_Stream3_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Stream3_IRQn 0 */

  /* USER CODE END DMA1_Stream3_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart10_tx);
  /* USER CODE BEGIN DMA1_Stream3_IRQn 1 */

  /* USER CODE END DMA1_Stream3_IRQn 1 */
}

/**
  * @brief This function handles DMA1 stream4 global interrupt.
  */
void DMA1_Stream4_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Stream4_IRQn 0 */

  /* USER CODE END DMA1_Stream4_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_spi3_rx);
  /* USER CODE BEGIN DMA1_Stream4_IRQn 1 */

  /* USER CODE END DMA1_Stream4_IRQn 1 */
}

/**
  * @brief This function handles DMA1 stream5 global interrupt.
  */
void DMA1_Stream5_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Stream5_IRQn 0 */

  /* USER CODE END DMA1_Stream5_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_spi3_tx);
  /* USER CODE BEGIN DMA1_Stream5_IRQn 1 */

  /* USER CODE END DMA1_Stream5_IRQn 1 */
}

/**
  * @brief This function handles EXTI line[9:5] interrupts.
  */
void EXTI9_5_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI9_5_IRQn 0 */

  /* USER CODE END EXTI9_5_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(nGATE_IN_iso_Pin);
  HAL_GPIO_EXTI_IRQHandler(PSU2_ALARM_Pin);
  /* USER CODE BEGIN EXTI9_5_IRQn 1 */

  /* USER CODE END EXTI9_5_IRQn 1 */
}

/**
  * @brief This function handles TIM1 update interrupt.
  */
void TIM1_UP_IRQHandler(void)
{
  /* USER CODE BEGIN TIM1_UP_IRQn 0 */

  /* USER CODE END TIM1_UP_IRQn 0 */
  HAL_TIM_IRQHandler(&htim1);
  /* USER CODE BEGIN TIM1_UP_IRQn 1 */

  /* USER CODE END TIM1_UP_IRQn 1 */
}

/**
  * @brief This function handles TIM4 global interrupt.
  */
void TIM4_IRQHandler(void)
{
  /* USER CODE BEGIN TIM4_IRQn 0 */

  /* USER CODE END TIM4_IRQn 0 */
  HAL_TIM_IRQHandler(&htim4);
  /* USER CODE BEGIN TIM4_IRQn 1 */

  /* USER CODE END TIM4_IRQn 1 */
}

/**
  * @brief This function handles I2C2 event interrupt.
  */
void I2C2_EV_IRQHandler(void)
{
  /* USER CODE BEGIN I2C2_EV_IRQn 0 */

  /* USER CODE END I2C2_EV_IRQn 0 */
  HAL_I2C_EV_IRQHandler(&hi2c2);
  /* USER CODE BEGIN I2C2_EV_IRQn 1 */

  /* USER CODE END I2C2_EV_IRQn 1 */
}

/**
  * @brief This function handles I2C2 error interrupt.
  */
void I2C2_ER_IRQHandler(void)
{
  /* USER CODE BEGIN I2C2_ER_IRQn 0 */

  /* USER CODE END I2C2_ER_IRQn 0 */
  HAL_I2C_ER_IRQHandler(&hi2c2);
  /* USER CODE BEGIN I2C2_ER_IRQn 1 */

  /* USER CODE END I2C2_ER_IRQn 1 */
}

/**
  * @brief This function handles USART1 global interrupt.
  */
void USART1_IRQHandler(void)
{
  /* USER CODE BEGIN USART1_IRQn 0 */

  /* USER CODE END USART1_IRQn 0 */
  HAL_UART_IRQHandler(&huart1);
  /* USER CODE BEGIN USART1_IRQn 1 */

  /* USER CODE END USART1_IRQn 1 */
}

/**
  * @brief This function handles EXTI line[15:10] interrupts.
  */
void EXTI15_10_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI15_10_IRQn 0 */

  /* USER CODE END EXTI15_10_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(nHW_SETPOINT_FLT_Pin);
  HAL_GPIO_EXTI_IRQHandler(nMAIN_PWR_FLT_5V_Pin);
  HAL_GPIO_EXTI_IRQHandler(PSU1_ALARM_Pin);
  HAL_GPIO_EXTI_IRQHandler(nFP_BUTTON_Pin);
  /* USER CODE BEGIN EXTI15_10_IRQn 1 */

  /* USER CODE END EXTI15_10_IRQn 1 */
}

/**
  * @brief This function handles SPI3 global interrupt.
  */
void SPI3_IRQHandler(void)
{
  /* USER CODE BEGIN SPI3_IRQn 0 */

  /* USER CODE END SPI3_IRQn 0 */
  HAL_SPI_IRQHandler(&hspi3);
  /* USER CODE BEGIN SPI3_IRQn 1 */

  /* USER CODE END SPI3_IRQn 1 */
}

/**
  * @brief This function handles USART6 global interrupt.
  */
void USART6_IRQHandler(void)
{
  /* USER CODE BEGIN USART6_IRQn 0 */

  /* USER CODE END USART6_IRQn 0 */
  HAL_UART_IRQHandler(&huart6);
  /* USER CODE BEGIN USART6_IRQn 1 */

  /* USER CODE END USART6_IRQn 1 */
}

/**
  * @brief This function handles SPI5 global interrupt.
  */
void SPI5_IRQHandler(void)
{
  /* USER CODE BEGIN SPI5_IRQn 0 */

  /* USER CODE END SPI5_IRQn 0 */
  HAL_SPI_IRQHandler(&hspi5);
  /* USER CODE BEGIN SPI5_IRQn 1 */

  /* USER CODE END SPI5_IRQn 1 */
}

/**
  * @brief This function handles TIM16 global interrupt.
  */
void TIM16_IRQHandler(void)
{
  /* USER CODE BEGIN TIM16_IRQn 0 */

  /* USER CODE END TIM16_IRQn 0 */
  HAL_TIM_IRQHandler(&htim16);
  /* USER CODE BEGIN TIM16_IRQn 1 */

  /* USER CODE END TIM16_IRQn 1 */
}

/**
  * @brief This function handles USART10 global interrupt.
  */
void USART10_IRQHandler(void)
{
  /* USER CODE BEGIN USART10_IRQn 0 */

  /* USER CODE END USART10_IRQn 0 */
  HAL_UART_IRQHandler(&huart10);
  /* USER CODE BEGIN USART10_IRQn 1 */

  /* USER CODE END USART10_IRQn 1 */
}

/* USER CODE BEGIN 1 */
// Input capture callbacks per rimando a moduli custom
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
	//Flow meters
	if (htim->Instance == TIM4){
		FlowMeter_CaptureCallback(htim);
	}
}

// Callback UART per rimando a moduli custom
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    LaseQ_TxCpltCallback(huart);
    /* USART1 RS485: notifica task che il DMA TX è completato */
    if (huart->Instance == USART1) {
        Rs485Cmd_OnTxCplt();
    }
    /* USART6: AMC, IT (non DMA — nessuno stream DMA configurato per
     * USART6 in CubeMX), vedi AMC.c */
    if (huart->Instance == USART6) {
        AMC_ITTxCallback(huart);
    }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    LaseQ_RxCpltCallback(huart);
    /* USART1 usa DMA+IDLE → HAL_UARTEx_RxEventCallback, non questo */
    if (huart->Instance == USART6) {
        AMC_ITRxCallback(huart);
    }
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    /*
     * Callback DMA+IDLE per USART1 RS485.
     * Scatta su: IDLE line (frame ASCII breve), HT (disabilitato), TC (buffer pieno).
     * Size = byte ricevuti nel buffer DMA dall'ultimo avvio.
     */
    if (huart->Instance == USART1) {
        Rs485Cmd_OnRxEvent(Size);
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    /*
     * ORE/FE/NE su USART1: HAL chiama UART_EndRxTransfer → DMA/RXNEIE
     * disabilitati.  Senza questo override la ricezione resta morta.
     */
    if (huart->Instance == USART1) {
        Rs485Cmd_OnError();
    }
    if (huart->Instance == USART6) {
        AMC_ITErrorCallback(huart);
    }
    /* USART10: LaseQ, DMA half-duplex. Senza questo override, su
     * ORE/FE/NE il DMA abortito da HAL non verrebbe mai segnalato al
     * task Comms, che resterebbe ad attendere il timeout pieno di
     * laseq_transact() ad ogni errore bus invece di essere sbloccato
     * subito (vedi LaseQ_ErrorCallback). */
    if (huart->Instance == USART10) {
        LaseQ_ErrorCallback(huart);
    }
}

/*
 * Callback SPI/I2C per AD7490 e SHT35: entrambi i driver sono passati da
 * polling bloccante a IT + semaforo FreeRTOS (vedi AD7490.c/SHT35.c).
 * Come per le UART sopra, il dispatch per istanza vive qui; il modulo
 * specifico si limita a segnalare il proprio semaforo.
 */
void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
    if (hspi->Instance == SPI5) {
        AD7490_ITCallback();
    }
    if (hspi->Instance == SPI3) {
        COM_Interface_SPI_Cplt_Callback();
    }
}

void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi)
{
    if (hspi->Instance == SPI5) {
        AD7490_ITErrorCallback();
    }
    if (hspi->Instance == SPI3) {
        COM_Interface_SPI_Error_Callback();
    }
}

void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    if (hi2c->Instance == I2C2) {
        SHT35_ITTxCallback();
    }
}

void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    if (hi2c->Instance == I2C2) {
        SHT35_ITRxCallback();
    }
}

void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c)
{
    if (hi2c->Instance == I2C2) {
        SHT35_ITErrorCallback();
    }
}

//Callback EXTI per rimando a moduli custom
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    //Rimando alle callback specifiche per i vari moduli

	//EXT interface
	EXT_Interface_Callback(GPIO_Pin);
	//Flood sensors

	//SAB module
	if (GPIO_Pin == nSAB_INTLCK_A_STATUS_Pin) { SAB_IntlckA_EXTI_Callback(); }
	if (GPIO_Pin == nSAB_INTLCK_B_STATUS_Pin) { SAB_IntlckB_EXTI_Callback(); }

	//LASE Q

	//PSU
	PSU_Interface_Callback(GPIO_Pin);

	//Front Panel
	if (GPIO_Pin == nFP_BUTTON_Pin) { FPButton_EXTI_Callback(); }

	//COM interface
	if (GPIO_Pin == nCOM_INT_IN_Pin)  { COM_Interface_MsgReady_EXTI_Callback(); }
	if (GPIO_Pin == nCOM_PWR_FLT_Pin) { COM_Interface_PwrFlt_EXTI_Callback(); }

	//Board supply

	//AMC

	//SD card

	//Contactors

}
/* USER CODE END 1 */
