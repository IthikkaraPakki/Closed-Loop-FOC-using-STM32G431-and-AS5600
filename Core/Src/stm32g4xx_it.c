/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32g4xx_it.c
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
#include "stm32g4xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "closedloop.h"
#include <stdlib.h>
#include <ctype.h>
#include <string.h>
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
volatile uint32_t isr_count = 0;
volatile uint32_t isr_frequency = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */
void SEGGER_SYSVIEW_RecordExitISR(void);
void SEGGER_SYSVIEW_RecordEnterISR(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern TIM_HandleTypeDef htim4;

/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M4 Processor Interruption and Exception Handlers          */
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
  * @brief This function handles Prefetch fault, memory access fault.
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
/* STM32G4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32g4xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles TIM1 update interrupt and TIM16 global interrupt.
  */
void TIM1_UP_TIM16_IRQHandler(void)
{
  /* USER CODE BEGIN TIM1_UP_TIM16_IRQn 0 */
  SEGGER_SYSVIEW_RecordEnterISR();
  static uint8_t error = 1;
  isr_count++;
  /* USER CODE END TIM1_UP_TIM16_IRQn 0 */
  /* USER CODE BEGIN TIM1_UP_TIM16_IRQn 1 */
  switch (system_state)
  {

  case STATE_CALIBRATION:

	   if(MoveCommand)
	   {
		  Run_Open_Loop();
	   }
	   break;

  case STATE_OPEN_LOOP:
	   Run_Open_Loop();
	   break;

  case STATE_CLOSED_LOOP:
	   Run_Closed_Loop();
	   break;

  case STATE_FAULT:
	  if(error)
	  {	   // Stop motor - set neutral PWM
	       LL_TIM_OC_SetCompareCH1(TIM1, PWM_ARR / 2);
	       LL_TIM_OC_SetCompareCH2(TIM1, PWM_ARR / 2);
	       LL_TIM_OC_SetCompareCH3(TIM1, PWM_ARR / 2);

	       LL_TIM_DisableAllOutputs(TIM1);

	       error = 0;
	  }
	  break;
  }

  LL_TIM_ClearFlag_UPDATE(TIM1);

  SEGGER_SYSVIEW_RecordExitISR();
  /* USER CODE END TIM1_UP_TIM16_IRQn 1 */
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
  * @brief This function handles USART2 global interrupt / USART2 wake-up interrupt through EXTI line 26.
  */
void USART2_IRQHandler(void)
{
  /* USER CODE BEGIN USART2_IRQn 0 */

	if (LL_USART_IsActiveFlag_ORE(USART2) || LL_USART_IsActiveFlag_FE(USART2))
	    {
	        LL_USART_ClearFlag_ORE(USART2);
	        LL_USART_ClearFlag_FE(USART2);
	        // Won't return, continue to check if there is valid data too
	    }

	if (LL_USART_IsActiveFlag_RXNE(USART2))
	    {
	        char c = LL_USART_ReceiveData8(USART2);
	        static char cmd_buf[12];
	        static uint8_t cmd_idx = 0;

	        // 1. Process on Enter keys
	        if (c == '\n' || c == '\r')
	        {
	            if (cmd_idx > 0)
	            {
	                cmd_buf[cmd_idx] = '\0';

	                // Find where 'R' actually is in the buffer (handles leading spaces)
	                char *r_ptr = strchr(cmd_buf, 'R');

	                // VALIDATION:
	                // - 'R' must exist
	                // - The character immediately after 'R' must be a digit or '-'
	                if (r_ptr != NULL && (isdigit((unsigned char)r_ptr[1]) || r_ptr[1] == '-'))
	                {
	                    // Additional Strictness: Ensure it's not "RPM"
	                    // Check that the char after 'R' isn't 'P'
	                    if (r_ptr[1] != 'P')
	                    {
	                    	speed_ref_rpm = (float)atoi(r_ptr + 1);
	                    }
	                }
	            }
	            cmd_idx = 0;
	        }
	        // 2. Collect characters (Ignore only the terminators)
	        else if (cmd_idx < 11)
	        {
	            cmd_buf[cmd_idx++] = c;
	        }
	    }

  /* USER CODE END USART2_IRQn 0 */
  /* USER CODE BEGIN USART2_IRQn 1 */

	// TX interrupt - send next byte
    if (LL_USART_IsActiveFlag_TXE(USART2))
    {
        if (usart_tx_tail != usart_tx_head)  // Buffer not empty?
        {
            LL_USART_TransmitData8(USART2, usart_tx_buffer[usart_tx_tail]);
            usart_tx_tail = (usart_tx_tail + 1) % USART_TX_BUFFER_SIZE;
        }
        else  // Buffer empty
        {
            LL_USART_DisableIT_TXE(USART2);  // Stop interrupts
            usart_tx_busy = 0;
        }
    }

  /* USER CODE END USART2_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
