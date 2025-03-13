/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32g4xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "math.h"
#include "stdlib.h"
#include "stdio.h"
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

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
extern double omega;
extern uint16_t adc_datas[2];
/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern FDCAN_HandleTypeDef hfdcan1;
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
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
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

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32G4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32g4xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles DMA1 channel1 global interrupt.
  */
void DMA1_Channel1_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Channel1_IRQn 0 */
  	if( LL_DMA_IsActiveFlag_TC1(DMA1) == 1){
		LL_DMA_ClearFlag_TC1(DMA1);
	}

  /* USER CODE END DMA1_Channel1_IRQn 0 */

  /* USER CODE BEGIN DMA1_Channel1_IRQn 1 */

  /* USER CODE END DMA1_Channel1_IRQn 1 */
}

/**
  * @brief This function handles FDCAN1 interrupt 0.
  */
void FDCAN1_IT0_IRQHandler(void)
{
  /* USER CODE BEGIN FDCAN1_IT0_IRQn 0 */

  /* USER CODE END FDCAN1_IT0_IRQn 0 */
  HAL_FDCAN_IRQHandler(&hfdcan1);
  /* USER CODE BEGIN FDCAN1_IT0_IRQn 1 */

  /* USER CODE END FDCAN1_IT0_IRQn 1 */
}

/**
  * @brief This function handles TIM1 trigger and commutation interrupts and TIM17 global interrupt.
  */
void TIM1_TRG_COM_TIM17_IRQHandler(void)
{
  /* USER CODE BEGIN TIM1_TRG_COM_TIM17_IRQn 0 */

  /* USER CODE END TIM1_TRG_COM_TIM17_IRQn 0 */

  /* USER CODE BEGIN TIM1_TRG_COM_TIM17_IRQn 1 */

  /* USER CODE END TIM1_TRG_COM_TIM17_IRQn 1 */
}

/**
  * @brief This function handles TIM6 global interrupt, DAC1 and DAC3 channel underrun error interrupts.
  */
void TIM6_DAC_IRQHandler(void)
{
  /* USER CODE BEGIN TIM6_DAC_IRQn 0 */
	static float theta = 0.0f;

  /* USER CODE END TIM6_DAC_IRQn 0 */

  /* USER CODE BEGIN TIM6_DAC_IRQn 1 */
	if(LL_TIM_IsActiveFlag_UPDATE(TIM6)){
		theta += omega;
		if(theta > 360.0f){
			theta -= 360.0f;
		}

		if(omega < 5.0){
			rotateSin(theta, 500u);
			//rotate120Deg(theta, 200u);
		}else{
			//rotate120Deg(theta, 400u);
			rotateSin(theta, 300u);
		}

		LL_TIM_ClearFlag_UPDATE(TIM6);
	}

  /* USER CODE END TIM6_DAC_IRQn 1 */
}

/* USER CODE BEGIN 1 */
void rotate120Deg(float theta_, uint16_t power_){
	static uint8_t state = 0u;
	switch(state){
		case 0:		//330~30
			if(theta_ > 30.0f){
				LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH1);
				LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH1N);
				LL_TIM_OC_SetCompareCH1(TIM1, power_);
				LL_TIM_CC_DisableChannel(TIM1, LL_TIM_CHANNEL_CH3);
				LL_TIM_CC_DisableChannel(TIM1, LL_TIM_CHANNEL_CH3N);
				state++;
			}
			break;
		case 1:		//60~120
			if(theta_ > 90.0f){
				LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH3);
				LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH3N);
				LL_TIM_OC_SetCompareCH3(TIM1, 0);
				LL_TIM_CC_DisableChannel(TIM1, LL_TIM_CHANNEL_CH2);
				LL_TIM_CC_DisableChannel(TIM1, LL_TIM_CHANNEL_CH2N);
				state++;
			}
			break;
		case 2:		//120~180
			if(theta_ > 150.0f){
				LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH2);
				LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH2N);
				LL_TIM_OC_SetCompareCH2(TIM1, power_);
				LL_TIM_CC_DisableChannel(TIM1, LL_TIM_CHANNEL_CH1);
				LL_TIM_CC_DisableChannel(TIM1, LL_TIM_CHANNEL_CH1N);
				state++;
			}
			break;
		case 3:		//180~240
			if(theta_ > 210.0f){
				LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH1);
				LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH1N);
				LL_TIM_OC_SetCompareCH1(TIM1, 0);
				LL_TIM_CC_DisableChannel(TIM1, LL_TIM_CHANNEL_CH3);
				LL_TIM_CC_DisableChannel(TIM1, LL_TIM_CHANNEL_CH3N);
				state++;
			}
			break;
		case 4:		//240~300;
			if(theta_ > 270.0f){
				LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH3);
				LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH3N);
				LL_TIM_OC_SetCompareCH3(TIM1, power_);
				LL_TIM_CC_DisableChannel(TIM1, LL_TIM_CHANNEL_CH2);
				LL_TIM_CC_DisableChannel(TIM1, LL_TIM_CHANNEL_CH2N);
				state++;
			}
			break;
		case 5:		//300~360;
			if(theta_ > 330.0f){
				LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH2);
				LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH2N);
				LL_TIM_OC_SetCompareCH2(TIM1, 0);
				LL_TIM_CC_DisableChannel(TIM1, LL_TIM_CHANNEL_CH1);
				LL_TIM_CC_DisableChannel(TIM1, LL_TIM_CHANNEL_CH1N);
			}
			if(theta_<270.0f){
				state = 0u;
			}
			break;
	}
}

void rotateSin(float theta_, uint16_t power_){
	float pwm[3];
	int32_t theta = theta_>180.0f ? (int32_t)((theta_-360.0f)/180.0f*2147483648.0f) : (int32_t)((theta_)/180.0f*2147483648.0f);

	LL_CORDIC_SetFunction(CORDIC, LL_CORDIC_FUNCTION_SINE);
	LL_CORDIC_WriteData(CORDIC, (uint32_t)theta);
	LL_CORDIC_WriteData(CORDIC, 0x7FFFFFFF);	

	pwm[1] = sin((theta_+120.0f) * M_PI / 180.0f) + 1.0f;
	pwm[2] = sin((theta_+240.0f) * M_PI / 180.0f) + 1.0f;

	LL_TIM_OC_SetCompareCH2(TIM1, (uint16_t)(power_*pwm[1])>>1);
	LL_TIM_OC_SetCompareCH3(TIM1, (uint16_t)(power_*pwm[2])>>1);

	while(!LL_CORDIC_IsActiveFlag_RRDY(CORDIC));
	int32_t tmp = LL_CORDIC_ReadData(CORDIC);
	uint16_t value = (((tmp >> 10)*power_)>>22) + (power_>>1);
	LL_CORDIC_ReadData(CORDIC);
	LL_TIM_OC_SetCompareCH1(TIM1, value);

	LL_TIM_CC_EnableChannel(TIM1,LL_TIM_CHANNEL_CH1);
	LL_TIM_CC_EnableChannel(TIM1,LL_TIM_CHANNEL_CH1N);
	LL_TIM_CC_EnableChannel(TIM1,LL_TIM_CHANNEL_CH2);
	LL_TIM_CC_EnableChannel(TIM1,LL_TIM_CHANNEL_CH2N);
	LL_TIM_CC_EnableChannel(TIM1,LL_TIM_CHANNEL_CH3);
	LL_TIM_CC_EnableChannel(TIM1,LL_TIM_CHANNEL_CH3N);
}

/* USER CODE END 1 */
