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
#define KP (int32_t)(0.5 * 1024)		//0.5
#define KI (int32_t)(0.002 * 1024)		//0.002

#define FREQ_T (int32_t)(1024/32)		//T[ms] (32kHz)

#define RAD0   (int32_t)(  0*M_PI/180*1024)	//  0deg
#define RAD30  (int32_t)( 30*M_PI/180*1024)	// 30deg
#define RAD60  (int32_t)( 60*M_PI/180*1024)	// 60deg
#define RAD90  (int32_t)( 90*M_PI/180*1024)	// 90deg
#define RAD120 (int32_t)(120*M_PI/180*1024)	//120deg
#define RAD150 (int32_t)(150*M_PI/180*1024)	//150deg
#define RAD180 (int32_t)(180*M_PI/180*1024)	//180deg
#define RAD210 (int32_t)(210*M_PI/180*1024)	//210deg
#define RAD240 (int32_t)(240*M_PI/180*1024)	//240deg
#define RAD270 (int32_t)(270*M_PI/180*1024)	//270deg
#define RAD300 (int32_t)(300*M_PI/180*1024)	//300deg
#define RAD330 (int32_t)(330*M_PI/180*1024)	//330deg
#define RAD360 (int32_t)(360*M_PI/180*1024)	//360deg
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
extern RingBuf pi_u;		//for pi control
extern RingBuf pi_e;		//for pi control

extern int32_t omg_tg;
extern int32_t omg_est;
extern int32_t adv_ang;
extern int32_t rot_est;
extern uint16_t adc_datas[2];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/

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
  * @brief This function handles DMA1 channel2 global interrupt.
  */
void DMA1_Channel2_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Channel2_IRQn 0 */

  /* USER CODE END DMA1_Channel2_IRQn 0 */

  /* USER CODE BEGIN DMA1_Channel2_IRQn 1 */

  /* USER CODE END DMA1_Channel2_IRQn 1 */
}

/**
  * @brief This function handles DMA1 channel3 global interrupt.
  */
void DMA1_Channel3_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Channel3_IRQn 0 */

  /* USER CODE END DMA1_Channel3_IRQn 0 */

  /* USER CODE BEGIN DMA1_Channel3_IRQn 1 */

  /* USER CODE END DMA1_Channel3_IRQn 1 */
}

/**
  * @brief This function handles DMA1 channel4 global interrupt.
  */
void DMA1_Channel4_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Channel4_IRQn 0 */

  /* USER CODE END DMA1_Channel4_IRQn 0 */

  /* USER CODE BEGIN DMA1_Channel4_IRQn 1 */

  /* USER CODE END DMA1_Channel4_IRQn 1 */
}

/**
  * @brief This function handles TIM1 update interrupt and TIM16 global interrupt.
  */
void TIM1_UP_TIM16_IRQHandler(void)
{
  /* USER CODE BEGIN TIM1_UP_TIM16_IRQn 0 */

  /* USER CODE END TIM1_UP_TIM16_IRQn 0 */

  /* USER CODE BEGIN TIM1_UP_TIM16_IRQn 1 */

  /* USER CODE END TIM1_UP_TIM16_IRQn 1 */
}

/**
  * @brief This function handles COMP1, COMP2 and COMP3 interrupts through EXTI lines 21, 22 and 29.
  */
void COMP1_2_3_IRQHandler(void)
{
  /* USER CODE BEGIN COMP1_2_3_IRQn 0 */
  if(LL_EXTI_IsActiveFlag_0_31(LL_EXTI_LINE_21)){
	LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_21);
	LL_EXTI_DisableRisingTrig_0_31(LL_EXTI_LINE_21);
  }

  /* USER CODE END COMP1_2_3_IRQn 0 */

  /* USER CODE BEGIN COMP1_2_3_IRQn 1 */

  /* USER CODE END COMP1_2_3_IRQn 1 */
}

/* USER CODE BEGIN 1 */
void forceRotate(float theta_, uint16_t power_){
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

static inline void rotate120Deg(int32_t *rot_est_, int32_t omg_est_, int32_t adv_ang_, RingBuf *pi_e_, RingBuf *pi_u_){
	(*rot_est_) += (omg_est_*FREQ_T) >> 10;
	int32_t ctl_ang = (*rot_est_) + adv_ang_;

	ctl_ang += ctl_ang < RAD0 ? RAD360 : RAD0;
	ctl_ang -= ctl_ang > RAD360 ? RAD360 : RAD0;

	//PI control section
	pi_e_->data[pi_e_->pnt] = omg_tg - omg_est;
	pi_u_->data[pi_u_->pnt] = 2*pi_u_->data[(pi_u_->pnt+0x03)|0x03] - pi_u_->data[(pi_u_->pnt+0x02)|0x03] - (KP*pi_e_->data[(pi_e_->pnt+0x03)|0x03])>>10 + ((KP+KI)*pi_e_->data[pi_e_->pnt])>>10;

	pi_u_->data[pi_u_->pnt] = pi_u_->data[pi_u_->pnt] > 999<<10 ? 999<<10 : pi_u_->data[pi_u_->pnt];

	if(ctl_ang >= RAD30 && ctl_ang < RAD90){
		LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH1);
		LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH1N);
		LL_TIM_OC_SetCompareCH1(TIM1, pi_u_->data[pi_u_->pnt]>>10);

		LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH2);
		LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH2N);
		LL_TIM_OC_SetCompareCH2(TIM1, 0);

		LL_TIM_CC_DisableChannel(TIM1, LL_TIM_CHANNEL_CH3);
		LL_TIM_CC_DisableChannel(TIM1, LL_TIM_CHANNEL_CH3N);
	}else if(ctl_ang >= RAD90 && ctl_ang < RAD150){
		LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH1);
		LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH1N);
		LL_TIM_OC_SetCompareCH1(TIM1, pi_u_->data[pi_u_->pnt]>>10);

		LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH2);
		LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH2N);

		LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH3);
		LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH3N);
		LL_TIM_OC_SetCompareCH3(TIM1, 0);
	}else if(ctl_ang >= RAD150 && ctl_ang < RAD210){
		LL_TIM_CC_DisableChannel(TIM1, LL_TIM_CHANNEL_CH1);
		LL_TIM_CC_DisableChannel(TIM1, LL_TIM_CHANNEL_CH1N);

		LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH2);
		LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH2N);
		LL_TIM_OC_SetCompareCH2(TIM1, pi_u_->data[pi_u_->pnt]>>10);

		LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH3);
		LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH3N);
		LL_TIM_OC_SetCompareCH3(TIM1, 0);
	}else if(ctl_ang >= RAD210 && ctl_ang < RAD270){
		LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH1);
		LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH1N);
		LL_TIM_OC_SetCompareCH1(TIM1, 0);

		LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH2);
		LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH2N);
		LL_TIM_OC_SetCompareCH2(TIM1, pi_u_->data[pi_u_->pnt]>>10);

		LL_TIM_CC_DisableChannel(TIM1, LL_TIM_CHANNEL_CH3);
		LL_TIM_CC_DisableChannel(TIM1, LL_TIM_CHANNEL_CH3N);
	}else if(ctl_ang >= RAD270 && ctl_ang < RAD330){
		LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH1);
		LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH1N);
		LL_TIM_OC_SetCompareCH1(TIM1, 0);

		LL_TIM_CC_DisableChannel(TIM1, LL_TIM_CHANNEL_CH2);
		LL_TIM_CC_DisableChannel(TIM1, LL_TIM_CHANNEL_CH2N);

		LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH3);
		LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH3N);
		LL_TIM_OC_SetCompareCH3(TIM1, pi_u_->data[pi_u_->pnt]>>10);
	}else{
		LL_TIM_CC_DisableChannel(TIM1, LL_TIM_CHANNEL_CH1);
		LL_TIM_CC_DisableChannel(TIM1, LL_TIM_CHANNEL_CH1N);

		LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH2);
		LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH2N);
		LL_TIM_OC_SetCompareCH2(TIM1, 0);

		LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH3);
		LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH3N);
		LL_TIM_OC_SetCompareCH3(TIM1, pi_u_->data[pi_u_->pnt]>>10);
	}

	//update ring buffer pointer
	pi_u_->pnt = (pi_u_->pnt+1) | 0x03;
	pi_e_->pnt = (pi_e_->pnt+1) | 0x03;
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
