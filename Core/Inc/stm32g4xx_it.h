/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32g4xx_it.h
  * @brief   This file contains the headers of the interrupt handlers.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __STM32G4xx_IT_H
#define __STM32G4xx_IT_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "math.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */
#define KP (int32_t)(400 * 1024)		//0.5
#define KI (int32_t)(0.01* 1024)		//0.002

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
/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void NMI_Handler(void);
void HardFault_Handler(void);
void MemManage_Handler(void);
void BusFault_Handler(void);
void UsageFault_Handler(void);
void SVC_Handler(void);
void DebugMon_Handler(void);
void PendSV_Handler(void);
void SysTick_Handler(void);
void TIM1_UP_TIM16_IRQHandler(void);
void TIM6_DAC_IRQHandler(void);
void COMP1_2_3_IRQHandler(void);
/* USER CODE BEGIN EFP */
void rotateSin(float theta_, uint16_t power_);

static inline void rotate120Deg(int32_t *rot_est_, int32_t *pre_ctl_, int32_t omg_tgt_, int32_t omg_est_, int32_t adv_ang_, RingBuf *pi_e_, RingBuf *pi_u_, uint8_t *wake_up_){
	(*rot_est_) += (omg_est_*FREQ_T) >> 10;
	int32_t ctl_ang = (*rot_est_) + adv_ang_;

	*rot_est_ += *rot_est_ < RAD0 ? RAD360 : RAD0;
	*rot_est_ -= *rot_est_ > RAD360 ? RAD360 : RAD0;

	ctl_ang += ctl_ang < RAD0 ? RAD360 : RAD0;
	ctl_ang -= ctl_ang > RAD360 ? RAD360 : RAD0;

	//PI control section
	pi_e_->data[pi_e_->pnt] = omg_tgt_ - omg_est_;
	pi_u_->data[pi_u_->pnt] = (KP*pi_e_->data[(pi_e_->pnt+0x03)&0x03]) + ((KP+KI)*pi_e_->data[pi_e_->pnt]);
	pi_u_->data[pi_u_->pnt] >>= 10;
	pi_u_->data[pi_u_->pnt] += (2*pi_u_->data[(pi_u_->pnt+0x03)&0x03]) - (pi_u_->data[(pi_u_->pnt+0x02)&0x03]);
	//pi_u_->data[pi_u_->pnt] = KP*pi_e_->data[pi_e_->pnt];
	//pi_u_->data[pi_u_->pnt] >>= 10;

	pi_u_->data[pi_u_->pnt] = pi_u_->data[pi_u_->pnt] > 200<<10 ? 200<<10 : pi_u_->data[pi_u_->pnt];
	pi_u_->data[pi_u_->pnt] = pi_u_->data[pi_u_->pnt] < 0 ? 0 : pi_u_->data[pi_u_->pnt];

	if(ctl_ang >= RAD30 && ctl_ang < RAD90){
		LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH1);
		LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH1N);
		LL_TIM_OC_SetCompareCH1(TIM1, pi_u_->data[pi_u_->pnt]>>10);

		LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH2);
		LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH2N);
		LL_TIM_OC_SetCompareCH2(TIM1, 0);

		LL_TIM_CC_DisableChannel(TIM1, LL_TIM_CHANNEL_CH3);
		LL_TIM_CC_DisableChannel(TIM1, LL_TIM_CHANNEL_CH3N);

		if(*wake_up_){
			LL_EXTI_EnableRisingTrig_0_31(LL_EXTI_LINE_29);
			*wake_up_ = 0;
		}else if(*pre_ctl_ < RAD30 || *pre_ctl_ >= RAD90){
			*wake_up_ = 1;
		}
	}else if(ctl_ang >= RAD90 && ctl_ang < RAD150){
		LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH1);
		LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH1N);
		LL_TIM_OC_SetCompareCH1(TIM1, pi_u_->data[pi_u_->pnt]>>10);

		LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH2);
		LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH2N);

		LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH3);
		LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH3N);
		LL_TIM_OC_SetCompareCH3(TIM1, 0);

		if(*wake_up_){
			LL_EXTI_EnableRisingTrig_0_31(LL_EXTI_LINE_22);
			*wake_up_ = 0;
		}else if(*pre_ctl_ < RAD90 || *pre_ctl_ >= RAD150){
			*wake_up_ = 1;
		}
	}else if(ctl_ang >= RAD150 && ctl_ang < RAD210){
		LL_TIM_CC_DisableChannel(TIM1, LL_TIM_CHANNEL_CH1);
		LL_TIM_CC_DisableChannel(TIM1, LL_TIM_CHANNEL_CH1N);

		LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH2);
		LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH2N);
		LL_TIM_OC_SetCompareCH2(TIM1, pi_u_->data[pi_u_->pnt]>>10);

		LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH3);
		LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH3N);
		LL_TIM_OC_SetCompareCH3(TIM1, 0);

		if(*wake_up_){
			LL_EXTI_EnableRisingTrig_0_31(LL_EXTI_LINE_21);
			*wake_up_ = 0;
		}else if(*pre_ctl_ < RAD150 || *pre_ctl_ >= RAD210){
			*wake_up_ = 1;
		}
	}else if(ctl_ang >= RAD210 && ctl_ang < RAD270){
		LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH1);
		LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH1N);
		LL_TIM_OC_SetCompareCH1(TIM1, 0);

		LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH2);
		LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH2N);
		LL_TIM_OC_SetCompareCH2(TIM1, pi_u_->data[pi_u_->pnt]>>10);

		LL_TIM_CC_DisableChannel(TIM1, LL_TIM_CHANNEL_CH3);
		LL_TIM_CC_DisableChannel(TIM1, LL_TIM_CHANNEL_CH3N);

		if(*wake_up_){
			LL_EXTI_EnableRisingTrig_0_31(LL_EXTI_LINE_29);
			*wake_up_ = 0;
		}else if(*pre_ctl_ < RAD210 || *pre_ctl_ >= RAD270){
			*wake_up_ = 1;
		}
	}else if(ctl_ang >= RAD270 && ctl_ang < RAD330){
		LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH1);
		LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH1N);
		LL_TIM_OC_SetCompareCH1(TIM1, 0);

		LL_TIM_CC_DisableChannel(TIM1, LL_TIM_CHANNEL_CH2);
		LL_TIM_CC_DisableChannel(TIM1, LL_TIM_CHANNEL_CH2N);

		LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH3);
		LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH3N);
		LL_TIM_OC_SetCompareCH3(TIM1, pi_u_->data[pi_u_->pnt]>>10);

		if(*wake_up_){
			LL_EXTI_EnableRisingTrig_0_31(LL_EXTI_LINE_22);
			*wake_up_ = 0;
		}else if(*pre_ctl_ < 270 || *pre_ctl_ >= RAD330){
			*wake_up_ = 1;
		}
	}else{
		LL_TIM_CC_DisableChannel(TIM1, LL_TIM_CHANNEL_CH1);
		LL_TIM_CC_DisableChannel(TIM1, LL_TIM_CHANNEL_CH1N);

		LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH2);
		LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH2N);
		LL_TIM_OC_SetCompareCH2(TIM1, 0);

		LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH3);
		LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH3N);
		LL_TIM_OC_SetCompareCH3(TIM1, pi_u_->data[pi_u_->pnt]>>10);

		if(*wake_up_){
			LL_EXTI_EnableRisingTrig_0_31(LL_EXTI_LINE_21);
			*wake_up_ = 0;
		}else if(*pre_ctl_ <= RAD330 && *pre_ctl_ > RAD30){
			*wake_up_ = 1;
		}
	}

	//update ring buffer pointer
	pi_u_->pnt = (pi_u_->pnt+1) & 0x03;
	pi_e_->pnt = (pi_e_->pnt+1) & 0x03;

	*pre_ctl_ = ctl_ang;
}
/* USER CODE END EFP */

#ifdef __cplusplus
}
#endif

#endif /* __STM32G4xx_IT_H */
