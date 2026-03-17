#include "foc.h"

#include "main.h"

void adc_setup(void){
	// activate ADC2
	LL_DMA_EnableIT_TC(DMA1, LL_DMA_CHANNEL_1);
	LL_ADC_Enable(ADC2);

	// setup DMA1 channel 1 for ADC2
	LL_DMA_DisableChannel(DMA1, LL_DMA_CHANNEL_1);
	LL_DMA_ConfigAddresses(DMA1, LL_DMA_CHANNEL_1, LL_ADC_DMA_GetRegAddr(ADC2, LL_ADC_DMA_REG_REGULAR_DATA), (uint32_t)&adc_datas, LL_DMA_DIRECTION_PERIPH_TO_MEMORY);
	LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_1, ADC_BUF);
	LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_1);

	// setup FMAC
	LL_FMAC_DisableStart(FMAC);
	LL_FMAC_ConfigX1(FMAC, LL_FMAC_WM_0_THRESHOLD_1, ADC_BUF, ADC_BUF+X1_PAD);
	LL_FMAC_ConfigY(FMAC, LL_FMAC_WM_0_THRESHOLD_1, 2*ADC_BUF+X1_PAD, Y_PAD);

	// setup DMA1 channel 3 for FMAC read
	LL_DMA_DisableChannel(DMA1, LL_DMA_CHANNEL_3);
	LL_DMA_ConfigAddresses(DMA1, LL_DMA_CHANNEL_3, (uint32_t)&FMAC->RDATA, (uint32_t)&y_buffer , LL_DMA_DIRECTION_PERIPH_TO_MEMORY);
	LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_3, 2);
	LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_3);

	// config FMAC X2 buffer
	LL_FMAC_ConfigX2(FMAC, 0x00, ADC_BUF);
	LL_FMAC_ConfigFunc(FMAC, LL_FMAC_PROCESSING_START, LL_FMAC_FUNC_LOAD_X2, ADC_BUF, 0, 0);
	for(uint8_t i=0;i<ADC_BUF;i++){
		LL_FMAC_WriteData(FMAC, x2_buffer[i]);
	}

	// init Y buffer
	LL_FMAC_ConfigFunc(FMAC, LL_FMAC_PROCESSING_START, LL_FMAC_FUNC_LOAD_Y, Y_SIZE, 0, 0);
	for(uint8_t i=0;i<Y_SIZE;i++){
		LL_FMAC_WriteData(FMAC, 0);
	}

	// enable FMAC DMA
	LL_FMAC_EnableDMAReq_READ(FMAC);
	LL_FMAC_EnableDMAReq_WRITE(FMAC);

	// config FMAC for convolution FIR
	LL_FMAC_ConfigFunc(FMAC, LL_FMAC_PROCESSING_START, LL_FMAC_FUNC_CONVO_FIR, ADC_BUF, 0, R_VAL);

	// setup DMA1 channel 2 for FMAC write
	LL_DMA_DisableChannel(DMA1, LL_DMA_CHANNEL_2);
	LL_DMA_ConfigAddresses(DMA1, LL_DMA_CHANNEL_2, (uint32_t)&adc_datas, (uint32_t)&FMAC->WDATA, LL_DMA_DIRECTION_MEMORY_TO_PERIPH);
	LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_2, ADC_BUF);
	LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_2);

	LL_ADC_REG_StartConversion(ADC2);
	LL_FMAC_EnableStart(FMAC);
}