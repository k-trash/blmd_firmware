#ifndef __FOC_LIB_H__
#define __FOC_LIB_H__

#define ADC_BUF 34
#define Y_SIZE 10
#define X1_PAD 2
#define Y_PAD 1
#define R_VAL 0

typedef int16_t q15_t;
typedef int32_t q31_t;

//blackman fir 17 tap sampling 400kHz cutoff 20kHz
const int16_t x2_buffer[ADC_BUF] = {0, 0, 18, 0, 110, 0, 359, 0, 843, 0, 1560, 0, 2371, 0, 3025, 0, 3277, 0, 3025, 0, 2371, 0, 1560, 0, 843, 0, 359, 0, 110, 0, 18, 0, 0, 0};

void adc_setup(void);
void cordic_setup(void);

#endif