/*
 * init.h
 *
 * Created: 25.11.2019 17:08:54
 *  Author: yu.vlasikov
 */ 


#ifndef INIT_H_
#define INIT_H_

#include <asf.h>

#define 	ADC_OVER_SAMP_POSTIVE_PIN   ADCCH_POS_PIN1
#define 	ADC_OVER_SAMP_NEGATIVE_PIN   ADCCH_NEG_PIN2

void timerInit();
/*
 *
*/
static void timerC1_tick();
void timerD1_tick();
void tcc1_init();
void tcd1_init();

void pwmInit();
static void pwm_callback_1 ();
static void pwm_callback_2 ();

void acInit();
static void example_aca_interrupt_callback(AC_t *ac, uint8_t channel, enum ac_status_t status);

void adcInit();
uint8_t ReadCalibrationByte( uint8_t index );

void int_adcb_init(void);
void init_adc(void);
static void adc_handler(ADC_t *adc, uint8_t ch_mask, adc_result_t result);
int start_int_adcb_conv(void);


#endif /* INIT_H_ */