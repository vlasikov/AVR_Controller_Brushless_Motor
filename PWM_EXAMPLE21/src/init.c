/*
 * init.c
 *
 * Created: 25.11.2019 17:09:46
 *  Author: yu.vlasikov
 */ 

#include <init.h>
#include <motor.h>
#include <conf_oversampling.h>

extern uint8_t MotorStatus;
extern uint16_t Top_tc_period = 15000;

static uint8_t second = 0;
static uint16_t tc_clksel_div = TC_CLKSEL_DIV8_gc;
struct ac_config aca_config;

void timerInit(){
	tcc1_init();
	tcd1_init();
}

/*
 *
*/
void timerC1_tick(){	
	if (MotorStatus==0)	
		MotorStop();
		
	if (MotorStatus==1)	
		MotorNextPhase();
		
	if (MotorStatus==3)
		MotorNextPhase();
		
}

void timerD1_tick(){
	ioport_toggle_pin_level(IOPORT_CREATE_PIN(PORTD, 3));
	if (second < 100){
		second++;
	}
	
	switch(second){
		case 1:
			MotorStatus = 0;
			break;			
		case 2:
			MotorStatus = 1;
			pwm_overflow_int_callback(&pwm_botA, pwm_callback_2);
			break;
		default:
			break;
	}
	
	tc_write_clock_source(&TCC1, tc_clksel_div);
}

void tcc1_init()
{
	//Initialisation Timer TCC1
	tc_enable(&TCC1);	
	tc_set_overflow_interrupt_callback(&TCC1, timerC1_tick); //Cr?ation d'un callback qui sera execut? quand un overflow du timer sera d?clench?.
	tc_set_wgm(&TCC1, TC_WG_NORMAL);		//Choix du mode du timer0, dans ce cas il comptera jusqu'? sa valeur "TOP" et retombera ? 0
	tc_write_period(&TCC1, Top_tc_period);			//D?finition de la valeur "TOP"	
	tc_set_overflow_interrupt_level(&TCC1, TC_INT_LVL_LO);	//Activation de l'interruption du timer 				
	tc_write_clock_source(&TCC1, tc_clksel_div);		//Activation de l'horloge du timer 0
}

void tcd1_init()
{
	//Initialisation Timer TCD1
	tc_enable(&TCD1);	
	tc_set_overflow_interrupt_callback(&TCD1, timerD1_tick); //Cr?ation d'un callback qui sera execut? quand un overflow du timer sera d?clench?.
	tc_set_wgm(&TCD1, TC_WG_NORMAL);		//Choix du mode du timer0, dans ce cas il comptera jusqu'? sa valeur "TOP" et retombera ? 0
	tc_write_period(&TCD1, 30000);			//D?finition de la valeur "TOP"	
	tc_set_overflow_interrupt_level(&TCD1, TC_INT_LVL_LO);	//Activation de l'interruption du timer 				
	tc_write_clock_source(&TCD1, TC_CLKSEL_DIV1024_gc);		//Activation de l'horloge du timer 0
}

void pwmInit(){
	/* Set PWM to TC E1, channel A (PC0 = PWM_TOPC4), 250 Hz */
	pwm_init(&pwm_botA, PWM_TCC0, PWM_CH_A, 25000);
	//pwm_overflow_int_callback(&pwm_botA, pwm_callback_2);
	pwm_init(&pwm_botB, PWM_TCC0, PWM_CH_B, 25000);
	pwm_init(&pwm_botC, PWM_TCC0, PWM_CH_C, 25000);

	/*
	  Start PWM
	*/
	pwm_start(&pwm_botA, 0);
	pwm_start(&pwm_botB, 0);
	pwm_start(&pwm_botC, 0);	
}

/**
 * \brief PWM channel 1 interrupt callback function
 */
void pwm_callback_1 (){
}

volatile uint16_t rv_reg;
volatile uint16_t counter = 0;
void pwm_callback_2 (){
	if (MotorStatus <= 3){
		MotorPhazeControl2();
	}
}

void acInit(){
//ac_set_interrupt_callback(&ACA, example_aca_interrupt_callback);

	/* Setup the analog comparator B in window mode. */
	ac_set_mode(&aca_config, AC_MODE_SINGLE );
	// ac_set_voltage_scaler(&aca_config, 11);
	ac_set_hysteresis(&aca_config, AC_HYSMODE_LARGE_gc);
	ac_set_positive_reference(&aca_config, AC_MUXPOS_PIN0_gc);
	ac_set_negative_reference(&aca_config, AC_MUXNEG_PIN5_gc);
	ac_set_interrupt_mode(&aca_config, AC_INT_MODE_BOTH_EDGES);	// по обоим фронтам
	ac_set_interrupt_level(&aca_config, AC_INT_LVL_MED);
//	ac_set_high_speed_mode(&aca_config);
	
//	ACA.CTRLA |= AC_AC0OUT_bm;
//	PORTA.DIR |= PIN7_bm;
	
	/*
	 * Write configuration of analog comparator B channel 0, half of window
	 * configuration.
	 */
	ac_write_config(&ACA, 0, &aca_config);
	
//	ac_set_positive_reference(&aca_config, AC_MUXPOS_PIN1_gc);
//	ac_write_config(&ACA, 1, &aca_config);
	
	/* Enable all the analog comparator channels. */
	ac_enable(&ACA, 0);
//	ac_enable(&ACA, 1);
}

/**
 * \brief Analog comparator interrupt callback function
 */
void example_aca_interrupt_callback(AC_t *ac, uint8_t channel, enum ac_status_t status){	
}

void adcInit(){
	/* значение смещения считанное из сигнатуры процессора */
	ADCB.CAL=0xff;
	/* беззнаковый режим, автоматический режим, 12-битный результат с правым выравниванием */
	ADCB.CTRLB = ADC_RESOLUTION_12BIT_gc | 0x08;
	/* разрешение работы бэндгап-элемента, внутреннее опорное напряжение 1В */
	ADCB.REFCTRL = ADC_REFSEL_INT1V_gc | 0x02;
	/* периферийная частота = clk/16 (2MHz/16)*/
	ADCB.PRESCALER = ADC_PRESCALER_DIV16_gc;
	/* канал 0 ADCA настроен на внешний несимметричных вход */
	ADCB.CH0.CTRL = ADC_CH_INPUTMODE_SINGLEENDED_gc;
	/* Ножка 3 порта А настроена как положительный вход */
	ADCB.CH0.MUXCTRL = ADC_CH_MUXPOS_PIN1_gc;  
	
	
	
	
	//ADCB.CALL = ReadCalibrationByte( offsetof(NVM_PROD_SIGNATURES_t, ADCBCAL0) );
	//ADCB.CALH = ReadCalibrationByte( offsetof(NVM_PROD_SIGNATURES_t, ADCBCAL1) );
	ADCB.CTRLB = ADC_RESOLUTION_12BIT_gc;       // 12 bit conversion
	ADCB.PRESCALER = ADC_PRESCALER_DIV256_gc;   // peripheral clk/256 (32MHz/256=125KHz)
	ADCB.REFCTRL = ADC_REFSEL_INT1V_gc;         // internal 1V reference

	ADCB.CH0.CTRL = ADC_CH_INPUTMODE_INTERNAL_gc | ADC_CH_GAIN_1X_gc;
	ADCB.CH0.MUXCTRL = ADC_CH_MUXINT_TEMP_gc;
											// Internal Temp Sensor
	ADCB.CH1.CTRL = ADC_CH_INPUTMODE_INTERNAL_gc | ADC_CH_GAIN_1X_gc;
	ADCB.CH1.MUXCTRL = ADC_CH_MUXINT_SCALEDVCC_gc;  // Internal VCC Sensor
	
	
	
	
	/* Разрешение работы АЦП */
	ADCB.CTRLA|=ADC_ENABLE_bm;  
	
	ADCB.CH0.CTRL |= ADC_CH_START_bm;
}

uint8_t ReadCalibrationByte( uint8_t index )
{
	uint8_t result;

	/* Load the NVM Command register to read the calibration row. */
	NVM_CMD = NVM_CMD_READ_CALIB_ROW_gc;
	result = pgm_read_byte(index);

	/* Clean up NVM Command register. */
	NVM_CMD = NVM_CMD_NO_OPERATION_gc;

	return( result );
}


#define VCC 3.3							// XXX lol
#define VCCmV 3300 						// XXX lol
#define VCCuV 3300000 						// XXX lol

void int_adcb_init(void) {
	
	PORTB.DIR &= 0x0;					// set PB3 as input

	ADCB.CTRLB |= ADC_RESOLUTION_12BIT_gc;			// set for 12 bit right adjusted mode
	ADCB.REFCTRL |= ADC_REFSEL_VCC_gc;			// set aref to VCC / 1.6V
	ADCB.PRESCALER = ADC_PRESCALER_DIV256_gc;			// clk/8

	// set for single ended conv, 1X
	ADCB.CH0.CTRL = ADC_CH_INPUTMODE_SINGLEENDED_gc;

	ADCB.CH0.MUXCTRL = ADC_CH_MUXPOS_PIN1_gc;		// set the adc mux for PB3
	
	ADCB.CTRLA |= 0x01;					// set bit one to enable adcb
}


int start_int_adcb_conv(void){

	volatile int res, res_mv;

	/* start a conv on what we earlier set as ch0 */
	ADCB.CH0.CTRL |= ADC_CH_START_bm;
	
	res = (ADCB.CH0.INTFLAGS);
	/* wait for conv to complete */
	while(!(ADCB.CH0.INTFLAGS));
	
	/* gcc will totally handle this read for us. */
	res = ADCB.CH0RES;
	adc_clear_interrupt_flag(&ADCB, ADC_CH0);
	
	/* the conversion theory: n * (AREF / 2**12) (only in mV to avoid floats) */
	res_mv = (res * (VCCuV / 16) / 4096) / 100;

	return res_mv;
}

/**
 * \brief Static variable to store total ADC Offset value for total number
 *        of sample
 */
static int16_t adc_offset = 0;

/* ! \brief Static variable to accumulate sampled ADC result */
static volatile int64_t adc_result_accumulator = 0;

/* ! \brief Static variable to store offset for single sample */
static int8_t adc_offset_one_sample = 0;

/* ! \brief Static variable to keep number of samples for oversampling */
static volatile uint16_t adc_samplecount = 0;

/**
 * \brief Global variable/flag to indicate that one set of
 *         oversampling is done for start processing
 */
volatile bool adc_oversampled_flag = false;

/* ! \brief Static variable to to store single sampled ADC result */
static volatile int64_t adc_result_one_sample = 0;
static volatile int64_t adc_result_one_sample_old = 0;

/* ! \brief Static variable to keep ADC configuration parameters */
static struct adc_config adc_conf;

/* ! \brief Static variable to keep ADC channel configuration parameters */
static struct adc_channel_config adc_ch_conf;


/**
 * \brief This function initialize the ADCB,gets ADCB-CH0 offset and configure
 *        ADCB-CH0 for oversampling
 *  - ADCB-CH0 is configured in 12bit, signed differential mode without gain
 *  - To read ADC offset, ADCB-Pin3(PB3) used as both +ve and -ve input
 *  - After reading ADC offset,to start oversampling,ADCB +ve and -ve input
 *    are configured
 */
void init_adc(void)
{
	/* Initialize configuration structures */
	adc_read_configuration(&ADCB, &adc_conf);
	adcch_read_configuration(&ADCB, ADC_CH0, &adc_ch_conf);

	/* Configure the ADCB module:
	 * - Signed, 12-bit resolution
	 * - External reference on AREFB pin.
	 * - 250 KSPS ADC clock rate
	 * - Manual conversion triggering
	 * - Callback function
	 */
	//adc_set_conversion_parameters(&adc_conf, ADC_SIGN_ON, ADC_RES_12, ADC_REF_VCCDIV2);
	adc_set_conversion_parameters(&adc_conf, ADC_SIGN_OFF, ADC_RES_12, ADC_REF_VCCDIV2);
	adc_set_clock_rate(&adc_conf, 250000UL);
	adc_set_conversion_trigger(&adc_conf, ADC_TRIG_MANUAL, 1, 0);
	adc_write_configuration(&ADCB, &adc_conf);
	adc_set_callback(&ADCB, &adc_handler);

	/* Configure ADC B channel 0 for offset calculation
	 * - Differential mode without gain
	 * - Selected Pin3 (PB3) as +ve and -ve input for offset calculation
	 */
//	adcch_set_input(&adc_ch_conf, ADCCH_POS_PIN3, ADCCH_NEG_PIN3, 1);
	adcch_set_input(&adc_ch_conf, ADCCH_POS_PIN1, ADCCH_NEG_PAD_GND, 0);	// gain 0.5
	adcch_write_configuration(&ADCB, ADC_CH0, &adc_ch_conf);

	/* Enable ADCB */
	adc_enable(&ADCB);

	/* Get ADC offset in to ADC_Offset variable and disable ADC */
//	adc_offset_one_sample = adc_offset_get_signed();

	/* Find ADC_Offset for for total number of samples */
//	adc_offset = adc_offset_one_sample * ADC_OVER_SAMPLED_NUMBER;

	/* Disable ADC to configure for oversampling */
	adc_disable(&ADCB);

	/* Configure the ADCB module for oversampling:
	 * - Signed, 12-bit resolution
	 * - External reference on AREFB pin.
	 * - 250 KSPS ADC clock rate
	 * - Free running mode on Channel0 ( First Channel)
	 */

	adc_set_conversion_trigger(&adc_conf, ADC_TRIG_FREERUN_SWEEP, 1, 0);
	adc_write_configuration(&ADCB, &adc_conf);

	/* Configure ADC B channel 0 for oversampling input
	 * - Differential mode without gain
	 * - Selected Pin1 (PB1) as +ve and Pin2 (PB2) as-ve input
	 * - Conversion complete interrupt
	 */
	adcch_set_input(&adc_ch_conf, ADC_OVER_SAMP_POSTIVE_PIN, ADC_OVER_SAMP_NEGATIVE_PIN, 1);
	adcch_set_interrupt_mode(&adc_ch_conf, ADCCH_MODE_COMPLETE);
	adcch_enable_interrupt(&adc_ch_conf);
	adcch_write_configuration(&ADCB, ADC_CH0, &adc_ch_conf);

	/* Enable ADCB */
	adc_enable(&ADCB);
}

/**
 * \brief Callback function for ADCB-CH0 interrupts
 *  - Interrupt is configured for Conversion Complete Interrupt
 *  - ADCA CH0 result is accumulated
 *  - ADC sample count is incremented
 *  - Check If ADC sample count reached up to number of oversampling required
 *  - If so, disable ADC interrupt and set flag to start oversampling process
 *
 * \param adc Pointer to ADC module.
 * \param ch_mask ADC channel mask.
 * \param result Conversion result from ADC channel.
 */

uint16_t ADC = 0;
uint16_t unsignedADC = 0;
static void adc_handler(ADC_t *adc, uint8_t ch_mask, adc_result_t result)
{
	/*  Get Result from ADCB-CH0 Register and Accumulate */
	adc_result_accumulator +=  result;

	/* Increment sample count */
	adc_samplecount++;

	/* Check if sample count has reached oversample count */
	if (adc_samplecount >= ADC_OVER_SAMPLED_NUMBER) {
		/* Disable ADCB-CHO conversion complete interrupt until stored
		 * samples are processed
		 */
//		adcch_disable_interrupt(&adc_ch_conf);
		adcch_write_configuration(&ADCB, ADC_CH0, &adc_ch_conf);

		/* Clear any pending interrupt request by clearing interrupt
		 * flag
		 */
		adc_clear_interrupt_flag(&ADCB, ADC_CH0);

		/*Set adc_oversampled_flag to start oversampling process from
		 * main function
		 */
		adc_oversampled_flag = true;

		/* Store single sample ADC result to find analog input without
		 * oversampling
		 */
		adc_result_one_sample = result;
		//adc_result_accumulator = 0;
		//ADC = result;//adc_result_accumulator >> 8;  // acc/256/1024
		ADC = 3000;
//		ADC = result;
// 		if (result >= 0){
// 			unsignedADC = result + 0x;
// 		}else{
// 			unsignedADC = 
// 		}
		
		adc_samplecount = 0;
		adc_result_accumulator = 0;
	}
}

/**
 * \brief This function get the offset of the ADCB when it is configured
 *	      in signed mode
 *  \note The ADC must be configured and enabled before this function is run.
 * \return Offset on the ADCB
 */
static int8_t adc_offset_get_signed(void)
{
	int16_t offset = 0;
	uint8_t i;

	/* Sum four samples */
	for (i = 0; i < 4; i++) {
		/* Do one conversion to find offset */
		adc_start_conversion(&ADCB, ADC_CH0);
		adc_wait_for_interrupt_flag(&ADCB, ADC_CH0);

		/* Accumulate conversion results */
		offset += adc_get_result(&ADCB, ADC_CH0);
	}

	/* Return mean value */
	return ((int8_t)(offset / 4));
}