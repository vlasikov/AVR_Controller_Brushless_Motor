/*
 * init.c
 *
 * Created: 25.11.2019 17:09:46
 *  Author: yu.vlasikov
 */ 

#include <init.h>
#include <motor.h>

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
	
	
	
	
	ADCB.CALL = ReadCalibrationByte( offsetof(NVM_PROD_SIGNATURES_t, ADCBCAL0) );
	ADCB.CALH = ReadCalibrationByte( offsetof(NVM_PROD_SIGNATURES_t, ADCBCAL1) );
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

	ADCB.CTRLA |= 0x01;					// set bit one to enable adcb
	ADCB.CTRLB |= ADC_RESOLUTION_12BIT_gc;			// set for 12 bit right adjusted mode
	ADCB.REFCTRL |= ADC_REFSEL_VCC_gc;			// set aref to VCC / 1.6V
	ADCB.PRESCALER = ADC_PRESCALER_DIV256_gc;			// clk/8

	// set for single ended conv, 1X
	ADCB.CH0.CTRL = ADC_CH_INPUTMODE_SINGLEENDED_gc;

	ADCB.CH0.MUXCTRL = ADC_CH_MUXPOS_PIN1_gc;		// set the adc mux for PB3
}


int start_int_adcb_conv(void){

	volatile int res, res_mv;

	/* start a conv on what we earlier set as ch0 */
	ADCB.CH0.CTRL |= ADC_CH_START_bm;

	res = (ADCB.INTFLAGS & ADC_CH1IF_bm);
	/* wait for conv to complete */
	while(!(ADCB.INTFLAGS & ADC_CH1IF_bm));
	
	/* gcc will totally handle this read for us. */
	res = ADCB.CH0RES;

	/* the conversion theory: n * (AREF / 2**12) (only in mV to avoid floats) */
	res_mv = (res * (VCCuV / 16) / 4096) / 100;

	return res_mv;
}
