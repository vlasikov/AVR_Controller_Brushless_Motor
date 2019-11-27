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