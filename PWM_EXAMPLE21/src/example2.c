/**
 * \file
 *
 * \brief PWM example for XMEGA
 *
 * Copyright (c) 2014-2018 Microchip Technology Inc. and its subsidiaries.
 *
 * \asf_license_start
 *
 * \page License
 *
 * Subject to your compliance with these terms, you may use Microchip
 * software and any derivatives exclusively with Microchip products.
 * It is your responsibility to comply with third party license terms applicable
 * to your use of third party software (including open source software) that
 * may accompany Microchip software.
 *
 * THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS". NO WARRANTIES,
 * WHETHER EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE,
 * INCLUDING ANY IMPLIED WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY,
 * AND FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT WILL MICROCHIP BE
 * LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE, INCIDENTAL OR CONSEQUENTIAL
 * LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND WHATSOEVER RELATED TO THE
 * SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP HAS BEEN ADVISED OF THE
 * POSSIBILITY OR THE DAMAGES ARE FORESEEABLE.  TO THE FULLEST EXTENT
 * ALLOWED BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL CLAIMS IN ANY WAY
 * RELATED TO THIS SOFTWARE WILL NOT EXCEED THE AMOUNT OF FEES, IF ANY,
 * THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR THIS SOFTWARE.
 *
 * \asf_license_stop
 *
 */

 /**
 * \mainpage
 *
 * \section intro Introduction
 * This example shows how to use the \ref pwm_group with interrupts.
 *
 * \section files Main Files
 * - pwm.c PWM service implementation
 * - pwm.h PWM service definitions
 * - example2.c example application
 *
 * \section device_info Device Info
 * This example is made specifically for XMEGA-A1 Xplained. However, any XMEGA
 * board with PWM_TOPCs connected to PE0 and PE4 may be used.
 *
 * \section exampPWM_TOPCescription Description of the example
 * This example uses two PWM channels on two different PWM_TOPCs. An interrupt is
 * triggered on each PWM TC overrun, and the interrupt handler will increase
 * the duty cycle of the PWM channels, thus fading the PWM_TOPCs out (since the PWM_TOPCs
 * are active low, increased duty cycle means more dimmed light).
 * The two PWM channels are running two different frequencies to make the
 * fading speed different.
 *
 * \section dependencies Dependencies
 * This example depends on the following module:
 * - \ref pwm_group
 *
 * \section apiinfo PWM Service API
 * The PWM service API can be found \ref pwm_group "here"
 *
 * \section compinfo Compilation info
 * This software was written for the GNU GCC. Other compilers
 * may or may not work.
 *
 * \section contactinfo Contact Information
 * For further information, please visit
 * <a href="http://www.microchip.com/">atmel.com</a>.\n
 */
/*
 * Support and FAQ: visit <a href="https://www.microchip.com/support/">Microchip Support</a>
 */

#include <asf.h>

struct pwm_config pwm_botA;
struct pwm_config pwm_botB;
struct pwm_config pwm_botC;

#define PWM_TOPA IOPORT_CREATE_PIN(PORTD, 0)
#define PWM_TOPB IOPORT_CREATE_PIN(PORTD, 1)
#define PWM_TOPC IOPORT_CREATE_PIN(PORTD, 2)

volatile uint8_t duty_cycle_percent_topA = 0;
volatile uint8_t duty_cycle_percent_topB = 0;
volatile uint8_t duty_cycle_percent_botA = 0;

volatile uint8_t step = 0;
volatile uint8_t step_old = 0;

/**
 * \brief PWM channel 1 interrupt callback function
 */
static void pwm_callback_1 (void)
{
	/* Increase (and wrap at 100) the duty cycle */
	//if (duty_cycle_percent_topA++ >= 100) {
	//	duty_cycle_percent_topA = 50;
	//}
	/* Set new duty cycle value */
	//pwm_set_duty_cycle_percent(&pwm_topA, duty_cycle_percent_topA);
}

#define PD3 IOPORT_CREATE_PIN(PORTD, 3)
static struct ac_config aca_config;
static uint8_t second = 0;
static uint16_t tc_clksel_div = TC_CLKSEL_DIV8_gc;
static uint16_t Top_tc_period = 15000;
static const uint16_t Top_tc_period_min = 2000;
static uint8_t MotorPower = 10;
static uint8_t MotorPowerMax = 25;
static uint8_t MotorStatus = 0;

static uint16_t DelayC = 0;
static uint16_t DelayCMax = 8;

static uint16_t StepTime = 0;
/**
 * \brief PWM channel 2 interrupt callback function
 */

void MotorPhazeControl();


void MotorPhazeControl2() {
	uint8_t ac0Out;
	if (ac_get_status(&ACA, 0)){
		ac0Out = 0;
	}
	else{
		ac0Out = 1;
	}
	switch (step){
		case 0:
			if (step_old != step) {
				ac_disable(&ACA, 0);
				ac_set_positive_reference(&aca_config, AC_MUXPOS_PIN0_gc);		// phase C
				ac_write_config(&ACA, 0, &aca_config);
				ac_enable(&ACA, 0);
				
				DelayC = 0;
			}
		
			if (ac0Out)															// phase C
				ioport_set_pin_level(IOPORT_CREATE_PIN(PORTC, 5), 1);
			else
				ioport_set_pin_level(IOPORT_CREATE_PIN(PORTC, 5), 0);
		
			if (!ac0Out){
				if (DelayC > DelayCMax){
					if (MotorStatus == 2){
						MotorPhazeControl();
						return;
					}
				}
				DelayC++;			
			}
			else{
				DelayC = 0;
			}
			break;
		case 1:
			if (step_old != step) {
				ac_disable(&ACA, 0);
				ac_set_positive_reference(&aca_config, AC_MUXPOS_PIN1_gc);		// phase B
				ac_write_config(&ACA, 0, &aca_config);
				ac_enable(&ACA, 0);
				
				DelayC = 0;
			}
			
			if (ac0Out)															// phase B
				ioport_set_pin_level(IOPORT_CREATE_PIN(PORTC, 4), 1);
			else
				ioport_set_pin_level(IOPORT_CREATE_PIN(PORTC, 4), 0);
				
			if (ac0Out){
				if (DelayC > DelayCMax){
					if (MotorStatus == 2){
						MotorPhazeControl();
						return;
					}
				}
				DelayC++;
			}
			else{
				DelayC = 0;
			}	
			break;
		case 2:
			if (step_old != step) {
				ac_disable(&ACA, 0);
				ac_set_positive_reference(&aca_config, AC_MUXPOS_PIN2_gc);		// phase A
				ac_write_config(&ACA, 0, &aca_config);
				ac_enable(&ACA, 0);
			
				DelayC = 0;
			}
		
			if (ac0Out)															// phase A
				ioport_set_pin_level(IOPORT_CREATE_PIN(PORTC, 3), 1);
			else
				ioport_set_pin_level(IOPORT_CREATE_PIN(PORTC, 3), 0);
		
			if (!ac0Out){
				if (DelayC > DelayCMax){
					if (MotorStatus == 2){
						MotorPhazeControl();
						return;
					}
				}
				DelayC++;
			}
			else{
				DelayC = 0;
			}
			break;
		case 3:
			if (step_old != step) {
				ac_disable(&ACA, 0);
				ac_set_positive_reference(&aca_config, AC_MUXPOS_PIN0_gc);		// phase C
				ac_write_config(&ACA, 0, &aca_config);
				ac_enable(&ACA, 0);
			}
		
			if (ac0Out)
				ioport_set_pin_level(IOPORT_CREATE_PIN(PORTC, 5), 1);			// phase C
			else
				ioport_set_pin_level(IOPORT_CREATE_PIN(PORTC, 5), 0);
			
			if (ac0Out){
				if (DelayC > DelayCMax){
					if (MotorStatus == 2){
						MotorPhazeControl();
						return;
					}
				}
				DelayC++;
			}
			else{
				DelayC = 0;
			}
			break;
		case 4:
			if (step_old != step) {
				ac_disable(&ACA, 0);
				ac_set_positive_reference(&aca_config, AC_MUXPOS_PIN1_gc);		// phase B
				ac_write_config(&ACA, 0, &aca_config);
				ac_enable(&ACA, 0);
				
				DelayC = 0;
			}
		
			if (ac0Out)															// phase B
				ioport_set_pin_level(IOPORT_CREATE_PIN(PORTC, 4), 1);
			else
				ioport_set_pin_level(IOPORT_CREATE_PIN(PORTC, 4), 0);
		
			if (!ac0Out){
				if (DelayC > DelayCMax){
					if (MotorStatus == 2){
						MotorPhazeControl();
						return;
					}
				}
				DelayC++;
			}
			else{
				DelayC = 0;
			}
			break;
		case 5:
			if (step_old != step) {
				ac_disable(&ACA, 0);
				ac_set_positive_reference(&aca_config, AC_MUXPOS_PIN2_gc);		// phase A
				ac_write_config(&ACA, 0, &aca_config);
				ac_enable(&ACA, 0);
				
				DelayC = 0;
			}
		
			if (ac0Out)															// phase A
				ioport_set_pin_level(IOPORT_CREATE_PIN(PORTC, 3), 1);
			else
				ioport_set_pin_level(IOPORT_CREATE_PIN(PORTC, 3), 0);
		
			if (ac0Out){
				if (DelayC > DelayCMax){
					if (MotorStatus == 2){
						MotorPhazeControl();
						return;
					}
				}
				DelayC++;
			}
			else{
				DelayC = 0;
			}
			break;
		default:
		break;
	}
	step_old = step;
}

void MotorPhazeControl(){
	
	
	if (++step >= 6) {
		step = 0;
	}
	
	switch (step){
		case 0:
			if (Top_tc_period > Top_tc_period_min) {
				if (MotorStatus == 1){
					Top_tc_period *= 0.99;
				}
			
				if (Top_tc_period < 30000){
					MotorPower = 10;
				}
				if (Top_tc_period < 15000){
					MotorPower = 15;
				}
				if (Top_tc_period < 10000){
					MotorPower = 15;
				}
				if (Top_tc_period < 5000){
					MotorPower = 22;
					MotorStatus = 2;
				}
				if (Top_tc_period < 2500){
					MotorPower = 30;
					MotorStatus = 2;
				}
			
				tc_write_period(&TCC1, Top_tc_period);
			}
		
			pwm_set_duty_cycle_percent(&pwm_botA, 0);
		
			ioport_set_pin_level(PWM_TOPB, 0);
		
			ioport_set_pin_level(PWM_TOPC, 0);
			pwm_set_duty_cycle_percent(&pwm_botC, 0);
		
			ioport_set_pin_level(PWM_TOPA, 1);
			//pwm_set_duty_cycle_percent(&pwm_botB, MotorPower);
			break;
		case 1:
			pwm_set_duty_cycle_percent(&pwm_botA, 0);
		
			ioport_set_pin_level(PWM_TOPB, 0);
			pwm_set_duty_cycle_percent(&pwm_botB, 0);
		
			ioport_set_pin_level(PWM_TOPC, 0);
		
			ioport_set_pin_level(PWM_TOPA, 1);
			pwm_set_duty_cycle_percent(&pwm_botC, MotorPower);
			break;
		case 2:
			ioport_set_pin_level(PWM_TOPA, 0);
			pwm_set_duty_cycle_percent(&pwm_botA, 0);
		
			pwm_set_duty_cycle_percent(&pwm_botB, 0);
		
			ioport_set_pin_level(PWM_TOPC, 0);
		
			ioport_set_pin_level(PWM_TOPB, 1);
			//pwm_set_duty_cycle_percent(&pwm_botC, MotorPower);
		break;
		case 3:
			ioport_set_pin_level(PWM_TOPA, 0);
		
			pwm_set_duty_cycle_percent(&pwm_botB, 0);
		
			ioport_set_pin_level(PWM_TOPC, 0);
		
			pwm_set_duty_cycle_percent(&pwm_botC, 0);
		
			ioport_set_pin_level(PWM_TOPB, 1);
			pwm_set_duty_cycle_percent(&pwm_botA, MotorPower);
			break;
		case 4:
			ioport_set_pin_level(PWM_TOPA, 0);
		
			ioport_set_pin_level(PWM_TOPB, 0);
			pwm_set_duty_cycle_percent(&pwm_botB, 0);
		
			pwm_set_duty_cycle_percent(&pwm_botC, 0);
		
			ioport_set_pin_level(PWM_TOPC, 1);
			// pwm_set_duty_cycle_percent(&pwm_botA, MotorPower);
			break;
		case 5:
			ioport_set_pin_level(PWM_TOPA, 0);
			pwm_set_duty_cycle_percent(&pwm_botA, 0);
			
			ioport_set_pin_level(PWM_TOPB, 0);
			
			pwm_set_duty_cycle_percent(&pwm_botC, 0);
			
			ioport_set_pin_level(PWM_TOPC, 1);
			pwm_set_duty_cycle_percent(&pwm_botB, MotorPower);
			break;
		default:
			break;
	}
}

void MotorStop(){
	ioport_set_pin_level(PWM_TOPA, 0);
	pwm_set_duty_cycle_percent(&pwm_botA, 0);
	
	ioport_set_pin_level(PWM_TOPB, 0);
	
	pwm_set_duty_cycle_percent(&pwm_botC, 0);
	
	ioport_set_pin_level(PWM_TOPC, 1);
	pwm_set_duty_cycle_percent(&pwm_botB, 0);
}

static void pwm_callback_2 (void)
{
	/* Increase (and wrap at 100) the duty cycle */
	//if (duty_cycle_percent_botA++ >= 100) {
	//	duty_cycle_percent_botA = 0;
	//}
	//duty_cycle_percent_botA = 75;
	/* Set new duty cycle value */
	//pwm_set_duty_cycle_percent(&pwm_botA, duty_cycle_percent_botA);

	if (MotorStatus <= 2){
		MotorPhazeControl2();
	}
}

/*
 *
*/
static void timerC1_tick(void){	
	if (MotorStatus==0)	
		MotorStop();
		
	if (MotorStatus==1)	
		MotorPhazeControl();
//	else
//		MotorStop();
}


void timerD1_tick(void){
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

void tcc1_init(void)
{
	//Initialisation Timer TCC1
	tc_enable(&TCC1);	
	tc_set_overflow_interrupt_callback(&TCC1, timerC1_tick); //Cr?ation d'un callback qui sera execut? quand un overflow du timer sera d?clench?.
	tc_set_wgm(&TCC1, TC_WG_NORMAL);		//Choix du mode du timer0, dans ce cas il comptera jusqu'? sa valeur "TOP" et retombera ? 0
	tc_write_period(&TCC1, Top_tc_period);			//D?finition de la valeur "TOP"	
	tc_set_overflow_interrupt_level(&TCC1, TC_INT_LVL_LO);	//Activation de l'interruption du timer 				
	tc_write_clock_source(&TCC1, tc_clksel_div);		//Activation de l'horloge du timer 0
}

void tcd1_init(void)
{
	//Initialisation Timer TCD1
	tc_enable(&TCD1);	
	tc_set_overflow_interrupt_callback(&TCD1, timerD1_tick); //Cr?ation d'un callback qui sera execut? quand un overflow du timer sera d?clench?.
	tc_set_wgm(&TCD1, TC_WG_NORMAL);		//Choix du mode du timer0, dans ce cas il comptera jusqu'? sa valeur "TOP" et retombera ? 0
	tc_write_period(&TCD1, 30000);			//D?finition de la valeur "TOP"	
	tc_set_overflow_interrupt_level(&TCD1, TC_INT_LVL_LO);	//Activation de l'interruption du timer 				
	tc_write_clock_source(&TCD1, TC_CLKSEL_DIV1024_gc);		//Activation de l'horloge du timer 0
}

/**
 * \brief Analog comparator interrupt callback function
 *
 * This function is called when an interrupt has occurred on a channel in analog
 * comparator.
 *
 * \param ac Pointer to the analog comparator (AC) base address which caused
 *           the interrupt
 * \param channel The analog comparator channel that caused the interrupt
 * \param status Analog comparator window status given by a \ref ac_status_t
 *               value
 */
static void example_aca_interrupt_callback(AC_t *ac, uint8_t channel,
		enum ac_status_t status)
{	
/*			
	if (step == 1){
		if (ac_get_status(&ACA, 0))
			ioport_set_pin_level(IOPORT_CREATE_PIN(PORTC, 3), 1);
		else
			ioport_set_pin_level(IOPORT_CREATE_PIN(PORTC, 3), 0);
	}
	
	if (step == 4){
		if (ac_get_status(&ACA, 0))
			ioport_set_pin_level(IOPORT_CREATE_PIN(PORTC, 4), 1);
		else
			ioport_set_pin_level(IOPORT_CREATE_PIN(PORTC, 4), 0);
	}
	
	/*
	 * If trigger was caused by moving into above or below, switch to
	 * trigger by being inside. If trigger was caused by moving inside the
	 * window, switch to trigger on outside.
	 */
/*
	if (status != AC_STATUS_INSIDE) {
		ac_set_interrupt_mode(&aca_config, AC_INT_MODE_INSIDE_WINDOW);
	} else {
		ac_set_interrupt_mode(&aca_config, AC_INT_MODE_OUTSIDE_WINDOW);
	}
*/
//	ac_disable(&ACA, 0);
//	ac_disable(&ACA, 2);
//	ac_disable(&ACA, 0);
//	ac_write_config(&ACA, 0, &aca_config);
//	ac_enable(&ACA, 0);
//	ac_enable(&ACA, 1);

//	example_ac_update_window_leds(status);
}

/**
 * \brief Example 2 main application routine
 */
int main( void )
{
	/* Initialize interrupt controller, board and sysclock */
	pmic_init();
	sysclk_init();

	/* Enable global interrupts */
	cpu_irq_enable();
	
	ioport_set_pin_dir(PWM_TOPA, IOPORT_DIR_OUTPUT);
	ioport_set_pin_dir(PWM_TOPB, IOPORT_DIR_OUTPUT);
	ioport_set_pin_dir(PWM_TOPC, IOPORT_DIR_OUTPUT);
	tcc1_init();
	ioport_set_pin_dir(IOPORT_CREATE_PIN(PORTD, 3), IOPORT_DIR_OUTPUT);
	tcd1_init();
	
	ioport_set_pin_dir(IOPORT_CREATE_PIN(PORTC, 3), IOPORT_DIR_OUTPUT);
	ioport_set_pin_dir(IOPORT_CREATE_PIN(PORTC, 4), IOPORT_DIR_OUTPUT);
	ioport_set_pin_dir(IOPORT_CREATE_PIN(PORTC, 5), IOPORT_DIR_OUTPUT);

	/*
	  Set up PWM channel
	*/

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
	
	ac_set_interrupt_callback(&ACA, example_aca_interrupt_callback);

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
	
	while(1) {
		/* Do nothing. Everything is handPWM_TOPC by interrupts. */
	}
}
