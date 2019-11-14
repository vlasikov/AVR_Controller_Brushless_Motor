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

/**
 * \brief PWM channel 2 interrupt callback function
 */
static void pwm_callback_2 (void)
{
	/* Increase (and wrap at 100) the duty cycle */
	//if (duty_cycle_percent_botA++ >= 100) {
	//	duty_cycle_percent_botA = 0;
	//}
	//duty_cycle_percent_botA = 75;
	/* Set new duty cycle value */
	//pwm_set_duty_cycle_percent(&pwm_botA, duty_cycle_percent_botA);
}

static void timerC1_tick(void){
	if (++step >= 6) {
		step = 0;
	}
	
	switch (step){
		case 0:
			ioport_set_pin_level(PWM_TOPA, 1);
			pwm_set_duty_cycle_percent(&pwm_botA, 0);
		
			ioport_set_pin_level(PWM_TOPB, 0);
			pwm_set_duty_cycle_percent(&pwm_botB, 10);
		
			ioport_set_pin_level(PWM_TOPC, 0);
			pwm_set_duty_cycle_percent(&pwm_botC, 0);
			break;
		case 1:
			ioport_set_pin_level(PWM_TOPA, 1);
			pwm_set_duty_cycle_percent(&pwm_botA, 0);
			
			ioport_set_pin_level(PWM_TOPB, 0);
			pwm_set_duty_cycle_percent(&pwm_botB, 0);
			
			ioport_set_pin_level(PWM_TOPC, 0);
			pwm_set_duty_cycle_percent(&pwm_botC, 10);
			break;
		case 2:
			ioport_set_pin_level(PWM_TOPA, 0);
			pwm_set_duty_cycle_percent(&pwm_botA, 0);
		
			ioport_set_pin_level(PWM_TOPB, 1);
			pwm_set_duty_cycle_percent(&pwm_botB, 0);
		
			ioport_set_pin_level(PWM_TOPC, 0);
			pwm_set_duty_cycle_percent(&pwm_botC, 10);
			break;
		case 3:
			ioport_set_pin_level(PWM_TOPA, 0);
			pwm_set_duty_cycle_percent(&pwm_botA, 10);//75
		
			ioport_set_pin_level(PWM_TOPB, 1);
			pwm_set_duty_cycle_percent(&pwm_botB, 0);
		
			ioport_set_pin_level(PWM_TOPC, 0);
			pwm_set_duty_cycle_percent(&pwm_botC, 0);
			break;
		case 4:
			ioport_set_pin_level(PWM_TOPA, 0);
			pwm_set_duty_cycle_percent(&pwm_botA, 10);//75
		
			ioport_set_pin_level(PWM_TOPB, 0);
			pwm_set_duty_cycle_percent(&pwm_botB, 0);
		
			ioport_set_pin_level(PWM_TOPC, 1);
			pwm_set_duty_cycle_percent(&pwm_botC, 0);
			break;
		case 5:
			ioport_set_pin_level(PWM_TOPA, 0);
			pwm_set_duty_cycle_percent(&pwm_botA, 0);
		
			ioport_set_pin_level(PWM_TOPB, 0);
			pwm_set_duty_cycle_percent(&pwm_botB, 10);
		
			ioport_set_pin_level(PWM_TOPC, 1);
			pwm_set_duty_cycle_percent(&pwm_botC, 0);
			break;
		default:
		break;
	}
}
void tc_init(void)
{
	//Initialisation du Timer 0
	tc_enable(&TCC1);	
	tc_set_overflow_interrupt_callback(&TCC1, timerC1_tick); //Cr?ation d'un callback qui sera execut? quand un overflow du timer sera d?clench?.
	tc_set_wgm(&TCC1, TC_WG_NORMAL);		//Choix du mode du timer0, dans ce cas il comptera jusqu'? sa valeur "TOP" et retombera ? 0
	tc_write_period(&TCC1, 15000);			//D?finition de la valeur "TOP"	
	tc_set_overflow_interrupt_level(&TCC1, TC_INT_LVL_LO);	//Activation de l'interruption du timer 				
	tc_write_clock_source(&TCC1, TC_CLKSEL_DIV8_gc);		//Activation de l'horloge du timer 0
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
	tc_init();

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

	while(1) {
		/* Do nothing. Everything is handPWM_TOPC by interrupts. */
	}
}
