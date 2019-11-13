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
 * board with LEDs connected to PE0 and PE4 may be used.
 *
 * \section exampledescription Description of the example
 * This example uses two PWM channels on two different LEDs. An interrupt is
 * triggered on each PWM TC overrun, and the interrupt handler will increase
 * the duty cycle of the PWM channels, thus fading the LEDs out (since the LEDs
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

struct pwm_config pwm_topA;
struct pwm_config pwm_botA;
struct pwm_config pwm_topB;

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
	if (duty_cycle_percent_topA++ >= 100) {
		duty_cycle_percent_topA = 50;
	}
	duty_cycle_percent_topA = 10;
	/* Set new duty cycle value */
	pwm_set_duty_cycle_percent(&pwm_topA, duty_cycle_percent_topA);
}

static void pwm_callback_topB (void)
{
	/* Increase (and wrap at 100) the duty cycle */
	if (duty_cycle_percent_topB++ >= 50) {
		duty_cycle_percent_topB = 10;
	}
	/* Set new duty cycle value */
	pwm_set_duty_cycle_percent(&pwm_topB, duty_cycle_percent_topB);
	
	//pwm_set_duty_cycle_percent(&pwm_topA, 20);
	
	
	if (step++ >= 6) {
		step = 0;
	}
	
	switch (step){
		case 0:
			pwm_set_duty_cycle_percent(&pwm_topA, 100);
			pwm_set_duty_cycle_percent(&pwm_botA, 0);
			break;
		case 1:
			pwm_set_duty_cycle_percent(&pwm_topA, 0);
			pwm_set_duty_cycle_percent(&pwm_botA, 0);
			break;
		case 2:
			pwm_set_duty_cycle_percent(&pwm_topA, 0);
			pwm_set_duty_cycle_percent(&pwm_botA, 0);
			break;
		case 3:
			pwm_set_duty_cycle_percent(&pwm_topA, 0);
			pwm_set_duty_cycle_percent(&pwm_botA, 75);
			break;
		case 4:
			pwm_set_duty_cycle_percent(&pwm_topA, 0);
			pwm_set_duty_cycle_percent(&pwm_botA, 75);
			break;
		case 5:
			pwm_set_duty_cycle_percent(&pwm_topA, 100);
			pwm_set_duty_cycle_percent(&pwm_botA, 0);
			break;
		default:
			break;
	}
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

	/*
	  Set up first PWM channel
	*/

	/* Set PWM to TC E0, channel A (PD0 = LED0), 75 Hz */
	pwm_init(&pwm_topA, PWM_TCD0, PWM_CH_A, 2000);
	//pwm_overflow_int_callback(&pwm_topA, pwm_callback_1);
	pwm_init(&pwm_topB, PWM_TCD0, PWM_CH_B, 2000);
	pwm_overflow_int_callback(&pwm_topB, pwm_callback_topB);

	/*
	  Set up second PWM channel
	*/

	/* Set PWM to TC E1, channel A (PC0 = LED4), 250 Hz */
	pwm_init(&pwm_botA, PWM_TCC0, PWM_CH_A, 25000);
	//pwm_overflow_int_callback(&pwm_botA, pwm_callback_2);

	/*
	  Start PWM
	*/

	pwm_start(&pwm_topA, duty_cycle_percent_topA);
	pwm_start(&pwm_topB, duty_cycle_percent_topB);
	pwm_start(&pwm_botA, duty_cycle_percent_botA);

	while(1) {
		/* Do nothing. Everything is handled by interrupts. */
	}
}
