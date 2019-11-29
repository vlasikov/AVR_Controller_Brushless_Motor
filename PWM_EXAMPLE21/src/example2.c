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
#include <init.h>
#include <motor.h>

/**
 * \brief Example 2 main application routine
 */
uint16_t rv_reg;

int main( void )
{
	/* Initialize interrupt controller, board and sysclock */
	pmic_init();
	sysclk_init();
/*	
	ioport_set_pin_dir(PWM_TOPA, IOPORT_DIR_OUTPUT);
	ioport_set_pin_dir(PWM_TOPB, IOPORT_DIR_OUTPUT);
	ioport_set_pin_dir(PWM_TOPC, IOPORT_DIR_OUTPUT);
	//tcc1_init();
	ioport_set_pin_dir(IOPORT_CREATE_PIN(PORTD, 3), IOPORT_DIR_OUTPUT);
	//tcd1_init();
	timerInit();
	
	ioport_set_pin_dir(IOPORT_CREATE_PIN(PORTC, 3), IOPORT_DIR_OUTPUT);
	ioport_set_pin_dir(IOPORT_CREATE_PIN(PORTC, 4), IOPORT_DIR_OUTPUT);
	ioport_set_pin_dir(IOPORT_CREATE_PIN(PORTC, 5), IOPORT_DIR_OUTPUT);

	
	pwmInit();	
	acInit();
	//adcInit();
*/	
	int_adcb_init();

//	init_adc();
	
	/* Enable global interrupts */
	cpu_irq_enable();
	
	while(1) {
		/* Do nothing. Everything is handPWM_TOPC by interrupts. */
		
		start_int_adcb_conv();
/*		
		while(((ADCB.CH0.INTFLAGS & ADC_CH_CHIF_bm) == 0x00));
		rv_reg = ADCB.CH0RES;
		if(rv_reg>0){
			rv_reg = 1;
		}
*/
	}
}
