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

#include <asf.h>
#include <motor.h>
#include <init.h>

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

#define PD3 IOPORT_CREATE_PIN(PORTD, 3)
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
	
	ioport_set_pin_dir(IOPORT_CREATE_PIN(PORTD, 3), IOPORT_DIR_OUTPUT);
	timerInit();
	
	ioport_set_pin_dir(IOPORT_CREATE_PIN(PORTC, 3), IOPORT_DIR_OUTPUT);
	ioport_set_pin_dir(IOPORT_CREATE_PIN(PORTC, 4), IOPORT_DIR_OUTPUT);
	ioport_set_pin_dir(IOPORT_CREATE_PIN(PORTC, 5), IOPORT_DIR_OUTPUT);

	pwmInit();
	
	acInit();
	
	while(1) {
		/* Do nothing. Everything is handPWM_TOPC by interrupts. */
	}
}
