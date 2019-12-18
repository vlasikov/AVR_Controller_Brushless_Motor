/*
 * motor.c
 *
 * Created: 25.11.2019 16:44:19
 *  Author: yu.vlasikov
 */ 

#include <motor.h>
#include <asf.h>

extern struct ac_config aca_config;
static uint16_t step = 0;
static uint8_t step_old = 0;
static uint16_t DelyayForMotor = 0;
static uint16_t DelyayForMotorMax = 2;
uint8_t MotorStatus = 0;
static uint8_t MotorPower = 16;

uint16_t Top_tc_period = 15000;

extern uint16_t ADC;

/*
 * —мещение фазы управлени€ на 60 градусов
 */
void MotorNextPhase(){	
	if (++step >= 6) {
		step = 0;
	}
	
	switch (step){
		case 0:
			if (MotorStatus == 1){
				Top_tc_period *= 0.95;
			}
			
			if (Top_tc_period < 5000){
				MotorPower = 22;
				MotorStatus = 2;
			}
			
			if (MotorStatus == 2) {								// выставл€ем мощность по потенциометру
				if (ADC < 2000) {
					ADC = 2000;
				}
				if (ADC > 3000) {
					ADC = 3000;
				}
					
				MotorPower = 23.0 * (1.0 + (ADC - 2500)/1500.0); // от 15 до 30 хватит 23*1.3
			}
			tc_write_period(&TCC1, Top_tc_period);
		
			// выставл€ем правильно транзисторы
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

/*
 * ‘ункици€ провер€ет положение двигател€ и задает новый угол повората в рабочем режиме (не при разгоне)
 * частота вызова статична 25 к√ц
 * выводит на пины PC3, 4, 5 знак фаз A, B, C. соответственно.
 */
static uint16_t counter = 0; // счетчик оборотов двигател€. такстируетс€ 25 к√ц

void MotorPhazeControl2() {
	uint8_t ac0Out;
	if (ac_get_status(&ACA, 0)){												// считываем уровень свободной фазы
		ac0Out = 0;
	}
	else{
		ac0Out = 1;
	}
	
	if (MotorStatus == 2){
 		if (counter > 200){														// 2500 - 600 RPM
 			MotorStop();
 		}
	}
	counter ++;
	switch (step){
		case 0:
			if (step_old != step) {												// если фаза управлени€ помен€лась
				ac_disable(&ACA, 0);											// переключаем компаратор к следующему пину
				ac_set_positive_reference(&aca_config, AC_MUXPOS_PIN0_gc);		// phase C
				ac_write_config(&ACA, 0, &aca_config);							// 
				ac_enable(&ACA, 0);												// 
			
				DelyayForMotor = 0;
				
				counter = 0;													// обнул€ем врем€ просто€ двигател€
			}
		
			if (ac0Out)															// phase C
				ioport_set_pin_level(IOPORT_CREATE_PIN(PORTC, 5), 1);
			else
				ioport_set_pin_level(IOPORT_CREATE_PIN(PORTC, 5), 0);
		
			if (!ac0Out){
				if (DelyayForMotor > DelyayForMotorMax){
					if (MotorStatus == 2){
						MotorNextPhase();
						return;
					}
				}
				DelyayForMotor++;
			}
			else{
				DelyayForMotor = 0;
			}
			break;
		case 1:
			if (step_old != step) {
				ac_disable(&ACA, 0);
				ac_set_positive_reference(&aca_config, AC_MUXPOS_PIN1_gc);		// phase B
				ac_write_config(&ACA, 0, &aca_config);
				ac_enable(&ACA, 0);
			
				DelyayForMotor = 0;
			}
		
			if (ac0Out)															// phase B
				ioport_set_pin_level(IOPORT_CREATE_PIN(PORTC, 4), 1);
			else
				ioport_set_pin_level(IOPORT_CREATE_PIN(PORTC, 4), 0);
		
			if (ac0Out){
				if (DelyayForMotor > DelyayForMotorMax){
					if (MotorStatus == 2){
						MotorNextPhase();
						return;
					}
				}
				DelyayForMotor++;
			}
			else{
				DelyayForMotor = 0;
			}
			break;
		case 2:
			if (step_old != step) {
				ac_disable(&ACA, 0);
				ac_set_positive_reference(&aca_config, AC_MUXPOS_PIN2_gc);		// phase A
				ac_write_config(&ACA, 0, &aca_config);
				ac_enable(&ACA, 0);
			
				DelyayForMotor = 0;
			}
		
			if (ac0Out)															// phase A
				ioport_set_pin_level(IOPORT_CREATE_PIN(PORTC, 3), 1);
			else
				ioport_set_pin_level(IOPORT_CREATE_PIN(PORTC, 3), 0);
		
			if (!ac0Out){
				if (DelyayForMotor > DelyayForMotorMax){
					if (MotorStatus == 2){
						MotorNextPhase();
						return;
					}
				}
				DelyayForMotor++;
			}
			else{
				DelyayForMotor = 0;
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
				if (DelyayForMotor > DelyayForMotorMax){
					if (MotorStatus == 2){
						MotorNextPhase();
						return;
					}
				}
				DelyayForMotor++;
			}
			else{
				DelyayForMotor = 0;
			}
			break;
		case 4:
			if (step_old != step) {
				ac_disable(&ACA, 0);
				ac_set_positive_reference(&aca_config, AC_MUXPOS_PIN1_gc);		// phase B
				ac_write_config(&ACA, 0, &aca_config);
				ac_enable(&ACA, 0);
			
				DelyayForMotor = 0;
			}
		
			if (ac0Out)															// phase B
				ioport_set_pin_level(IOPORT_CREATE_PIN(PORTC, 4), 1);
			else
				ioport_set_pin_level(IOPORT_CREATE_PIN(PORTC, 4), 0);
		
			if (!ac0Out){
				if (DelyayForMotor > DelyayForMotorMax){
					if (MotorStatus == 2){
						MotorNextPhase();
						return;
					}
				}
				DelyayForMotor++;
			}
			else{
				DelyayForMotor = 0;
			}
			break;
		case 5:
			if (step_old != step) {
				ac_disable(&ACA, 0);
				ac_set_positive_reference(&aca_config, AC_MUXPOS_PIN2_gc);		// phase A
				ac_write_config(&ACA, 0, &aca_config);
				ac_enable(&ACA, 0);
			
				DelyayForMotor = 0;
			}
		
			if (ac0Out)															// phase A
				ioport_set_pin_level(IOPORT_CREATE_PIN(PORTC, 3), 1);
			else
				ioport_set_pin_level(IOPORT_CREATE_PIN(PORTC, 3), 0);
		
			if (ac0Out){
				if (DelyayForMotor > DelyayForMotorMax){
					if (MotorStatus == 2){
						MotorNextPhase();
						return;
					}
				}
				DelyayForMotor++;
			}
			else{
				DelyayForMotor = 0;
			}
			break;
		default:
			break;
	}
	step_old = step;
}

/*
 * ќстановка двигател€
 */

void MotorStop(){
	ioport_set_pin_level(PWM_TOPA, 0);
	pwm_set_duty_cycle_percent(&pwm_botA, 0);
	
	ioport_set_pin_level(PWM_TOPB, 0);	
	pwm_set_duty_cycle_percent(&pwm_botC, 0);
	
	ioport_set_pin_level(PWM_TOPC, 1);
	pwm_set_duty_cycle_percent(&pwm_botB, 0);
	
	MotorStatus = 0;
}
