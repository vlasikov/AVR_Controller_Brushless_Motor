/*
 * motor.c
 *
 * Created: 25.11.2019 16:44:19
 *  Author: yu.vlasikov
 */ 

#include <motor.h>
#include <asf.h>

//Status{STOP = 0, START = 1, RUN = 2,};
extern struct ac_config aca_config;
static uint16_t step = 0;
static uint8_t step_old = 0;
static uint16_t DelyayForMotor = 0;
static uint16_t DelyayForMotorMax = 1;			// ����� ������������ ���� ����� �������� ������� ����� 0
uint8_t MotorStatus = STOP;
static uint8_t MotorPower = MOTOR_POWER_START;

uint16_t Top_tc_period = MOTOR_PERIOD_START;

extern uint16_t ADC;

/*
 * �������� ���� ���������� �� 60 ��������
 */
void MotorNextPhase(){	
	if (++step >= 6) {
		step = 0;
	}
	
	switch (step){
		case 0:
			if (MotorStatus == START){
				Top_tc_period *= 0.90;
			}
			
			if (Top_tc_period < 4500){
				MotorStatus = RUN;
			}
			
			if (MotorStatus == RUN) {								// ���������� �������� �� �������������
				if (ADC < 2000) {
					ADC = 2000;
				}
				if (ADC > 3000) {
					ADC = 3000;
				}
					
				MotorPower = 23.0 * (1.0 + (ADC - 2500)/1500.0); // �� 15 �� 30 ������ 23*1.3
			}
			tc_write_period(&TCC1, Top_tc_period);
		
			// ���������� ��������� �����������
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
 * ����� ������� ������.
 * � ������ 22 �������. ������� ���������� 1850 �������� � ������ (30.8 ��).
 * �� ���� ������ ���� ����� �������� ��� �������, �.�. �� ������ ������ ���� ������ 11 ��������.
 * �.�. �� ���� ����� ��� �� ��������� � �������� 30.8 * 11 = 339 ��, ��� �������� 3 ��.
 * �.�. ���� ����� ��������� ������� ��������� (� 2-3 ����) ��� �����, ��� �������������. 
 * ����������� ����� ��� ������� �������� 8 �� ��� 8000 ���.
 */
static uint16_t criticalTime_us = 0; // ������� �������� ���������. ������������ 25 ���
const  uint16_t criticalTimeMax_us = 8000;

/*
 * ����� ���������� �� ������� (PWM)
 * �������� ��������� ��������� ��������� � ������ ����� ���� �������� � ������� ������ (�� ��� �������)
 * ������� ������ �������� 25 ���
 * ������� �� ���� PC3, 4, 5 ���� ��� A, B, C. ��������������.
 */
void MotorPhazeControl2() {
	uint8_t ac0Out;
	if (ac_get_status(&ACA, 0)){												// ��������� ������� ��������� ����
		ac0Out = 0;
	}
	else{
		ac0Out = 1;
	}
	
	if (MotorStatus == RUN){
 		if (criticalTime_us > criticalTimeMax_us){								// ������ �� ������������ ��������� 2500 - 600 RPM, �� ����� 200 ����, ���� �����-��
 			MotorStop();
 		}
	}
	criticalTime_us += 40;
	switch (step){
		case 0:
			if (step_old != step) {												// ���� ���� ���������� ����������
				ac_disable(&ACA, 0);											// ����������� ���������� � ���������� ����
				ac_set_positive_reference(&aca_config, AC_MUXPOS_PIN0_gc);		// phase C
				ac_write_config(&ACA, 0, &aca_config);							// 
				ac_enable(&ACA, 0);												// 
			
				DelyayForMotor = 0;
				
				criticalTime_us = 0;													// �������� ����� ������� ���������
			}
		
			if (ac0Out)															// phase C
				ioport_set_pin_level(IOPORT_CREATE_PIN(PORTC, 5), 1);
			else
				ioport_set_pin_level(IOPORT_CREATE_PIN(PORTC, 5), 0);
		
			if (!ac0Out){
				if (DelyayForMotor > DelyayForMotorMax){
					if (MotorStatus == RUN){
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
					if (MotorStatus == RUN){
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
					if (MotorStatus == RUN){
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
					if (MotorStatus == RUN){
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
					if (MotorStatus == RUN){
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
					if (MotorStatus == RUN){
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
 * ��������� ���������
 */

extern uint16_t systemTime_ms;
/*
 *
 */
void MotorStop(){
	ioport_set_pin_level(PWM_TOPA, 0);
	pwm_set_duty_cycle_percent(&pwm_botA, 0);
	
	ioport_set_pin_level(PWM_TOPB, 0);	
	pwm_set_duty_cycle_percent(&pwm_botC, 0);
	
	ioport_set_pin_level(PWM_TOPC, 1);
	pwm_set_duty_cycle_percent(&pwm_botB, 0);
	
	// ��� �������� � ��������� ��������
	Top_tc_period = MOTOR_PERIOD_START;
	MotorPower = MOTOR_POWER_START;
	MotorStatus = STOP;	
	systemTime_ms = 0;
}
