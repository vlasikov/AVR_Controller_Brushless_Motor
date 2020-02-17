/*
 * motor.h
 *
 * Created: 25.11.2019 16:44:49
 *  Author: yu.vlasikov
 */ 


#ifndef MOTOR_H_
#define MOTOR_H_

#define MOTOR_POWER_START 15
#define MOTOR_PERIOD_START 30000

#define PWM_TOPA IOPORT_CREATE_PIN(PORTD, 0)
#define PWM_TOPB IOPORT_CREATE_PIN(PORTD, 1)
#define PWM_TOPC IOPORT_CREATE_PIN(PORTD, 2)

struct pwm_config pwm_botA;
struct pwm_config pwm_botB;
struct pwm_config pwm_botC;

void MotorPhazeControl2();
void MotorStop();
void MotorNextPhase();

enum Status{STOP, START, RUN,};


#endif /* MOTOR_H_ */