#ifndef __MOTOR_PWM_H__
#define __MOTOR_PWM_H__

void TIMx_PWMInit(uint16_t prescaler, uint16_t pulse);
void motor_pid_controller(int, int, int);

#endif