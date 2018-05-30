#include "stm32f10x.h"
#include <stdio.h>
#include "macros.h"

static TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure; // only visible in this file
static TIM_OCInitTypeDef TimOCInitStructure;
extern u16 motor_pwm_period; // defined in main()
static int err_l1, err_l1_1, err_l1_2, delta_pulse_l1, pulse_l1,
           err_l2, err_l2_1, err_l2_2, delta_pulse_l2, pulse_l2,
           err_r1, err_r1_1, err_r1_2, delta_pulse_r1, pulse_r1,
           err_r2, err_r2_1, err_r2_2, delta_pulse_r2, pulse_r2;

extern int vl1_target, vl2_target, vr1_target, vr2_target, vl1, vl2, vr1, vr2;

void TIMx_PWMInit(uint16_t prescaler, uint16_t pulse)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    // TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
    // TIM_OCInitTypeDef TimOCInitStructure;
    /* GPIO settings below */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE); // GPIO clock
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    /* GPIO settings above */
    /* TIMx clock settings below */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE);
    TIM_TimeBaseInitStructure.TIM_Prescaler = prescaler;
    TIM_TimeBaseInitStructure.TIM_Period = motor_pwm_period;
    TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM5, &TIM_TimeBaseInitStructure);
    /* TIMx clock settings above */
    /* PWM config below */
    TimOCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
    TimOCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
    TimOCInitStructure.TIM_Pulse = pulse; // pulse evaluate
    TimOCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OC1Init(TIM5, &TimOCInitStructure);
    TIM_OC2Init(TIM5, &TimOCInitStructure);
    TIM_OC3Init(TIM5, &TimOCInitStructure);
    TIM_OC4Init(TIM5, &TimOCInitStructure); // channel 1-4 config
    TIM_OC1PreloadConfig(TIM5, TIM_OCPreload_Enable);
    TIM_OC2PreloadConfig(TIM5, TIM_OCPreload_Enable);
    TIM_OC3PreloadConfig(TIM5, TIM_OCPreload_Enable);
    TIM_OC4PreloadConfig(TIM5, TIM_OCPreload_Enable); // channel 1-4 config
    TIM_ARRPreloadConfig(TIM5, ENABLE);
    TIM_Cmd(TIM5, ENABLE);
    TIM_CtrlPWMOutputs(TIM5, ENABLE);
    /* PWM config above */
    // config direction control output to L298N
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE);
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_4 
                                | GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7;
    GPIO_Init(GPIOD, &GPIO_InitStructure);
    return;
}

void motor_pid_controller(int kp, int ki, int kd) // pid config for motor l1, l2, r1, r2
{
    // for motor l1:
	
    err_l1 = (vl1_target - vl1);
    delta_pulse_l1 = kp * (err_l1 - err_l1_1) + ki * err_l1 + kd * (err_l1 - 2 * err_l1_1 + err_l1_2);
    err_l1_2 = err_l1_1;
    err_l1_1 = err_l1;
    pulse_l1 += delta_pulse_l1;
    if (pulse_l1 < 0)
    {
        GPIO_SetBits(GPIOD, GPIO_Pin_0);
        GPIO_ResetBits(GPIOD, GPIO_Pin_1);
        if (pulse_l1 < -motor_pwm_period) pulse_l1 = -motor_pwm_period;
        TimOCInitStructure.TIM_Pulse = -pulse_l1;
    }
    else
    {
        GPIO_SetBits(GPIOD, GPIO_Pin_1);
        GPIO_ResetBits(GPIOD, GPIO_Pin_0);
        if (pulse_l1 > motor_pwm_period) pulse_l1 = motor_pwm_period;
        TimOCInitStructure.TIM_Pulse = pulse_l1;
    }
	  // printf("pulse = %d v = %d  err_l1 = %d \n", pulse_l1, vl1, err_l1);
    TIM_OC1Init(TIM5, &TimOCInitStructure);
    TIM_OC1PreloadConfig(TIM5, TIM_OCPreload_Enable);

    // for motor l2:
    err_l2 = (vl2_target - vl2);
    delta_pulse_l2 = kp * (err_l2 - err_l2_1) + ki * err_l2 + kd * (err_l2 - 2 * err_l2_1 + err_l2_2);
    err_l2_2 = err_l2_1;
    err_l2_1 = err_l2;
    pulse_l2 += delta_pulse_l2;
    if (pulse_l2 < 0)
    {
        GPIO_SetBits(GPIOD, GPIO_Pin_2);
        GPIO_ResetBits(GPIOD, GPIO_Pin_3);
        if (pulse_l2 < -motor_pwm_period) pulse_l2 = -motor_pwm_period;
        TimOCInitStructure.TIM_Pulse = -pulse_l2;
    }
    else
    {
        GPIO_SetBits(GPIOD, GPIO_Pin_3);
        GPIO_ResetBits(GPIOD, GPIO_Pin_2);
        if (pulse_l2 > motor_pwm_period) pulse_l2 = motor_pwm_period;
        TimOCInitStructure.TIM_Pulse = pulse_l2;
    }
    TIM_OC2Init(TIM5, &TimOCInitStructure);
    TIM_OC2PreloadConfig(TIM5, TIM_OCPreload_Enable);
		//printf("pulse = %d vl2 = %d  err_l2 = %d \n", pulse_l2, vl2, err_l2);

    // for motor r1:
    err_r1 = (vr1_target - vr1);
    delta_pulse_r1 = kp * (err_r1 - err_r1_1) + ki * err_r1 + kd * (err_r1 - 2 * err_r1_1 + err_r1_2);
    err_r1_2 = err_r1_1;
    err_r1_1 = err_r1;
    pulse_r1 += delta_pulse_r1;
    if (pulse_r1 < 0)
    {
        GPIO_SetBits(GPIOD, GPIO_Pin_4);
        GPIO_ResetBits(GPIOD, GPIO_Pin_5);
        if (pulse_r1 < -motor_pwm_period) pulse_r1 = -motor_pwm_period;
        TimOCInitStructure.TIM_Pulse = -pulse_r1;
    }
    else
    {
        GPIO_SetBits(GPIOD, GPIO_Pin_5);
        GPIO_ResetBits(GPIOD, GPIO_Pin_4);
        if (pulse_r1 > motor_pwm_period) pulse_r1 = motor_pwm_period;
        TimOCInitStructure.TIM_Pulse = pulse_r1;
    }
    TIM_OC3Init(TIM5, &TimOCInitStructure);
    TIM_OC3PreloadConfig(TIM5, TIM_OCPreload_Enable);

    // for motor r2:
    err_r2 = (vr2_target - vr2);
    delta_pulse_r2 = kp * (err_r2 - err_r2_1) + ki * err_r2 + kd * (err_r2 - 2 * err_r2_1 + err_r2_2);
    err_r2_2 = err_r2_1;
    err_r2_1 = err_r2;
    pulse_r2 += delta_pulse_r2;
    if (pulse_r2 < 0)
    {
        GPIO_SetBits(GPIOD, GPIO_Pin_6);
        GPIO_ResetBits(GPIOD, GPIO_Pin_7);
        if (pulse_r2 < -motor_pwm_period) pulse_r2 = -motor_pwm_period;
        TimOCInitStructure.TIM_Pulse = -pulse_r2;
    }
    else
    {
        GPIO_SetBits(GPIOD, GPIO_Pin_7);
        GPIO_ResetBits(GPIOD, GPIO_Pin_6);
        if (pulse_r2 > motor_pwm_period) pulse_r2 = motor_pwm_period;
        TimOCInitStructure.TIM_Pulse = pulse_r2; // debug, should be r2
    }
    TIM_OC4Init(TIM5, &TimOCInitStructure);
    TIM_OC4PreloadConfig(TIM5, TIM_OCPreload_Enable);
		//printf("pulse = %d vr2 = %d  err_r2 = %d \n", pulse_r2, vr2, err_r2);

    TIM_ARRPreloadConfig(TIM5, ENABLE);
    TIM_Cmd(TIM5, ENABLE);
    TIM_CtrlPWMOutputs(TIM5, ENABLE);
    return;
}
