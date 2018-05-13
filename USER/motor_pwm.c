#include "stm32f10x.h"
#include <stdio.h>
#include "macros.h"

static TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure; // only visible in this file
static TIM_OCInitTypeDef TimOCInitStructure;
extern u16 motor_pwm_period; // defined in main()
static int err, err1, err2, delta_pulse, pulse;

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

void motor_pid_controller(int kp, int ki, int kd)
{
    // int R, delta_pulse;
    // extern int R_target; // R_target is given by the general controller, global var
    // R = W * (vr1 + vr2 + vl1 + vl2) / (vr1 - vl1 + vr2 - vl2) / 2; // is prevision good?
    // err = R - R_target;
    extern int v_target, v;
    err = (v_target - v);
    delta_pulse = kp * (err - err1) + ki * err + kd * (err - 2 * err1 + err2);
    err2 = err1;
    err1 = err;
    pulse += delta_pulse;
    if (pulse < 0)
    {
        GPIO_SetBits(GPIOD, GPIO_Pin_0);
        GPIO_ResetBits(GPIOD, GPIO_Pin_1);
        if (pulse < -motor_pwm_period) pulse = -motor_pwm_period;
        TimOCInitStructure.TIM_Pulse = -pulse;
    }
    else
    {
        GPIO_SetBits(GPIOD, GPIO_Pin_1);
        GPIO_ResetBits(GPIOD, GPIO_Pin_0);
        if (pulse > motor_pwm_period) pulse = motor_pwm_period;
        TimOCInitStructure.TIM_Pulse = pulse;
    }
		printf("pulse = %d v = %d  err = %d \n", pulse, v, err);

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
    TIM_Cmd(TIM5, ENABLE);
    TIM_CtrlPWMOutputs(TIM5, ENABLE);
    return;
}
