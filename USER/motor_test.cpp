#include "stm32f10x.h"

void TIMx_PWMInit(char TIM_x, char TIM_channel, uint16_t prescaler, 
                  uint16_t period, uint16_t pulse, char GPIO_port, char GPIO_pin)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
    TIM_OCInitTypeDef TimOCInitStructure;
    /* GPIO settings below */
    switch GPIO_port
    {
        case 'A': RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE); // GPIO clock
        case 'B': RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
        case 'C': RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
        case 'D': RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE);
    }

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0; // FIXME!!!!!!!!!!! not necessarily pin 9
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure); // FIXME!!!!!!!!!!! not necessarily GPIOA
    /* GPIO settings above */
    /* TIMx clock settings below */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);
    TIM_TimeBaseInitStructure.TIM_Prescaler = prescaler; // FIXME:!!! don't know data type!! find out first before assign prescaler u_int16_t
    TIM_TimeBaseInitStructure.TIM_Period = period; // FIXME:!!! don't know data type!!
    TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
    switch TIM_x
    {
        case 1: TIM_TimebaseInit(TIM1, &TIM_TimeBaseInitStructure);
        case 2: TIM_TimebaseInit(TIM2, &TIM_TimeBaseInitStructure);
        case 3: TIM_TimebaseInit(TIM3, &TIM_TimeBaseInitStructure);
        case 4: TIM_TimebaseInit(TIM4, &TIM_TimeBaseInitStructure);
        case 5: TIM_TimebaseInit(TIM5, &TIM_TimeBaseInitStructure);
        case 6: TIM_TimebaseInit(TIM6, &TIM_TimeBaseInitStructure);
        case 7: TIM_TimebaseInit(TIM7, &TIM_TimeBaseInitStructure);
        case 8: TIM_TimebaseInit(TIM8, &TIM_TimeBaseInitStructure);
    }
    /* TIMx clock settings above */
    /* PWM config below */
    TimOCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
    TimOCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
    TimOCInitStructure.TIM_Pulse = pulse; // FIXME:!!! don't know data type!!
    TimOCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OC2Init(TIM1, &TimOCInitStructure); // FIXME!!! don't know channel 2!!
    TIM_OC2PreloadConfig(TIM1, Config); // FIXME!!! don't know TIM1, channel 2!! AND every sentence below
    TIM_ARRPreloadConfig(TIM1, Config);
    TIM_Cmd(TIM1, ENABLE);
    TIM_CtrlPWMOutputs(TIM1, ENABLE);
    /* PWM config above */
    return;
}

int main()
{
    char GPIO_port = 'A';
    char GPIO_pin = 9; // 0, 1, 2, 3
    char TIM_x = 1; // TIM data type: a struct pointer
    char TIM_channel = 2;
    uint16_t prescaler = 72 - 1; // 0~65535
    uint16_t period = 49;
    uint16_t pulse = 30;
    TIMx_PWMInit(TIM_x, TIM_channel, prescaler, period, pulse, GPIO_port, GPIO_pin); // set GPIO
    while (1);
	return 0;
}