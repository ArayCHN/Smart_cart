#include "stm32f10x.h"
#include <stdio.h>
#include "macros.h"

void EncodeInit(void)
{
    GPIO_InitTypeDef GPIO_InitStructure; // PB6, PB7
    TIM_TimeBaseInitTypeDef  TIM4_TimeBaseStructure; // TIM4 CH1, 2 - PB6, 7 - Encoder_R1
    TIM_ICInitTypeDef TIM4_ICInitStructure;

    /* TIM4 Clock && GPIO enable */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

    /* Configure PB6, 7 */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;       
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING; // Problematic
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    /* Configure TIM4 */
    TIM4_TimeBaseStructure.TIM_Prescaler = 0; // encoder signal as time source, do not prescale        
    TIM4_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM4_TimeBaseStructure.TIM_Period = encoder_counter_reload - 1; // what value?
    TIM4_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM4_TimeBaseStructure.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(TIM4, &TIM4_TimeBaseStructure);

    TIM_EncoderInterfaceConfig(TIM4, TIM_EncoderMode_TI12, TIM_ICPolarity_BothEdge,
                               TIM_ICPolarity_BothEdge);

    TIM_ICStructInit(&TIM4_ICInitStructure);
    TIM4_ICInitStructure.TIM_ICFilter = 6; // filter, choose what value?
    TIM_ICInit(TIM4, &TIM4_ICInitStructure);

    TIM_ClearFlag(TIM4, TIM_FLAG_Update);
    // TIM_ITConfig(TIM4, TIM_IT_Update, ENABLE);

    TIM_SetCounter(TIM4, 0); // TIM4 -> CNT = 0
    TIM_Cmd(TIM4, ENABLE);
}

extern static int last_cnt, counts, v, timer4cnt; // signed! cuz v can be negative

int get_encoder_counts()
{
    timer4cnt = TIM4 -> CNT; // for debug
    counts = timer4cnt - last_cnt;
    if (counts < - encoder_counter_reload / 2) // counter overflowed
        counts += encoder_counter_reload;
        else
            if (counts > encoder_counter_reload / 2) // counter overflowed
        counts -= encoder_counter_reload;
    last_cnt = TIM4 -> CNT;
    return counts;
}
