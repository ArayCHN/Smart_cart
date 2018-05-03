#include "stm32f10x.h"
#include <stdio.h>
#include "macros.h"

extern static int cnt_ch3, prev_cnt_ch3, cnt_ch4, prev_cnt_ch4, delta_t3, delta_t4, cycles3, cycles4;
extern u32 v3, v4, num, den;

extern void TIM8_CC_IRQHandler()
{
    if (TIM_GetITStatus(TIM8, TIM_IT_CC3) == SET) // if it is channel 3
    {
        TIM_ClearITPendingBit(TIM8, TIM_IT_CC3);
        cnt_ch3 = TIM_GetCapture3(TIM8);
        delta_t3 =  cycles3 * time_interval_reload + cnt_ch3 - prev_cnt_ch3;
        
        
        num = (u32)(wheel_perimeter) * (u32)(SystemCoreClock / time_interval_prescaler) * 10;
        den = spokes_num * reduction_ratio * (delta_t3 / 4);
        v3 = num / den;
        // if (v3 > 1000); //debug
        cycles3 = 0;
        prev_cnt_ch3 = cnt_ch3;
        // printf("v3:%d    v4: %d\n", v3, v4); // debug
    }
    else if (TIM_GetITStatus(TIM8, TIM_IT_CC4) == SET)
    {
        TIM_ClearITPendingBit(TIM8, TIM_IT_CC4);
        cnt_ch4 = TIM_GetCapture4(TIM8);
        delta_t4 =  cycles4 * time_interval_reload + cnt_ch4 - prev_cnt_ch4;
        prev_cnt_ch4 = cnt_ch4;
        cycles4 = 0;

        num = (u32)(wheel_perimeter) * (u32)(SystemCoreClock / time_interval_prescaler) * 10;
        den = spokes_num * reduction_ratio * (delta_t4 / 4);
        v4 = num / den;
    }
    else; // other interrupt?? do nothing! how come there is other interrupt?
    return;
}

extern void TIM8_UP_IRQHandler() // runs every 0.083s. acceptable
{
    if (TIM_GetITStatus(TIM8, TIM_IT_Update) == SET)
    {
        cycles3 ++;
        if (cycles3 > 5)
            {
                v3 = 0;
                if (cycles3 > 10) cycles3 = 10;
            }
        cycles4 ++;
        if (cycles4 > 5)
            {
                v4 = 0;
                if (cycles4 > 10) cycles4 = 10;
            }
        TIM_ClearITPendingBit(TIM8, TIM_IT_Update);
    }
    return;
}

/* initialize TIM8 for input detection for encoder  */
void TIM8_Cap_Init() // capture mode
{      
    TIM_ICInitTypeDef  TIM8_ICInitStructure;
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
    NVIC_InitTypeDef NVIC_InitStructure;
    GPIO_InitTypeDef GPIO_InitStructure;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8, ENABLE);  
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);

    /* Configure PC8, 9 */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9;       
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD; // pull-down input
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 
    GPIO_Init(GPIOC, &GPIO_InitStructure);
    GPIO_ResetBits(GPIOC, GPIO_Pin_8 | GPIO_Pin_9);

    // Below: TIM8 init, must be the same as TIM8 init in encoder mode!
    TIM_TimeBaseStructure.TIM_Period = time_interval_reload - 1;
    TIM_TimeBaseStructure.TIM_Prescaler = time_interval_prescaler - 1; // down to 720000 Hz
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; //tDTS = tCK_INT  
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseStructure.TIM_RepetitionCounter = 0; // only for TIM1 and 8
    TIM_TimeBaseInit(TIM8, &TIM_TimeBaseStructure);

    TIM_ICStructInit(&TIM8_ICInitStructure);
    TIM8_ICInitStructure.TIM_Channel = TIM_Channel_3; 
    TIM8_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising; 
    TIM8_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
    TIM8_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV4; // prescale input signal, capture performs every 4 events
    TIM8_ICInitStructure.TIM_ICFilter = 0; // filter: 0
    TIM_ICInit(TIM8, &TIM8_ICInitStructure);
    TIM8_ICInitStructure.TIM_Channel = TIM_Channel_4;
    TIM_ICInit(TIM8, &TIM8_ICInitStructure);
    
    // interuption config
    NVIC_InitStructure.NVIC_IRQChannel = TIM8_CC_IRQn;
        
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2; // lower than systick (0) cuz this vel updates much faster   
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;   
    NVIC_Init(&NVIC_InitStructure);

    NVIC_InitStructure.NVIC_IRQChannel = TIM8_UP_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_Init(&NVIC_InitStructure);

    TIM_ITConfig(TIM8, TIM_IT_CC3|TIM_IT_CC4|TIM_IT_Update, ENABLE); // channel 3, 4
    TIM_Cmd(TIM8, ENABLE);
}
