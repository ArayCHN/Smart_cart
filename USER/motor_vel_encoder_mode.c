#include "stm32f10x.h"
#include <stdio.h>
#include "macros.h"

extern int last_cnt_l1, counts_l1, timer8cnt; // signed! cuz v can be negative
extern int last_cnt_r1, counts_r1, timer4cnt; // signed! cuz v can be negative

void EncodeInit(void)
{
    GPIO_InitTypeDef GPIO_InitStructure; // PB6, PB7
    TIM_TimeBaseInitTypeDef  TIM4_TimeBaseStructure; // TIM4 CH1, 2 - PB6, 7 - Encoder_R1
    TIM_ICInitTypeDef TIM4_ICInitStructure;
    TIM_TimeBaseInitTypeDef  TIM8_TimeBaseStructure; // TIM8 CH1, 2 - PC6, 7 - Encoder_R1
    TIM_ICInitTypeDef TIM8_ICInitStructure;
	  
	  // TIM4:

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


    // TIM8:

    /* TIM8 Clock && GPIO enable */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);

    /* Configure PB6, 7 */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;       
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING; // Problematic
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 
    GPIO_Init(GPIOC, &GPIO_InitStructure);

    /* Configure TIM8 */
    TIM8_TimeBaseStructure.TIM_Prescaler = 0; // encoder signal as time source, do not prescale        
    TIM8_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM8_TimeBaseStructure.TIM_Period = encoder_counter_reload - 1; // what value?
    TIM8_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM8_TimeBaseStructure.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(TIM8, &TIM8_TimeBaseStructure);

    TIM_EncoderInterfaceConfig(TIM8, TIM_EncoderMode_TI12, TIM_ICPolarity_BothEdge,
                               TIM_ICPolarity_BothEdge);

    TIM_ICStructInit(&TIM8_ICInitStructure);
    TIM8_ICInitStructure.TIM_ICFilter = 6; // filter, choose what value?
    TIM_ICInit(TIM8, &TIM8_ICInitStructure);

    TIM_ClearFlag(TIM8, TIM_FLAG_Update);
    // TIM_ITConfig(TIM8, TIM_IT_Update, ENABLE);

    TIM_SetCounter(TIM8, 0); // TIM8 -> CNT = 0
    TIM_Cmd(TIM8, ENABLE);
}

int get_encoder_counts_l1() // from TIM8
{
    timer8cnt = TIM8 -> CNT; // for debug
    counts_l1 = timer8cnt - last_cnt_l1;
    if (counts_l1 < - encoder_counter_reload / 2) // counter overflowed
        counts_l1 += encoder_counter_reload;
        else
            if (counts_l1 > encoder_counter_reload / 2) // counter overflowed
        counts_l1 -= encoder_counter_reload;
    last_cnt_l1 = TIM8 -> CNT;
    return counts_l1;
}

int get_encoder_counts_r1() // from TIM4
{
    timer4cnt = TIM4 -> CNT; // for debug
    counts_r1 = timer4cnt - last_cnt_r1;
    if (counts_r1 < - encoder_counter_reload / 2) // counter overflowed
        counts_r1 += encoder_counter_reload;
        else
            if (counts_r1 > encoder_counter_reload / 2) // counter overflowed
        counts_r1 -= encoder_counter_reload;
    last_cnt_r1 = TIM4 -> CNT;
    return counts_r1;
}

void encoder_vel_calc()
{
    extern int vl1, vr1;
    int omega, num, den;
    // v = get_encoder_counts() * wheel_perimeter * 1000 / (spokes_num * reduction_ratio * sysTick_period * 4); // update velocity
    // v in mm/s; all vals must be signed int32 so that the multiplication doesn't overflow & there are pos & ne
    omega = get_encoder_counts_l1(); // n counts per 50ms
    den = spokes_num * reduction_ratio * encoder_period * 4; // divide encoder counter by 4
    num = omega * wheel_perimeter * 1000 * 10; // *10 cuz reduction ratio is 21.3 instead of 213
    vl1 = num / den;

    omega = get_encoder_counts_r1(); // n circles' 50ms
    den = spokes_num * reduction_ratio * encoder_period * 4; // divide encoder counter by 4
    num = omega * wheel_perimeter * 1000 * 10; // *10 cuz reduction ratio is 21.3 instead of 213
    vr1 = num / den;
}