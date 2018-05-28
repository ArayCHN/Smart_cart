// TIM1 CH1, 4
#include "stm32f10x.h"
#include <stdio.h>
#include "macros.h"
#include "stdlib.h" // abs()

extern int cnt_ch1, prev_cnt_ch1, cnt_ch4, prev_cnt_ch4, delta_t1, delta_t4, cycles1, cycles4, vl2, vr2;
extern u32 num, den;
u8 positive_direction = 1;
static int value_series[2][6], sum[2]; // value_series[0][i]: recent 5 values of v_l2; value_series[1][i]: recent 5 values of v_r2; 
// sum[0]: sum of recent 5 values of v_l2

int mean_filter(int new_value, int num) // mean value filter, result in time lag, no longer in use
{
	int i, mean_value;
	sum[num] = sum[num] + new_value - value_series[num][0];
    value_series[num][5] = new_value;
    for (i = 0; i <= 4; i ++)
    {
        value_series[num][i] = value_series[num][i + 1];
    }
    //mean_value = (value_series[num][0] + value_series[num][1] + value_series[num][2] + value_series[num][3] + value_series[num][4]) / 5;
		mean_value = sum[num] / 5;
		//printf("value_series:%d %d %d %d %d %d \n", value_series[num][0], value_series[num][1], value_series[num][2], value_series[num][3], value_series[num][4], value_series[num][5]);
    return mean_value;
}

extern void TIM1_CC_IRQHandler()
{
	  int tmp;
    if (GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_5) == SET)
        positive_direction = 1;
    else
        positive_direction = 0; // determine direction
    if (TIM_GetITStatus(TIM1, TIM_IT_CC1) == SET) // if it is channel 1
    {
        TIM_ClearITPendingBit(TIM1, TIM_IT_CC1);
        cnt_ch1 = TIM_GetCapture1(TIM1);
        delta_t1 = cycles1 * time_interval_reload + cnt_ch1 - prev_cnt_ch1;
        // delta_t1 = mean_filter(delta_t1, 0); // 0 for function mean_filter() to recognize as v_l2

        num = (u32)(wheel_perimeter) * (u32)(SystemCoreClock / time_interval_prescaler) * 10;
        den = spokes_num * reduction_ratio * delta_t1;
			  tmp = vl2;
        vl2 = num / den; // u32, only positive!
        if (!positive_direction) vl2 = - vl2;
        // if (vl2 > 1000); // debug
			  // vl2 = mean_filter(vl2, 0);
			  if (abs(vl2) > 1000) vl2 = tmp;
        cycles1 = 0;
        prev_cnt_ch1 = cnt_ch1;
        // printf("vl2:%d    direction: %d\n", vl2, positive_direction); // debug
    }
    else if (TIM_GetITStatus(TIM1, TIM_IT_CC4) == SET)
    {
        TIM_ClearITPendingBit(TIM1, TIM_IT_CC4);
        cnt_ch4 = TIM_GetCapture4(TIM1);
        delta_t4 =  cycles4 * time_interval_reload + cnt_ch4 - prev_cnt_ch4;
        // delta_t4 = mean_filter(delta_t4, 1); // 1 for function mean_filter() to recognize as v_r2

        num = (u32)(wheel_perimeter) * (u32)(SystemCoreClock / time_interval_prescaler) * 10;
        den = spokes_num * reduction_ratio * delta_t4;
			  tmp = vr2;
        vr2 = num / den;
			// vr2 = mean_filter(vl2, 0);
        if (!positive_direction) vr2 = - vr2;
			  if (abs(vr2) > 1000) vr2 = tmp;
        prev_cnt_ch4 = cnt_ch4;
        cycles4 = 0;
    }
    else; // other interrupt?? do nothing! how come there is other interrupt?
    return;
}

extern void TIM1_UP_IRQHandler() // runs every 0.083s. acceptable. update interrupt, when counter completed a cycle
{
    if (TIM_GetITStatus(TIM1, TIM_IT_Update) == SET)
    {
        cycles1 ++;
        if (cycles1 > 2)
            {
                vl2 = 0;
                if (cycles1 > 5) cycles1 = 5;
            }
        cycles4 ++;
        if (cycles4 > 2)
            {
                vr2 = 0;
                if (cycles4 > 5) cycles4 = 5;
            }
        TIM_ClearITPendingBit(TIM1, TIM_IT_Update);
    }
    return;
}

/* initialize TIM1 for input detection for encoder  */
void TIM1_Cap_Init() // capture mode, TIM1CH1 - PA8, TIM1CH4 - PA11
{      
    TIM_ICInitTypeDef  TIM1_ICInitStructure;
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
    NVIC_InitTypeDef NVIC_InitStructure;
    GPIO_InitTypeDef GPIO_InitStructure;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);  
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);

    /* Configure PA8, 11 */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_11;       
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD; // pull-down input
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    GPIO_ResetBits(GPIOA, GPIO_Pin_8 | GPIO_Pin_11);

    // Below: TIM1 init
    TIM_TimeBaseStructure.TIM_Period = time_interval_reload - 1;
    TIM_TimeBaseStructure.TIM_Prescaler = time_interval_prescaler - 1; // down to 720000 Hz
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; //tDTS = tCK_INT  
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseStructure.TIM_RepetitionCounter = 0; // only for TIM1 and 8
    TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);

    TIM_ICStructInit(&TIM1_ICInitStructure);
    TIM1_ICInitStructure.TIM_Channel = TIM_Channel_1; 
    TIM1_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising; 
    TIM1_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
    TIM1_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1; // don't prescale input signal, capture performs every event
    TIM1_ICInitStructure.TIM_ICFilter = 0; // filter: 0
    TIM_ICInit(TIM1, &TIM1_ICInitStructure);
    TIM1_ICInitStructure.TIM_Channel = TIM_Channel_4;
    TIM_ICInit(TIM1, &TIM1_ICInitStructure);
    
    // interuption config
    NVIC_InitStructure.NVIC_IRQChannel = TIM1_CC_IRQn;
        
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2; // lower than systick (0) cuz this vel updates much faster   
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;   
    NVIC_Init(&NVIC_InitStructure);

    NVIC_InitStructure.NVIC_IRQChannel = TIM1_UP_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_Init(&NVIC_InitStructure);

    TIM_ITConfig(TIM1, TIM_IT_CC1|TIM_IT_CC4|TIM_IT_Update, ENABLE); // channel 1, 4
    TIM_Cmd(TIM1, ENABLE);

    // below: PE5, 6 init to determine rotation direction
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE, ENABLE);
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_6; // for encoder Pin B input
    GPIO_Init(GPIOE, &GPIO_InitStructure);
}
