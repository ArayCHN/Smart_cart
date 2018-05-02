#include "stm32f10x.h"
#include <stdio.h>

void TIMx_PWMInit(uint16_t prescaler, uint16_t period, uint16_t pulse)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
    TIM_OCInitTypeDef TimOCInitStructure;
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
    TIM_TimeBaseInitStructure.TIM_Period = period;
    TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM5, &TIM_TimeBaseInitStructure);
    /* TIMx clock settings above */
    /* PWM config below */
    TimOCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
    TimOCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
    TimOCInitStructure.TIM_Pulse = pulse; // pulse evaluate
    TimOCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OC1Init(TIM1, &TimOCInitStructure);
    TIM_OC2Init(TIM1, &TimOCInitStructure);
    TIM_OC3Init(TIM1, &TimOCInitStructure);
    TIM_OC4Init(TIM1, &TimOCInitStructure); // channel 1-4 config
    TIM_OC1PreloadConfig(TIM5, TIM_OCPreload_Enable);
    TIM_OC2PreloadConfig(TIM5, TIM_OCPreload_Enable);
    TIM_OC3PreloadConfig(TIM5, TIM_OCPreload_Enable);
    TIM_OC4PreloadConfig(TIM5, TIM_OCPreload_Enable); // channel 1-4 config
    TIM_ARRPreloadConfig(TIM5, ENABLE);
    TIM_Cmd(TIM5, ENABLE);
    TIM_CtrlPWMOutputs(TIM5, ENABLE);
    /* PWM config above */
    return;
}

#define encoder_counter_reload 8000 // rotate each round, encoder around 200
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

    //TIM_EncoderInterfaceConfig(TIM4, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising,
     //                          TIM_ICPolarity_Rising); // double rise or 4 times??
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

static int last_cnt, counts, v, timer4cnt; // signed! cuz v can be negative
int get_encoder_counts()
{
    timer4cnt = TIM4 -> CNT; // for debug
    counts = timer4cnt - last_cnt;
    if (counts < - encoder_counter_reload / 2) // counter overflowed
        counts += encoder_counter_reload;
		else
			if (counts > encoder_counter_reload / 2) // counter overflowed
        counts -= encoder_counter_reload;
		// printf("%d %d %d \n", last_cnt, timer4cnt, counts);
    last_cnt = TIM4 -> CNT;
    return counts;
}

// THESE PARSMS NEED TO BE CUSTOMIZED!
#define spokes_num 11
#define reduction_ratio 213 // reduction ratio = 21.3
#define sysTick_period 50 // update v every 50ms
#define wheel_perimeter 210 // in mm

extern void SysTick_Handler() // does "void" have to be written within ()?
{
	// NOTE: add 4 for every pulse in encoder! divide by 4 in the end!
    // v = get_encoder_counts() * wheel_perimeter * 1000 / (spokes_num * reduction_ratio * sysTick_period * 4); // update velocity
	// v in mm/s; all vals must be signed int32 so that the multiplication doesn't overflow
	int den = spokes_num * reduction_ratio * sysTick_period * 4; // divide encoder counter by 4
	int num = get_encoder_counts() * wheel_perimeter * 1000 * 10; // *10 cuz reduction ratio is 21.3 instead of 213
	v = num / den;
	printf("%d \n", v);
    return;
}

void systickInit()
{
    SysTick_Config(SystemCoreClock / 1000 * sysTick_period); // SystemCoreClock == 72MHz
    NVIC_SetPriority(SysTick_IRQn, 0); // set priority to 0, should be highest in system
    return;
}

/* initialize TIM8 for input detection for encoder  */
void TIM8_Cap_Init() // capture mode
{      
    TIM_ICInitTypeDef  TIM8_ICInitStructure;
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;

    RCC_APB1PeriphClockCmd(RCC_APB2Periph_TIM8, ENABLE);  
    // Below: TIM8 init, must be the same as TIM8 init in encoder mode!
    TIM_TimeBaseStructure.TIM_Period = encoder_counter_reload - 1;
    TIM_TimeBaseStructure.TIM_Prescaler = 0;
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; //tDTS = tCK_INT  
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseStructure.TIM_RepetitionCounter = 0; // only for TIM1 and 8
    TIM_TimeBaseInit(TIM8, &TIM_TimeBaseStructure);

    TIM_ICStructInit(&TIM8_ICInitStructure);
    TIM8_ICInitStructure.TIM_Channel = TIM_Channel_3; 
    TIM8_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising; 
    TIM8_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
    TIM8_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV8; // prescale input signal, capture performs every 8 events
    TIM8_ICInitStructure.TIM_ICFilter = 0x03; // filter: < 72/4 Hz, T > 111 ns
    TIM_ICInit(TIM8, &TIM8_ICInitStructure);
      
    // interuption config
    NVIC_InitTypeDef NVIC_InitStructure;
    NVIC_InitStructure.NVIC_IRQChannel = TIM8_CC_IRQn;    
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1; // lower than systick (0) cuz this vel updates much faster   
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;   
    NVIC_Init(&NVIC_InitStructure);
    
    TIM_ITConfig(TIM8, TIM_IT_CC3|TIM_IT_CC4, ENABLE); // channel 3, 4
    TIM_Cmd(TIM8, ENABLE);
}

static int cnt_ch3, prev_cnt_ch3 = -1, cnt_ch4 prev_cnt_ch4 = -1; // prev_cnt init -1 to prevent den==0

extern void TIM8_CC_IRQHandler()
{
    if (TIM_GetITStatus(TIM8, TIM_IT_CC3) == SET) // if it is channel 3
    {
        TIM_ClearITPendingBit(TIM1, TIM_IT_CC3);
        cnt_ch3 = TIM_GetCapture3(TIM8);
        int delta_t3 = cnt_ch3 - prev_cnt_ch3;
        prev_cnt_ch3 = cnt_ch3;
        if (delta_t3 < - encoder_counter_reload / 2) // counter overflowed
            delta_t3 += encoder_counter_reload;
        else
        if (delta_t3 > encoder_counter_reload / 2) // counter overflowed
            delta_t3 -= encoder_counter_reload;

        int num = wheel_perimeter * 72000000; // almost overflows int32, can't multiply anything more
        int den = spokes_num * (reduction_ratio / 10) * (delta_t3 / 8);
        // Note! have to sacrifice 21.3 -> 21 becuz wheel_perimeter*72000000 * 10 will overflow int32!
        v3 = num / den;
        printf("v3:%d \n", v3);
    }
    else if TIM_GetITStatus(TIM8, TIM_IT_CC4) == SET
    {
        TIM_ClearITPendingBit(TIM1, TIM_IT_CC4);
        cnt_ch4 = TIM_GetCapture4(TIM8);
        int delta_t4 = cnt_ch4 - prev_cnt_ch4;
        prev_cnt_ch4 = cnt_ch4;
        if (delta_t4 < - encoder_counter_reload / 2) // counter overflowed
            delta_t4 += encoder_counter_reload;
        else
        if (delta_t4 > encoder_counter_reload / 2) // counter overflowed
            delta_t4 -= encoder_counter_reload;

        int num = wheel_perimeter * 72000000; // change 72000000, don't directly use a number!
        int den = spokes_num * (reduction_ratio/10) * (delta_t4 / 5);
        // Note! have to sacrifice 21.3 -> 21 becuz wheel_perimeter*72000000 * 10 will overflow int32!
        v4 = num / den;
    }
    else; // other interrupt?? do nothing! how come there is other interrupt?
} 

int main()
{
    uint16_t prescaler = 72 - 1; // 0~65535
    uint16_t period = 49;
    uint16_t pulse = 30;
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2); // priority group config, 2 bits preemption, 2 bits sub
    // priority group config must be before anything!
    TIMx_PWMInit(prescaler, period, pulse); // set timer for motor PWM
	EncodeInit(); // use encoder mode for motor
    systickInit(); // encoder mode interupt (in fact, exception)
    TIM8_Cap_Init(9999, 0);
    while (1);
	return 0;
}
