#include "stm32f10x.h"
#include <stdio.h>
#include "macros.h"
#include "delay.h"

u8 tim2_reload_flag = 0, tim6_reload_flag = 0, ultra_record_flag_l = 0, ultra_record_flag_r = 0, bounce_flag = 0;
extern u8 obstacle_mode_flag;

void TIM2_Init(void)
{  
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    NVIC_InitTypeDef NVIC_InitStructure;
    /* TIM2 clock enable */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);  
    /* Time base configuration */
    TIM_DeInit(TIM2);
    TIM_TimeBaseStructure.TIM_Period = 3000; // cnt > 3000 means > 50cm
    TIM_TimeBaseStructure.TIM_Prescaler = 71; // 1000000 cnts/s
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;  
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;

    TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);
    TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE); // interrupts when cnt > 3000
    TIM_Cmd(TIM2, DISABLE); // closed for now

    NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;   
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
    TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
}

void TIM6_Init(void)
{  
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
    /* TIM6 clock enable */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6, ENABLE);  
    /* Time base configuration */
    TIM_DeInit(TIM6);
    TIM_TimeBaseStructure.TIM_Period = 3000; // cnt > 3000 means > 50cm
    TIM_TimeBaseStructure.TIM_Prescaler = 71; // 1000000 cnts/s
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;  
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  

    TIM_TimeBaseInit(TIM6, &TIM_TimeBaseStructure);
    TIM_ITConfig(TIM6, TIM_IT_Update, ENABLE); // interrupts when cnt > 3000
    TIM_Cmd(TIM6, DISABLE); // closed for now
	
	  NVIC_InitStructure.NVIC_IRQChannel = TIM6_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;   
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
    TIM_ClearITPendingBit(TIM6, TIM_IT_Update);
}

void Ultrasonic_Init(void)
{
    NVIC_InitTypeDef NVIC_InitStructure;
    EXTI_InitTypeDef EXTI_InitStructure;
    GPIO_InitTypeDef GPIO_InitStructure;
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE | RCC_APB2Periph_AFIO, ENABLE);  
    
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;         
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;  
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_2; // trig - PE0, 2
    GPIO_Init(GPIOE, &GPIO_InitStructure);
    
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;  
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1 | GPIO_Pin_3; // echo - PE1, 3
    GPIO_Init(GPIOE, &GPIO_InitStructure);
    GPIO_WriteBit(GPIOE, GPIO_Pin_0, Bit_RESET); // clear trig signal first, can't use |
    GPIO_WriteBit(GPIOE, GPIO_Pin_2, Bit_RESET);
    
    // interrupt config
    EXTI_ClearITPendingBit(EXTI_Line1 | EXTI_Line3);
    GPIO_EXTILineConfig(GPIO_PortSourceGPIOE, GPIO_PinSource1);
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOE, GPIO_PinSource3);
    EXTI_InitStructure.EXTI_Line = EXTI_Line1 | EXTI_Line3;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;  
    EXTI_Init(&EXTI_InitStructure);
    
    NVIC_InitStructure.NVIC_IRQChannel = EXTI1_IRQn;  
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3; // lower, cuz obstacle is relatively far
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;   
    NVIC_Init(&NVIC_InitStructure);
    NVIC_InitStructure.NVIC_IRQChannel = EXTI3_IRQn;
    NVIC_Init(&NVIC_InitStructure);
}

void Ultrasonic_Trig(void)
{
    if (!bounce_flag) obstacle_mode_flag = NONE_OBSTACLE;
    bounce_flag = 0; // record if ultrasonic wave bounced back in the last emission within time threshold
    GPIO_WriteBit(GPIOE, GPIO_Pin_0, Bit_SET);
    GPIO_WriteBit(GPIOE, GPIO_Pin_2, Bit_SET);
    delay_us(15); // HOW to delay?
    GPIO_WriteBit(GPIOE, GPIO_Pin_0, Bit_RESET);
    GPIO_WriteBit(GPIOE, GPIO_Pin_2, Bit_RESET);
    return;
}

void EXTI1_IRQHandler(void)
{
    if (EXTI_GetITStatus(EXTI_Line1) != RESET)
    {
		EXTI_ClearITPendingBit(EXTI_Line1);
        if (GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_1) == 1) // rising edge  
        {
            TIM_SetCounter(TIM2, 0);
            tim2_reload_flag = 0;
            TIM_Cmd(TIM2, ENABLE);
            ultra_record_flag_l = 1; // mark: l start to record
        }
        else // falling edge
        {
            TIM_Cmd(TIM2, DISABLE);
            if (ultra_record_flag_l) // make sure this falling edge is not a noise
            {
                if (!tim2_reload_flag) // distance not exceed threshold
                {
                    obstacle_mode_flag = LEFT_OBSTACLE;
                    bounce_flag = 1;
                }
                ultra_record_flag_l = 0;
                // printf(" %d ", TIM2->CNT);
            }
        }
    }
}

void EXTI3_IRQHandler(void)
{
    if (EXTI_GetITStatus(EXTI_Line3) != RESET)
    {
        EXTI_ClearITPendingBit(EXTI_Line3);
        if (GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_3) == 1) // rising edge  
        {
             TIM_SetCounter(TIM6, 0);
             tim6_reload_flag = 0;
             TIM_Cmd(TIM6, ENABLE);
             ultra_record_flag_r = 1; // mark: r start to record
        }
        else // falling edge
        {
            TIM_Cmd(TIM6, DISABLE);
            if (ultra_record_flag_r) // make sure this falling edge is not a noise
            {
                if (!tim6_reload_flag) // distance not exceed threshold
                {
                    bounce_flag = 1;
                    obstacle_mode_flag = RIGHT_OBSTACLE;
                }
                ultra_record_flag_r = 0;
								printf(" %d ", TIM6->CNT);
            }
        }
    }
}

void TIM2_IRQHandler(void)
{
    tim2_reload_flag = 1;
    TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
    return;
}

void TIM6_IRQHandler(void)
{
    tim6_reload_flag = 1;
	  TIM_ClearITPendingBit(TIM6, TIM_IT_Update);
    return;
}