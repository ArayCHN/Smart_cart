#include "stm32f10x.h"
#include <stdio.h>
#include "macros.h"

void TIMx_PWMInit(uint16_t prescaler, uint16_t period, uint16_t pulse);

void EncodeInit(void);

static int last_cnt, counts, v, timer4cnt; // signed! cuz v can be negative
int get_encoder_counts();

void TIM8_Cap_Init(void); // init for time interval method; need to change to TIM1! in motor_vel_time_interval.c

// vars for time interval method
static int cnt_ch3, prev_cnt_ch3 = -1, cnt_ch4, prev_cnt_ch4 = -1, delta_t3, delta_t4, cycles3, cycles4; // prev_cnt init -1 to prevent den==0
u32 v3, v4, num, den;
// cycles: the cycles of reloading in TIM. cuz there might have been multiple cycles between two interrupts!

u8 obstacle_mode_flag = 0; // for ultrasonic, record if it is now in obstacle mode!
void TIM2_Init(void);
void TIM6_Init(void);
void Ultrasonic_Init(void);
void Ultrasonic_Trig(void);

// below: systick, whose exception mechanism is shared among several devices
static u8 ultra_cnt;
extern void SysTick_Handler()
{
    // v = get_encoder_counts() * wheel_perimeter * 1000 / (spokes_num * reduction_ratio * sysTick_period * 4); // update velocity
    // v in mm/s; all vals must be signed int32 so that the multiplication doesn't overflow & there are pos & neg
    int den = spokes_num * reduction_ratio * sysTick_period * 4; // divide encoder counter by 4
    int num = get_encoder_counts() * wheel_perimeter * 1000 * 10; // *10 cuz reduction ratio is 21.3 instead of 213
    v = num / den;

    ultra_cnt = 1 - ultra_cnt;
    if (ultra_cnt == 0) Ultrasonic_Trig(); // ultrasonic update frequency = 1/2 * encoder vel update freq
    // printf("%d \n", v); // debug
    // printf("v3:%d    v4: %d\n", v3, v4); // debug
    return;
}

void systickInit()
{
    SysTick_Config(SystemCoreClock / 1000 * sysTick_period); // SystemCoreClock == 72MHz
    NVIC_SetPriority(SysTick_IRQn, 0); // set priority to 0, should be highest in system
    return;
}

int main()
{
    u16 prescaler = 72 - 1, period = 49, pulse = 30; // 0~65535
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2); // priority group config, 2 bits preemption, 2 bits sub
    // priority group config must be before anything!
    TIMx_PWMInit(prescaler, period, pulse); // set timer for motor PWM
    EncodeInit(); // use encoder mode for motor
    systickInit(); // encoder mode interupt (in fact, exception)
    TIM8_Cap_Init(); // init: motor velocity calculation with time interval

    TIM2_Init();
    TIM6_Init();
    Ultrasonic_Init();

    while (1)
    {
        if (obstacle_mode_flag == NONE_OBSTACLE)
        {
            printf("none!\n");
        }
        else
        {
            if (obstacle_mode_flag == RIGHT_OBSTACLE)
                printf("right!\n");
            else
                printf("left!\n");
        }
    }
    return 0;
}
