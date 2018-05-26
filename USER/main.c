#include "stm32f10x.h"
#include <stdio.h>
#include "macros.h"

#include "ccd.h"
#include "controller.h"
int left_line_dist, right_line_dist, mid_position_dist; // in ccd

u16 prescaler = 72 - 1, motor_pwm_period = 999, pulse = 300; // 0~65535, for motor
int vl1_target, vl2_target, vr1_target, vr2_target; // target vel
void TIMx_PWMInit(uint16_t prescaler, uint16_t pulse);
void motor_pid_controller(int, int, int);

int vl1, vl2, vr1, vr2;

// definitions for encoder mode vel calc
void EncodeInit(void);
int last_cnt_l1 = 0, counts_l1 = 0, timer8cnt = 0;
int last_cnt_r1 = 0, counts_r1 = 0, timer4cnt = 0; // signed! cuz v can be negative
int get_encoder_counts_l1(void);
int get_encoder_counts_r1(void);

// definitions for time interval method
void TIM1_Cap_Init(void); // init for time interval method; need to change to TIM1! in motor_vel_time_interval.c
int cnt_ch1 = 0, prev_cnt_ch1 = -1, cnt_ch4 = 0, prev_cnt_ch4 = -1, 
	  delta_t1 = 0, delta_t4 = 0, cycles1 = 0, cycles4 = 0; // prev_cnt init -1 to prevent den==0
int num, den;
// cycles: the cycles of reloading in TIM. cuz there might have been multiple cycles between two interrupts!

u8 obstacle_mode_flag = 0; // for ultrasonic, record if it is now in obstacle mode!
void TIM2_Init(void);
void TIM6_Init(void);
void Ultrasonic_Init(void);
void Ultrasonic_Trig(void);

// definitions for controller()!
int R_target; // R_target is given by the general controller, global var
void controller_doubleline(u8, u8, u8);
void controller_singleline(u8, u8, u8);

// definitions for linear ccd
int delta_x;

// below: systick, whose exception mechanism is shared among several devices
static u8 ultra_cnt; // only visible in main.c
int omega, v_target;
extern void SysTick_Handler()
{
    Read_CCD(); // need to change the frequency this is carried out!
    // v = get_encoder_counts() * wheel_perimeter * 1000 / (spokes_num * reduction_ratio * sysTick_period * 4); // update velocity
    // v in mm/s; all vals must be signed int32 so that the multiplication doesn't overflow & there are pos & ne
    omega = get_encoder_counts_l1(); // n circles' 50ms
    den = spokes_num * reduction_ratio * sysTick_period * 4; // divide encoder counter by 4
    num = omega * wheel_perimeter * 1000 * 10; // *10 cuz reduction ratio is 21.3 instead of 213
    vl1 = num / den;

    omega = get_encoder_counts_r1(); // n circles' 50ms
    den = spokes_num * reduction_ratio * sysTick_period * 4; // divide encoder counter by 4
    num = omega * wheel_perimeter * 1000 * 10; // *10 cuz reduction ratio is 21.3 instead of 213
    vr1 = num / den;

    ultra_cnt = 1 - ultra_cnt;
    if (ultra_cnt == 0) Ultrasonic_Trig(); // ultrasonic update frequency = 1/2 * encoder vel update freq
    // printf("%d \n", v); // debug
    // printf("v3:%d    v4: %d\n", v3, v4); // debug
    // if (obstacle_mode_flag == NONE_OBSTACLE) // debug - ultrasonic
    //     {
    //         printf("none!\n");
    //     }
    //     else
    //     {
    //         if (obstacle_mode_flag == RIGHT_OBSTACLE)
    //             printf("right!\n");
    //         else
    //            printf("left!\n");
    //     }
    motor_pid_controller(1, 1, 1); // kp, ki, kd
		printf("vl1 %d vl2 %d\n", vl1, vl2);
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
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2); // priority group config, 2 bits preemption, 2 bits sub
    // priority group config must be before anything!
    TIMx_PWMInit(prescaler, pulse); // set timer for motor PWM
    EncodeInit(); // use encoder mode for motor
    systickInit(); // encoder mode interupt (in fact, exception)
    TIM1_Cap_Init(); // init: motor velocity calculation with time interval

    TIM2_Init(); // for ultrasonic
    TIM6_Init(); // for ultrasonic
    Ultrasonic_Init(); // GPIO: PE

    Adc_Init(); // for ccd
    STRU_BODYCONTROL_INFO BodyControlInfo; // used in simple_controller() in controller.c
    simple_controller_init(BodyControlInfo);
    STRU_BODYCONTROL_TARGET BodyControlTarget;

    while (1)
    {
        ccd_get_line();
        if (control_mode == 0) // wr
        if (obstacle_mode_flag == NONE_OBSTACLE)
        {
            //printf("none!\n");
            delta_x = mid_position_dist;
            controller(1, 1, 1); // should be put on a time basis instead of always running!
        }
        else
        {
            //if (obstacle_mode_flag == RIGHT_OBSTACLE)
            //    printf("right!\n");
            //else
            //   printf("left!\n");
            // obtain single line position!
            if (obstacle_mode_flag == RIGHT_OBSTACLE) // ob on right, go to left
                delta_x = left_line_dist;
            else // ob on left, go to right
                delta_x = right_line_dist;
            controller(1, 1, 1);
        }
        else // control_mode == 1, zk
        simple_controller();
    }
    // return 0; - never carried out
}
