#include "stm32f10x.h"
#include <stdio.h>
#include "macros.h"

#include "ccd.h"
#include "controller.h"
#include <stdlib.h> // abs()

int left_line_dist, right_line_dist, mid_position_dist; // in ccd

u16 prescaler = 72 - 1, motor_pwm_period = 999, pulse = 0; // 0~65535, for motor
int vl1_target, vl2_target, vr1_target, vr2_target; // target vel
#include "motor_pwm.h"

int vl1, vl2, vr1, vr2;

// definitions for encoder mode vel calc
#include "motor_vel_encoder_mode.h"
int last_cnt_l1 = 0, counts_l1 = 0, timer8cnt = 0;
int last_cnt_r1 = 0, counts_r1 = 0, timer4cnt = 0; // signed! cuz v can be negative

// definitions for time interval method
void TIM1_Cap_Init(void); // init for time interval method; need to change to TIM1! in motor_vel_time_interval.c
int cnt_ch1 = 0, prev_cnt_ch1 = -1, cnt_ch4 = 0, prev_cnt_ch4 = -1, 
	  delta_t1 = 0, delta_t4 = 0, cycles1 = 0, cycles4 = 0; // prev_cnt init -1 to prevent den==0
int num, den;
// cycles: the cycles of reloading in TIM. cuz there might have been multiple cycles between two interrupts!

u8 obstacle_mode_flag = 0, last_obstacle_mode_flag = 0; // for ultrasonic, record if it is now in obstacle mode!
int last_obstacle_time_stamp = -obstacle_time_threshold;
#include "ultrasonic.h"

// definitions for controller()!
int R_target; // R_target is given by the general controller, global var
void controller(u8, u8, u8);
void simple_controller_init();
void simple_controller();

// definitions for linear ccd
int delta_x;
void ccd_get_line();

// for zhukai's simple controller
STRU_BODYCONTROL_INFO BodyControlInfo; // used in simple_controller() in controller.c
STRU_BODYCONTROL_TARGET BodyControlTarget;

// below: systick, whose exception mechanism is shared among several devices
int omega, v_target;
static int systick_count, time_ccd_exposure, time_ultra;
extern void SysTick_Handler()
{
    systick_count ++; // every 1 ms
	  //printf("%d\n", systick_count);
    systick_count %= 3600000; // an hour, long enough!
    if (systick_count % ccd_exposure_period == 0)
        time_ccd_exposure = 1; // time for ccd exposure
    else
        time_ccd_exposure = 0;
    if (systick_count % encoder_period == 0)
		{
        encoder_vel_calc(); // update vl1, vl2
				motor_pid_controller(1, 1, 1); // kp, ki, kd
		}
    if (systick_count % ultra_period == 0)
        time_ultra = 1;
    else
        time_ultra = 0;
    if (systick_count % control_period == 0)
		{
			      vl1_target = 0;
					  vl2_target = 0;
					  vr1_target = 0;
					  vr2_target = 0;
            ccd_get_line();
            if (control_mode == 0) // wr control
                if (obstacle_mode_flag == NONE_OBSTACLE && systick_count - last_obstacle_time_stamp > obstacle_time_threshold)
                // no obstacle detected now by ultrasonic module and, last obstacle was at least 3 secs ago, now safe!
                {
                    printf("none!\n");
                    delta_x = mid_position_dist;
                    //controller(1, 1, 1);
                }
                else
                {
                    if (obstacle_mode_flag == RIGHT_OBSTACLE || (obstacle_mode_flag == NONE_OBSTACLE && last_obstacle_mode_flag == RIGHT_OBSTACLE)) // ob on right, go to left
										{
											  printf("right, %d, %d\n", obstacle_mode_flag, last_obstacle_time_stamp);
                        delta_x = left_line_dist;
											  if (obstacle_mode_flag == RIGHT_OBSTACLE)
												{
													last_obstacle_mode_flag = RIGHT_OBSTACLE;
													last_obstacle_time_stamp = systick_count;
												}
										}
                    else // ob on left, go to right
										{
											  printf("left, %d, %d\n", obstacle_mode_flag, last_obstacle_time_stamp);
                        delta_x = right_line_dist;
											  if (obstacle_mode_flag == LEFT_OBSTACLE)
												{
													last_obstacle_mode_flag = LEFT_OBSTACLE;
													last_obstacle_time_stamp = systick_count;
												}
										}
                    //controller(1, 1, 1);
                }
            else // control_mode == 1, zk control
                simple_controller();
		}
    if (systick_count % vel_control_period == 0)
		{
            //printf("vl1 %d vl2 %d vr1 %d  vr2 %d\n", vl1, vl2, vr1, vr2); // debug
            // now we have vl1, vr1, vl2, vr2 (the former two come from encoder, latter two come from time interval mode)
            /*if (abs(vl1 - vl2) > abs(vl1) / 3) // if two vel deviate too much, go with encoder mode
                vl2 = vl1;
            else
                vl1 = (vl1 + vl2) / 2;
            if (abs(vr1 - vr2) > abs(vr1) / 3) // if two vel deviate too much, go with encoder mode
                vr2 = vr1;
            else
                vr1 = (vr1 + vr2) / 2;
					*/
           motor_pid_controller(1, 1, 1); // kp, ki, kd
		}
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
    simple_controller_init();

    while (1)
    {
        if (time_ccd_exposure) 
				{
					time_ccd_exposure = 0;
					  Read_CCD(); // need to change the frequency this is carried out!
				}
        if (time_ultra)
				{
					time_ultra = 0;
					Ultrasonic_Trig();
				}
    }
    // return 0; - never carried out
}
