#ifndef __MACROS_H__
#define __MACROS_H__

// these params are already fixed!
#define spokes_num 11
#define reduction_ratio 213 // reduction ratio = 21.3
#define sysTick_period 50 // update v every 50ms
#define wheel_perimeter 210 // in mm

// parameter for vel calc with encoder mode
#define encoder_counter_reload 8000 // rotate each round, encoder around 200

// paramters for motor vel calc with time interval method
#define time_interval_reload 60000
#define time_interval_prescaler 100

// which side the obstacle is on
#define NONE_OBSTACLE 0
#define LEFT_OBSTACLE 1
#define RIGHT_OBSTACLE 2

#endif