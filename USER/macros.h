#ifndef __MACROS_H__
#define __MACROS_H__

// these params are already fixed!
#define spokes_num 11
#define reduction_ratio 213 // reduction ratio = 21.3
#define wheel_perimeter 210 // in mm

// parameter for vel calc with encoder mode
#define encoder_counter_reload 8000 // rotate each round, encoder around 200

// paramters for motor vel calc with time interval method
#define time_interval_reload 60000
#define time_interval_prescaler 100

#define obstacle_time_threshold 3000 // 3s
// which side the obstacle is on
#define NONE_OBSTACLE 0
#define LEFT_OBSTACLE 1
#define RIGHT_OBSTACLE 2

// parameters for overall controller
#define cart_width 200 // need more precision!
#define ccd_distance 300 // sees the line 30 cm ahead, needs modification!
#define lane_width 200 // needs modification!

#define ccd_width 600 // 600 mm width for each ccd shot

#define control_mode 0 // wr control
#define max_vel 500
#define mid_vel 200
#define min_vel 100

// paramters for the frequencies at which each component is triggered
#define sysTick_period 1 // enter sysTick interrupt every 1 ms
#define ultra_period 107 // trigger ultrasonic module every 107ms 
#define encoder_period 47 // update every 50 ms
#define control_period 103
#define ccd_exposure_period 10
#define vel_control_period 53 // 14 normally, 50 for debug

#endif
