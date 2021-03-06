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

#define obstacle_time_threshold 1000 // 3s
// which ultrasonic module is emitting
#define LEFT_ULTRA 0
#define RIGHT_ULTRA 1
// which side the obstacle is on
#define NONE_OBSTACLE 0
#define LEFT_OBSTACLE 1
#define RIGHT_OBSTACLE 2

// parameters for overall controller
#define cart_width 180 // need more precision!
#define ccd_distance 300 // sees the line 30 cm ahead, needs modification!
#define lane_width 200 // needs modification!

#define ccd_width 600 // 600 mm width for each ccd shot
#define single_line_displacement 10 // when tracking single line, go further 10 pixels

#define control_mode 0 // 0-wr control, 1-zk control
#define max_vel 800
#define mid_vel 500
#define min_vel 300

#define vconst 800
#define vturn 150

// paramters for the frequencies at which each component is triggered
#define sysTick_period 1 // enter sysTick interrupt every 1 ms
#define ultra_period 67 // trigger ultrasonic module every 107ms 
#define encoder_period 27 // update every 50 ms
#define control_period 69
#define ccd_exposure_period 8
#define vel_control_period 27 // 14 normally, 50 for debug

#endif
