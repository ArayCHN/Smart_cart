#include "stm32f10x.h"
#include <stdio.h>
#include "macros.h"
#include "stdlib.h" // abs() for integer. write my own abs() eventually!!! to save time!!

static int err, err1, err2, R_inverse, delta_R_inverse, v_CoM, R, R_abs;
extern float beta;
extern int ccd_line_length, delta_x; // obtain from zhukai!
extern int vl1_target, vl2_target, vr1_target, vr2_target;

void controller_doubleline(u8 kp, u8 ki, u8 kd)
{
    // beta = acos(lane_width / ccd_line_length);
    // control: R_inverse = 3000/R, where R is the turing radius in mm
    // the larger the deviation, the larger 3000/R is, the smaller the turning radius
    // R: by default (calculation), turns right. negative R means turning left.
    err = delta_x; // returned from zhukai, should be double line error now!
    // delta_x req: if the line midpoint is leaning to right, delta_x > 0
    delta_R_inverse = kp * (err - err1) + ki * err + kd * (err - 2 * err1 + err2);
    err2 = err1;
    err1 = err;
    R_inverse += delta_R_inverse;
    R = 3000 / R_inverse;
    R_abs = abs(R);
    if (R_abs >= 1000)
        v_CoM = 300; // need to adjust!! define in macro, max_vel, mid_vel, min_vel
    else
        if (R_abs >= 50)
            v_CoM = 100;
        else
            v_CoM = 50;
    // solve for vr1, vr2, vl1, vl2 based on v_CoM and R
    // R = cart_width * (vr1 + vr2 + vl1 + vl2) / (vr1 - vl1 + vr2 - vl2) / 2; // is precision good? MAY REQUIRE CHANGE!
    // R > 0: turn right; R < 0: turn left
    vl1_target = v_CoM * (1 + cart_width / R / 2);
    vl2_target = v_CoM * (1 + cart_width / R / 2);
    vr1_target = v_CoM * (1 - cart_width / R / 2);
    vr2_target = v_CoM * (1 - cart_width / R / 2);
    return;
}

void controller_singleline(u8 kp, u8 ki, u8 kd)
{
    // beta = acos(lane_width / ccd_line_length);
    // control: R_inverse = 3000/R, where R is the turing radius in mm
    // the larger the deviation, the larger 3000/R is, the smaller the turning radius
    // R: by default (calculation), turns right. negative R means turning left.
    err = delta_x; // obtained from zhukai, should be single line error now!
    // delta_x req: if the line midpoint is leaning to right, delta_x > 0
    delta_R_inverse = kp * (err - err1) + ki * err + kd * (err - 2 * err1 + err2);
    err2 = err1;
    err1 = err;
    R_inverse += delta_R_inverse;
    R = 3000 / R_inverse;
    R_abs = abs(R);
    if (R_abs >= 1000)
        v_CoM = 300; // need to adjust!! define in macro, max_vel, mid_vel, min_vel
    else
        if (R_abs >= 50)
            v_CoM = 100;
        else
            v_CoM = 50;
    // solve for vr1, vr2, vl1, vl2 based on v_CoM and R
    // R = cart_width * (vr1 + vr2 + vl1 + vl2) / (vr1 - vl1 + vr2 - vl2) / 2; // is precision good? MAY REQUIRE CHANGE!
    // R > 0: turn right; R < 0: turn left
    vr1_target = v_CoM * (1 - cart_width / R / 2);
    vr2_target = v_CoM * (1 - cart_width / R / 2);
    vl1_target = v_CoM * (1 + cart_width / R / 2);
    vl2_target = v_CoM * (1 + cart_width / R / 2);
    return;
}
