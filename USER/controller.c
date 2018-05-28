#include "stm32f10x.h"
#include <stdio.h>
#include "macros.h"
#include "stdlib.h" // abs() for integer. write my own abs() eventually!!! to save time!!
#include "controller.h"

static int err, err1, err2, R_inverse, delta_R_inverse, v_CoM, R, R_abs;
extern float beta;
extern int ccd_line_length, delta_x;
extern int vl1_target, vl2_target, vr1_target, vr2_target;
extern int obstacle_mode_flag;

extern STRU_BODYCONTROL_INFO BodyControlInfo; // defined in main.c, used in simple_controller() in controller.c
extern STRU_BODYCONTROL_TARGET BodyControlTarget;

void controller(u8 kp, u8 ki, u8 kd) // these params require tuning!
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
    vl2_target = vl1_target; // v_CoM * (1 + cart_width / R / 2);
    vr1_target = v_CoM * (1 - cart_width / R / 2);
    vr2_target = vr1_target; // v_CoM * (1 - cart_width / R / 2);
    return;
}

void simple_controller_init(void) // these val require tuning!
{
    BodyControlInfo.BodyAngularPIDKp = 1;
    BodyControlInfo.BodyLinearPIDKp = 1;
    BodyControlInfo.BodyLinearMAX = 30;
    return;
}

void simple_controller(void) // from zhukai's code
{
	  int TargetPosition;
	  int8_t Bias; u8 BiasAbs;
	  extern int MiddlePosition, RightLine, LeftLine; // defined in ccd.c
    if (obstacle_mode_flag == NONE_OBSTACLE)
        TargetPosition = MiddlePosition;
    else if (obstacle_mode_flag == LEFT_OBSTACLE)
        TargetPosition = RightLine;
    else
        TargetPosition = LeftLine;
    Bias=TargetPosition-64;
    if (Bias>0)  {BiasAbs = Bias;}
    else         {BiasAbs = -Bias;}
    BodyControlTarget.BodyTargetAngularVelocity = BodyControlInfo.BodyAngularPIDKp * Bias; 
    BodyControlTarget.BodyTargetLinearVelocity = BodyControlInfo.BodyLinearMAX - BodyControlInfo.BodyLinearPIDKp * BiasAbs;
    BodyControlTarget.MotorLeftTargetVelocity = BodyControlTarget.BodyTargetAngularVelocity + BodyControlTarget.BodyTargetLinearVelocity;
    BodyControlTarget.MotorRightTargetVelocity = -BodyControlTarget.BodyTargetAngularVelocity + BodyControlTarget.BodyTargetLinearVelocity;

    vl1_target = BodyControlTarget.MotorLeftTargetVelocity;
    vl2_target = vl1_target;
    vr1_target = BodyControlTarget.MotorRightTargetVelocity;
    vr2_target = vr1_target;
    return;
}
