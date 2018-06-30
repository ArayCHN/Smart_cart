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

int Bias;
int BiasAbs;
int BiasLast=0;
int BiasAddition;

void controller(u8 kp, u8 ki, u8 kd) // these params require tuning!
{
    // beta = acos(lane_width / ccd_line_length);
    // control: R_inverse = 3000/R, where R is the turing radius in mm
    // the larger the deviation, the larger 3000/R is, the smaller the turning radius
    // R: by default (calculation), turns right. negative R means turning left.
    err = delta_x; // returned from zhukai, should be double line error now!
	  // printf("err:%d\n", delta_x);
    // delta_x req: if the line midpoint is leaning to right, delta_x > 0
    delta_R_inverse = kp * (err - err1) + ki * err + kd * (err - 2 * err1 + err2);
	  delta_R_inverse /= 10;
    err2 = err1;
    err1 = err;
    R_inverse += delta_R_inverse;
    R = 40000 / R_inverse;
    R_abs = abs(R);
    if (R_abs >= 1000)
        v_CoM = max_vel; // need to adjust!! define in macro, max_vel, mid_vel, min_vel
    else
        if (R_abs >= 50)
            v_CoM = mid_vel;
        else
            v_CoM = min_vel;
    // solve for vr1, vr2, vl1, vl2 based on v_CoM and R
    // R = cart_width * (vr1 + vr2 + vl1 + vl2) / (vr1 - vl1 + vr2 - vl2) / 2; // is precision good? MAY REQUIRE CHANGE!
    // R > 0: turn right; R < 0: turn left
    vl1_target = v_CoM + v_CoM * cart_width / R / 2;
    vl2_target = vl1_target; // v_CoM * (1 + cart_width / R / 2);
    vr1_target = v_CoM - v_CoM * cart_width / R / 2;
    vr2_target = vr1_target; // v_CoM * (1 - cart_width / R / 2);
    return;
}

void simple_controller_init(void) // these val require tuning!
{
    BodyControlInfo.BodyAngularPIDKp = 20;  //30,25,25,25,15
    BodyControlInfo.BodyAngularPIDKd = 45;  //00,30,30,35,45
    BodyControlInfo.BodyLinearPIDKp = 10;   //20,25,15,15,15
    BodyControlInfo.BodyLinearPIDKd = 15;   //10,10,20,20,20
    
    BodyControlInfo.BodyLinearMAX = 800;    //600,650,650,800,800
    return;
}

void simple_controller(void) // from zhukai's code
{
    int TargetPosition;
    extern int MiddlePosition, RightLine, LeftLine; // defined in ccd.c
    if (obstacle_mode_flag == NONE_OBSTACLE)
        TargetPosition = MiddlePosition;
    else if (obstacle_mode_flag == LEFT_OBSTACLE)
        TargetPosition = RightLine+10;
    else
        TargetPosition = LeftLine-10;
    Bias=TargetPosition-64;
    
    BiasAddition=Bias-BiasLast;          //
    
    if (Bias>0)  {BiasAbs = Bias;}
    else         {BiasAbs = -Bias;}
    BodyControlTarget.BodyTargetAngularVelocity = BodyControlInfo.BodyAngularPIDKp * Bias+BodyControlInfo.BodyAngularPIDKd*BiasAddition; 
    BodyControlTarget.BodyTargetLinearVelocity = BodyControlInfo.BodyLinearMAX - BodyControlInfo.BodyLinearPIDKp * BiasAbs-BodyControlInfo.BodyLinearPIDKd*BiasAddition;
    BodyControlTarget.MotorLeftTargetVelocity = BodyControlTarget.BodyTargetAngularVelocity + BodyControlTarget.BodyTargetLinearVelocity;
    BodyControlTarget.MotorRightTargetVelocity = -BodyControlTarget.BodyTargetAngularVelocity + BodyControlTarget.BodyTargetLinearVelocity;

    BiasLast=Bias;           //
    
    vl1_target = BodyControlTarget.MotorLeftTargetVelocity;
    vl2_target = vl1_target;
    vr1_target = BodyControlTarget.MotorRightTargetVelocity;
    vr2_target = vr1_target;
    return;
}

