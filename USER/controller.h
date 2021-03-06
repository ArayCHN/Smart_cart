// for simple_controller
typedef struct
{
    int BodyAngularPIDKp;
    int BodyLinearPIDKp;
    int BodyLinearMAX;
}STRU_BODYCONTROL_INFO;

typedef struct
{
    int BodyTargetAngularVelocity;
    int BodyTargetLinearVelocity;
    int MotorLeftTargetVelocity;
    int MotorRightTargetVelocity;
}STRU_BODYCONTROL_TARGET;

extern int Bias;
extern int BiasAbs;
extern int BiasLast;
extern int BiasAddition;