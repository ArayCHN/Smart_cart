#ifndef __BODYCONTROL_H
#define __BODYCONTROL_H	
#include "sys.h"

void ValueCompensate(void);
void MedianFilter(void);
u8 GetMid(u8 a, u8 b, u8 c);
void SelectThrehold(void);
void SearchLine(void);
void BodyControl(void);
void GenerateTarget(void);
void MotorTargetVelocityPID(void);
void BodyControl_init(void);

typedef struct
{
	int BodyAngularPIDKp;
//	int BodyAngularPIDKi;
//	int BodyAngularPIDKd;
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


#endif 
