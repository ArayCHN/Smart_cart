#include "BodyControl.h"
extern u8 CCD[128];
STRU_BODYCONTROL_INFO BodyControlInfo;
STRU_BODYCONTROL_TARGET BodyControlTarget;

u8 pixel=0;
u8 LeftLine=40;
u8 LeftLineState=0;
u8 RightLine=88;
u8 RightLineState=0;
u8 MiddlePosition=64;
u8 TargetPosition=64;

const u8 CcdCompensationValue[128]={0};
u8 CcdFiltered[128];
int CcdDifferential[127];
int CcdDifferentialMax=0;
int CcdDifferentialMin=0;
int CcdDifferentialThrehold;

void BodyControl_init(void)
{
	BodyControlInfo.BodyAngularPIDKp=1;
	BodyControlInfo.BodyLinearPIDKp=1;
	BodyControlInfo.BodyLinearMAX=100;
}

void BodyControl(void)
{
	ValueCompensate();
	MedianFilter();
	SelectThrehold();
	SearchLine();
	GenerateTarget();
	MotorTargetVelocityPID();
}

void ValueCompensate(void)
{
	for(pixel=0;pixel<128;pixel++)
	{
		CCD[pixel]+=CcdCompensationValue[pixel];
	}
}

void MedianFilter(void)
{
	CcdFiltered[0]=CCD[0];
	for(pixel=1;pixel<127;pixel++)
	{
		CcdFiltered[pixel]=GetMid(CCD[pixel-1],CCD[pixel],CCD[pixel+1]);
	}
	CcdFiltered[127]=CCD[127];
}

u8 GetMid(u8 a, u8 b, u8 c)
{
	u8 m=0;
	if(a>b){m=b;b=a;a=m;}
	if(b>c){m=c;c=b;b=m;}
	if(a>b){m=b;b=a;a=m;}
	return b;
}

void SelectThrehold(void)
{
	for(pixel=0;pixel<127;pixel++)
	{
		CcdDifferential[pixel]=CcdFiltered[pixel+1]-CcdFiltered[pixel];
		if(CcdDifferentialMax<CcdDifferential[pixel])
		{
			CcdDifferentialMax=CcdDifferential[pixel];
		}
		if(CcdDifferentialMin>CcdDifferential[pixel])
		{  
			CcdDifferentialMin=CcdDifferential[pixel];
		}
	}
	CcdDifferentialThrehold=CcdDifferentialMax/2;
}

void SearchLine(void)
{
	u8 State=0;
	u8 RisingEdge;
	u8 FallingEdge;
	u8 breadth;
	int CcdDifferentialThreholdDown=-CcdDifferentialThrehold;	
	if(MiddlePosition<26) {MiddlePosition=26;}
	else if(MiddlePosition>101) {MiddlePosition=101;}
    // search for right line
	for(pixel=MiddlePosition;pixel<127;pixel++)
	{
		if(State==0&&CcdDifferential[pixel]<CcdDifferentialThreholdDown)
		{
			FallingEdge=pixel;
			State=1;
		}
		if(State==1&&CcdDifferential[pixel]>CcdDifferentialThrehold)
		{
			RisingEdge=pixel;
			State=2;
			breadth=RisingEdge-FallingEdge;		
			break;			
		}
	}
	// determine right line ligitimacy
	if(State==2&&breadth>3&&breadth<15)
	{
		RightLine=(RisingEdge+FallingEdge)/2;
		RightLineState=1;
		State=0;
	}
	else
	{
		RightLineState=0;
		State=0;
	}
    // search for left line
	for(pixel=MiddlePosition;pixel>0;pixel--)
	{
		if(State==0&&CcdDifferential[pixel]>CcdDifferentialThrehold)
		{
			RisingEdge=pixel;
			State=1;
		}
		if(State==1&&CcdDifferential[pixel]<CcdDifferentialThreholdDown)
		{
			FallingEdge=pixel;
			State=2;
			breadth=RisingEdge-FallingEdge;
			break;
		}
	}	
	// determine left line validity
	if(State==2&&breadth>3&&breadth<15)
	{
		LeftLine=(RisingEdge+FallingEdge)/2;
		LeftLineState=1;
		State=0;
	}
	else
	{
		LeftLineState=0;
		State=0;
	}

    // update current mid point	
	if(LeftLineState==1&&RightLineState==1)
	{
		MiddlePosition=(LeftLine+RightLine)/2;
	}
	if(LeftLineState==1&&RightLineState==0)
	{
		MiddlePosition=LeftLine+25;
	}
	else if(LeftLineState==0&&RightLineState==1)
	{
		MiddlePosition=RightLine-25;
	}
}


void GenerateTarget(void)
{
	if(LeftUltrasonicState==0&&RightUltrasonicState==0)
	{
		TargetPosition=(LeftLine+RightLine)/2;
	}
	if(LeftUltrasonicState==1)
	{
		TargetPosition=RightLine;
	}
	else if(RightUltrasonicState==1)
	{
		TargetPosition=LeftLine;
	}
}

void MotorTargetVelocityPID(void)
{
	int8_t Bias=TargetPosition-64;
	u8 BiasAbs;
	if (Bias>0)  {BiasAbs=Bias;}
	else         {BiasAbs=-Bias;}
	BodyControlTarget.BodyTargetAngularVelocity = BodyControlInfo.BodyAngularPIDKp * Bias; 
	BodyControlTarget.BodyTargetLinearVelocity=BodyControlInfo.BodyLinearMAX-BodyControlInfo.BodyLinearPIDKp*BiasAbs;
	BodyControlTarget.MotorLeftTargetVelocity=BodyControlTarget.BodyTargetAngularVelocity+BodyControlTarget.BodyTargetLinearVelocity;
	BodyControlTarget.MotorRightTargetVelocity=-BodyControlTarget.BodyTargetAngularVelocity + BodyControlTarget.BodyTargetLinearVelocity;
}
