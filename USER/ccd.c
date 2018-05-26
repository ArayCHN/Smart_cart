#include "stm32f10x.h"
#include "ccd.h"
#include "macros.h"
#include <stdio.h>
#include "delay.h"

// PA5/ADC12_IN5;  SI:PA7/TIM3_CH2;  CLK:PA6/TIM3_CH1 

// #define CCD_SI   PAout(7)   //SI
// #define CCD_CLK  PAout(6)   //CLK 
u8 CCD[128]={0};

u8 pixel=0;
int LeftLine=40;
u8 LeftLineState=0;
int RightLine=88;
u8 RightLineState=0;
int MiddlePosition=64;

extern int left_line_dist, right_line_dist, mid_position_dist;

const u8 CcdCompensationValue[128]={0};
u8 CcdFiltered[128];
int CcdDifferential[127];
int CcdDifferentialMax=0;
int CcdDifferentialMin=0;
int CcdDifferentialThreshold;

void  Adc_Init(void)
{ 	
	ADC_InitTypeDef ADC_InitStructure; 
	GPIO_InitTypeDef GPIO_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_ADC1, ENABLE ); // enable ADC clk
 	RCC_ADCCLKConfig(RCC_PCLK2_Div6); // ADC scaler 6, 72M/6=12, total value < 14M
	                      
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN; // PA5/ADC_IN5
	GPIO_Init(GPIOA, &GPIO_InitStructure);	

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure); // PA6, ccd clock
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
	GPIO_Init(GPIOA, &GPIO_InitStructure); // PA7, ccd SI

	ADC_DeInit(ADC1);

	ADC_InitStructure.ADC_Mode = ADC_Mode_Independent; // ADC1, ADC2 independent mode
	ADC_InitStructure.ADC_ScanConvMode = DISABLE;
	ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
	ADC_InitStructure.ADC_NbrOfChannel = 1;
	ADC_Init(ADC1, &ADC_InitStructure);

	ADC_Cmd(ADC1, ENABLE);
	ADC_ResetCalibration(ADC1);  
	while(ADC_GetResetCalibrationStatus(ADC1));
	ADC_StartCalibration(ADC1);
	while(ADC_GetCalibrationStatus(ADC1));

    return;
}

u16 Get_Adc(u8 ch) // ch: channel
{
	ADC_RegularChannelConfig(ADC1, ch, 1, ADC_SampleTime_239Cycles5 );
	ADC_SoftwareStartConvCmd(ADC1, ENABLE);
	while(!ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC ));
	return ADC_GetConversionValue(ADC1);
}


void Read_CCD(void) // evrey 10-15 ms
{
	u8 i=0,pixel=0;
	GPIO_ResetBits(GPIOA, GPIO_Pin_7); // CCD_SI=0;PA7
	delay_us(1);
	
	GPIO_SetBits(GPIOA, GPIO_Pin_7); // CCD_SI=1;
	delay_us(1);
	GPIO_ResetBits(GPIOA, GPIO_Pin_6); // CCD_CLK=0;6
	delay_us(1);

	GPIO_SetBits(GPIOA, GPIO_Pin_6); // CCD_CLK=1;
	GPIO_ResetBits(GPIOA, GPIO_Pin_7); // CCD_SI=0;
	delay_us(1);
	for(i=0;i<128;i++)
	{ 
		GPIO_ResetBits(GPIOA, GPIO_Pin_6); // CCD_CLK=0; 
		delay_us(1);
		CCD[pixel]=Get_Adc(5)>>4;
		++pixel;
		GPIO_SetBits(GPIOA, GPIO_Pin_6); // CCD_CLK=1;
		delay_us(1); 
    }
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

void SelectThreshold(void)
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
    CcdDifferentialThreshold=CcdDifferentialMax/2;
}

void SearchLine(void)
{
    u8 State=0;
    u8 RisingEdge;
    u8 FallingEdge;
    u8 breadth;
    int CcdDifferentialThresholdDown=-CcdDifferentialThreshold;   
    if(MiddlePosition<26) {MiddlePosition=26;}
    else if(MiddlePosition>101) {MiddlePosition=101;}
    // search for right line
    for(pixel=MiddlePosition;pixel<127;pixel++)
    {
        if(State==0&&CcdDifferential[pixel]<CcdDifferentialThresholdDown)
        {
            FallingEdge=pixel;
            State=1;
        }
        if(State==1&&CcdDifferential[pixel]>CcdDifferentialThreshold)
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
        if(State==0&&CcdDifferential[pixel]>CcdDifferentialThreshold)
        {
            RisingEdge=pixel;
            State=1;
        }
        if(State==1&&CcdDifferential[pixel]<CcdDifferentialThresholdDown)
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
    return;
}

void ccd_get_line(void)
{
    ValueCompensate();
    MedianFilter();
    SelectThreshold();
    SearchLine();
    // after this, middle line / left line / right line will be obtained
    right_line_dist = (RightLine - 63) / 128 * ccd_width; // int val, can be positive or negative!
    left_line_dist = (LeftLine - 63) / 128 * ccd_width;
    mid_position_dist = (MiddlePosition - 63) / 128 * ccd_width; // these are delta_x
    return;
}