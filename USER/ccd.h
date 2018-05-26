#ifndef __CCD_H
#define __CCD_H

void Adc_Init(void);
void Read_CCD(void);
u16  Get_Adc(u8 ch);

void ValueCompensate(void);
void MedianFilter(void);
u8 GetMid(u8 a, u8 b, u8 c);
void SelectThreshold(void);
void SearchLine(void);

#endif
