#ifndef __LED_H
#define __LED_H	 
#include "sys.h"

#define LED0 PBout(13)// PB13 D2
#define LED1 PBout(14)// PB14 D3

void LED_Init(void);	 				    
#endif
