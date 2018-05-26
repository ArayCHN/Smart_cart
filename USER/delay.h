#ifndef __DELAY_H__
#define __DELAY_H__ 
#include <stdio.h>

void delay_us(u16 delay_time)
{   
    u16 i, j;  
    for (i = 0; i < delay_time; i++)  
        for (j = 0; j < 9; j++);
    return;
}

#endif