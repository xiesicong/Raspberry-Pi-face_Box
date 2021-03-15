#ifndef __SYSTICK_H
#define __SYSTICK_H


#include "stm32f10x.h"


void                    SysTick_Init                            ( void );
void                    TimingDelay_Decrement                   ( void );
void                    Delay_us                                ( __IO u32 nTime );
void                    Delay_ms                                ( unsigned int ms );
void                    Delay_s                                ( unsigned int ms);


#endif /* __SYSTICK_H */
