#include "stm32l4xx_hal.h"   // Device header
float ElaspTime;
uint16_t OverFlowTimes=0;
void tic(void)                            //记录开始时间
{
SysTick->CTRL |= (1<<2);   //????,HCLK
SysTick->CTRL |= (1<<1);   //????
SysTick->VAL=0X00;            //?????????,????????
SysTick->LOAD=0XFFFFFF;     //??????
SysTick->CTRL |= (1<<0);     //?????
}

void toc(void)                            //记录结束时间
{
uint32_t ClkNum;
SysTick->CTRL &= ~(1<<0); //?????
ClkNum=SysTick->VAL; //???????
ElaspTime=(OverFlowTimes*((float)0xffffff/SystemCoreClock)+(float)(0xffffff-ClkNum)/SystemCoreClock); //????
OverFlowTimes=0;
//printf("\r\nEscaple time is %f\r\n",ElaspTime);
}
/*?????*/
void ots_SysTick_Handler(void)
{
OverFlowTimes++;
}