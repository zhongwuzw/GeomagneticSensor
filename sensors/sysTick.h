#ifndef __SYSTICK_H__
#define __SYSTICK_H__

//uint16_t fac_us;
//uint16_t fac_ms;
extern float ElaspTime;
void toc(void);
void tic(void);
void ots_SysTick_Handler(void);

#endif