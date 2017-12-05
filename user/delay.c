/**************************************************************
* Copyright (C) 2008-2017, Thunder Software Technology Co.,Ltd.
* All rights reserved.
****************************************************************/
#include "stdint.h"
#include "stm32l4xx.h"
#include "delay.h"

#define SYSTEM_SUPPORT_OS 1

uint16_t fac_us;
uint16_t fac_ms;

void delay_us(uint32_t nus)
{
	int i;
	for(i=0; i<10*nus; i++){}
}
