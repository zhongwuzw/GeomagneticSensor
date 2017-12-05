/**************************************************************
* Copyright (C) 2008-2017, Thunder Software Technology Co.,Ltd.
* All rights reserved.
****************************************************************/
#ifndef __UART_TEST__
#define __UART_TEST__

#ifdef __cplusplus
 extern "C" {
#endif 

#include "los_sys.h"
#include "los_tick.h"
#include "los_task.ph"
#include "los_config.h"

#include "los_bsp_led.h"
#include "los_bsp_key.h"
#include "los_bsp_uart.h"
#include "los_inspect_entry.h"
#include "los_demo_entry.h"
#include "cmsis_os.h"

#include <string.h>
#include "stm32l4xx_hal.h"

#include "los_dev_st_uart.h"
#include "los_dev_st_spi.h"
#include "udp_coap_interface.h"

void FT_UART_Send();
void NB_TEST_Uart();
void NB_TEST_Uart_once(int status);
int NB_Init();
void NB_TEST();

#ifdef __cplusplus
 extern "C" {
#endif 

#endif
