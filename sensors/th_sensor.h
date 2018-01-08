/**************************************************************
* Copyright (C) 2008-2017, Thunder Software Technology Co.,Ltd.
* All rights reserved.
****************************************************************/
#ifndef __TH_SENSOR_H__
#define __TH_SENSOR_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "los_sys.h"
#include "los_tick.h"
#include "los_task.ph"
#include "los_config.h"

#include "los_inspect_entry.h"
#include "los_demo_entry.h"
#include "cmsis_os.h"

#include <string.h>
#include <stdint.h>
#include <stdlib.h>

#include "stm32l4xx_hal.h"

#define  SHT20_WRITE_ADDRESS		0x60
#define	 SHT20_READ_ADDRESS			0x61
extern uint16_t  POLYNOMIAL ;

typedef enum
{
    SHT20_REG_CRA   = 0x00 ,
    SHT20_REG_CRB    = 0x01 ,
    SHT20_REG_MODE   = 0x02
} SHT20_Command;

typedef enum
{
    SHT2x_RES_12_14BIT = 0x00,
    SHT2x_RES_8_12BIT  = 0x01,
    SHT2x_RES_10_13BIT = 0x80,
    SHT2x_RES_11_11BIT = 0x81,
    SHT2x_RES_MASK     = 0x81
} SHT20_Resolution;

typedef enum
{
    HUMIDITY,
    TEMP
} SHT20_Measure_dat;

/*以下函数是温湿度传感器的I2C接口*/
void _IIC_Error_Handler(char *File, int Line);

static void th_iic_gpio_Init(void);

void th_iic_Init(void);

uint8_t i2c_reg_write( uint8_t device_addr, uint8_t reg, uint8_t value );

uint8_t i2c_reg_read( uint8_t device_addr, uint8_t reg, uint8_t *value );

uint8_t i2c_reg_read_len( uint8_t device_addr, uint8_t reg, uint32_t *value , uint8_t len);

uint8_t i2c_cmd_send( uint8_t device_addr, uint8_t cmd);


/*以下函数是对温湿度传感器的具体操作*/
uint8_t SHT20_crc_check(uint8_t data[], uint8_t nbrOfBytes, uint8_t checksum);

void SHT20_rest(void);

uint8_t SHT20_user_reg_read(uint8_t *RegValue);

uint8_t SHT20_user_reg_write(uint8_t RegValue);

uint16_t SHT20_measure_HM(SHT20_Measure_dat Mtype, uint16_t *RegValue);

float SHT20_humidityRH_cal(uint16_t dat);

float SHT20_temperatureC_cal(uint16_t dat);

void mmc_test(void);

#ifdef __cplusplus
extern "C" {
#endif

#endif