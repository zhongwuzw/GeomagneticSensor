/**************************************************************************//**
 * @file
 * @brief MMC3316xMT driver code for Energy Micro EFM32TG110F32
 * @note
 *    null
 * @author Rock
 * @version 1.0.0
 ******************************************************************************
 * @section License
 * <b>(C) Copyright 2012 Energy Micro AS, http://www.energymicro.com</b>
 ******************************************************************************
 * NOTES:
 ******************************************************************************/

#ifndef __MMC3260XMT_H
#define __MMC3260XMT_H

#include "stm32l4xx.h"
#include "cmsis_os.h"
#include <stdbool.h>
/*
 *  DEVICE ADDRESS
 */
#define MEMSIC3316xMT_ADDRESS   0x30//MMC33162MT //0x60 // MMC33160MT

/*
 *  REGISTER ADDRESS
 */
#define XOUT_LOW                0x00
#define XOUT_HIGH               0x01
#define YOUT_LOW                0x02
#define YOUT_HIGH               0x03
#define ZOUT_LOW                0x04
#define ZOUT_HIGH               0x05
#define MMC_STATUS              0x06
#define INTERNAL_CONTROL0       0x07
#define INTERNAL_CONTROL1       0x08
#define PRODUCTID0              0x10
#define PRODUCTID1              0x20

extern uint8_t TAKE_MEASUREMENT;
extern uint8_t MMC_3260_RESET;
extern uint8_t MMC_3260_SET;
extern uint8_t TWICERESET;

typedef struct
{
    uint32_t x;
    uint32_t y;
    uint32_t z;
} MEMSIC3316xMT_DATA;

extern MEMSIC3316xMT_DATA  MMC3316xMT_Result;
extern bool mmc3316xmtAcqData (void);
extern bool mmc3316xmtAcqReset (void);
extern bool mmc3316xmtAcqSet (void);
extern MEMSIC3316xMT_DATA *mmc3316xmtGetData (void);

bool MMC3316xmt_SetReset(void);
void Mmc3316xmt_Sensor();

#endif