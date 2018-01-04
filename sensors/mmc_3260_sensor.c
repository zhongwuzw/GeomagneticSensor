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
#include "mmc_3260_sensor.h"
#include "uart_test.h"
#include "delay.h"
#include "mpu_sensor.h"

MEMSIC3316xMT_DATA  MMC3260xMT_Result;          /* 当前采集的数据结果                */
MEMSIC3316xMT_DATA  AMR_3260_Result_Normal;   /* 正常模式数据结果                */
MEMSIC3316xMT_DATA  AMR_3260_Result_Capture;  /* offset校准模式数据结果                */

uint8_t AMR_3260_XYZ_Register[6];           /* 传感器内部XL XH YL YH ZL ZH的值 */
uint8_t AMR_3260_XYZ_Register_Normal[6];    /* 传感器内部XL XH YL YH ZL ZH的值 */
uint8_t AMR_3260_XYZ_Register_Capture[6];   /* 传感器内部XL XH YL YH ZL ZH的值 */

/*
 *  OPERATE
 */
uint8_t MMC_3260_RESET              =     0x40;
uint8_t MMC_3260_SET                =     0x20;

/***************************************************************************//**
 * @brief
 *   mmc3316xmt（RESET-SET）获取数据，磁化、测量、读取
 *   时序：RESET - 60ms - SET - 60ms - Take Measure - 查询采集
 *
 * @param[in]
 *   null.
 *
 * @return
 *   null.
 ******************************************************************************/
bool mmc3316xmtAcqData (void)
{
    uint8_t AMRState;

    // 向内部寄存器INTERNAL_CONTROL0写入一个字节的测试启动指令TAKE_MEASUREMENT
    if (MPU6050_I2C_ByteWrite(MEMSIC3316xMT_ADDRESS, &TAKE_MEASUREMENT, INTERNAL_CONTROL0) != 0)
    {
        char log[128];
        memset(log, 0, sizeof(log));
        snprintf(log, sizeof(log), "w\n");
        los_dev_uart_write(LOS_STM32L476_UART3, log, sizeof(log), 1000);
        return false;
    }
    osDelay(15);
    // 从寄存器MMC_STATUS中读取一个字节的数据赋值给AMRState
    if (MPU6050_I2C_BufferRead(MEMSIC3316xMT_ADDRESS, &AMRState, MMC_STATUS, 1) != 0)
    {
        return false;
    }
    else
    {
        if (!(AMRState & 0x01))
        {
            return false;
        }
    }
    // 从slave内部地址XOUT_LOW开始，连续读取6个字节到寄存器AMR_3260_XYZ_Register中
    if (MPU6050_I2C_BufferRead(MEMSIC3316xMT_ADDRESS, AMR_3260_XYZ_Register, XOUT_LOW, 6) != 0)
    {
        return false;
    }
    return true;
}

/***************************************************************************//**
 * @brief
 *   mmc3316xmt RESET获取数据，磁化、测量、读取
 *   时序：RESET-60ms-Take Measure-15ms-查询采集
 *
 * @param[in]
 *   null.
 *
 * @return
 *   null.
 ******************************************************************************/
bool mmc3316xmtAcqReset (void)
{
    uint8_t AMRState;
    // 向内部寄存器INTERNAL_CONTROL0写入一个字节的复位指令RESET
    if (MPU6050_I2C_ByteWrite(MEMSIC3316xMT_ADDRESS, &MMC_3260_RESET, INTERNAL_CONTROL0) != 0)
    {
        return false;
    }

    /* 注意起磁的完成 至少需要50mS*/
    osDelay(60);
    // 向内部寄存器INTERNAL_CONTROL0写入一个字节的测试启动指令TAKE_MEASUREMENT
    if (MPU6050_I2C_ByteWrite(MEMSIC3316xMT_ADDRESS, &TAKE_MEASUREMENT, INTERNAL_CONTROL0) != 0)
    {
        return false;
    }

    osDelay(15);
    // 从寄存器MMC_STATUS中读取一个字节的数据赋值给AMRState
    if (MPU6050_I2C_BufferRead(MEMSIC3316xMT_ADDRESS, &AMRState, MMC_STATUS, 1) != 0)
    {
        return false;
    }
    else
    {
        if (!(AMRState & 0x01))
        {
            return false;
        }
    }
    // 从slave内部地址XOUT_LOW开始，连续读取6个字节到寄存器AMR_3260_XYZ_Register中
    if (MPU6050_I2C_BufferRead(MEMSIC3316xMT_ADDRESS, AMR_3260_XYZ_Register_Normal, XOUT_LOW, 6) != 0)
    {
        return false;
    }
    return true;
}

/***************************************************************************//**
 * @brief
 *   mmc3316xmt SET获取数据，磁化、测量、读取
 *   时序：SET-60ms-Take Measure- 15ms-查询采集
 *
 * @param[in]
 *   null.
 *
 * @return
 *   null.
 ******************************************************************************/
bool mmc3316xmtAcqSet (void)
{
    uint8_t AMRState;
    // 向内部寄存器INTERNAL_CONTROL0写入一个字节的配置指令SET
    if (MPU6050_I2C_ByteWrite(MEMSIC3316xMT_ADDRESS, &MMC_3260_SET, INTERNAL_CONTROL0) != 0)
    {
        return false;
    }
    /* 注意起磁的完成 至少需要50mS*/
    osDelay(60);
    // 向内部寄存器INTERNAL_CONTROL0写入一个字节的测试启动指令TAKE_MEASUREMENT
    if (MPU6050_I2C_ByteWrite(MEMSIC3316xMT_ADDRESS, &TAKE_MEASUREMENT, INTERNAL_CONTROL0) != 0)
    {
        return false;
    }
    osDelay(15);
    // 从寄存器MMC_STATUS中读取一个字节的数据赋值给AMRState
    if (MPU6050_I2C_BufferRead(MEMSIC3316xMT_ADDRESS, &AMRState, MMC_STATUS , 1) != 0)
    {
        return false;
    }
    else
    {
        if (!(AMRState & 0x01))
        {
            return false;
        }
    }
    // 从slave内部地址XOUT_LOW开始，连续读取6个字节到寄存器AMR_3260_XYZ_Register中
    if (MPU6050_I2C_BufferRead(MEMSIC3316xMT_ADDRESS, AMR_3260_XYZ_Register_Capture, XOUT_LOW , 6) != 0)
    {
        return false;
    }
    return true;
}

/***************************************************************************//**
 * @brief
 *   对6个数据寄存器中的值进行处理
 *
 * @param[in] i2c
 *   Pointer to I2C peripheral register block.
 *
 * @return MEMSIC3316xMT_DATA
 *   返回
 ******************************************************************************/
MEMSIC3316xMT_DATA *mmc3316xmtGetData (void)
{
    MMC3260xMT_Result.x = (AMR_3260_XYZ_Register[1] & 0x3F) << 8 | AMR_3260_XYZ_Register[0];
    MMC3260xMT_Result.y = (AMR_3260_XYZ_Register[3] & 0x3F) << 8 | AMR_3260_XYZ_Register[2];
    MMC3260xMT_Result.z = (AMR_3260_XYZ_Register[5] & 0x3F) << 8 | AMR_3260_XYZ_Register[4];
    return (&MMC3260xMT_Result);
}

bool MMC3316xmt_SetReset(void)
{
    //if(!mmc3316xmtAcqReset())
    //  return false;

    //if(!mmc3316xmtAcqSet())
    //  return false;

    //return true;
    // 向内部寄存器INTERNAL_CONTROL0写入一个字节的复位指令RESET
    if (MPU6050_I2C_ByteWrite(MEMSIC3316xMT_ADDRESS, &MMC_3260_RESET, INTERNAL_CONTROL0) != 0)
    {
        return false;
    }
    osDelay(60);
    // 向内部寄存器INTERNAL_CONTROL0写入一个字节的配置指令SET
    if (MPU6050_I2C_ByteWrite(MEMSIC3316xMT_ADDRESS, &MMC_3260_SET, INTERNAL_CONTROL0) != 0)
    {
        return false;
    }
    /* 注意起磁的完成 至少需要50mS*/
    osDelay(60);

    return true;
}

void Mmc3316xmt_Sensor()
{
    MPU6050_I2C_Init();
    osDelay(100);
    while(1)
    {
        if(mmc3316xmtAcqData())
        {
            mmc3316xmtGetData();

            uint32_t x = MMC3260xMT_Result.x;
            uint32_t y = MMC3260xMT_Result.y;
            uint32_t z = MMC3260xMT_Result.z;

            char log[128];
            memset(log, 0, sizeof(log));
            snprintf(log, sizeof(log), "x is %d,y is %d,z is %d\n", x, y, z);
            los_dev_uart_write(LOS_STM32L476_UART3, log, sizeof(log), 1000);

        }
        osDelay(1000);
    }

}