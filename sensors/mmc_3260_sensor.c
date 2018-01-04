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

MEMSIC3316xMT_DATA  MMC3260xMT_Result;          /* ��ǰ�ɼ������ݽ��                */
MEMSIC3316xMT_DATA  AMR_3260_Result_Normal;   /* ����ģʽ���ݽ��                */
MEMSIC3316xMT_DATA  AMR_3260_Result_Capture;  /* offsetУ׼ģʽ���ݽ��                */

uint8_t AMR_3260_XYZ_Register[6];           /* �������ڲ�XL XH YL YH ZL ZH��ֵ */
uint8_t AMR_3260_XYZ_Register_Normal[6];    /* �������ڲ�XL XH YL YH ZL ZH��ֵ */
uint8_t AMR_3260_XYZ_Register_Capture[6];   /* �������ڲ�XL XH YL YH ZL ZH��ֵ */

/*
 *  OPERATE
 */
uint8_t MMC_3260_RESET              =     0x40;
uint8_t MMC_3260_SET                =     0x20;

/***************************************************************************//**
 * @brief
 *   mmc3316xmt��RESET-SET����ȡ���ݣ��Ż�����������ȡ
 *   ʱ��RESET - 60ms - SET - 60ms - Take Measure - ��ѯ�ɼ�
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

    // ���ڲ��Ĵ���INTERNAL_CONTROL0д��һ���ֽڵĲ�������ָ��TAKE_MEASUREMENT
    if (MPU6050_I2C_ByteWrite(MEMSIC3316xMT_ADDRESS, &TAKE_MEASUREMENT, INTERNAL_CONTROL0) != 0)
    {
        char log[128];
        memset(log, 0, sizeof(log));
        snprintf(log, sizeof(log), "w\n");
        los_dev_uart_write(LOS_STM32L476_UART3, log, sizeof(log), 1000);
        return false;
    }
    osDelay(15);
    // �ӼĴ���MMC_STATUS�ж�ȡһ���ֽڵ����ݸ�ֵ��AMRState
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
    // ��slave�ڲ���ַXOUT_LOW��ʼ��������ȡ6���ֽڵ��Ĵ���AMR_3260_XYZ_Register��
    if (MPU6050_I2C_BufferRead(MEMSIC3316xMT_ADDRESS, AMR_3260_XYZ_Register, XOUT_LOW, 6) != 0)
    {
        return false;
    }
    return true;
}

/***************************************************************************//**
 * @brief
 *   mmc3316xmt RESET��ȡ���ݣ��Ż�����������ȡ
 *   ʱ��RESET-60ms-Take Measure-15ms-��ѯ�ɼ�
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
    // ���ڲ��Ĵ���INTERNAL_CONTROL0д��һ���ֽڵĸ�λָ��RESET
    if (MPU6050_I2C_ByteWrite(MEMSIC3316xMT_ADDRESS, &MMC_3260_RESET, INTERNAL_CONTROL0) != 0)
    {
        return false;
    }

    /* ע����ŵ���� ������Ҫ50mS*/
    osDelay(60);
    // ���ڲ��Ĵ���INTERNAL_CONTROL0д��һ���ֽڵĲ�������ָ��TAKE_MEASUREMENT
    if (MPU6050_I2C_ByteWrite(MEMSIC3316xMT_ADDRESS, &TAKE_MEASUREMENT, INTERNAL_CONTROL0) != 0)
    {
        return false;
    }

    osDelay(15);
    // �ӼĴ���MMC_STATUS�ж�ȡһ���ֽڵ����ݸ�ֵ��AMRState
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
    // ��slave�ڲ���ַXOUT_LOW��ʼ��������ȡ6���ֽڵ��Ĵ���AMR_3260_XYZ_Register��
    if (MPU6050_I2C_BufferRead(MEMSIC3316xMT_ADDRESS, AMR_3260_XYZ_Register_Normal, XOUT_LOW, 6) != 0)
    {
        return false;
    }
    return true;
}

/***************************************************************************//**
 * @brief
 *   mmc3316xmt SET��ȡ���ݣ��Ż�����������ȡ
 *   ʱ��SET-60ms-Take Measure- 15ms-��ѯ�ɼ�
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
    // ���ڲ��Ĵ���INTERNAL_CONTROL0д��һ���ֽڵ�����ָ��SET
    if (MPU6050_I2C_ByteWrite(MEMSIC3316xMT_ADDRESS, &MMC_3260_SET, INTERNAL_CONTROL0) != 0)
    {
        return false;
    }
    /* ע����ŵ���� ������Ҫ50mS*/
    osDelay(60);
    // ���ڲ��Ĵ���INTERNAL_CONTROL0д��һ���ֽڵĲ�������ָ��TAKE_MEASUREMENT
    if (MPU6050_I2C_ByteWrite(MEMSIC3316xMT_ADDRESS, &TAKE_MEASUREMENT, INTERNAL_CONTROL0) != 0)
    {
        return false;
    }
    osDelay(15);
    // �ӼĴ���MMC_STATUS�ж�ȡһ���ֽڵ����ݸ�ֵ��AMRState
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
    // ��slave�ڲ���ַXOUT_LOW��ʼ��������ȡ6���ֽڵ��Ĵ���AMR_3260_XYZ_Register��
    if (MPU6050_I2C_BufferRead(MEMSIC3316xMT_ADDRESS, AMR_3260_XYZ_Register_Capture, XOUT_LOW , 6) != 0)
    {
        return false;
    }
    return true;
}

/***************************************************************************//**
 * @brief
 *   ��6�����ݼĴ����е�ֵ���д���
 *
 * @param[in] i2c
 *   Pointer to I2C peripheral register block.
 *
 * @return MEMSIC3316xMT_DATA
 *   ����
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
    // ���ڲ��Ĵ���INTERNAL_CONTROL0д��һ���ֽڵĸ�λָ��RESET
    if (MPU6050_I2C_ByteWrite(MEMSIC3316xMT_ADDRESS, &MMC_3260_RESET, INTERNAL_CONTROL0) != 0)
    {
        return false;
    }
    osDelay(60);
    // ���ڲ��Ĵ���INTERNAL_CONTROL0д��һ���ֽڵ�����ָ��SET
    if (MPU6050_I2C_ByteWrite(MEMSIC3316xMT_ADDRESS, &MMC_3260_SET, INTERNAL_CONTROL0) != 0)
    {
        return false;
    }
    /* ע����ŵ���� ������Ҫ50mS*/
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