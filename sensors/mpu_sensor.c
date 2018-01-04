/* ============================================================================================
MPU6050 device I2C library code for ARM STM32F103xx is placed under the MIT license
Copyright (c) 2012 Harinadha Reddy Chintalapalli

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
================================================================================================
*/

/* Includes */
#include "mpu_sensor.h"
#include "uart_test.h"
#include "delay.h"

#define IIC_SDA GPIO_PIN_9
#define IIC_SCL GPIO_PIN_8

typedef uint8_t BYTE;
BYTE HMC_BUF[8];

/** @defgroup MPU6050_Library
* @{
*/

/** Power on and prepare for general usage.
 * This will activate the device and take it out of sleep mode (which must be done
 * after start-up). This function also sets both the accelerometer and the gyroscope
 * to their most sensitive settings, namely +/- 2g and +/- 250 degrees/sec, and sets
 * the clock source to use the X Gyro for reference, which is slightly better than
 * the default internal clock source.
 */
void MPU6050_Initialize()
{
    MPU6050_SetClockSource(MPU6050_CLOCK_PLL_XGYRO);
    MPU6050_SetFullScaleGyroRange(MPU6050_GYRO_FS_250);
    MPU6050_SetFullScaleAccelRange(MPU6050_ACCEL_FS_2);
    MPU6050_SetSleepModeStatus(DISABLE);
}

void HMC5983_Initialize()
{
    // 8-average, 15Hz default, normal measurement
    uint8_t option_for_0 = 0x70;
    uint8_t status = MPU6050_I2C_ByteWrite(0x1E, &option_for_0, 0x00);
    osDelay(100);

    // Gain = 5
    uint8_t option_for_1 = 0xA0;
    MPU6050_I2C_ByteWrite(0x1E, &option_for_1, 0x01);
    osDelay(100);

    // Continuous-measurement mode
    uint8_t option_for_2 = 0x00;
    MPU6050_I2C_ByteWrite(0x1E, &option_for_2, 0x02);
    osDelay(100);
}

/** Verify the I2C connection.
 * Make sure the device is connected and responds as expected.
 * @return True if connection is valid, FALSE otherwise
 */
bool MPU6050_TestConnection()
{
    if(MPU6050_GetDeviceID() == 0x34) //0b110100; 8-bit representation in hex = 0x34
        return TRUE;
    else
        return FALSE;
}
// WHO_AM_I register

/** Get Device ID.
 * This register is used to verify the identity of the device (0b110100).
 * @return Device ID (should be 0x68, 104 dec, 150 oct)
 * @see MPU6050_RA_WHO_AM_I
 * @see MPU6050_WHO_AM_I_BIT
 * @see MPU6050_WHO_AM_I_LENGTH
 */
uint8_t MPU6050_GetDeviceID()
{
    uint8_t tmp;
    MPU6050_ReadBits(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_WHO_AM_I, MPU6050_WHO_AM_I_BIT, MPU6050_WHO_AM_I_LENGTH, &tmp);
    return tmp;
}
/** Set clock source setting.
 * An internal 8MHz oscillator, gyroscope based clock, or external sources can
 * be selected as the MPU-60X0 clock source. When the internal 8 MHz oscillator
 * or an external source is chosen as the clock source, the MPU-60X0 can operate
 * in low power modes with the gyroscopes disabled.
 *
 * Upon power up, the MPU-60X0 clock source defaults to the internal oscillator.
 * However, it is highly recommended that the device be configured to use one of
 * the gyroscopes (or an external clock source) as the clock reference for
 * improved stability. The clock source can be selected according to the following table:
 *
 * <pre>
 * CLK_SEL | Clock Source
 * --------+--------------------------------------
 * 0       | Internal oscillator
 * 1       | PLL with X Gyro reference
 * 2       | PLL with Y Gyro reference
 * 3       | PLL with Z Gyro reference
 * 4       | PLL with external 32.768kHz reference
 * 5       | PLL with external 19.2MHz reference
 * 6       | Reserved
 * 7       | Stops the clock and keeps the timing generator in reset
 * </pre>
 *
 * @param source New clock source setting
 * @see MPU6050_GetClockSource()
 * @see MPU6050_RA_PWR_MGMT_1
 * @see MPU6050_PWR1_CLKSEL_BIT
 * @see MPU6050_PWR1_CLKSEL_LENGTH
 */
void MPU6050_SetClockSource(uint8_t source)
{
    MPU6050_WriteBits(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_CLKSEL_BIT, MPU6050_PWR1_CLKSEL_LENGTH, source);
}

/** Set full-scale gyroscope range.
 * @param range New full-scale gyroscope range value
 * @see MPU6050_GetFullScaleGyroRange()
 * @see MPU6050_GYRO_FS_250
 * @see MPU6050_RA_GYRO_CONFIG
 * @see MPU6050_GCONFIG_FS_SEL_BIT
 * @see MPU6050_GCONFIG_FS_SEL_LENGTH
 */
void MPU6050_SetFullScaleGyroRange(uint8_t range)
{
    MPU6050_WriteBits(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_GYRO_CONFIG, MPU6050_GCONFIG_FS_SEL_BIT, MPU6050_GCONFIG_FS_SEL_LENGTH, range);
}

// GYRO_CONFIG register

/** Get full-scale gyroscope range.
 * The FS_SEL parameter allows setting the full-scale range of the gyro sensors,
 * as described in the table below.
 *
 * <pre>
 * 0 = +/- 250 degrees/sec
 * 1 = +/- 500 degrees/sec
 * 2 = +/- 1000 degrees/sec
 * 3 = +/- 2000 degrees/sec
 * </pre>
 *
 * @return Current full-scale gyroscope range setting
 * @see MPU6050_GYRO_FS_250
 * @see MPU6050_RA_GYRO_CONFIG
 * @see MPU6050_GCONFIG_FS_SEL_BIT
 * @see MPU6050_GCONFIG_FS_SEL_LENGTH
 */
uint8_t MPU6050_GetFullScaleGyroRange()
{
    uint8_t tmp;
    MPU6050_ReadBits(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_GYRO_CONFIG, MPU6050_GCONFIG_FS_SEL_BIT, MPU6050_GCONFIG_FS_SEL_LENGTH, &tmp);
    return tmp;
}
/** Get full-scale accelerometer range.
 * The FS_SEL parameter allows setting the full-scale range of the accelerometer
 * sensors, as described in the table below.
 *
 * <pre>
 * 0 = +/- 2g
 * 1 = +/- 4g
 * 2 = +/- 8g
 * 3 = +/- 16g
 * </pre>
 *
 * @return Current full-scale accelerometer range setting
 * @see MPU6050_ACCEL_FS_2
 * @see MPU6050_RA_ACCEL_CONFIG
 * @see MPU6050_ACONFIG_AFS_SEL_BIT
 * @see MPU6050_ACONFIG_AFS_SEL_LENGTH
 */
uint8_t MPU6050_GetFullScaleAccelRange()
{
    uint8_t tmp;
    MPU6050_ReadBits(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_ACCEL_CONFIG, MPU6050_ACONFIG_AFS_SEL_BIT, MPU6050_ACONFIG_AFS_SEL_LENGTH, &tmp);
    return tmp;
}
/** Set full-scale accelerometer range.
 * @param range New full-scale accelerometer range setting
 * @see MPU6050_GetFullScaleAccelRange()
 */
void MPU6050_SetFullScaleAccelRange(uint8_t range)
{
    MPU6050_WriteBits(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_ACCEL_CONFIG, MPU6050_ACONFIG_AFS_SEL_BIT, MPU6050_ACONFIG_AFS_SEL_LENGTH, range);
}

/** Get sleep mode status.
 * Setting the SLEEP bit in the register puts the device into very low power
 * sleep mode. In this mode, only the serial interface and internal registers
 * remain active, allowing for a very low standby current. Clearing this bit
 * puts the device back into normal mode. To save power, the individual standby
 * selections for each of the gyros should be used if any gyro axis is not used
 * by the application.
 * @return Current sleep mode enabled status
 * @see MPU6050_RA_PWR_MGMT_1
 * @see MPU6050_PWR1_SLEEP_BIT
 */
bool MPU6050_GetSleepModeStatus()
{
    uint8_t tmp;
    MPU6050_ReadBit(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_SLEEP_BIT, &tmp);
    if(tmp == 0x00)
        return FALSE;
    else
        return TRUE;
}
/** Set sleep mode status.
 * @param enabled New sleep mode enabled status
 * @see MPU6050_GetSleepModeStatus()
 * @see MPU6050_RA_PWR_MGMT_1
 * @see MPU6050_PWR1_SLEEP_BIT
 */
void MPU6050_SetSleepModeStatus(FunctionalState NewState)
{
    MPU6050_WriteBit(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_SLEEP_BIT, NewState);
}

/** Get raw 6-axis motion sensor readings (accel/gyro).
 * Retrieves all currently available motion sensor values.
 * @param AccelGyro 16-bit signed integer array of length 6
 * @see MPU6050_RA_ACCEL_XOUT_H
 */
void MPU6050_GetRawAccelGyro(int16_t *AccelGyro)
{
    uint8_t tmpBuffer[14];
    MPU6050_I2C_BufferRead(MPU6050_DEFAULT_ADDRESS, tmpBuffer, MPU6050_RA_ACCEL_XOUT_H, 14);
    /* Get acceleration */
    for(int i = 0; i < 3; i++)
        AccelGyro[i] = ((int16_t)((uint16_t)tmpBuffer[2 * i] << 8) + tmpBuffer[2 * i + 1]);
    /* Get Angular rate */
    for(int i = 4; i < 7; i++)
        AccelGyro[i - 1] = ((int16_t)((uint16_t)tmpBuffer[2 * i] << 8) + tmpBuffer[2 * i + 1]);

}

/** Write multiple bits in an 8-bit device register.
 * @param slaveAddr I2C slave device address
 * @param regAddr Register regAddr to write to
 * @param bitStart First bit position to write (0-7)
 * @param length Number of bits to write (not more than 8)
 * @param data Right-aligned value to write
 */
void MPU6050_WriteBits(uint8_t slaveAddr, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t data)
{
    //      010 value to write
    // 76543210 bit numbers
    //    xxx   args: bitStart=4, length=3
    // 00011100 mask byte
    // 10101111 original value (sample)
    // 10100011 original & ~mask
    // 10101011 masked | value
    uint8_t tmp;
    MPU6050_I2C_BufferRead(slaveAddr, &tmp, regAddr, 1);
    uint8_t mask = ((1 << length) - 1) << (bitStart - length + 1);
    data <<= (bitStart - length + 1); // shift data into correct position
    data &= mask; // zero all non-important bits in data
    tmp &= ~(mask); // zero all important bits in existing byte
    tmp |= data; // combine data with existing byte
    MPU6050_I2C_ByteWrite(slaveAddr, &tmp, regAddr);
}
/** write a single bit in an 8-bit device register.
 * @param slaveAddr I2C slave device address
 * @param regAddr Register regAddr to write to
 * @param bitNum Bit position to write (0-7)
 * @param value New bit value to write
 */
void MPU6050_WriteBit(uint8_t slaveAddr, uint8_t regAddr, uint8_t bitNum, uint8_t data)
{
    uint8_t tmp;
    MPU6050_I2C_BufferRead(slaveAddr, &tmp, regAddr, 1);
    tmp = (data != 0) ? (tmp | (1 << bitNum)) : (tmp & ~(1 << bitNum));
    MPU6050_I2C_ByteWrite(slaveAddr, &tmp, regAddr);
}
/** Read multiple bits from an 8-bit device register.
 * @param slaveAddr I2C slave device address
 * @param regAddr Register regAddr to read from
 * @param bitStart First bit position to read (0-7)
 * @param length Number of bits to read (not more than 8)
 * @param data Container for right-aligned value (i.e. '101' read from any bitStart position will equal 0x05)
 * @param timeout Optional read timeout in milliseconds (0 to disable, leave off to use default class value in readTimeout)
 */
void MPU6050_ReadBits(uint8_t slaveAddr, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t *data)
{
    // 01101001 read byte
    // 76543210 bit numbers
    //    xxx   args: bitStart=4, length=3
    //    010   masked
    //   -> 010 shifted
    uint8_t tmp;
    MPU6050_I2C_BufferRead(slaveAddr, &tmp, regAddr, 1);
    uint8_t mask = ((1 << length) - 1) << (bitStart - length + 1);
    tmp &= mask;
    tmp >>= (bitStart - length + 1);
    *data = tmp;
}

/** Read a single bit from an 8-bit device register.
 * @param slaveAddr I2C slave device address
 * @param regAddr Register regAddr to read from
 * @param bitNum Bit position to read (0-7)
 * @param data Container for single bit value
 * @param timeout Optional read timeout in milliseconds (0 to disable, leave off to use default class value in readTimeout)
 */
void MPU6050_ReadBit(uint8_t slaveAddr, uint8_t regAddr, uint8_t bitNum, uint8_t *data)
{
    uint8_t tmp;
    MPU6050_I2C_BufferRead(slaveAddr, &tmp, regAddr, 1);
    *data = tmp & (1 << bitNum);
}

/**********************************STM32 I2C Interface**********************************/
/**
* @brief  Initializes the I2C peripheral used to drive the MPU6050
* @param  None
* @return None
*/
void MPU6050_I2C_Init()
{
    GPIO_InitTypeDef GPIO_InitStruct;

    /* GPIO Ports Clock Enable */
    __HAL_RCC_GPIOB_CLK_ENABLE();

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8 | GPIO_PIN_9, GPIO_PIN_RESET);

    /*Configure GPIO pins : PB14 PB7 */
    GPIO_InitStruct.Pin = GPIO_PIN_8 | GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Pull = GPIO_PULLDOWN;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    HAL_GPIO_WritePin(GPIOB, IIC_SCL, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOB, IIC_SDA, GPIO_PIN_SET);
}

void MPU6050_I2C_Start(void)
{
    IIC_SDA_OUT();
    HAL_GPIO_WritePin(GPIOB, IIC_SDA, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOB, IIC_SCL, GPIO_PIN_SET);
    delay_us(4);
    HAL_GPIO_WritePin(GPIOB, IIC_SDA, GPIO_PIN_RESET);
    delay_us(4);
    HAL_GPIO_WritePin(GPIOB, IIC_SCL, GPIO_PIN_RESET);
}

void MPU6050_I2C_Stop(void)
{
    IIC_SDA_OUT();
    HAL_GPIO_WritePin(GPIOB, IIC_SCL, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, IIC_SDA, GPIO_PIN_RESET);
    delay_us(4);
    HAL_GPIO_WritePin(GPIOB, IIC_SCL, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOB, IIC_SDA, GPIO_PIN_SET);
    delay_us(4);
}

uint8_t MPU6050_I2C_Wait_Ack(void)
{
    uint8_t ucErrTime = 0;
    IIC_SDA_IN();
    HAL_GPIO_WritePin(GPIOB, IIC_SDA, GPIO_PIN_SET);
    delay_us(1);
    HAL_GPIO_WritePin(GPIOB, IIC_SCL, GPIO_PIN_SET);
    delay_us(1);
    while(HAL_GPIO_ReadPin(GPIOB, IIC_SDA))
    {
        ucErrTime++;
        if(ucErrTime > 250)
        {
            MPU6050_I2C_Stop();
            return 1;
        }
    }
    HAL_GPIO_WritePin(GPIOB, IIC_SCL, GPIO_PIN_RESET);
    return 0;
}

void MPU6050_I2C_Ack(void)
{
    HAL_GPIO_WritePin(GPIOB, IIC_SCL, GPIO_PIN_RESET);
    IIC_SDA_OUT();
    HAL_GPIO_WritePin(GPIOB, IIC_SDA, GPIO_PIN_RESET);
    delay_us(2);
    HAL_GPIO_WritePin(GPIOB, IIC_SCL, GPIO_PIN_SET);
    delay_us(2);
    HAL_GPIO_WritePin(GPIOB, IIC_SCL, GPIO_PIN_RESET);
}

void MPU6050_I2C_NAck(void)
{
    HAL_GPIO_WritePin(GPIOB, IIC_SCL, GPIO_PIN_RESET);
    IIC_SDA_OUT();
    HAL_GPIO_WritePin(GPIOB, IIC_SDA, GPIO_PIN_SET);
    delay_us(2);
    HAL_GPIO_WritePin(GPIOB, IIC_SCL, GPIO_PIN_SET);
    delay_us(2);
    HAL_GPIO_WritePin(GPIOB, IIC_SCL, GPIO_PIN_RESET);
}

void MPU6050_I2C_Send_Byte(uint8_t txd)
{
    uint8_t t;
    IIC_SDA_OUT();
    HAL_GPIO_WritePin(GPIOB, IIC_SCL, GPIO_PIN_RESET);
    for(t = 0; t < 8; t++)
    {
        if((txd & 0x80) >> 7)
            HAL_GPIO_WritePin(GPIOB, IIC_SDA, GPIO_PIN_SET);
        else
            HAL_GPIO_WritePin(GPIOB, IIC_SDA, GPIO_PIN_RESET);
        txd <<= 1;
        delay_us(2);
        HAL_GPIO_WritePin(GPIOB, IIC_SCL, GPIO_PIN_SET);
        delay_us(2);
        HAL_GPIO_WritePin(GPIOB, IIC_SCL, GPIO_PIN_RESET);
        delay_us(2);
    }
}

uint8_t MPU6050_I2C_Read_Byte(unsigned char ack)
{
    unsigned char i, receive = 0;
    IIC_SDA_IN();
    for(i = 0; i < 8; i++ )
    {
        HAL_GPIO_WritePin(GPIOB, IIC_SCL, GPIO_PIN_RESET);
        delay_us(2);
        HAL_GPIO_WritePin(GPIOB, IIC_SCL, GPIO_PIN_SET);
        receive <<= 1;
        if(HAL_GPIO_ReadPin(GPIOB, IIC_SDA))receive++;
        delay_us(1);
    }
    if (!ack)
        MPU6050_I2C_NAck();
    else
        MPU6050_I2C_Ack();
    return receive;
}


/**
* @brief  Writes one byte to the  MPU6050.
* @param  slaveAddr : slave address MPU6050_DEFAULT_ADDRESS
* @param  pBuffer : pointer to the buffer  containing the data to be written to the MPU6050.
* @param  writeAddr : address of the register in which the data will be written
* @return None
*/
uint8_t MPU6050_I2C_ByteWrite(uint8_t slaveAddr, uint8_t *pBuffer, uint8_t writeAddr)
{
    MPU6050_I2C_Start();
    MPU6050_I2C_Send_Byte((slaveAddr << 1) | 0);
    if(MPU6050_I2C_Wait_Ack())
    {
        MPU6050_I2C_Stop();
        return 1;
    }
    MPU6050_I2C_Send_Byte(writeAddr);
    MPU6050_I2C_Wait_Ack();
    MPU6050_I2C_Send_Byte(*pBuffer);
    if(MPU6050_I2C_Wait_Ack())
    {
        MPU6050_I2C_Stop();
        return 1;
    }
    MPU6050_I2C_Stop();
    return 0;
}

/**
* @brief  Reads a block of data from the MPU6050.
* @param  slaveAddr  : slave address MPU6050_DEFAULT_ADDRESS
* @param  pBuffer : pointer to the buffer that receives the data read from the MPU6050.
* @param  readAddr : MPU6050's internal address to read from.
* @param  NumByteToRead : number of bytes to read from the MPU6050 ( NumByteToRead >1  only for the Mgnetometer readinf).
* @return None
*/

uint8_t MPU6050_I2C_BufferRead(uint8_t slaveAddr, uint8_t *pBuffer, uint8_t readAddr, uint16_t NumByteToRead)
{
    MPU6050_I2C_Start();
    MPU6050_I2C_Send_Byte((slaveAddr << 1) | 0);
    if(MPU6050_I2C_Wait_Ack())
    {
        MPU6050_I2C_Stop();
        return 1;
    }
    MPU6050_I2C_Send_Byte(readAddr);
    MPU6050_I2C_Wait_Ack();
    MPU6050_I2C_Start();
    MPU6050_I2C_Send_Byte((slaveAddr << 1) | 1);
    MPU6050_I2C_Wait_Ack();
    while(NumByteToRead)
    {
        if(NumByteToRead == 1)*pBuffer = MPU6050_I2C_Read_Byte(0);
        else *pBuffer = MPU6050_I2C_Read_Byte(1);
        NumByteToRead--;
        pBuffer++;
    }
    MPU6050_I2C_Stop();
    return 0;
}

short sTmp;

void MPU6050_test()
{
    MPU6050_I2C_Init();
    osDelay(100);
    HMC5983_Initialize();

    while(1)
    {
        osDelay(100);

        // read x,y,z from register
        uint8_t x_msb = 0;
        MPU6050_I2C_BufferRead(0x1E, &x_msb, 0x03, 1);
        uint8_t x_lsb = 0;
        MPU6050_I2C_BufferRead(0x1E, &x_lsb, 0x04, 1);

        uint8_t y_msb = 0;
        MPU6050_I2C_BufferRead(0x1E, &y_msb, 0x07, 1);
        uint8_t y_lsb = 0;
        MPU6050_I2C_BufferRead(0x1E, &y_lsb, 0x08, 1);

        uint8_t z_msb = 0;
        MPU6050_I2C_BufferRead(0x1E, &z_msb, 0x05, 1);
        uint8_t z_lsb = 0;
        MPU6050_I2C_BufferRead(0x1E, &z_lsb, 0x06, 1);

        short x = x_msb << 8 | x_lsb;
        short y = y_msb << 8 | y_lsb;
        short z = z_msb << 8 | z_lsb;

        float scale_x = x * 0.92;
        float scale_y = y * 0.92;
        float scale_z = z * 0.92;

        char log[128];
        memset(log, 0, sizeof(log));
        snprintf(log, sizeof(log), "x is %.2f,y is %.2f,z is %.2f\n", scale_x, scale_y, scale_z);
        los_dev_uart_write(LOS_STM32L476_UART3, log, sizeof(log), 1000);

        // reset register to 0x03
        uint8_t rebaseRegisterAddr = 0x00;
        MPU6050_I2C_ByteWrite(0x1E, &rebaseRegisterAddr, 0x03);
        osDelay(1000);
    }
}
/**
 * @}
 */ /* end of group MPU6050_Library */