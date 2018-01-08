#include "th_sensor.h"
#include "uart_test.h"
#include "mmc_sensor.h"
#include <stdio.h>
#include <math.h>
#include <string.h>
#include <stdlib.h>

uint16_t  POLYNOMIAL = 0x131;
I2C_HandleTypeDef th_iic;

typedef uint16_t BYTE;
BYTE BUF[8];

void _IIC_Error_Handler(char *File, int Line)
{
    osDelay(1000);
}

static void th_iic_gpio_Init(void)
{

    /* GPIO Ports Clock Enable */
    __HAL_RCC_GPIOB_CLK_ENABLE();

}

void th_iic_Init(void)
{
    th_iic_gpio_Init();

    th_iic.Instance = I2C1;
    //th_iic.Init.Timing = 0x10909CEC;
    th_iic.Init.Timing = 0x00000004;
    th_iic.Init.OwnAddress1 = 0;
    th_iic.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    th_iic.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    th_iic.Init.OwnAddress2 = 0;
    th_iic.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
    th_iic.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    th_iic.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
    if (HAL_I2C_Init(&th_iic) != HAL_OK)
    {
        _IIC_Error_Handler(__FILE__, __LINE__);
    }

    /**Configure Analogue filter
    */
    if (HAL_I2CEx_ConfigAnalogFilter(&th_iic, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
    {
        _IIC_Error_Handler(__FILE__, __LINE__);
    }

    /**Configure Digital filter
    */
    if (HAL_I2CEx_ConfigDigitalFilter(&th_iic, 0) != HAL_OK)
    {
        _IIC_Error_Handler(__FILE__, __LINE__);
    }

}

uint8_t i2c_reg_write( uint8_t device_addr, uint8_t reg, uint8_t value )
{
    uint8_t status = HAL_OK;
    uint8_t TxData[2] = {0};
    TxData[0] = reg;
    TxData[1] = value;

    status = HAL_I2C_Master_Transmit(&th_iic, device_addr, TxData, 2, 1000);
    if(status != HAL_OK)
    {
        char log1[128];
        memset(log1, 0, sizeof(log1));
        snprintf(log1, sizeof(log1), "3\n");
        los_dev_uart_write(LOS_STM32L476_UART3, log1, sizeof(log1), 1000);
        osDelay(100);
    }
    return status;
}

uint8_t i2c_reg_read( uint8_t device_addr, uint8_t reg, uint8_t *value )
{
    uint8_t status1 = HAL_OK, status2 = HAL_OK;
    static uint8_t  msg[1];
    uint8_t value_read[2];
    msg[0] = reg;

    status1 = HAL_I2C_Master_Transmit(&th_iic, device_addr, msg, 1, 1000);
    status2 = HAL_I2C_Master_Receive(&th_iic, device_addr, value_read, 1, 1000);

    *value = value_read[0];
    if((status1 != HAL_OK) || (status2 != HAL_OK))
    {
        char log1[128];
        memset(log1, 0, sizeof(log1));
        snprintf(log1, sizeof(log1), "4\n");
        los_dev_uart_write(LOS_STM32L476_UART3, log1, sizeof(log1), 1000);
        osDelay(100);
    }
    return (status1 | status2);
}


void SHT20_rest(void)
{
    uint8_t status = HAL_OK;
    i2c_reg_write(SHT20_WRITE_ADDRESS, 0x00, 0x70);
    i2c_reg_write(SHT20_WRITE_ADDRESS, 0x01, 0xA0);
    i2c_reg_write(SHT20_WRITE_ADDRESS, 0x02, 0x00);
    HAL_Delay(6);
    if(status != HAL_OK)
    {
    }
}

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
bool mmc3260AcqData (void)
{
    uint8_t AMRState;

    // 向内部寄存器INTERNAL_CONTROL0写入一个字节的测试启动指令TAKE_MEASUREMENT
    if (i2c_reg_write(SHT20_WRITE_ADDRESS, INTERNAL_CONTROL0, TAKE_MEASUREMENT) != 0)
    {
        char log[128];
        memset(log, 0, sizeof(log));
        snprintf(log, sizeof(log), "w\n");
        los_dev_uart_write(LOS_STM32L476_UART3, log, sizeof(log), 1000);
        return false;
    }
    osDelay(15);
    // 从寄存器MMC_STATUS中读取一个字节的数据赋值给AMRState
    uint8_t status;
    if (i2c_reg_read( SHT20_READ_ADDRESS, MMC_STATUS, &status ) != 0)
    {
        return false;
    }
    else
    {
			return true;
			        char log[128];
        memset(log, 0, sizeof(log));
        snprintf(log, sizeof(log), "7\n");
        los_dev_uart_write(LOS_STM32L476_UART3, log, sizeof(log), 1000);
        if (!(AMRState & 0x01))
        {
            return false;
        }
    }
    return true;
}

void mmc_test(void)
{
    char log[128];

    memset(log, 0, sizeof(log));
    snprintf(log, sizeof(log), "6\n");
    los_dev_uart_write(LOS_STM32L476_UART3, log, sizeof(log), 1000);
    HAL_Delay(1200);

    while(1)
    {
        if (mmc3260AcqData())
        {
            // 从slave内部地址XOUT_LOW开始，连续读取6个字节到寄存器AMR_3260_XYZ_Register中
            uint8_t x_low;
            i2c_reg_read( SHT20_READ_ADDRESS, 0x00, &x_low );
            uint8_t x_high;
            i2c_reg_read( SHT20_READ_ADDRESS, 0x01, &x_high );
            uint8_t y_low;
            i2c_reg_read( SHT20_READ_ADDRESS, 0x02, &y_low );

            uint8_t y_high;
            i2c_reg_read( SHT20_READ_ADDRESS, 0x03, &y_high );
            uint8_t z_low;
            i2c_reg_read( SHT20_READ_ADDRESS, 0x04, &z_low );

            uint8_t z_high;
            i2c_reg_read( SHT20_READ_ADDRESS, 0x05, &z_high );


            short x = (x_high & 0x3F) << 8 | x_low;
            short y = (y_high & 0x3F) << 8 | y_low;
            short z = (z_high & 0x3F) << 8 | z_low;

            memset(log, 0, sizeof(log));
            snprintf(log, sizeof(log), "x is %d,y is %d,z is %d\n", x, y, z);
            los_dev_uart_write(LOS_STM32L476_UART3, log, sizeof(log), 1000);
            HAL_Delay(1200);
        }
        else
        {
            memset(log, 0, sizeof(log));
            snprintf(log, sizeof(log), "5\n");
            los_dev_uart_write(LOS_STM32L476_UART3, log, sizeof(log), 1000);
            HAL_Delay(1200);
        }
    }
}
