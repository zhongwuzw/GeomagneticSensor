#include "th_sensor.h"
#include "uart_test.h"
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

    if (HAL_I2C_IsDeviceReady(&th_iic, 0x60, 1000, 1000) != HAL_OK)
    {
        char log1[128];
        memset(log1, 0, sizeof(log1));
        snprintf(log1, sizeof(log1), "1\n");
        los_dev_uart_write(LOS_STM32L476_UART3, log1, sizeof(log1), 1000);
    }
    else
    {
        char log1[128];
        memset(log1, 0, sizeof(log1));
        snprintf(log1, sizeof(log1), "2\n");
        los_dev_uart_write(LOS_STM32L476_UART3, log1, sizeof(log1), 1000);
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
    }
    return status;
}

uint8_t i2c_reg_read_hmc( uint8_t device_addr, uint8_t reg) //len =  6
{
    uint8_t status1 = HAL_OK, status2 = HAL_OK;
    static uint8_t  msg[1];
    uint8_t value_read[8] = {0};
    msg[0] = reg;

    status1 = HAL_I2C_Master_Transmit(&th_iic, device_addr, msg, 1, 1000);
    status2 = HAL_I2C_Master_Receive(&th_iic, device_addr, value_read, 6, 1000);

    int i = 0;
    for (i = 0; i < 6; i++)
    {
        BUF[i] = value_read[i];
    }

    if((status1 != HAL_OK) || (status2 != HAL_OK))
    {
    }
    return (status1 | status2);
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
    if((status1 != HAL_OK) || (status2 != HAL_OK)) {}
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

void SHT20_test(void)
{
    char log[128];
    uint16_t data[2] = {0};
    uint16_t hum = 0;

    SHT20_rest();
    osDelay(1000);

    while(1)
    {
        i2c_reg_read_hmc(SHT20_READ_ADDRESS, 0x06);
        i2c_reg_write(SHT20_WRITE_ADDRESS, 0x03, 0x00);
        //float x = BUF[1] << 8 | BUF[0];
        //	float y = BUF[3] << 8 | BUF[2];
        //		float z = BUF[5] << 8 | BUF[4];

        short x = BUF[0] << 8 | BUF[1];
        short y = BUF[2] << 8 | BUF[3];
        short z = BUF[4] << 8 | BUF[5];

        char log1[128];
        memset(log1, 0, sizeof(log1));
        snprintf(log1, sizeof(log1), "x is %d\n", x);
        //    los_dev_uart_write(LOS_STM32L476_UART3, log1, sizeof(log1), 1000);
        HAL_Delay(100);

        memset(log, 0, sizeof(log));
        //snprintf(log, sizeof(log), "x is %d,y is %d,z is %d\n",x,y,z);
        snprintf(log, sizeof(log), "x0 is %x,x1 is %x\n", BUF[0], BUF[1]);
        // los_dev_uart_write(LOS_STM32L476_UART3, log, sizeof(log), 1000);
        HAL_Delay(1200);
    }
}
