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
#include "uart_test.h"
#include "delay.h"
#include "mmc_sensor.h"
#include "mpu_sensor.h"


#define IIC_SDA GPIO_PIN_9
#define IIC_SCL GPIO_PIN_8

typedef uint8_t BYTE;
BYTE MMC_BUF[8];
MEMSIC35240PJ_DATA  MMC35240PJ_Result;          /* ¦Ì¡À?¡ã2¨¦?¡¥¦Ì?¨ºy?Y?¨¢1?                */
MEMSIC35240PJ_DATA  AMR_Result_Capture;         /* offsetD¡ê¡Á??¡ê¨º?¨ºy?Y?¨¢1?                */

MEMSIC35240PJ_SETDATA  MMC35240PJ_Setresult;
MEMSIC35240PJ_SETDATA  AMR_XYZ_Register_Set;

MEMSIC35240PJ_RESETDATA  MMC35240PJ_Resetresult;
MEMSIC35240PJ_RESETDATA  AMR_XYZ_Register_Reset;

uint8_t AMR_XYZ_Register[6];           /* ¡ä??D?¡Â?¨²2?XL XH YL YH ZL ZH¦Ì??¦Ì */
// uint8_t AMR_XYZ_Register_Set[6];       /* ¡ä??D?¡Â?¨²2?XL XH YL YH ZL ZH¦Ì??¦Ì */
// uint8_t AMR_XYZ_Register_Reset[6];     /* ¡ä??D?¡Â?¨²2?XL XH YL YH ZL ZH¦Ì??¦Ì */
/*
 *  OPERATE
 */
uint8_t TAKE_MEASUREMENT   =     0x01;
uint8_t MMC_RESET              =     0x40;
uint8_t MMC_SET                =     0x20;
uint8_t TWICERESET         =     0x40;

uint8_t ModeSet = OUT_DATA_MODE_14; //OUT_DATA_MODE_16_0 OUT_DATA_MODE_14 (BW ==  01)  OUT_DATA_MODE_12

uint8_t MMAt[10] ;
uint8_t mycrc[1] ;


/////////////////////////////////////////////////////////////////////////

bool mmc35240pjAcqData (void)
{
    uint8_t AMRState;

    if (MPU6050_I2C_ByteWrite(MEMSIC35240PJ_ADDRESS, &TAKE_MEASUREMENT, INTERNAL_CONTROL0) != 0)
    {
        return false;
    }
    osDelay(4);      //   14bit ??? ???? 3ms
    if (MPU6050_I2C_BufferRead(MEMSIC35240PJ_ADDRESS, &AMRState, MMC_STATUS, 1) != 0)
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
    if (MPU6050_I2C_BufferRead(MEMSIC35240PJ_ADDRESS, AMR_XYZ_Register, XOUT_LOW, 6) != 0)
    {
        return false;
    }
    return true;
}

/***************************************************************************//**
 * @brief
 *
 *
 * @param[in] i2c
 *   full
 *
 * @return
 *   full
 ******************************************************************************/

MEMSIC35240PJ_DATA *mmc35240pjGetData (void)
{
    MMC35240PJ_Result.x = (AMR_XYZ_Register[1]) << 8 | AMR_XYZ_Register[0];
    MMC35240PJ_Result.y = (AMR_XYZ_Register[3]) << 8 | AMR_XYZ_Register[2];
    MMC35240PJ_Result.z = (AMR_XYZ_Register[5]) << 8 | AMR_XYZ_Register[4];

    if(OUT_DATA_MODE_14 == ModeSet)
    {
        MMC35240PJ_Result.x >>= 2;
        MMC35240PJ_Result.y >>= 2;
        MMC35240PJ_Result.z >>= 2;
    }
    else if(OUT_DATA_MODE_12 == ModeSet)
    {
        MMC35240PJ_Result.x >>= 4;
        MMC35240PJ_Result.y >>= 4;
        MMC35240PJ_Result.z >>= 4;
    }

    return (&MMC35240PJ_Result);
}

void Mmc35240pj_Sensor(void)
{
    //  if(MMC35240PJ_SetOutData_Length())
    //  {
    if(mmc35240pjAcqData())
    {
        mmc35240pjGetData();

#if PRINT_SOURCE_DATA_BY_W
        if(OUT_DATA_MODE_16_0 == ModeSet || OUT_DATA_MODE_16_1 == ModeSet)
            UART1_sendBuffer("\r\n MMC3260PJ 16Bits DataOut ", strlen("\r\n MMC3260PJ 16Bits DataOut "));
        else if(OUT_DATA_MODE_14 == ModeSet )
            UART1_sendBuffer("\r\n MMC3260PJ 14Bits DataOut ", strlen("\r\n MMC3260PJ 14Bits DataOut "));
        else if(OUT_DATA_MODE_12 == ModeSet)
            UART1_sendBuffer("\r\n MMC3260PJ 12Bits DataOut ", strlen("\r\n MMC3260PJ 12Bits DataOut "));

        //        sprintf(strBuffer," X: %d Y: %d Z: %d",MMC35240PJ_Result.x,MMC35240PJ_Result.y,MMC35240PJ_Result.z);
        UART1_sendBuffer(strBuffer, strlen(strBuffer));
#endif


#if PRINT_SOURCE_DATA_BY_P
        MMAt[1] = ((uint16_t)MMC35240PJ_Result.x) / 256;
        MMAt[0] = ((uint16_t)MMC35240PJ_Result.x) % 256;
        MMAt[3] = ((uint16_t)MMC35240PJ_Result.y) / 256;
        MMAt[2] = ((uint16_t)MMC35240PJ_Result.y) % 256;
        MMAt[5] = ((uint16_t)MMC35240PJ_Result.z) / 256;
        MMAt[4] = ((uint16_t)MMC35240PJ_Result.z) % 256;
        MMAt[7] = ((uint16_t)0) / 256;;
        MMAt[6] = ((uint16_t)0) % 256;;
        MYCRC16(MMAt, mycrc, 8);
        MMAt[8] = mycrc[1];
        MMAt[9] = mycrc[0];
        //UART1Write(MMAt,10);
        //delay_ms(20);
        UART1_sendBuffer(MMAt, 10);
#endif
    }
    // }
}

/***************************************************************************//**
 * @brief
 *   ?6?????????????
 *
 * @param[in] i2c
 *   Pointer to I2C peripheral register block.
 *
 * @return MEMSIC35240PJ_DATA
 *   ??
 ******************************************************************************/


bool mmc35240pjAcqSet (void)
{
    uint8_t AMRState;
    osDelay(60);     //  time form Refill cap to SET/RESET
    if (MPU6050_I2C_ByteWrite(MEMSIC35240PJ_ADDRESS, &MMC_SET, INTERNAL_CONTROL0) != 0)
    {
        return false;
    }
    osDelay(1);     //    wait time to complete SET/RESET
    if (MPU6050_I2C_ByteWrite(MEMSIC35240PJ_ADDRESS, &TAKE_MEASUREMENT, INTERNAL_CONTROL0) != 0)
    {
        return false;
    }
    osDelay(4);     //   14bit ??? ???? 3ms
    if (MPU6050_I2C_BufferRead(MEMSIC35240PJ_ADDRESS, &AMRState, MMC_STATUS, 1) != 0)
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
    if (MPU6050_I2C_BufferRead(MEMSIC35240PJ_ADDRESS, AMR_XYZ_Register, XOUT_LOW , 6) != 0)
    {
        return false;
    }
    return true;
}

bool MMC35240PJ_SetReset(void)
{
    osDelay(60);
    if (MPU6050_I2C_ByteWrite(MEMSIC35240PJ_ADDRESS, &MMC_SET, INTERNAL_CONTROL0) != 0)
    {
        return false;
    }
    osDelay(60);
    if (MPU6050_I2C_ByteWrite(MEMSIC35240PJ_ADDRESS, &MMC_RESET, INTERNAL_CONTROL0) != 0)
    {
        return false;
    }
    osDelay(60);
    return true;
}

bool mmc35240pjAcqReset (void)
{
    uint8_t AMRState;
    osDelay(60);     //  time form Refill cap to SET/RESET
    if (MPU6050_I2C_ByteWrite(MEMSIC35240PJ_ADDRESS, &MMC_RESET, INTERNAL_CONTROL0) != 0)
    {
        return false;
    }
    osDelay(1);   //    wait time to complete SET/RESET
    if (MPU6050_I2C_ByteWrite( MEMSIC35240PJ_ADDRESS, &TAKE_MEASUREMENT, INTERNAL_CONTROL0) != 0)
    {
        return false;
    }

    osDelay(4);  //   14bit ??? ???? 3ms
    if (MPU6050_I2C_BufferRead(MEMSIC35240PJ_ADDRESS, &AMRState, MMC_STATUS,  1) != 0)
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
    if (MPU6050_I2C_BufferRead(MEMSIC35240PJ_ADDRESS, AMR_XYZ_Register, XOUT_LOW , 6) != 0)
    {
        return false;
    }
    return true;
}


void MMC_test()
{


    uint32_t Masetx, Masety, Masetz, Maresetx, Maresety, Maresetz;
    uint32_t Xoffset, Yoffset, Zoffset;
    uint32_t Maxz, Minz;
    uint32_t Mzo, Mzs;

    uint32_t     Mxo = 8192;
    uint32_t     Myo = 8192;
    uint32_t     Mxs = 512;
    uint32_t     Mys = 512;
    uint32_t     Maxx = 0, Minx = 0, Maxy = 0, Miny = 0;

    float   angle, xcal, ycal, Modx, Mody;

    short temp;
    MPU6050_I2C_Init();
    osDelay(100);

    MMC35240PJ_SetReset();             //  ¨¦?¦Ì? Reset/set

    while(1)
    {
   //     if(Odd_Int_flag)       //  Key 1 ?D??
  //      {

            if( mmc35240pjAcqSet())
            {
                mmc35240pjGetData();
                Masetx =   MMC35240PJ_Result.x;
                Masety =   MMC35240PJ_Result.y;
                Masetz =   MMC35240PJ_Result.z;
            }
            //   DelayNms(120);
            if(mmc35240pjAcqReset())
            {
                mmc35240pjGetData();
                Maresetx =   MMC35240PJ_Result.x;
                Maresety =   MMC35240PJ_Result.y;
                Maresetz =   MMC35240PJ_Result.z;
            }

            Xoffset = (Masetx + Maresetx) / 2;
            Yoffset = (Masety + Maresety) / 2;
            Zoffset = (Masetz + Maresetz) / 2;

            osDelay(120);

  //          Odd_Int_flag = 0;
            //       GPIO_PinModeSet(gpioPortA, 2, gpioModePushPull, 1);  /* ?D??2¨´¡Á¡Â     LED1  ?e  */
   //     }

     //   if(Even_Int_flag)      //   Key 2 ?D??
     //   {
            //       GPIO_PinModeSet(gpioPortA, 2, gpioModePushPull, 0);  /*   LED2  ¨¢¨¢  */

            Mmc35240pj_Sensor();
            Maxx = MMC35240PJ_Result.x;
            Minx = MMC35240PJ_Result.x;
            Maxy = MMC35240PJ_Result.y;
            Miny = MMC35240PJ_Result.y;
            Maxz = MMC35240PJ_Result.z;
            Minz = MMC35240PJ_Result.z;

            osDelay(30);

      //      do
      //      {
                Mmc35240pj_Sensor();
                if (MMC35240PJ_Result.x > Maxx)  Maxx = MMC35240PJ_Result.x;
                if (MMC35240PJ_Result.x < Minx)  Minx = MMC35240PJ_Result.x;
                if (MMC35240PJ_Result.y > Maxy)  Maxy = MMC35240PJ_Result.y;
                if (MMC35240PJ_Result.y < Miny)  Miny = MMC35240PJ_Result.y;
                if (MMC35240PJ_Result.z > Maxz)  Maxz = MMC35240PJ_Result.z;
                if (MMC35240PJ_Result.z < Minz)  Minz = MMC35240PJ_Result.z;

                Mxo = (Maxx + Minx) / 2;
                Myo = (Maxy + Miny) / 2;
                Mzo = (Maxz + Minz) / 2;
                Mxs = (Maxx - Minx) / 2;
                Mys = (Maxy - Miny) / 2;
                Mzs = (Maxz - Minz) / 2;

       //     }
        //    while(!Odd_Int_flag);

            //      sprintf(strBuffer,"\r\n Offset by rotate:  X: %d Y: %d Z: %d",Mxo,Myo,Mzo);
            //      memset(strBuffer,0,strlen(strBuffer));
            //     UART1_sendBuffer(strBuffer, strlen(strBuffer));
            //       sprintf(strBuffer,"\r\n Sensit by rotate:  X: %d Y: %d Z: %d",Mxs,Mys,Mzs);
            //     UART1_sendBuffer(strBuffer, strlen(strBuffer));
            //       memset(strBuffer,0,strlen(strBuffer));


         //   Even_Int_flag = 0;
            //       GPIO_PinModeSet(gpioPortA, 2, gpioModePushPull, 1);  /* ?D??2¨´¡Á¡Â     LED2  ?e  */
    //    }

        osDelay(50);
    }
}
/**
 * @}
 */ /* end of group MPU6050_Library */