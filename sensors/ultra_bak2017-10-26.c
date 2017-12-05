#include "ultra.h"
#include "sysTick.h"
#include "gpio_handle.h"
#include "stm32l4xx_hal.h"
#include "delay.h"
#include "los_dev_st_uart.h"
#include "los_dev_st_spi.h"

extern float ElaspTime;
float distanceInCM;
int interruptflag=0;
int echoresponse=0;
	uint32_t  tickstart = 0;
	uint32_t  tickstop = 0;

//GPIO使能
static void MX_GPIO_ULTRA_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct;
  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
	
	/*Configure GPIO pins : PD10(Input)  */
  GPIO_InitStruct.Pin = GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;//GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);
	/*Configure GPIO pins : PD11(Output) */
	GPIO_InitStruct.Pin = GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_PULLDOWN;
	GPIO_InitStruct.Speed=GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);
	
	
  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
/*EXTI_InitTypeDef EXTI_InitStructure;

EXTI_ClearITPendingBit(EXTI_Line4);
EXTI_ClearITPendingBit(EXTI_Line8);

EXTI_InitStructure.EXTI_Line = EXTI_Line4 | EXTI_Line8 ; 
EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt; 
EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
EXTI_InitStructure.EXTI_LineCmd = ENABLE;                                          
EXTI_Init(&EXTI_InitStructure);*/

}

/*UltraSonic interrupt callback

*/
void EXTI15_10_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI0_IRQn 0 */

  /* USER CODE END EXTI0_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_10);
  /* USER CODE BEGIN EXTI0_IRQn 1 */

  /* USER CODE END EXTI0_IRQn 1 */
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	int count=0;
	int i=0;
	for(i=0;i<200;i++)
		{
			//filt pulse,check if it is real rising or falling
		}
	int powerlevel=0;

	//float distanceInCM;
		/*char msg[64];
		snprintf(msg,sizeof(msg),"enter interrupt callback function\n");
		los_dev_uart_write(LOS_STM32L476_UART3, msg, strlen(msg), 100);	
		osDelay(1000);*/
	if(HAL_GPIO_ReadPin(GPIOD,GPIO_PIN_10))
	{
		if(interruptflag==0)
		{
			tickstart=HAL_GetTick();
			interruptflag=1;
		}
	}
	else if(interruptflag==1)
	{
		tickstop=HAL_GetTick();
		interruptflag=0;
		echoresponse=1;
	}

				//distanceInCM = (tickstop-tickstart)/58.0;
//				return tickstop-tickstart;
			
}

/*get tramsmit time from send wave until recv wave
author:syl
date:2017-10-25
*/
int GetEchoSignal(void)
{
	int count=0;
	int powerlevel=0;
	uint32_t  tickstart = 0;
	uint32_t  tickstop = 0;
	float distanceInCM;
	//校验是否出现高电平
    while(!HAL_GPIO_ReadPin(GPIOD,GPIO_PIN_10))
    {
			count++;
			if(count>5000)
				return -1;
			while(1){
				if(!HAL_GPIO_ReadPin(GPIOD,GPIO_PIN_10))
					continue;
				else{
					tickstart=HAL_GetTick();
					while(HAL_GPIO_ReadPin(GPIOD,GPIO_PIN_10)){
					continue;
					}
					tickstop=HAL_GetTick();
				}
				
		char msg[64];
		snprintf(msg,sizeof(msg),"start time is %d,stop time is %d\n",tickstart,tickstop);
		los_dev_uart_write(LOS_STM32L476_UART3, msg, strlen(msg), 100);	
		osDelay(100);
				//distanceInCM = (tickstop-tickstart)/58.0;
				return tickstop-tickstart;
			}
		}
}

/*
return distance data
*/
float getdistance(){

}

/*measure distance from ultraSonic
author:syl
date:2017-10-25
*/
void ultra_test()
{
	MX_GPIO_ULTRA_Init();
	osDelay(1000);
	/*LED0_OUT();
	osDelay(500);
	LED1_OUT();
	
	
	osDelay(500);*/
	//float distanceInCM;


	while(1)
	{
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_11, GPIO_PIN_RESET);
		osDelay(1000);
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_11, GPIO_PIN_SET);

		osDelay(1000);

		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_11, GPIO_PIN_RESET);
		while(echoresponse==0){
			osDelay(1);
			continue;
		}
		echoresponse=0;
		char msg[64];
		distanceInCM=(tickstop-tickstart)*17.7;//time*344*100/1000/2
		snprintf(msg,sizeof(msg),"start time is %d,stop time is %d,distance is %.2f\n",tickstart,tickstop,distanceInCM);
		los_dev_uart_write(LOS_STM32L476_UART3, msg, strlen(msg), 500);	
		osDelay(1000);
		
		/*tickstart=HAL_GetTick();
		totaltime=GetEchoSignal();
		distanceInCM = totaltime*34.4/2;

		char msg[64];
		snprintf(msg,sizeof(msg),"start time is %d,stop time is %d,distance is %.2f\n",tickstart,tickstop,distanceInCM);
		los_dev_uart_write(LOS_STM32L476_UART3, msg, strlen(msg), 100);	
		osDelay(1000);
		*/

		//osDelay(1000);
		//toc();
		/*uint32_t time=HAL_GetTick()-tickstart;
		//printf("ElaspTime is %.2f\n",ElaspTime);
		
    distanceInCM = time/58.0;
    //distanceInCM =(distanceInCM*100.0)/100.0;
		char msg[64];
		snprintf(msg,sizeof(msg),"ultra sensor get distance is %.2f\n",distanceInCM);
		los_dev_uart_write(LOS_STM32L476_UART3, msg, strlen(msg), 100);	
		osDelay(1000);*/

	}
}