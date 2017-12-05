#include "ultra.h"
#include "sysTick.h"
#include "gpio_handle.h"
#include "stm32l4xx_hal.h"
#include "delay.h"
#include "los_dev_st_uart.h"
#include "los_dev_st_spi.h"
#include "uart_test.h"
//#define HEARTBEATINTERVAL 30//time interval to send heartbeat message
#define CHECKINTERVAL 5 //time interval to send ultrasonic wave
#define WAITTIME 1 //if first status change is detected, this value is for other 4 times ultracheck interval(little than CHECKINTERVAL)


#define THRESHOLD 50.0//threshold to judge if there is a car
extern float ElaspTime;
float distanceInCM;
int interruptflag=0;
int echoresponse=0;
int sensor_status=102;
uint32_t  tickstart = 0;
uint32_t  tickstop = 0;
int debugflag = 1;

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
	/*Configure GPIO pins : PD12(Output) used to supply power */
	GPIO_InitStruct.Pin = GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_PULLDOWN;
	GPIO_InitStruct.Speed=GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);	
	
  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

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
			//filt burst,check if it is real rising or falling
		}
	int powerlevel=0;

	if(HAL_GPIO_ReadPin(GPIOD,GPIO_PIN_10))
	{
		if(interruptflag==0&&echoresponse==0)
		{
			tickstart=HAL_GetTick();
			interruptflag=1;
		}
	}
	else if(interruptflag==1&&echoresponse==0)
	{
		tickstop=HAL_GetTick();
		interruptflag=0;
		echoresponse=1;
	}
			
}

/*get tramsmit time from send wave until recv wave
author:syl
para:none
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
		if(debugflag){
			snprintf(msg,sizeof(msg),"start time is %d,stop time is %d\n",tickstart,tickstop);
			los_dev_uart_write(LOS_STM32L476_UART3, msg, strlen(msg), 100);	
		
			osDelay(100);
		}
				//distanceInCM = (tickstop-tickstart)/58.0;
				return tickstop-tickstart;
			}
		}
}
/*
send trig high and low level signal
author:syl
para:none
date:2017-10-26
*/

void sendTrigSignal(){
	//power on
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_SET);
		//osDelay(1);
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_11, GPIO_PIN_RESET);
		//osDelay(100);
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_11, GPIO_PIN_SET);

		osDelay(1);

		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_11, GPIO_PIN_RESET);
	//power off
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_RESET);

}

/*
return distance data
*/
float getdistance(){
	
		int count=0;
		echoresponse=0;
		sendTrigSignal();

		while(echoresponse==0){
			count++;
			if(count>2000)
				return -1.0;//error
			osDelay(1);
			continue;
		}
		char msg[64];
		distanceInCM=(tickstop-tickstart)*17.7;//time*344*100/1000/2
		if(debugflag){
			snprintf(msg,sizeof(msg),"start time is %d,stop time is %d,distance is %.2f\n",tickstart,tickstop,distanceInCM);
			los_dev_uart_write(LOS_STM32L476_UART3, msg, strlen(msg), 500);	
			osDelay(1000);
		}
		return distanceInCM;
}
/*
log print to serial function
author:syl
para:msg,string need to be printed
date:2017-10-25
*/
void otslog(char* msg){
	  if(debugflag){
			los_dev_uart_write(LOS_STM32L476_UART3, msg, strlen(msg), 500);	
			osDelay(1000);
		}
}

/*measure distance from ultraSonic
author:syl
para:none
date:2017-10-25
*/
void ultra_test()
{
	MX_GPIO_ULTRA_Init();
	osDelay(1000);

	int loopcount=0;
	float distance[4];
	float currentdistance=-1.0;
	int i=0;
	int count=0;
	int isused=0;
	int ischanged=0;

	//NB_Init();

	while(1)
	{
/*if(loopcount%(1000*HEARTBEATINTERVAL/CHECKINTERVAL)==0){
	//TODO:need to add send data function call
	otslog("send heartbeat message!\n");
	NB_TEST_Uart_once(sensor_status);

	}*/
			if(sensor_status != 103){//first make sure it is not booked status
				currentdistance=getdistance();
				//judge if status is changed
				if(currentdistance<THRESHOLD&&sensor_status !=101)//????????????????
				{
					ischanged=1;
					int j=0;
					for(j=0;j<sizeof(distance)/sizeof(distance[0]);j++){
						distance[j]=getdistance();
						if(distance[j]>THRESHOLD)
						{   //resource release
							ischanged=0;
							break;
						}
            loopcount++;
						osDelay(WAITTIME*1000);
					}
					if(ischanged==1&&isused==0)//?????????
					{
						isused=1;
						sensor_status = 101;
					//TODO:send message and check response
						otslog("sensor_status change to 101,report to ots cloud now!\n");
			//			NB_TEST_Uart_once(sensor_status);

					}
				}else if(currentdistance>THRESHOLD&&sensor_status !=102){

					ischanged=1;
					int j=0;
					for(j=0;j<sizeof(distance)/sizeof(distance[0]);j++){
						distance[j]=getdistance();
						if(distance[j]<THRESHOLD)
						{
							ischanged=0;
							break;
						}
						loopcount++;
					osDelay(CHECKINTERVAL*1000);
					}
					if(ischanged==1&&isused==1)//?????????
					{
						isused=0;
						sensor_status = 102;
					//TODO:send message and check response
						otslog("sensor_status change to 102,report to ots cloud now!\n");
				//		NB_TEST_Uart_once(sensor_status);
					}
				}else{
					//????sleep 1s??
					loopcount++;
					osDelay(CHECKINTERVAL*1000);
				}
				//}
		}else
		{
			currentdistance=getdistance();
			//if(currentdistance<THRESHOLD&&isused==0)//????????????????
			if(currentdistance<THRESHOLD&&sensor_status != 101)//????????????????
			{
				ischanged=1;
				int j=0;
				for(j=0;j<sizeof(distance)/sizeof(distance[0]);j++){
					distance[j]=getdistance();
					if(distance[j]>THRESHOLD)
					{   //resource release
						ischanged=0;
						break;
					}
          loopcount++;
					osDelay(WAITTIME*1000);
				}
				if(ischanged==1&&isused==0)//?????????
				{
					isused=1;
					sensor_status = 101;
					//TODO:send message and check response
					otslog("sensor_status change to 101,report to ots cloud now!\n");
					//NB_TEST_Uart_once(sensor_status);

				}else
				{
				}
			}else{
        loopcount++;
				osDelay(CHECKINTERVAL*1000);
			}
		}
		
	}
}