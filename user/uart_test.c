/**************************************************************
* Copyright (C) 2008-2017, Thunder Software Technology Co.,Ltd.
* All rights reserved.
****************************************************************/
#include "uart_test.h"
#include "ultra.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
//#include "udp_link.h"

#define HBINTERVAL 30

typedef struct _otsdata{
char msgID[16];
char deviceID[32];
char timestamp[32];
int rStatus;
int nStatus;//0 not connected,1 connected
int powerinfo;
char nbVersion[32];
char swVersion[16];
int sensorStatus;
int nbRSRP;
int nbSINR;
int nbTAC;
int nbPCI;
int nbECI;
int errorCode;
}otsdata;


extern double TemRet;
extern double HumRet;
extern short sTmp;
int isCreatedSocket=0;//±êê?ê?µúò?'?·?êy?Y?1ê?DY??oó·?êy?Y
int isAttachSuccess=1;//ê?·???×?3é1|
int isNeedReboot=0;//ê?·?Dèòa??????×é
int isNeedRetransmit=1;//ê?·?Dèòa??'?êy?Y
volatile uint8_t InitImeiFlag = 0;
char ImeiNumber[15];
#define MSG_1  "AT+NSOCR=DGRAM,17,%d,1\r\n"
#define MSG_2  "AT+NSOST=0,%s,%d,%d,%s\r\n"
#define IP "192.168.16.71"
#define PORT 9100
#define LOOP 5

int hbinterval;
long tfs;



/*
void encodetest()
	{
	lzo_uint slen, dlen;
	unsigned char __LZO_MMODEL src[512] = "weizabcdefg";
	unsigned char __LZO_MMODEL dst[512];

	char log[256];
	memset(log,0,sizeof(log));

	if (lzo_init() != LZO_E_OK)
	{
		snprintf(log,sizeof(log),"Init error\n");
		los_dev_uart_write(LOS_STM32L476_UART3, log, strlen(log), 100);	
		memset(log,0,sizeof(log));
	}

	if(encodestring(src, slen, dst, &dlen, wrkmem)){
		snprintf(log,sizeof(log),"Encode failed\n");
		los_dev_uart_write(LOS_STM32L476_UART3, log, strlen(log), 100);	
		memset(log,0,sizeof(log));
	}

	if(decodestring(dst, dlen, src, &slen)){
		snprintf(log,sizeof(log),"Decode failed\n");
		los_dev_uart_write(LOS_STM32L476_UART3, log, strlen(log), 100);	
		memset(log,0,sizeof(log));
	}

		snprintf(log,sizeof(log),src);
		los_dev_uart_write(LOS_STM32L476_UART3, log, strlen(log), 100);	
		memset(log,0,sizeof(log));
}*/

void FT_UART_Send()
{
	los_dev_uart_write(LOS_STM32L476_UART3, "Welcome to use ThunderSoft NB-IoT develop board!\n", 49, 100);	
	osDelay(200);
	los_dev_uart_write(LOS_STM32L476_UART3, "»¶Ó­Ê¹ÓÃNB-IoT\n",15, 100);
	osDelay(200);

	while(1)
	{
		// 
		//?-?··??í??êa?èoíNB-IoT???éIMEI?µ
		char send_buff[1024];
		sprintf(send_buff, "01.Temperature is %f, Humidity is %f\n", TemRet, HumRet);
		los_dev_uart_write( LOS_STM32L476_UART3, send_buff, strlen(send_buff), 1000);
		osDelay(200);
		memset(send_buff, 0, sizeof(send_buff));
		if(InitImeiFlag)
		{
			sprintf(send_buff, "02.NB-Iot module IMEI : %s\n", ImeiNumber);
			los_dev_uart_write( LOS_STM32L476_UART3, send_buff, strlen(send_buff), 1000);
			osDelay(200);
			memset(send_buff, 0, sizeof(send_buff));
		}
		sprintf(send_buff, "03.MPU motion sensor raw data is : %d\n", sTmp);
		los_dev_uart_write( LOS_STM32L476_UART3, send_buff, strlen(send_buff), 1000);
		osDelay(200);
		memset(send_buff, 0, sizeof(send_buff));
	}
}

int NB_Init(){
	char read_buff[128];
	char log[128];
	char cmd[7][64] = {"AT+CFUN?\r\n","AT+CFUN=1\r\n", "AT+NEARFCN=0,3700\r\n", "AT+CGDCONT=1,\"IP\",\"CMNET\"\r\n","AT+CGATT=1\r\n", "AT+NSMI=1\r\n", "AT+NNMI=2\r\n"};
	char *tmp = NULL;
		int i=0;

	for(i=0;i<10;i++)
	{
		memset(log,0,sizeof(log));
		memset(read_buff,0,sizeof(read_buff));
	los_dev_uart_write(LOS_STM32L476_UART2, cmd[0], strlen(cmd[0])+1, 1000);
	los_dev_uart_read(LOS_STM32L476_UART2, read_buff, 128, 5000);
	osDelay(200);
		snprintf(log,sizeof(log),"cmd is %s,response is %s\n",cmd[0],read_buff);
					los_dev_uart_write(LOS_STM32L476_UART3, "Welcome to use ThunderSoft NB-IoT develop board!\n", 49, 100);	
	osDelay(200);
		los_dev_uart_write(LOS_STM32L476_UART3, log, sizeof(log), 1000);
			osDelay(200);

	if(strstr(read_buff,"CFUN")!=NULL)
		break;
	}
	for(i=1;i<7;i++)
	{
		memset(log,0,sizeof(log));
		memset(read_buff,0,sizeof(read_buff));
		los_dev_uart_write(LOS_STM32L476_UART2, cmd[i], strlen(cmd[i])+1, 1000);
		los_dev_uart_read(LOS_STM32L476_UART2, read_buff, 128, 5000);
		osDelay(200);
		snprintf(log,sizeof(log),"cmd is %s,response is %s\n",cmd[i],read_buff);
		los_dev_uart_write(LOS_STM32L476_UART3, log, strlen(log)+1, 100);	
			osDelay(200);

	}
	/***************¼ì²é×¢ÍøÊÇ·ñ³É¹¦********************/
	char cmdcheck[64]="AT+CGATT?\r\n";
	for(i=0;i<10;i++)
	{
		memset(log,0,sizeof(log));
		memset(read_buff,0,sizeof(read_buff));
		los_dev_uart_write(LOS_STM32L476_UART2, cmdcheck, strlen(cmdcheck)+1, 1000);
		los_dev_uart_read(LOS_STM32L476_UART2, read_buff, 128, 5000);
				osDelay(200);
		snprintf(log,sizeof(log),"Loop number is %d,cmd is AT+CGATT?,response is %s\n",i+1,read_buff);
		los_dev_uart_write(LOS_STM32L476_UART3, log, strlen(log)+1, 100);	
			osDelay(200);

		tmp=strstr(read_buff,"CGATT:1");
		if(tmp!=NULL)
		{
			break;
		}
	}
	if(i>9)
	{
		char error[64]="Attch failed!\n";
		isAttachSuccess=0;
		los_dev_uart_write(LOS_STM32L476_UART3, error, strlen(error)+1, 100);	
			osDelay(200);

		return -1;
	}
	return 0;
}
int NB_Reboot(){
	//char read_buff[64];
	char log[128];
	char cmd[64] = "AT+NRB\r\n";
	char *tmp = NULL;
	los_dev_uart_write(LOS_STM32L476_UART2, cmd, strlen(cmd)+1, 8000);
	//los_dev_uart_read(LOS_STM32L476_UART2, read_buff, 64, 5000);
	snprintf(log,sizeof(log),"cmd is AT+NRB\n");
	los_dev_uart_write(LOS_STM32L476_UART3, log, strlen(log)+1, 100);	
	osDelay(200);

	isAttachSuccess=0;
	isCreatedSocket=0;
}

void trans(unsigned char c, char *s)
{
		char tmp[3];
		sprintf(tmp, "%X", c);
		if(tmp[1] == 0){
				tmp[1] = tmp[0];
				tmp[0] = '0';
		}
		memcpy(s, tmp, 2);
}
void trans_full(char *src, char *dst)
{
    int i, j;
    char tmp[3];

    for(i = 0, j = 0; src[i]; i ++, j += 2){
        trans(src[i], tmp);
        dst[j] = tmp[0];
        dst[j + 1] = tmp[1];
    }

    dst[j] = 0;
}

int trans_back(char *src, char *dst)
{
    int i, j;
    unsigned int c;
    char tmp[3];

    if(strlen(src) % 2 != 0){
                    return -1;
    }

    for(i = 0, j = 0; src[i]; i += 2, j ++){
        snprintf(tmp, 3, "%s", src + i);
        sscanf(tmp, "%X", &c);
        dst[j] = c;
    }

    dst[j] = '\0';

    return 0;
}

int NB_SendData(char* ip,int port,char* orginalmsg)//send and receive response data
{
	char msg[] = "AT+NSOCL=0\r\n";
	char msrc[512];
	char mdst[512];
	char read_buff[128];
	char log[128];
	char* tmp=NULL;
	if(isCreatedSocket==0)
	{
	/*************´´½¨socket****************/
	memset(read_buff,0,sizeof(read_buff));
	los_dev_uart_write(LOS_STM32L476_UART2, msg, strlen(msg)+1, 1000);
	los_dev_uart_read(LOS_STM32L476_UART2, read_buff, 128, 5000);
				osDelay(200);

	/**************ÅäÖÃ¶Ë¿Ú***************/
	snprintf(msrc, 1024, MSG_1, port);
	los_dev_uart_write(LOS_STM32L476_UART2, msrc, strlen(msrc)+1, 1000);
	los_dev_uart_read(LOS_STM32L476_UART2, read_buff, 128, 5000);
	//osDelay(200);
	/*memset(log,0,sizeof(log));
	snprintf(log,sizeof(log),"cmd is %s,response is %s",msrc,read_buff);
	los_dev_uart_write(LOS_STM32L476_UART3, log, strlen(log)+1, 100);	
	osDelay(200);*/

	tmp=strstr(read_buff,"OK");
	if(tmp==NULL)
		return -1;
	isCreatedSocket=1;
	}
	/**************±àÂë×ª»»¼°Êý¾Ý·¢ËÍ***************/
	memset(msrc,0,sizeof(msrc));
	/*memset(log,0,sizeof(log));
	snprintf(log,sizeof(log),"start trans\n");
	los_dev_uart_write(LOS_STM32L476_UART3, log, strlen(log)+1, 100);	
	osDelay(200);*/
	trans_full(orginalmsg, mdst);//×ª»»Îª16½øÖÆ
	/*memset(log,0,sizeof(log));
	snprintf(log,sizeof(log),"end trans\n");
	los_dev_uart_write(LOS_STM32L476_UART3, log, strlen(log)+1, 100);	
	osDelay(200);*/
	snprintf(msrc, sizeof(msrc), MSG_2, ip, port, strlen(orginalmsg), mdst);
	/*memset(log,0,sizeof(log));
	snprintf(log,sizeof(log),"start send data\n");
	los_dev_uart_write(LOS_STM32L476_UART3, log, strlen(log)+1, 100);	
	osDelay(200);*/
	los_dev_uart_write(LOS_STM32L476_UART2, msrc, strlen(msrc)+1, 100);
	int i=0;
	for(i=0;i<3;i++)
	{
	memset(read_buff,0,sizeof(read_buff));
	/*
	memset(log,0,sizeof(log));
	snprintf(log,sizeof(log),"start readdata\n");
	los_dev_uart_write(LOS_STM32L476_UART3, log, strlen(log)+1, 100);	
	osDelay(200);*/
	los_dev_uart_read(LOS_STM32L476_UART2, read_buff, sizeof(read_buff), 5000);
	osDelay(200);

	memset(log,0,sizeof(log));
	snprintf(log,sizeof(log),"loop count is %d ,response is %s",i+1,read_buff);
	los_dev_uart_write(LOS_STM32L476_UART3, log, strlen(log)+1, 100);	
	osDelay(200);
		
		tmp=strstr(read_buff,"NSONMI:");
		//tmp=strstr(read_buff,"OK");
		if(tmp!=NULL)
		{
			isNeedRetransmit=0;
			break;
		}
	}
	los_dev_uart_write(LOS_STM32L476_UART3, "NB send over", strlen("NB send over")+1, 100);
	osDelay(200);
	if(i>2)
	{
		isNeedRetransmit=1;
		los_dev_uart_write(LOS_STM32L476_UART3, "NB send need Retransmit", strlen("NB send need Retransmit")+1, 100);
		osDelay(200);
		return -1;
	}
	/*
	memset(log,0,sizeof(log));
	snprintf(log,sizeof(log),"send data success!\n");
	los_dev_uart_write(LOS_STM32L476_UART3, log, strlen(log)+1, 100);	
	osDelay(200);*/
	los_dev_uart_write(LOS_STM32L476_UART3, "NB send func over", strlen("NB send func over")+1, 100);
	osDelay(200);
	return 0;
}
/*
author:syl
time:2017-09-12
para:smsg,response data from server,must malloc memory before call this function

*/
int NB_ReadData(char *smsg)
{
		char msg[512];
		
		int count = 0;
		char msgg[] = "AT+NSOCL=0\r\n";
		int i;
		char *tok1, *tok2 = NULL;
    char response[512];
	  char s1[32], s2[32], s3[32], s4[32], s5[256], s6[128];
	  int res;

		memset(msg,0,sizeof(msg));
		memset(response,0,sizeof(response));
    
	  los_dev_uart_write(LOS_STM32L476_UART3, "start read response\n", strlen("start read response\n")+1, 100);
	  osDelay(200);
		snprintf(msg,sizeof(msg),"AT+NSORF=0,512\r\n");
		for(count=0;count<3;count++){
		los_dev_uart_write(LOS_STM32L476_UART2, msg, strlen(msg)+1, 100);
		osDelay(200);
		los_dev_uart_read(LOS_STM32L476_UART2, response, sizeof(response), 5000);
		osDelay(200);
    los_dev_uart_write(LOS_STM32L476_UART3, response, strlen(response)+1, 100);	
		osDelay(200);
		
					if(strstr(response, "OK")!=NULL&&strstr(response,",0")!=NULL){
						for(tok1 = response; *tok1 == ' ' || *tok1 == '\n' || *tok1 == '\t' || *tok1 == '\r'; tok1 ++);
						for(tok2 = tok1; *tok2 != '\n' && *tok2 != '\r' && *tok2 != '\t' && *tok2 != ' ' && *tok2; tok2 ++){
							if(*tok2 == ','){
								*tok2 = ' ';
							}
						}
						//los_dev_uart_write(LOS_STM32L476_UART3, "tok response\n", strlen("tok response\n")+1, 100);
						//osDelay(200);
						//res = sscanf(tok1, "%[^,]%[^,]%[^,]%[^,]%[^,]%[^,]", s1, s2, s3, s4, s5, s6);
						res = sscanf(tok1, "%s%s%s%s%s%s", s1, s2, s3, s4, s5, s6);
						//snprintf(s1, 512, "\n{%d}\n", res);
						//los_dev_uart_write(LOS_STM32L476_UART3, s1, strlen(s1)+1, 100);
						osDelay(200);
						los_dev_uart_write(LOS_STM32L476_UART3, s5, strlen(s5)+1, 100);	
						osDelay(200);
						if(res >= 5){
							trans_back(s5, smsg);
						}

/*						
						
					if((tok1 = strstr(msg, ",0")) != NULL){				
						tok1 = tok1 -1;
					for(i = 0; i < tok1 - msg -1; i ++){
						if(*(tok1 - i) == ','){
							tok2 = tok1 - i;
							break;
						}
					}

					if(tok2){
							snprintf(response, i + 1, "%s", tok2 + 1);
							trans_back(response, smsg);
					}

				}
*/
				return 0;
					}
		}

		return -1;
}
#if 1

int send_single_response_command(char *cmd, char *response, char *res)
{
	char *tok1;
	//char response[512];
	
	los_dev_uart_write(LOS_STM32L476_UART3, cmd, strlen(cmd)+1, 100);	
	osDelay(200);

	memset(response,0,512);
	los_dev_uart_write(LOS_STM32L476_UART2, cmd, strlen(cmd)+1, 1000);
	los_dev_uart_read(LOS_STM32L476_UART2, response, 512, 5000);
	osDelay(200);
	
	//memset(response,0,512);	
	los_dev_uart_write(LOS_STM32L476_UART3, response, strlen(response)+1, 100);	
	osDelay(200);
	for(tok1 = response; *tok1 == ' ' || *tok1 == '\n' || *tok1 == '\t' || *tok1 == '\r' || *tok1 == 0; tok1 ++);
	if(*tok1 == 0){
		return -1;
	}
	
	los_dev_uart_write(LOS_STM32L476_UART3, tok1, strlen(tok1)+1, 100);	
	osDelay(200);
	
	sscanf(tok1, "%s", res);
	
	return 0;
}

int send_check_response_command(char *cmd, char *response, char *check)
{
	//char response[512];
	memset(response,0,512);
	los_dev_uart_write(LOS_STM32L476_UART2, cmd, strlen(cmd)+1, 1000);
	osDelay(200);
	los_dev_uart_read(LOS_STM32L476_UART2, response, 512, 5000);
	osDelay(200);
	
	if(strstr(response, check))
		return 1;
	else
		return 0;
}

int send_nuestats_command(otsdata *d, char *response)
{
//	char response[512];
	char *tok1;
	
	memset(response,0,512);
	los_dev_uart_write(LOS_STM32L476_UART2, "AT+NUESTATS\r\n", strlen("AT+NUESTATS\r\n")+1, 100);
	osDelay(200);
	los_dev_uart_read(LOS_STM32L476_UART2, response, 512, 5000);
	osDelay(200);
	
	if((tok1 = strstr(response, "Cell ID:")) != NULL){
		d->nbECI = atoi(tok1 + 8);
	}
	
	if((tok1 = strstr(response, "SNR:")) != NULL){
		d->nbSINR = atoi(tok1 + 4);
	}
	
	if((tok1 = strstr(response, "PCI:")) != NULL){
		d->nbPCI = atoi(tok1 + 4);
	}
	
	return 0;
}


int send_cgsn_command(otsdata *d, char *response)
{
//	char response[512];
	char *tok1;
	
	memset(response,0,512);
	los_dev_uart_write(LOS_STM32L476_UART2, "AT+CGSN=1\r\n", strlen("AT+CGSN=1\r\n")+1, 100);
	osDelay(200);
	los_dev_uart_read(LOS_STM32L476_UART2, response, 512, 5000);
	osDelay(200);
	
	los_dev_uart_write(LOS_STM32L476_UART3, response, strlen(response)+1, 100);	
	osDelay(200);
	
	if((tok1 = strstr(response, "CGSN:")) != NULL){
		sscanf(tok1 + 5, "%s", d->deviceID);
	}
	
	return 0;
}

int send_nuestatscell_command(otsdata *d, char *response)
{
//	char response[512];
	char *tok1, *tok2;
	
	memset(response,0,512);
	los_dev_uart_write(LOS_STM32L476_UART2, "AT+NUESTATS=CELL\r\n", strlen("AT+NUESTATS=CELL\r\n")+1, 100);
	osDelay(200);
	los_dev_uart_read(LOS_STM32L476_UART2, response, 512, 5000);
	osDelay(200);
	
	for(tok1 = response; *tok1 == ' ' || *tok1 == '\n' || *tok1 == '\t' || *tok1 == '\r' || *tok1 == 0; tok1 ++);
	if(*tok1 == 0){
		return -1;
	}
	
	for(tok2 = tok1; *tok2 != '\n' && *tok2 != '\r' && *tok2 != '\t' && *tok2 != ' ' && *tok2; tok2 ++){
							if(*tok2 == ','){
								*tok2 = ' ';
							}
	}
	
	sscanf(tok1, "%*s%*s%*s%*s%d", &d->nbRSRP);
	return 0;
}
int init_main_msg(otsdata *d, char *response)
{
	char log[512];
	memset(d, 0, sizeof(otsdata));
  strcpy(d->timestamp, "1262275200");
	d->rStatus = 101;

	d->powerinfo = 98;
	d->sensorStatus = 1;
	strcpy(d->swVersion, "OTS_CC_1.0");
	d->errorCode = 0;
	d->nbTAC = 23;
	
	//d->msgID
	
	send_cgsn_command(d, response);
	//send_single_response_command("AT+CGSN=1\r\n", response, d->deviceID);
	//send_single_response_command("AT+CFUN?\r\n", response, d->deviceID);
	//d->timestamp
	snprintf(log, 512, "{%s,,,%s}", response, d->nbVersion);
	los_dev_uart_write(LOS_STM32L476_UART3, log, strlen(log)+1, 100);	
	osDelay(200);
	
	if(send_check_response_command("AT+CGATT?\r\n", response, "1"))
		d->nStatus = 1;
	else
		d->nStatus = 0;

	send_single_response_command("AT+CGMR\r\n", response, d->nbVersion);
	
	los_dev_uart_write(LOS_STM32L476_UART3, d->nbVersion, strlen(d->nbVersion)+1, 100);	
		osDelay(200);
	send_nuestats_command(d, response);
	send_nuestatscell_command(d, response);
	//d->nbRSRP= ;
	//d->nbSINR= ;
	//d->nbTAC = ;
	//d->nbPCI = ;
	//d->nbECI = ;
	
	return 0;
}
#endif
void NB_TEST_Uart()
{
	int i = 88;
	otsdata mdata;
	char read_buff[512];
	char log[64];
	char *tmp = NULL;
	int transmitCount=0;
	long times, timef;
//	NB_Init();
	char testdata[512];//msg need to be sent
	init_main_msg(&mdata, read_buff);
	
	while(1)//×'ì??ú?à?-?·
	{
	snprintf(read_buff, 512, "%08d,%s,%s,%d,%d,%d,%s,%s,%d,%d,%d,%d,%d,%d,%d", i++, mdata.deviceID, mdata.timestamp, mdata.rStatus, mdata.nStatus, mdata.powerinfo, mdata.nbVersion, mdata.swVersion, mdata.sensorStatus, mdata.nbRSRP, mdata.nbSINR, mdata.nbTAC, mdata.nbPCI, mdata.nbECI, mdata.errorCode);
	los_dev_uart_write(LOS_STM32L476_UART3, read_buff, strlen(read_buff)+1, 100);	
	osDelay(200);
	memset(log,0,sizeof(log));
	snprintf(log,sizeof(log),"start loop\n");
	los_dev_uart_write(LOS_STM32L476_UART3, log, strlen(log)+1, 100);	
	osDelay(200);
		long* ts;
		long* tr;
		
	//ÔÝÊ±ÊÖ¹¤ÖÃÎ»
	isNeedRetransmit=1;
	if(isNeedReboot==1)
			NB_Reboot();
	if(isAttachSuccess==0)
		NB_Init();
	
	if(isNeedRetransmit==1)
	{
	
		/***************init array*********************/
		ts=(long*)malloc(LOOP*4);
		tr=(long*)malloc(LOOP*4);
		/*int count=0;
		for(count=0;count<LOOP;count++)
	{
		memset(testdata,0,sizeof(testdata));
		snprintf(testdata, sizeof(testdata), "%0*d", 32, count);
		int res;
		ts[count]=HAL_GetTick();
		res=NB_SendData(IP,PORT,testdata);
		tr[count]=HAL_GetTick();
		
		if(res==0)
		{
			memset(log,0,sizeof(log));
			snprintf(log,sizeof(log),"send data success\n");
			los_dev_uart_write(LOS_STM32L476_UART3, log, strlen(log)+1, 100);	
			osDelay(200);
		}
		else
		{
			memset(log,0,sizeof(log));
			snprintf(log,sizeof(log),"send data failed\n");
			los_dev_uart_write(LOS_STM32L476_UART3, log, strlen(log)+1, 100);	
			osDelay(200);
		}
		
		
		memset(read_buff,0,sizeof(read_buff));
		res=NB_ReadData(read_buff);
		osDelay(200);//¶ÁÊý¾ÝÖ®ºó¼Ó¸ö0.2sµÄÊ±ÑÓ
		if(res==0)
		{
			memset(log,0,sizeof(log));
			snprintf(log,sizeof(log),"read data success,response is %s\n",read_buff);
			los_dev_uart_write(LOS_STM32L476_UART3, log, strlen(log)+1, 100);	
			osDelay(200);
		}
		else
		{
			memset(log,0,sizeof(log));
			snprintf(log,sizeof(log),"read data failed\n");
			los_dev_uart_write(LOS_STM32L476_UART3, log, strlen(log)+1, 100);	
			osDelay(200);
		}
		
		memset(log,0,sizeof(log));
		snprintf(log,sizeof(log),"data index is %d, send time is %ld,recv time is %ld\n",count,ts[count],tr[count]);
		los_dev_uart_write(LOS_STM32L476_UART3, log, strlen(log)+1, 100);	
		osDelay(200);
	}
		
	
		long maxRTT=5000,minRTT=0;
	double avgRTT=0.0;
	//while(link!=NULL)
	for(count=0;count<LOOP;count++)
	{
	if(maxRTT<tr[count]-ts[count])
		maxRTT=tr[count]-ts[count];
	if(minRTT>tr[count]-ts[count])
		minRTT=tr[count]-ts[count];
		avgRTT+=tr[count]-ts[count]+0.0;
	memset(log,0,sizeof(log));
	snprintf(log,sizeof(log),"RTT index is %d, RTT is %ld\n",count,tr[count]-ts[count]);
	los_dev_uart_write(LOS_STM32L476_UART3, log, strlen(log)+1, 100);	
		osDelay(200);
	}
	
	memset(log,0,sizeof(log));
	snprintf(log,sizeof(log),"maxRTT is %ld, minRTT is %ld,avgRTT is %.2lf\n",maxRTT,minRTT,avgRTT/LOOP);
	los_dev_uart_write(LOS_STM32L476_UART3, log, strlen(log)+1, 100);	
	osDelay(200);
	*/
	
	
	
	  //times = HAL_GetTick(); 
		//NB_SendData(IP,PORT,"00000002,8090100033,1503645090,102,1,97,V100R100C10B657SP1,RGIoT OS version 1.0,0,-877,181,0001,20,12198912,0");
int res;
    NB_SendData(IP,PORT, read_buff);
		res=NB_ReadData(read_buff);
		//timef = HAL_GetTick();
		osDelay(200);//¶ÁÊý¾ÝÖ®ºó¼Ó¸ö0.2sµÄÊ±ÑÓ
		if(res==0)
		{
			memset(log,0,sizeof(log));
			snprintf(log,sizeof(log),"read data success,response is %s\n",read_buff);
			los_dev_uart_write(LOS_STM32L476_UART3, log, strlen(log)+1, 100);	
			osDelay(200);
		}
		else
		{
			memset(log,0,sizeof(log));
			snprintf(log,sizeof(log),"read data failed\n");         
			los_dev_uart_write(LOS_STM32L476_UART3, log, strlen(log)+1, 100);	
			osDelay(200);
		}
		




		/*if(transmitCount<3)//ÖØÆôÐ£Ñé£¬ÖØ´«³¬¹ýÈý´Î¾ÍÖØÆôÄ£×é
			transmitCount++;
		else
			isNeedReboot=1;*/
	}
	memset(read_buff,0,sizeof(read_buff));
	if(ts!=NULL)
		free(ts);
	if(tr!=NULL)
		free(tr);
	}
	/*??è?NB-IoT???éµ?IMEIo?*/
	/*if(!InitImeiFlag)
	{
		los_dev_uart_write(LOS_STM32L476_UART2, "AT+CGSN=1\r\n", 11, 100);
		los_dev_uart_read(LOS_STM32L476_UART2, read_buff, 64, 5000);
		osDelay(200);
			
		tmp = strstr(read_buff, "CGSN");
		if(tmp!=NULL)
		{
			ImeiNumber[0] = read_buff[8];
			ImeiNumber[1] = read_buff[9];
			ImeiNumber[2] = read_buff[10];
			ImeiNumber[3] = read_buff[11];
			ImeiNumber[4] = read_buff[12];
			ImeiNumber[5] = read_buff[13];
			ImeiNumber[6] = read_buff[14];
			ImeiNumber[7] = read_buff[15];
			ImeiNumber[8] = read_buff[16];
			ImeiNumber[9] = read_buff[17];
			ImeiNumber[10] = read_buff[18];
			ImeiNumber[11] = read_buff[19];
			ImeiNumber[12] = read_buff[20];
			ImeiNumber[13] = read_buff[21];
			ImeiNumber[14] = read_buff[22];
			InitImeiFlag = 1;
		}
	}*/
}
void NB_TEST(){
NB_Init();
	NB_TEST_Uart_once(101);
}

int parse_nb_response(char *res)
{
	int i, j, start = 0;
	char *tok, *tok1;
	char s1[32], s2[32];
	int ret;
	
	for(tok1 = res; *tok1 == ' ' || *tok1 == '\n' || *tok1 == '\t' || *tok1 == '\r' || *tok1 == 0; tok1 ++);
	los_dev_uart_write(LOS_STM32L476_UART3, tok1, strlen(tok1)+1, 100);	
	osDelay(200);
	
	for(i = 0, j = 0; tok1[i]; i ++){
		if(tok1[i] == ','){
			if(j % 2 == 0){
				tok1[i] = ' ';
			}
			j ++;
		}
		
	}
	
	los_dev_uart_write(LOS_STM32L476_UART3, tok1, strlen(tok1)+1, 100);	
	osDelay(200);
	while(1){
		if(!start){
			tok = strtok(tok1, ",");
			start = 1;
		}
		else{
			tok = strtok(NULL, ",");
		}
		
		if(tok == NULL){
			break;
		}
		
		if(sscanf(tok, "%s%s", s1, s2) != 2){
			continue;
		}
		
		los_dev_uart_write(LOS_STM32L476_UART3, s1, strlen(s1)+1, 100);	
		osDelay(200);
		los_dev_uart_write(LOS_STM32L476_UART3, s2, strlen(s2)+1, 100);	
		osDelay(200);
		
		switch(atoi(s1)){
			case 200:
				tfs = atol(s2);
				break;
			case 700:
				hbinterval = atoi(s2);
				break;
			case 80:
				debugflag = 1;
				break;
			case 81:
				debugflag = 0;
				break;
			default:
				break;
		}
	}
}
void NB_TEST_Uart_once(int status)
{
	static int i = 1;
	int count = 0;
	otsdata mdata;
	char read_buff[512];
	char log[512];
	char *tmp = NULL;
	int transmitCount=0;
	long times = 0, timef = 0;
	int c_status;
	NB_Init();
	char testdata[512];//msg need to be sent
	memset(read_buff,0,sizeof(read_buff));
	char msg[32] = "AT+CGSN=1\r\n";
	int retransmit_count = 1, flag_reboot= 0;
	//memset(msg,0,sizeof(msg));
	//snprintf(msg,strlen(""));
	//los_dev_uart_write(LOS_STM32L476_UART2, msg, strlen(msg)+1, 1000);
	//los_dev_uart_read(LOS_STM32L476_UART2, read_buff, 128, 5000);
	//osDelay(200);
	
	//los_dev_uart_write(LOS_STM32L476_UART3, read_buff, strlen(read_buff)+1, 100);	
	//osDelay(200);

	init_main_msg(&mdata, read_buff);
	c_status = sensor_status;
	hbinterval = HBINTERVAL;
while(1){
	if((c_status == sensor_status && count++ % hbinterval != 0) && retransmit_count == 0){
		osDelay(1000);
		continue;
	}
	count = 1;
	c_status = sensor_status;
	//snprintf(read_buff, 512, "%08d,8090100333,1503645090,%d,1,97,V100R100C10B657SP1,RGIoT OS version 1.0,0,-877,181,0001,20,12198912,0", i++, c_status);
	times = HAL_GetTick();
	
	if(times && timef){
		snprintf(mdata.timestamp, 16, "%ld", tfs + (times - timef) / 1000);
	}
	snprintf(read_buff, 512, "%08d,%s,%s,%d,%d,%d,%s,%s,%d,%d,%d,%d,%d,%d,%d", i, mdata.deviceID, mdata.timestamp, c_status, mdata.nStatus, mdata.powerinfo, mdata.nbVersion, mdata.swVersion, mdata.sensorStatus, mdata.nbRSRP, mdata.nbSINR, mdata.nbTAC, mdata.nbPCI, mdata.nbECI, mdata.errorCode);
	memset(log,0,sizeof(log));
	snprintf(log,sizeof(log),"\n{%s}\n",read_buff);
	los_dev_uart_write(LOS_STM32L476_UART3, log, strlen(log)+1, 100);	
	osDelay(200);
	memset(log,0,sizeof(log));
	snprintf(log,sizeof(log),"start loop\n");
	los_dev_uart_write(LOS_STM32L476_UART3, log, strlen(log)+1, 100);	
		osDelay(200);
		long* ts;
		long* tr;
	//ÔÝÊ±ÊÖ¹¤ÖÃÎ»
	isNeedRetransmit=1;
	if(isNeedReboot==1 || flag_reboot){
			flag_reboot = 0;
			memset(log,0,sizeof(log));
			snprintf(log,sizeof(log),"start reboot..\n");         
			los_dev_uart_write(LOS_STM32L476_UART3, log, strlen(log)+1, 100);	
			NB_Reboot(); 
	}
	if(isAttachSuccess==0)
		NB_Init();
	
	if(isNeedRetransmit==1)
	{
	
		/***************init array*********************/
		ts=(long*)malloc(LOOP*4);
		tr=(long*)malloc(LOOP*4);

		//NB_SendData(IP,PORT,"00000002,8090100033,1503645090,102,1,97,V100R100C10B657SP1,RGIoT OS version 1.0,0,-877,181,0001,20,12198912,0");
		int res;
    if(NB_SendData(IP,PORT, read_buff) == -1){
			retransmit_count ++;
			memset(log,0,sizeof(log));
			snprintf(log,sizeof(log),"write data failed, retransmit_count: %d\n", retransmit_count);         
			los_dev_uart_write(LOS_STM32L476_UART3, log, strlen(log)+1, 100);	
			osDelay(200);
			
			if(retransmit_count >= 3){
				flag_reboot = 1;
				retransmit_count = 0;
			}
			continue;
		}
		res=NB_ReadData(read_buff);
		timef = HAL_GetTick();
		osDelay(200);//¶ÁÊý¾ÝÖ®ºó¼Ó¸ö0.2sµÄÊ±ÑÓ
		if(res==0)
		{
			retransmit_count = 0;
			memset(log,0,sizeof(log));
			snprintf(log,sizeof(log),"\nread data success,retransmit_count: %d, response is %s\n",retransmit_count, read_buff);
			los_dev_uart_write(LOS_STM32L476_UART3, log, strlen(log)+1, 100);	
			osDelay(200);
			i ++;
			parse_nb_response(read_buff);
		}
		else
		{
			retransmit_count ++;
			if(retransmit_count >= 3){
				retransmit_count = 0;
				flag_reboot = 1;
			}
			memset(log,0,sizeof(log));
			snprintf(log,sizeof(log),"read data failed, retransmit_count: %d\n", retransmit_count);         
			los_dev_uart_write(LOS_STM32L476_UART3, log, strlen(log)+1, 100);	
			osDelay(200);
		}
	
	memset(read_buff,0,sizeof(read_buff));
	if(ts!=NULL)
		free(ts);
	if(tr!=NULL)
		free(tr);
	}
	
	}
}