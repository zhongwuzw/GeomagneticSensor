#ifndef __ULTRA_H__
#define __ULTRA_H__
extern float distanceInCM;
extern int sensor_status;
extern int debugflag;
void ultra_test();
int GetEchoSignal(void);
static void MX_GPIO_ULTRA_Init(void);
#endif