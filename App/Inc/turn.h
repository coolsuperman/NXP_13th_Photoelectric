#ifndef _TURN_H
#define _TURN_H
#include"include.h"


#define TURN_FTM    FTM1   //舵机使用的FTM
#define TURN_CHX    FTM_CH0
#define TURN_FREQ   300

#define TURN_DUTY_NOM   440
#define TURN_DUTY_MIN   10
#define TURN_DUTY_MAX   900

#define TURN_PID_MAX   10000
#define TURN_PID_Min   -10000

uint16 get_precision(FTMn_e turn_ftm);
void turn_init(void);
void turn_duty(uint16 duty);

#endif