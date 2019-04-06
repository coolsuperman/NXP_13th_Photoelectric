#ifndef __MOVE_H
#define __MOVE_H
#include "include.h"
#define MOVE1_FTM    FTM0
//#define MOVE2_FTM  FTM3
#define MOVE_FREQ     10000

#define MOVE1_CHX   FTM_CH0
#define MOVE2_CHX   FTM_CH2
#define MOVE3_CHX   FTM_CH1
#define MOVE4_CHX   FTM_CH3

void move_init(void);
void movel(int16 duty);
void mover(int8 duty);
void ftm_enable_complementary(FTMn_e ftmn);
void move_forwardl(int16 duty);
void move_backl(uint8 duty);
void move_forwardr(uint8 duty);
void move_backr(uint8 duty);

# endif