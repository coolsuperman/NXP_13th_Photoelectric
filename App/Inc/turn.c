#include"turn.h"
uint16 get_precision(FTMn_e turn_ftm)
{
  switch(turn_ftm)
  {
  case FTM0:return FTM0_PRECISON;
  case FTM1:return FTM1_PRECISON;
  case FTM2:return FTM2_PRECISON;
  
  default:ASSERT(0);
  }
  return 100;
}

void turn_init(void)
{
  ASSERT(get_precision(TURN_FTM)==1000u);
  ftm_pwm_init(TURN_FTM,TURN_CHX,TURN_FREQ,TURN_DUTY_NOM);
}

void turn_duty(uint16 duty)
{
//  ASSERT(duty>=TURN_DUTY_MIN);
//  ASSERT(duty<=TURN_DUTY_MAX);
  ftm_pwm_duty(TURN_FTM,TURN_CHX,duty);
}
