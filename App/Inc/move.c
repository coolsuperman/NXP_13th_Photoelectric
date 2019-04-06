#include"move.h"
void move_init(void)
{
     ftm_pwm_init(MOVE1_FTM,MOVE1_CHX,MOVE_FREQ,0);
     ftm_pwm_init(MOVE1_FTM,MOVE2_CHX,MOVE_FREQ,0);
     ftm_pwm_init(MOVE1_FTM,MOVE3_CHX,MOVE_FREQ,0);
     ftm_pwm_init(MOVE1_FTM,MOVE4_CHX,MOVE_FREQ,0);
     ftm_enable_complementary(MOVE1_FTM);
}

void movel(int16 duty)
{    uint16_t tmp2 ;
     uint16_t tmp;
     ASSERT(duty<=1000);
     ASSERT(duty>=-1000);
     tmp=duty;
     tmp2=(duty+1000)/2 ;
     if(duty>=0)
     {
       move_forwardl(tmp);
     }
     else
     {
       move_backl(tmp2);
     }
}

void mover(int8 duty)
{
     uint8_t tmp;
     ASSERT(duty<=100);
     ASSERT(duty>=-100);
     tmp=(duty+100)/2;
     if(tmp>=50)
     {
       move_forwardr(tmp);
     }
     else
     {
       move_backr(tmp);
     }
}

void ftm_enable_complementary(FTMn_e ftmn)
{
     FTM_COMBINE_REG(FTMN[ftmn])&=~(FTM_COMBINE_DECAPEN0_MASK | FTM_COMBINE_DECAPEN1_MASK);
     FTM_COMBINE_REG(FTMN[ftmn]) |= FTM_COMBINE_COMP0_MASK | FTM_COMBINE_COMP1_MASK;
     FTM_QDCTRL_REG(FTMN[ftmn]) &= ~FTM_QDCTRL_QUADEN_MASK;
}

void move_forwardl(int16 duty)
{
    // ASSERT(duty>50);
     ftm_pwm_duty(MOVE1_FTM,MOVE1_CHX,duty);
}

void move_backl(uint8 duty)
{
     //ASSERT(duty<=50);
     ftm_pwm_duty(MOVE1_FTM,MOVE2_CHX,duty);
}

void move_forwardr(uint8 duty)
{
     ASSERT(duty>50);
     ftm_pwm_duty(MOVE1_FTM,MOVE1_CHX,duty);
}

void move_backr(uint8 duty)
{
     ASSERT(duty<=50);
     ftm_pwm_duty(MOVE1_FTM,MOVE4_CHX,duty);
}
       