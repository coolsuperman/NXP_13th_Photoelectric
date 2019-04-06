
# include "PID.h"

/**********************定义*******************************/

typedef struct PID
{
 int SetPoint; //设定目标 Desired Value
 double Proportion; //比例常数 Proportional Const
 double Integral; //积分常数 Integral Const
 double Derivative; //微分常数 Derivative Const
 int LastError; //Error[-1]
 int PrevError; //Error[-2]
} PID;

static PID sPID;
static PID *sptr = &sPID; 


/**********************函数*******************************/

void IncPIDInit(void)
{
   sptr->LastError = 0; //Error[-1]
   sptr->PrevError = 0; //Error[-2]
   sptr->Proportion = P_DATA; //比例常数 Proportional Const
   sptr->Integral = I_DATA; //积分常数 Integral Const
   sptr->Derivative = D_DATA; //微分常数 Derivative Const
   sptr->SetPoint =100; //目标是 100
}

int g_CurrentVelocity;
int g_Flag;
//*****************************************************
//增量式 PID 控制设计
//*****************************************************

int IncPIDCalc(int NextPoint)
{
   int iError, iIncpid; //当前误差
   iError = sptr->SetPoint - NextPoint; //增量计算
   iIncpid = (int)(sptr->Proportion * iError - sptr->Integral * sptr->LastError + sptr->Derivative * sptr->PrevError);
   sptr->PrevError = sptr->LastError; //存储误差，用于下次计算
   sptr->LastError = iError;
   return(iIncpid); //返回增量值
   }

