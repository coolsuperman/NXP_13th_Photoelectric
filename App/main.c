/*!
 *     COPYRIGHT NOTICE
 *     Copyright (c) 2013,山外科技
 *     All rights reserved.
 *     技术讨论：山外论坛 http://www.vcan123.com
 *
 *     除注明出处外，以下所有内容版权均属山外科技所有，未经允许，不得用于商业用途，
 *     修改内容时必须保留山外科技的版权声明。
 *
 * @file       main.c
 * @brief      山外K60 平台主程序
 * @author     山外科技
 * @version    v5.0
 * @date       2013-08-28
 */

#include "common.h"
#include "include.h"
#include "oled.h"
#include "stdint.h"
#include "turn.h"
#include "move.h"

/**********************宏定义*******************************/
//#define P_D               3
//#define I_D               0.5
//#define D_D               5
#define D_J                2
#define	IMG_BLACK		0x00
#define	IMG_WHITE		0xff
#define FIND_CENTER		0
#define FIND_LEFT		1
#define FIND_RIGHT		2
#define CENTER_POINT	        CAMERA_W/2
#define POINT_COUNT             10 
#define FIND_COUNT              3
#define mid0_1                  7
#define mid0_2                  3
#define mid0_3                  5
#define mid1_1                  17
#define mid1_2                  13
#define mid1_3                  15
#define mid2_1                  27
#define mid2_2                  23
#define mid2_3                  25
#define mid3_1                  30
#define mid3_2                  31
#define mid3_3                  32
#define mid4_1                  40
#define mid4_2                  42
#define mid4_3                  44
#define mid5_1                  50
#define mid5_2                  52
#define mid5_3                  54
#define left_Round              1
#define right_Round             2
#define N                       12
#define left_Roundabout          1//判断是左环岛还是右环岛的标志位
#define right_Roundabout         2

#define Roundabout_Pre           3 //前方环岛标志位
#define Roundabout_GoInto        4//进入环岛标志位            
#define Roundabout_In            5 //在环岛中标志位
#define Roundabout_PreOut        6
#define Roundabout_Out           7
#define Roundabout_After         8
/*********************变量定义****************************/
int16 mid3_av;
int16 mid2_av;
int16 mid0_av;
int16 Base_line;
double P_D;
double I_D;
double D_D;
int cross= 0 ;
int16 piancha_old= 0 ;
int16 piancha=0;
int16 slopl[59]={0};
int16 slopr[59]={0};
int16 leftBound[CAMERA_H]={0};
int16 leftBoundlast=0;
int16 leftLine_last=0;
int16 rightLine_last=0;
int16 rightBound[CAMERA_H]={0};
int16 rightBoundlast=0;
int16 centerLine[CAMERA_H+1] = {0};			// 最后一个元素用来记录转向点对应的行数  (60+1)
int16 leftLine[CAMERA_H] = {0};                         //记录左边线行数
int16 rightLine[CAMERA_H] = {0};                        //记录右边线行数
uint8 imgbuff[CAMERA_SIZE];                             //定义存储接收图像的数组
uint8 img[CAMERA_H][CAMERA_W];
uint8 front_left[1][3];                                 //选取行12,13,14，列1 ，3 ，5 作为左前标志区域
uint8 front_mid_l[1][3];                                 //选取行12,13,14，列1 ，3 ，5 作为左前标志区域
uint8 front_mid_r[1][3];                                  //选取行12,13,14，列38，40，42作为中前标志区域
uint8 front_right[1][3];                                //选取行12,13,14，列74，76，78作为右前标志区域
uint8 mid_left[1][3];                                   //选取行27,28,29，列1 ，3 ，5 作为左中标志区域
uint8 mid_mid[1][3];                                    //选取行27,28,29，列38，40，42作为中中标志区域
uint8 mid_right[1][3];                                  //选取行27,28,29，列74，76，78作为右中标志区域
uint8 back_left[1][3];                                  //选取行55,56,57，列1 ，3 ，5 作为左后标志区域
uint8 back_mid[1][3];                                   //选取行55,56,57，列38，40，42作为中后标志区域
uint8 back_right[1][3];                                 //选取行55,56,57，列74，76，78作为右后标志区域
uint8 mid_front_left[1][3];
uint8 mid_front_right[1][3];
uint8 mid_front_mid[1][3];
uint8 huandao_row0l[10];
uint8 huandao_row1l[10];
uint8 huandao_row2l[10];
uint8 huandao_row3l[10];
uint8 huandao_row4l[10];
uint8 huandao_row0r[10];
uint8 huandao_row1r[10];
uint8 huandao_row2r[10];
uint8 huandao_row3r[10];
uint8 huandao_row4r[10];
uint8 midline;
int8 step1 = 0;
uint8 step2;
uint8 step3;
uint8 stepup;
uint8 step_30_L;
uint8 step_30_R;
int Obs_L=0;
int Obs_R=0;
uint8 stepmid;
uint8 stepdown;
uint8 startline;
uint8 runsignal;
int set  ;
int turn_delay;
int turn_first;
int turn_second;
int wall_turnleft=0;
int wall_turnright=0;
int delaytime;
int whiteline;
int16 whitel;
int16 whiter;
int blackline;
int whitelineL;
int whitelineL_all;
int whitelineR_all;
int whitelineR;
int whitelineLf;
int whitelineRf;
int line_countl;
int whitelineLcounter;
int whitelineRcounter;
static int counter_allowL=0;
static int counter_allowR=0;
int fix=0;
uint8 runsignar;
int line_countr;
uint8 no_wanl;
uint8 no_wanr;
uint16 stoptime;
uint16 turn_time1;
uint16 turn_time2;
uint16 turn_time3;
uint16 test ;
uint16 test2 ;
int xilo1l;
int xilo2l;
int xilo3l;
int xilo4l;
int kulo1l;
int kulo2l;
int kulo3l;
int kulo4l;
int xilo0l;
int kulo0l; 
int xilo1r;
int xilo2r;
int xilo3r;
int xilo4r;
int kulo1r;
int kulo2r;
int kulo3r;
int kulo4r;
int xilo0r;
int kulo0r;

int region1;
int region2l;
int region2r;
int region3;
int region4;
int region5;
int region6;
int region7;
int region8;
int region9;
int region10;
int region11;
int region12;
int HD0l;
int HD1l;
int HD2l;
int HD3l;
int HD4l;
int HD0r;
int HD1r;
int HD2r;
int HD3r;
int HD4r;
int16 speed;
uint16 duoji_last = 485;

  int16 Roundabout_firstCheck_L;
  int16 Roundabout_firstCheck_R;
  int16 Roundabout_secondCheck_L;
  int16 Roundabout_secondCheck_R;
  int16 Roundabout_thirdCheck_L;
  int16 Roundabout_thirdCheck_R;
  int16 Roundabout_forthCheck_L;
  int16 Roundabout_forthCheck_R;

int Roundabout_flag =0;
int Roundabout_turnflag;
int Roundabout_directionflag;


unsigned char val_d[]={"V:"};
int16 val;
int inflection_left_sink;
int inflection_left_raise;
int inflection_right_sink;
int inflection_right_raise;
int mutationl=0;
int mutationr=0;
int16 pre_error=0;
int16 error_last=0 ; 
uint8_t old_midlav2 ;
uint16_t middlex ;
int16 duty_n ;
int16 sloper_left_last=0;
int16 sloper_right_last=0;
int16 inflection_count_right_sink=0;
int16 inflection_count_right_raise=0;
int16 inflection_count_left_sink=0;
int16 inflection_count_left_raise=0;
int left_flag ;
int right_flag ;
int keep_left = 0 ;
int keep_right = 0;
int boundry_left=0;
int boundry_right=0;
int16 mid1_av,SD5_error,SD5_error_last=0,SD5_error_abs;
uint16 SD5_PWM;
uint16 SD_5_round;
int16 duty_fin;
int16 duty_out=0;
int16 error=0;
int8  BC_speed;
int16  BC;

double P ;
//uint8 m,n;

//static uint8 leftFindFlag;			        // 用来标记左黑线是否找到
//static uint8 rightFindFlag;				// 用来标记右黑
//static int16 rightCount;
//static int16 findLine;
//unsigned char val_c[1];
//int16 mid1_av;
//int16 mid2_av;
//int16 mid_av;    

/*********************函数声明***************************/

void findCenterLine(uint8 (* img)[CAMERA_W]);
void findbound(uint8(* img)[CAMERA_W]);
int16 createPoint(int type, int line);
void PORTA_IRQHandler();
void dianjicontorl();
void DMA0_IRQHandler();
void steering_control();
void PIT0_IRQHandler(void);
void speedcontrol(int16 duty_t);
void motorcontrol();
void SD5_Control();
void SD5_Control_all();
void areaextract();
void areasignal();
void inflection_left();
void inflection_right();
void SD5_Control_Roundabout();
int16 siadaoshibie();
void Roundabout_Control() ;
void cross_control();
void cross_wall();

//char filter() ;
//void zhangbin(uint8 a);
//volatile uint32 cnvtime;                //输入捕捉值
//void OLED_midline();

/********************函数主干**********x****************/
/*!
 *  @brief      main函数
 *  @since      v5.0
 *  @note       山外摄像头 LCD 测试实验
 */

void  main(void)
{
    uint8_t i;
    Site_t site     = {0, 0};                           //显示图像左上角位置
    Size_t imgsize  = {CAMERA_W, CAMERA_H};             //图像大小
    Size_t size;                                        //显示区域图像大小
    size.H = 64;
    size.W = 128;
#if 1
    OLED_Init();
    OLED_CLS();
    for(i=0; i<4; i++)
    {
		 OLED_P32x32Ch(i*32,0,i);
		 OLED_P32x32Ch(i*32,4,i+4);
		 OLED_P32x32Ch(i*32,8,i+2);
		 OLED_P32x32Ch(i*32,12,i+3);
     } 
     OLED_Fill(0x00);
#endif
    camera_init(imgbuff);
    move_init();
    turn_init();
    NVIC_SetPriority(PORTA_IRQn,0);         //配置优先级
    NVIC_SetPriority(DMA0_IRQn,1);          //配置优先级
    NVIC_SetPriority(PIT0_IRQn,2);          //配置优先级
    port_init (PTA10,  PULLUP );
    set_vector_handler(PORTA_VECTORn , PORTA_IRQHandler);   //设置 PORTA 的中断服务函数为 PORTA_IRQHandler
    set_vector_handler(DMA0_VECTORn ,DMA0_IRQHandler);    //设置 DMA0 的中断服务函数为 PORTA_IRQHandler
    ftm_quad_init(FTM2);                                    //FTM1 正交解码初始化（所用的管脚可查 port_cfg.h ）
    pit_init_ms(PIT0, 10);                                 //初始化PIT0，定时时间为： 1000ms
    set_vector_handler(PIT0_VECTORn ,PIT0_IRQHandler);      //设置PIT0的中断服务函数为 PIT0_IRQHandler
    
    enable_irq (PIT0_IRQn);
    //duty_out=80;
    
    OLED_P6x8Str(81,0,val_d); 
    while(1)
    {
     // turn_duty(315);
   //    speed=100;
//       speed=100;
//////       DELAY_MS(2000);
//    //        speed=85;
//////////       DELAY_MS(2000);
          camera_get_img();
          img_extract(img, imgbuff, CAMERA_SIZE);
          findbound(img);
          areaextract() ;
          areasignal() ;
          cross_control();
          Roundabout_Control() ;
          findCenterLine(img);
          OLED_Drawcamera(img); 
          SD5_Control_all();
          speedcontrol(speed);
          motorcontrol();
       
   //vcan_sendimg(img,CAMERA_W*CAMERA_H);
//        
//        
//       

    }
}



/*!
 *  @brief      PIT0中断服务函数
 *  @since      v5.0
 */
void PIT0_IRQHandler(void)
{
  unsigned char* number [10]= {"0","1","2","3","4","5","6","7","8","9"};
 // uint8 chrge = 1;
    //int16 val_v;
    uint8 val_g_d;
    uint8 val_s_d; 
    uint8 val_b_d;
//    uint8 val_g_v;
//    uint8 val_s_v; 
//    uint8 val_b_v;
    uint8 val_g_p;
    uint8 val_s_p;
    uint8 val_q_p;
    uint8 val_b_p;
    uint8 val_w_p;
    unsigned char val_error[] = { "Toggle!"};
    unsigned char val_suc[] = {"dm/s"};
    unsigned char val_clear[] = {"     "};
    unsigned char val_0[] = {"0"};
    val = ftm_quad_get(FTM2);//获取FTM 正交解码 的脉冲数(负数表示反方向)
    //filter() ;
    //val_v = (val*9)/8;
    duty_n =(int) ((((((double)(val/31.5))+44.4)/2)-22)*100-19);
    val_b_d= duty_n/100;
    val_s_d=(duty_n%100)/10;
    val_g_d= duty_n%10;
//    val_b_v= val_v/100;
//    val_s_v=(val_v%100)/10;
//    val_g_v= val_v%10;
    val_w_p=val/10000;
    val_q_p=(val%10000)/1000;
    val_b_p=(val%1000)/100;
    val_s_p=(val%100)/10;
    val_g_p=val%10;
    ftm_quad_clean(FTM2);

    if(val>0)
    {
       OLED_P6x8Str(83,4,val_clear);
       OLED_P6x8Str(81,2,val_clear);
       OLED_P6x8Str(105,2, number[val_g_p]);
       OLED_P6x8Str(99,2, number [val_s_p]);
       OLED_P6x8Str(93,2, number [val_b_p]);
       OLED_P6x8Str(87,2, number [val_q_p]);
       OLED_P6x8Str(81,2, number [val_w_p]);
       //OLED_P6x8Str(93,3, number[val_g_v]);
       //OLED_P6x8Str(87,3, number [val_s_v]);
       //OLED_P6x8Str(81,3, number [val_b_v]);
       OLED_P6x8Str(93,3, number[val_g_d]);
       OLED_P6x8Str(87,3, number [val_s_d]);
       OLED_P6x8Str(81,3, number [val_b_d]);
       OLED_P6x8Str(105,3, val_suc);
       //OLED_P6x8Str(100,2,val_suc);
       OLED_P6x8Str(100,3,val_clear);
    }
    else if(val == 0)
    {
      OLED_P6x8Str(81,3,val_clear);
      OLED_P6x8Str(81,2,val_clear);
      OLED_P6x8Str(100,5,val_clear);
      OLED_P6x8Str(83,4,val_clear);
      OLED_P6x8Str(85,1,val_clear);
      OLED_P6x8Str(100,2,val_clear);
      OLED_P6x8Str(100,3,val_0);
    }
    else
    {
      OLED_P6x8Str(81,3,val_clear);
      OLED_P6x8Str(85,1,val_clear);
      OLED_P6x8Str(100,2,val_clear);
      OLED_P6x8Str(83,4,val_error);
      OLED_P6x8Str(100,3,val_clear);
    }
    if(Obs_L==1||Obs_R==1)
    {
      turn_time1++;
    }
    if(turn_time1>=20&&(Obs_L==1||Obs_R==1))
    {
      turn_first=1;      
    }
    if(turn_first==1&&(Obs_L==1||Obs_R==1))
    {
      turn_time1=0;
      turn_time2++;
    }
    if(turn_time2>=20&&(Obs_L==1||Obs_R==1)&&turn_first==1)
    {
      turn_second=1;      
    }
    if(turn_first==1&&turn_second==1&&(Obs_L==1||Obs_R==1))
    {
      turn_time2=0;
      turn_time3++;
    }
    if(turn_time3>=20&&turn_first==1&&turn_second==1&&(Obs_L==1||Obs_R==1))
    {
      Obs_L=0;
      Obs_R=0;
      turn_first=0;
      turn_second=0;
      turn_time3=0;
    }
    if(startline==1)
    {
      stoptime++;
    }
    if(stoptime>=300)
    {
      runsignal=1;
      stoptime=0;
      startline=0;
    }
    if(turn_delay==1)
    {
      delaytime++;
    }
    if(delaytime>=70)
    {
      turn_delay=0;
      delaytime=0;
    }
   // speedcontrol(speed);
    PIT_Flag_Clear(PIT0);       //清中断标志位
}
  
/*!
 *  @brief      PORTA中断服务函数
 *  @since      v5.0
 */
void PORTA_IRQHandler()
{
    uint8  n;    //引脚号
    uint32 flag;

    while(!PORTA_ISFR);
    flag = PORTA_ISFR;
    PORTA_ISFR  = ~0;                                   //清中断标志位

    n = 29;                                             //场中断
    if(flag & (1 << n))                                 //PTA29触发中断
    {
        camera_vsync();
    }
#if ( CAMERA_USE_HREF == 1 )                            //使用行中断
    n = 28;
    if(flag & (1 << n))                                 //PTA28触发中断
    {
        camera_href();
    }

    
    
 void Hisr()
 {
 static uint16 Hn=0; //正在采集的行数

 //行中断来了延时一下，跳过消隐区
 DELAY_A();

 for(i=0;i<每行元素数目;i++)
 {
 //采集图像值，并保存在对应的内存数组里
 port2buff(Hn++,i);
DELAY_B();
 }

 Hn++;
 }
 ///////
#endif
    
}



/*!
 *  @brief      DMA0中断服务函数
 *  @since      v5.0
 */

void DMA0_IRQHandler()
{
    camera_dma();
}



//void steering_control()//舵机控制
//{  
//  if(boundry_left==1&&boundry_right==0)
//    {
//       SD5_PWM=660;
//       boundry_left=0;
//       boundry_right=0;
//    }
//else if(boundry_left==0&&boundry_right==1)
//    {
//       SD5_PWM=340;
//       boundry_left=0;
//       boundry_right=0;
//    }
// else if (boundry_left==1&&boundry_right==1)
//    {
//       SD5_PWM=duoji_last;
//       boundry_left=0;
//       boundry_right=0;
//    }
// else if (boundry_left==0&&boundry_right==0)
//  {
//    if (region8 == 0)
//    {
//      SD5_PWM=duoji_last;
//    }
//    else
//    {
//       SD5_Control();
//    }       
//       boundry_left=0;
//       boundry_right=0;
//  }
//       turn_duty(SD5_PWM);
//       duoji_last=SD5_PWM;
//}




void SD5_Control()
{
  int temp ; 
  uint8 SD5_Kp,SD5_Kd;
  Roundabout_flag=0;
  Roundabout_directionflag=0;
 //int16 mid3_av;
 //  if(fix==0)
//  {
//    SD5_Control();
//    if(SD5_PWM<485)
//    {
//      fix=1;
//    }
//    else if(SD5_PWM>485)
//    {
//      fix=2;
//    }
//    else
//      fix=0;
//  }
//if(boundry_right==1&&fix==1&&boundry_left!=1/*&&cross == 0*/)
//  {
//    SD5_PWM=340;
//    boundry_right=0;
//  }
//else if(boundry_left==1&&fix==2&&boundry_right!=1/*&&cross == 0*/)
//{ 
//  SD5_PWM=660;
//  boundry_left=0;  
//}
//
//else
//{
//if (keep_left == 0 && keep_right == 0)
//{ 
  mid1_av=(int)((centerLine[mid4_1]*1.2)+(centerLine[mid4_2]*1)+(centerLine[mid4_3]*0.8))/3;
  mid0_av=(int)((centerLine[mid3_1]*0.8)+(centerLine[mid3_2]*1)+(centerLine[mid3_3]*1.2))/3;
  mid2_av=(int)((centerLine[mid5_1]*0.8)+(centerLine[mid5_2]*1)+(centerLine[mid3_3]*1.2))/3;
  Base_line=80-mid0_av;
  if(Base_line>=50) Base_line=50;
  if(Base_line<=30) Base_line=30;
  SD5_error=Base_line-mid1_av;
//  test= centerLine[centerLine[60]+5] ;
//  test2 = centerLine[60]+5 ;
  //SD5_error=mid2_av-test;
  //t=centerLine[60]+15
  if(SD5_error>=0)
  SD5_error_abs=SD5_error;
  else SD5_error_abs=(-1*SD5_error);
  mid3_av= mid2_av-40;
  
//  if((centerLine[60]+5)>59)
//  {
//    temp = duoji_last ;
//    turn_duty(temp);
//    duoji_last = temp ;
//  }
  
 if(cross == 0)
  { 
//      if(duty_n<=200&&duty_n>=87)
//    {
//      SD5_Kp=11;
//      SD5_Kd=19;
//      error = (int)(SD5_Kp*SD5_error+SD5_Kd*(SD5_error-SD5_error_last));
//      SD5_PWM =485 + error-mid3_av;
//    }
//    else if(duty_n<87&&duty_n>=0)
//    {
   if   (region5==0&&region8==0&&region10==0)
  {
    SD5_PWM = duoji_last;
  }
// else if(0x03<step1<0x06 &&whitelineLf<=20/*&&(20<=whiteline||whiteline<=35)*/)
//  {
//    wall_turnright = 1;
//  // if (whitelineLf<=20)
//  //  {
//     SD5_PWM=375;
//  //  }
//  //  else
//  //  {
//   //   SD5_PWM = duoji_last;
//      //wall_turnright = 0 ;  
//   //}
//  }
//  else if(whitelineLf>20 && wall_turnright == 1)
//  {
//    if (whiteline>=38)
//    {
//      SD5_PWM= 625;
//      wall_turnright = 0;
//      wall_turnleft = 0;
//    }
//    else
//    {
//      SD5_PWM = 485 ;
//    }
//  }
//  else if(whitelineLf==20||whitelineRf==20)
//  {
 //   SD5_PWM = 485 ;
 //   wall_turnright = 0;
 //    wall_turnleft = 0;
 // }
   else 
    {
      wall_turnright = 0;
      wall_turnleft = 0;
      BC_speed = duty_n - speed ;
      wall_turnright = 0 ;
      SD5_Kp=SD5_error_abs/8;
      SD5_Kd=7;
      error = (int)(SD5_Kp*SD5_error+SD5_Kd*(SD5_error-SD5_error_last));  
      BC =  BC_speed*error;
      if (BC_speed>0&&((-100<=error<=-20)||(20<=error<=100)))
      {
        if(BC>0)
        SD5_PWM =485 + error+BC_speed*1;
        else
        SD5_PWM =485 + error-BC_speed*1;
      }
      else
      {
        SD5_PWM =485 + error;
      }
//     }
//    else
//    {
//      SD5_Kp=0;
//      SD5_Kd=0;
//      error = (int)(SD5_Kp*SD5_error+SD5_Kd*(SD5_error-SD5_error_last));
//      SD5_PWM =485 + error-mid3_av;
    // SD5_PWM =485-(mid3_av*3);
 //   error = (int)(SD5_Kp*SD5_error+SD5_Kd*(SD5_error-SD5_error_last));
//    SD5_PWM =550 + error-mid3_av;
//    }
//  else if((SD5_error_abs>1|| SD5_error_abs<-1) && cross == 0)
//  {
//    
//    SD5_Kp=8;
//    SD5_Kd=20;
//    error = (int)(SD5_Kp*SD5_error+SD5_Kd*(SD5_error-SD5_error_last));
//    SD5_PWM =560 + error;
//  }
//  else if((SD5_error_abs>=-1&& SD5_error_abs<=3) && cross == 0)
//  {
//    
//    SD5_PWM=534-4*mid3_av;
//     }
//  if(cross==1)
//  {
//    SD5_PWM=485-(mid3_av*3);
//    cross = 0;    
//  }
  if(SD5_PWM<=300)
  {
    SD5_PWM=300;
  }
  if(SD5_PWM>=700)
  {
    SD5_PWM=700;
  }
  
    }
   if(Obs_L==1)
   {
     SD5_PWM=400;
     if(turn_first==1)
     {

       SD5_PWM=570;
       if(turn_second==1)
       {

         SD5_PWM=485;
       }         
     }
   }
   if(Obs_R==1)
   {
     SD5_PWM=570;
     if(turn_first==1)
     {
       SD5_PWM=400;
       if(turn_second==1)
       {
         SD5_PWM=485;
       }         
     }
   }
  }
//
 else SD5_PWM=485-mid3_av*3;
  SD5_error_last=SD5_error;
  
//  //SD5_PWM=0; 
// //  }
//     if(SD5_PWM<485)
//    {
//      fix=1;
//    }
//    else if(SD5_PWM>485)
//    {
//      fix=2;
//    }
//    else
//      fix=0;
//}
//  else if (keep_left == 1 && keep_right == 0)
//  {
//    SD5_PWM=660;
//    keep_left = 0 ;
//    keep_right = 0 ;
//  }
//else if (keep_left == 0 && keep_right == 1)
//  {
//    SD5_PWM=340;
//     keep_left = 0 ;
//    keep_right = 0 ;
//    
//  }
//else
//  {
//    //turn_duty (485) ; 
//    //turn_duty(660);
//     keep_left = 0 ;
//    keep_right = 0 ;
//  }
  if(SD5_PWM>=550||SD5_PWM<=450)
  {
    turn_delay=1;
  }
  turn_duty(SD5_PWM);
  duoji_last = SD5_PWM;
}

void SD5_Control_Roundabout()
{
  mid2_av=(int)((centerLine[mid5_1]*0.8)+(centerLine[mid5_2]*1)+(centerLine[mid5_3]*1.2))/3;
  mid3_av=mid2_av-40;
  if (Roundabout_directionflag==left_Roundabout )
  {
    if (Roundabout_flag == Roundabout_Pre)
    {
      SD5_PWM=485-mid3_av*3;      
    }
    else if (Roundabout_flag==Roundabout_GoInto)
    {
      SD5_PWM=485+5*whitel;
      if(SD5_PWM>=660)
      {
        SD5_PWM=660;
      }
      SD_5_round=SD5_PWM;
    }
    else if (Roundabout_flag==Roundabout_In)
    {
      //steering_control();
     // SD5_Control();
//      if(whitel<=18)
//      {
        SD5_PWM=SD_5_round-(mid3_av*2/3+2*whitel/3)*3;
       // SD5_PWM=SD_5_round-(mid3_av+7/*2*whitel/5*/)*4;
//      }
//      else
//      {
//        SD5_PWM=SD_5_round-mid3_av*5;
//      }
    }
    else if (Roundabout_flag==Roundabout_PreOut)
    {
     // steering_control();
      //SD5_PWM=610;
       SD5_PWM=SD_5_round-(mid3_av+whitel/3)*4;
       //SD5_PWM=SD_5_round-(mid3_av+7/*whitel/3*/)*4;
    }
     else if (Roundabout_flag==Roundabout_After)
    {
      SD5_Control();
      Roundabout_flag =0;
      Roundabout_directionflag=0;
      //SD5_PWM=610;
    }
    else if (Roundabout_flag==Roundabout_Out)
    {
     //steering_control();
      SD5_PWM=485-mid3_av*3;
      //SD5_PWM=486-mid3_av*4;
      //Roundabout_flag =0;
    }
  }
  else if (Roundabout_directionflag==right_Roundabout)
  {
    if (Roundabout_flag == Roundabout_Pre)
    {
      SD5_PWM=485-mid3_av*3; 
     // SD5_PWM=390;
    }
    else if (Roundabout_flag==Roundabout_GoInto)
    {
      SD5_PWM=485-5*whiter;
      if(SD5_PWM<=340)
      {
        SD5_PWM=340;
      }
      SD_5_round=SD5_PWM;
    }
    else if (Roundabout_flag==Roundabout_In)
    {
//      if(whitel<=18)
//      {
       SD5_PWM=SD_5_round-(mid3_av-2*whiter/3)*3;
      //SD5_PWM=380;
//      }
//      else
//      {
//        SD5_PWM=SD_5_round-mid3_av*5;
//      }
    }
    else if (Roundabout_flag==Roundabout_PreOut)
    {
     // steering_control();
      //SD5_PWM=SD_5_round-(mid3_av-whiter)*3;
      SD5_PWM=380;
    }
     else if (Roundabout_flag==Roundabout_After)
    {
      SD5_Control();
      Roundabout_flag =0;
      Roundabout_directionflag=0;
      //SD5_PWM=610;
    }
    else if (Roundabout_flag==Roundabout_Out)
    {
     //steering_control();
      SD5_PWM=485-mid3_av*3;
      //SD5_PWM=380;
      //Roundabout_flag =0;
    }
  }
  else
  {
    turn_duty(duoji_last);
    
  }
 
  turn_duty(SD5_PWM);
  duoji_last=SD5_PWM;
}

void SD5_Control_all()
{
 if(Roundabout_flag!=0)    SD5_Control_Roundabout();
 //else if (0x03<step1<0x06 )
 //{
 //  cross_wall();
 //}
  else                     // steering_control();
  { 
    SD5_Control();
  }
//  turn_duty(SD5_PWM);
//  duoji_last=SD5_PWM;
}

void cross_control()
{
  if(whiteline>=60) cross=1;
  if(whiteline<70&&blackline<=75) cross=0;
}
  

void Roundabout_Control()
{
      set = 0;      
      inflection_left();
      inflection_right();
 
   ////////////////////////////////////////////////////////////////////////////判断前方是不是环岛
  if(Roundabout_flag==0)
    {

       if(mutationr<=1&&(inflection_left_sink>0)&& HD1r == 0 && HD2r == 0 && HD3r == 0 && HD4r ==0&&HD0r==0 &&line_countl>=55&&(inflection_left_raise==2||inflection_left_raise==1)&&inflection_right_sink==0&&inflection_right_raise==0 &&(region2l==1||region2r==1) &&region4==1&&region8==1&&region5==1 )
        {
           Roundabout_flag = Roundabout_Pre;                           //是环岛
           Roundabout_directionflag=left_Roundabout;                 //判断是左环岛
        }
       else if(mutationl<1&&inflection_left_sink==0&& HD1l == 0 && HD2l == 0 && HD3l == 0 && HD4l ==0&&HD0l==0&&line_countr>=55&&inflection_left_raise==0&&(inflection_right_sink>0)&&(inflection_right_raise==2||inflection_right_raise==1) &&(region2l==1||region2r==1)&&region6==1&&region8==1&&region5==1  )
        {
           Roundabout_flag=Roundabout_Pre;
           Roundabout_directionflag=right_Roundabout;
            
        }
     }
  //////////////////////////////////////////////////////////////////////////////判断是不是该打转向进入环岛
  else if(Roundabout_flag==Roundabout_Pre)
  {
       
       if(Roundabout_directionflag==left_Roundabout &&whitelineLcounter>=2/*&& HD1l == 0 && HD2l == 0 && HD3l == 0 && HD4l ==0&&HD0l==0 && region5 == 1&&region8 == 1&&region7 == 1*/)
       {
         Roundabout_flag=Roundabout_GoInto;   //环岛标志位置为将要打转向进入环岛
       }
   else if(Roundabout_directionflag==right_Roundabout&&whitelineRcounter>=2/*&& HD1r == 0 && HD2r == 0 && HD3r == 0 && HD4r ==0&&HD0r==0&& region5 == 1&&region8 == 1&&region9 == 1*/)

       {
         Roundabout_flag=Roundabout_GoInto ;
       }      
  }

  else if(Roundabout_flag == Roundabout_GoInto)   
  {
        if (Roundabout_directionflag==left_Roundabout &&line_countl<=55&&region6==0/*&&(region2l==0&&region2r==0)&&region9==0*/)
           {
            Roundabout_flag=Roundabout_In;
            set = 1;
           }
        else if (Roundabout_directionflag==right_Roundabout &&line_countr<=55/*&&region12==0&&(region2l==0&&region2r==0)&&region7==0*/&&region4==0)
           {
            Roundabout_flag=Roundabout_In;
            set = 1;
           }
        
    }  
    else if(Roundabout_flag==Roundabout_In)
    {

       if (Roundabout_directionflag==left_Roundabout /*&& region4==0&& region12==0*/ && region6==0 && region10==1 && region5==1 && region11==1 && whiteline>=75)
         {
           Roundabout_flag=Roundabout_PreOut;   
         } 
       
      else if (Roundabout_directionflag==right_Roundabout /*&& region4==0 && region12==0*/ && region4==0 && region10==1 && region5==1 && region11==1 && whiteline>=75)
         {
           Roundabout_flag=Roundabout_PreOut;   
         }
    }
    else if(Roundabout_flag==Roundabout_PreOut)
    {

       if (Roundabout_directionflag==left_Roundabout&&region1==1/*&& (region2l==1|| region2r==1) &&region12==1*/&&inflection_right_sink==0&&region11==0)
         {
           Roundabout_flag=Roundabout_Out;   
         } 
       
      else if (Roundabout_directionflag==right_Roundabout&&region3==1/*&&region7==1/*&& (region2l==1|| region2r==1) &&region12==1*/&&inflection_left_sink==0&&region10==0)
         {
           Roundabout_flag=Roundabout_Out;   
         }
    }
    else if(Roundabout_flag==Roundabout_Out)
    {

       if (Roundabout_directionflag==left_Roundabout &&region7==0)
         {
           Roundabout_flag=Roundabout_After; 
          // Roundabout_directionflag = 0;
         } 
       if (Roundabout_directionflag==right_Roundabout &&region9==0)
         {
           Roundabout_flag=Roundabout_After;  
         //  Roundabout_directionflag = 0 ;
         }
    }
 
}

void inflection_left()
{
  static int16 line;
  static int inflection_pre=0;
  static int inflection_warning=0;
  int16 sloper=0;  

  mutationl=0;
  
  for(line=59;line>=1;line--)
  {
    slopl[line-1]=leftBound[line-1]-leftBound[line];
    sloper=slopl[line-1];
    if (sloper==0)
    {
      continue;
    }
    if(sloper>0)
    {
      if(inflection_pre==0)
      {
//        if(inflection_warning==0)
//        {
          inflection_pre=1;
          sloper_left_last=sloper;
          continue;
//        }
//        else if(sloper*sloper_left_last>0)
//        {
//          inflection_count_left_raise++;
//          sloper_left_last=sloper;
//          inflection_warning=0;
//          continue;
//        }
      }
      else if(inflection_pre==1)
      {
         if(sloper*sloper_left_last<0)
        { 
          inflection_count_left_raise++;
          sloper_left_last=sloper;
          inflection_pre=0;
        //  inflection_warning=1;
          continue;
         }
      }
    }

    if(sloper<0)
    {
      if(inflection_pre==0)
      {
//        if(inflection_warning==0)
//        {
          inflection_pre=1;
          sloper_left_last=sloper;
          continue;
//        }
//        else if(sloper*sloper_left_last>0)
//        {
//          inflection_count_left_sink++;
//          sloper_left_last=sloper;
//          inflection_warning=0;
//          continue;
//        }        
      }
      else if(inflection_pre==1)
      {
         if(sloper*sloper_left_last<0)
        {   
          inflection_count_left_sink++;
          sloper_left_last=sloper;
          inflection_pre=0;
      //    inflection_warning=1;
          continue;
         }
      }
    }
    if(sloper<=-2||sloper>=2)
    {
      mutationl++;
    }
  }
  inflection_left_sink=inflection_count_left_sink;
  inflection_left_raise=inflection_count_left_raise;
  inflection_count_left_sink=0;
  inflection_count_left_raise=0;
}

void inflection_right()
{
  static int16 line;
  static int inflection_pre=0;
  static int inflection_warning=0;
  int16 sloper=0;

  mutationr=0;
  
  for(line=59;line>=1;line--)
  {
    slopr[line-1]=rightBound[line-1]-rightBound[line];
    sloper=slopr[line-1];
    if (sloper==0)
    {
      continue;
    }
    if(sloper>0)
    {
      if(inflection_pre==0)
      {
//        if(inflection_warning==0)
//        {
          inflection_pre=1;
          sloper_right_last=sloper;
          continue;
//        }
//        else if(sloper*sloper_right_last>0)
//        {
//          inflection_count_right_raise++;
//          sloper_right_last=sloper;
//          inflection_warning=0;
//          continue;
//        }
      }
      else if(inflection_pre==1)
      {
         if(sloper*sloper_right_last<0)
        {  
          inflection_count_right_raise++;
          sloper_right_last=sloper;
          inflection_pre=0;
       //  inflection_warning=1;
          continue;
         }
      }
    }

    if(sloper<0)
    {
      if(inflection_pre==0)
      {
//        if(inflection_warning==0)
//        {
          inflection_pre=1;
          sloper_right_last=sloper;
          continue;
//        }
//        else if(sloper*sloper_right_last>0)
//        {
//          inflection_count_right_sink++;
//          sloper_right_last=sloper;
//          inflection_warning=0;
//          continue;
//        }        
      }
      else if(inflection_pre==1)
      {
         if(sloper*sloper_right_last<0)
        {
          inflection_count_right_sink++;
          sloper_right_last=sloper;
          inflection_pre=0;
       //   inflection_warning=1;
          continue;
         }
      }     
    }
    if(sloper<=-2||sloper>=2)
    {
      mutationr++;
    }
  }
  inflection_right_sink=inflection_count_right_sink;
  inflection_right_raise=inflection_count_right_raise;
  inflection_count_right_sink=0;
  inflection_count_right_raise=0;
}

void dianjicontorl()
{
  uint8 black_left ;
  uint8 black_right ;
  black_left = region1+region2l+region2r;
  black_right =region3+region2l+region2r;
 if (region4 == 1 && region7 == 1 && region8 == 1/*||(region4 == 1 && region7 == 1 && region9 == 1)*/)
 {
   left_flag = 1 ;
 }
 else
 {
   left_flag = 0 ;
 }
 if ((region6 == 1 && region9 == 1 && region8 == 1)/*||(region6 == 1 && region7 == 1 && region8 == 1)*/)
 {
   right_flag = 1 ;
 }
 else
 {
   right_flag = 0 ;
 }
 if (region5 == 0)
 {
   
  if(left_flag ==1 && right_flag ==0)
  {
    if (black_left <4)
    {
      if (region10 == 1)
     {
      turn_duty (600);
      keep_left = 1 ;
      duoji_last = 600;
     }
    else
    {
      turn_duty (660);
      keep_left = 1 ;
      duoji_last = 660;
    }
    }
    else
    {
      steering_control();
      //turn_duty (485);
     //  keep_left = 1 ;
    
  }
  }
  else if(left_flag == 0 && right_flag == 1)
  {
    if (black_right <4)
    {
      if(region11 == 1)
      {
      turn_duty (315);
      keep_right =1;
      duoji_last = 315 ;
      }
      else
     {
      turn_duty (340);
      keep_right =1;
      duoji_last = 340 ;
     }
    }
    else
    {
      steering_control();
      //turn_duty (485); 
      //keep_right =1;
    }
   
  }
  else
   {
     steering_control();
     // turn_duty (485); 
   }
}
 else
   {
     SD5_Control();
      //turn_duty (485); 
   }

}



void motorcontrol()
{
  int i;
  int16 sum1=0;
  int16 sum2=0;

  int16 mid1_av_new,mid2_av_new;
  for(i=15;i<=20;i++)
  {
    sum1+=centerLine[i];
  }
  mid1_av_new=(sum1/6);
  for(i=58;i<=59;i++)
  {
    sum2+=centerLine[i];
  }
  mid2_av_new=(sum2/2);
  piancha=mid1_av_new-mid2_av_new;
  if(Base_line<=35||Base_line>=45||Roundabout_flag!=0||turn_delay==1||Obs_L==1)
  {
   speed=58;
  }
  else
  {
   speed=95;
  }
  if(startline==1&&runsignal==0)
  {
    speed=0;
  }
  piancha_old = piancha ;
}

void areaextract()
{
  uint8_t i,h;
  static int16 row;
  i=0;
  for(row = CAMERA_H-1;row >=0;row--) 
  {
         if(row==56/*||row==56||row==57*/)
         {
             for(h=0;h<3;h++)
             {
               back_left[i][h]=img[row][h*2+1];
               back_mid[i][h]=img[row][h*2+38];
               back_right[i][h]=img[row][h*2+74]; 
             }
         }
         if(row==20/*||row==28||row==29*/)
         {
             for(h=0;h<3;h++)
             {
               mid_left[i][h]=img[row][h*2+1];
               mid_front_mid[i][h]=img[row][h*2+30];
               mid_right[i][h]=img[row][h*2+74]; 
             }
         }
         
         if(row==44/*||row==28||row==29*/)
         {
             for(h=0;h<3;h++)
             {
               mid_front_left[i][h]=img[row][h*2+1];
               mid_mid[i][h]=img[row][h*2+38];
               mid_front_right[i][h]=img[row][h*2+74];
             }
         }
         if(row==17)
         {
             for(h=0;h<10;h++)
             {
               huandao_row4l[h]=img[row][h]; 
               huandao_row4r[h]=img[row][h+69];
             }
         }
         if(row==13/*||row==13||row==14*/)
         {
             for(h=0;h<3;h++)
             {
               front_left[i][h]=img[row][h*2+1];
               front_mid_l[i][h]=img[row][h*2+30];
               front_mid_r[i][h]=img[row][h*2+46];
               front_right[i][h]=img[row][h*2+74]; 
               huandao_row3l[h]=img[row][h];
               huandao_row3r[h]=img[row][h+69];
             }
         }
         if(row==9)
         {
             for(h=0;h<10;h++)
             {
               huandao_row2l[h]=img[row][h];
               huandao_row2r[h]=img[row][h+69];
             }
         }
         if(row==5)
         {

             for(h=0;h<10;h++)
             {
               huandao_row1l[h]=img[row][h];
               huandao_row1r[h]=img[row][h+69];
             }
         } 
         if(row==2)
         {
             for(h=0;h<10;h++)
             {
               huandao_row0l[h]=img[row][h];
               huandao_row0r[h]=img[row][h+69];
             }
         }
     //  i++;
  }
}

void areasignal()
{
  uint8_t i=0;
  uint8_t singalline1B=0,singalline1W=0;
  uint8_t singalline20B=0,singalline20W=0;
  uint8_t singalline21B=0,singalline21W=0;
  uint8_t singalline3B=0,singalline3W=0;
  uint8_t singalline4B=0,singalline4W=0;
  uint8_t singalline5B=0,singalline5W=0;
  uint8_t singalline6B=0,singalline6W=0;
  uint8_t singalline7B=0,singalline7W=0;
  uint8_t singalline8B=0,singalline8W=0;
  uint8_t singalline9B=0,singalline9W=0;
  uint8_t singalline10B=0,singalline10W=0;
  uint8_t singalline11B=0,singalline11W=0;
  uint8_t singalline12B=0,singalline12W=0;
  xilo1l=0;
  xilo2l=0;
  xilo3l=0;
  xilo4l=0;
  kulo1l=1;
  kulo2l=1;
  kulo3l=1;
  kulo4l=1;
  xilo0l=0;
  kulo0l=1;
  xilo1r=0;
  xilo2r=0;
  xilo3r=0;
  xilo4r=0;
  kulo1r=1;
  kulo2r=1;
  kulo3r=1;
  kulo4r=1;
  xilo0r=0;
  kulo0r=1;

  
//  for(i=0;i<3;i++)
//  {
     if((front_left[i][0]==IMG_BLACK)&&(front_left[i][1]==IMG_BLACK)&&(front_left[i][2]==IMG_BLACK))
     {
       singalline1B=1;
     }
     if((front_mid_l[i][0]==IMG_BLACK)&&(front_mid_l[i][1]==IMG_BLACK)&&(front_mid_l[i][2]==IMG_BLACK))
     {
       singalline20B=1;
     }
     if((front_mid_r[i][0]==IMG_BLACK)&&(front_mid_r[i][1]==IMG_BLACK)&&(front_mid_r[i][2]==IMG_BLACK))
     {
       singalline21B=1;
     }
     if((front_right[i][0]==IMG_BLACK)&&(front_right[i][1]==IMG_BLACK)&&(front_right[i][2]==IMG_BLACK))
     {
       singalline3B=1;
     }
     if((mid_left[i][0]==IMG_BLACK)&&(mid_left[i][1]==IMG_BLACK)&&(mid_left[i][2]==IMG_BLACK))
     {
       singalline4B=1;
     }
     if((mid_mid[i][0]==IMG_BLACK)&&(mid_mid[i][1]==IMG_BLACK)&&(mid_mid[i][2]==IMG_BLACK))
     {
       singalline5B=1;
     }
     if((mid_right[i][0]==IMG_BLACK)&&(mid_right[i][1]==IMG_BLACK)&&(mid_right[i][2]==IMG_BLACK))
     {
       singalline6B=1;
     }
     if((back_left[i][0]==IMG_BLACK)&&(back_left[i][1]==IMG_BLACK)&&(back_left[i][2]==IMG_BLACK))
     {
       singalline7B=1;
     }
     if((back_mid[i][0]==IMG_BLACK)&&(back_mid[i][1]==IMG_BLACK)&&(back_mid[i][2]==IMG_BLACK))
     {
       singalline8B=1;
     }
     if((back_right[i][0]==IMG_BLACK)&&(back_right[i][1]==IMG_BLACK)&&(back_right[i][2]==IMG_BLACK))
     {
       singalline9B=1;
     }
     if((mid_front_left[i][0]==IMG_BLACK)&&(mid_front_left[i][1]==IMG_BLACK)&&(mid_front_left[i][2]==IMG_BLACK))
     {
       singalline10B=1;
     }
     if((mid_front_right[i][0]==IMG_BLACK)&&(mid_front_right[i][1]==IMG_BLACK)&&(mid_front_right[i][2]==IMG_BLACK))
     {
       singalline11B=1;
     }
     if((mid_front_mid[i][0]==IMG_BLACK)&&(mid_front_mid[i][1]==IMG_BLACK)&&(mid_front_mid[i][2]==IMG_BLACK))
     {
       singalline12B=1;
     }
     if((front_left[i][0]==IMG_WHITE)&&(front_left[i][1]==IMG_WHITE)&&(front_left[i][2]==IMG_WHITE))
     {
       singalline1W=1;
     }
     if((front_mid_l[i][0]==IMG_WHITE)&&(front_mid_l[i][1]==IMG_WHITE)&&(front_mid_l[i][2]==IMG_WHITE))
     {
       singalline20W=1;
     }
     if((front_mid_r[i][0]==IMG_WHITE)&&(front_mid_r[i][1]==IMG_WHITE)&&(front_mid_r[i][2]==IMG_WHITE))
     {
       singalline21W=1;
     }
     if((front_right[i][0]==IMG_WHITE)&&(front_right[i][1]==IMG_WHITE)&&(front_right[i][2]==IMG_WHITE))
     {
       singalline3W=1;
     }
     if((mid_left[i][0]==IMG_WHITE)&&(mid_left[i][1]==IMG_WHITE)&&(mid_left[i][2]==IMG_WHITE))
     {
       singalline4W=1;
     }
     if((mid_mid[i][0]==IMG_WHITE)&&(mid_mid[i][1]==IMG_WHITE)&&(mid_mid[i][2]==IMG_WHITE))
     {
       singalline5W=1;
     }
     if((mid_right[i][0]==IMG_WHITE)&&(mid_right[i][1]==IMG_WHITE)&&(mid_right[i][2]==IMG_WHITE))
     {
       singalline6W=1;
     }
     if((back_left[i][0]==IMG_WHITE)&&(back_left[i][1]==IMG_WHITE)&&(back_left[i][2]==IMG_WHITE))
     {
       singalline7W=1;
     }
     if((back_mid[i][0]==IMG_WHITE)&&(back_mid[i][1]==IMG_WHITE)&&(back_mid[i][2]==IMG_WHITE))
     {
       singalline8W=1;
     }
     if((back_right[i][0]==IMG_WHITE)&&(back_right[i][1]==IMG_WHITE)&&(back_right[i][2]==IMG_WHITE))
     {
       singalline9W=1;
     }
     if((mid_front_left[i][0]==IMG_WHITE)&&(mid_front_left[i][1]==IMG_WHITE)&&(mid_front_left[i][2]==IMG_WHITE))
     {
       singalline10W=1;
     }
     if((mid_front_right[i][0]==IMG_WHITE)&&(mid_front_right[i][1]==IMG_WHITE)&&(mid_front_right[i][2]==IMG_WHITE))
     {
       singalline11W=1;
     }
     if((mid_front_mid[i][0]==IMG_WHITE)&&(mid_front_mid[i][1]==IMG_WHITE)&&(mid_front_mid[i][2]==IMG_WHITE))
     {
       singalline12W=1;
     }
     for(i=0;i<10;i++)
     {
       if(huandao_row1l[i]==IMG_WHITE)
       {
         xilo1l=1;
         kulo1l=0;
       }
       if(huandao_row2l[i]==IMG_WHITE)
       {
         xilo2l=1;
         kulo2l=0;
       }
       if(huandao_row3l[i]==IMG_WHITE)
       {
         xilo3l=1;
         kulo3l=0;
       }
       if(huandao_row4l[i]==IMG_WHITE)
       {
         xilo4l=1;
         kulo4l=0;
       }
       if(huandao_row0l[i]==IMG_WHITE)
       {
         xilo0l=1;
         kulo0l=0;
       }
     }
     for(i=0;i<10;i++)
     {
       if(huandao_row1r[i]==IMG_WHITE)
       {
         xilo1r=1;
         kulo1r=0;
       }
       if(huandao_row2r[i]==IMG_WHITE)
       {
         xilo2r=1;
         kulo2r=0;
       }
       if(huandao_row3r[i]==IMG_WHITE)
       {
         xilo3r=1;
         kulo3r=0;
       }
       if(huandao_row4r[i]==IMG_WHITE)
       {
         xilo4r=1;
         kulo4r=0;
       }
       if(huandao_row0r[i]==IMG_WHITE)
       {
         xilo0r=1;
         kulo0r=0;
       }
     }
//  }
  if(xilo1l == 1 && kulo1l == 0)
  {
    HD1l = 1 ;
  }
  else 
  {
    HD1l = 0 ;
  }
  if(xilo2l == 1 && kulo2l == 0)
  {
    HD2l = 1 ;
  }
  else 
  {
    HD2l = 0 ;
  }
  if(xilo3l == 1 && kulo3l== 0)
  {
    HD3l = 1 ;
  }
  else 
  {
    HD3l = 0 ;
  }
  if(xilo4l == 1 && kulo4l == 0)
  {
    HD4l = 1 ;
  }
  else 
  {
    HD4l = 0 ;
  }
  if(xilo0l == 1 && kulo0l == 0)
  {
    HD0l = 1 ;
  }
  else 
  {
    HD0l = 0 ;
  }
    if(xilo1r == 1 && kulo1r == 0)
  {
    HD1r = 1 ;
  }
  else 
  {
    HD1r = 0 ;
  }
  if(xilo2r == 1 && kulo2r == 0)
  {
    HD2r = 1 ;
  }
  else 
  {
    HD2r = 0 ;
  }
  if(xilo3r == 1 && kulo3r == 0)
  {
    HD3r = 1 ;
  }
  else 
  {
    HD3r = 0 ;
  }
  if(xilo4r == 1 && kulo4r == 0)
  {
    HD4r = 1 ;
  }
  else 
  {
    HD4r = 0 ;
  }
  if(xilo0r == 1 && kulo0r == 0)
  {
    HD0r = 1 ;
  }
  else 
  {
    HD0r = 0 ;
  }
  if(singalline1B==1){region1=0;}
  if(singalline1W==1){region1=1;}
  if(singalline20B==1){region2l=0;}
  if(singalline20W==1){region2l=1;}
  if(singalline21B==1){region2r=0;}
  if(singalline21W==1){region2r=1;}
  if(singalline3B==1){region3=0;}
  if(singalline3W==1){region3=1;}
  if(singalline4B==1){region4=0;}
  if(singalline4W==1){region4=1;}
  if(singalline5B==1){region5=0;}
  if(singalline5W==1){region5=1;}
  if(singalline6B==1){region6=0;}
  if(singalline6W==1){region6=1;}
  if(singalline7B==1){region7=0;}
  if(singalline7W==1){region7=1;}
  if(singalline8B==1){region8=0;}
  if(singalline8W==1){region8=1;}
  if(singalline9B==1){region9=0;}
  if(singalline9W==1){region9=1;}
  if(singalline10B==1){region10=0;}
  if(singalline10W==1){region10=1;}
  if(singalline11B==1){region11=0;}
  if(singalline11W==1){region11=1;}
  if(singalline12B==1){region12=0;}
  if(singalline12W==1){region12=1;}
}
  

void speedcontrol(int16 duty_t)
{
 // move_forwardl(duty_t);
  
  error=duty_t-duty_n;
  P_D=10;
  I_D=0.5;
  D_D=3;
  if(error>=20)
  {
    duty_out=400;
  }
  else if(error<=-20)
  {
    duty_out=-50;
  }
  else
  {
  duty_fin=(int)(((error-error_last)*P_D)+(error*I_D)+((error+pre_error-2*error_last)*D_D)); 
  
  pre_error=error_last; 
  error_last = error ;
 
//    if (duty_n>duty_t+20)
//    {
//       duty_out=10;
//     }
//    else if (duty_n<=duty_t-50)
//    {
//       duty_out=800;
//     }
//    else
//     {
//       duty_out+=duty_fin;
//     }
  duty_out+=duty_fin;
  if (duty_out>=400)
  { duty_out=400;}
  else if (duty_out<=-50)
  { duty_out=-50;}
  }
  ftm_pwm_duty(FTM0,FTM_CH0,duty_out+500);

}

void findbound(uint8(* img)[CAMERA_W])
{
    uint8_t j;
    int8 step1 = 0;
    int8 steplf=0;
    int8 steprf=0;
    uint8_t out_countl=0;
    uint8_t out_countr=0;
    leftBoundlast=0;
    whiteline=0;
    blackline=0;
    whitelineL=0;
    whitelineR=0;
    whitelineLf=0;
    whitelineRf=0;
//    whitelineLcounter=0;
//    whitelineRcounter=0;
    rightBoundlast=79;
 // uint8_t i,h;
  static uint8 leftFind;			        // 用来标记左黑线是否找到
  static uint8 rightFind;				// 用来标记右黑线是否找到
  static int16 Line;
//  static int counter_allowL=0;
//  static int counter_allowR=0;
  int8 temp1;
  int8 temp2;
  for(Line = CAMERA_H-1;Line >=0;Line--)
  {
    rightFind=0;
    leftFind=0;
    whitelineL_all=0;
    whitelineR_all=0;
    for(temp1 = 0;temp1 < CAMERA_W-3;temp1++)
    {
       if(leftFind == 0
        &&img[Line][temp1] == IMG_BLACK
        &&img[Line][temp1+1] == IMG_BLACK   // findLine 行数
        &&img[Line][temp1+2] == IMG_WHITE     // 39黑，40、41、42白，findline为39            
        &&img[Line][temp1+3] ==IMG_WHITE   //              
       )   //              
        {
            leftBound[Line] = temp1+2;  //保存左边界列数            
            leftFind = 1;                          //left line find,the flag set
         }
       if(rightFind == 0
        &&img[Line][temp1] == IMG_WHITE
        &&img[Line][temp1+1] == IMG_WHITE     // findLine 行数             //40黑，39、38、37白，findline为40
        &&img[Line][temp1+2] == IMG_BLACK
        &&img[Line][temp1+3] == IMG_BLACK)
        {
            rightBound[Line] = temp1+2;   //保存右边界列数            
            rightFind = 1;      
              //right line has been finded,the flag set
         }
           if((Line==40||Line==41||Line==42)
                  &&/*img[findLine][CENTER_POINT-temp-1] == IMG_WHITE  // findLine 行数
                  &&img[findLine][CENTER_POINT-temp] == IMG_BLACK     // 39黑，40、41、42白，findline为39            
                  &&img[findLine][CENTER_POINT-temp+1] == IMG_BLACK   //              
                  &&img[findLine][CENTER_POINT-temp+2] == IMG_BLACK*/
                   rightBound[Line]<40 )   //
              {
                boundry_left=1;
              //move_forwardl(150);
              }
          if((Line==40||Line==41||Line==42)
                  &&/*img[findLine][CENTER_POINT+temp] == IMG_WHITE     // findLine 行数  
                  &&img[findLine][CENTER_POINT+temp-1] == IMG_BLACK   //40黑，39、38、37白，findline为40
                  &&img[findLine][CENTER_POINT+temp-2] == IMG_BLACK
                  &&img[findLine][CENTER_POINT+temp-3] == IMG_BLACK*/
                    leftBound[Line]>40)
               {
                 boundry_right=1;
               //move_forwardl(150);
               }       
         if(leftFind == 1 &&rightFind == 1)
         {
           if((leftBound[Line]-leftBoundlast)>=40||(leftBound[Line]-leftBoundlast)<=-40)
           {
             leftFind = 0;
             continue;
           }
           if(rightBound[Line]>=leftBound[Line])
           {
             break;
           }
           else
           {
             rightFind = 0;
             continue;
           }
         }
       if(Line==30)
       {
       if(img[Line][temp1] == IMG_WHITE
        &&img[Line][temp1+1] == IMG_WHITE     // findLine 行数             //40黑，39、38、37白，findline为40
        &&img[Line][temp1+2] == IMG_WHITE
        &&img[Line][temp1+3] == IMG_WHITE)
       {
         whiteline++;
       }
       }
       if(Line==50)
       {
       if(img[Line][temp1] == IMG_WHITE
        &&img[Line][temp1+1] == IMG_WHITE     // findLine 行数             //40黑，39、38、37白，findline为40
        &&img[Line][temp1+2] == IMG_WHITE
        &&img[Line][temp1+3] == IMG_WHITE)
       {
         blackline++;
       }
       }
       
    }
        if(leftFind == 0)
        {
          if(leftBoundlast>=40)
          {
            leftBound[Line]=79;
          }
          else
          {
            leftBound[Line]=0;
          }
        }
        if(rightFind == 0)	
       {
          if(rightBoundlast>=40)
          {
            rightBound[Line]=79;
          }
          else
          {
            rightBound[Line]=0;
          }
        }
       // centerLine[Line] = (leftBound[Line]+rightBound[Line])/2;
        if(img[Line][CENTER_POINT]==IMG_WHITE)
        {
         out_countl++;
        }
        if(img[Line][CENTER_POINT]==IMG_WHITE)
        {
         out_countr++;
        }

    if(Line==43)
    {
     step1=0;
       for(temp2=0;temp2<CAMERA_W;temp2++)
       {
         if((img[Line][temp2+1] == IMG_BLACK&&img[Line][temp2] == IMG_WHITE)||(img[Line][temp2+1] == IMG_WHITE&&img[Line][temp2] == IMG_BLACK))
         {
             step1++;
         }

       }
    stepup=step1;
    }
    if(Line==30)
    {
      steplf=0;
      steprf=0;
       for(temp2=0;temp2<CAMERA_W/2;temp2++)
       {
         if((img[Line][temp2+1] == IMG_BLACK&&img[Line][temp2] == IMG_WHITE)||(img[Line][temp2+1] == IMG_WHITE&&img[Line][temp2] == IMG_BLACK))
         {
             steplf++;
         }

       }
       for(temp2=CAMERA_W/2;temp2<CAMERA_W;temp2++)
       {
         if((img[Line][temp2+1] == IMG_BLACK&&img[Line][temp2] == IMG_WHITE)||(img[Line][temp2+1] == IMG_WHITE&&img[Line][temp2] == IMG_BLACK))
         {
             steprf++;
         }

       }
       step_30_L=steplf;
       step_30_R=steprf;
    }
    if(Line==43)//change
    {
       for(temp2=0;temp2<CAMERA_W/2;temp2++)
       {
         if(img[Line][temp2] == IMG_WHITE)
         {
             whitelineLf++;
         }
       }
       for(temp2=CAMERA_W/2;temp2<CAMERA_W;temp2++)
       {
         if(img[Line][temp2] == IMG_WHITE)
         {
             whitelineRf++;
         }
       }       
    }

    if(Line==50&&Roundabout_flag!=0)
    {
       for(temp2=0;temp2<CAMERA_W/2;temp2++)
       {
         if(img[Line][temp2] == IMG_WHITE)
         {
             whitelineL++;
         }
       }
       if(whitelineL<40&&counter_allowL==0)
       {
         counter_allowL=1;
       }
       if(whitelineL>=40&&counter_allowL==1)
       {
         whitelineLcounter++;
         counter_allowL=0;
       }

       for(temp2=CAMERA_W/2;temp2<CAMERA_W;temp2++)
       {
         if(img[Line][temp2] == IMG_WHITE)
         {
             whitelineR++;
         }
       }
       if(whitelineR<38&&counter_allowR==0)
       {
         counter_allowR=1;
       }
       if(whitelineR>=38&&counter_allowR==1)
       {
         whitelineRcounter++;
         counter_allowR=0;
       }       
    }
    for(temp2=0;temp2<CAMERA_W/2;temp2++)
    {
      if(img[Line][temp2] == IMG_WHITE)
        {
             whitelineL_all++;
        }
    }
     if(whitelineL_all>=40&&Roundabout_flag==Roundabout_GoInto)
     {
       whitel=Line;
     }
     for(temp2=CAMERA_W/2;temp2<CAMERA_W;temp2++)
     {
       if(img[Line][temp2] == IMG_WHITE)
       {
           whitelineR_all++;
       }
     }
     if(whitelineR_all>=40&&Roundabout_flag==Roundabout_GoInto)
     {
       whiter=Line;
     } 
    //change
  if(Roundabout_flag==0)
  {
    whitelineLcounter=0;
    whitelineRcounter=0;
  }

  if(Line==30)
  {
    if(step_30_L==3&&img[Line][1] == IMG_BLACK&&img[Line][2] == IMG_BLACK&&img[Line][3] == IMG_BLACK&&step_30_R==1&&img[Line][76] == IMG_BLACK&&img[Line][77] == IMG_BLACK&&img[Line][78] == IMG_BLACK)
    {
      Obs_L=1;
    }
    if(step_30_R==3&&img[Line][1] == IMG_BLACK&&img[Line][2] == IMG_BLACK&&img[Line][3] == IMG_BLACK&&step_30_L==1&&img[Line][76] == IMG_BLACK&&img[Line][77] == IMG_BLACK&&img[Line][78] == IMG_BLACK)
    {
      Obs_R=1;
    }
  }
  if(stepup>=10)
  {
    startline=1;
  }
  else
  {
    runsignal=0;
  }
  leftBoundlast=leftBound[Line];
  rightBoundlast=rightBound[Line];
  }
  line_countl=out_countl;
  line_countr=out_countr;
  out_countl=0;
  out_countr=0;
  if(inflection_right_sink==0&&HD1l==1&&HD2l==1)
  {
    no_wanl=1;
  }
  else no_wanl=0;
  if(inflection_left_sink==0&&region2r==1)
  {
    no_wanr=1;
  }
  else no_wanr=0;
//      for(j=0;j<CAMERA_H;j++)
//         {
//           img[j][centerLine[j]]=0;
//           img[j][leftLine[j]]=0;
//           img[j][rightLine[j]]=0;
//         }
}

void findCenterLine(uint8 (* img)[CAMERA_W])
{
  
  uint8_t j;
  leftLine_last=0;
  rightLine_last=79;
 // uint8_t i,h;
  static uint8 leftFindFlag;			        // 用来标记左黑线是否找到
  static uint8 rightFindFlag;				// 用来标记右黑线是否找到
  static int16 leftCount;                                 //
  static int16 rightCount;
  static int16 findLine;
  int8 temp;
  // 前十行从中间往两边查找
  for(findLine = CAMERA_H-1;findLine >CAMERA_H-11;findLine--) 
  {  
         leftFindFlag = 0;
         rightFindFlag = 0;
         for(temp = 0;temp < CENTER_POINT;temp++)                //CENTER_POINT = 40  中心点   从左向中间扫描
         {
         // 寻找左黑线                                          
//               if((leftFindFlag == 0|| rightFindFlag == 0)                                  
//                  &&img[findLine][CENTER_POINT-temp-1] == IMG_BLACK   // findLine 行数
//                  &&img[findLine][CENTER_POINT-temp-2] == IMG_BLACK     // 39黑，40、41、42白，findline为39            
//                  &&img[findLine][CENTER_POINT-temp-3] == IMG_WHITE   //              
//                  &&img[findLine][CENTER_POINT-temp-4] == IMG_WHITE)   //              
//                  {
//                      rightLine[findLine] = CENTER_POINT-temp-2;  //保存左边界列数
//                      rightFindFlags = 1;                          //left line find,the flag set
//                   }
               if(leftFindFlag == 0
                  &&img[findLine][CENTER_POINT-temp-2] == IMG_BLACK
                  &&img[findLine][CENTER_POINT-temp-1] == IMG_BLACK   // findLine 行数
                  &&img[findLine][CENTER_POINT-temp] == IMG_WHITE     // 39黑，40、41、42白，findline为39            
                  &&img[findLine][CENTER_POINT-temp+1] ==IMG_WHITE   //              
                  &&img[findLine][CENTER_POINT-temp+2] == IMG_WHITE)   //              
//                  {
//                      leftLine[findLine] = CENTER_POINT-temp-1;  //保存左边界列数
//                      leftFindFlag = 1;                          //left line find,the flag set
//                   }
       {
             leftLine[findLine] = CENTER_POINT-temp-1;  //保存左边界列数
             leftLine_last=leftLine[findLine];
             leftFindFlag = 1; 
       }
        //寻找右黑线
//               if((leftFindFlag == 0|| rightFindFlag == 0)
//                  &&img[findLine][CENTER_POINT+temp] == IMG_BLACK     // findLine 行数  
//                  &&img[findLine][CENTER_POINT+temp+1] == IMG_BLACK   //40黑，39、38、37白，findline为40
//                  &&img[findLine][CENTER_POINT+temp+2] == IMG_WHITE
//                  &&img[findLine][CENTER_POINT+temp+3] == IMG_WHITE)
//                 {
//                      leftLine[findLine] = CENTER_POINT+temp+1;   //保存右边界列数
//                      leftFindFlags = 1;      
//                      //right line has been finded,the flag set
//                   }
               if(leftFindFlag == 0
                  &&img[findLine][CENTER_POINT+temp+1] == IMG_BLACK
                  &&img[findLine][CENTER_POINT+temp] == IMG_BLACK     // findLine 行数  
                  &&img[findLine][CENTER_POINT+temp-1] == IMG_WHITE   //40黑，39、38、37白，findline为40
                  &&img[findLine][CENTER_POINT+temp-2] == IMG_WHITE
                  &&img[findLine][CENTER_POINT+temp-3] == IMG_WHITE)
//                 {
//                      rightLine[findLine] = CENTER_POINT+temp;   //保存右边界列数
//                      rightFindFlag = 1;      
//                      //right line has been finded,the flag set
//                   }
       {
         rightLine[findLine] = CENTER_POINT+temp;
         rightLine_last=rightLine[findLine];
         rightFindFlag = 1;
       }
        if(leftFindFlag == 1 &&rightFindFlag == 1)
         {
           if(rightLine[findLine]>=leftLine[findLine])
           {
             break;
           }
           else
           {
             rightFindFlag = 0;
             continue;
           }
         }
//               if(leftFindFlag == 1 &&rightFindFlag == 1)
//                  break;
         }
        // 对未找到的黑线进行补全
//        if(leftFindFlag == 0)	leftLine[findLine] =  0;
//        if(rightFindFlag == 0)	rightLine[findLine] = CAMERA_W-1;
        if(leftFindFlag == 0)
        {
          if(leftLine_last>=40)
          {
            leftLine[findLine]=79;
          }
          else
          {
            leftLine[findLine]=0;
          }
        }
        if(rightFindFlag == 0)	
       {
          if(rightLine_last>=40)
          {
            rightLine[findLine]=79;
          }
          else
          {
            rightLine[findLine]=0;
          }
        }         
        // 对中线进行赋值
        centerLine[findLine] = (leftLine[findLine]+rightLine[findLine])/2;
  }
  // 十行后根据前面行位置查找黑线
  for(findLine = CAMERA_H-11; findLine >= 0; findLine--)
  {
         leftFindFlag = 0;
         rightFindFlag = 0;
         // 预测下一行黑线位置
         leftCount = createPoint(FIND_LEFT, findLine);//根据最小二乘法找到左边的点
         rightCount = createPoint(FIND_RIGHT, findLine);
         /* 在预测点的左右 FIND_COUNT 个点查找黑线位置 */
         // 寻找左黑线
         for(temp = 0; temp < FIND_COUNT*2+1; temp++)
         {
           if(leftFindFlag != 0)
             break;
           else if((leftCount-temp+FIND_COUNT)+3 > CAMERA_W-1)  //>79
             continue;
           else if((leftCount-temp+FIND_COUNT) < 0)
             break;
           else if( img[findLine][leftCount-temp+FIND_COUNT-1] == IMG_BLACK
                        && img[findLine][leftCount-temp+FIND_COUNT] == IMG_BLACK                   
			&& img[findLine][leftCount-temp+FIND_COUNT+1] == IMG_WHITE
			&& img[findLine][leftCount-temp+FIND_COUNT+2] == IMG_WHITE
			&& img[findLine][leftCount-temp+FIND_COUNT+3] == IMG_WHITE)
           {
             leftLine[findLine] = leftCount-temp+FIND_COUNT;
             leftFindFlag = 1;
           }
         }
           // 寻找右黑线
           for(temp = 0; temp < FIND_COUNT*2+1; temp++)
           {
             if(rightFindFlag != 0)
               break;
             else if((rightCount+temp-FIND_COUNT)-3 < 0)
	       continue;
             else if(rightCount+temp-FIND_COUNT > CAMERA_W-1)
               break;
             else if(img[findLine][rightCount+temp-FIND_COUNT+1] == IMG_BLACK
                        && img[findLine][rightCount+temp-FIND_COUNT] == IMG_BLACK
			&& img[findLine][rightCount+temp-FIND_COUNT-1] == IMG_WHITE
			&& img[findLine][rightCount+temp-FIND_COUNT-2] == IMG_WHITE
			&& img[findLine][rightCount+temp-FIND_COUNT-3] == IMG_WHITE)
             {
               rightLine[findLine] = rightCount+temp-FIND_COUNT;
               rightFindFlag=1;
             }
           }
           // 补全未找到的左右黑线
         if(leftFindFlag == 0)     leftLine[findLine] = leftCount;
         if(rightFindFlag == 0)    rightLine[findLine] = rightCount;
         /* 查找中线 */
         if(leftLine[findLine] > 0 && rightLine[findLine] < CAMERA_W-1)
           {// 左右黑线都存在则取左右黑线中值作为黑线值
           centerLine[findLine] = (leftLine[findLine]+rightLine[findLine])/2;
           //temp_last = (leftLine[findLine]+rightLine[findLine])/2 ;
           }
         // 左黑线超出范围
         else if(leftLine[findLine] <= 0 && rightLine[findLine] < CAMERA_W-1)   //左边界不存在，右边界存在
         {
           temp = centerLine[findLine+1] - (rightLine[findLine+1] - rightLine[findLine]);// 根据右黑线的偏移量来确定中线

            if(temp <= 0)
            {
              centerLine[findLine] = 0;   // 中线超出范围则跳出循环，记录该行为转向行
              //centerLine[CAMERA_H]=findLine;
              break;
            }
            else
             centerLine[findLine] = temp;
             //temp_last = temp ;
         }
         // 右黑线超出范围
         else if(leftLine[findLine] > 0 && rightLine[findLine] >= CAMERA_W-1)  //右边界不存在，左边界存在
         {
            temp = centerLine[findLine+1] + (leftLine[findLine] - leftLine[findLine+1]);// 根据左黑线的偏移量来确定中线
           if(temp >= CAMERA_W-1)
           {
             centerLine[findLine] = CAMERA_W-1;    //// 中线超出范围则跳出循环，记录该行为转向行
             //centerLine[CAMERA_H]=findLine;
             break;
           }
           else
             centerLine[findLine] =temp; 
            //  temp_last = temp ;
         }
         // 左右黑线都超出范围
         else
         { 
           cross = 1;
           temp = createPoint(FIND_CENTER, findLine);// 根据最小二乘法补全中线
	   //temp = centerLine[findLine+1] + (rightLine[findLine] - rightLine[findLine+1]);   // 根据中线偏移量补全中线
           if(temp <= 0)
           {
             centerLine[findLine] = 0;// 中线超出范围则跳出循环，记录该行为转向行
             //centerLine[CAMERA_H]=findLine;
	     break;
           }
           else if(temp >= CAMERA_W-1)
           {
             centerLine[findLine] = CAMERA_W-1;// 中线超出范围则跳出循环，记录该行为转向行
             //centerLine[CAMERA_H]=findLine;
             break;
           }          
         }

//         if(centerLine[CAMERA_H]==1)
//           break;
  //       i++;
             
 //   #endif       
  }
  if(findLine<0 && centerLine[0]<0)
    centerLine[0] = 0;
  else if(findLine<0 && centerLine[0]>CAMERA_W-1)
    centerLine[0]=CAMERA_W-1;
  // 最后一个元素用来记录转向行
  centerLine[CAMERA_H] = (findLine < 0) ? 0 : findLine;
 // return (int16 *)centerLine;
  for(j=0;j<CAMERA_H;j++)
         {
           img[j][centerLine[j]]=0;
           img[j][leftLine[j]]=0;
           img[j][rightLine[j]]=0;
         }
}

/* 利用最小二乘法生成需要补全的点 */
int16 createPoint(int type, int line)
{
  int16 *linePointer;
  int8 tmp = 0;
  double sumX = 0;
  double sumY = 0;
  double averageX = 0;
  double averageY = 0;
  double sumUp = 0;
  double sumDown = 0;
  double parameterY; //参数A
  double parameterX;
  if(type == FIND_LEFT)
    linePointer = &leftLine[line];
  else if(type == FIND_RIGHT)
    linePointer = &rightLine[line];
  else
    linePointer = &centerLine[line];
  // 取邻近的 POINT_COUNT 个点进行拟合   取邻近的10个点进行拟合
  while(++tmp <= POINT_COUNT)      //tmp前自増    <=10     POINT_COUNT=10
  {
    sumX += (line+tmp);       //10行之和
    sumY += linePointer[tmp]; //10列之和
  }
  --tmp;
  averageX = sumX/tmp;//行数平均值
  averageY = sumY/tmp;//列数平均值
  do
  {
    sumUp += (linePointer[tmp]-averageY) * (line+tmp-averageX); //sum[(X-Xav)*(Y-Yav)]=sum[△X*△Y]
    sumDown += (line+tmp-averageX) * (line+tmp-averageX);       //sum[(X-Xav)^2]=sum[△X^2]
  }while(--tmp > 0);
  if(sumDown == 0)
    parameterX = 0;
  else
    parameterX = sumUp/sumDown;
  parameterY=averageY-parameterX*averageX;
  return (int16)(parameterY+parameterX*line+0.5);
    
}

void cross_wall()
{
  wall_turnright = 1;
   if(/*0x03<step1<0x06 &&*/whitelineLf<=20)
  {
    //wall_turnright = 1;
  //  if (whitelineLf<=20)
  //  {
      SD5_PWM=400;
  //  }
  //  else
  //  {
      //SD5_PWM = duoji_last;
      //wall_turnright = 0 ;
   // }
  }
  else if(whitelineLf>20/* && wall_turnright == 1*/)
  {
    if (whiteline>=38)
    {
      SD5_PWM= 600;
      wall_turnright = 0;
      wall_turnleft = 0;
    }
    else
    {
      SD5_PWM = 485 ;
    }
  }
  else
  {
    //SD5_PWM = duoji_last;
    SD5_Control();
    wall_turnright = 0;
    //plus = 1 
  }
}
 

