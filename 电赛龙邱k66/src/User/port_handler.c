#include "include.h"

float    Gyio_X,Gyio_Y,Gyio_Z;                           //三轴角加速度
float    angle_X_1,angle_Y_1,angle_Z_1;                        //三轴欧拉角
float  mv_error_x,mv_error_y;
char Re_buf[4];

extern float  distance_taget;
extern int    problems;
extern int    now_pwmx ;
extern int    MODE;
extern int pwm2;
extern int   k_error;
int  ceshi;

int duoji_pwm_X;
int duoji_pwm_Y=0;
int jiguang_distance;                                      //激光测距
int fangpao_flag=0;                                          //发炮标志位
float shuru_angle_X=0;
/******************************
定时器中断
*****************************/
void PIT0_Interrupt()
{
    static int flag_fangxiang=1;
    static int time_remuber=0;
    static int time_remuber_2=0;
    PIT_Flag_Clear(PIT0);                                             //清中断标志位 
    duoji_pwm_X=duoji_PID_X();
    duoji_pwm_Y=duoji_PID_Y(distance_taget);
    if(MODE==1)                                                      //模式1，输入距离，自动发射
    {
      duoji_pwm_X=0;
      Set_Servo_pwm(duoji_pwm_X, duoji_pwm_Y); 
      if(problems==50)
         fangpao_flag=1;
    }
    else if(MODE==2)                                                //模式2，输入距离与角度,拨动8（马上再拨回去）延时2秒发射
    {
      int pwmmm_x,pwmmm_y;
      pwmmm_x=2385-shuru_angle_X*(520.0/30);
      pwmmm_y=2343+duoji_pwm_Y;
      if(problems==50)
      {
        FTM_PWM_Duty(FTM3, FTM_CH1,pwmmm_x);
        FTM_PWM_Duty(FTM3, FTM_CH0,pwmmm_y);
        time_remuber_2++;
        if(time_remuber_2>200)
        {
          fangpao_flag=1;
          time_remuber_2=0;
        }
      }
      
    }
    else if(MODE==3)                                                 //模式3，自动寻找目标，自动发射
    {
      static int num=0;
      static int jj=1;
      static int pwm_yy_2=2343;
     if(problems==50)
     {
      if(fabs(mv_error_x)<2)
      {
        num++;
      }
      Set_Servo_pwm(duoji_pwm_X, duoji_pwm_Y); 
      if(num>100&&jj==1)
      {
        pwm_yy_2=duoji_PID_Y(jiguang_distance*0.01-0.3);                //改过，还没测，原来是减0.39
        jj=0;
      }
      else if(num>200)
        fangpao_flag=1;
      if(num>400)
        num=0;
      Set_Servo_pwm(duoji_pwm_X, pwm_yy_2);
     }
    }
    else if(MODE==4)                                                 //模式4，拨8开始不断摇头，找到目标就发射
    {
      static int  pwm_yy;
      if(problems==50)
      {
        pwm_yy=duoji_PID_Y(2.5);   
        if(now_pwmx<2868&&now_pwmx>2860)
          flag_fangxiang=0;
        else if(now_pwmx>1820&&now_pwmx<1826)
          flag_fangxiang=1;
        if(flag_fangxiang)
          duoji_pwm_X=3;
        else if(!flag_fangxiang)
          duoji_pwm_X=-3;
        
        if(fabs(mv_error_x)<2)
        {
            fangpao_flag=1;
        }
        Set_Servo_pwm(duoji_pwm_X, pwm_yy);
      }
      else
      {
        FTM_PWM_Duty(FTM3, FTM_CH1,2863);
        FTM_PWM_Duty(FTM3, FTM_CH0,2343); 
        pwm2=2863;
      }
      
    }
    else if(MODE==5)  
    {
      static int flag=0;
      static int ii=1;
      static int  pwm_yy;
      if(problems==50)
      {
        if(now_pwmx<2868&&now_pwmx>2860)
          flag_fangxiang=0;
        else if(now_pwmx>1820&&now_pwmx<1826)
          flag_fangxiang=1;
        if(flag_fangxiang)
          duoji_pwm_X=3;
        else if(!flag_fangxiang)
          duoji_pwm_X=-3;
        
        if(fabs(mv_error_x)<2)
        {
          if(flag==0)
            flag=1;
          else if(flag==2)
          {
            fangpao_flag=1;
            flag=0;
          }
          if(flag==1&&ii==1)
          {
            pwm_yy=duoji_PID_Y(jiguang_distance*0.01-0.39);   
            ii=0;
          }
        }
        if(flag==1&&fabs(mv_error_x)>10)
        {
          flag=2;
        }
        
        Set_Servo_pwm(duoji_pwm_X, pwm_yy);
      }
      else
      {
        FTM_PWM_Duty(FTM3, FTM_CH1,2863);
        FTM_PWM_Duty(FTM3, FTM_CH0,2343); 
        pwm2=2863;
        flag=0;
      }
    }
    if(fangpao_flag==1)
    {
      time_remuber++;
      time_remuber_2=0;
      if(time_remuber>100)
      {
        time_remuber=0;
        fangpao_flag=0;
        if(MODE!=4&&MODE!=5)
          problems=0;
      }
    }
} 



/**********************************
  GY61串口中断
*******************************/
void UART2_IRQHandler(void)                                      
{ 
  
    static unsigned char counter=0; 
    static   short sign=0;
    static unsigned char Re_buf_1[11];
    float    Acce[3],Gyio[3],angle[3];
    
    
    Re_buf_1[counter]=UART_Get_Char(UART_2);                    //无限等待接受1个字节
    
    if(counter==0&&Re_buf_1[0]!=0x55)
      return;                                         //第 0 号数据不是帧头，跳过 
    counter++;
    if(counter==11)                                   //接收到 11 个数据 
    { 
      counter=0;                                     //重新赋值，准备下一帧数据的接收
      sign=1; 
    }                         
    
    if(sign) 
    { 
      sign=0; 
      if(Re_buf_1[0]==0x55)                                                       //检查帧头 
      {
        switch(Re_buf_1[1]) 
        {
          case 0x51:                                                            //加速度计
            Acce[0]=((short)(Re_buf_1[3]<<8|Re_buf_1[2]))/32768.0*16; 
            Acce[1]=((short)(Re_buf_1[5]<<8|Re_buf_1[4]))/32768.0*16;  
            Acce[2]=((short)(Re_buf_1[7]<<8|Re_buf_1[6]))/32768.0*16;
             break; 
          case 0x52:                                                            //陀螺仪
            Gyio[0]=((short)(Re_buf_1[3]<<8|Re_buf_1[2]))/32768.0*2000; 
            Gyio[1]=((short)(Re_buf_1[5]<<8|Re_buf_1[4]))/32768.0*2000; 
            Gyio[2]=((short)(Re_buf_1[7]<<8|Re_buf_1[6]))/32768.0*2000; 
            Gyio_X=Gyio[0];
            Gyio_Y=Gyio[1];
            Gyio_Z=Gyio[2];
             break; 
          case 0x53:                                                            //欧拉角
            angle[0]=((short)(Re_buf_1[3]<<8|Re_buf_1[2]))/32768.0*180; 
            angle[1]=((short)(Re_buf_1[5]<<8|Re_buf_1[4]))/32768.0*180; 
            angle[2]=((short)(Re_buf_1[7]<<8|Re_buf_1[6]))/32768.0*180; 
            angle_X_1=angle[0];
            angle_Y_1=angle[1];
            angle_Z_1=angle[2];
             break; 
        }
      }
    }
}




/***********************
激光测距串口中断
***********************/
void UART3_IRQHandler(void)
{
    static unsigned char counter=0; 
    static   short sign=0;
    static unsigned char Re_buf_2[4];


    
    Re_buf_2[counter]=UART_Get_Char(UART_3); 
    if(counter==0&&Re_buf_2[0]!=0x59)
      return;                                         //第 0 号数据不是帧头，跳过   
    if(counter==1&&Re_buf_2[1]!=0x59)
    {
      counter=0;
      return;                                         //第 1号数据不是帧头，跳过 
    }
    counter++;
    if(counter==4)
    {
      sign=1;
      counter=0;
    }
    if(sign)
    {
      jiguang_distance=(Re_buf_2[3]<<8|Re_buf_2[2]);
    }
    if((jiguang_distance%5)>=3)
      jiguang_distance=5*(jiguang_distance/5)+5;
    else if((jiguang_distance%5)<3&&(jiguang_distance%5)!=0)
      jiguang_distance=5*(jiguang_distance/5);
}



/**************************
openmv串口中断
**************************/
void UART4_IRQHandler(void)
{
  static unsigned char counter=0; 
  static   short sign=0;
  static  float   mv_error_x_ago;
  Re_buf[counter]=UART_Get_Char(UART_4);
  if(counter==0&&Re_buf[0]!='a')
     return;                                         //第 0 号数据不是帧头，跳过 
  counter++;
  if(counter==4)                                   //接收到 11 个数据 
    { 
      counter=0;                                     //重新赋值，准备下一帧数据的接收
      sign=1; 
    }   
  if(sign)
  {
     sign=0; 
     mv_error_x_ago=mv_error_x;

     if(Re_buf[1]=='-')
        mv_error_x=(short)((Re_buf[2]-48)*10+(Re_buf[3]-48))*-1+3;
     else
        mv_error_x=(short)((Re_buf[2]-48)*10+(Re_buf[3]-48))+3;

     if(mv_error_x>80||mv_error_x<-80||Re_buf[2]=='a'||Re_buf[2]=='+'||Re_buf[2]=='-'||Re_buf[3]=='a'||Re_buf[3]=='+'||Re_buf[3]=='-')
        mv_error_x=mv_error_x_ago;

  }
   
}