/*LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
��ƽ    ̨�������������ܿƼ�MK66FX1M0VLQ18������
����    д��CHIUSIR
����    ע��
������汾��V1.0
�������¡�2017��01��20��
�������Ϣ�ο����е�ַ��
����    վ��http://www.lqist.cn
���Ա����̡�http://shop36265907.taobao.com
���������䡿chiusir@163.com
QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ*/
#include "include.h"
#include "math.h"




extern float K_Gyro_y;
extern float Angle_Balance,Gyro_Balance,Gyro_Turn;              //ƽ����� ƽ�������� ת��������
extern float pitch;
extern int Encoder_Left;                                   //���ұ��������������
extern int Encoder_Right;                            
extern float  mv_error_x;
extern float  mv_error_y;

int bianmaqi[2] ;                                           //�洢������
float angle_X;                                                //X��Ƕ�
float    angle_X_ago ;                                       //��һ��X��Ƕ�
float   icm_gyro_y_ago;
float   pitch_g_ago;
float    kaerman_angle;



float P_roportion_IN=0.4;                              //1
float I_ntegral_IN=0.0;
float D_ifferential_IN=2.0;                           //10


float   speed_v0=5.69;                                            //���ٶ�
float   angle_paotai;                                             //��̨���
float   hight_paotai=0.1;                                       //��̨�����߶�
float   distance_taget=0.0;                                      //��ˮƽ����


float    jianpan_AD_max=2349.0;                                       //����AD���ֵ
float    jianpan_AD_min=346.0;                                       //����AD��Сֵ
float     bibiibli;   

int16  AD_valu[5];
int    Value_get_jianpan=-1;                                //���̹��ܱ�ʶ
int    problems=1;                                          //����ѡ�� 

int   now_pwmx ;
int   now_pwmy ;
int   MODE=1;
int   k_error=0;                                            //����ƫ����� 


float  value_jianpan;
int pwm2=2385;                                                  //�¶������ʽ
float    angleangle=0;
int  adadad[8];
/***********************
ADC��ʼ��
***********************/
void ADC_ready()
{
    ADC_Init(ADC_1);
    ADC_Init(ADC_0);
    ADC_Start(ADC_0,adc_num1,adc_digit); 
    ADC_Start(ADC_1,adc_num2,adc_digit);
    ADC_Start(ADC_1,adc_num3,adc_digit);
    ADC_Start(ADC_1,adc_num4,adc_digit);
    ADC_Start(ADC_1,adc_num5,adc_digit);
    ADC_Start(ADC_1,adc_num7,adc_digit);
    ADC_Start(ADC_1,adc_num8,adc_digit);
}


/*******************
��ʼ������ͨ��PWM
********************/
/***********************
�϶��  2343    781    3906   ��ƫ30�� 1822
�¶��  2343    781    3906   ��ƫ30�� 1822   ��ƫ30�� 2864
**************************/
void Motor_Init(void)
{
    Servo_FTM_PWM_Init(FTM3,FTM_CH0,100*1000000/64/100,2343);
    Servo_FTM_PWM_Init(FTM3,FTM_CH1,100*1000000/64/100,2385);
}





/*************************
����GPIO��ʼ��
************************/
void  bomakaiguan()
{
    GPIO_Init(PTC,0,GPI,1);               //0
    GPIO_Init(PTC,1,GPI,1);              //1
    GPIO_Init(PTC,2,GPI,1);              //2
    GPIO_Init(PTC,3,GPI,1);             //3
    GPIO_Init(PTC,4,GPI,1);             //4
    GPIO_Init(PTC,5,GPI,1);               //5
    GPIO_Init(PTC,6,GPI,1);              //6
    GPIO_Init(PTC,7,GPI,1);              //7
    GPIO_Init(PTC,7,GPI,1); 
    GPIO_Init(PTC,18,GPO,0);             //�ŵ�io
    GPIO_Init(PTC,19,GPO,0);             //���io
}



/***************************
���뿪�ع��ܺ���
***************************/
void  bomakaiguan_function()
{
  int    i;
  short  sum_bomakaiguan=0;    

  adadad[0]=GPIO_Get(PTC0)*1;
  adadad[1]=GPIO_Get(PTC1)*10;
  adadad[2]=GPIO_Get(PTC2)*100;
  adadad[3]=GPIO_Get(PTC3)*1000;
  adadad[4]=GPIO_Get(PTC4)*2;
  adadad[5]=GPIO_Get(PTC5)*20;
  adadad[6]=GPIO_Get(PTC6)*200;
  adadad[7]=GPIO_Get(PTC7)*2000;

    for(i=0;i<8;i++)
     sum_bomakaiguan+=adadad[i];
       
    if(sum_bomakaiguan==3)                                  //51                       
    {
      problems=10;
    } 
    else if(sum_bomakaiguan==12)                           //52                 
    {
      problems=20;
    }
    else if(sum_bomakaiguan==102)                         //53
    { 
      problems=30;
    }
    else if(sum_bomakaiguan==1002)                        //54
    {
      problems=40;
    }    
    else if(sum_bomakaiguan==21)                       //61                                             
    {
      problems=31;
    }
    else if(sum_bomakaiguan==30)                       //62   
    {
      problems=32;
    }
    else if(sum_bomakaiguan==120)                       //63     
    {
      problems=33;
    }
    else if(sum_bomakaiguan==1020)                       //64
    {
      problems=34;
    }
    else if(sum_bomakaiguan==2021)                       //681      ��ť�������
    {
      problems=311;
    }
    else if(sum_bomakaiguan==2030)                       //682      ��ť����Ƕ�
    {
      problems=321;
    }
    else if(sum_bomakaiguan==2030)                       //683      �����������
    {
      problems=331;
    }
    else if(sum_bomakaiguan==2030)                       //684       ��ť�������
    {
      problems=341;
    }
    else if(sum_bomakaiguan==201)                           //71                 
    {
      problems=41;
      MODE=1;
    }
    else if(sum_bomakaiguan==210)                           //72                 
    {
      problems=42;
      MODE=2;
    }
    else if(sum_bomakaiguan==300)                           //73                 
    {
      problems=43;
      MODE=3;
    }
    else if(sum_bomakaiguan==1200)                           //74                 
    {
      problems=44;
      MODE=4;
    }
    else  if(sum_bomakaiguan==2000)                          //8
    {
      problems=50;
    }
    else if(sum_bomakaiguan==2001)                           //18     �Ȳ�1�ٲ�8
    {
      MODE=5;
      problems=45;
    }
}

/***************************************
�������뺯��  
��˳�������λ��ʮ��λ���ٷ�λ����λ��
����11������������
****************************************/
float  scanf_distance_taget()
{
  float distance=0.0;
  int   num_times=1;
  while(1)
  {
     Value_get_jianpan= Key_detect();
      if(Value_get_jianpan!=-1&&num_times==1&&Value_get_jianpan!=10)
      {
        distance+=Value_get_jianpan;
        num_times++;
        time_delay_ms(300);
      }     
      else if(Value_get_jianpan!=-1&&num_times==2&&Value_get_jianpan!=10)
      {
        distance+=(Value_get_jianpan)*0.1;
        num_times++;
        time_delay_ms(300);
      }  
      else if(Value_get_jianpan!=-1&&num_times==3&&Value_get_jianpan!=10)
      {
        distance+=(Value_get_jianpan)*0.01;
        num_times++;
        time_delay_ms(300);
      }  
      
      if(Value_get_jianpan==10)  
      {
        time_delay_ms(300);
        return distance;
      }
      else if(Value_get_jianpan==20)
      {
         time_delay_ms(300);
         return -distance;
      }
  }
}
/*************************
�������
ǰʮ������Ӧ0~9
��11����Ϊ�˳�����
*************************/
int  Key_detect()
{
  int  Value_get_jianpan_1;
  int  value_jianpan_2;
  int  error;
  
  error=jianpan_AD_max-jianpan_AD_min;
  AD_valu[0]=ADC_Ave(ADC_0,adc_num1,adc_digit,10);
  value_jianpan=(AD_valu[0]-jianpan_AD_min)/error;
  time_delay_ms(10);
  AD_valu[0]=ADC_Ave(ADC_0,adc_num1,adc_digit,10);
  value_jianpan_2=(AD_valu[0]-jianpan_AD_min)/error;
  if(fabs(value_jianpan-value_jianpan_2)>2)
     return -1;
  else
  {
    if(value_jianpan<0)
        value_jianpan=0;
    if(value_jianpan<1.2&&value_jianpan>0.93)                             
      Value_get_jianpan_1=9; 
    else if(value_jianpan<0.93&&value_jianpan>0.83)                             
      Value_get_jianpan_1=6;
    else if(value_jianpan<0.83&&value_jianpan>0.76)
      Value_get_jianpan_1=3;
    else if(value_jianpan<0.76&&value_jianpan>0.715)
      Value_get_jianpan_1=20;                                        
    else if(value_jianpan<0.715&&value_jianpan>0.65)
      Value_get_jianpan_1=10;                                        
    else if(value_jianpan<0.65&&value_jianpan>0.60)
      Value_get_jianpan_1=2;
    else if(value_jianpan<0.6&&value_jianpan>0.55)
      Value_get_jianpan_1=5;
    else if(value_jianpan<0.55&&value_jianpan>0.45)
      Value_get_jianpan_1=8;
    else if(value_jianpan<0.45&&value_jianpan>0.30)
      Value_get_jianpan_1=7;
    else if(value_jianpan<0.3&&value_jianpan>0.2)
      Value_get_jianpan_1=4;
    else if(value_jianpan<0.2&&value_jianpan>0.1)
      Value_get_jianpan_1=1;
    else if(value_jianpan<0.05&&value_jianpan>=0)
      Value_get_jianpan_1=0;
    else
      Value_get_jianpan_1=-1;
    return Value_get_jianpan_1;
  }
}

/***************************
���̽���
***************************/
int  jianpanjiaozheng_max()
{
  int  value_jianpan_1;
  int  value_jianpan_2;
  
  AD_valu[0]=ADC_Ave(ADC_0,adc_num1,adc_digit,10);
  value_jianpan_1= AD_valu[0];
  time_delay_ms(10);
  AD_valu[0]=ADC_Ave(ADC_0,adc_num1,adc_digit,10);
  value_jianpan_2=AD_valu[0];
  if(abs(value_jianpan_1-value_jianpan_2)>10)
     return 0;
  else
  {
    jianpan_AD_max=AD_valu[0];
    return 1;
  }
}
int   jianpanjiaozheng_min()
{
  int  value_jianpan_1;
  int  value_jianpan_2;
  
  AD_valu[0]=ADC_Ave(ADC_0,adc_num1,adc_digit,10);
  value_jianpan_1= AD_valu[0];
  time_delay_ms(10);
  AD_valu[0]=ADC_Ave(ADC_0,adc_num1,adc_digit,10);
  value_jianpan_2=AD_valu[0];
  if(abs(value_jianpan_1-value_jianpan_2)>10)
     return 0;
  else
  {
    jianpan_AD_min=AD_valu[0];
    return 1;
  }
}


/****************************
lm20602�ǶȻ�ȡ
****************************/

void Get_Dip_Angle(void)
{
    float k=0.0;                                  //kalman�˲��ݴ�
    extern float pitch_g;
    angle_X_ago=angle_X;
    icm_gyro_y_ago=icm_gyro_y;
    pitch_g_ago=pitch_g;

    get_icm20602_accdata(); //��ȡ���ٶ�����
    get_icm20602_gyro();    //��ȡ����������

    
    pitch=atan2((float)icm_acc_x,(float)icm_acc_z)*180/PI;
    
    pitch_g+=(K_Gyro_y)*0.24*0.001*4;
    angle_X-=(icm_gyro_x-11)*0.12*0.001*4;
    Gyro_Balance=K_Gyro_y;                                                      //����ƽ����ٶ�  //���������̷���ת����Ҳ����ͨ�������������Ĳ���ʵ��
    
    k=Kalman_Filter(pitch,-1*((icm_gyro_y-4)/16.38));                        //�������˲��������
    kaerman_angle=k;
    

    
    bianmaqi[0]=FTM_AB_Get(FTM2);
    bianmaqi[1]=FTM_AB_Get(FTM1);
 
    
    Encoder_Left=-bianmaqi[0];                                //����Ϊǰ��+������Ϊ- ��ÿ?ms�ɼ�һ��    
    Encoder_Right=bianmaqi[1];                                 //����Ϊǰ��+������Ϊ-  


    Angle_Balance=k;                                          //����ƽ�����
    Gyro_Turn=icm_gyro_x;                                       //����ת����ٶ�    
    
}




int  duoji_PID_X()
{
   static int error_x_ago,error_x;
   static float integral=0;
   static float differential=0;
   float        integral_max=100.0; 
   int           OUT_PWM;
      error_x_ago=error_x;
      error_x=mv_error_x;
      differential=error_x-error_x_ago;
      integral+=error_x;
      if(abs(error_x)<2)
        integral=0;
      if(integral>integral_max)       integral=integral_max;
      else if(integral<-integral_max)  integral=-integral_max;
      OUT_PWM= P_roportion_IN*error_x + I_ntegral_IN*integral + D_ifferential_IN*differential;
      return  -OUT_PWM;
      
}


int  duoji_PID_Y(float S_1)
{
   static int OUT_PWM;
   float anglr_to_pwm=17.356;
   static float   angle_from_s1;
   S_1+=k_error;                                             //ʵ�ʼӾ���ƫ�����
   
   if(S_1>=3&&S_1<4)
      angle_from_s1=25.0;
   else if(S_1>=2.95&&S_1<3)
      angle_from_s1=23.1;
   else if(S_1>=2.9&&S_1<2.95)
      angle_from_s1=22.0;
   else if(S_1>=2.85&&S_1<2.9)
      angle_from_s1=21.0;
   else if(S_1>=2.8&&S_1<2.85)
      angle_from_s1=20.1;
   else if(S_1>=2.75&&S_1<2.8)
      angle_from_s1=19.2;
   else if(S_1>=2.7&&S_1<2.75)
      angle_from_s1=17.9;
   else if(S_1>=2.65&&S_1<2.7)
      angle_from_s1=16.8;
   else if(S_1>=2.6&&S_1<2.65)
      angle_from_s1=15.8;
   else if(S_1>=2.55&&S_1<2.6)
      angle_from_s1=14.5;
   else if(S_1>=2.5&&S_1<2.55)
      angle_from_s1=13.3;
   else if(S_1>=2.45&&S_1<2.5)
      angle_from_s1=12.5;
   else if(S_1>=2.4&&S_1<2.45)
      angle_from_s1=12.0;
   else if(S_1>=2.35&&S_1<2.4)
      angle_from_s1=11.5;
   else if(S_1>=2.3&&S_1<2.35)
      angle_from_s1=11;
   else if(S_1>=2.25&&S_1<2.3)
      angle_from_s1=10.5;   
   else if(S_1>=2.2&&S_1<2.25)
      angle_from_s1=9.5;
   else if(S_1>=2.15&&S_1<2.2)
      angle_from_s1=8.5;
   else if(S_1>=2.1&&S_1<2.15)
      angle_from_s1=8.0;
   else if(S_1>=2.05&&S_1<2.1)
      angle_from_s1=7.5; 
   else if(S_1>=1&&S_1<2.05)
      angle_from_s1=7.0;   
   else 
     angle_from_s1=0;
 //  angle_from_s1=angleangle;
   OUT_PWM=angle_from_s1*anglr_to_pwm;
   return  -OUT_PWM; 
}






void Set_Servo_pwm(int x_angle ,int y_angle)
{
  int pwm1=2343;                                          //�϶��λ��ʽ
  pwm1+=y_angle;
  pwm2+=x_angle;
   if(pwm1>2343)
     pwm1=2343;
   else if(pwm1<1563)
     pwm1=1563;
   if(pwm2>3100)
     pwm2=3100;
   else if(pwm2<1600)
     pwm2=1600;
  now_pwmx=pwm2;
  now_pwmy=pwm1;
      FTM_PWM_Duty(FTM3, FTM_CH0,pwm1);
      FTM_PWM_Duty(FTM3, FTM_CH1,pwm2);
}

void Set_Pwm(int MotorLeft,int MotorRight)                           
{
    
    int Limit=3800;                             //pwm�޷�//����Ҫ,�ɸġ�
    int dead=0;                                         //<30?
    MotorLeft*=1;
    MotorRight*=1;
    if(MotorLeft>Limit) MotorLeft=Limit;
    if(MotorLeft<-Limit)  MotorLeft=-Limit;	
    if(MotorRight>Limit)   MotorRight=Limit;
    if(MotorRight<-Limit)  MotorRight=-Limit;	
    
    
    if(MotorLeft<=0)
    {
        FTM_PWM_Duty(FTM3, FTM_CH0, 0);      //PTA4
        FTM_PWM_Duty(FTM0, FTM_CH2,-(MotorLeft+dead));             //PTA5
    }
    else
    {
        //        FTM_PWM_Duty(FTM0, FTM_CH0, 0);
        //        FTM_PWM_Duty(FTM0, FTM_CH1, 0);     
        //        DELAY_us(100);                 //������,��ת���Ա������
        FTM_PWM_Duty(FTM3, FTM_CH0,MotorLeft+dead);             //PTA4      
        FTM_PWM_Duty(FTM0, FTM_CH2, 0 );     //PTA5
    }
    
    if(MotorRight<=0)
    {
        FTM_PWM_Duty(FTM0, FTM_CH3, -(MotorRight+dead));           //PTA6
        FTM_PWM_Duty(FTM0, FTM_CH4, 0);  //PTA7
    }
    else
    {
        
        //        FTM_PWM_Duty(FTM0, FTM_CH0, 0);
        //        FTM_PWM_Duty(FTM0, FTM_CH1, 0);     
        //        DELAY_us(100);                 //������,��ת���Ա������
        //        
        FTM_PWM_Duty(FTM0, FTM_CH3,0);
        FTM_PWM_Duty(FTM0, FTM_CH4, MotorRight+dead );
    }
}

