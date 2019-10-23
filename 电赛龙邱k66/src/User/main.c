/****************************************************************************************************
【平    台】龙邱K66FX智能车VD母板
【编    写】CHIUSIR
【E-mail  】chiusir@163.com
【软件版本】V1.0
【最后更新】2018年4月28日
【相关信息参考下列地址】
【网    站】http://www.lqist.cn
【淘宝店铺】http://shop36265907.taobao.com
------------------------------------------------
【dev.env.】IAR8.2及以上
【Target  】K66FX1M0VLQ18
【Crystal 】 50.000Mhz
【busclock】110.000MHz
【pllclock】220.000MHz
******************************************************************************************************/
#include "include.h" 

int Encoder_Left,Encoder_Right;                         //左右编码器的脉冲计数
int tim=20;                                               //pit定时中断周期
unsigned short send_data[9];                               //上位机数据

extern float   distance_taget;
extern int     problems;
extern float shuru_angle_X;
extern int16  AD_valu[5];
extern int fangpao_flag;
extern int   k_error;

int   iii=1650;                  //40度1650
int   jjj;



void main(void)
{    
    
    PLL_Init(PLL200);             //初始化PLL为?M,总 线为？M
    DisableInterrupts;            //关闭中断
    LED_Init();

    TFTSPI_Init(); 
    TFTSPI_CLS(u16BLACK);          //清屏幕
    time_delay_ms(1000);
    PIT_Init(PIT0, tim);          //中断定时 tim ms
    Motor_Init();                //电机初始化 
    bomakaiguan();                //拨码开关初始化
    UART_Init(UART_4,115200);      //openmv串口初始化
    UART_Init(UART_2,115200);        //GY61串口初始化
    UART_Init(UART_3,115200);       //激光串口
    UART_Irq_En(UART_4);           //openmv串口中断
    UART_Irq_En(UART_3);           //超声波串口中断初始化
    UART_Irq_En(UART_2);            //激光串口中断初始化
    ADC_ready();                   //ADC初始化
 
    EnableInterrupts;             //开启中断
    
    huamian_1();
    

    
    while(1)  
    { 

     //  FTM_PWM_Duty(FTM3, FTM_CH0,iii);
       static int once_time[5]={1,1,1,1,1};           
        TFT_tuxiangxianshi();
        if(problems==31&&once_time[0]==1)                            //键盘输入距离
        {
          once_time[1]=1;
          once_time[4]=1;
          distance_taget=scanf_distance_taget();
          once_time[0]=0;
        }
        else if(problems==32&&once_time[1]==1)                      //键盘输入角度
        {
          once_time[0]=1;
          once_time[4]=1;
          shuru_angle_X=scanf_distance_taget()*10;
          once_time[1]=0;
        }
        else if(problems==32&&once_time[4]==1)                      //键盘输入误差
        {
          once_time[0]=1;
          once_time[1]=1;
          k_error=scanf_distance_taget()*10;
          once_time[4]=0;
        }
        else if(problems==33&&once_time[2]==1)                     //键盘AD最大值矫正
        {
          once_time[3]=1;   
          while(!jianpanjiaozheng_max())
          {
            ;
          }
          once_time[2]=0;
        }
        else if(problems==34&&once_time[3]==1)                    //键盘AD最小值矫正
        { 
          once_time[2]=1;   
          while(!jianpanjiaozheng_min())
          {
            ;
          }
          once_time[3]=0;
        }
        else if(problems==311)                                      //旋钮输入距离
        {
          AD_valu[3]=ADC_Ave(ADC_1,adc_num4,adc_digit,10);
          distance_taget=2+AD_valu[3]/4096.0;              
        }
        else if(problems==321)                                      //旋钮输入角度
        {
          AD_valu[4]=ADC_Ave(ADC_1,adc_num5,adc_digit,10);
          shuru_angle_X=-30+AD_valu[4]/4096.0*60;              
        }   
        else if(problems==341)                                     //旋钮输入距离误差
        {
          AD_valu[4]=ADC_Ave(ADC_1,adc_num5,adc_digit,10);
          k_error=-10+AD_valu[4]/4096.0*20;              
        } 
        
        if(fangpao_flag==0)
        {
          GPIO_Ctrl(PTC,18,0);                    //放炮io置0
          time_delay_ms(50);     
          GPIO_Ctrl(PTC,19,1);                    //充电io置1 
        }
        else if(fangpao_flag==1)
        {
          GPIO_Ctrl(PTC,19,0);                   //充电io置0
          time_delay_ms(50);               
          GPIO_Ctrl(PTC,18,1);                   //放炮io置1
        }
    }

}






























