/****************************************************************************************************
��ƽ    ̨������K66FX���ܳ�VDĸ��
����    д��CHIUSIR
��E-mail  ��chiusir@163.com
������汾��V1.0
�������¡�2018��4��28��
�������Ϣ�ο����е�ַ��
����    վ��http://www.lqist.cn
���Ա����̡�http://shop36265907.taobao.com
------------------------------------------------
��dev.env.��IAR8.2������
��Target  ��K66FX1M0VLQ18
��Crystal �� 50.000Mhz
��busclock��110.000MHz
��pllclock��220.000MHz
******************************************************************************************************/
#include "include.h" 

int Encoder_Left,Encoder_Right;                         //���ұ��������������
int tim=20;                                               //pit��ʱ�ж�����
unsigned short send_data[9];                               //��λ������

extern float   distance_taget;
extern int     problems;
extern float shuru_angle_X;
extern int16  AD_valu[5];
extern int fangpao_flag;
extern int   k_error;

int   iii=1650;                  //40��1650
int   jjj;



void main(void)
{    
    
    PLL_Init(PLL200);             //��ʼ��PLLΪ?M,�� ��Ϊ��M
    DisableInterrupts;            //�ر��ж�
    LED_Init();

    TFTSPI_Init(); 
    TFTSPI_CLS(u16BLACK);          //����Ļ
    time_delay_ms(1000);
    PIT_Init(PIT0, tim);          //�ж϶�ʱ tim ms
    Motor_Init();                //�����ʼ�� 
    bomakaiguan();                //���뿪�س�ʼ��
    UART_Init(UART_4,115200);      //openmv���ڳ�ʼ��
    UART_Init(UART_2,115200);        //GY61���ڳ�ʼ��
    UART_Init(UART_3,115200);       //���⴮��
    UART_Irq_En(UART_4);           //openmv�����ж�
    UART_Irq_En(UART_3);           //�����������жϳ�ʼ��
    UART_Irq_En(UART_2);            //���⴮���жϳ�ʼ��
    ADC_ready();                   //ADC��ʼ��
 
    EnableInterrupts;             //�����ж�
    
    huamian_1();
    

    
    while(1)  
    { 

     //  FTM_PWM_Duty(FTM3, FTM_CH0,iii);
       static int once_time[5]={1,1,1,1,1};           
        TFT_tuxiangxianshi();
        if(problems==31&&once_time[0]==1)                            //�����������
        {
          once_time[1]=1;
          once_time[4]=1;
          distance_taget=scanf_distance_taget();
          once_time[0]=0;
        }
        else if(problems==32&&once_time[1]==1)                      //��������Ƕ�
        {
          once_time[0]=1;
          once_time[4]=1;
          shuru_angle_X=scanf_distance_taget()*10;
          once_time[1]=0;
        }
        else if(problems==32&&once_time[4]==1)                      //�����������
        {
          once_time[0]=1;
          once_time[1]=1;
          k_error=scanf_distance_taget()*10;
          once_time[4]=0;
        }
        else if(problems==33&&once_time[2]==1)                     //����AD���ֵ����
        {
          once_time[3]=1;   
          while(!jianpanjiaozheng_max())
          {
            ;
          }
          once_time[2]=0;
        }
        else if(problems==34&&once_time[3]==1)                    //����AD��Сֵ����
        { 
          once_time[2]=1;   
          while(!jianpanjiaozheng_min())
          {
            ;
          }
          once_time[3]=0;
        }
        else if(problems==311)                                      //��ť�������
        {
          AD_valu[3]=ADC_Ave(ADC_1,adc_num4,adc_digit,10);
          distance_taget=2+AD_valu[3]/4096.0;              
        }
        else if(problems==321)                                      //��ť����Ƕ�
        {
          AD_valu[4]=ADC_Ave(ADC_1,adc_num5,adc_digit,10);
          shuru_angle_X=-30+AD_valu[4]/4096.0*60;              
        }   
        else if(problems==341)                                     //��ť����������
        {
          AD_valu[4]=ADC_Ave(ADC_1,adc_num5,adc_digit,10);
          k_error=-10+AD_valu[4]/4096.0*20;              
        } 
        
        if(fangpao_flag==0)
        {
          GPIO_Ctrl(PTC,18,0);                    //����io��0
          time_delay_ms(50);     
          GPIO_Ctrl(PTC,19,1);                    //���io��1 
        }
        else if(fangpao_flag==1)
        {
          GPIO_Ctrl(PTC,19,0);                   //���io��0
          time_delay_ms(50);               
          GPIO_Ctrl(PTC,18,1);                   //����io��1
        }
    }

}






























