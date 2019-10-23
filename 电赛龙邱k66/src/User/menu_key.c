/**********************718����ʵ���ҿ���������*********************
*  ��д��718����ʵ����
*  ƽ̨��STM32F103ZET6
*  ˵�����������ߵ�ˮƽ���ޣ����в���֮�����������½⡣
*		 �����Ҷ࿴�������ֲᡣ     
******************************************************************/
#include "include.h"
#include "menu_key.h"

int16 AD_valu[8][3];
int16 AD[8];
int16 AD_sum[8];
/************************
ADC�˿ڳ�ʼ��
adc_chose��ADC0/ADC1
adc_digit������
************************/



void adc_ready()                                               //ADC�ɼ���ʼ��
{
  ADC_Init(adc_chose);
  ADC_Start(adc_chose,adc_num1,adc_digit); 
  ADC_Start(adc_chose,adc_num2,adc_digit); 
  ADC_Start(adc_chose,adc_num3,adc_digit);
  ADC_Start(adc_chose,adc_num4,adc_digit);
  ADC_Start(adc_chose,adc_num5,adc_digit);
  ADC_Start(adc_chose,adc_num6,adc_digit);
  ADC_Start(adc_chose,adc_num7,adc_digit);
  ADC_Start(adc_chose,adc_num8,adc_digit);
//  ADC_Init(ADC_0);
//  ADC_Start(ADC_0,adc_num3,adc_digit);
//  ADC_Start(ADC_0,adc_num4,adc_digit);

}

/********************
ADC�ɼ���ƽ���˲�
ad_valu[5]��ԭʼ����
AD[5]���˲�֮�������
********************/



void Read_adc()                                               //ADC�ɼ�ԭʼ����
{
   int16  i=0,j=0;
   int16  ad_valu[8];

  
     ad_valu[0]=ADC_Ave(adc_chose,adc_num1,adc_digit,10);     
     ad_valu[1]=ADC_Ave(adc_chose,adc_num2,adc_digit,10);     
     ad_valu[2]=ADC_Ave(adc_chose,adc_num3,adc_digit,10);    
     ad_valu[3]=ADC_Ave(adc_chose,adc_num4,adc_digit,10);    
     ad_valu[4]=ADC_Ave(adc_chose,adc_num5,adc_digit,10); 
     ad_valu[5]=ADC_Ave(adc_chose,adc_num6,adc_digit,10); 
     ad_valu[6]=ADC_Ave(adc_chose,adc_num7,adc_digit,10); 
     ad_valu[7]=ADC_Ave(adc_chose,adc_num8,adc_digit,10); 

   
    for(j=0;j<8;j++)
    for(i=0;i<huanum-1;i++)                                                    //�����˲�
    {
      AD_valu[j][i]=AD_valu[j][i+1];

    }
    for(i=0;i<8;i++) 
    {
      AD_valu[i][huanum-1]=ad_valu[i];
    }
    for(j=0;j<8;j++)
    for(i=0;i<huanum;i++)
    {
      AD_sum[j]+=AD_valu[j][i];
    }
    for(i=0;i<8;i++)
    {
      AD[i]=AD_sum[i]/huanum;                                 //��׼ȷ��ADֵ
      AD_sum[i]=0;
    }
   
}
/*****************���ᰴ����ʼ��***********/
  
void Five_axis_keyInit()
{
   //���ų�ʼ��
   KEY_Init();
//   GPIO_Init(PTB,20,GPI_UP,1);          //LEFT
//   GPIO_Init(PTB,21,GPI_UP,1);          //UP
//   GPIO_Init(PTB,22,GPI_UP,1);          //RIGHT   
   //GPIO_Init(PTB,8,GPI_UP,1);           //DOWN  
   //GPIO_Init(PTA,28,GPI_UP,1);          //MIDDLE
   //�ⲿ�жϳ�ʼ��
   EXTI_Init(PTB, 20, falling_up);
   EXTI_Init(PTB, 21, falling_up);
   EXTI_Init(PTB, 22, falling_up);
   //EXTI_Init(PTB, 8, falling_up);
   //EXTI_Init(PTA, 28,  falling_up );
}


/**
  * @brief  �ж��ĸ�����������
  * @param  None
  * @retval None
  */
//int Key_detect(void)
//{
//	uint16_t adcx;
//	float temp;
//	
//	adcx=AD[0];//��ȡadcֵ
//	temp=(float)adcx*(3.3/4096);					//�����ѹ
//	//�ж��ĸ�����
//	if (temp<=KEY_UP_ADC+0.1&&temp>=KEY_UP_ADC-0.1){
//		time_delay_ms(50);//��ʱ����
//		adcx=AD[0];//��ȡadcֵ
//		temp=(float)adcx*(3.3/4096);					//�����ѹ
//		if (temp<=KEY_UP_ADC+0.1&&temp>=KEY_UP_ADC-0.1){
//			return KEY_UP;
//		}
//	}
//	else if(temp<=KEY_DOWN_ADC+0.1&&temp>=KEY_DOWN_ADC-0.1){
//		time_delay_ms(50);
//		adcx=AD[0];//��ȡadcֵ
//		temp=(float)adcx*(3.3/4096);					//�����ѹ
//		if (temp<=KEY_DOWN_ADC+0.1&&temp>=KEY_DOWN_ADC-0.1){
//			return KEY_DOWN;
//		}
//	}
//	else if(temp<=KEY_RIGHT_ADC+0.1&&temp>=KEY_RIGHT_ADC-0.1){
//		time_delay_ms(50);
//		adcx=AD[0];//��ȡadcֵ
//		temp=(float)adcx*(3.3/4096);					//�����ѹ
//		if (temp<=KEY_RIGHT_ADC+0.1&&temp>=KEY_RIGHT_ADC-0.1){
//			return KEY_RIGHT;
//		}
//	}
//	else if(temp<=KEY_LEFT_ADC+0.1&&temp>=KEY_LEFT_ADC-0.1){
//		time_delay_ms(50);
//		adcx=AD[0];//��ȡadcֵ
//		temp=(float)adcx*(3.3/4096);					//�����ѹ
//		if (temp<=KEY_LEFT_ADC+0.1&&temp>=KEY_LEFT_ADC-0.1){
//			return KEY_LEFT;
//		}
//	}
//	else if(temp<=KEY_ENDER_ADC+0.1&&temp>=KEY_ENDER_ADC-0.1){
//		time_delay_ms(50);
//		adcx=AD[0];//��ȡadcֵ
//		temp=(float)adcx*(3.3/4096);					//�����ѹ
//		if (temp<=KEY_ENDER_ADC+0.1&&temp>=KEY_ENDER_ADC-0.1){
//			return KEY_ENDER;
//		}
//	}
//	else{
//		return 0;
//	}
//	return 0;
//}




/*************************
�������
ǰʮ������Ӧ0~9
��11����Ϊ�˳�����
*************************/
int  Key_detect(void)
{
  Read_adc();
  static int Value_get_jianpan;
  static float  value_jianpan;
  //AD_valu[0]=ADC_Ave(adc_chose,adc_num1,adc_digit,10);
  value_jianpan=AD[0]/4096.0*120.0;
  if(value_jianpan<15&&value_jianpan>5)                             
    Value_get_jianpan=0;
  else if(value_jianpan<25&&value_jianpan>15)
    Value_get_jianpan=1;
  else if(value_jianpan<35&&value_jianpan>25)
    Value_get_jianpan=2;
  else if(value_jianpan<45&&value_jianpan>35)
    Value_get_jianpan=3;
  else if(value_jianpan<55&&value_jianpan>45)
    Value_get_jianpan=4;
  else if(value_jianpan<65&&value_jianpan>55)
    Value_get_jianpan=5;
  else if(value_jianpan<75&&value_jianpan>65)
    Value_get_jianpan=6;
  else if(value_jianpan<85&&value_jianpan>75)
    Value_get_jianpan=7;
  else if(value_jianpan<95&&value_jianpan>85)
    Value_get_jianpan=8;
  else if(value_jianpan<105&&value_jianpan>95)
    Value_get_jianpan=9;
  else if(value_jianpan<115&&value_jianpan>105)
    Value_get_jianpan=10;//ȷ�ϼ�
  else if(value_jianpan>105)
    Value_get_jianpan=-1;//ȡ����
  else if(value_jianpan<5)
    Value_get_jianpan=12;//�ް�������
  
  return Value_get_jianpan;
}
