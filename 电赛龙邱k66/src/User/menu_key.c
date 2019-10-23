/**********************718创新实验室开发板例程*********************
*  编写：718创新实验室
*  平台：STM32F103ZET6
*  说明：由于作者的水平有限，若有不足之处，还请大家谅解。
*		 建议大家多看看数据手册。     
******************************************************************/
#include "include.h"
#include "menu_key.h"

int16 AD_valu[8][3];
int16 AD[8];
int16 AD_sum[8];
/************************
ADC端口初始化
adc_chose：ADC0/ADC1
adc_digit：精度
************************/



void adc_ready()                                               //ADC采集初始化
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
ADC采集与平滑滤波
ad_valu[5]：原始数据
AD[5]：滤波之后的数据
********************/



void Read_adc()                                               //ADC采集原始数据
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
    for(i=0;i<huanum-1;i++)                                                    //滑动滤波
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
      AD[i]=AD_sum[i]/huanum;                                 //较准确的AD值
      AD_sum[i]=0;
    }
   
}
/*****************五轴按键初始化***********/
  
void Five_axis_keyInit()
{
   //引脚初始化
   KEY_Init();
//   GPIO_Init(PTB,20,GPI_UP,1);          //LEFT
//   GPIO_Init(PTB,21,GPI_UP,1);          //UP
//   GPIO_Init(PTB,22,GPI_UP,1);          //RIGHT   
   //GPIO_Init(PTB,8,GPI_UP,1);           //DOWN  
   //GPIO_Init(PTA,28,GPI_UP,1);          //MIDDLE
   //外部中断初始化
   EXTI_Init(PTB, 20, falling_up);
   EXTI_Init(PTB, 21, falling_up);
   EXTI_Init(PTB, 22, falling_up);
   //EXTI_Init(PTB, 8, falling_up);
   //EXTI_Init(PTA, 28,  falling_up );
}


/**
  * @brief  判断哪个按键被按下
  * @param  None
  * @retval None
  */
//int Key_detect(void)
//{
//	uint16_t adcx;
//	float temp;
//	
//	adcx=AD[0];//获取adc值
//	temp=(float)adcx*(3.3/4096);					//计算电压
//	//判断哪个按键
//	if (temp<=KEY_UP_ADC+0.1&&temp>=KEY_UP_ADC-0.1){
//		time_delay_ms(50);//延时消抖
//		adcx=AD[0];//获取adc值
//		temp=(float)adcx*(3.3/4096);					//计算电压
//		if (temp<=KEY_UP_ADC+0.1&&temp>=KEY_UP_ADC-0.1){
//			return KEY_UP;
//		}
//	}
//	else if(temp<=KEY_DOWN_ADC+0.1&&temp>=KEY_DOWN_ADC-0.1){
//		time_delay_ms(50);
//		adcx=AD[0];//获取adc值
//		temp=(float)adcx*(3.3/4096);					//计算电压
//		if (temp<=KEY_DOWN_ADC+0.1&&temp>=KEY_DOWN_ADC-0.1){
//			return KEY_DOWN;
//		}
//	}
//	else if(temp<=KEY_RIGHT_ADC+0.1&&temp>=KEY_RIGHT_ADC-0.1){
//		time_delay_ms(50);
//		adcx=AD[0];//获取adc值
//		temp=(float)adcx*(3.3/4096);					//计算电压
//		if (temp<=KEY_RIGHT_ADC+0.1&&temp>=KEY_RIGHT_ADC-0.1){
//			return KEY_RIGHT;
//		}
//	}
//	else if(temp<=KEY_LEFT_ADC+0.1&&temp>=KEY_LEFT_ADC-0.1){
//		time_delay_ms(50);
//		adcx=AD[0];//获取adc值
//		temp=(float)adcx*(3.3/4096);					//计算电压
//		if (temp<=KEY_LEFT_ADC+0.1&&temp>=KEY_LEFT_ADC-0.1){
//			return KEY_LEFT;
//		}
//	}
//	else if(temp<=KEY_ENDER_ADC+0.1&&temp>=KEY_ENDER_ADC-0.1){
//		time_delay_ms(50);
//		adcx=AD[0];//获取adc值
//		temp=(float)adcx*(3.3/4096);					//计算电压
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
矩阵键盘
前十个键对应0~9
第11个键为退出输入
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
    Value_get_jianpan=10;//确认键
  else if(value_jianpan>105)
    Value_get_jianpan=-1;//取消键
  else if(value_jianpan<5)
    Value_get_jianpan=12;//无按键输入
  
  return Value_get_jianpan;
}
