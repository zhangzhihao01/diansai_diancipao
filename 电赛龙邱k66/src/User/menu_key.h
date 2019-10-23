#ifndef _MENU_KEY_H_
#define _MENU_KEY_H_

#include "include.h"

/////////////////////∞¥º¸∫Í∂®“Â////////////////////////
#define KEY_UP_ADC 0.1
#define KEY_UP 1

#define KEY_DOWN_ADC 0.8
#define KEY_DOWN 2

#define KEY_LEFT_ADC 1.6
#define KEY_LEFT 3

#define KEY_RIGHT_ADC 2.2
#define KEY_RIGHT 4

#define KEY_ENDER_ADC 2.7
#define KEY_ENDER 5
/***************************/
#define  adc_num1   ADC1_SE12                //5Ë∑Øadc
#define  adc_num2   ADC1_SE10               
#define  adc_num3   ADC1_SE11               
#define  adc_num4   ADC1_SE8               
#define  adc_num5   ADC1_SE9
#define  adc_num6   ADC0_SE16
#define  adc_num7   ADC1_SE5a
#define  adc_num8   ADC1_SE7a 
#define  adc_chose      ADC_1              //ADC_1/2
#define  adc_digit  ADC_12bit              //ADCÁ≤æÂ∫¶
#define   huanum       3                   //ÊªëÂä®Âπ≥ÂùáÊª§Ê≥¢ÁöÑÊ¨°Êï∞
/*******************/
#define  key_up         !PTC14_IN
#define  key_down       !PTB8_IN
#define  key_left       !PTC12_IN
#define  key_right      !PTC15_IN
#define  key_ender      !PTA28_IN
void Five_axis_keyInit();
void Adc_Init(void);			//adc≥ı ºªØ
uint16_t Get_Adc(uint8_t Ch);					//ªÒ»°“ª¥Œadc÷µ
uint16_t Get_Adc_Average(uint8_t Ch,uint8_t Times);		//ªÒ»°»Œ“‚¥Œadcµƒ∆Ωæ˘÷µ
int Key_detect(void);						//ºÏ≤‚∞¥º¸

#endif
