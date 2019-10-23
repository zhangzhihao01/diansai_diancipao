/*LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
��ƽ    ̨�������������ܿƼ�MK66FX1M0VLQ18���İ�
����    д��CHIUSIR
����    ע��
������汾��V1.0
�������¡�2016��08��20��
�������Ϣ�ο����е�ַ��
����    վ��http://www.lqist.cn
���Ա����̡�http://shop36265907.taobao.com
���������䡿chiusir@163.com
QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ*/

#include "include.h"

void LED_Init(void)
{
    //GPIO_Init(PTA,17,GPO,1);
    GPIO_Init(PTC,0,GPO,1);  
    GPIO_Init(PTE,26,GPO,1);
    GPIO_Init(PTD,15,GPO,1);
    GPIO_Init(PTD,14,GPO,1);              
    GPIO_Init(PTD,12,GPO,1);              
    GPIO_Init(PTD,11,GPO,1);              
}

void LED_Ctrl(LEDn_e ledno, LEDs_e sta)
{
    switch(ledno) 
    {
    case LED0:
        if(sta==ON)        GPIO_Ctrl(PTA,17,0);
        else if(sta==OFF) GPIO_Ctrl(PTA,17,1);
        else if(sta==RVS) GPIO_Reverse (PTA, 17);
        break;
        
    case LED1:
        if(sta==ON)        GPIO_Ctrl(PTC,0,0);
        else if(sta==OFF) GPIO_Ctrl(PTC,0,1);
        else if(sta==RVS) GPIO_Reverse (PTC,0);
        break;    
    case LED2:
        if(sta==ON)        GPIO_Ctrl(PTE, 26,0);
        else if(sta==OFF) GPIO_Ctrl(PTE, 26,1);
        else if(sta==RVS) GPIO_Reverse (PTE, 26);
        break;
    case LED3:
        if(sta==ON)        GPIO_Ctrl(PTD, 15,0);
        else if(sta==OFF) GPIO_Ctrl(PTD, 15,1);
        else if(sta==RVS) GPIO_Reverse (PTD, 15);
        break;
    case LED4:                                  //��
        if(sta==ON)        GPIO_Ctrl(PTD,11,0);
        else if(sta==OFF) GPIO_Ctrl(PTD,11,1);
        else if(sta==RVS) GPIO_Reverse (PTD,11);
        break;
    case LED5:                                  //��
        if(sta==ON)        GPIO_Ctrl(PTD,12,0);
        else if(sta==OFF) GPIO_Ctrl(PTD,12,1);
        else if(sta==RVS) GPIO_Reverse (PTD,12);
        break;
    case LED6:                                  //��
        if(sta==ON)        GPIO_Ctrl(PTD,14,0);
        else if(sta==OFF) GPIO_Ctrl(PTD,14,1);
        else if(sta==RVS) GPIO_Reverse(PTD,14);
        break;
    case LEDALL:
        if(sta==ON) 
        {       
            GPIO_Ctrl(PTA,17,0);
            GPIO_Ctrl(PTC,0,0);
            GPIO_Ctrl(PTE,26,0);
            GPIO_Ctrl(PTD,15,0);
        }
        else if(sta==OFF)
        { 
            GPIO_Ctrl(PTA,17,1);
            GPIO_Ctrl(PTC,0,1);
            GPIO_Ctrl(PTE, 26,1);
            GPIO_Ctrl(PTD,15,1);
            
        }
        else if(sta==RVS)
        {       
            GPIO_Reverse (PTA, 17);     
            GPIO_Reverse (PTC, 0);         
            GPIO_Reverse (PTE, 26);
            GPIO_Reverse (PTD, 15);      
            
        }
        break;
    default:
        break;    
    }   
}