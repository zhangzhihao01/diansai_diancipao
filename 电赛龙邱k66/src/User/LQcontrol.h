/*LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
��ƽ    ̨�������������ܿƼ�MK66FX1M0VLQ18���İ�
����    д��CHIUSIR
����    ע��
������汾��V1.0
�������¡�2017��01��20��
�������Ϣ�ο����е�ַ��
����    վ��http://www.lqist.cn
���Ա����̡�http://shop36265907.taobao.com
���������䡿chiusir@163.com
QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ*/
#ifndef __LQ_CTRL_H_
#define __LQ_CTRL_H_
    
    #define DIFFERENCE 30    

    void Get_Dip_Angle();
    void Motor_Pwm_Out(s16  Speed_L ,s16  Speed_R ,s16 MaxPwm );    
    int  duoji_PID_X();
    int  duoji_PID_Y(float S_1);
    void Set_Pwm(int MotorLeft,int MotorRight);
    void Set_Servo_pwm(int x_angle ,int y_angle);
    void ADC_ready();
    void Motor_Init(void);
    void  bomakaiguan();
    float  scanf_distance_taget();
    int  Key_detect();
    void  bomakaiguan_function();
    int  jianpanjiaozheng_max();
    int  jianpanjiaozheng_min();

#endif
    
#define  adc_num1   ADC0_SE10                //5·adc
#define  adc_num2   ADC1_SE10               
#define  adc_num3   ADC1_SE11               
#define  adc_num4   ADC1_SE8               
#define  adc_num5   ADC1_SE9
#define  adc_num6   ADC0_SE16
#define  adc_num7   ADC1_SE5a
#define  adc_num8   ADC1_SE7a 
#define  adc_chose      ADC_1              //ADC_1/2
#define  adc_digit  ADC_12bit              //ADC����


#define g     9.8
#define pi_1  3.1416  