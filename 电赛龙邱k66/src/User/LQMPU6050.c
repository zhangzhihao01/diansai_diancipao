/*******************************************************************************
��ƽ    ̨������KV58F24���ܳ�VDĸ��
����    д��CHIUSIR
��E-mail  ��chiusir@163.com
������汾��V1.0
�������¡�2017��12��15��
�������Ϣ�ο����е�ַ��
����    վ��http://www.lqist.cn
���Ա����̡�http://shop36265907.taobao.com
------------------------------------------------
��dev.env.��IAR7.80.4�����ϰ汾
��Target  ��MKV58F1M0VLQ24
��Crystal �� 50.000Mhz
��busclock��137.500MHz
��pllclock��275.000MHz

���������ڵ����У��û������ڴλ������޸�--2018/11/20
������ܵ��Բ��������ң���������õ���������㣬��Ҷ����л��ģ�����

QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ*/
#include "include.h"
#include "math.h"
#include "u_iic.h"

#define huzhuanjiao (180/(3.1416))


float Accel_x;                    //X����ٶ�ֵ�ݴ�
float Accel_y;                     //Y����ٶ�ֵ�ݴ�
float Accel_z;                    //Z����ٶ�ֵ�ݴ�

float Gyro_x;                     //X�������������ݴ�
float K_Gyro_y;                     //Y�������������ݴ�
float Gyro_z;                   //Z�������������ݴ�

float Angle_x_temp;               //�ɼ��ٶȼ����x��б�Ƕ�
float Angle_y_temp;             //�ɼ��ٶȼ����y��б�Ƕ�


extern float Angle;
float Angle_dot=0.0;            //������б�Ƕ�




float pitch=0.0;
float pitch_g=-36.0f;                                       //�����ǻ��ֽǶ�,��ʼֵ

float Q_angle = 0.001;                                      //Ԥ�⣨���̣���������,�Ƕ����Ŷ�0.001f
float Q_gyro  = 0.005f;                                     //Ԥ�⣨���̣���������,���ٶ����Ŷ�0.005f
float R_angle = 0.3f;                                       //�������۲⣩��������0.5

extern float tim;

char  C_0     = 1;
float Q_bias, Angle_err;                //Q_angle��ʾ���ٶȼƹ�������Э����;Q_bias��ʾ�����ǹ�������Э����
float PCt_0, PCt_1, E;
float K_0, K_1, t_0, t_1;
float Pdot[4] ={0,0,0,0};
float PP[2][2] = { { 1, 0 },{ 0, 1 } };
short Acc[3];
short Gyr[3];
 
float  dt;



float Kalman_Filter(float Accel,float Gyro)      //������ٶȼ���ĽǶȺ�
             
{
        
  dt=tim/1000.0;
  Angle+=(Gyro - Q_bias) * dt; //�������
  
  
  Pdot[0]=Q_angle - PP[0][1] - PP[1][0]; // Pk-����������Э�����΢��
  
  Pdot[1]=- PP[1][1];
  Pdot[2]=- PP[1][1];
  Pdot[3]=Q_gyro;
  
  PP[0][0] += Pdot[0] * dt;   // Pk-����������Э����΢�ֵĻ���
  PP[0][1] += Pdot[1] * dt;   // =����������Э����
  PP[1][0] += Pdot[2] * dt;
  PP[1][1] += Pdot[3] * dt;
  
  Angle_err = Accel - Angle;        //zk-�������
  
  PCt_0 = C_0 * PP[0][0];
  PCt_1 = C_0 * PP[1][0];
  
  E = R_angle + C_0 * PCt_0;
  
  K_0 = PCt_0 / E;
  K_1 = PCt_1 / E;
  
  t_0 = PCt_0;
  t_1 = C_0 * PP[0][1];
  
  PP[0][0] -= K_0 * t_0;                 //����������Э����
  PP[0][1] -= K_0 * t_1;
  PP[1][0] -= K_1 * t_0;
  PP[1][1] -= K_1 * t_1;
  
  Angle += K_0 * Angle_err;         //�������//���ŽǶ�
  Q_bias += K_1 * Angle_err;         //�������//������Ư
  K_Gyro_y = Gyro - Q_bias;         //���ֵ(�������)��΢��=���Ž��ٶ�
  
  return Angle;
  
}


/***************
��ʼ��MPU6050
����ֵ:0,�ɹ�
 ����,�������
****************/
uint8_t MPU6050_Init(void)
{
    uint8_t res;
    IIC_Init();                                                  //MPU6050 ֧��400K I2C
    res=MPU_Read_Byte(MPU6050_ADDR,WHO_AM_I);                     //��ȡMPU6050��ID

    res = 0;
    res += MPU_Write_Byte(MPU6050_ADDR,MPU_PWR_MGMT1_REG,0X80);         //��λMPU6050
    time_delay_ms(100);                                                 //��ʱ100ms
    res += MPU_Write_Byte(MPU6050_ADDR,MPU_PWR_MGMT1_REG,0X00);         //����MPU6050
    res += MPU_Set_Gyro_Fsr(3);					        	        //�����Ǵ�����,��2000dps   
    res += MPU_Set_Accel_Fsr(1);					       	 	//���ٶȴ�����,��4g
    res += MPU_Set_Rate(1000);						       	 	//���ò�����1000Hz
    res += MPU_Write_Byte(MPU6050_ADDR,MPU_CFG_REG,0x02);               //�������ֵ�ͨ�˲���   98hz
    res += MPU_Write_Byte(MPU6050_ADDR,MPU_INT_EN_REG,0X00);            //�ر������ж�
    res += MPU_Write_Byte(MPU6050_ADDR,MPU_USER_CTRL_REG,0X00);         //I2C��ģʽ�ر�

    res += MPU_Write_Byte(MPU6050_ADDR,MPU_PWR_MGMT1_REG,0X01);  	//����CLKSEL,PLL X��Ϊ�ο�
    res += MPU_Write_Byte(MPU6050_ADDR,MPU_PWR_MGMT2_REG,0X00);  	//���ٶ��������Ƕ�����
    
    
    return 0;
}




/**************************************
����MPU6050�����Ǵ����������̷�Χ
fsr:0,��250dps;1,��500dps;2,��1000dps;3,��2000dps
����ֵ:0,���óɹ�
����,����ʧ�� 
**************************************/
uint8_t MPU_Set_Gyro_Fsr(uint8_t fsr)
{
    return MPU_Write_Byte(MPU6050_ADDR,MPU_GYRO_CFG_REG,fsr<<3);//���������������̷�Χ  
}






/*****************************************
����MPU6050���ٶȴ����������̷�Χ
fsr:0,��2g;1,��4g;2,��8g;3,��16g
����ֵ:0,���óɹ�
����,����ʧ�� 
****************************************/
uint8_t MPU_Set_Accel_Fsr(uint8_t fsr)
{
    return MPU_Write_Byte(MPU6050_ADDR,MPU_ACCEL_CFG_REG,fsr<<3);//���ü��ٶȴ����������̷�Χ  
}








/***********************************
����MPU6050�����ֵ�ͨ�˲���
lpf:���ֵ�ͨ�˲�Ƶ��(Hz)
����ֵ:0,���óɹ�
����,����ʧ�� 
************************************/
uint8_t MPU_Set_LPF(uint16_t lpf)
{
    uint8_t data=0;
    if(lpf>=188)data=1;
    else if(lpf>=98)data=2;
    else if(lpf>=42)data=3;
    else if(lpf>=20)data=4;
    else if(lpf>=10)data=5;
    else data=6; 
    return MPU_Write_Byte(MPU6050_ADDR,MPU_CFG_REG,data);//�������ֵ�ͨ�˲���  
}






/*******************************************
����MPU6050�Ĳ�����(�ٶ�Fs=1KHz)
rate:4~1000(Hz)
����ֵ:0,���óɹ�
����,����ʧ�� 
**********************************************/
uint8_t MPU_Set_Rate(uint16_t rate)
{
    uint8_t data;
    if(rate>1000)rate=1000;
    if(rate<4)rate=4;
    data=1000/rate-1;
    data=MPU_Write_Byte(MPU6050_ADDR,MPU_SAMPLE_RATE_REG,data);	//�������ֵ�ͨ�˲���
    return MPU_Set_LPF(rate/2);	//�Զ�����LPFΪ�����ʵ�һ��
}





/********************************************
�õ��¶�ֵ
����ֵ:�¶�ֵ(������100��)
********************************************/
short MPU_Get_Temperature(void)
{
    uint8_t buf[2]; 
    short raw;
    float temp;
    MPU_Read_Len(MPU6050_ADDR,MPU_TEMP_OUTH_REG,2,buf); 
    raw=((uint16_t)buf[0]<<8)|buf[1];  
    temp=21+((double)raw)/333.87;  
    return (short)temp*100;
}






/********************************************
�õ�������ֵ(ԭʼֵ)
gx,gy,gz:������x,y,z���ԭʼ����(������)
����ֵ:0,�ɹ�
����,�������
**********************************************/
uint8_t MPU_Get_Gyroscope(short *gx,short *gy,short *gz)
{
    uint8_t buf[6],res; 
    res=MPU_Read_Len(MPU6050_ADDR,MPU_GYRO_XOUTH_REG,6,buf);
    if(res==0)
    {
        *gx=((uint16_t)buf[0]<<8)|buf[1];  
        *gy=((uint16_t)buf[2]<<8)|buf[3];  
        *gz=((uint16_t)buf[4]<<8)|buf[5];
    } 	
    return res;
}




/*****************************************
�õ����ٶ�ֵ(ԭʼֵ)
gx,gy,gz:������x,y,z���ԭʼ����(������)
����ֵ:0,�ɹ�
����,�������
******************************************/
uint8_t MPU_Get_Accelerometer(short *ax,short *ay,short *az)
{
    uint8_t buf[6],res;  
    res=MPU_Read_Len(MPU6050_ADDR,MPU_ACCEL_XOUTH_REG,6,buf);
    if(res==0)
    {
        *ax=((uint16_t)buf[0]<<8)|buf[1];  
        *ay=((uint16_t)buf[2]<<8)|buf[3];  
        *az=((uint16_t)buf[4]<<8)|buf[5];
    } 	
    return res;
}





/*****************************************
�õ��Ӽ�ֵ���¶�ֵ�����ٶ�ֵ(ԭʼֵ)
gx,gy,gz:������x,y,z���ԭʼ����(������)
����ֵ:0,�ɹ�
����,�������
*****************************************/
uint8_t MPU_Get_Raw_data(short *ax,short *ay,short *az,short *gx,short *gy,short *gz)
{
    uint8_t buf[14],res; 
    
    res=MPU_Read_Len(MPU6050_ADDR,MPU_ACCEL_XOUTH_REG,14,buf);
      
    if(res==0)
    {
        *ax=((uint16_t)buf[0]<<8)|buf[1];  
        *ay=((uint16_t)buf[2]<<8)|buf[3];  
        *az=((uint16_t)buf[4]<<8)|buf[5];
        *gx=((uint16_t)buf[8]<<8)|buf[9];  
        *gy=((uint16_t)buf[10]<<8)|buf[11];  
        *gz=((uint16_t)buf[12]<<8)|buf[13];
    } 	
    return res;
}





/*******************************
IIC����д
addr:������ַ 
reg:�Ĵ�����ַ
len:д�볤��
buf:������
����ֵ:0,����
����,�������
******************************/
uint8_t MPU_Write_Len(uint8_t addr,uint8_t reg,uint8_t len,uint8_t *buf)
{
    return IIC_WriteMultByteToSlave(addr, reg, len, buf);
} 




/**************************
IIC������
addr:������ַ
reg:Ҫ��ȡ�ļĴ�����ַ
len:Ҫ��ȡ�ĳ���
buf:��ȡ�������ݴ洢��
����ֵ:0,����
����,�������
*************************/
uint8_t MPU_Read_Len(uint8_t addr,uint8_t reg,uint8_t len,uint8_t *buf)
{ 
    return IIC_ReadMultByteFromSlave(addr, reg, len, buf);     
}




/**************************
IICдһ���ֽ� 
devaddr:����IIC��ַ
reg:�Ĵ�����ַ
data:����
����ֵ:0,����
����,�������
**************************/
uint8_t MPU_Write_Byte(uint8_t addr,uint8_t reg,uint8_t value)
{
    return IIC_WriteByteToSlave(addr, reg, value);
}



/****************************
IIC��һ���ֽ� 
reg:�Ĵ�����ַ 
����ֵ:����������
****************************/
uint8_t MPU_Read_Byte(uint8_t addr,uint8_t reg)
{
    uint8_t value;
    IIC_ReadByteFromSlave(addr, reg, &value);
    return value;
}





