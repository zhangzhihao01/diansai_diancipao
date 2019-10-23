/*******************************************************************************
【平    台】龙邱KV58F24智能车VD母板
【编    写】CHIUSIR
【E-mail  】chiusir@163.com
【软件版本】V1.0
【最后更新】2017年12月15日
【相关信息参考下列地址】
【网    站】http://www.lqist.cn
【淘宝店铺】http://shop36265907.taobao.com
------------------------------------------------
【dev.env.】IAR7.80.4及以上版本
【Target  】MKV58F1M0VLQ24
【Crystal 】 50.000Mhz
【busclock】137.500MHz
【pllclock】275.000MHz

本程序尚在调试中，用户可以在次基础上修改--2018/11/20
如果你能调试并共享给大家，相信赠人玫瑰手留余香，大家都会感谢你的！！！

QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ*/
#include "include.h"
#include "math.h"
#include "u_iic.h"

#define huzhuanjiao (180/(3.1416))


float Accel_x;                    //X轴加速度值暂存
float Accel_y;                     //Y轴加速度值暂存
float Accel_z;                    //Z轴加速度值暂存

float Gyro_x;                     //X轴陀螺仪数据暂存
float K_Gyro_y;                     //Y轴陀螺仪数据暂存
float Gyro_z;                   //Z轴陀螺仪数据暂存

float Angle_x_temp;               //由加速度计算的x倾斜角度
float Angle_y_temp;             //由加速度计算的y倾斜角度


extern float Angle;
float Angle_dot=0.0;            //最终倾斜角度




float pitch=0.0;
float pitch_g=-36.0f;                                       //陀螺仪积分角度,初始值

float Q_angle = 0.001;                                      //预测（过程）噪声方差,角度置信度0.001f
float Q_gyro  = 0.005f;                                     //预测（过程）噪声方差,角速度置信度0.005f
float R_angle = 0.3f;                                       //测量（观测）噪声方差0.5

extern float tim;

char  C_0     = 1;
float Q_bias, Angle_err;                //Q_angle表示加速度计过程噪声协方差;Q_bias表示陀螺仪过程噪声协方差
float PCt_0, PCt_1, E;
float K_0, K_1, t_0, t_1;
float Pdot[4] ={0,0,0,0};
float PP[2][2] = { { 1, 0 },{ 0, 1 } };
short Acc[3];
short Gyr[3];
 
float  dt;



float Kalman_Filter(float Accel,float Gyro)      //输入加速度计算的角度和
             
{
        
  dt=tim/1000.0;
  Angle+=(Gyro - Q_bias) * dt; //先验估计
  
  
  Pdot[0]=Q_angle - PP[0][1] - PP[1][0]; // Pk-先验估计误差协方差的微分
  
  Pdot[1]=- PP[1][1];
  Pdot[2]=- PP[1][1];
  Pdot[3]=Q_gyro;
  
  PP[0][0] += Pdot[0] * dt;   // Pk-先验估计误差协方差微分的积分
  PP[0][1] += Pdot[1] * dt;   // =先验估计误差协方差
  PP[1][0] += Pdot[2] * dt;
  PP[1][1] += Pdot[3] * dt;
  
  Angle_err = Accel - Angle;        //zk-先验估计
  
  PCt_0 = C_0 * PP[0][0];
  PCt_1 = C_0 * PP[1][0];
  
  E = R_angle + C_0 * PCt_0;
  
  K_0 = PCt_0 / E;
  K_1 = PCt_1 / E;
  
  t_0 = PCt_0;
  t_1 = C_0 * PP[0][1];
  
  PP[0][0] -= K_0 * t_0;                 //后验估计误差协方差
  PP[0][1] -= K_0 * t_1;
  PP[1][0] -= K_1 * t_0;
  PP[1][1] -= K_1 * t_1;
  
  Angle += K_0 * Angle_err;         //后验估计//最优角度
  Q_bias += K_1 * Angle_err;         //后验估计//最优零漂
  K_Gyro_y = Gyro - Q_bias;         //输出值(后验估计)的微分=最优角速度
  
  return Angle;
  
}


/***************
初始化MPU6050
返回值:0,成功
 其他,错误代码
****************/
uint8_t MPU6050_Init(void)
{
    uint8_t res;
    IIC_Init();                                                  //MPU6050 支持400K I2C
    res=MPU_Read_Byte(MPU6050_ADDR,WHO_AM_I);                     //读取MPU6050的ID

    res = 0;
    res += MPU_Write_Byte(MPU6050_ADDR,MPU_PWR_MGMT1_REG,0X80);         //复位MPU6050
    time_delay_ms(100);                                                 //延时100ms
    res += MPU_Write_Byte(MPU6050_ADDR,MPU_PWR_MGMT1_REG,0X00);         //唤醒MPU6050
    res += MPU_Set_Gyro_Fsr(3);					        	        //陀螺仪传感器,±2000dps   
    res += MPU_Set_Accel_Fsr(1);					       	 	//加速度传感器,±4g
    res += MPU_Set_Rate(1000);						       	 	//设置采样率1000Hz
    res += MPU_Write_Byte(MPU6050_ADDR,MPU_CFG_REG,0x02);               //设置数字低通滤波器   98hz
    res += MPU_Write_Byte(MPU6050_ADDR,MPU_INT_EN_REG,0X00);            //关闭所有中断
    res += MPU_Write_Byte(MPU6050_ADDR,MPU_USER_CTRL_REG,0X00);         //I2C主模式关闭

    res += MPU_Write_Byte(MPU6050_ADDR,MPU_PWR_MGMT1_REG,0X01);  	//设置CLKSEL,PLL X轴为参考
    res += MPU_Write_Byte(MPU6050_ADDR,MPU_PWR_MGMT2_REG,0X00);  	//加速度与陀螺仪都工作
    
    
    return 0;
}




/**************************************
设置MPU6050陀螺仪传感器满量程范围
fsr:0,±250dps;1,±500dps;2,±1000dps;3,±2000dps
返回值:0,设置成功
其他,设置失败 
**************************************/
uint8_t MPU_Set_Gyro_Fsr(uint8_t fsr)
{
    return MPU_Write_Byte(MPU6050_ADDR,MPU_GYRO_CFG_REG,fsr<<3);//设置陀螺仪满量程范围  
}






/*****************************************
设置MPU6050加速度传感器满量程范围
fsr:0,±2g;1,±4g;2,±8g;3,±16g
返回值:0,设置成功
其他,设置失败 
****************************************/
uint8_t MPU_Set_Accel_Fsr(uint8_t fsr)
{
    return MPU_Write_Byte(MPU6050_ADDR,MPU_ACCEL_CFG_REG,fsr<<3);//设置加速度传感器满量程范围  
}








/***********************************
设置MPU6050的数字低通滤波器
lpf:数字低通滤波频率(Hz)
返回值:0,设置成功
其他,设置失败 
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
    return MPU_Write_Byte(MPU6050_ADDR,MPU_CFG_REG,data);//设置数字低通滤波器  
}






/*******************************************
设置MPU6050的采样率(假定Fs=1KHz)
rate:4~1000(Hz)
返回值:0,设置成功
其他,设置失败 
**********************************************/
uint8_t MPU_Set_Rate(uint16_t rate)
{
    uint8_t data;
    if(rate>1000)rate=1000;
    if(rate<4)rate=4;
    data=1000/rate-1;
    data=MPU_Write_Byte(MPU6050_ADDR,MPU_SAMPLE_RATE_REG,data);	//设置数字低通滤波器
    return MPU_Set_LPF(rate/2);	//自动设置LPF为采样率的一半
}





/********************************************
得到温度值
返回值:温度值(扩大了100倍)
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
得到陀螺仪值(原始值)
gx,gy,gz:陀螺仪x,y,z轴的原始读数(带符号)
返回值:0,成功
其他,错误代码
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
得到加速度值(原始值)
gx,gy,gz:陀螺仪x,y,z轴的原始读数(带符号)
返回值:0,成功
其他,错误代码
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
得到加计值、温度值、角速度值(原始值)
gx,gy,gz:陀螺仪x,y,z轴的原始读数(带符号)
返回值:0,成功
其他,错误代码
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
IIC连续写
addr:器件地址 
reg:寄存器地址
len:写入长度
buf:数据区
返回值:0,正常
其他,错误代码
******************************/
uint8_t MPU_Write_Len(uint8_t addr,uint8_t reg,uint8_t len,uint8_t *buf)
{
    return IIC_WriteMultByteToSlave(addr, reg, len, buf);
} 




/**************************
IIC连续读
addr:器件地址
reg:要读取的寄存器地址
len:要读取的长度
buf:读取到的数据存储区
返回值:0,正常
其他,错误代码
*************************/
uint8_t MPU_Read_Len(uint8_t addr,uint8_t reg,uint8_t len,uint8_t *buf)
{ 
    return IIC_ReadMultByteFromSlave(addr, reg, len, buf);     
}




/**************************
IIC写一个字节 
devaddr:器件IIC地址
reg:寄存器地址
data:数据
返回值:0,正常
其他,错误代码
**************************/
uint8_t MPU_Write_Byte(uint8_t addr,uint8_t reg,uint8_t value)
{
    return IIC_WriteByteToSlave(addr, reg, value);
}



/****************************
IIC读一个字节 
reg:寄存器地址 
返回值:读到的数据
****************************/
uint8_t MPU_Read_Byte(uint8_t addr,uint8_t reg)
{
    uint8_t value;
    IIC_ReadByteFromSlave(addr, reg, &value);
    return value;
}





