#ifndef __U_IIC_H
#define __U_IIC_H


#include "include.h"


#define SCL_PORT  PTE    //SCLʹ��D�˿�     1.�޸�ģ��IIC����ʱ��Ҫ�޸�
#define SCL_INDEX 12       //SCLʹ��D8����
#define SDA_PORT  PTE    //SDAʹ��D�˿�     
#define SDA_INDEX 11      //SDAʹ��D9����
//����MPU6050�ӿڣ�GPIOģ��IIC

#define SDA_IN()  {GPIO_PDDR_REG(SDA_PORT) &= ~(1<<SDA_INDEX);}	//����
#define SDA_OUT() {GPIO_PDDR_REG(SDA_PORT) |= (1<<SDA_INDEX);} //���

//IO��������	 
#define IIC_SCL    PTE12_OUT //SCL             2.�޸�ģ��iic����ʱ��Ҫ�޸�
#define IIC_SDA    PTE11_OUT //SDA	 
#define READ_SDA   PTE11_IN  //����SDA 


//IIC_1���в�������
void delay_nop(int n);
void m_IIC_Init(void);        //��ʼ��IIC��IO��3.�޸�ģ��iic����ʱ��Ҫ�޸�			 
void m_IIC_Start(void);			  //����IIC��ʼ�ź�
void m_IIC_Stop(void);	  	  //����IICֹͣ�ź�
void m_IIC_Ack(void);					//IIC����ACK�ź�
void m_IIC_NAck(void);				//IIC������ACK�ź�
uint8_t m_IIC_WaitAck(void); 		 //IIC�ȴ�ACK�ź�
void m_IIC_SendByte(uint8_t data);  //IIC����һ���ֽ�
uint8_t m_IIC_ReadByte(uint8_t ack);//IIC��ȡһ���ֽ�
uint8_t m_IIC_ReadByteFromSlave(uint8_t I2C_Addr,uint8_t reg,uint8_t *buf);
uint8_t m_IIC_ReadMultByteFromSlave(uint8_t dev, uint8_t reg, uint8_t length, uint8_t *data);
uint8_t m_IIC_WriteByteToSlave(uint8_t I2C_Addr,uint8_t reg,uint8_t buf);
uint8_t m_IIC_WriteMultByteToSlave(uint8_t dev, uint8_t reg, uint8_t length, uint8_t* data);



#endif