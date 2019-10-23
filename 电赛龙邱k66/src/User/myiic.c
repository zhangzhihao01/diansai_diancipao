#include "include.h"
#include "myiic.h"
/*
PTD8   SCL
PTD9   SDA
*/
void m_delay_nop(int n)
{
    uint8_t t;
    for(t=0;t<n;t++)
    {
        asm("nop");asm("nop");
        asm("nop");asm("nop");
        asm("nop");asm("nop");
        asm("nop");asm("nop");
        asm("nop");asm("nop");
    }
}
/******************************************************************************
*��  ����void IIC_Init(void)
*�����ܣ�IIC��ʼ��
*��  ������
*����ֵ����
*��  ע����
*******************************************************************************/	
void m_IIC_Init(void)
{			
    GPIO_Init(PTE,12,GPO,1);
    GPIO_Init(PTE,11,GPO,1);
	
	IIC_SCL=1;
	IIC_SDA=1;
}
/******************************************************************************
*��  ����void IIC_Start(void)
*�����ܣ�����IIC��ʼ�ź�
*��  ������
*����ֵ����
*��  ע����
*******************************************************************************/	
void m_IIC_Start(void)
{
	SDA_OUT(); //sda����� 
	IIC_SDA=1;	
	IIC_SCL=1;
	m_delay_nop(3);
 	IIC_SDA=0; //START:when CLK is high,DATA change form high to low 
	m_delay_nop(3);
	IIC_SCL=0; //ǯסI2C���ߣ�׼�����ͻ�������� 
}

/******************************************************************************
*��  ����void IIC_Stop(void)
*�����ܣ�����IICֹͣ�ź�
*��  ������
*����ֵ����
*��  ע����
*******************************************************************************/	  
void m_IIC_Stop(void)
{
	SDA_OUT(); //sda�����
	IIC_SCL=0;
	IIC_SDA=0; //STOP:when CLK is high DATA change form low to high
  m_delay_nop(3);
	IIC_SCL=1; 
	IIC_SDA=1; //����I2C���߽����ź�
  m_delay_nop(3);							   	
}

/******************************************************************************
*��  ��: uint8_t IIC_WaitAck(void)
*������: �ȴ�Ӧ���źŵ��� ����ЧӦ�𣺴ӻ���9�� SCL=0 ʱ SDA ���ӻ�����,
                            ���� SCL = 1ʱ SDA��ȻΪ�ͣ�
*��  ������
*����ֵ��1������Ӧ��ʧ��
         0������Ӧ��ɹ�
*��  ע���ӻ���������Ӧ��
*******************************************************************************/
uint8_t m_IIC_WaitAck(void)
{
	uint8_t ucErrTime=0;
	SDA_IN(); //SDA����Ϊ����  ���ӻ���һ���͵�ƽ��ΪӦ�� 
	IIC_SDA=1;m_delay_nop(3);	   
	IIC_SCL=1;m_delay_nop(3);;	 
	while(READ_SDA)
	{
		ucErrTime++;
		if(ucErrTime>250)
		{
			IIC_Stop();
			return 1;
		}
	}
	IIC_SCL=0; //ʱ�����0 	   
	return 0;  
} 

/******************************************************************************
*��  ��: void IIC_Ack(void)
*������: ����ACKӦ�� ������������һ���ֽ����ݺ�����������ACK֪ͨ�ӻ�һ��
                       �ֽ���������ȷ���գ�
*��  ������
*����ֵ����
*��  ע���������ӻ���Ӧ��
*******************************************************************************/

void m_IIC_Ack(void)
{
	IIC_SCL=0;
	SDA_OUT();
	IIC_SDA=0;
	m_delay_nop(3);
	IIC_SCL=1;
	m_delay_nop(3);
	IIC_SCL=0;
}

/******************************************************************************
*��  ��: void IIC_NAck(void)
*������: ����NACKӦ�� ���������������һ���ֽ����ݺ�����������NACK֪ͨ�ӻ�
                        ���ͽ������ͷ�SDA,�Ա���������ֹͣ�źţ�
*��  ������
*����ֵ����
*��  ע���������ӻ���Ӧ��
*******************************************************************************/
void m_IIC_NAck(void)
{
	IIC_SCL=0;
	SDA_OUT();
	IIC_SDA=1;
	m_delay_nop(3);
	IIC_SCL=1;
	m_delay_nop(3);
	IIC_SCL=0;
}					 				     

/******************************************************************************
*��  ����void IIC_SendByte(uint8_t txd)
*��  �ܣ�IIC����һ���ֽ�
*��  ����data Ҫд������
*����ֵ����
*��  ע���������ӻ���
*******************************************************************************/		  
void m_IIC_SendByte(uint8_t data)
{                        
    uint8_t t;   
	  SDA_OUT(); 	    
    IIC_SCL=0; //����ʱ�ӿ�ʼ���ݴ���
    for(t=0;t<8;t++)
    {              
      IIC_SDA=(data&0x80)>>7;
      data<<=1;
			m_delay_nop(3);			
		  IIC_SCL=1;
		  m_delay_nop(3);
			IIC_SCL=0;	
		  m_delay_nop(3);
    }	 
} 	 
   
/******************************************************************************
*��  ����uint8_t IIC_ReadByte(uint8_t ack)
*��  �ܣ�IIC��ȡһ���ֽ�
*��  ����ack=1 ʱ���������ݻ�û������ ack=0 ʱ����������ȫ���������
*����ֵ����
*��  ע���ӻ���������
*******************************************************************************/	
uint8_t m_IIC_ReadByte(uint8_t ack)
{
	uint8_t i,receive=0;
	SDA_IN(); //SDA����Ϊ����ģʽ �ȴ����մӻ���������
  for(i=0;i<8;i++ )
	{
     IIC_SCL=0; 
     m_delay_nop(3);
		 IIC_SCL=1;
     receive<<=1;
     if(READ_SDA)receive++; //�ӻ����͵ĵ�ƽ
	   m_delay_nop(3); 
   }					 
    if(ack)
        m_IIC_Ack(); //����ACK 
    else
        m_IIC_NAck(); //����nACK  
    return receive;
}

/******************************************************************************
*��  ����uint8_t IIC_ReadByteFromSlave(uint8_t I2C_Addr,uint8_t addr)
*�����ܣ���ȡָ���豸 ָ���Ĵ�����һ��ֵ
*��  ����I2C_Addr  Ŀ���豸��ַ
		     reg	     �Ĵ�����ַ
         *buf      ��ȡ����Ҫ�洢�ĵ�ַ    
*����ֵ������ 1ʧ�� 0�ɹ�
*��  ע����
*******************************************************************************/ 
uint8_t m_IIC_ReadByteFromSlave(uint8_t I2C_Addr,uint8_t reg,uint8_t *buf)
{
	m_IIC_Start();	
	m_IIC_SendByte(I2C_Addr);	 //���ʹӻ���ַ
	if(m_IIC_WaitAck()) //����ӻ�δӦ�������ݷ���ʧ��
	{
		m_IIC_Stop();
		return 1;
	}
	m_IIC_SendByte(reg); //���ͼĴ�����ַ
	m_IIC_WaitAck();	  
	
	m_IIC_Start();
	m_IIC_SendByte(I2C_Addr+1); //�������ģʽ			   
	m_IIC_WaitAck();
	*buf=m_IIC_ReadByte(0);	   
  m_IIC_Stop(); //����һ��ֹͣ����
	return 0;
}

/******************************************************************************
*��  ����uint8_t IIC_WriteByteFromSlave(uint8_t I2C_Addr,uint8_t addr��uint8_t buf))
*�����ܣ�д��ָ���豸 ָ���Ĵ�����һ��ֵ
*��  ����I2C_Addr  Ŀ���豸��ַ
		     reg	     �Ĵ�����ַ
         buf       Ҫд�������
*����ֵ��1 ʧ�� 0�ɹ�
*��  ע����
*******************************************************************************/ 
uint8_t m_IIC_WriteByteToSlave(uint8_t I2C_Addr,uint8_t reg,uint8_t data)
{
	m_IIC_Start();
	m_IIC_SendByte(I2C_Addr); //���ʹӻ���ַ
	if(m_IIC_WaitAck())
	{
		m_IIC_Stop();
		return 1; //�ӻ���ַд��ʧ��
	}
	m_IIC_SendByte(reg); //���ͼĴ�����ַ
  m_IIC_WaitAck();	  
	m_IIC_SendByte(data); 
	if(m_IIC_WaitAck())
	{
		m_IIC_Stop(); 
		return 1; //����д��ʧ��
	}
	m_IIC_Stop(); //����һ��ֹͣ����

  //return 1; //status == 0;
	return 0;
}

/******************************************************************************
*��  ����uint8_t IICreadBytes(uint8_t dev, uint8_t reg, uint8_t length, uint8_t *data)
*�����ܣ���ȡָ���豸 ָ���Ĵ����� length��ֵ
*��  ����dev     Ŀ���豸��ַ
		     reg	   �Ĵ�����ַ
         length  Ҫ�����ֽ���
		     *data   ���������ݽ�Ҫ��ŵ�ָ��
*����ֵ��1�ɹ� 0ʧ��
*��  ע����
*******************************************************************************/ 
uint8_t m_IIC_ReadMultByteFromSlave(uint8_t dev, uint8_t reg, uint8_t length, uint8_t *data)
{
  uint8_t count = 0;
	uint8_t temp;
	m_IIC_Start();
	m_IIC_SendByte(dev); //���ʹӻ���ַ
	if(m_IIC_WaitAck())
	{
		m_IIC_Stop(); 
		return 1; //�ӻ���ַд��ʧ��
	}
	m_IIC_SendByte(reg); //���ͼĴ�����ַ
  m_IIC_WaitAck();	  
	m_IIC_Start();
	m_IIC_SendByte(dev+1); //�������ģʽ	
	m_IIC_WaitAck();
  for(count=0;count<length;count++)
	{
		if(count!=(length-1))
		temp = m_IIC_ReadByte(1); //��ACK�Ķ�����
		else  
		temp = m_IIC_ReadByte(0); //���һ���ֽ�NACK

		data[count] = temp;
	}
    m_IIC_Stop(); //����һ��ֹͣ����
    //return count;
	 return 0;
}

/******************************************************************************
*��  ����uint8_t IICwriteBytes(uint8_t dev, uint8_t reg, uint8_t length, uint8_t* data)
*�����ܣ�������ֽ�д��ָ���豸 ָ���Ĵ���
*��  ����dev     Ŀ���豸��ַ
		     reg	   �Ĵ�����ַ
         length  Ҫд���ֽ���
		     *data   Ҫд������ݽ�Ҫ��ŵ�ָ��
*����ֵ��1�ɹ� 0ʧ��
*��  ע����
*******************************************************************************/ 
uint8_t m_IIC_WriteMultByteToSlave(uint8_t dev, uint8_t reg, uint8_t length, uint8_t* data)
{
  
 	uint8_t count = 0;
	m_IIC_Start();
	m_IIC_SendByte(dev); //���ʹӻ���ַ
	if(m_IIC_WaitAck())
	{
		m_IIC_Stop();
		return 1; //�ӻ���ַд��ʧ��
	}
	m_IIC_SendByte(reg); //���ͼĴ�����ַ
  m_IIC_WaitAck();	  
	for(count=0;count<length;count++)
	{
		m_IIC_SendByte(data[count]); 
		if(m_IIC_WaitAck()) //ÿһ���ֽڶ�Ҫ�ȴӻ�Ӧ��
		{
			m_IIC_Stop();
			return 1; //����д��ʧ��
		}
	}
	m_IIC_Stop(); //����һ��ֹͣ����

	return 0;
}




