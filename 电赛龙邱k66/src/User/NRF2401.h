#ifndef __NRF2401_H
#define __NRF2401_H

#include "stdint.h" 
/******* NRF24L01���� *********/
#define R_REGISTER   0X00//�����üĴ���
#define W_REGISTER   0x20//д���üĴ���
#define R_RX_PAYLOAD 0X61//��RX��Ч����
#define W_TX_PAYLOAD 0XA0//дTX��Ч����
#define FLUSH_TX     0XE1//���TX FIFO�Ĵ��� Ӧ���ڷ���ģʽ
#define FLUSH_RX     0XE2//���RX FIFO�Ĵ��� Ӧ���ڽ���ģʽ
#define REUSE_TX_PL  0XE3//����ʹ����һ���ݰ�
#define NOP          0XFF
/*******************************************************NRF24L01�Ĵ�����ַ*******************************************************/
#define CONFIG       0X00 //bit0(PRIM_RX ):1����ģʽ 0����ģʽ bit1(pwr_up):1�ϵ�0����bit3��1 16λCRCУ�� 0 ��λCRCУ��
//bit3:CRCʹ�� bit4:�������ж�MAX_RT bit5:�������ж�TX_DS bit6�������ж�RX_RD
#define EN_AA        0X01//ʹ��0����5ͨ�����Զ�Ӧ����
#define EN_RXADDR    0X02//���յ�ַ���� 0-5 ͨ�� ��Ĭ��ͨ��0ͨ��1����
#define SETUP_AW     0X03//���õ�ַ��� 00����Ч 01��3�ֽ� 10���ֽ� 11��5�ֽ�
#define SETUP_RETR   0X04//�����Զ��ط� 3:0 �Զ��ط����� 7:4 �Զ��ط���ʱ
#define RF_CH        0X05//��Ƶͨ�� 6:0 ����nRF24l01����Ƶ��
#define RF_SETUP     0X06//��Ƶ�Ĵ��� 0:�������Ŵ������� 2:1 ���书�� 3:����Ч��
#define RF_STATUS    0X07//״̬�Ĵ��� 0:TX FIFO�Ĵ�������־ 3:1 ��������ͨ���� 4:�ﵽ����ط��ж�
//5�����ݷ�������ж� 6�����ݽ����ж�
#define MAX_TX  		         0x10  //����ط�����
#define TX_OK   		         0x20  //�������
#define RX_OK   		         0x40  //�������
#define RX_P_NO              0x0E  //
#define OBSERVE_TX   0X08//3:0 �ط�������(���������ݰ�ʱ��λ) 7:4 ���ݰ���ʧ������(дRF_CHʱ��λ)
#define CD           0X09//�ز����
#define RX_ADDR_P0   0X0A//����ͨ��0���յ�ַ ����󳤶�:5�ֽ�(��д���ֽڣ���д�ֽ������SETUP_AW�趨)
#define RX_ADDR_P1   0X0B//����ͨ��1���յ�ַ ����󳤶�:5�ֽ�(��д���ֽڣ���д�ֽ������SETUP_AW�趨)
#define RX_ADDR_P2   0X0C//����ͨ��2���յ�ַ ,����ֽڿ����á����ֽڲ��ֱ�����RX_ADDR_P1[39:8]���
#define RX_ADDR_P3   0X0D//����ͨ��3���յ�ַ ,����ֽڿ����á����ֽڲ��ֱ�����RX_ADDR_P1[39:8]���
#define RX_ADDR_P4   0X0E//����ͨ��4���յ�ַ ,����ֽڿ����á����ֽڲ��ֱ�����RX_ADDR_P1[39:8]���
#define RX_ADDR_P5   0X0F//����ͨ��5���յ�ַ ,����ֽڿ����á����ֽڲ��ֱ�����RX_ADDR_P1[39:8]���
#define TX_ADDR      0X10//���͵�ַ 39:0
#define RX_PW_P0     0X11//��������ͨ��0��Ч���ݿ��(��1��32�ֽ�)
#define RX_PW_P1     0X12//��������ͨ��1��Ч���ݿ��(��1��32�ֽ�)
#define RX_PW_P2     0X13//��������ͨ��2��Ч���ݿ��(��1��32�ֽ�)
#define RX_PW_P3     0X14//��������ͨ��3��Ч���ݿ��(��1��32�ֽ�)
#define RX_PW_P4     0X15//��������ͨ��4��Ч���ݿ��(��1��32�ֽ�)
#define RX_PW_P5     0X16//��������ͨ��5��Ч���ݿ��(��1��32�ֽ�)
#define FIFO_STATUS  0X17//FIFO״̬�Ĵ��� 0:RX FIFO�Ĵ����ձ�־ 1:RX FIFO�Ĵ�������־ 4:TX FIFO�Ĵ����ձ�־

#define  RX_DR 6 //���ݽ�������жϱ�־λ
#define  TX_DR 5 //���ݷ�������жϱ�־λ (״̬�Ĵ���λ��)
#define IT_TX 0x7E //����ģʽ
#define IT_RX 0x7F //����ģʽ


#define TX_ADR_WIDTH 5
#define RX_ADR_WIDTH 5
#define TX_PAYLO_WIDTH 32
#define RX_PAYLO_WIDTH 32
#define NRF24L01_IRQ GPIO_Get(PTB18)

extern uint8_t NRF_TX_DATA[TX_PAYLO_WIDTH];
extern uint8_t NRF_RX_DATA[TX_PAYLO_WIDTH];

void NRF24l01_Init(void);		
u8 NRF24L01_Check(void);
void NRFset_Mode(uint8_t mode);
uint8_t NRF24L01_Write_Buf(uint8_t reg, uint8_t *pBuf, uint8_t len);
uint8_t NRF24L01_Read_Buf(uint8_t reg, uint8_t *pBuf, uint8_t len);			  
uint8_t NRF24l01_read_reg(uint8_t reg);					
uint8_t NRF24l01_write_reg(uint8_t reg, uint8_t value);	


u8 NRF24L01_TxPacket(u8 *txbuf);//����һ����������
u8 NRF24L01_RxPacket(u8 *rxbuf);//����һ����������	
void NRF24L01_RX_Mode(void);//����Ϊ����ģʽ
void NRF24L01_TX_Mode(void);//����Ϊ����ģʽ
void sendvra(void);     //����һ������
u8 rxcheck(void);            //���ձ���
void sendState(u8 state,int num);   //����

#define dattxed 0x01
#define datrxed 0x02
#define carstop 0x03


#endif
