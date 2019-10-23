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

#include"include.h"




//unsigned char data_to_send[16];
void Data_Send(UARTn_e uratn,unsigned short int *pst)
{
        unsigned char _cnt=0;	unsigned char sum = 0;
	unsigned char data_to_send[23];         //���ͻ���
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0x02;
	data_to_send[_cnt++]=0;
	data_to_send[_cnt++]=(unsigned char)(pst[0]>>8);  //��8λ
	data_to_send[_cnt++]=(unsigned char)pst[0];  //��8λ
	data_to_send[_cnt++]=(unsigned char)(pst[1]>>8);
	data_to_send[_cnt++]=(unsigned char)pst[1];
	data_to_send[_cnt++]=(unsigned char)(pst[2]>>8);
	data_to_send[_cnt++]=(unsigned char)pst[2];
	data_to_send[_cnt++]=(unsigned char)(pst[3]>>8);
	data_to_send[_cnt++]=(unsigned char)pst[3];
	data_to_send[_cnt++]=(unsigned char)(pst[4]>>8);
	data_to_send[_cnt++]=(unsigned char)pst[4];
	data_to_send[_cnt++]=(unsigned char)(pst[5]>>8);
	data_to_send[_cnt++]=(unsigned char)pst[5];
	data_to_send[_cnt++]=(unsigned char)(pst[6]>>8);
	data_to_send[_cnt++]=(unsigned char)pst[6];
	data_to_send[_cnt++]=(unsigned char)(pst[7]>>8);
	data_to_send[_cnt++]=(unsigned char)pst[7];
	data_to_send[_cnt++]=(unsigned char)(pst[8]>>8);
	data_to_send[_cnt++]=(unsigned char)pst[8];
	
	
	data_to_send[3] = _cnt-4;
	
	sum = 0;
        
	for(unsigned char i=0;i<_cnt;i++)
		sum += data_to_send[i];
        
	data_to_send[_cnt++] = sum;

	
        
        
        for(unsigned char i=0;i<_cnt;i++)
        UART_Put_Char(uratn,data_to_send[i]);
        
      
}

void Send_PID(UARTn_e uratn,float p1,float i1,float d1,float p2,float i2,float d2,float p3,float i3,float d3)
{
        unsigned char _cnt=0;	unsigned char sum = 0;
	unsigned char data_to_send[23];         //���ͻ���
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0x10;
	data_to_send[_cnt++]=0;
	data_to_send[_cnt++]=(unsigned char)((int)p1>>8);  //��8λ
	data_to_send[_cnt++]=(unsigned char)p1;  //��8λ
	data_to_send[_cnt++]=(unsigned char)((int)i1>>8);
	data_to_send[_cnt++]=(unsigned char)i1;
	data_to_send[_cnt++]=(unsigned char)((int)d1>>8);
	data_to_send[_cnt++]=(unsigned char)d1;
	data_to_send[_cnt++]=(unsigned char)((int)p2>>8);
	data_to_send[_cnt++]=(unsigned char)p2;
	data_to_send[_cnt++]=(unsigned char)((int)i2>>8);
	data_to_send[_cnt++]=(unsigned char)i2;
	data_to_send[_cnt++]=(unsigned char)((int)d2>>8);
	data_to_send[_cnt++]=(unsigned char)d2;
	data_to_send[_cnt++]=(unsigned char)((int)p3>>8);
	data_to_send[_cnt++]=(unsigned char)p3;
	data_to_send[_cnt++]=(unsigned char)((int)i3>>8);
	data_to_send[_cnt++]=(unsigned char)i3;
	data_to_send[_cnt++]=(unsigned char)((int)d3>>8);
	data_to_send[_cnt++]=(unsigned char)d3;
	
	data_to_send[3] = _cnt-4;
	
	sum = 0;
        
	for(unsigned char i=0;i<_cnt;i++)
		sum += data_to_send[i];
        
	data_to_send[_cnt++] = sum;

	
        
        
        for(unsigned char i=0;i<_cnt;i++)
        UART_Put_Char(uratn,data_to_send[i]);
        
      
}
void Send_MotoPWM(UARTn_e uratn,int MotorLeft,int MotorRight)
{
        unsigned char _cnt=0;	unsigned char sum = 0;
	unsigned char data_to_send[23];         //���ͻ���
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0x06;
	data_to_send[_cnt++]=0;
	data_to_send[_cnt++]=(unsigned char)(MotorLeft>>8);  //��8λ
	data_to_send[_cnt++]=(unsigned char)MotorLeft;  //��8λ
	data_to_send[_cnt++]=(unsigned char)(MotorRight>>8);
	data_to_send[_cnt++]=(unsigned char)MotorRight;
	data_to_send[_cnt++]=0;
	data_to_send[_cnt++]=0;
	data_to_send[_cnt++]=0;
	data_to_send[_cnt++]=0;
	data_to_send[_cnt++]=0;
	data_to_send[_cnt++]=0;
	data_to_send[_cnt++]=0;
	data_to_send[_cnt++]=0;
	data_to_send[_cnt++]=0;
	data_to_send[_cnt++]=0;
	data_to_send[_cnt++]=0;
	data_to_send[_cnt++]=0;
	data_to_send[_cnt++]=0;
	data_to_send[_cnt++]=0;
	
	data_to_send[3] = _cnt-4;
	
	sum = 0;
        
	for(unsigned char i=0;i<_cnt;i++)
		sum += data_to_send[i];
        
	data_to_send[_cnt++] = sum;

	
        
        
        for(unsigned char i=0;i<_cnt;i++)
        UART_Put_Char(uratn,data_to_send[i]);
        
      
}