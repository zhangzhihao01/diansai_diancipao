#ifndef __Serial_oscilloscope_H__
#define __Serial_oscilloscope_H__

void Data_Send(UARTn_e uratn,unsigned short int *data);
void Send_MotoPWM(UARTn_e uratn,int m_1,int m_2);
void Send_PID(UARTn_e uratn,float p1,float i1,float d1,float p2,float i2,float d2,float p3,float i3,float d3);
#endif