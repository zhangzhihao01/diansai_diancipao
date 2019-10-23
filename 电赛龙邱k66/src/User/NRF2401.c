/*LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
��ƽ    ̨������i.MX RT1052���İ�-���ܳ���
����    д��Z
��E-mail  ��chiusir@163.com
������汾��V1.0
�������¡�2018��12��22��
�������Ϣ�ο����е�ַ��
����    վ��http://www.lqist.cn
���Ա����̡�http://shop36265907.taobao.com
------------------------------------------------
��dev.env.��IAR8.20.1�����ϰ汾
��Target �� i.MX RT1052
��Crystal�� 24.000Mhz
��ARM PLL�� 1200MHz
��SYS PLL�� 528MHz
��USB PLL�� 480MHz
================================================*/

#include "include.h"
/************************************************************************
*������ֲ�޸���
*ֻ��Ҫ����ԭ��ͼ�޸Ķ�Ӧ�Ķ˿�ʱ�� �˿� ����
оƬģʽ CE   PTB19
Ƭѡ     CS   PTB10
SPIʱ��  SCK  PTB11
������� MOSI PTB16
�������� MISO PTB17
�ж�     IRQ  PTB18     
************************************************************************/
//�������ŵ�ƽ
#define NRF_SCN_LOW   //PT2_0 = 0
#define NRF_SCN_HIGH  //PT2_0 = 1
#define NRF_CE_LOW    PTB19_OUT = 0
#define NRF_CE_HIGH   PTB19_OUT = 1
#define NRF_SPI         SPI_1
#define NRF_CS          SPI_PCS1
u8 tmp_buf[33];
u8 tmp_nuf[33];
float *pars[20];
float *vra[7];
float vratmp[7]={1,1,1,1,1,1,1};












/******************************************************
��ȡ���ŵ�ƽ
#define NRF_IRQ_READ  (NRF_IRQ_PORT->IDR & NRF_IRQ)
#define NRFAddrMax 50 //NRF���һ���ֽڵ�ַ���Ϊ50
*****************************************************/
uint8_t NRFaddr = 0xFF; //��ʼ��NRF���һ�ֽڵ�ַ

uint8_t NRF_TX_DATA[TX_PAYLO_WIDTH];//NRF���ͻ�����
uint8_t NRF_RX_DATA[RX_PAYLO_WIDTH];//NRF���ջ�����

uint8_t TX_ADDRESS[TX_ADR_WIDTH]={0x34,0x43,0xac,0x10,0xee}; //���͵�ַ
uint8_t RX_ADDRESS[RX_ADR_WIDTH]={0x34,0x43,0xac,0x10,0xee}; //���յ�ַ

void NRF24L01_Config(void);
void NRF_GetAddr(void);
uint8_t NRF_WriteReadByte(uint8_t data)
{
    uint8_t buff = data;
   PTB10_OUT=0;
    spi_mosi(NRF_SPI, NRF_CS, &buff, &buff, 1); //����buff���ݣ������浽buff��
    PTB10_OUT=1;
    return buff;
}
void delayus(u32 us)
{
    u16 i;
    while(us--)
    {
        for(i=0;i<10;i++);
    }
}
void delay_ms(u32 ms)
{
    delayus(ms * 1000);
}





/*****************************************************************************
*��  ����void NRF24l01_Init(void)
*��  �ܣ�NRF����GPIO��ʼ��
*��  ������
*����ֵ����
*��  ע����
*****************************************************************************/
void NRF24l01_Init(void)
{
    u8 sta;
    spi_init(NRF_SPI, NRF_CS, MASTER,5000*1000);                     //��ʼ��SPI,����ģʽ
    GPIO_Init (PTB, 19, GPO,0);                                        //��ʼ��CE��Ĭ�Ͻ������ģʽ
    GPIO_Init (PTB, 10, GPO,1);
    GPIO_Init(PTB,18,GPI,0);

    sta=NRF24l01_read_reg(RF_STATUS);
    NRF24l01_write_reg(W_REGISTER+RF_STATUS,sta);//����жϱ�־
    NRF_SCN_HIGH; //ʧ��NRF
    
    NRF_CE_LOW; //����ģʽ
 NRF24L01_Check(); //���NRF24L01�Ƿ���MCUͨ�� 
 //GPIO_Reverse(PTA, 17);
 delay_ms(200);
 NRF24l01_write_reg(FLUSH_TX,0xff);//���TX FIFO�Ĵ��� 
 NRF24l01_write_reg(FLUSH_RX,0xff);//���RX FIFO�Ĵ��� 
 NRF24L01_TX_Mode();
 NRF24L01_RX_Mode();


}






/*****************************************************************************
*��  ����uint8_t NRF24l01_write_reg(uint8_t reg,uint8_t value)
*��  �ܣ�дһ�ֽ����ݵ��Ĵ���
*��  ����reg�� �Ĵ�����ַ
*        val:  Ҫд�������
*����ֵ��status
*��  ע��NRF2401������ֲֻ���SPI�����޸ĳ��Լ��ļ���
*****************************************************************************/
uint8_t NRF24l01_write_reg(uint8_t reg,uint8_t value)
{
	uint8_t buff[2];

    buff[0] = reg;          //�ȷ��ͼĴ���
    buff[1] = value;          //�ٷ�������
PTB10_OUT=0;
    spi_mosi(NRF_SPI, NRF_CS, buff, buff, 2); //����buff�����ݣ����ɼ��� buff��
PTB10_OUT=1;
    /*����״̬�Ĵ�����ֵ*/
    return buff[0];
}






/*****************************************************************************
*��  ����uint8_t NRF24l01_read_reg(uint8_t reg)
*��  �ܣ���һ�ֽ����ݵ��Ĵ���
*��  ����reg�� �Ĵ�����ַ
*����ֵ��reg_val
*��  ע��NRF2401������ֲֻ���SPI�����޸ĳ��Լ��ļ���
*****************************************************************************/
uint8_t NRF24l01_read_reg(uint8_t reg)
{
	uint8 buff[2];

    buff[0] = reg;          //�ȷ��ͼĴ���
PTB10_OUT=0;
    spi_mosi(NRF_SPI, NRF_CS, buff, buff, 2); //����buff���ݣ������浽buff��
PTB10_OUT=1;
    /*���ؼĴ�����ֵ*/
    return buff[1];
}











/*****************************************************************************
*��  ����uint8_t NRF24L01_Write_Buf(uint8_t reg, uint8_t *pBuf, uint8_t len)
*��  �ܣ�дһ�����ݵ��Ĵ���
*��  ����reg�� �Ĵ�����ַ
*       pBuf�� Ҫд�����ݵĵ�ַ
*        len:  Ҫд������ݳ���
*����ֵ��status
*��  ע��NRF2401������ֲֻ���SPI�����޸ĳ��Լ��ļ���
*****************************************************************************/
uint8_t NRF24L01_Write_Buf(uint8_t reg, uint8_t *pBuf, uint8_t len)
{
  PTB10_OUT=0;
	spi_mosi_cmd(NRF_SPI, NRF_CS, &reg , &reg, pBuf, NULL, 1 , len); //���� reg ��pBuf ���ݣ�������
   PTB10_OUT=1;
    return reg;    //����NRF24L01��״̬
}











/*****************************************************************************
*��  ����uint8_t NRF24l01_read_reg(uint8_t reg, uint8_t *pBuf, uint8_t len)
*��  �ܣ���һ�����ݵ��Ĵ���
*��  ����reg�� �Ĵ�����ַ
*       pBuf�� Ҫ��ȡ���ݵĵ�ַ
*        len:  Ҫ��ȡ�����ݳ���
*����ֵ��status
*��  ע��NRF2401������ֲֻ���SPI�����޸ĳ��Լ��ļ���
*****************************************************************************/
uint8_t NRF24L01_Read_Buf(uint8_t reg, uint8_t *pBuf, uint8_t len)
{
  PTB10_OUT=0;
	spi_mosi_cmd(NRF_SPI, NRF_CS, &reg , &reg, NULL, pBuf, 1 , len); //����reg�����յ�buff
PTB10_OUT=1;
    return reg;    //����NRF24L01��״̬
}








/*****************************************************************************
*��  ����uint8_t NRF24L01_testConnection(void)
*��  �ܣ����NRF2401��MCU��SPI�����Ƿ�ͨ������
*��  ������
*����ֵ��1������ 0δ����
*��  ע����
*****************************************************************************/
uint8_t NRF24L01_testConnection(void)
{
	uint8_t buf[5]={0XA5,0XA5,0XA5,0XA5,0XA5};
	uint8_t i; 	 
	NRF24L01_Write_Buf(W_REGISTER+TX_ADDR,buf,5); //д��5���ֽڵĵ�ַ.	
	NRF24L01_Read_Buf(TX_ADDR,buf,5); //����д��ĵ�ַ  
	for(i=0;i<5;i++)
        if(buf[i]!=0XA5)break;	 							   
	if(i!=5)return 0; //���24L01����	
	return 1;	//��⵽24L01
}	
u8 NRF24L01_Check(void)
{
   // char txt[16]="    ";
	while(!NRF24L01_testConnection())
	{
        //sprintf(txt,"NRF_FAIL");
        //LCD_P8x16Str(35,2,(unsigned char *)txt);
//		printf("\rNRF2401 no connect...\r\n");
		GPIO_Reverse(PTA, 17);   //LEDָʾ��������״̬
        delay_ms(100);
	}
    //sprintf(txt,"NRF_OK");
    //LCD_P8x16Str(35,2,(unsigned char *)txt);
        return 0;
}












/***********************************
����Ϊ����ԭ������
����NRF24L01����һ������
txbuf:�����������׵�ַ
����ֵ:�������״��
***********************************/
u8 NRF24L01_TxPacket(u8 *txbuf)
{
	 NRF24l01_write_reg(W_REGISTER+CONFIG,0x0e);    //���û�������ģʽ�Ĳ���;PWR_UP,EN_CRC,16BIT_CRC,����ģʽ,���������ж�
	u8 sta;
 	//SPI1_SetSpeed(SPI_BaudRatePrescaler_16);//spi�ٶ�Ϊ10.5Mhz��24L01�����SPIʱ��Ϊ10Mhz��   
	NRF_CE_LOW;
  NRF24L01_Write_Buf(W_TX_PAYLOAD,txbuf,TX_PAYLO_WIDTH);//д���ݵ�TX BUF  32���ֽ�
 	NRF_CE_HIGH;//��������	   
	while(NRF24L01_IRQ!=0);//�ȴ��������
	//delay_ms(1);
	sta=NRF24l01_read_reg(RF_STATUS);  //��ȡ״̬�Ĵ�����ֵ	   
	NRF24l01_write_reg(W_REGISTER+RF_STATUS,sta); //���TX_DS��MAX_RT�жϱ�־
       NRF24l01_write_reg(W_REGISTER+CONFIG,0x0f);    //���û�������ģʽ�Ĳ���;PWR_UP,EN_CRC,16BIT_CRC,����ģʽ,���������ж�
	if(sta&MAX_TX)//�ﵽ����ط�����
	{
		NRF24l01_write_reg(FLUSH_TX,0xff);//���TX FIFO�Ĵ��� 
                GPIO_Reverse(PTD, 15);
		return MAX_TX; 
	}
	if(sta&TX_OK)//�������
	{
          GPIO_Reverse(PTC, 0);
          return TX_OK;
	}
        GPIO_Reverse(PTA, 17);
	return sta;//����ԭ����ʧ��
}







/**************************************
����NRF24L01����һ������
txbuf:�����������׵�ַ
����ֵ:0��������ɣ��������������
**************************************/
u8 NRF24L01_RxPacket(u8 *rxbuf)
{
	u8 sta;		    							   
	//SPI1_SetSpeed(SPI_BaudRatePrescaler_16); //spi�ٶ�Ϊ10.5Mhz��24L01�����SPIʱ��Ϊ10Mhz��   
	sta=NRF24l01_read_reg(RF_STATUS);  //��ȡ״̬�Ĵ�����ֵ    	 
	NRF24l01_write_reg(W_REGISTER+RF_STATUS,sta); //���TX_DS��MAX_RT�жϱ�־
	if(sta&RX_OK)//���յ�����
	{
		NRF24L01_Read_Buf(R_RX_PAYLOAD,rxbuf,RX_PAYLO_WIDTH);//��ȡ����
		NRF24l01_write_reg(FLUSH_RX,0xff);//���RX FIFO�Ĵ��� 
                GPIO_Reverse(PTE, 26);
		return 0; 
	}	   
	return 1;//û�յ��κ�����
}





/****************************************************
�ú�����ʼ��NRF24L01��RXģʽ
����RX��ַ,дRX���ݿ��,ѡ��RFƵ��,�����ʺ�LNA HCURR
��CE��ߺ�,������RXģʽ,�����Խ���������
*****************************************************/		   
void NRF24L01_RX_Mode(void)
{
  NRF_CE_LOW;	  
  NRF24L01_Write_Buf(W_REGISTER+RX_ADDR_P0,(u8*)RX_ADDRESS,RX_ADR_WIDTH);//дRX�ڵ��ַ
	
  NRF24l01_write_reg(W_REGISTER+EN_AA,0x01);    //ʹ��ͨ��0���Զ�Ӧ��    
  NRF24l01_write_reg(W_REGISTER+EN_RXADDR,0x01);//ʹ��ͨ��0�Ľ��յ�ַ  
  NRF24l01_write_reg(W_REGISTER+SETUP_AW,0x03);	
  NRF24l01_write_reg(W_REGISTER+RF_CH,40);	     //����RFͨ��Ƶ��		  
  NRF24l01_write_reg(W_REGISTER+RX_PW_P0,RX_PAYLO_WIDTH);//ѡ��ͨ��0����Ч���ݿ�� 	    
  NRF24l01_write_reg(W_REGISTER+RF_SETUP,0x07);//����TX�������,0db����,2Mbps,���������濪��   
  NRF24l01_write_reg(W_REGISTER+CONFIG, 0x0f);//���û�������ģʽ�Ĳ���;PWR_UP,EN_CRC,16BIT_CRC,����ģʽ 
  NRF_CE_HIGH; //CEΪ��,�������ģʽ 
}




/*****************************************************************************************
�ú�����ʼ��NRF24L01��TXģʽ
����TX��ַ,дTX���ݿ��,����RX�Զ�Ӧ��ĵ�ַ,���TX��������,ѡ��RFƵ��,�����ʺ�LNA HCURR
PWR_UP,CRCʹ��
��CE��ߺ�,������RXģʽ,�����Խ���������		   
CEΪ�ߴ���10us,����������.
*****************************************************************************************/	 
void NRF24L01_TX_Mode(void)
{														 
	NRF_CE_LOW;	    
  NRF24L01_Write_Buf(W_REGISTER+TX_ADDR,(u8*)TX_ADDRESS,TX_ADR_WIDTH);//дTX�ڵ��ַ 
  NRF24L01_Write_Buf(W_REGISTER+RX_ADDR_P0,(u8*)RX_ADDRESS,RX_ADR_WIDTH); //����TX�ڵ��ַ,��ҪΪ��ʹ��ACK	  

  NRF24l01_write_reg(W_REGISTER+EN_AA,0x01);     //ʹ��ͨ��0���Զ�Ӧ��    
  NRF24l01_write_reg(W_REGISTER+EN_RXADDR,0x01); //ʹ��ͨ��0�Ľ��յ�ַ  
	NRF24l01_write_reg(W_REGISTER+SETUP_AW,0x03);			
  NRF24l01_write_reg(W_REGISTER+SETUP_RETR,0x1a);//�����Զ��ط����ʱ��:500us + 86us;����Զ��ط�����:10��
  NRF24l01_write_reg(W_REGISTER+RF_CH,40);       //����RFͨ��Ϊ40
  NRF24l01_write_reg(W_REGISTER+RF_SETUP,0x07);  //����TX�������,0db����,2Mbps,���������濪��   
  NRF24l01_write_reg(W_REGISTER+CONFIG,0x0e);    //���û�������ģʽ�Ĳ���;PWR_UP,EN_CRC,16BIT_CRC,����ģʽ,���������ж�
	NRF_CE_HIGH;//CEΪ��,10us����������
}


/***************************
����һ���ַ�
**************************/
void sendvra(void)
{
  rxcheck();
  int vra32;
  tmp_buf[0]=dattxed;
  u8 i;
  for(i=0;i<7;i++)
  {
      *((int*)&tmp_buf[i*4+3])=*(int*)vra[i];
      vra32=*((int*)&tmp_buf[i*4+3]);
      vra32++;
  }
  NRF24L01_TxPacket(tmp_buf);
  NRF24L01_RX_Mode();
}





/*********************************
2401����
********************************/
u8 rxcheck(void)
{
  int num=0,i;
  if(!NRF24L01_RxPacket(tmp_buf))
  {NRF24L01_RX_Mode();
    for(i=1;i<4;i++)
    {
      num+=tmp_nuf[i];
      num<<=8;
    }
    num+=tmp_nuf[i];
    switch(tmp_buf[0])
    {
    case datrxed:if(tmp_buf[1]%2)
      {
        *pars[tmp_buf[1]/2]=*((float*)&tmp_buf[8]);
      }break;
    case carstop:while(1)GPIO_Reverse(PTE,26);
    default: break;
    }
    return tmp_buf[0];
  }
  else
  {
    NRF24L01_RX_Mode();
    return tmp_buf[0];
  }
}



/*********************************
2401����
********************************/
void sendState(u8 state,int num)
{
  NRF24L01_TX_Mode();
  u8 text[33],i;
  text[0]=state;
  for(i=4;i>1;i--)
  {
    text[i]=num&0xff;
    num>>=8;
  }
  for(i=0;i<20;i++)
    NRF24L01_TxPacket(text);
}
