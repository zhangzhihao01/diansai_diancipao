/*LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
【平    台】龙邱i.MX RT1052核心板-智能车板
【编    写】Z
【E-mail  】chiusir@163.com
【软件版本】V1.0
【最后更新】2018年12月22日
【相关信息参考下列地址】
【网    站】http://www.lqist.cn
【淘宝店铺】http://shop36265907.taobao.com
------------------------------------------------
【dev.env.】IAR8.20.1及以上版本
【Target 】 i.MX RT1052
【Crystal】 24.000Mhz
【ARM PLL】 1200MHz
【SYS PLL】 528MHz
【USB PLL】 480MHz
================================================*/

#include "include.h"
/************************************************************************
*代码移植修改区
*只需要根据原理图修改对应的端口时钟 端口 引脚
芯片模式 CE   PTB19
片选     CS   PTB10
SPI时钟  SCK  PTB11
主机输出 MOSI PTB16
主机输入 MISO PTB17
中断     IRQ  PTB18     
************************************************************************/
//设置引脚电平
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
读取引脚电平
#define NRF_IRQ_READ  (NRF_IRQ_PORT->IDR & NRF_IRQ)
#define NRFAddrMax 50 //NRF最后一个字节地址最大为50
*****************************************************/
uint8_t NRFaddr = 0xFF; //初始化NRF最后一字节地址

uint8_t NRF_TX_DATA[TX_PAYLO_WIDTH];//NRF发送缓冲区
uint8_t NRF_RX_DATA[RX_PAYLO_WIDTH];//NRF接收缓冲区

uint8_t TX_ADDRESS[TX_ADR_WIDTH]={0x34,0x43,0xac,0x10,0xee}; //发送地址
uint8_t RX_ADDRESS[RX_ADR_WIDTH]={0x34,0x43,0xac,0x10,0xee}; //接收地址

void NRF24L01_Config(void);
void NRF_GetAddr(void);
uint8_t NRF_WriteReadByte(uint8_t data)
{
    uint8_t buff = data;
   PTB10_OUT=0;
    spi_mosi(NRF_SPI, NRF_CS, &buff, &buff, 1); //发送buff数据，并保存到buff里
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
*函  数：void NRF24l01_Init(void)
*功  能：NRF引脚GPIO初始化
*参  数：无
*返回值：无
*备  注：无
*****************************************************************************/
void NRF24l01_Init(void)
{
    u8 sta;
    spi_init(NRF_SPI, NRF_CS, MASTER,5000*1000);                     //初始化SPI,主机模式
    GPIO_Init (PTB, 19, GPO,0);                                        //初始化CE，默认进入待机模式
    GPIO_Init (PTB, 10, GPO,1);
    GPIO_Init(PTB,18,GPI,0);

    sta=NRF24l01_read_reg(RF_STATUS);
    NRF24l01_write_reg(W_REGISTER+RF_STATUS,sta);//清除中断标志
    NRF_SCN_HIGH; //失能NRF
    
    NRF_CE_LOW; //待机模式
 NRF24L01_Check(); //检查NRF24L01是否与MCU通信 
 //GPIO_Reverse(PTA, 17);
 delay_ms(200);
 NRF24l01_write_reg(FLUSH_TX,0xff);//清除TX FIFO寄存器 
 NRF24l01_write_reg(FLUSH_RX,0xff);//清除RX FIFO寄存器 
 NRF24L01_TX_Mode();
 NRF24L01_RX_Mode();


}






/*****************************************************************************
*函  数：uint8_t NRF24l01_write_reg(uint8_t reg,uint8_t value)
*功  能：写一字节数据到寄存器
*参  数：reg： 寄存器地址
*        val:  要写入的数据
*返回值：status
*备  注：NRF2401代码移植只需把SPI驱动修改成自己的即可
*****************************************************************************/
uint8_t NRF24l01_write_reg(uint8_t reg,uint8_t value)
{
	uint8_t buff[2];

    buff[0] = reg;          //先发送寄存器
    buff[1] = value;          //再发送数据
PTB10_OUT=0;
    spi_mosi(NRF_SPI, NRF_CS, buff, buff, 2); //发送buff里数据，并采集到 buff里
PTB10_OUT=1;
    /*返回状态寄存器的值*/
    return buff[0];
}






/*****************************************************************************
*函  数：uint8_t NRF24l01_read_reg(uint8_t reg)
*功  能：读一字节数据到寄存器
*参  数：reg： 寄存器地址
*返回值：reg_val
*备  注：NRF2401代码移植只需把SPI驱动修改成自己的即可
*****************************************************************************/
uint8_t NRF24l01_read_reg(uint8_t reg)
{
	uint8 buff[2];

    buff[0] = reg;          //先发送寄存器
PTB10_OUT=0;
    spi_mosi(NRF_SPI, NRF_CS, buff, buff, 2); //发送buff数据，并保存到buff里
PTB10_OUT=1;
    /*返回寄存器的值*/
    return buff[1];
}











/*****************************************************************************
*函  数：uint8_t NRF24L01_Write_Buf(uint8_t reg, uint8_t *pBuf, uint8_t len)
*功  能：写一组数据到寄存器
*参  数：reg： 寄存器地址
*       pBuf： 要写入数据的地址
*        len:  要写入的数据长度
*返回值：status
*备  注：NRF2401代码移植只需把SPI驱动修改成自己的即可
*****************************************************************************/
uint8_t NRF24L01_Write_Buf(uint8_t reg, uint8_t *pBuf, uint8_t len)
{
  PTB10_OUT=0;
	spi_mosi_cmd(NRF_SPI, NRF_CS, &reg , &reg, pBuf, NULL, 1 , len); //发送 reg ，pBuf 内容，不接收
   PTB10_OUT=1;
    return reg;    //返回NRF24L01的状态
}











/*****************************************************************************
*函  数：uint8_t NRF24l01_read_reg(uint8_t reg, uint8_t *pBuf, uint8_t len)
*功  能：读一组数据到寄存器
*参  数：reg： 寄存器地址
*       pBuf： 要读取数据的地址
*        len:  要读取的数据长度
*返回值：status
*备  注：NRF2401代码移植只需把SPI驱动修改成自己的即可
*****************************************************************************/
uint8_t NRF24L01_Read_Buf(uint8_t reg, uint8_t *pBuf, uint8_t len)
{
  PTB10_OUT=0;
	spi_mosi_cmd(NRF_SPI, NRF_CS, &reg , &reg, NULL, pBuf, 1 , len); //发送reg，接收到buff
PTB10_OUT=1;
    return reg;    //返回NRF24L01的状态
}








/*****************************************************************************
*函  数：uint8_t NRF24L01_testConnection(void)
*功  能：检查NRF2401与MCU的SPI总线是否通信正常
*参  数：无
*返回值：1已连接 0未连接
*备  注：无
*****************************************************************************/
uint8_t NRF24L01_testConnection(void)
{
	uint8_t buf[5]={0XA5,0XA5,0XA5,0XA5,0XA5};
	uint8_t i; 	 
	NRF24L01_Write_Buf(W_REGISTER+TX_ADDR,buf,5); //写入5个字节的地址.	
	NRF24L01_Read_Buf(TX_ADDR,buf,5); //读出写入的地址  
	for(i=0;i<5;i++)
        if(buf[i]!=0XA5)break;	 							   
	if(i!=5)return 0; //检测24L01错误	
	return 1;	//检测到24L01
}	
u8 NRF24L01_Check(void)
{
   // char txt[16]="    ";
	while(!NRF24L01_testConnection())
	{
        //sprintf(txt,"NRF_FAIL");
        //LCD_P8x16Str(35,2,(unsigned char *)txt);
//		printf("\rNRF2401 no connect...\r\n");
		GPIO_Reverse(PTA, 17);   //LED指示程序运行状态
        delay_ms(100);
	}
    //sprintf(txt,"NRF_OK");
    //LCD_P8x16Str(35,2,(unsigned char *)txt);
        return 0;
}












/***********************************
以下为正点原子例程
启动NRF24L01发送一次数据
txbuf:待发送数据首地址
返回值:发送完成状况
***********************************/
u8 NRF24L01_TxPacket(u8 *txbuf)
{
	 NRF24l01_write_reg(W_REGISTER+CONFIG,0x0e);    //配置基本工作模式的参数;PWR_UP,EN_CRC,16BIT_CRC,发射模式,开启所有中断
	u8 sta;
 	//SPI1_SetSpeed(SPI_BaudRatePrescaler_16);//spi速度为10.5Mhz（24L01的最大SPI时钟为10Mhz）   
	NRF_CE_LOW;
  NRF24L01_Write_Buf(W_TX_PAYLOAD,txbuf,TX_PAYLO_WIDTH);//写数据到TX BUF  32个字节
 	NRF_CE_HIGH;//启动发送	   
	while(NRF24L01_IRQ!=0);//等待发送完成
	//delay_ms(1);
	sta=NRF24l01_read_reg(RF_STATUS);  //读取状态寄存器的值	   
	NRF24l01_write_reg(W_REGISTER+RF_STATUS,sta); //清除TX_DS或MAX_RT中断标志
       NRF24l01_write_reg(W_REGISTER+CONFIG,0x0f);    //配置基本工作模式的参数;PWR_UP,EN_CRC,16BIT_CRC,接收模式,开启所有中断
	if(sta&MAX_TX)//达到最大重发次数
	{
		NRF24l01_write_reg(FLUSH_TX,0xff);//清除TX FIFO寄存器 
                GPIO_Reverse(PTD, 15);
		return MAX_TX; 
	}
	if(sta&TX_OK)//发送完成
	{
          GPIO_Reverse(PTC, 0);
          return TX_OK;
	}
        GPIO_Reverse(PTA, 17);
	return sta;//其他原因发送失败
}







/**************************************
启动NRF24L01发送一次数据
txbuf:待发送数据首地址
返回值:0，接收完成；其他，错误代码
**************************************/
u8 NRF24L01_RxPacket(u8 *rxbuf)
{
	u8 sta;		    							   
	//SPI1_SetSpeed(SPI_BaudRatePrescaler_16); //spi速度为10.5Mhz（24L01的最大SPI时钟为10Mhz）   
	sta=NRF24l01_read_reg(RF_STATUS);  //读取状态寄存器的值    	 
	NRF24l01_write_reg(W_REGISTER+RF_STATUS,sta); //清除TX_DS或MAX_RT中断标志
	if(sta&RX_OK)//接收到数据
	{
		NRF24L01_Read_Buf(R_RX_PAYLOAD,rxbuf,RX_PAYLO_WIDTH);//读取数据
		NRF24l01_write_reg(FLUSH_RX,0xff);//清除RX FIFO寄存器 
                GPIO_Reverse(PTE, 26);
		return 0; 
	}	   
	return 1;//没收到任何数据
}





/****************************************************
该函数初始化NRF24L01到RX模式
设置RX地址,写RX数据宽度,选择RF频道,波特率和LNA HCURR
当CE变高后,即进入RX模式,并可以接收数据了
*****************************************************/		   
void NRF24L01_RX_Mode(void)
{
  NRF_CE_LOW;	  
  NRF24L01_Write_Buf(W_REGISTER+RX_ADDR_P0,(u8*)RX_ADDRESS,RX_ADR_WIDTH);//写RX节点地址
	
  NRF24l01_write_reg(W_REGISTER+EN_AA,0x01);    //使能通道0的自动应答    
  NRF24l01_write_reg(W_REGISTER+EN_RXADDR,0x01);//使能通道0的接收地址  
  NRF24l01_write_reg(W_REGISTER+SETUP_AW,0x03);	
  NRF24l01_write_reg(W_REGISTER+RF_CH,40);	     //设置RF通信频率		  
  NRF24l01_write_reg(W_REGISTER+RX_PW_P0,RX_PAYLO_WIDTH);//选择通道0的有效数据宽度 	    
  NRF24l01_write_reg(W_REGISTER+RF_SETUP,0x07);//设置TX发射参数,0db增益,2Mbps,低噪声增益开启   
  NRF24l01_write_reg(W_REGISTER+CONFIG, 0x0f);//配置基本工作模式的参数;PWR_UP,EN_CRC,16BIT_CRC,接收模式 
  NRF_CE_HIGH; //CE为高,进入接收模式 
}




/*****************************************************************************************
该函数初始化NRF24L01到TX模式
设置TX地址,写TX数据宽度,设置RX自动应答的地址,填充TX发送数据,选择RF频道,波特率和LNA HCURR
PWR_UP,CRC使能
当CE变高后,即进入RX模式,并可以接收数据了		   
CE为高大于10us,则启动发送.
*****************************************************************************************/	 
void NRF24L01_TX_Mode(void)
{														 
	NRF_CE_LOW;	    
  NRF24L01_Write_Buf(W_REGISTER+TX_ADDR,(u8*)TX_ADDRESS,TX_ADR_WIDTH);//写TX节点地址 
  NRF24L01_Write_Buf(W_REGISTER+RX_ADDR_P0,(u8*)RX_ADDRESS,RX_ADR_WIDTH); //设置TX节点地址,主要为了使能ACK	  

  NRF24l01_write_reg(W_REGISTER+EN_AA,0x01);     //使能通道0的自动应答    
  NRF24l01_write_reg(W_REGISTER+EN_RXADDR,0x01); //使能通道0的接收地址  
	NRF24l01_write_reg(W_REGISTER+SETUP_AW,0x03);			
  NRF24l01_write_reg(W_REGISTER+SETUP_RETR,0x1a);//设置自动重发间隔时间:500us + 86us;最大自动重发次数:10次
  NRF24l01_write_reg(W_REGISTER+RF_CH,40);       //设置RF通道为40
  NRF24l01_write_reg(W_REGISTER+RF_SETUP,0x07);  //设置TX发射参数,0db增益,2Mbps,低噪声增益开启   
  NRF24l01_write_reg(W_REGISTER+CONFIG,0x0e);    //配置基本工作模式的参数;PWR_UP,EN_CRC,16BIT_CRC,接收模式,开启所有中断
	NRF_CE_HIGH;//CE为高,10us后启动发送
}


/***************************
发送一串字符
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
2401接收
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
2401发送
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
