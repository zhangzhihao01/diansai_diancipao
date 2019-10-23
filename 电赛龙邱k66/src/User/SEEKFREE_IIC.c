/*********************************************************************************************************************
 * COPYRIGHT NOTICE
 * Copyright (c) 2017,逐飞科技
 * All rights reserved.
 * 技术讨论QQ群：179029047
 *
 * 以下所有内容版权均属逐飞科技所有，未经允许不得用于商业用途，
 * 欢迎各位使用并传播本程序，修改内容时必须保留逐飞科技的版权声明。
 *
 * @file       		SEEKFREE_IIC.c
 * @brief      		模拟IIC函数库
 * @company	   		成都逐飞科技有限公司
 * @author     		Go For It(1325536866)
 * @Software 		IAR 7.7 or MDK 5.17
 * @Target core		MK66FX
 * @Taobao   		https://seekfree.taobao.com/
 * @date       		2017-09-19
 * @note	
					模拟IIC接线定义
					------------------------------------ 
						SDA                 C17
						SCL                 C16
					------------------------------------ 
 ********************************************************************************************************************/




#include "include.h"
#include "SEEKFREE_IIC.h"




#define SCL_PORT  PTD    //SCL使用D端口     1.修改模拟IIC引脚时需要修改
#define SCL_INDEX 8      //SCL使用D8引脚
#define SDA_PORT  PTD    //SDA使用D端口     
#define SDA_INDEX 9      //SDA使用D9引脚



#define SDA             GPIO_Get(SEEKFREE_SDA)
#define SDA0()          GPIO_Ctrl(PTD,9,0)       		//IO口输出低电平
#define SDA1()          GPIO_Ctrl(PTD,9,1) 		       //IO口输出高电平  
#define SCL0()          GPIO_Ctrl(PTD,8,0) 		       //IO口输出低电平
#define SCL1()          GPIO_Ctrl(PTD,8,1) 		       //IO口输出高电平
#define DIR_OUT()       {GPIO_PDDR_REG(SDA_PORT) |= (1<<SDA_INDEX);}    //输出方向
#define DIR_IN()        {GPIO_PDDR_REG(SDA_PORT) &= ~(1<<SDA_INDEX);}	    //输入方向



//内部数据定义
uint8_t IIC_ad_main; //器件从地址	    
uint8_t IIC_ad_sub;  //器件子地址	   
uint8_t *IIC_buf;    //发送|接收数据缓冲区	    
uint8_t IIC_num;     //发送|接收数据个数	     

#define ack 1      //主应答
#define no_ack 0   //从应答	 


//内部使用，用户无需调用
void IIC_start(void)
{
        DIR_OUT();
	SDA1();
	SCL1();
	delay_nop(3);
	SDA0();
	delay_nop(3);
	SCL0();
}

//内部使用，用户无需调用
void IIC_stop(void)
{
        DIR_OUT();
	SDA0();
	SCL0();
	delay_nop(3);
	SCL1();
	delay_nop(3);
	SDA1();
	delay_nop(3);
}

//主应答(包含ack:SDA=0和no_ack:SDA=0)
//内部使用，用户无需调用
void I2C_SendACK(unsigned char ack_dat)
{
    SCL0();
	delay_nop(3);
	if(ack_dat) SDA0();
    else    	
    SDA1();
    SCL1();
    delay_nop(3);
    SCL0();
    delay_nop(3);
}


static int SCCB_WaitAck(void)
{
        SCL0();
	DIR_IN();
	delay_nop(3);
	SCL1();
        delay_nop(3);
	
    if(SDA)           //应答为高电平，异常，通信失败
    {
        DIR_OUT();
        SCL0();
        return 0;
    }
    DIR_OUT();
    SCL0();
	delay_nop(3);
    return 1;
}

//字节发送程序
//发送c(可以是数据也可是地址)，送完后接收从应答
//不考虑从应答位
//内部使用，用户无需调用
void send_ch(uint8 c)
{
	uint8 i = 8;
    while(i--)
    {
        if(c & 0x80)	SDA1();//SDA 输出数据
        else			SDA0();
        c <<= 1;
        delay_nop(3);
        SCL1();                //SCL 拉高，采集信号
        delay_nop(3);
        SCL0();                //SCL 时钟线拉低
    }
	SCCB_WaitAck();
}

//字节接收程序
//接收器件传来的数据，此程序应配合|主应答函数|使用
//内部使用，用户无需调用
uint8 read_ch1(uint8 ack_x)
{
    uint8 i;
    uint8 c;
    c=0;
    SCL0();
    delay_nop(3);
    SDA1();             
    DIR_IN();           //置数据线为输入方式
    for(i=0;i<8;i++)
    {
        delay_nop(3);
        SCL0();         //置时钟线为低，准备接收数据位
        delay_nop(3);
        SCL1();         //置时钟线为高，使数据线上数据有效
        delay_nop(3);
        c<<=1;
        if(SDA) 
        {
            c+=1;   //读数据位，将接收的数据存c
        }
    }
    DIR_OUT();
	SCL0();
	delay_nop(3);
	I2C_SendACK(ack_x);
	
    return c;
}

//字节接收程序
//接收器件传来的数据，此程序应配合|主应答函数|IIC_ack_main()使用
//内部使用，用户无需调用
uint8 read_ch(void)
{
    uint8 i;
    uint8 c;
    c=0;
    SCL0();
    delay_nop(3);
    SDA1();             //置数据线为输入方式
    DIR_IN();
    for(i=0;i<8;i++)
    {
        delay_nop(3);
        SCL0();         //置时钟线为低，准备接收数据位
        delay_nop(3);
        SCL1();         //置时钟线为高，使数据线上数据有效
        delay_nop(3);
        c<<=1;
        if(SDA) c+=1;   //读数据位，将接收的数据存c
    }
    DIR_OUT();
	SCL0();
	delay_nop(3);
	I2C_SendACK(no_ack);
	
    return c;
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      模拟IIC写数据到设备寄存器函数
//  @param      dev_add			设备地址(低七位地址)
//  @param      reg				寄存器地址
//  @param      dat				写入的数据
//  @return     void						
//  @since      v1.0
//  Sample usage:				
//-------------------------------------------------------------------------------------------------------------------
void simiic_write_reg(uint8 dev_add, uint8 reg, uint8 dat)
{
	IIC_start();
    send_ch( (dev_add<<1) | 0x00);   //发送器件地址加写位
	send_ch( reg );   				 //发送从机寄存器地址
	send_ch( dat );   				 //发送需要写入的数据
	IIC_stop();
}


//-------------------------------------------------------------------------------------------------------------------
//  @brief      模拟IIC从设备寄存器读取数据
//  @param      dev_add			设备地址(低七位地址)
//  @param      reg				寄存器地址
//  @param      type			选择通信方式是IIC  还是 SCCB
//  @return     uint8			返回寄存器的数据			
//  @since      v1.0
//  Sample usage:				
//-------------------------------------------------------------------------------------------------------------------
uint8 simiic_read_reg(uint8 dev_add, uint8 reg, IIC_type type)
{
	uint8 dat;
	IIC_start();
    send_ch( (dev_add<<1) | 0x00);  //发送器件地址加写位
	send_ch( reg );   				//发送从机寄存器地址
	if(type == SCCB)
         IIC_stop();
	IIC_start();
	send_ch( (dev_add<<1) | 0x01);  //发送器件地址加读位
	dat = read_ch();   				//发送需要写入的数据
	IIC_stop();
	
	return dat;
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      模拟IIC读取多字节数据
//  @param      dev_add			设备地址(低七位地址)
//  @param      reg				寄存器地址
//  @param      dat_add			数据保存的地址指针
//  @param      num				读取字节数量
//  @param      type			选择通信方式是IIC  还是 SCCB
//  @return     uint8			返回寄存器的数据			
//  @since      v1.0
//  Sample usage:				
//-------------------------------------------------------------------------------------------------------------------
void simiic_read_regs(uint8 dev_add, uint8 reg, uint8 *dat_add, uint8 num, IIC_type type)
{
	IIC_start();
    send_ch( (dev_add<<1) | 0x00);  //发送器件地址加写位
	send_ch( reg );   				//发送从机寄存器地址
	if(type == SCCB)IIC_stop();
	
	IIC_start();
	send_ch( (dev_add<<1) | 0x01);  //发送器件地址加读位
    while(--num)
    {
        *dat_add = read_ch1(ack); //读取数据
        dat_add++;
    }
    *dat_add = read_ch1(no_ack); //读取数据
	IIC_stop();
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      模拟IIC端口初始化
//  @param      NULL
//  @return     void	
//  @since      v1.0
//  Sample usage:				
//-------------------------------------------------------------------------------------------------------------------
void IIC_init(void)
{
    GPIO_Init(PTD,8,GPO,1);
    GPIO_Init(PTD,9,GPO,1);
	port_init_NoAlt (SEEKFREE_SCL, ODO | PULLUP);//ODO
	port_init_NoAlt (SEEKFREE_SDA, ODO | PULLUP);
}

#define EX_REF_CLK  50 //(定义外部参考时钟为50MHZ)
uint32 mcgout_clk_mhz = 100;
uint32 core_clk_mhz = 100;
uint32 bus_clk_mhz = 100;



void get_clk(void)
{
    mcgout_clk_mhz = EX_REF_CLK * ((MCG->C6 & MCG_C6_VDIV0_MASK) + 16) / ((MCG->C5 & MCG_C5_PRDIV0_MASK) + 1)/2;
    core_clk_mhz = mcgout_clk_mhz / ((SIM->CLKDIV1 >> SIM_CLKDIV1_OUTDIV1_SHIFT) + 1);
    bus_clk_mhz = mcgout_clk_mhz / (((SIM->CLKDIV1 >> SIM_CLKDIV1_OUTDIV2_SHIFT) & 0x0f) + 1);
    
    //uart_init(DEBUG_PORT,DEBUG_BAUD);   //初始化调试串口，如果不使用printf可以屏蔽
}
