#ifndef _OLED_H_
#define _OLED_H_			  	 

    	
#define OLED_MODE 0
#define SIZE 8
#define XLevelL		0x00
#define XLevelH		0x10
#define Max_Column	128
#define Max_Row		64
#define	Brightness	0xFF 
#define X_WIDTH 	128
#define Y_WIDTH 	64	    						  
//-----------------OLED IIC端口定义----------------  					   



#define OLED_SCLK_Clr() GPIO_Ctrl(PTB,4,0)//SCL IIC接口的时钟信号
#define OLED_SCLK_Set() GPIO_Ctrl(PTB,4,1)

#define OLED_SDIN_Clr() GPIO_Ctrl(PTB,0,0)//SDA IIC接口的数据信号
#define OLED_SDIN_Set() GPIO_Ctrl(PTB,0,1)

 		     
#define OLED_CMD  0	//写命令
#define OLED_DATA 1	//写数据 






void LCD_PrintFloat(unsigned char x,unsigned char y,float num);
void LCD_PrintFloat_abs(unsigned char x,unsigned char y,float num);








//OLED控制用函数
void OLED_WR_Byte(unsigned Dat,unsigned Cmd);  
void OLED_Display_On(void);
void OLED_Display_Off(void);	   							   		    
void OLED_Init(void);
void OLED_Clear(void);
void OLED_DrawPoint(uint8_t x,uint8_t y,uint8_t t);
void OLED_Fill(uint8_t x1,uint8_t y1,uint8_t x2,uint8_t y2,uint8_t Dot);
void OLED_ShowChar(uint8_t x,uint8_t y,uint8_t Chr,uint8_t Char_Size);
void OLED_ShowNum(uint8_t x,uint8_t y,uint32_t Num,uint8_t Len,uint8_t Size);
void OLED_ShowString(uint8_t x,uint8_t y, uint8_t *p,uint8_t Char_Size);	 
void OLED_Set_Pos(unsigned char x, unsigned char y);
void OLED_ShowCHinese(uint8_t x,uint8_t y,uint8_t No);
void OLED_DrawBMP(unsigned char x0, unsigned char y0,unsigned char x1, unsigned char y1,unsigned char BMP[]);
void Delay_50ms(unsigned int Del_50ms);
void Delay_1ms(unsigned int Del_1ms);
void Fill_picture(unsigned char Fill_Data);
void Picture(void);
void ii_IIC_Start(void);
void ii_IIC_Stop(void);
void Write_IIC_Command(unsigned char IIC_Command);
void Write_IIC_Data(unsigned char IIC_Data);
void Write_IIC_Byte(unsigned char IIC_Byte);
void ii_IIC_Wait_Ack(void);
#endif  

