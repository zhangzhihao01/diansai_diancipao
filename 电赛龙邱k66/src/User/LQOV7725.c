/*******************************************************************************
��ƽ    ̨������K66FX���ܳ�VDĸ��
����    д��CHIUSIR
��E-mail  ��chiusir@163.com
������汾��V1.0
�������¡�2018��4��28��
�������Ϣ�ο����е�ַ��
����    վ��http://www.lqist.cn
���Ա����̡�http://shop36265907.taobao.com
------------------------------------------------
��dev.env.��IAR7.80.4������
��Target  ��K66FX1M0VLQ18
��Crystal �� 50.000Mhz
��busclock��100.000MHz
��pllclock��200.000MHz
20180424�����µ�DMA������ʽ
******************************************************************************/
#include "include.h"

u8 Image_Data[IMAGEH][IMAGEW];  //ͼ��ԭʼ���ݴ��

u8 Image_Use[LCDH][LCDW]; //ѹ����֮�����ڴ����Ļ��ʾ����
u8 Pixle[LCDH][LCDW];              //��ֵ��������OLED��ʾ������
uint8_t Threshold;                  //OSTU��򷨼����ͼ����ֵ
u8  Line_Cont=0;          //�м���
u8  Field_Over_Flag=0;    //����ʶ
//u8 Left[500][2];
//u8 Right[500][2];
//u8 Middle[500][2];
int error[8]={0,0,0,0,0,0,0,0};     //���鴢��error����ʷ

int OFFSET0=0;      //��Զ������������ֵ�ۺ�ƫ����
int OFFSET1=0;      //�ڶ���
int OFFSET2=0;      //�����������
int TXV=0;          //���ε���߶ȣ��Ҹ߶�
int lt,rt;          //��ȡ��Ե������ĳ���
int p,d;            //PD����

//����ͷͼ��ɼ��жϴ�����
void PORTD_Interrupt(void)
{     
  //���ж�PTD14
  if((PORTD_ISFR & 0x4000))//���ж� (1<<14)
  {    
    PORTD_ISFR |= 0x4000;  //����жϱ�ʶ
    // �û�����            
    DMATransDataStart(DMA_CH4,(uint32_t)(&Image_Data[Line_Cont][0]));   //����DMA���� 
    if(Line_Cont > IMAGEH)  //�ɼ�����
    { 
      Line_Cont=0; 
      return ;
    } 
    ++Line_Cont;            //�м���
    return ; 
  }
  //���ж�PTD15
  if((PORTD_ISFR & 0x8000))//(1<<15)
  {
    PORTD_ISFR |= 0x8000; //����жϱ�ʶ
    // �û����� 
    Line_Cont = 0;         //�м�������
    Field_Over_Flag=1;     //��������ʶ
  } 
}

/*************************************************************************
*                    �����������ܿƼ� 
*
*  �������ƣ�void SendPicture()
*  ����˵��������ͷ���ݷ���
*  ����˵������
*  �������أ���
*  �޸�ʱ�䣺
*  ��    ע��
*************************************************************************/
void SendPicture(void)
{
  int i = 0, j = 0;
  UART_Put_Char(UART_4,0xff);//����֡ͷ��־
  for(i=(IMAGEH-1);i>=0;i--)      //���
  {
    for(j=(IMAGEW-1);j>=0;j--)    //����ӵ�0�е��У��û�����ѡ���Ե�������ʵ�����
    {
      if(Image_Data[i][j]==0xff)
      {
        Image_Data[i][j]=0xfe;//��ֹ���ͱ�־λ
      }
      UART_Put_Char(UART_4,Image_Data[i][j]);
    }
  }
}
/****************��ʾͼƬ���ӵ�Ƭ����DATA����********************************/	
                     /*��ʼλ��x        y    ͼƬ���      �߶�   ͼƬ������*/
void TFTSPI_Show_Pic3(uint8_t xs,uint8_t ys,uint8_t w,uint8_t h,uint8_t ppic[LCDH][LCDW]) 
{
    unsigned int i,j;	
    TFTSPI_Set_Pos(xs,ys,xs+w-1,ys+h);
    for(i=0;i<h;i+=1)  //
    { 	
      for(j=0; j<w; j+=1) //
      {
       
        if((i==endline)||(i==startline))//����ȡ���߶α����
        {
          TFTSPI_Write_Word(0x8000); 
        }
        else if(ppic[i][j]==1)
        {
            TFTSPI_Write_Word(0xffff); //��λ��ǰ��������������ϳ�һ��16λ���ݱ�ʾ����ֵ  
//        TFTSPI_Draw_Dot(j/2, i/2+20, (ppic[i][2*j]<<8)+(ppic[i][2*j+1]));
        }
//        else if(ppic[i][j]==2)
//        {
//          TFTSPI_Write_Word(0x8000);
//        }
        else
        {
            TFTSPI_Write_Word(0x0000); //��λ��ǰ��������������ϳ�һ��16λ���ݱ�ʾ����ֵ 
        }
        
      }
        
    }
 }
//����������
void Test_OV7725(void)
{  
  float duty;
  
  
  OV7725_Init();        //����ͷ��ʼ��
//  TFTSPI_CLS(u16WHITE);
EXTI_Init(PTC, 17, rising_down);
  while(1)
  { 
    LED_Ctrl(LED1, RVS);   //LEDָʾ��������״̬
    if(Field_Over_Flag)    //���һ��ͼ��ɼ�����ʾ���������ݵ���λ��
    {
//quickAdaptiveThreshold(Image_Data);
    // SendPicture();     //�������ݵ���λ����ע��Э���ʽ����ͬ����λ����ԭ������Ӧ�޸�
//      UARTSendPicture2(Image_Data);
 // time_delay_ms(5000);
    Get_Use_Image();     //�ɼ�ͼ�����ݴ������
    Get_01_Value();      //��ֵ��ͼ������
                  
    //Threshold = GetOSTU(Image_Data);   //OSTU��� ��ȡȫ����ֵ
     //BinaryImage(Image_Data,Threshold); //��ֵ��ͼ������
   // my_seek();
  //  TFTSPI_Show_Pic3(0, 8, 160, 120, Pixle );   // ����TFT18����ʾ��̬��ֵ��ͼ��

 duty=PDcalculate();
 Step_Angle_Set(duty);
//
//     
      Field_Over_Flag= 0;       
    }    
  }
}

// OV7725_Init Port Init
void OV7725_Init(void)
{     
    uint8_t id[2];    //�������ͷid
    uint8_t ack = 0;  //�������мĴ��������Ƿ����   
    uint16_t width = IMAGEW, height = IMAGEH;
    uint16_t hstart, vstart, hsize;  
  //����ͷ�Ĵ�������
    SCCB_Init();                     //������ַ�ڶ����ߣ�
    OV7725_SoftwareReset();   //�Ĵ����ָ���ʼֵ
    time_delay_ms(500);  
      /*7725���ֱ��� 640 * 480*/
    if ((IMAGEW > 640) || (IMAGEH > 480))
    {
        TFTSPI_P8X8Str(50,0,(u8*)"7725 dpi!!", u16RED, u16BLUE);                      //����ͷʶ�ֱ������ù���ֹͣ����
        while(1);
    }
    ack += SCCB_RegRead(OV7725_SCCB_ADDR,OV7725_PID_REG,&id[0]);  //��ȡ����ͷid�߰�λ
    ack += SCCB_RegRead(OV7725_SCCB_ADDR,OV7725_VER_REG,&id[1]);  //��ȡ����ͷid�Ͱ�λ
    if(OV7725_REVISION != (((uint32_t)id[0] << 8U) | (uint32_t)id[1]))//����ͷ�汾�Ĵ��� 
    {     
        
        TFTSPI_P8X8Str(50,0,(u8*)"OV7725 Failed", u16RED, u16BLUE);                      //����ͷʶ��ʧ�ܣ�ֹͣ����
        while(1); 
    } 
    else                                                   //оƬID��ȷ
    {
        TFTSPI_P8X8Str(50,0,(u8*)"OV7725 OK", u16RED, u16BLUE);
    }
    ack += OV7725_Init_Regs();     //�Ȱ��ٷ���Ĭ�ϳ�ʼ�� VGA ������Ҫ�ٸ�
    
    ack += SCCB_RegWrite(OV7725_SCCB_ADDR,OV7725_COM2_REG, 0x03);  //����������� Bit[4]: ˯��ģʽ Bit[1:0]: ��������00: 1x 01: 2x 10: 3x 11: 4x
//    ack += SCCB_RegWrite(OV7725_SCCB_ADDR,OV7725_COM7_REG, 0x46);  //  ʹ��QVGA �� RGB565 ��ʽ
//                                                                  /*OV7725_COM7_REG
//                                                                    Bit[7]: ���üĴ���
//                                                                    0: ������
//                                                                    1: �����мĴ�������ΪĬ��ֵ
//                                                                    Bit[6]: �ֱ������ã�7725���֧�����ֱַ��ʣ������������Զ���ֱ����൱�ڽ�ȡ�����ֱַ������е�һ���֣��ᶪʧ��Ұ��
//                                                                    0: VGA
//                                                                    1: QVGA
//                                                                    Bit[5]: BT.656Э�鿪/��ѡ��
//                                                                    Bit[4]: ��������ԭʼֵ
//                                                                    Bit[3:2]: RGB�����ʽ�ؼ�
//                                                                    00: GBR4:2:2
//                                                                    01: RGB565
//                                                                    10: RGB555
//                                                                    11: RGB444
//                                                                    Bit[1:0]: �����ʽ����
//                                                                    00: YUV
//                                                                    01: Processed Bayer RAW
//                                                                    10: RGB
//                                                                    11: Bayer RAW */
//    ack += SCCB_RegWrite(OV7725_SCCB_ADDR,OV7725_COM3_REG, 0x00);//Bit[6]: 0ˮƽ����� 1ˮƽ����  ע�⿪��ˮƽ����ʱ��ÿ���0x32�Ĵ�����Bit[7]:����ͼ���Ե����-�ھ���ģʽ��Ӧ������Ϊ1
//                                                                 //Bit[4]: 0 UYVYģʽ  1 YUYVģʽ  
//                                                                 //Bit[3]: 0 С��      1 ��� 
//    hstart = (0x3fU << 2U);   //ͼ��ˮƽ��ʼλ�� ʹ��VGAʱ Ϊ0x23
//    vstart = (0x03U  << 1U) ;   //ͼ��ֱ��ʼλ�� ʹ��VGAʱ Ϊ0x07   
    
    ack += SCCB_RegWrite(OV7725_SCCB_ADDR,OV7725_COM7_REG, 0x43);  //  ʹ��QVGA �� Bayer RAW ��ʽ
                                                                  /*OV7725_COM7_REG
                                                                    Bit[7]: ���üĴ���
                                                                    0: ������
                                                                    1: �����мĴ�������ΪĬ��ֵ
                                                                    Bit[6]: �ֱ������ã�7725���֧�����ֱַ��ʣ������������Զ���ֱ����൱�ڽ�ȡ�����ֱַ������е�һ���֣��ᶪʧ��Ұ��
                                                                    0: VGA
                                                                    1: QVGA
                                                                    Bit[5]: BT.656Э�鿪/��ѡ��
                                                                    Bit[4]: ��������ԭʼֵ
                                                                    Bit[3:2]: RGB�����ʽ�ؼ�
                                                                    00: GBR4:2:2
                                                                    01: RGB565
                                                                    10: RGB555
                                                                    11: RGB444
                                                                    Bit[1:0]: �����ʽ����
                                                                    00: YUV
                                                                    01: Processed Bayer RAW
                                                                    10: RGB
                                                                    11: Bayer RAW */
    ack += SCCB_RegWrite(OV7725_SCCB_ADDR,OV7725_DSP_CTRL4_REG, 0x4a);  /*Bit[1:0]: Output selection
                                                                          00: YUV or RGB
                                                                          01: YUV or RGB
                                                                          10: RAW8
                                                                          11: RAW10*/
    ack += SCCB_RegWrite(OV7725_SCCB_ADDR,OV7725_COM3_REG, 0x00);//Bit[6]: 0ˮƽ����� 1ˮƽ����  ע�⿪��ˮƽ����ʱ��ÿ���0x32�Ĵ�����Bit[7]:����ͼ���Ե����-�ھ���ģʽ��Ӧ������Ϊ1
                                                                 //Bit[4]: 0 UYVYģʽ  1 YUYVģʽ  
                                                                 //Bit[3]: ��С��
    ack += SCCB_RegWrite(OV7725_SCCB_ADDR,OV7725_COM8_REG, 0xff);/*Bit[7]: ���ÿ���AGC/AEC�㷨
                                                                    Bit[6]: AEC -��������
                                                                    0: �������ڴ�ֱ�հ�
                                                                    1: ���޵Ĳ���
                                                                    Bit[5]: �����˿�/��
                                                                    Bit[4]: ���õ�������ֵ��AEC
                                                                    0: ���ع�ʱ��������1/100��1/120
                                                                    ������κι��������µ����ô�ͨ�˲���
                                                                    1: �����ع�ʱ��С��1/100��1/120
                                                                    �����ǿ�������µ����ô�ͨ�˲���
                                                                    Bit[3]: ���õ�AEC��/�ؿ���
                                                                    0: ������С�ع�ʱ��Ϊһ��
                                                                    1: �����ع�ʱ��С��һ��
                                                                    Bit[2]: �Զ��������ʹ
                                                                    0: �ֶ�ģʽ
                                                                    1: �Զ�ģʽ
                                                                    Bit[1]: AWB Enable
                                                                    0: �ֶ�ģʽ
                                                                    1: �Զ�ģʽ
                                                                    Bit[0]: AEC Enable
                                                                    0: �ֶ�ģʽ
                                                                    1: �Զ�ģʽ*/
    ack += SCCB_RegWrite(OV7725_SCCB_ADDR,OV7725_CNST_REG, 0x20);/*�Աȶ����� ��һ��ֵΪ0x20  ԽС�Աȶ�Խ��*/
//    ack += SCCB_RegWrite(OV7725_SCCB_ADDR,OV7725_BRIGHT_REG, 0x20);/*��������*/
//    ack += SCCB_RegWrite(OV7725_SCCB_ADDR,OV7725_SIGN_REG, 0x20);/*��������*/
    hstart = 0x3fU << 2U;   //ͼ��ˮƽ��ʼλ��
    vstart = 0x03U << 1U;   //ͼ��ֱ��ʼλ��

    ack += SCCB_RegWrite(OV7725_SCCB_ADDR,OV7725_COM4_REG, 0x41);/* ���໷4��Ƶ ������
                                                                    Bit[7:6]: ���໷��Ƶ������
                                                                    00: ���ñ�Ƶ
                                                                    01: 4��Ƶ
                                                                    10: 6��Ƶ
                                                                    11: 8��Ƶ
                                                                    Bit[5:4]: �Զ��عⴰ�ڴ�С
                                                                    00: Full window
                                                                    01: 1/2 window
                                                                    10: 1/4 window
                                                                    11: Low 2/3 window*/
    ack += SCCB_RegWrite(OV7725_SCCB_ADDR,OV7725_CLKRC_REG, 0x07);/*ʱ������ ʱ�� = 24M * 4 / ����7+1��*2��= 6M  ʱ��Խ�ߣ�֡��Խ�� ����DMA���ܽ��ܲ��˻������ 
                                                                    Bit[6]: ֱ��ʹ���ⲿʱ��(û��ʱ��Ԥ�̶ȿ���)
                                                                    Bit[5:0]: ʱ�� = 24M �� ���໷��Ƶ�� /[(CLKRC[5:0] + 1) �� 2]*/
    ack += SCCB_RegWrite(OV7725_SCCB_ADDR,OV7725_DM_LNL_REG, 0x00);/*�����еͰ�λ�� ���������п��Խ���֡�ʣ��ʵ���Ӱ�֡�����õ��Լ���Ҫ��֡��*/
    ack += SCCB_RegWrite(OV7725_SCCB_ADDR, OV7725_DM_LNH_REG, 0x00); /*�����и߰�λ*/
    ack += SCCB_RegWrite(OV7725_SCCB_ADDR, OV7725_EXHCL_REG, 0x00); //�������ز���LSB������ˮƽ��������������ص�LSB
    ack += SCCB_RegWrite(OV7725_SCCB_ADDR, OV7725_ADVFL_REG, 0x00); //��ֱͬ������������(1λ����1��)��LSB
    ack += SCCB_RegWrite(OV7725_SCCB_ADDR, OV7725_ADVFH_REG, 0x00); //��ֱͬ�����������е�MSB 
    /* Resolution and timing. */
    hsize = width + 16;

    /* �������ͼƬ��С. */
    SCCB_RegWrite(OV7725_SCCB_ADDR,OV7725_HSTART_REG, hstart >> 2U);  //ˮƽ��ʼλ�ø�λ  ��Ϊ�Ĵ�����8λ�� ���255���������640 * 480 �Ų��£�����10λ���ݣ������Ǹ�8λ��ʣ�µ���λ����OV7725_HREF_REG��
    SCCB_RegWrite(OV7725_SCCB_ADDR,OV7725_HSIZE_REG, hsize >> 2U);    //ˮƽ��ȸ�λ      ��Ϊ�Ĵ�����8λ�� ���255���������640 * 480 �Ų��£�����10λ���ݣ������Ǹ�8λ��ʣ�µ���λ����OV7725_HREF_REG��
    SCCB_RegWrite(OV7725_SCCB_ADDR,OV7725_VSTART_REG, vstart >> 1U);  //��ֱ��ʼλ�ø�λ  ��Ϊ�Ĵ�����8λ�� ���255���������640 * 480 �Ų��£�����9λ���ݣ������Ǹ�8λ��ʣ�µ���λ����OV7725_HREF_REG��
    SCCB_RegWrite(OV7725_SCCB_ADDR,OV7725_VSIZE_REG, height >> 1U);   //��ֱ�߶ȸ�λ      ��Ϊ�Ĵ�����8λ�� ���255���������640 * 480 �Ų��£�����9λ���ݣ������Ǹ�8λ��ʣ�µ���λ����OV7725_HREF_REG��
    SCCB_RegWrite(OV7725_SCCB_ADDR,OV7725_HOUTSIZE_REG, width >> 2U); //ˮƽ�����ȸ�λ  ��Ϊ�Ĵ�����8λ�� ���255���������640 * 480 �Ų��£�����10λ���ݣ������Ǹ�8λ��ʣ�µ���λ����OV7725_EXHCH_REG��
    SCCB_RegWrite(OV7725_SCCB_ADDR,OV7725_VOUTSIZE_REG, height >> 1U);//��ֱ����߶ȸ�λ  ��Ϊ�Ĵ�����8λ�� ���255���������640 * 480 �Ų��£�����9λ���ݣ������Ǹ�8λ��ʣ�µ���λ����OV7725_EXHCH_REG��
    SCCB_RegWrite(OV7725_SCCB_ADDR,OV7725_HREF_REG,
                    ((vstart & 1U) << 6U) | ((hstart & 3U) << 4U) | ((height & 1U) << 2U) | ((hsize & 3U) << 0U)); //ˮƽ��Ⱥʹ�ֱ�߶ȵĵ�λ
    SCCB_RegWrite(OV7725_SCCB_ADDR,OV7725_EXHCH_REG, ((height & 1U) << 2U) | ((width & 3U) << 0U));  //ˮƽ����ʹ�ֱ����ĵ�2λ�͵�1λ
    
    //GPIO�ڳ�ʼ��
    EXTI_Init(PTD,14,rising_down);   //���ж�
    EXTI_Init(PTD,15,falling_up);    //���ж�  
    GPIO_Init(PTD,0,GPI,0);          //��λ���������      
    GPIO_Init(PTD,1,GPI,0);
    GPIO_Init(PTD,2,GPI,0);
    GPIO_Init(PTD,3,GPI,0);
    GPIO_Init(PTD,4,GPI,0);
    GPIO_Init(PTD,5,GPI,0);
    GPIO_Init(PTD,6,GPI,0);
    GPIO_Init(PTD,7,GPI,0);
    //��ʼ��DMA�ɼ�  
//    DMA_PORTx2BUFF_Init (DMA_CH4, (void *)&PTD_BYTE0_IN,(void*)Image_Data, PTD13, DMA_BYTE1, (IMAGEW * 2 > 511 ? 511:IMAGEW * 2), DMA_rising); //һ��DMA�������512���ֽ� 
    DMA_PORTx2BUFF_Init (DMA_CH4, (void *)&PTD_BYTE0_IN,(void*)Image_Data, PTD13, DMA_BYTE1, (IMAGEW ), DMA_rising); //һ��DMA�������512���ֽ�  
}


// ��ȡ��Ҫ��ͼ������
__ramfunc void Get_Use_Image(void)
{
  int i = 0,j = 0,row = 0,line = 0;
  
  for(i = 0; i  < IMAGEH; i+=2)  //240�У�ÿ2�вɼ�һ�У�
  {
    for(j = 0;j < IMAGEW; j+=2)  //320��ÿ2�вɼ�һ�У�
    {        
      Image_Use[row][line] = Image_Data[i][j];         
      line++;        
    }      
    line = 0;
    row++;      
  }  
}

//���վ�ֵ�ı������ж�ֵ��
void Get_01_Value(void)
{
  int i = 0,j = 0;
  u8 GaveValue;
  u32 tv=0;
  char txt[16];
  
  
  //�ۼ�
  for(i = 0; i <LCDH; i++)
  {    
    for(j = 0; j <LCDW; j++)
    {                            
      tv+=Image_Use[i][j];   //�ۼ�  
    } 
  }
  GaveValue=tv/LCDH/LCDW;     //��ƽ��ֵ,����Խ��ԽС��ȫ��Լ35��������ĻԼ160��һ������´�Լ100 

  //���վ�ֵ�ı������ж�ֵ��
  GaveValue=GaveValue*7/10+10;        //�˴���ֵ���ã����ݻ����Ĺ������趨 
  for(i = 0; i < LCDH; i++)
  {
    for(j = 0; j < LCDW; j++)
    {                                
      if(Image_Use[i][j] >GaveValue)//ƽ��ֵ��ֵ
  // if(Image_Use[i][j] >Threshold) //�����ֵ   ��ֵԽ����ʾ������Խ�࣬��ǳ��ͼ��Ҳ����ʾ����    
        Pixle[i][j] =1;        
      else                                        
        Pixle[i][j] =0;
    }    
  }
  
  p=geterror();
  d=getderror();
  
  
  if(p>0)//����λ��ƫ����1��0
  {
    sprintf(txt,"%03d:%03d",p,d); 
  TFTSPI_P8X8Str(0,0,(u8*)txt,u16RED,u16BLUE);
  }
  else//ƫ��
  {
  sprintf(txt,"%03d:%03d",p,d); 
  TFTSPI_P8X8Str(0,0,(u8*)txt,u16RED,u16BLUE);
  
  }
}

void Step_Angle_Set(float duty)//duty�ǹ�һ����ֵ�����������Ǹ������Ҵ��
{
  int angle;
  //�����Ƕ���һ����Χ�ڣ����ɳ�Խ�����������������յ�           
  if(duty<-0.99) duty=-0.99;
  else if(duty>0.99) duty=0.99; 
  if(duty>0)
  {
    angle=(int)(Step_Middle-duty*(Step_Middle-Step_Left));
  }
  else
  {
    angle=(int)(Step_Middle-duty*(Step_Right-Step_Middle));
  }
  Servo_FTM_PWM_Init(FTM3,FTM_CH7,bus_clk_M*1000000/64/50,angle);
}

int geterror(void)//����(��ƫ��Ϊ��ƫ��Ϊ��
{
  int perror;
  u8 i,j;
  for(i=startline;i<endline;i++)
  {
    for(j=0;j<80;j++)
    {
    if(Pixle[i][j])
    {
      perror++;
    }
    }
        for(j=80;j<160;j++)
    {
    if(Pixle[i][j])
    {
      perror--;
    }
    }
  }
  for(i=0;i<7;i++)
  {
    error[i]=error[1+i];
  }
   error[7]=perror;

   return perror;
}


 int getderror(void)
 {
   u8 i;
   int derror=0;
   for(i=0;i<7;i++)
   {
      derror+=error[i+1]-error[i];
     derror>>=1;
   }
   return derror;
 }


float PDcalculate(void)
{
  float result;
  
  //if((p>340)||(p<-340))
  result=kp2*p+kd*d;
 // else
  //result=kp1*p+kd*d;  
  
  
  return result;//��һ����ֵ
}



//��ʾͼ��OLEDģ��
void Draw_Road(void)
{ 	 
  u8 i = 0, j = 0,temp=0;
  
  //����֡ͷ��־
  for(i=8;i<56;i+=8)//6*8=48�� 
  {
    LCD_Set_Pos(18,i/8+1);//��ʼλ��
    for(j=0;j<LCDW;j++)  //����
    { 
      temp=0;
      if(Pixle[0+i][j]) temp|=1;
      if(Pixle[1+i][j]) temp|=2;
      if(Pixle[2+i][j]) temp|=4;
      if(Pixle[3+i][j]) temp|=8;
      if(Pixle[4+i][j]) temp|=0x10;
      if(Pixle[5+i][j]) temp|=0x20;
      if(Pixle[6+i][j]) temp|=0x40;
      if(Pixle[7+i][j]) temp|=0x80;
      LCD_WrDat(temp); 	  	  	  	  
    }
  }  
}
//�������Ϸ���Χ��������
void Pixle_Filter(void)
{  
  int nr; //��
  int nc; //��
  
  for(nr=1; nr<LCDH-1; nr++)
  {  	    
    for(nc=1; nc<LCDW-1; nc=nc+1)
    {
      if((Pixle[nr][nc]==0)&&(Pixle[nr-1][nc]+Pixle[nr+1][nc]+Pixle[nr][nc+1]+Pixle[nr][nc-1]>2))         
      {
        Pixle[nr][nc]=1;
      }	
      else if((Pixle[nr][nc]==1)&&(Pixle[nr-1][nc]+Pixle[nr+1][nc]+Pixle[nr][nc+1]+Pixle[nr][nc-1]<2))         
      {
        Pixle[nr][nc]=0;
      }	
    }	  
  }  
}

void quickAdaptiveThreshold(uint8_t tmImage[IMAGEH][IMAGEW])

{
	int t = 15; //����
	int s = 40; //ͼ����1/8
	const int T = 9; //T�Ǳ��⸡��������
	const int power2S = 1 << T; //ͬʱ�˳�2��T�η�,���⸡������
	int factor = power2S * (100-t) / (100*s); //ǰs�����ؾ�ֵ*power2s��������ȥpower2s
	int gn = 127 * s; //��һ��һ��ʼ�� ֵ������ʼ��Ϊ127*S
	int q = power2S - power2S / s; //�ȱ����б���
	int pn, hn;//pn�ǵڼ������ص�ĻҶ�ֵ��hn��ƽ���Ҷ�ֵ
	unsigned char *scanline = NULL;//��ʼ������Ϊ��
	int *prev_gn = NULL;//��ʼ������Ϊ��
	for (int i = 0; i < IMAGEW; i++) 
	{
		prev_gn[i] = gn;//Ȩֵ��ʼ��
	}
	//���ҽ���ɨ��������
	for (int y = 0; y < IMAGEH; y ++ )
	{	
		for ( int x = 0; x <IMAGEW; x ++ )    //���д�������ɨ��
		{
                        scanline[x] = tmImage[y][x];//ÿһ�����ص���и�ֵ
			pn = scanline[x];
			gn = ((gn * q) >> T) + pn; //ͬʱ�˳�2��T�η�,���⸡�����㣬pn����f��i��j��
			hn = (gn + prev_gn[x]) >> 1;     //����������ƽ�������ص㣬prev_gn[x]������һ�е�x�е����غͣ����Ǹ�����������ɨ��ģ�������prev_gn[x]��һǧǰs���غ�
			prev_gn[x] = gn; 		
			if(pn < (hn*factor) >> T)
                        {
                         tmImage[y][x] = 0; 
                        } 
                        else
                        {
                          tmImage[y][x] = 1;
                        }
                         
		}
		y ++ ;
		if ( y == IMAGEH)
		{
			break;
		}
		for ( int x = IMAGEW-1; x >= 0; x --)  //���д�������ɨ��
                {
                        scanline[x] = tmImage[y][x];
			pn = scanline[x] ;
			gn = ((gn * q) >> T) + pn; 
			hn = (gn + prev_gn[x]) >> 1;
			prev_gn[x] = gn; 		
			if(pn < (hn*factor) >> T)
                        {
                          tmImage[y][x] = 0;
                        } 
                        else
                        {
                          tmImage[y][x] = 1;
                        }
                         
		}
	}

	
}






//
//
//
//void my_seek(void)
//{
//  int hang=0;
//  int lie;
//  int l,r;
//  int lhang,rhang;
//  int tt;
//
//
///*���߾���ʶ����������(���߶��Ǻڵ� */
// 
//  while(Pixle[hang][0]==1||Pixle[hang][159]==1)
//  {
//  hang++;
//  }
///*�ҵ�һ�εı�Ե*/
//  for(r=0;r<LCDW;r++)
//  {
//  if(Pixle[hang][r]==1)
//  {
//  Right[0][0]=hang;
//  Right[0][1]=r;
//  break;
//  }
//  /*********�����ã����ܿ���ȥ��*/
// /* if(r==(LCDW-1))
//  {
//    hang++;
//    r=0;
//  }*/
//  }
//  
//  
//  
//  
//  /*
//  for(l=(LCDW-1);l>=0;l--)
//  {
//  if(Pixle[hang][l]==1)
//  {
//  Left[0][0]=hang;
//  Left[0][1]=l;
//  break;
//  }
//  }
//  */
//  
//  
//  
//  
//  
///******/
//  rhang=hang;
//  lhang=hang;
///******/
//
//  
//  
//  for(tt=1;tt<499;tt++)
//  {
//    if((tt>20)&&((r==0)||(r==159)||(rhang==119)))
//      break;//
//    else
//    {
//      
//            
//                    
//                  if(Pixle[rhang+1][r]==1)//��������
//                  {
//                    if(Pixle[rhang+1][r-1]==0)
//                    {
//                      Right[tt][0]=rhang+1;
//                      Right[tt][1]=r;
//                      rhang=rhang+1;
//                    }
//                    else
//                    {
//                      if(Pixle[rhang][r-1]==0)
//                      {
//                        Right[tt][0]=rhang+1;
//                        Right[tt][1]=r-1;   
//                        rhang++;
//                        r--;
//                      }
//                      else
//                      {
//                        if(Pixle[rhang-1][r-1]==0)
//                        {
//                          Right[tt][0]=rhang;
//                          Right[tt][1]=r-1;   
//                          r--;
//                        } 
//                        else
//                        {
//                          if(Pixle[rhang-1][r]==0)
//                          {
//                            Right[tt][0]=rhang-1;
//                            Right[tt][1]=r-1; 
//                            rhang--;
//                            r--;
//                           } 
//                          else
//                          {
//                            Right[tt][0]=rhang-1;
//                            Right[tt][1]=r; 
//                            rhang--;
//                          }
//                        }
//                      }
//                    }
//                  }
//                  else   //���м���
//                  {
//                    if(Pixle[rhang+1][r+1]==1)
//                    {
//                      Right[tt][0]=rhang+1;
//                      Right[tt][1]=r+1;
//                      rhang++;
//                      r++;
//                    }
//                    else
//                    {
//                      if(Pixle[rhang][r+1]==1)
//                      {
//                        Right[tt][0]=rhang;
//                        Right[tt][1]=r+1;   
//                        
//                        r++;
//                      }
//                      else
//                      {
//                        if(Pixle[rhang-1][r+1]==1)
//                        {
//                          Right[tt][0]=rhang-1;
//                          Right[tt][1]=r+1; 
//                          r++;
//                          rhang--;
//                        } 
//                        else
//                        {
//                          if(Pixle[rhang-1][r]==1)
//                          {
//                            Right[tt][0]=rhang-1;
//                            Right[tt][1]=r;
//                            rhang--;
//                            
//                           } 
//                          else
//                          {
//                           Right[tt][0]=rhang-1;
//                           Right[tt][1]=r-1;
//                           rhang--;
//                           r--;
//                          }
//                        }
//                      }
//                    }    
//                  }
//        tt++;
//    }
//  
//  }
//  
//  rt=tt;
//  for(tt=0;tt<rt;tt++)
//  {
//    Pixle[Right[tt][0]][Right[tt][1]]=2;
//  }
//
//
//
//
//
//  
//
//}
/***************************************************************************
*                                                                          *
*  �������ƣ�int Seek_Road(void)                                           *
*  ����˵����Ѱ�Ұ�ɫ����ƫ��ֵ                                            *
*  ����˵������                                                            *
*  �������أ�ֵ�Ĵ�С                                                      *
*  �޸�ʱ�䣺2017-07-16                                                    *
*  ��    ע�����м�Ϊ0������һ���Ҳ��һ����ֵ����1�����                *
*            ��������ӵ�һ�п�ʼ�������ڶ��н�����                        *
*            ������Ϊ��������ֵԽ��˵��Խƫ��ߣ�                        *
*            ������Ϊ��������ֵԽ��˵��Խƫ�ұߡ�                        *
***************************************************************************/ 
/*void Seek_Road(void)
{  
  int nr; //��
  int nc; //��
  int temp=0;//��ʱ��ֵ
  //for(nr=1; nr<MAX_ROW-1; nr++)
  temp=0;
  for(nr=8; nr<24; nr++)
  {  	    
    for(nc=MAX_COL/2;nc<MAX_COL;nc=nc+1)
    {
      if(Pixle[nr][nc])
      {
        ++temp;
      }			   
    }
    for(nc=0; nc<MAX_COL/2; nc=nc+1)
    {
      if(Pixle[nr][nc])
      {
        --temp;
      }			   
    }		  
  }
  OFFSET0=temp;
  temp=0;
  for(nr=24; nr<40; nr++)
  {  	    
    for(nc=MAX_COL/2;nc<MAX_COL;nc=nc+1)
    {
      if(Pixle[nr][nc])
      {
        ++temp;
      }			   
    }
    for(nc=0; nc<MAX_COL/2; nc=nc+1)
    {
      if(Pixle[nr][nc])
      {
        --temp;
      }			   
    }		  
  }
  OFFSET1=temp;    	
  temp=0;
  for(nr=40; nr<56; nr++)
  {  	    
    for(nc=MAX_COL/2;nc<MAX_COL;nc=nc+1)
    {
      if(Pixle[nr][nc])
      {
        ++temp;
      }			   
    }
    for(nc=0; nc<MAX_COL/2; nc=nc+1)
    {
      if(Pixle[nr][nc])
      {
        --temp;
      }			   
    }		  
  }
  OFFSET2=temp;   	
  return;  
}

u8 zb[48],yb[48];



void FindTiXing(void)
{
  int nr; //��
  int nc; //��     
  
  for(nr=0; nr<48; nr++)
  {  	    
    zb[nr]=0;
    yb[nr]=100;   
  }  	
  for(nr=0; nr<48; nr++)
  {  	    
    for(nc=2;nc<MAX_COL-2;nc++)
    {
      if((Pixle[nr+8][nc-1]==0)&&(Pixle[nr+8][nc]==0)&&(Pixle[nr+8][nc+1]==1)&&(Pixle[nr+8][nc+2]==1))
      {
        zb[nr]=nc;//����أ�Խ��Խ��
      }
      if((Pixle[nr+8][nc-1]==1)&&(Pixle[nr+8][nc]==1)&&(Pixle[nr+8][nc+1]==0)&&(Pixle[nr+8][nc+2]==0))
      {
        yb[nr]=nc;//�ұ��أ�Խ��ԽС
      }                   
    }	    
  }
  TXV=0;
  for(nr=0; nr<47; nr++)
  {  	    
    if((zb[nr]>=zb[nr+1])&&(zb[nr]>0))   TXV++;          
    if((yb[nr]<=yb[nr+1])&&(yb[nr]<100)) TXV--;          
  }  	   
  return;  
}

*/
/***************************************************************
* 
* �������ƣ�SendPicture 
* ����˵��������ͼ����λ�� ����ͬ����λ��ע���޸Ķ�Ӧ�����ݽ���Э��
* ����˵���� 
* �������أ�void 
* �޸�ʱ�䣺2018��3��27�� 
* �� ע�� 
***************************************************************/ 
/*void UARTSendPicture2(uint8_t  tmImage[IMAGEH][IMAGEW * 2]) 
{ 
  int i = 0, j = 0; 
  UART_Put_Char(UART_4,0x01); //����֡ͷ��־ WindowsFormsApplication1.exe
  UART_Put_Char(UART_4,0xfe); //����֡ͷ��־ WindowsFormsApplication1.exe
  for(i=0;i < IMAGEH; i++) 
  { 
    for(j=0;j <IMAGEW * 2;j++) 
    { 
      if(tmImage[i][j]==0xfe) 
      { 
        tmImage[i][j]=0xff; //��ֹ���ͱ�־λ 
      } 
      UART_Put_Char(UART_4,tmImage[i][j]); 
    } 
  }
  UART_Put_Char(UART_4,0xfe); //����֡β��־ 
  UART_Put_Char(UART_4,0x01); //����֡β��־ 
} 

void UARTSendPicture(uint8_t tmImage[IMAGEH][IMAGEW]) 
{ 
  int i = 0, j = 0; 
  UART_Put_Char(UART_4,0xFF); //����֡ͷ��־ DEMOK��λ��  
  for(i=0;i < IMAGEH; i++) 
  { 
    for(j=0;j <IMAGEW;j++) 
    { 
      if(tmImage[i][j]==0xFF) 
      { 
        tmImage[i][j]=0xFE; //��ֹ���ͱ�־λ 
      } 
      UART_Put_Char(UART_4,tmImage[i][j]); 
    } 
  }
} */
/*
void SendPicture(void)
{
  int i = 0, j = 0;
  UART_Put_Char(UART_4,0xff);//����֡ͷ��־
  for(i=0;i<Frame_Height;i++)      //���
  {
    for(j=0;j<Frame_Width;j++)    
    {
      if(Image_Data[i][j]==0xff)
      {
        Image_Data[i][j]=0xfe;//��ֹ���ͱ�־λ
      }
      UART_Put_Char(UART_4,Image_Data[i][j]);
    }
  }
}
*/

/*************************************************************** 
* 
* �������ƣ�uint8_t GetOSTU(uint8_t tmImage[IMAGEH][IMAGEW]) 
* ����˵��������ֵ��С 
* ����˵���� 
* �������أ���ֵ��С 
* �޸�ʱ�䣺2018��3��27�� 
* �� ע�� 
�ο���https://blog.csdn.net/zyzhangyue/article/details/45841255
      https://www.cnblogs.com/moon1992/p/5092726.html
      https://www.cnblogs.com/zhonghuasong/p/7250540.html     
Ostu������������������ͨ��ͳ������ͼ���ֱ��ͼ������ʵ��ȫ����ֵT���Զ�ѡȡ�����㷨����Ϊ��
1) �ȼ���ͼ���ֱ��ͼ������ͼ�����е����ص㰴��0~255��256��bin��ͳ������ÿ��bin�����ص�����
2) ��һ��ֱ��ͼ��Ҳ����ÿ��bin�����ص����������ܵ����ص�
3) i��ʾ�������ֵ��Ҳ��һ���Ҷȼ�����0��ʼ����
4) ͨ����һ����ֱ��ͼ��ͳ��0~i �Ҷȼ�������(��������ֵ�ڴ˷�Χ�����ؽ���ǰ������) ��ռ����ͼ��ı���w0����ͳ��ǰ�����ص�ƽ���Ҷ�u0��ͳ��i~255�Ҷȼ�������(��������ֵ�ڴ˷�Χ�����ؽ�����������) ��ռ����ͼ��ı���w1����ͳ�Ʊ������ص�ƽ���Ҷ�u1��
5) ����ǰ�����غͱ������صķ��� g = w0*w1*(u0-u1) (u0-u1)
6) i++��ת��4)��ֱ��iΪ256ʱ��������
7�������g��Ӧ��iֵ��Ϊͼ���ȫ����ֵ
ȱ��:OSTU�㷨�ڴ�����ղ����ȵ�ͼ���ʱ��Ч�������Բ��ã���Ϊ���õ���ȫ��������Ϣ��
***************************************************************/ 
uint8_t GetOSTU(uint8_t tmImage[IMAGEH][IMAGEW]) 
{ 
  int16_t i,j; 
  uint32_t Amount = 0; 
  uint32_t PixelBack = 0; 
  uint32_t PixelIntegralBack = 0; 
  uint32_t PixelIntegral = 0; 
  int32_t PixelIntegralFore = 0; 
  int32_t PixelFore = 0; 
  double OmegaBack, OmegaFore, MicroBack, MicroFore, SigmaB, Sigma; // ��䷽��; 
  int16_t MinValue, MaxValue; 
  uint8_t Threshold = 0;
  uint8_t HistoGram[256];              //  

  for (j = 0; j < 256; j++)  HistoGram[j] = 0; //��ʼ���Ҷ�ֱ��ͼ 
  
  for (j = 0; j < IMAGEH; j++) 
  { 
    for (i = 0; i < IMAGEW; i++) 
    { 
      HistoGram[tmImage[j][i]]++; //ͳ�ƻҶȼ���ÿ������������ͼ���еĸ���
    } 
  } 
  
  for (MinValue = 0; MinValue < 256 && HistoGram[MinValue] == 0; MinValue++) ;        //��ȡ��С�Ҷȵ�ֵ
  for (MaxValue = 255; MaxValue > MinValue && HistoGram[MinValue] == 0; MaxValue--) ; //��ȡ���Ҷȵ�ֵ
      
  if (MaxValue == MinValue)     return MaxValue;         // ͼ����ֻ��һ����ɫ    
  if (MinValue + 1 == MaxValue)  return MinValue;        // ͼ����ֻ�ж�����ɫ
    
  for (j = MinValue; j <= MaxValue; j++)    Amount += HistoGram[j];        //  ��������
    
  PixelIntegral = 0;
  for (j = MinValue; j <= MaxValue; j++)
  {
    PixelIntegral += HistoGram[j] * j;//�Ҷ�ֵ����
  }
  SigmaB = -1;
  for (j = MinValue; j < MaxValue; j++)
  {
    PixelBack = PixelBack + HistoGram[j];    //ǰ�����ص���
    PixelFore = Amount - PixelBack;         //�������ص���
    OmegaBack = (double)PixelBack / Amount;//ǰ�����ذٷֱ�
    OmegaFore = (double)PixelFore / Amount;//�������ذٷֱ�
    PixelIntegralBack += HistoGram[j] * j;  //ǰ���Ҷ�ֵ
    PixelIntegralFore = PixelIntegral - PixelIntegralBack;//�����Ҷ�ֵ
    MicroBack = (double)PixelIntegralBack / PixelBack;   //ǰ���ҶȰٷֱ�
    MicroFore = (double)PixelIntegralFore / PixelFore;   //�����ҶȰٷֱ�
    Sigma = OmegaBack * OmegaFore * (MicroBack - MicroFore) * (MicroBack - MicroFore);//������䷽��
    if (Sigma > SigmaB)                    //����������䷽��g //�ҳ������䷽���Լ���Ӧ����ֵ
    {
      SigmaB = Sigma;
      Threshold = j;
    }
  }
  return Threshold;                        //���������ֵ;
} 
/*************************************************************** 
* 
* �������ƣ�void BinaryImage(uint8_t tmImage[IMAGEH][IMAGEW]) 
* ����˵����ͼ�����ݶ�ֵ�� 
* ����˵���� 
* �������أ�void 
* �޸�ʱ�䣺2018��3��27�� 
* �� ע�� 
***************************************************************/ 
void BinaryImage(uint8_t tmImage[IMAGEH][IMAGEW],uint8_t ThresholdV) 
{ 
  int i = 0, j = 0; 
  for(i = 0;i < IMAGEH;i++) 
  { 
    for(j = 0; j< IMAGEW;j++) 
    { 
      if(tmImage[i][j] >= ThresholdV) 
      { 
        tmImage[i][j] = 0xFE; 
      } 
      else 
      { 
        tmImage[i][j] = 0X00; 
      } 
    } 
  } 
} 





/***************************************************************************
 *7725����ͷ����
 * 
 */
/*��ʼ��ov7725  �Զ��ع� û����Ч*/
uint8_t OV7725_Init_Regs(void)
{
    uint8_t ack = 0;
    ack += SCCB_RegWrite(OV7725_SCCB_ADDR,0x3d, 0x03);
    ack += SCCB_RegWrite(OV7725_SCCB_ADDR,0x42, 0x7f);
    ack += SCCB_RegWrite(OV7725_SCCB_ADDR,0x4d, 0x09);

    ack += SCCB_RegWrite(OV7725_SCCB_ADDR,0x64, 0xff);
    ack += SCCB_RegWrite(OV7725_SCCB_ADDR,0x65, 0x20);
    ack += SCCB_RegWrite(OV7725_SCCB_ADDR,0x66, 0x00);
    ack += SCCB_RegWrite(OV7725_SCCB_ADDR,0x67, 0x48);
    ack += SCCB_RegWrite(OV7725_SCCB_ADDR,0x0f, 0xc5);
    ack += SCCB_RegWrite(OV7725_SCCB_ADDR,0x13, 0xff);

    ack += SCCB_RegWrite(OV7725_SCCB_ADDR,0x63, 0xe0);
    ack += SCCB_RegWrite(OV7725_SCCB_ADDR,0x14, 0x11);
    ack += SCCB_RegWrite(OV7725_SCCB_ADDR,0x22, 0x3f);
    ack += SCCB_RegWrite(OV7725_SCCB_ADDR,0x23, 0x07);
    ack += SCCB_RegWrite(OV7725_SCCB_ADDR,0x24, 0x40);
    ack += SCCB_RegWrite(OV7725_SCCB_ADDR,0x25, 0x30);
    ack += SCCB_RegWrite(OV7725_SCCB_ADDR,0x26, 0xa1);
    ack += SCCB_RegWrite(OV7725_SCCB_ADDR,0x2b, 0x00);
    ack += SCCB_RegWrite(OV7725_SCCB_ADDR,0x6b, 0xaa);
    ack += SCCB_RegWrite(OV7725_SCCB_ADDR,0x0d, 0x41);

    ack += SCCB_RegWrite(OV7725_SCCB_ADDR,0x90, 0x05);
    ack += SCCB_RegWrite(OV7725_SCCB_ADDR,0x91, 0x01);
    ack += SCCB_RegWrite(OV7725_SCCB_ADDR,0x92, 0x03);
    ack += SCCB_RegWrite(OV7725_SCCB_ADDR,0x93, 0x00);

    ack += SCCB_RegWrite(OV7725_SCCB_ADDR,0x94, 0x90);
    ack += SCCB_RegWrite(OV7725_SCCB_ADDR,0x95, 0x8a);
    ack += SCCB_RegWrite(OV7725_SCCB_ADDR,0x96, 0x06);
    ack += SCCB_RegWrite(OV7725_SCCB_ADDR,0x97, 0x0b);
    ack += SCCB_RegWrite(OV7725_SCCB_ADDR,0x98, 0x95);
    ack += SCCB_RegWrite(OV7725_SCCB_ADDR,0x99, 0xa0);
    ack += SCCB_RegWrite(OV7725_SCCB_ADDR,0x9a, 0x1e);

    ack += SCCB_RegWrite(OV7725_SCCB_ADDR,0x9b, 0x08);

    ack += SCCB_RegWrite(OV7725_SCCB_ADDR,0x9c, 0x20);

    ack += SCCB_RegWrite(OV7725_SCCB_ADDR,0x9e, 0x81);

    ack += SCCB_RegWrite(OV7725_SCCB_ADDR,0xa6, 0x04);


    ack += SCCB_RegWrite(OV7725_SCCB_ADDR,0x7e, 0x0c);
    ack += SCCB_RegWrite(OV7725_SCCB_ADDR,0x7f, 0x16);
    ack += SCCB_RegWrite(OV7725_SCCB_ADDR,0x80, 0x2a);
    ack += SCCB_RegWrite(OV7725_SCCB_ADDR,0x81, 0x4e);
    ack += SCCB_RegWrite(OV7725_SCCB_ADDR,0x82, 0x61);
    ack += SCCB_RegWrite(OV7725_SCCB_ADDR,0x83, 0x6f);
    ack += SCCB_RegWrite(OV7725_SCCB_ADDR,0x84, 0x7b);
    ack += SCCB_RegWrite(OV7725_SCCB_ADDR,0x85, 0x86);
    ack += SCCB_RegWrite(OV7725_SCCB_ADDR,0x86, 0x8e);
    ack += SCCB_RegWrite(OV7725_SCCB_ADDR,0x87, 0x97);
    ack += SCCB_RegWrite(OV7725_SCCB_ADDR,0x88, 0xa4);
    ack += SCCB_RegWrite(OV7725_SCCB_ADDR,0x89, 0xaf);
    ack += SCCB_RegWrite(OV7725_SCCB_ADDR,0x8a, 0xc5);
    ack += SCCB_RegWrite(OV7725_SCCB_ADDR,0x8b, 0xd7);
    ack += SCCB_RegWrite(OV7725_SCCB_ADDR,0x8c, 0xe8);
    return ack;
}

/*���� 7725�ļĴ������ָ�Ĭ��ֵ*/
void OV7725_SoftwareReset(void)
{
    SCCB_RegWrite(OV7725_SCCB_ADDR,OV7725_COM7_REG, 0x80);
}

/*��ƽ������ 0:�Զ�ģʽ1:����2,����3,�칫��4,����5,ҹ��*/
void OV7725_LightModeConfigs(uint8_t mode)
{
    switch(mode)
    {
        case 1:  //����
            SCCB_RegWrite(OV7725_SCCB_ADDR,OV7725_COM8_REG, 0xfd);
            SCCB_RegWrite(OV7725_SCCB_ADDR,OV7725_BLUE_REG, 0x5a);
            SCCB_RegWrite(OV7725_SCCB_ADDR,OV7725_RED_REG, 0x5c);
            SCCB_RegWrite(OV7725_SCCB_ADDR,OV7725_COM5_REG, 0x65);
            SCCB_RegWrite(OV7725_SCCB_ADDR,OV7725_ADVFL_REG, 0x00);
            SCCB_RegWrite(OV7725_SCCB_ADDR,OV7725_ADVFH_REG, 0x00);
            break;
        case 2:  //����
            SCCB_RegWrite(OV7725_SCCB_ADDR,OV7725_COM8_REG, 0xfd);
            SCCB_RegWrite(OV7725_SCCB_ADDR,OV7725_BLUE_REG, 0x58);
            SCCB_RegWrite(OV7725_SCCB_ADDR,OV7725_RED_REG, 0x60);
            SCCB_RegWrite(OV7725_SCCB_ADDR,OV7725_COM5_REG, 0x65);
            SCCB_RegWrite(OV7725_SCCB_ADDR,OV7725_ADVFL_REG, 0x00);
            SCCB_RegWrite(OV7725_SCCB_ADDR,OV7725_ADVFH_REG, 0x00);
        
            break;
        case 3:  //�칫��
            SCCB_RegWrite(OV7725_SCCB_ADDR,OV7725_COM8_REG, 0xfd);
            SCCB_RegWrite(OV7725_SCCB_ADDR,OV7725_BLUE_REG, 0x84);
            SCCB_RegWrite(OV7725_SCCB_ADDR,OV7725_RED_REG, 0x4c);
            SCCB_RegWrite(OV7725_SCCB_ADDR,OV7725_COM5_REG, 0x65);
            SCCB_RegWrite(OV7725_SCCB_ADDR,OV7725_ADVFL_REG, 0x00);
            SCCB_RegWrite(OV7725_SCCB_ADDR,OV7725_ADVFH_REG, 0x00);
            break;
        case 4:   //����
            SCCB_RegWrite(OV7725_SCCB_ADDR,OV7725_COM8_REG, 0xfd);
            SCCB_RegWrite(OV7725_SCCB_ADDR,OV7725_BLUE_REG, 0x96);
            SCCB_RegWrite(OV7725_SCCB_ADDR,OV7725_RED_REG, 0x40);
            SCCB_RegWrite(OV7725_SCCB_ADDR,OV7725_COM5_REG, 0x65);
            SCCB_RegWrite(OV7725_SCCB_ADDR,OV7725_ADVFL_REG, 0x00);
            SCCB_RegWrite(OV7725_SCCB_ADDR,OV7725_ADVFH_REG, 0x00);
            break;
        case 5:   //����
            SCCB_RegWrite(OV7725_SCCB_ADDR,OV7725_COM8_REG, 0xff);
            SCCB_RegWrite(OV7725_SCCB_ADDR,OV7725_BLUE_REG, 0x80);
            SCCB_RegWrite(OV7725_SCCB_ADDR,OV7725_RED_REG, 0x80);
            SCCB_RegWrite(OV7725_SCCB_ADDR,OV7725_COM5_REG, 0xe5);
            SCCB_RegWrite(OV7725_SCCB_ADDR,OV7725_ADVFL_REG, 0x00);
            SCCB_RegWrite(OV7725_SCCB_ADDR,OV7725_ADVFH_REG, 0x00);
            break;
        default:   //�Զ�ģʽ
            SCCB_RegWrite(OV7725_SCCB_ADDR,OV7725_COM8_REG, 0xff);
            SCCB_RegWrite(OV7725_SCCB_ADDR,OV7725_BLUE_REG, 0x80);
            SCCB_RegWrite(OV7725_SCCB_ADDR,OV7725_RED_REG, 0x80);
            SCCB_RegWrite(OV7725_SCCB_ADDR,OV7725_COM5_REG, 0x65);
            SCCB_RegWrite(OV7725_SCCB_ADDR,OV7725_ADVFL_REG, 0x00);
            SCCB_RegWrite(OV7725_SCCB_ADDR,OV7725_ADVFH_REG, 0x00);
            break;
    }
}
/*7725֧��һЩ�򵥵���Ч0:��ͨģʽ 1.�ڰ� 2.����  3,ƫ��4,ƫ��5,ƫ�� 6,��Ƭ*/
void OV7725_SpecialEffectConfigs(uint8_t mode)
{

    switch(mode)
    {
        case 1:  //�ڰ�
            SCCB_RegWrite(OV7725_SCCB_ADDR,OV7725_SDE_REG, 0x26);
            SCCB_RegWrite(OV7725_SCCB_ADDR,OV7725_UFIX_REG, 0x80);
            SCCB_RegWrite(OV7725_SCCB_ADDR,OV7725_VFIX_REG, 0x80);
            break;
        case 2:  //����
            SCCB_RegWrite(OV7725_SCCB_ADDR,OV7725_SDE_REG, 0x1e);
            SCCB_RegWrite(OV7725_SCCB_ADDR,OV7725_UFIX_REG, 0x40);
            SCCB_RegWrite(OV7725_SCCB_ADDR,OV7725_VFIX_REG, 0xa0);
            break;
        case 3:  //ƫ��
            SCCB_RegWrite(OV7725_SCCB_ADDR,OV7725_SDE_REG, 0x1e);
            SCCB_RegWrite(OV7725_SCCB_ADDR,OV7725_UFIX_REG, 0xa0);
            SCCB_RegWrite(OV7725_SCCB_ADDR,OV7725_VFIX_REG, 0x40);
            break;
        case 4:   //ƫ��
            SCCB_RegWrite(OV7725_SCCB_ADDR,OV7725_SDE_REG, 0x1e);
            SCCB_RegWrite(OV7725_SCCB_ADDR,OV7725_UFIX_REG, 0x80);
            SCCB_RegWrite(OV7725_SCCB_ADDR,OV7725_VFIX_REG, 0x40);
            break;
        case 5:   //ƫ��
            SCCB_RegWrite(OV7725_SCCB_ADDR,OV7725_SDE_REG, 0x1e);
            SCCB_RegWrite(OV7725_SCCB_ADDR,OV7725_UFIX_REG, 0x60);
            SCCB_RegWrite(OV7725_SCCB_ADDR,OV7725_VFIX_REG, 0x60);
            break;
        case 6:   //��Ƭ
            SCCB_RegWrite(OV7725_SCCB_ADDR,OV7725_SDE_REG, 0x46);
            SCCB_RegWrite(OV7725_SCCB_ADDR,OV7725_UFIX_REG, 0x00);
            SCCB_RegWrite(OV7725_SCCB_ADDR,OV7725_VFIX_REG, 0x00);
            break;
        default:   //��ͨģʽ
            SCCB_RegWrite(OV7725_SCCB_ADDR,OV7725_SDE_REG, 0x06);
            SCCB_RegWrite(OV7725_SCCB_ADDR,OV7725_UFIX_REG, 0x80);
            SCCB_RegWrite(OV7725_SCCB_ADDR,OV7725_VFIX_REG, 0x80);
            break;
    }
}














/*********************************************************************
 *����ͷSCCB�ײ�����
 *
 ***********************************************************************/



/*************************************************************************
* �����������ܿƼ� KV58���ܳ�ĸ��           
*
*  �������ƣ�void SCCB_Init(void)
*  ����˵��������SCCB��������ΪGPIO���ܣ���ʱ���������ݷ���
*  ����˵������
*  �������أ���
*  �޸�ʱ�䣺2017��12��5��
*  ��    ע��
*************************************************************************/
void SCCB_Init(void)
{
  GPIO_Init(PTD, 11,GPO,1);//����ΪGPIO����
  PORTD_BASE_PTR->PCR[11] |= 0x03; //����
  GPIO_Init(PTD, 10,GPO,1);//����ΪGPIO���� 
  PORTD_BASE_PTR->PCR[10] |= 0x03; //����
}
/*************************************************************************
* �����������ܿƼ� KV58���ܳ�ĸ��           
*
*  �������ƣ�void SCCB_Wait(void)
*  ����˵����SCCB�ȴ���ʾ
*  ����˵������
*  �������أ���
*  �޸�ʱ�䣺2017��12��5��
*  ��    ע��
*************************************************************************/
void SCCB_Wait(void)
{
  uint16_t i=0;
  for(i=0;i<100;i++)
  { 
    asm ("nop");
  }  
}

/*************************************************************************
* �����������ܿƼ� KV58���ܳ�ĸ��           
*
*  �������ƣ�void SCCB_Star(void)
*  ����˵������������
*  ����˵������
*  �������أ���
*  �޸�ʱ�䣺2017��12��5��
*  ��    ע��
*************************************************************************/
void SCCB_Star(void)
{
  SCL_Out;
  SDA_Out;
  SCCB_Wait();
  SDA_High;
  SCL_High; 
  SCCB_Wait();
  SDA_Low;
  SCCB_Wait();
  SCL_Low; 
  SCCB_Wait();
}
/*************************************************************************
* �����������ܿƼ� KV58���ܳ�ĸ��           
*
*  �������ƣ�void SCCB_Stop(void)
*  ����˵����ֹͣ����
*  ����˵������
*  �������أ���
*  �޸�ʱ�䣺2017��12��5��
*  ��    ע��
*************************************************************************/
void SCCB_Stop(void)
{
  SCL_Out;
  SDA_Out;
  SCCB_Wait();
  SDA_Low;
  SCCB_Wait();
  SCL_High; 
  SCCB_Wait();
  SDA_High;
  SCCB_Wait();  
}
/*************************************************************************
* �����������ܿƼ� KV58���ܳ�ĸ��           
*
*  �������ƣ�uint8 SCCB_SendByte(uint8 Data)
*  ����˵����SCCB�����ֽں���
*  ����˵����Ҫ���͵��ֽ�
*  �������أ�Ӧ���ź�
*  �޸�ʱ�䣺2017��12��5��
*  ��    ע��
*************************************************************************/
uint8_t SCCB_SendByte(uint8_t Data)
{
  uint8_t i;
  uint8_t Ack;
  SDA_Out;
  for( i=0; i<8; i++)
  {
    if(Data & 0x80) SDA_High;
    else            SDA_Low;    
    Data <<= 1;
    SCCB_Wait();
    SCL_High;      
    SCCB_Wait();
    SCL_Low;
    SCCB_Wait();
  }
  SDA_In;
  SCCB_Wait();
  
  SCL_High;
  SCCB_Wait();
  Ack = SDA_Data;
  SCL_Low;
  SCCB_Wait();
  SDA_Out;
  return Ack;
}
/*************************************************************** 

* 
* �������ƣ�uint8 SCCB_ReadByte(void) 
* ����˵����SCCB��ȡ�ֽں��� 
* ����˵���� 
* �������أ���ȡ�ֽ� 
* �޸�ʱ�䣺2017��12��5�� 
* �� ע�� 
***************************************************************/ 
uint8_t SCCB_ReadByte(void) 
{ 
  uint8_t i; 
  uint8_t byte = 0; 
  SCL_Out; 
  SDA_In; //ʹ������
  for( i=0; i<8; i++) 
  { 
    SCCB_Wait(); 
    SCL_High;
    SCCB_Wait();
    byte = byte<<1;
    if(SDA_Data)
        byte++;
    SCCB_Wait();
    SCL_Low;  
  }
  SDA_Out;
  SCCB_Wait(); 
  return byte; 
} 
/*************************************************************** 

* 
* �������ƣ�static void SCCB_Ack(void) 
* ����˵����IIC�лظ��ź� 
* ����˵���� 
* �������أ�void 
* �޸�ʱ�䣺2017��12��5�� 
* �� ע�� 
***************************************************************/ 
static void SCCB_Ack(void) 
{ 
  SCL_Out; 
  SDA_Out;
  SCL_Low;
  SDA_Low;
  SCCB_Wait();
  SCL_High;
  SCCB_Wait();
  SCL_Low;
  SCCB_Wait();
} 
/*************************************************************** 

* 
* �������ƣ�static void SCCB_NAck(void) 
* ����˵����IIC�޻ظ��ź� 
* ����˵���� 
* �������أ�void 
* �޸�ʱ�䣺2017��12��5�� 
* �� ע�� 
***************************************************************/ 
static void SCCB_NAck(void) 
{ 
  SCL_Out; 
  SDA_Out;
  SCL_Low;
  SCCB_Wait();
  SDA_High;
  SCCB_Wait();
  SCL_High;
  SCCB_Wait();
  SCL_Low;
  SCCB_Wait();
} 

/*************************************************************************
* �����������ܿƼ� KV58���ܳ�ĸ��           
*
*  �������ƣ�void SCCB_RegWrite(uint8 Device,uint8 Address,uint16 Data)
*  ����˵�������豸д���� 
*  ����˵����Ҫ���͵��ֽ�
*  �������أ�Ӧ���ź�
*  �޸�ʱ�䣺2017��12��5��
*  ��    ע��
*************************************************************************/
uint8_t SCCB_RegWrite(uint8 Device,uint8 Address,uint8_t Data)
{
  uint8_t Ack = 0;
  
    SCCB_Star();
    Ack = SCCB_SendByte(Device<<1);
   
    Ack = SCCB_SendByte(Address);
    
    Ack = SCCB_SendByte(Data);
    
    SCCB_Stop();
    return Ack;
}
/*************************************************************** 

* 
* �������ƣ�uint8_t SCB_RegRead(uint8_t Device,uint8_t Address,uint16_t *Data) 
* ����˵������ȡ���� 
* ����˵���� 
* �������أ�void 
* �޸�ʱ�䣺2017��12��5�� 
* �� ע�� 
***************************************************************/ 
uint8_t SCCB_RegRead(uint8_t Device,uint8_t Address,uint8_t *Data) 
{   
  uint8 Ack = 0;
  Device = Device<<1;
  SCCB_Star();
  Ack += SCCB_SendByte(Device);
  SCCB_Wait();
  Ack += SCCB_SendByte(Address);
  SCCB_Wait();
  SCCB_Stop();
  SCCB_Wait();
  
  SCCB_Star();
  Ack += SCCB_SendByte(Device | 0x01);
  
  *Data = SCCB_ReadByte();
//  SCCB_Ack();
//  *Data = *Data<<8;
  
//  *Data += SCCB_ReadByte();
  SCCB_NAck();
  
  SCCB_Stop();
  
  return  Ack;
} 
/***************************************************************  
* 
* �������ƣ�int SCCB_Probe(uint8_t chipAddr) 
* ����˵������ѯ�õ�ַ���豸�Ƿ���� 
* ����˵���� 
* �������أ�void 
* �޸�ʱ�䣺2017��12��5�� 
* �� ע�� 
***************************************************************/ 
int SCCB_Probe(uint8_t chipAddr) 
{ 
  uint8_t err;
  err = 0;
  chipAddr <<= 1;
  
  SCCB_Star();
  err = SCCB_SendByte(chipAddr);
  SCCB_Stop();
  
  return err;
}