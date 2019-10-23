
#ifndef __SPI_H__
#define __SPI_H__
/**********************************  SPI   ***************************************/
//PCS�ӿڣ����õ�ʱ����Ҫע�ͣ��Ͳ�����г�ʼ����Ӧ�Ĺܽ�
//      ģ��ͨ��    �˿�          ��ѡ��Χ                  ����

#define SPI0_SCK_PIN    PTA15       // PTA15��PTC5��PTD1        ȫ������ ALT2
#define SPI0_SOUT_PIN   PTA16       // PTA16��PTC6��PTD2        ȫ������ ALT2
#define SPI0_SIN_PIN    PTA17       // PTA17��PTC7��PTD3        ȫ������ ALT2

#define SPI0_PCS0_PIN   PTA14       // PTA14��PTC4��PTD0��      ȫ������ ALT2
#define SPI0_PCS1_PIN   PTC3        // PTC3��PTD4               ȫ������ ALT2
#define SPI0_PCS2_PIN   PTC2        // PTC2��PTD5               ȫ������ ALT2
#define SPI0_PCS3_PIN   PTC1        // PTC1��PTD6               ȫ������ ALT2
#define SPI0_PCS4_PIN   PTC0        // PTC0��                   ȫ������ ALT2
#define SPI0_PCS5_PIN   PTB23       // PTB23                    ALT3


#define SPI1_SCK_PIN    PTB11       // PTE2��PTB11��            ȫ������ ALT2
#define SPI1_SOUT_PIN   PTB16       // PTE1��PTB16��            ȫ������ ALT2
#define SPI1_SIN_PIN    PTB17       // PTE3��PTB17��            ȫ������ ALT2

#define SPI1_PCS0_PIN   PTB10       // PTE4��PTB10��            ȫ������ ALT2
#define SPI1_PCS1_PIN   PTE0        // PTE0��PTB9��             ȫ������ ALT2
#define SPI1_PCS2_PIN   PTE5        // PTE5��                   ȫ������ ALT2
#define SPI1_PCS3_PIN   PTE6        // PTE6��                   ȫ������ ALT2


#define SPI2_SCK_PIN    PTB21       // PTB21��PTD12             ȫ������ ALT2
#define SPI2_SOUT_PIN   PTB22       // PTB22��PTD13             ȫ������ ALT2
#define SPI2_SIN_PIN    PTB23       // PTB23��PTD14             ȫ������ ALT2
#define SPI2_PCS0_PIN   PTB20       // PTB20��PTD11             ȫ������ ALT2
#define SPI2_PCS1_PIN   PTD15       // PTD15                    ȫ������ ALT2
/*! ö��PORT ���� */
typedef enum
{
    //�жϷ�ʽ��DMA����ʽ������ֻ��ѡ����һ�֣����Բ�ѡ��
    //�жϷ�ʽѡ��
    IRQ_ZERO     = 0x08 << PORT_PCR_IRQC_SHIFT,   //�͵�ƽ����
    IRQ_RISING   = 0x09 << PORT_PCR_IRQC_SHIFT,   //�����ش���
    IRQ_FALLING  = 0x0A << PORT_PCR_IRQC_SHIFT,   //�½��ش���
    IRQ_EITHER   = 0x0B << PORT_PCR_IRQC_SHIFT,   //�����ش���
    IRQ_ONE      = 0x0C << PORT_PCR_IRQC_SHIFT,   //�ߵ�ƽ����

    //DMA����ѡ��
    DMA_RISING   = 0x01 << PORT_PCR_IRQC_SHIFT,   //�����ش���
    DMA_FALLING  = 0x02 << PORT_PCR_IRQC_SHIFT,   //�½��ش���
    DMA_EITHER   = 0x03 << PORT_PCR_IRQC_SHIFT,   //�����ش���


    HDS          = 0x01 << PORT_PCR_DSE_SHIFT,    //�������������
    ODO          = 0x01 << PORT_PCR_ODE_SHIFT,    //©�����
    PF           = 0x01 << PORT_PCR_PFE_SHIFT,    //����Դ�˲���
    SSR          = 0x01 << PORT_PCR_SRE_SHIFT,    //������仯��          Slow slew rate

    //��������ѡ��
    PULLDOWN     = 0x02 << PORT_PCR_PS_SHIFT,     //����
    PULLUP       = 0x03 << PORT_PCR_PS_SHIFT,     //����

    //���ܸ���ѡ��(�������Ҫ�ı书�ܸ���ѡ�񣬱���ԭ�ȵĹ��ܸ��ã�ֱ��ѡ��ALT0 )
    //��Ҫ�� K60 Signal Multiplexing and Pin Assignments
    ALT0         = 0x00 << PORT_PCR_MUX_SHIFT,
    ALT1         = 0x01 << PORT_PCR_MUX_SHIFT,    //GPIO
    ALT2         = 0x02 << PORT_PCR_MUX_SHIFT,
    ALT3         = 0x03 << PORT_PCR_MUX_SHIFT,
    ALT4         = 0x04 << PORT_PCR_MUX_SHIFT,
    ALT5         = 0x05 << PORT_PCR_MUX_SHIFT,
    ALT6         = 0x06 << PORT_PCR_MUX_SHIFT,
    ALT7         = 0x07 << PORT_PCR_MUX_SHIFT,
} port_cfg;
/**
 *  @brief ���ӻ�ģʽ
 */
typedef enum
{
    MASTER,    //����ģʽ
    SLAVE      //�ӻ�ģʽ
} SPI_CFG;

/**
 *  @brief SPIģ���
 */
typedef enum
{
    SPI_0,
    SPI_1,
    SPI_2
} SPIn_e;

/**
 *  @brief SPIģ��Ƭѡ��
 */
typedef enum
{
    SPI_PCS0 = 1 << 0,
    SPI_PCS1 = 1 << 1,
    SPI_PCS2 = 1 << 2,
    SPI_PCS3 = 1 << 3,
    SPI_PCS4 = 1 << 4,
    SPI_PCS5 = 1 << 5,
} SPI_PCSn_e;



extern uint32 spi_init       (SPIn_e, SPI_PCSn_e , SPI_CFG,uint32 baud);                                        //SPI��ʼ����ѡ��Ƭѡ�źţ�����ģʽ��������
uint32 spi_set_baud (SPIn_e,                      uint32 baud);

//�������շ��ͺ���
extern void spi_mosi       (SPIn_e, SPI_PCSn_e pcs,                              uint8 *modata, uint8 *midata,               uint32 len);    //SPI���ͽ��պ���,����databuff���ݣ����ѽ��յ������ݴ����databuff��(ע�⣬�Ḳ��ԭ����databuff)
extern void spi_mosi_cmd   (SPIn_e, SPI_PCSn_e pcs, uint8 *mocmd , uint8 *micmd , uint8 *modata, uint8 *midata, uint32 cmdlen , uint32 len); //SPI���ͽ��պ���,��spi_mosi��ȣ������ȷ���cmd �������Ĳ��裬���ֿ������ַ���

#endif