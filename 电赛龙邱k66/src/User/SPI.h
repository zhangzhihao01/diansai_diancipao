
#ifndef __SPI_H__
#define __SPI_H__
/**********************************  SPI   ***************************************/
//PCS接口，不用的时候需要注释，就不会进行初始化对应的管脚
//      模块通道    端口          可选范围                  建议

#define SPI0_SCK_PIN    PTA15       // PTA15、PTC5、PTD1        全部都是 ALT2
#define SPI0_SOUT_PIN   PTA16       // PTA16、PTC6、PTD2        全部都是 ALT2
#define SPI0_SIN_PIN    PTA17       // PTA17、PTC7、PTD3        全部都是 ALT2

#define SPI0_PCS0_PIN   PTA14       // PTA14、PTC4、PTD0、      全部都是 ALT2
#define SPI0_PCS1_PIN   PTC3        // PTC3、PTD4               全部都是 ALT2
#define SPI0_PCS2_PIN   PTC2        // PTC2、PTD5               全部都是 ALT2
#define SPI0_PCS3_PIN   PTC1        // PTC1、PTD6               全部都是 ALT2
#define SPI0_PCS4_PIN   PTC0        // PTC0、                   全部都是 ALT2
#define SPI0_PCS5_PIN   PTB23       // PTB23                    ALT3


#define SPI1_SCK_PIN    PTB11       // PTE2、PTB11、            全部都是 ALT2
#define SPI1_SOUT_PIN   PTB16       // PTE1、PTB16、            全部都是 ALT2
#define SPI1_SIN_PIN    PTB17       // PTE3、PTB17、            全部都是 ALT2

#define SPI1_PCS0_PIN   PTB10       // PTE4、PTB10、            全部都是 ALT2
#define SPI1_PCS1_PIN   PTE0        // PTE0、PTB9、             全部都是 ALT2
#define SPI1_PCS2_PIN   PTE5        // PTE5、                   全部都是 ALT2
#define SPI1_PCS3_PIN   PTE6        // PTE6、                   全部都是 ALT2


#define SPI2_SCK_PIN    PTB21       // PTB21、PTD12             全部都是 ALT2
#define SPI2_SOUT_PIN   PTB22       // PTB22、PTD13             全部都是 ALT2
#define SPI2_SIN_PIN    PTB23       // PTB23、PTD14             全部都是 ALT2
#define SPI2_PCS0_PIN   PTB20       // PTB20、PTD11             全部都是 ALT2
#define SPI2_PCS1_PIN   PTD15       // PTD15                    全部都是 ALT2
/*! 枚举PORT 配置 */
typedef enum
{
    //中断方式和DMA请求方式，两者只能选其中一种（可以不选）
    //中断方式选择
    IRQ_ZERO     = 0x08 << PORT_PCR_IRQC_SHIFT,   //低电平触发
    IRQ_RISING   = 0x09 << PORT_PCR_IRQC_SHIFT,   //上升沿触发
    IRQ_FALLING  = 0x0A << PORT_PCR_IRQC_SHIFT,   //下降沿触发
    IRQ_EITHER   = 0x0B << PORT_PCR_IRQC_SHIFT,   //跳变沿触发
    IRQ_ONE      = 0x0C << PORT_PCR_IRQC_SHIFT,   //高电平触发

    //DMA请求选择
    DMA_RISING   = 0x01 << PORT_PCR_IRQC_SHIFT,   //上升沿触发
    DMA_FALLING  = 0x02 << PORT_PCR_IRQC_SHIFT,   //下降沿触发
    DMA_EITHER   = 0x03 << PORT_PCR_IRQC_SHIFT,   //跳变沿触发


    HDS          = 0x01 << PORT_PCR_DSE_SHIFT,    //输出高驱动能力
    ODO          = 0x01 << PORT_PCR_ODE_SHIFT,    //漏极输出
    PF           = 0x01 << PORT_PCR_PFE_SHIFT,    //带无源滤波器
    SSR          = 0x01 << PORT_PCR_SRE_SHIFT,    //输出慢变化率          Slow slew rate

    //下拉上拉选择
    PULLDOWN     = 0x02 << PORT_PCR_PS_SHIFT,     //下拉
    PULLUP       = 0x03 << PORT_PCR_PS_SHIFT,     //上拉

    //功能复用选择(如果不需要改变功能复用选择，保留原先的功能复用，直接选择ALT0 )
    //需要查 K60 Signal Multiplexing and Pin Assignments
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
 *  @brief 主从机模式
 */
typedef enum
{
    MASTER,    //主机模式
    SLAVE      //从机模式
} SPI_CFG;

/**
 *  @brief SPI模块号
 */
typedef enum
{
    SPI_0,
    SPI_1,
    SPI_2
} SPIn_e;

/**
 *  @brief SPI模块片选号
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



extern uint32 spi_init       (SPIn_e, SPI_PCSn_e , SPI_CFG,uint32 baud);                                        //SPI初始化，选择片选信号，设置模式，波特率
uint32 spi_set_baud (SPIn_e,                      uint32 baud);

//主机接收发送函数
extern void spi_mosi       (SPIn_e, SPI_PCSn_e pcs,                              uint8 *modata, uint8 *midata,               uint32 len);    //SPI发送接收函数,发送databuff数据，并把接收到的数据存放在databuff里(注意，会覆盖原来的databuff)
extern void spi_mosi_cmd   (SPIn_e, SPI_PCSn_e pcs, uint8 *mocmd , uint8 *micmd , uint8 *modata, uint8 *midata, uint32 cmdlen , uint32 len); //SPI发送接收函数,与spi_mosi相比，多了先发送cmd 缓冲区的步骤，即分开两部分发送

#endif