//RuiXiaoliang 20170512
#include "bsp.h"
//#include "oled.h"
//#include "oo_oled.h"
//#include "spi_port.h"
#include "w5500_init.h"



//GPIO_InitTypeDef GPIO_InitStructure;
//USART_InitTypeDef USART_InitStructure;
//ADC_InitTypeDef ADC_InitStructure;
//DMA_InitTypeDef DMA_InitStructure;
//__IO uint16_t ADCConvertedValue;

//t_fifo_buffer com_fifo;

//#define ADC1_DR_Address    ((uint32_t)0x4001244C)

struct di_data di_value;
union di_define di_value_pre;
//struct di_data di_value_pre;
struct do_data do_value = {0xffffffff};
union fault_code_def fault_code;
//struct display_data dsp_data = {0,0,0,0,0,0,0,0,0,0,0};
//struct display_data dsp_data = {2964600000,1299200000,500,1901,2033,1702,0,0,4030,56,1380};
//struct com_send_data com_sdata = {0xaa,0x44,0xc2,0x01,0x01,0x4,{0,0},0};

//uint8_t com_rev_buf[COM_REV_BUF_LEN];

void di_init(u32 init);
void adc_init(void);
void usart1_init(void);
void spi_init(void);

void gpio_init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    //RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOC, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOC | RCC_APB2Periph_GPIOD, ENABLE);
	
    //DO
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10 | GPIO_Pin_12;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3 | GPIO_Pin_15;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10 | GPIO_Pin_12;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(GPIOC, &GPIO_InitStructure);

    GPIO_SetBits(GPIOA,GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10 | GPIO_Pin_12);
    GPIO_SetBits(GPIOB,GPIO_Pin_3 |GPIO_Pin_15);
    GPIO_SetBits(GPIOC,GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10 | GPIO_Pin_12);
	//GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5
					 //| GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10;
    //GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    //GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    //GPIO_Init(GPIOC, &GPIO_InitStructure);
    
    //DI
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_11 | GPIO_Pin_15;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_4 | GPIO_Pin_5
                    | GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10 | GPIO_Pin_11
                    | GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_4
                    | GPIO_Pin_5 | GPIO_Pin_11 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_Init(GPIOC, &GPIO_InitStructure);
    
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_Init(GPIOD, &GPIO_InitStructure);

}
void bsp_init(void)
{
	gpio_init();
    //spi_init();
    w5500_init();
	//di_init(((uint32_t)1 << DI_CHANNEL_NUM) -1);
    //adc_init();
    //spi_init();
    //usart1_init();
    di_init(DI_INIT_VALUE);
}

void di_init(uint32_t init)
{
    uint16_t i;
    di_value.di_new = init;
    di_value.di_filtered = init;//
    for(i=0;i<DI_CHANNEL_NUM;i++)
    {
        di_value.di_timer[i] = 0;
        di_value.di_filter_num[i] = DI_FILTER_VALUE;
    }
}

void read_di(struct di_data *p)
{
    /* Read a half-word from the memory */
    uint16_t i = 0;
    uint32_t tmp_di = 0;
    uint32_t tmp_dia = 0;
    uint32_t tmp_dib = 0;
    uint32_t tmp_dic = 0;
    uint32_t tmp_did = 0;
    uint32_t tmp;
    //p->di_new  =  GPIO_ReadInputData(GPIOC);
    //p->di_new  =  GPIO_ReadOutputDataBit(GPIOA,GPIO_Pin_0);
    tmp_dia = GPIO_ReadInputData(GPIOA) & 0x8807;
    tmp_dib = GPIO_ReadInputData(GPIOB) & 0x7ff7;
    tmp_dic = GPIO_ReadInputData(GPIOC) & 0xe83f;
    tmp_did = GPIO_ReadInputData(GPIOD) & 0x0004;
    tmp_di = (tmp_dia & 0x07) | ((tmp_dia & 0x0800) >> 8) | ((tmp_dia & 0x8000) >> 11); //5bit
    tmp_di |= ((tmp_dib & 0x07) << 5) | ((tmp_dib & 0x7ff0) << 4);                      //14bit
    tmp_di |= ((tmp_dic & 0x3f) << 19) | ((tmp_dic & 0x800) << 14) | ((tmp_dic & 0xe000) << 13);    //10bit
    tmp_di |= (tmp_did & 0x04) << 27;   //1bit
    p->di_new = tmp_di;
    tmp = p->di_new ^ p->di_filtered;
    while(i < DI_CHANNEL_NUM)
    {
        if(0 == (tmp & ((uint32_t)1<<i)))
        {
            p->di_timer[i] = 0;
        }
        else
        {
            ++p->di_timer[i];
            if(p->di_timer[i] > p->di_filter_num[i])
            {
                p->di_filtered &= ~((uint32_t)1<<i);
                p->di_filtered |=  ((uint32_t)1<<i) & p->di_new;
                p->di_timer[i] = 0;
            }
        }
        ++i;
    }
}
void write_do(struct do_data value)
{
    GPIO_WriteBit(GPIOA, GPIO_Pin_8, (BitAction)(value.led1));
    GPIO_WriteBit(GPIOA, GPIO_Pin_9, (BitAction)(value.lift_drop));
    GPIO_WriteBit(GPIOA, GPIO_Pin_10, (BitAction)(value.m_start));
    GPIO_WriteBit(GPIOA, GPIO_Pin_12, (BitAction)(value.m_brake));
    
    GPIO_WriteBit(GPIOB, GPIO_Pin_3, (BitAction)(value.usr2));
    GPIO_WriteBit(GPIOB, GPIO_Pin_15, (BitAction)(value.led6));
    
    GPIO_WriteBit(GPIOC, GPIO_Pin_6, (BitAction)(value.led5));
    GPIO_WriteBit(GPIOC, GPIO_Pin_7, (BitAction)(value.led4));
    GPIO_WriteBit(GPIOC, GPIO_Pin_8, (BitAction)(value.led3));
    GPIO_WriteBit(GPIOC, GPIO_Pin_9, (BitAction)(value.led2));
    GPIO_WriteBit(GPIOC, GPIO_Pin_10, (BitAction)(value.usr3));
    GPIO_WriteBit(GPIOC, GPIO_Pin_12, (BitAction)(value.usr1));
    //int i;
    //for(i=0;i<DO_CHANNEL_NUM;i++)
    //{
        //if(0 != (value & ((uint32_t)(0x01) << i)))
        //{
            //GPIO_WriteBit(GPIOC, GPIO_Pin_9, (BitAction)((~led & 0x10) >> 4));
        //}
    //}
}
void usart1_init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;
    
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE); 

    /* Configure the NVIC Preemption Priority Bits */  
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);

    /* Enable the USARTy Interrupt */
    NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

	/* USARTx configured as follow:
	- BaudRate = 115200 baud  
	- Word Length = 8 Bits
	- One Stop Bit
	- No parity
	- Hardware flow control disabled (RTS and CTS signals)
	- Receive and transmit enabled
	*/
	USART_InitStructure.USART_BaudRate = 115200;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

	//STM_EVAL_COMInit(COM1, &USART_InitStructure);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_USART1 | RCC_APB2Periph_AFIO, ENABLE);

	/* Configure USART Tx as alternate function push-pull */
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	/* Configure USART Rx as input floating */
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	/* USART configuration */
	USART_Init(USART1, &USART_InitStructure);

	/* Enable USART */
	USART_Cmd(USART1, ENABLE);

}

#if 0
void spi_init(void)
{
    SPI_InitTypeDef  SPI_InitStructure;
    GPIO_InitTypeDef GPIO_InitStructure;
    
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
    
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	GPIO_SetBits(GPIOA,GPIO_Pin_4);
    
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	/*!< Configure sFLASH_SPI pins: MISO */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

    /*!< Configure sFLASH_SPI pins: MOSI */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);


	GPIO_SetBits(GPIOA,GPIO_Pin_5|GPIO_Pin_6|GPIO_Pin_7);  //PB13/14/15иою╜
//#if 0
    SPI_InitTypeDef  SPI_InitStructure;

    GPIO_InitTypeDef GPIO_InitStructure;

    /*!< sFLASH_SPI_CS_GPIO, sFLASH_SPI_MOSI_GPIO, sFLASH_SPI_MISO_GPIO 
    and sFLASH_SPI_SCK_GPIO Periph clock enable */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

    /*!< sFLASH_SPI Periph clock enable */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE);

    /*!< Configure sFLASH_SPI pins: SCK */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    //GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    /*!< Configure sFLASH_SPI pins: MOSI */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;  
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    /*!< Configure sFLASH_SPI pins: MISO */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;  
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    /*!< Configure sFLASH_CS_PIN pin: sFLASH Card CS pin */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    /*!< Deselect the FLASH: Chip Select high */
    GPIO_SetBits(GPIOB, GPIO_Pin_8);
    GPIO_SetBits(GPIOB, GPIO_Pin_12);
    //OLED091_CS_SET();
    //OLED242_CS_SET();

    /*!< SPI configuration */
    SPI_InitStructure.SPI_Direction = SPI_Direction_1Line_Tx;
    SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
    SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
    SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;
    SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;
    SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
    SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_8;

    SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
    SPI_InitStructure.SPI_CRCPolynomial = 7;
    SPI_Init(SPI2, &SPI_InitStructure);

    /*!< Enable the sFLASH_SPI  */
    SPI_Cmd(SPI2, ENABLE);
    
}
#endif


void delay_ms(uint16_t t)
{
    while(t--)
    {
        uint32_t i = 72000;
        while(i--);
    }
}
//end of file
