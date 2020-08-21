#ifndef __SPI_PORT_H
#define __SPI_PORT_H

#ifdef __cplusplus
 extern "C" {
#endif 

#include "stm32f10x.h"
//#include "system_config.h"

/*#define  W5200_CS_HIGH()  do{GPIO_SetBits(W5200_CS_PORT,W5200_CS_PIN);}while(0)
#define  W5200_CS_LOW()  do{\
		GPIO_SetBits(NRF24L01_CS_PORT,NRF24L01_CS_PIN);\
		GPIO_SetBits(SD_CARD_CS_PORT,SD_CARD_CS_PIN);\
		GPIO_SetBits(SFLAH_CS_PORT,SFLAH_CS_PIN);\
		GPIO_ResetBits(W5200_CS_PORT,W5200_CS_PIN);}while(0)*/

#define SPI1_CS_LOW() do{GPIO_SetBits(GPIOA,GPIO_Pin_4);}while(0)
#define SPI1_CS_HIGH() do{GPIO_ResetBits(GPIOA,GPIO_Pin_4);}while(0)

extern void spi1_cs_low(void);
extern void spi1_cs_high(void);


//extern u8 spi1_send_byte(u8 byte);
//extern void spi1_cs_init(void);
//extern void spi1_init(void);
//extern void SPI1_Init(void);			 //初始化SPI口
//extern void SPI1_SetSpeed(u8 SpeedSet); //设置SPI速度
extern void spi_init(void);
extern uint8_t spi1_read_byte(void);
extern uint8_t spi1_read_write_byte(uint8_t tx_data);//SPI总线读写一个字节

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif
