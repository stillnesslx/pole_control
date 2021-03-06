//RuiXiaoliang
//20170515

#include "oled.h"
#include "oledfont.h"
#include "bsp.h"
uint8_t spi2_send_byte(uint8_t byte)
{
  /*!< Loop while DR register in not emplty */
  while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE) == RESET);

  /*!< Send byte through the SPI1 peripheral */
  SPI_I2S_SendData(SPI2, byte);

  /*!< Wait to receive a byte */
  //while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_RXNE) == RESET);

  /*!< Return the byte read from the SPI bus */
  return SPI_I2S_ReceiveData(SPI2);
}
void oled091_wr_byte(u8 dat,u8 cmd)
{	
	//u8 i;			  
	if(cmd)
	  OLED_DC_SET();
	else
	  OLED_DC_CLR();
	OLED091_CS_CLR();
    spi2_send_byte(dat);
//	for(i=0;i<8;i++)
//	{
//		OLED_SCLK_CLR();
//		if(dat&0x80)
//		   OLED_SDIN_SET();
//		else
//		   OLED_SDIN_CLR();
//		OLED_SCLK_SET();
//		dat<<=1;
//	}
    delay_ms(1);
	OLED091_CS_SET();
	OLED_DC_SET();
}

void OLED_Set_Pos(unsigned char x, unsigned char y) 
{ 
	oled091_wr_byte(0xb0+y,OLED_CMD);
	oled091_wr_byte(((x&0xf0)>>4)|0x10,OLED_CMD);
	oled091_wr_byte((x&0x0f)|0x01,OLED_CMD); 
}   	  
//开启OLED显示    
void OLED_Display_On(void)
{
	oled091_wr_byte(0X8D,OLED_CMD);  //SET DCDC命令
	oled091_wr_byte(0X14,OLED_CMD);  //DCDC ON
	oled091_wr_byte(0XAF,OLED_CMD);  //DISPLAY ON
}
//关闭OLED显示     
void OLED_Display_Off(void)
{
	oled091_wr_byte(0X8D,OLED_CMD);  //SET DCDC命令
	oled091_wr_byte(0X10,OLED_CMD);  //DCDC OFF
	oled091_wr_byte(0XAE,OLED_CMD);  //DISPLAY OFF
}		   			 
//清屏函数,清完屏,整个屏幕是黑色的!和没点亮一样!!!	  
void OLED_Clear(void)  
{  
	u8 i,n;		    
	for(i=0;i<8;i++)  
	{  
		oled091_wr_byte (0xb0+i,OLED_CMD);    //设置页地址（0~7）
		oled091_wr_byte (0x00,OLED_CMD);      //设置显示位置—列低地址
		oled091_wr_byte (0x10,OLED_CMD);      //设置显示位置—列高地址   
		for(n=0;n<128;n++)oled091_wr_byte(0,OLED_DATA); 
	} //更新显示
}


//在指定位置显示一个字符,包括部分字符
//x:0~127
//y:0~63
//mode:0,反白显示;1,正常显示				 
//size:选择字体 16/12 
void OLED_ShowChar(u8 x,u8 y,u8 chr)
{      	
	unsigned char c=0,i=0;	
		c=chr-' ';//得到偏移后的值			
		if(x>Max_Column-1){x=0;y=y+2;}
		if(SIZE ==16)
			{
			OLED_Set_Pos(x,y);	
			for(i=0;i<8;i++)
			oled091_wr_byte(F8X16[c*16+i],OLED_DATA);
			OLED_Set_Pos(x,y+1);
			for(i=0;i<8;i++)
			oled091_wr_byte(F8X16[c*16+i+8],OLED_DATA);
			}
			else {	
				OLED_Set_Pos(x,y+1);
				for(i=0;i<6;i++)
				oled091_wr_byte(F6x8[c][i],OLED_DATA);
				
			}
}
//m^n函数
u32 oled_pow(u8 m,u8 n)
{
	u32 result=1;	 
	while(n--)result*=m;    
	return result;
}				  
//显示2个数字
//x,y :起点坐标	 
//len :数字的位数
//size:字体大小
//mode:模式	0,填充模式;1,叠加模式
//num:数值(0~4294967295);	 		  
void OLED_ShowNum(u8 x,u8 y,u32 num,u8 len,u8 size)
{         	
	u8 t,temp;
	u8 enshow=0;						   
	for(t=0;t<len;t++)
	{
		temp=(num/oled_pow(10,len-t-1))%10;
		if(enshow==0&&t<(len-1))
		{
			if(temp==0)
			{
				OLED_ShowChar(x+(size/2)*t,y,' ');
				continue;
			}else enshow=1; 
		 	 
		}
	 	OLED_ShowChar(x+(size/2)*t,y,temp+'0'); 
	}
} 
//显示一个字符号串
void OLED_ShowString(u8 x,u8 y,u8 *chr)
{
	unsigned char j=0;
	while (chr[j]!='\0')
	{		OLED_ShowChar(x,y,chr[j]);
			x+=8;
		if(x>120){x=0;y+=2;}
			j++;
	}
}
//显示汉字
void OLED_ShowCHinese(u8 x,u8 y,u8 no)
{      			    
	u8 t,adder=0;
	OLED_Set_Pos(x,y);	
    for(t=0;t<16;t++)
		{
				oled091_wr_byte(Hzk[2*no][t],OLED_DATA);
				adder+=1;
     }	
		OLED_Set_Pos(x,y+1);	
    for(t=0;t<16;t++)
			{	
				oled091_wr_byte(Hzk[2*no+1][t],OLED_DATA);
				adder+=1;
      }					
}
/***********功能描述：显示显示BMP图片128×64起始点坐标(x,y),x的范围0～127，y为页的范围0～7*****************/
void OLED_DrawBMP(unsigned char x0, unsigned char y0,unsigned char x1, unsigned char y1,unsigned char BMP[])
{ 	
 unsigned int j=0;
 unsigned char x,y;
  
  if(y1%8==0) y=y1/8;      
  else y=y1/8+1;
	for(y=y0;y<y1;y++)
	{
		OLED_Set_Pos(x0,y);
    for(x=x0;x<x1;x++)
	    {      
	    	oled091_wr_byte(BMP[j++],OLED_DATA);	    	
	    }
	}
} 


//初始化SSD1306					    
void OLED_Init(void)
{ 	
 
 	 
 	GPIO_InitTypeDef  GPIO_InitStructure;
 	
 	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);	 //使能A端口时钟
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4|GPIO_Pin_5|GPIO_Pin_7;	 
 	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 		 //推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;//速度50MHz
 	GPIO_Init(GPIOA, &GPIO_InitStructure);	  //初始化GPIOD3,6
 	GPIO_SetBits(GPIOA,GPIO_Pin_5|GPIO_Pin_7|GPIO_Pin_4);	
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);	 //使能A端口时钟
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_8;	 
 	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 		 //推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;//速度50MHz
 	GPIO_Init(GPIOB, &GPIO_InitStructure);	  //初始化GPIOD3,6
 	GPIO_SetBits(GPIOB,GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_8);	



 
  OLED_RST_SET();
	delay_ms(100);
	OLED_RST_CLR();
	delay_ms(200);
	OLED_RST_SET();
					  
oled091_wr_byte(0xAE,OLED_CMD);//关闭显示
	
	oled091_wr_byte(0x40,OLED_CMD);//---set low column address
	oled091_wr_byte(0xB0,OLED_CMD);//---set high column address

	oled091_wr_byte(0xC8,OLED_CMD);//-not offset

	oled091_wr_byte(0x81,OLED_CMD);//设置对比度
	oled091_wr_byte(0xff,OLED_CMD);

	oled091_wr_byte(0xa1,OLED_CMD);//段重定向设置

	oled091_wr_byte(0xa6,OLED_CMD);//
	
	oled091_wr_byte(0xa8,OLED_CMD);//设置驱动路数
	oled091_wr_byte(0x1f,OLED_CMD);
	
	oled091_wr_byte(0xd3,OLED_CMD);
	oled091_wr_byte(0x00,OLED_CMD);
	
	oled091_wr_byte(0xd5,OLED_CMD);
	oled091_wr_byte(0xf0,OLED_CMD);
	
	oled091_wr_byte(0xd9,OLED_CMD);
	oled091_wr_byte(0x22,OLED_CMD);
	
	oled091_wr_byte(0xda,OLED_CMD);
	oled091_wr_byte(0x02,OLED_CMD);
	
	oled091_wr_byte(0xdb,OLED_CMD);
	oled091_wr_byte(0x49,OLED_CMD);
	
	oled091_wr_byte(0x8d,OLED_CMD);
	oled091_wr_byte(0x14,OLED_CMD);
	
	oled091_wr_byte(0xaf,OLED_CMD); 	
}  
//end of file
