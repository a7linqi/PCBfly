/**************************************************************
 * 
 * @brief
   ZIN-7套件
	 飞控爱好群551883670
	 淘宝地址：https://shop297229812.taobao.com/shop/view_shop.htm?mytmenu=mdianpu&user_number_id=2419305772
 ***************************************************************/
#include "nrf24l01.h"
#include "SPI.h"
#include <string.h>

#undef SUCCESS
#define SUCCESS 0
#undef FAILED
#define FAILED  1


#define MAX_TX  		0x10  //达到最大发送次数中断
#define TX_OK   		0x20  //TX发送完成中断
#define RX_OK   		0x40  //接收到数据中断

//////////////////////////////////////////////////////////////////////////////////////////////////////////
////24L01操作线   
//#define SPI_CSN_H    (GPIOB->BSRR = GPIO_Pin_12) // PB12
//#define SPI_CSN_L     (GPIOB->BRR = GPIO_Pin_12)    // PB12
//#define NRF24L01_CE_H (GPIOA->BSRR = GPIO_Pin_11)    // PB1
//#define NRF24L01_CE_L  (GPIOA->BRR = GPIO_Pin_11)    // PB1
//#define NRF24L01_IRQ   (GPIOB->IDR&GPIO_Pin_0)//IRQ主机数据输入 PB0


#include "nrf24l01.h"
#include "SPI.h"
#include <string.h>

#undef SUCCESS
#define SUCCESS 0
#undef FAILED
#define FAILED  1

/**************************************************************************************/
//配对密码//各套件主控与遥控的匹配码相同才可以实现通信
//只购买一套者默认地址5
//手上有两套及以上着，实现一对一通信匹配码在这里更改，遥控与主控上会有地址序号来区分不同套，
//地址5
const uint8_t TX_ADDRESS[]= {0xE1,0xE2,0xE3,0xE4,0xE5};	//本地地址
const uint8_t RX_ADDRESS[]= {0xE1,0xE2,0xE3,0xE4,0xE5};	//接收地址RX_ADDR_P0 == RX_ADDR
////地址4
//const uint8_t TX_ADDRESS[]= {0xE1,0xE2,0xE3,0xE4,0xE4};	//本地地址
//const uint8_t RX_ADDRESS[]= {0xE1,0xE2,0xE3,0xE4,0xE4};	//接收地址RX_ADDR_P0 == RX_ADDR
////地址3
//const uint8_t TX_ADDRESS[]= {0xE1,0xE2,0xE3,0xE4,0xE3};	//本地地址
//const uint8_t RX_ADDRESS[]= {0xE1,0xE2,0xE3,0xE4,0xE3};	//接收地址RX_ADDR_P0 == RX_ADDR
////地址2
//const uint8_t TX_ADDRESS[]= {0xE1,0xE2,0xE3,0xE4,0xE2};	//本地地址
//const uint8_t RX_ADDRESS[]= {0xE1,0xE2,0xE3,0xE4,0xE2};	//接收地址RX_ADDR_P0 == RX_ADDR
////地址1
//const uint8_t TX_ADDRESS[]= {0xE1,0xE2,0xE3,0xE4,0xE1};	//本地地址
//const uint8_t RX_ADDRESS[]= {0xE1,0xE2,0xE3,0xE4,0xE1};	//接收地址RX_ADDR_P0 == RX_ADDR
/**************************************************************************************/

//////////////////////////////////////////////////////////////////////////////////////////////////////////
//24L01操作线   
#define Set_NRF24L01_CSN    (GPIOB->BSRR = GPIO_Pin_4) 
#define Clr_NRF24L01_CSN     (GPIOB->BRR = GPIO_Pin_4)    
#define Set_NRF24L01_CE (GPIOB->BSRR = GPIO_Pin_3)    
#define Clr_NRF24L01_CE  (GPIOB->BRR = GPIO_Pin_3)    
#define READ_NRF24L01_IRQ   (((GPIOB->IDR&GPIO_Pin_5)==GPIO_Pin_5)?1:0)//IRQ主机数据输入 PB0

//初始化24L01的IO口
void NRF24L01_Configuration(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);//开启引脚复用
	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable , ENABLE);//PB3，PB4，PA15做普通IO
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);    //使能GPIO的时钟  CE
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3|GPIO_Pin_4;          //NRF24L01  
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;    //推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	Set_NRF24L01_CE;                                    //初始化时先拉高
	Set_NRF24L01_CSN;                                   //初始化时先拉高

    //配置NRF2401的IRQ
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU  ;     //上拉输入
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	SPI_Config();                                //初始化SPI
	Clr_NRF24L01_CE; 	                                //使能24L01
	Set_NRF24L01_CSN;                                   //SPI片选取消
}
//上电检测NRF24L01是否在位
//写5个数据然后再读回来进行比较，
//相同时返回值:0，表示在位;否则返回1，表示不在位	
u8 NRF24L01_Check(void)
{
	u8 buf[5]={0XA5,0XA5,0XA5,0XA5,0XA5};
	u8 buf1[5];
	u8 i;   	 
	NRF24L01_Write_Buf(SPI_WRITE_REG+TX_ADDR,buf,5);//写入5个字节的地址.	
	NRF24L01_Read_Buf(TX_ADDR,buf1,5);              //读出写入的地址  	
	for(i=0;i<5;i++)if(buf1[i]!=0XA5)break;					   
	if(i!=5)return FAILED;                               //NRF24L01不在位	
	return SUCCESS;		                                //NRF24L01在位
}	 	 
//通过SPI写寄存器
u8 NRF24L01_Write_Reg(u8 regaddr,u8 data)
{
	u8 status;	
    Clr_NRF24L01_CSN;                    //使能SPI传输
  	status =SPI_RW(regaddr); //发送寄存器号 
  	SPI_RW(data);            //写入寄存器的值
  	Set_NRF24L01_CSN;                    //禁止SPI传输	   
  	return(status);       		         //返回状态值
}
//读取SPI寄存器值 ，regaddr:要读的寄存器
u8 NRF24L01_Read_Reg(u8 regaddr)
{
	u8 reg_val;	    
	Clr_NRF24L01_CSN;                //使能SPI传输		
  	SPI_RW(regaddr);     //发送寄存器号
  	reg_val=SPI_RW(0XFF);//读取寄存器内容
  	Set_NRF24L01_CSN;                //禁止SPI传输		    
  	return(reg_val);                 //返回状态值
}	
//在指定位置读出指定长度的数据
//*pBuf:数据指针
//返回值,此次读到的状态寄存器值 
u8 NRF24L01_Read_Buf(u8 regaddr,u8 *pBuf,u8 datalen)
{
	u8 status,u8_ctr;	       
  	Clr_NRF24L01_CSN;                     //使能SPI传输
  	status=SPI_RW(regaddr);   //发送寄存器值(位置),并读取状态值   	   
	for(u8_ctr=0;u8_ctr<datalen;u8_ctr++)pBuf[u8_ctr]=SPI_RW(0XFF);//读出数据
  	Set_NRF24L01_CSN;                     //关闭SPI传输
  	return status;                        //返回读到的状态值
}
//在指定位置写指定长度的数据
//*pBuf:数据指针
//返回值,此次读到的状态寄存器值
u8 NRF24L01_Write_Buf(u8 regaddr, u8 *pBuf, u8 datalen)
{
	u8 status,u8_ctr;	    
	Clr_NRF24L01_CSN;                                    //使能SPI传输
  	status = SPI_RW(regaddr);                //发送寄存器值(位置),并读取状态值
  	for(u8_ctr=0; u8_ctr<datalen; u8_ctr++)SPI_RW(*pBuf++); //写入数据	 
  	Set_NRF24L01_CSN;                                    //关闭SPI传输
  	return status;                                       //返回读到的状态值
}				   
//启动NRF24L01发送一次数据
//txbuf:待发送数据首地址
//返回值:发送完成状况
u8 NRF24L01_TxPacket(u8 *txbuf)
{
	u8 state;   
	Clr_NRF24L01_CE;
	NRF24L01_Write_Buf(WR_TX_PLOAD,txbuf,TX_PLOAD_WIDTH);//写数据到TX BUF  32个字节
	Set_NRF24L01_CE;                                     //启动发送	   
	while(READ_NRF24L01_IRQ!=0);                         //等待发送完成
	state=NRF24L01_Read_Reg(STATUS);                     //读取状态寄存器的值	   
	NRF24L01_Write_Reg(SPI_WRITE_REG+STATUS,state);      //清除TX_DS或MAX_RT中断标志
	if(state&MAX_TX)                                     //达到最大重发次数
	{
		NRF24L01_Write_Reg(FLUSH_TX,0xff);               //清除TX FIFO寄存器 
		return MAX_TX; 
	}
	if(state&TX_OK)                                      //发送完成
	{
		return SUCCESS;
	}
	return FAILED;                                         //其他原因发送失败
}

//启动NRF24L01发送一次数据
//txbuf:待发送数据首地址
//返回值:0，接收完成；其他，错误代码
u8 NRF24L01_RxPacket(u8 *rxbuf)
{
	u8 state;		    							      
	state=NRF24L01_Read_Reg(STATUS);                //读取状态寄存器的值    	 
	NRF24L01_Write_Reg(SPI_WRITE_REG+STATUS,state); //清除TX_DS或MAX_RT中断标志
	if(state&RX_OK)                                 //接收到数据
	{
		NRF24L01_Read_Buf(RD_RX_PLOAD,rxbuf,RX_PLOAD_WIDTH);//读取数据
		NRF24L01_Write_Reg(FLUSH_RX,0xff);          //清除RX FIFO寄存器 
		return SUCCESS; 
	}	   
	return FAILED;                                      //没收到任何数据
}

void RX_Mode(void)
{
	Clr_NRF24L01_CE;	  
    //写RX节点地址
  	NRF24L01_Write_Buf(SPI_WRITE_REG+RX_ADDR_P0,(u8*)RX_ADDRESS,RX_ADR_WIDTH);

    //使能通道0的自动应答    
  	NRF24L01_Write_Reg(SPI_WRITE_REG+EN_AA,0);    
    //使能通道0的接收地址  	 
  	NRF24L01_Write_Reg(SPI_WRITE_REG+EN_RXADDR,0x01);
    //设置RF通信频率		  
  	NRF24L01_Write_Reg(SPI_WRITE_REG+RF_CH,1);	     
    //选择通道0的有效数据宽度 	    
  	NRF24L01_Write_Reg(SPI_WRITE_REG+RX_PW_P0,RX_PLOAD_WIDTH);
    //设置TX发射参数,0db增益,2Mbps,低噪声增益开启   
  	NRF24L01_Write_Reg(SPI_WRITE_REG+RF_SETUP,0x07);
    //配置基本工作模式的参数;PWR_UP,EN_CRC,16BIT_CRC,PRIM_RX接收模式 
  	NRF24L01_Write_Reg(SPI_WRITE_REG+NCONFIG, 0x0f); 
    //CE为高,进入接收模式 
  	Set_NRF24L01_CE;                                
}			  

void NRF24L01_init(void)
{
	NRF24L01_Configuration();  //相关引脚配置
  while(NRF24L01_Check() == FAILED);	//如果NRF错误
   RX_Mode(); //接收模式
}

/*********************END OF FILE******************************************************/


















