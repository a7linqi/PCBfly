/*******************************************************************
 *@title LED system
 *@brief flight light
 *@brief 
 *@time  2016.10.19
 *@editor小南&zin
 *飞控爱好QQ群551883670,邮箱759421287@qq.com
 ******************************************************************/
#include "stm32f10x.h"
#include "LED.h"
#include "ALL_DATA.h"
//机身前灯				 
#define fLED_H()  GPIOB->BRR = GPIO_Pin_0|GPIO_Pin_1
#define fLED_L()  GPIOB->BSRR = GPIO_Pin_0|GPIO_Pin_1
#define fLED_Toggle()  GPIOB->ODR ^= GPIO_Pin_0|GPIO_Pin_1

//-------------------------------------------------
//机身后灯				 
#define bLED_H()  GPIOA->BRR = GPIO_Pin_6|GPIO_Pin_7
#define bLED_L()  GPIOA->BSRR  = GPIO_Pin_6|GPIO_Pin_7
#define bLED_Toggle()  GPIOA->ODR ^= GPIO_Pin_6|GPIO_Pin_7

//-------------------------------------------------
//---------------------------------------------------------
/*     you can select the LED statue on enum contains            */
sLED LED = {100,AllFlashLight};  //LED initial statue is off;
                             //default 300ms flash the status
/**************************************************************
 *  LED Init
 * @param[in] 
 * @param[out] 
 * @return     
 ***************************************************************/
void LEDInit(void)	
{	
	GPIO_InitTypeDef GPIO_InitStructure;

	 RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOA , ENABLE);
  	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;		   
  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;		     
  	GPIO_Init(GPIOB, &GPIO_InitStructure);	
	fLED_L();//暗
	bLED_L();
}

/**************************************************************
 *  LED system
 * @param[in] 
 * @param[out] 
 * @return     
 ***************************************************************/	
void PilotLED() //flash 300MS interval
{
	static uint32_t LastTime = 0;

	if(SysTick_count - LastTime < LED.FlashTime)
	{

		return;
	}
	else
		LastTime = SysTick_count;
	switch(LED.status)
	{
		case AlwaysOff:      //常暗   
			bLED_L();
			fLED_L();
			break;
		case AllFlashLight:				  //全部同时闪烁
			fLED_Toggle();			
			bLED_Toggle();				
		  break;
		case AlwaysOn:  //常亮
			bLED_H();
			fLED_H();
		  break;
		case AlternateFlash: //交替闪烁
			bLED_H();
			fLED_L();
			LED.status = AllFlashLight;
			break;
		case WARNING:
			fLED_L();
			bLED_Toggle();
			LED.FlashTime = 100;
			break;
		case DANGEROURS:
			bLED_L();
			fLED_Toggle();
			LED.FlashTime = 70;
			break;
		default:
			LED.status = AlwaysOff;
			break;
	}
}

/**************************END OF FILE*********************************/



