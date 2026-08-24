/**************************************************************
 * 
 * @brief
   ZIN-7套件
	 飞控爱好群551883670
	 淘宝地址：https://shop297229812.taobao.com/shop/view_shop.htm?mytmenu=mdianpu&user_number_id=2419305772
 ***************************************************************/
#include "ALL_DEFINE.h"
#include "scheduler.h"



void TIM3_IRQHandler(void)   //TIM3中断 3ms
{
	
	if (TIM_GetITStatus(TIM3, TIM_IT_Update) != RESET) //检查指定的TIM中断发生与否:TIM 中断源 
	{
		static uint8_t cnt_3ms = 0;
		static uint8_t cnt_6ms = 0;	 
		SysTick_count++;
		cnt_3ms++;
		cnt_6ms++;
		
		if(cnt_3ms == 1)  //3ms更新一次
		{
			cnt_3ms = 0;
			MpuGetData();
			RC_Analy();			
			FlightPidControl(0.003f);
			MotorControl();
		}		
		if(cnt_6ms == 2) //6ms更新一次
		{
			cnt_6ms = 0;
			GetAngle(&MPU6050,&Angle,0.00626f);
		}		

		TIM_ClearITPendingBit(TIM3, TIM_IT_Update );  //清除TIMx的中断待处理位:TIM 中断源 
	}
}



