/**************************************************************
 * 
 * @brief
   ZIN-7套件
	 飞控爱好群551883670
	 淘宝地址：https://shop297229812.taobao.com/shop/view_shop.htm?mytmenu=mdianpu&user_number_id=2419305772
 ***************************************************************/
#include "ALL_DATA.h"
#include "nrf24l01.h"
#include "control.h"
#include <math.h>
#include "myMath.h"
#include "LED.h"
#include "Remote.h"
#include "mpu6050.h"
#include "imu.h"
#undef SUCCESS
#define SUCCESS 0
#undef FAILED
#define FAILED  1

/*****************************************************************************************
 *  通道数据处理
 * @param[in] 
 * @param[out] 
 * @return     
 ******************************************************************************************/	
	uint8_t RC_rxData[32];
void remote_unlock(void);	

void RC_Analy(void)  
{
		static uint16_t cnt_15s;
/*             Receive  and check RC data                               */	
	if(NRF24L01_RxPacket(RC_rxData)==SUCCESS)
	{ 	

		uint8_t i;
		uint8_t CheckSum=0;
		cnt_15s = 0;
		for(i=0;i<31;i++)//校验和
		{
			CheckSum +=  RC_rxData[i];
		}
		if(RC_rxData[31]==CheckSum && RC_rxData[0]==0xAA && RC_rxData[1]==0xAF)  //如果接收到的遥控数据正确
		{
			  Remote.roll = ((uint16_t)RC_rxData[4]<<8) | RC_rxData[5];  //通道1
			
				Remote.pitch = ((uint16_t)RC_rxData[6]<<8) | RC_rxData[7];  //通道2
	
				Remote.thr = 	((uint16_t)RC_rxData[8]<<8) | RC_rxData[9];   //通道3

				Remote.yaw =  ((uint16_t)RC_rxData[10]<<8) | RC_rxData[11];   //通道4
		
				Remote.AUX1 =  ((uint16_t)RC_rxData[12]<<8) | RC_rxData[13];   //通道5  左上角按键都属于通道5  
	
				Remote.AUX2 =  ((uint16_t)RC_rxData[14]<<8) | RC_rxData[15];   //通道6  右上角按键都属于通道6 

				Remote.AUX3 =  ((uint16_t)RC_rxData[16]<<8) | RC_rxData[17];   //通道7  按键请查看手册 

				Remote.AUX4 =  ((uint16_t)RC_rxData[18]<<8) | RC_rxData[19];   //通道8  
	
		
			
			
				//开始处理遥控数据
				{
							float roll_pitch_ratio = 0.04f;  //遥控控制姿态的量
					
					
							pidPitch.desired =-(Remote.pitch-1500)*roll_pitch_ratio;	 //将遥杆值作为飞行角度的期望值  
							pidRoll.desired = -(Remote.roll-1500)*roll_pitch_ratio;
							
					    if(Remote.yaw>1820 )
							{	//以下为遥控控制偏航角 +-号代表方向 0.75代表控制偏航角的旋转量							
								pidYaw.desired -= 0.45f;	
							}
							else if(Remote.yaw <1180)
							{
								pidYaw.desired += 0.45f;	
							}			
							//限制油门不要太过猛烈
							Remote.thr = 	LIMIT(Remote.thr,1000,2000);
				}		
		}
  }
	else //如果没有收到遥控信号，则判断为失连
	{
		cnt_15s++;
		if(cnt_15s>500)
		{
			cnt_15s = 0;
			ALL_flag.unlock = 0;  //失连锁定
			NRF24L01_init();  //初始NRF
		}
	}
	remote_unlock();
}

/*****************************************************************************************
 *  解锁判断
 * @param[in] 
 * @param[out] 
 * @return     
 ******************************************************************************************/	
void remote_unlock(void)
{
	volatile static uint8_t status=WAITING_1;
	static uint16_t cnt=0;

	if(Remote.thr<1050 &&Remote.yaw<1150)                         //油门遥杆左下角锁定
	{
		status = EXIT_255;
	}
	
	switch(status)
	{
		case WAITING_1://等待解锁
 //解锁三步奏，油门最低->油门最高->油门最低 看到LED灯不闪了 即完成解锁			
			if(Remote.thr<1100)  //第一步        
			{			 
					 status = WAITING_2;				 
			}		
			break;
		case WAITING_2://第二步 
			if(Remote.thr>1750)          
			{		
						static uint8_t cnt = 0;
					 	cnt++;		
						if(cnt>5) //最高油门需保持200ms以上，防止遥控开机初始化未完成的错误数据
						{	
								cnt=0;
								status = WAITING_3;
						}
			}			
			break;
		case WAITING_3:
			if(Remote.thr<1100)  //第三步        
			{			 
					 status = WAITING_4;				 
			}			
			break;			
		case WAITING_4:	//解锁前准备	               
				ALL_flag.unlock = 1;   //解锁标志位
				status = PROCESS_31;   //进入控制
				//MpuGetOffset();
				LED.status = AlwaysOn; //LED常亮	
	imu_rest();				
				 break;		
		case PROCESS_31:	//进入解锁状态
				if(Remote.thr<1020)    //当遥杆拉到最低 即不飞状态
				{
					if(cnt++ > 2000)                                     // 油门遥杆处于最低6S自动上锁
					{								
						status = EXIT_255;								
					}
				}
				else if(!ALL_flag.unlock)                           //其它紧急情况可直接锁定飞控，比如DIY一键降落者
				{
					status = EXIT_255;				
				}
				else					
					cnt = 0;
			break;
		case EXIT_255: //进入锁定
			LED.status = AllFlashLight;	//LED在锁定状态保持闪烁                                     //exit
			cnt = 0;
		
			LED.FlashTime = 100;  		 //LED在锁定状态保持闪烁时间间隔为300ms                                 
			ALL_flag.unlock = 0;      //锁定
			status = WAITING_1;     //返回等待解锁
			break;
		default:
			status = EXIT_255;
			break;
	}
}
/***********************END OF FILE*************************************/







