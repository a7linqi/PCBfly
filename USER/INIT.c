/**************************************************************
 * 
 * @brief
   ZIN-7套件
	 飞控爱好群551883670
	 淘宝地址：https://shop297229812.taobao.com/shop/view_shop.htm?mytmenu=mdianpu&user_number_id=2419305772
 ***************************************************************/
#include "ALL_DEFINE.h"


volatile uint32_t SysTick_count; //系统时间计数
_st_Mpu MPU6050;   //MPU6050原始数据
_st_AngE Angle;    //当前角度姿态值
_st_Remote Remote; //遥控通道值


_st_ALL_flag ALL_flag; //系统标志位，包含解锁标志位等


PidObject pidRateX; //内环PID数据
PidObject pidRateY;
PidObject pidRateZ;

PidObject pidPitch; //外环PID数据
PidObject pidRoll;
PidObject pidYaw;

void pid_param_Init(void); //PID控制参数初始化，改写PID并不会保存数据，请调试完成后直接在程序里更改 再烧录到飞控


/**************************************************************
 *  整个系统外设和传感器初始化
 * @param[in] 
 * @param[out] 
 * @return     
 ***************************************************************/
void ALL_Init(void)
{
	USB_HID_Init();
	pid_param_Init();       //PID参数初始化
	 
	

	delay_ms(100);

	LEDInit();              //LED闪灯初始化
	
	IIC_Init();             //I2C初始化		
//----------------------------------------	
// 水平静止标定，该功能只需要进行一次，不要每次进行。店家发货前已经进行一次标定了，标定完后会自动保存到MCU的FLASH中。
// 如需校准，重新打开即可，延时5S是为了插上电池后有充足的时间将飞行器放在地上进行水平静止标定。
//----------------------------------------		
//	USART1_Config();  //备用串口     
	
	MpuInit();              //MPU6050初始化

	NRF24L01_init();				//2.4G遥控通信初始化

	TIM2_PWM_Config();			//4路PWM初始化

	TIM3_Config();					//系统工作周期初始化 
	
}


/**************************************************************
 *  初始化PID参数
 * @brief 如果需要自己修改PID值，调这里就可以了
 * @param[out] 
 * @return     
 ***************************************************************/
void pid_param_Init(void)
{

//	pidRateX.kp = 1.5f;//huanbaoxian
//	pidRateY.kp = 1.5f;
//	pidRateZ.kp = 6.0f;
//	
//	pidRateX.ki = 0.04f;
//	pidRateY.ki = 0.04f;
//	pidRateZ.ki = 0.05f;	
//	
//	pidRateX.kd = 0.06f;
//	pidRateY.kd = 0.06f;
//	pidRateZ.kd = 0.4f;	
//	
//	pidPitch.kp = 4.0f;
//	pidRoll.kp = 4.0f;
//	pidYaw.kp = 3.0f;
	
//	pidRateX.kp = 2.0f;//1
//	pidRateY.kp = 2.0f;
//	pidRateZ.kp = 6.0f;
//	
//	pidRateX.ki = 0.05f;
//	pidRateY.ki = 0.05f;
//	pidRateZ.ki = 0.05f;	
//	
//	pidRateX.kd = 0.08f;
//	pidRateY.kd = 0.08f;
//	pidRateZ.kd = 0.5f;	
//	
//	pidPitch.kp = 7.0f;
//	pidRoll.kp = 7.0f;
//	pidYaw.kp = 4.0f;
	
	pidRateX.kp = 2.0f;//1
	pidRateY.kp = 2.0f;
	pidRateZ.kp = 6.0f;
	
	pidRateX.ki = 0.01f;
	pidRateY.ki = 0.01f;
	pidRateZ.ki = 0.05f;	
	
	pidRateX.kd = 0.08f;
	pidRateY.kd = 0.08f;
	pidRateZ.kd = 0.5f;	
	
	pidPitch.kp = 7.0f;
	pidRoll.kp = 7.0f;
	pidYaw.kp = 4.0f;		
}















