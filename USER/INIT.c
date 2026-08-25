/**************************************************************
 *
 * @brief
   ZIN-7 四轴飞控
   飞控交流群551883670
   淘宝地址：https://shop297229812.taobao.com/shop/view_shop.htm?mytmenu=mdianpu&user_number_id=2419305772
 ***************************************************************/
#include "ALL_DEFINE.h"


volatile uint32_t SysTick_count;  // 系统时钟计数（毫秒级，用于计时）
_st_Mpu MPU6050;                  // MPU6050传感器原始数据（陀螺仪+加速度计）
_st_AngE Angle;                   // 姿态解算后的角度（pitch俯仰、roll横滚、yaw偏航）
_st_Remote Remote;                // 遥控器通道值（thr油门、pitch、roll、yaw）


_st_ALL_flag ALL_flag;            // 系统标志位（unlock解锁状态等）


PidObject pidRateX;  // X轴角速度环PID（内环）
PidObject pidRateY;  // Y轴角速度环PID（内环）
PidObject pidRateZ;  // Z轴角速度环PID（内环）

PidObject pidPitch;  // 俯仰角角度环PID（外环）
PidObject pidRoll;   // 横滚角角度环PID（外环）
PidObject pidYaw;    // 偏航角角度环PID（外环）

void pid_param_Init(void); // PID控制参数初始化函数（修改PID参数后会保存到Flash，断电后不丢失，无需每次上电重新调参）


/**************************************************************
 *  整个系统所有外设初始化
 * @param[in]
 * @param[out]
 * @return
 ***************************************************************/
void ALL_Init(void)
{
	USB_HID_Init();
	pid_param_Init();       // PID参数初始化


	delay_ms(100);

	LEDInit();              // LED灯初始化

	IIC_Init();             // I2C初始化
//----------------------------------------
// 水平校准宏定义，这个函数只需要烧录一次，不需要每次调用，而且你之前已经调用一次了，校准数据会自动保存到MCU的FLASH中
// 如果校准过，下次打开即可，开机时5S内如果不推油门则会将当前姿态作为水平基准进行校准
//----------------------------------------
//	USART1_Config();  //串口配置

	MpuInit();              // MPU6050初始化

	NRF24L01_init();        // 2.4G遥控通信初始化

	TIM2_PWM_Config();      // 4路PWM初始化（控制四个电机）

	TIM3_Config();          // 系统定时器初始化

}


/**************************************************************
 *  初始化PID参数
 * @brief 主要调这里改PID值，改完就可以了
 * @param[out]
 * @return
 ***************************************************************/
void pid_param_Init(void)
{
//	pidRateX.kp = 1.5f;//环保线
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

	// 角速度环PID（内环）- 控制角速度，响应快
	pidRateX.kp = 2.0f;
	pidRateY.kp = 2.0f;
	pidRateZ.kp = 6.0f;

	pidRateX.ki = 0.01f;
	pidRateY.ki = 0.01f;
	pidRateZ.ki = 0.05f;

	pidRateX.kd = 0.08f;
	pidRateY.kd = 0.08f;
	pidRateZ.kd = 0.5f;

	// 角度环PID（外环）- 控制角度，保证稳定
	pidPitch.kp = 7.0f;
	pidRoll.kp = 7.0f;
	pidYaw.kp = 4.0f;
}
