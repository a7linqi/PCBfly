/**************************************************************
 *

 ***************************************************************/
#include "ALL_DATA.h"
#include "ALL_DEFINE.h"
#include "control.h"
#include "pid.h"
#include "math.h"

//------------------------------------------------------------------------------
#undef NULL
#define NULL 0
#undef DISABLE
#define DISABLE 0
#undef ENABLE
#define ENABLE 1
#undef REST
#define REST 0
#undef SET
#define SET 1
#undef EMERGENT
#define EMERGENT 0
//------------------------------------------------------------------------------

// PID对象指针数组，将6个PID结构体的地址存入数组
// 这样就可以用循环统一操作所有PID（如复位），而不需要逐个调用
// 复位时调用 pidRest(pPidObject, 6) 即可清零所有PID数据
PidObject *(pPidObject[])={&pidRateX,&pidRateY,&pidRateZ,&pidRoll,&pidPitch,&pidYaw};


/**************************************************************
 * 飞行PID控制函数
 * 功能：读取传感器数据，计算串级PID输出，控制飞机姿态
 * @param[in] dt  时间步长，单位秒
 * @return     无
 ***************************************************************/
void FlightPidControl(float dt)
{
	// 状态机：WAITING_1 -> READY_11 -> PROCESS_31 -> EXIT_255
	volatile static uint8_t status=WAITING_1;

	switch(status)
	{
		case WAITING_1: // 等待解锁
			if(ALL_flag.unlock)
			{
				status = READY_11;
			}
			break;
		case READY_11:  // 准备进入控制
			pidRest(pPidObject,6); // 复位所有PID数据，防止上次残留数据影响本次控制
			status = PROCESS_31;
			break;
		case PROCESS_31: // 正式飞行控制
			// 侧翻保护：角度超过50度判定为异常，触发紧急停机
			if(Angle.pitch<-50||Angle.pitch>50||Angle.roll<-50||Angle.roll>50)
				ALL_flag.unlock = EMERGENT;

			// ========== 读取传感器数据 ==========
			// 内环（角速度环）：读取陀螺仪数据，单位 度/秒
			pidRateX.measured = MPU6050.gyroX * Gyro_G;
			pidRateY.measured = MPU6050.gyroY * Gyro_G;
			pidRateZ.measured = MPU6050.gyroZ * Gyro_G;

			// 外环（角度环）：读取姿态解算后的角度，单位 度
			pidPitch.measured = Angle.pitch;
			pidRoll.measured = Angle.roll;
			pidYaw.measured = Angle.yaw;

			// ========== 串级PID控制 ==========
			// Roll轴（横滚）：外环PID -> 内环PID
			pidUpdate(&pidRoll,dt);          // 计算外环（角度环）PID
			pidRateX.desired = pidRoll.out;  // 外环输出作为内环的目标值
			pidUpdate(&pidRateX,dt);         // 计算内环（角速度环）PID

			// Pitch轴（俯仰）：外环PID -> 内环PID
			pidUpdate(&pidPitch,dt);         // 计算外环（角度环）PID
			pidRateY.desired = pidPitch.out; // 外环输出作为内环的目标值
			pidUpdate(&pidRateY,dt);         // 计算内环（角速度环）PID

			// Yaw轴（偏航）：直接调用串级PID函数
			CascadePID(&pidRateZ,&pidYaw,dt);
			break;
		case EXIT_255:  // 退出控制
			pidRest(pPidObject,6);    // 复位所有PID
			status = WAITING_1;       // 回到等待状态
			break;
		default:
			status = EXIT_255;
			break;
	}
	// 紧急停机保护：任何时候检测到紧急标志，立即退出控制
	if(ALL_flag.unlock == EMERGENT)
		status = EXIT_255;
}

/**************************************************************
 * 电机控制函数
 * 功能：根据油门和PID输出计算四个电机的PWM值
 * @param[in]  无
 * @param[out] 无
 * @return     无
 ***************************************************************/
int16_t motor[4];
#define MOTOR1 motor[0]  // 电机1（右上）
#define MOTOR2 motor[1]  // 电机2（左上）
#define MOTOR3 motor[2]  // 电机3（右下）
#define MOTOR4 motor[3]  // 电机4（左下）

//   PWM2     前       PWM1
//      *           *
//      	*       *
//    		  *   *
//      			*
//    		  *   *
//      	*       *
//      *           *
//    PWM4           PWM3

uint16_t low_thr_cnt;  // 低油门计数器，用于检测油门杆是否归零
void MotorControl(void)
{
	// 状态机：WAITING_1 -> WAITING_2 -> PROCESS_31 -> EXIT_255
	volatile static uint8_t status=WAITING_1;

	// 紧急停机保护：任何时候检测到紧急标志，立即停止电机
	if(ALL_flag.unlock == EMERGENT)
		status = EXIT_255;

	switch(status)
	{
		case WAITING_1: // 等待解锁
			MOTOR1 = MOTOR2 = MOTOR3 = MOTOR4 = 0;  // 上电后电机全部关闭
			if(ALL_flag.unlock)
			{
				status = WAITING_2;
			}
		case WAITING_2: // 解锁后，等待油门杆推高
			if(Remote.thr>1100)
			{
				low_thr_cnt=0;
				// 清零偏航角和yaw的PID数据，防止解锁瞬间乱转
				Angle.yaw = pidYaw.desired = pidYaw.measured = 0;
				pidRest(pPidObject,6);
				status = PROCESS_31;
			}
			break;
		case PROCESS_31: // 正式电机输出
			{
				int16_t temp,thr;
				temp = Remote.thr - 1000; // 油门值减去基准值（1000~2000 -> 0~1000）

				thr = temp;

				// 低油门检测：油门杆推到最低超过1.5秒，自动上锁
				if(temp<10)
				{
					low_thr_cnt++;
					if(low_thr_cnt>300) // 300次 * 5ms = 1500ms
					{
						thr = 0;
						pidRest(pPidObject,6);
						MOTOR1 = MOTOR2 = MOTOR3 = MOTOR4 = 0;
						status = WAITING_2;
						break;
					}
				}
				else low_thr_cnt=0;

				// 基础油门：四个电机相同的推力（限幅0~800，留200给姿态控制）
				MOTOR1 = MOTOR2 = MOTOR3 = MOTOR4 = LIMIT(temp,0,800);

				// 叠加姿态控制量：PID输出补偿到各电机
				// X轴正方向 = Roll右倾，Y轴正方向 = Pitch前倾，Z轴正方向 = Yaw顺时针
				MOTOR1 +=    + pidRateX.out + pidRateY.out + pidRateZ.out; // 右上
				MOTOR2 +=    - pidRateX.out + pidRateY.out - pidRateZ.out; // 左上
				MOTOR3 +=    + pidRateX.out - pidRateY.out - pidRateZ.out; // 右下
				MOTOR4 +=    - pidRateX.out - pidRateY.out + pidRateZ.out; // 左下
			}
			break;
		case EXIT_255:
			MOTOR1 = MOTOR2 = MOTOR3 = MOTOR4 = 0;  // 停止所有电机
			status = WAITING_1;
			break;
		default:
			break;
	}

	// 将电机值写入定时器PWM寄存器，限幅0~1000
	TIM2->CCR1 = LIMIT(MOTOR1,0,1000);
	TIM2->CCR2 = LIMIT(MOTOR2,0,1000);
	TIM2->CCR3 = LIMIT(MOTOR3,0,1000);
	TIM2->CCR4 = LIMIT(MOTOR4,0,1000);
}
/************************************END OF FILE********************************************/
