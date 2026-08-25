#ifndef _ALL_USER_DATA_H_

#define _ALL_USER_DATA_H_



typedef   signed          char int8_t;

typedef   signed short     int int16_t;

typedef   signed           int int32_t;

typedef   signed       long long int64_t;



    /* exact-width unsigned integer types */

typedef unsigned          char uint8_t;

typedef unsigned short     int uint16_t;

typedef unsigned           int uint32_t;

typedef unsigned       long long uint64_t;





#define NULL 0

extern volatile uint32_t SysTick_count;





typedef struct{

	int16_t accX;

	int16_t accY;

	int16_t accZ;

	int16_t gyroX;

	int16_t gyroY;

	int16_t gyroZ;

}_st_Mpu;





// 姿态角度结构体（由姿态解算得到）
typedef struct{
	float roll;    // 横滚角（左右倾斜角度，单位：度）
	float pitch;   // 俯仰角（前后低头角度，单位：度）
	float yaw;     // 偏航角（左右转头角度，单位：度）
}_st_AngE;







// 遥控器通道数据结构体
// 各通道值范围：1000~2000（中值1500）
typedef struct
{
	uint16_t roll;    // 横滚通道（左右倾斜，控制左右飞）
	uint16_t pitch;   // 俯仰通道（前后倾斜，控制前后飞）
	uint16_t thr;     // 油门通道（上下推拉，控制上升下降）
	uint16_t yaw;     // 偏航通道（左右旋转，控制转向）
	uint16_t AUX1;    // 辅助通道1（可自定义功能）
	uint16_t AUX2;    // 辅助通道2
	uint16_t AUX3;    // 辅助通道3
	uint16_t AUX4;    // 辅助通道4
}_st_Remote;







typedef volatile struct

{

	float desired;           // 目标值（遥控器打杆期望达到的角度或角速度）

	float offset;            // 偏移量（对目标值的微调补偿）

	float prevError;         // 上次计算的误差（用于微分项）

	float integ;             // 积分累计值（误差随时间的累加，用于消除稳态误差）

	float kp;                // 比例增益系数（P项，越大响应越快，但可能振荡）

	float ki;                // 积分增益系数（I项，消除稳态误差，但会导致超调）

	float kd;                // 微分增益系数（D项，抑制振荡，起阻尼作用）

	float IntegLimitHigh;    // 积分上限（防止积分饱和）

	float IntegLimitLow;     // 积分下限（防止积分饱和）

	float measured;          // 测量值（传感器实际读到的角度或角速度）

	float out;               // PID计算输出值（最终给电机的控制量）

	float OutLimitHigh;      // 输出上限（限制最大控制量）

	float OutLimitLow;       // 输出下限（限制最小控制量）

}PidObject;





typedef volatile struct

{

	uint8_t unlock:1;		//位域声明（1bit：0或我）

}_st_ALL_flag;





extern _st_Remote Remote;

extern _st_Mpu MPU6050;

extern _st_AngE Angle;





extern _st_ALL_flag ALL_flag;





extern	PidObject pidRateX;

extern	PidObject pidRateY;

extern	PidObject pidRateZ;



extern	PidObject pidPitch;

extern	PidObject pidRoll;

extern	PidObject pidYaw;



#endif

