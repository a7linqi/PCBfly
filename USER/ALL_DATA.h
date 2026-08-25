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





typedef struct{

	float roll;

	float pitch;

	float yaw;

}_st_AngE;







typedef struct

{

	uint16_t roll;

	uint16_t pitch;

	uint16_t thr;

	uint16_t yaw;

	uint16_t AUX1;

	uint16_t AUX2;

	uint16_t AUX3;

	uint16_t AUX4;

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

	uint8_t unlock:1;		//位域声明

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

