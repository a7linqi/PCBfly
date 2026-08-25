/*******************************************************************
 *@title PID 控制函数
 *@brief 本文件包含飞控系统中的PID初始化、PID计算等核心函数
 *@brief 历史修改数据：
 ******************************************************************/
#include "pid.h"
#include "myMath.h"

/**************************************************************
 *函数说明：复位PID控制器
 *功能：将PID控制器的积分项、上次误差、输出值和偏移量清零
 * @param[in] pid  PID对象指针数组
 * @param[in] len  PID对象数组长度
 * @return     无
 ***************************************************************/
void pidRest(PidObject **pid,const uint8_t len)
{
	uint8_t i;
	for(i=0;i<len;i++)
	{
	  	pid[i]->integ = 0;        // 清零积分累计值
	    pid[i]->prevError = 0;    // 清零上次误差
	    pid[i]->out = 0;          // 清零输出值
		pid[i]->offset = 0;       // 清零偏移量
	}
}

/**************************************************************
 *函数说明：PID控制器核心计算函数
 *功能：根据目标值和测量值计算PID输出
 *计算公式：out = Kp*error + Ki*∫error*dt + Kd*(error-prevError)/dt
 *
 * @param[in] pid         PID对象指针，包含Kp/Ki/Kd等参数
 * @param[in] dt          时间步长（采样周期），单位秒
 * @return     无（结果存入pid->out）
 ***************************************************************/
void pidUpdate(PidObject* pid,const float dt)
{
	 float error;   // 当前误差
	 float deriv;   // 误差微分（变化率）

    // 计算当前误差 = 目标值 - 测量值 + 偏移量
    error = pid->desired - pid->measured + pid->offset;

    // 积分项累加：误差 × 时间步长
    pid->integ += error * dt;

	//  pid->integ = LIMIT(pid->integ,pid->IntegLimitLow,pid->IntegLimitHigh); //积分项限幅（已注释）

    // 微分项计算：(当前误差 - 上次误差) / 时间步长
    deriv = (error - pid->prevError)/dt;

    // PID输出 = 比例项 + 积分项 + 微分项
    pid->out = pid->kp * error + pid->ki * pid->integ + pid->kd * deriv;

	//pid->out = LIMIT(pid->out,pid->OutLimitLow,pid->OutLimitHigh); //输出限幅（已注释）

    pid->prevError = error;  // 保存当前误差，供下次微分计算使用

}

/**************************************************************
 *函数说明：串级PID控制器
 *功能：先计算外环（角度环），再将外环输出作为内环（角速度环）的目标值
 *控制流程：角度环PID -> 角速度环PID -> 输出
 *
 * @param[in] pidRate  内环PID对象指针（角速度环）
 * @param[in] pidAngE  外环PID对象指针（角度环）
 * @param[in] dt       时间步长，单位秒
 * @return     无
 ***************************************************************/
void CascadePID(PidObject* pidRate,PidObject* pidAngE,const float dt)  //串级PID
{
	pidUpdate(pidAngE,dt);          // 先计算外环（角度环）
	pidRate->desired = pidAngE->out; // 外环输出作为内环的目标值
	pidUpdate(pidRate,dt);          // 再计算内环（角速度环）
}

/*******************************END*********************************/
