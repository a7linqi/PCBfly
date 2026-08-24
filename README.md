# PCBfly 四轴飞行器

基于 STM32F103 的微型四轴飞行器飞控项目。

## 硬件平台
- MCU: STM32F103C8T6
- 传感器: MPU6050 (I2C)
- 通信: NRF24L01 (SPI, 2.4G)
- 电机: 空心杯电机 × 4
- 调试: USB HID + 匿名上位机

## 软件架构
- 姿态解算: 四元数 + Mahony 互补滤波
- 控制算法: 串级 PID (角度环 + 角速度环)
- 电机混控: X 型布局四电机混合控制
- 通信协议: 匿名上位机协议

## 核心文件
- `USER/main.c` - 主程序入口
- `USER/INIT.c` - 系统初始化 + PID 参数
- `HAL/control.c` - 飞行控制 + 电机混控
- `HAL/remote.c` - 遥控数据解析
- `MATH/imu.c` - 姿态解算
- `MATH/pid.c` - PID 控制器

## 开发环境
- Keil MDK-5
- STM32F10x 标准外设库

## 学习笔记
- 姿态解算原理
- PID 调参方法
- 四轴飞行器控制原理
