/*
 * SCSCL.h
 * 飞特SCSCL系列串行舵机应用层程序
 * 日期: 2021.3.11
 * 作者: 
 */

#ifndef _SCSCL_H
#define _SCSCL_H

//内存表定义
//-------EPROM(只读)--------
#define SCSCL_VERSION_L 3
#define SCSCL_VERSION_H 4

//-------EPROM(读写)--------
#define SCSCL_ID 5
#define SCSCL_BAUD_RATE 6
#define SCSCL_MIN_ANGLE_LIMIT_L 9
#define SCSCL_MIN_ANGLE_LIMIT_H 10
#define SCSCL_MAX_ANGLE_LIMIT_L 11
#define SCSCL_MAX_ANGLE_LIMIT_H 12
#define SCSCL_CW_DEAD 26
#define SCSCL_CCW_DEAD 27

//-------SRAM(读写)--------
#define SCSCL_TORQUE_ENABLE 40
#define SCSCL_GOAL_POSITION_L 42
#define SCSCL_GOAL_POSITION_H 43
#define SCSCL_GOAL_TIME_L 44
#define SCSCL_GOAL_TIME_H 45
#define SCSCL_GOAL_SPEED_L 46
#define SCSCL_GOAL_SPEED_H 47
#define SCSCL_LOCK 48

//-------SRAM(只读)--------
#define SCSCL_PRESENT_POSITION_L 56
#define SCSCL_PRESENT_POSITION_H 57
#define SCSCL_PRESENT_SPEED_L 58
#define SCSCL_PRESENT_SPEED_H 59
#define SCSCL_PRESENT_LOAD_L 60
#define SCSCL_PRESENT_LOAD_H 61
#define SCSCL_PRESENT_VOLTAGE 62
#define SCSCL_PRESENT_TEMPERATURE 63
#define SCSCL_MOVING 66
#define SCSCL_PRESENT_CURRENT_L 69
#define SCSCL_PRESENT_CURRENT_H 70

#include "SCSerial.h"

class SCSCL : public SCSerial
{
public:
	SCSCL();
	SCSCL(u8 End);
	SCSCL(u8 End, u8 Level);
	virtual int WritePos(u8 ID, u16 Position, u16 Time, u16 Speed);//普通写单个舵机位置指令
	virtual int WritePosEx(u8 ID, s16 Position, u16 Speed, u8 ACC);//单个舵机位置指令
	virtual int RegWritePos(u8 ID, u16 Position, u16 Time, u16 Speed = 0);//异步写单个舵机位置指令(RegWriteAction生效)
	virtual void SyncWritePos(u8 ID[], u8 IDN, u16 Position[], u16 Time[], u16 Speed[]);//同步写多个舵机位置指令
	virtual int PWMMode(u8 ID);//PWM输出模式
	virtual int WritePWM(u8 ID, s16 pwmOut);//PWM输出模式指令
	virtual int EnableTorque(u8 ID, u8 Enable);//扭矩控制指令
	virtual int unLockEprom(u8 ID);//eprom解锁
	virtual int LockEprom(u8 ID);//eprom加锁
	virtual int FeedBack(int ID);//反馈舵机信息
	virtual int ReadPos(int ID);//读位置
	virtual int ReadSpeed(int ID);//读速度
	virtual int ReadLoad(int ID);//读输出至电机的电压百分比(0~1000)
	virtual int ReadVoltage(int ID);//读电压
	virtual int ReadTemper(int ID);//读温度
	virtual int ReadMove(int ID);//读移动状态
	virtual int ReadCurrent(int ID);//读电流
	virtual int ReadMode(int ID);
	virtual int CalibrationOfs(u8 ID);
	virtual int ReadInfoValue(int ID, int AddInput);
private:
	u8 Mem[SCSCL_PRESENT_CURRENT_H-SCSCL_PRESENT_POSITION_L+1];
};

#endif