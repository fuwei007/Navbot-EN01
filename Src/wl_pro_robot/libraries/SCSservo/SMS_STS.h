/*
 * SMS_STS.h
 * 飞特SMS/STS系列串行舵机应用层程序
 * 日期: 2021.3.11
 * 作者: 
 */

#ifndef _SMS_STS_H
#define _SMS_STS_H

//内存表定义
//-------EPROM(只读)--------
#define SMS_STS_MODEL_L 3
#define SMS_STS_MODEL_H 4

//-------EPROM(读写)--------
#define SMS_STS_ID 5
#define SMS_STS_BAUD_RATE 6
#define SMS_STS_MIN_ANGLE_LIMIT_L 9
#define SMS_STS_MIN_ANGLE_LIMIT_H 10
#define SMS_STS_MAX_ANGLE_LIMIT_L 11
#define SMS_STS_MAX_ANGLE_LIMIT_H 12
#define SMS_STS_CW_DEAD 26
#define SMS_STS_CCW_DEAD 27
#define SMS_STS_OFS_L 31
#define SMS_STS_OFS_H 32
#define SMS_STS_MODE 33

//-------SRAM(读写)--------
#define SMS_STS_TORQUE_ENABLE 40
#define SMS_STS_ACC 41
#define SMS_STS_GOAL_POSITION_L 42
#define SMS_STS_GOAL_POSITION_H 43
#define SMS_STS_GOAL_TIME_L 44
#define SMS_STS_GOAL_TIME_H 45
#define SMS_STS_GOAL_SPEED_L 46
#define SMS_STS_GOAL_SPEED_H 47
#define SMS_STS_TORQUE_LIMIT_L 48
#define SMS_STS_TORQUE_LIMIT_H 49
#define SMS_STS_LOCK 55

//-------SRAM(只读)--------
#define SMS_STS_PRESENT_POSITION_L 56
#define SMS_STS_PRESENT_POSITION_H 57
#define SMS_STS_PRESENT_SPEED_L 58
#define SMS_STS_PRESENT_SPEED_H 59
#define SMS_STS_PRESENT_LOAD_L 60
#define SMS_STS_PRESENT_LOAD_H 61
#define SMS_STS_PRESENT_VOLTAGE 62
#define SMS_STS_PRESENT_TEMPERATURE 63
#define SMS_STS_MOVING 66
#define SMS_STS_PRESENT_CURRENT_L 69
#define SMS_STS_PRESENT_CURRENT_H 70

#include "SCSerial.h"

class SMS_STS : public SCSerial
{
public:
	SMS_STS();
	SMS_STS(u8 End);
	SMS_STS(u8 End, u8 Level);
	virtual int WritePosEx(u8 ID, s16 Position, u16 Speed, u8 ACC = 0);//普通写单个舵机位置指令
	virtual int RegWritePosEx(u8 ID, s16 Position, u16 Speed, u8 ACC = 0);//异步写单个舵机位置指令(RegWriteAction生效)
	virtual void SyncWritePosEx(u8 ID[], u8 IDN, s16 Position[], u16 Speed[], u8 ACC[]);//同步写多个舵机位置指令
	virtual int WheelMode(u8 ID);//恒速模式
	virtual int WriteSpe(u8 ID, s16 Speed, u8 ACC = 0);//恒速模式控制指令
	virtual int EnableTorque(u8 ID, u8 Enable);//扭力控制指令
	virtual int unLockEprom(u8 ID);//eprom解锁
	virtual int LockEprom(u8 ID);//eprom加锁
	virtual int CalibrationOfs(u8 ID);//中位校准
	virtual int FeedBack(int ID);//反馈舵机信息
	virtual int ReadPos(int ID);//读位置
	virtual int ReadSpeed(int ID);//读速度
	virtual int ReadLoad(int ID);//读输出至电机的电压百分比(0~1000)
	virtual int ReadVoltage(int ID);//读电压
	virtual int ReadTemper(int ID);//读温度
	virtual int ReadMove(int ID);//读移动状态
	virtual int ReadCurrent(int ID);//读电流
	virtual int ReadMode(int ID);
private:
	u8 Mem[SMS_STS_PRESENT_CURRENT_H-SMS_STS_PRESENT_POSITION_L+1];
};

#endif