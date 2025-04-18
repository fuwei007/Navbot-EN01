/*
 * SCSerial.h
 * 飞特串行舵机硬件接口层程序
 * 日期: 2019.4.27
 * 作者: 
 */

#ifndef _SCSERIAL_H
#define _SCSERIAL_H

#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#include "SCS.h"

class SCSerial : public SCS
{
public:
	SCSerial();
	SCSerial(u8 End);
	SCSerial(u8 End, u8 Level);

protected:
	virtual int writeSCS(unsigned char *nDat, int nLen);//输出nLen字节
	virtual int readSCS(unsigned char *nDat, int nLen);//输入nLen字节
	virtual int writeSCS(unsigned char bDat);//输出1字节
	virtual void rFlushSCS();//
	virtual void wFlushSCS();//
public:
	unsigned long int IOTimeOut;//输入输出超时
	HardwareSerial *pSerial;//串口指针
	int Err;
public:
	virtual int getErr(){  return Err;  }
};

#endif