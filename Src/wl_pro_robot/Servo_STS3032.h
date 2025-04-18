//STS3032 Servo synchronous control command
//The calling method is: sms_sts.SyncWritePosEx(ID, 2, Position, Speed, ACC);

#ifndef _SERVO_STS3032_H
#define _SERVO_STS3032_H

	#if defined(ARDUINO) && ARDUINO >= 100
	#include "Arduino.h"
	#else
	#include "WProgram.h"
	#endif

//Data structure definition
typedef	char s8;
typedef	unsigned char u8;	
typedef	unsigned short u16;	
typedef	short s16;
typedef	unsigned long u32;	
typedef	long s32;

//Memory table definition
#define SMS_STS_ACC 41
#define INST_SYNC_WRITE 0x83

//STS3032 Serial servo communication layer protocol program
class SCS{
public:
	SCS();
	SCS(u8 End);
	SCS(u8 End, u8 Level);
	void syncWrite(u8 ID[], u8 IDN, u8 MemAddr, u8 *nDat, u8 nLen);//Synchronous write instruction
public:
	u8 Level;//Servo return level
	u8 End;//The big-endian structure of the processor
	u8 Error;//Steering gear status
	u8 syncReadRxPacketIndex;
	u8 syncReadRxPacketLen;
	u8 *syncReadRxPacket;
	u8 *syncReadRxBuff;
	u16 syncReadRxBuffLen;
	u16 syncReadRxBuffMax;
	u32 syncTimeOut;
protected:
	virtual int writeSCS(unsigned char *nDat, int nLen) = 0;
	virtual int writeSCS(unsigned char bDat) = 0;
	virtual void rFlushSCS() = 0;
	virtual void wFlushSCS() = 0;
protected:
	void Host2SCS(u8 *DataL, u8* DataH, u16 Data);//One 16-digit number is split into two 8-digit numbers
	int	Ack(u8 ID);//Return response
	int checkHead();//Frame header detection
};

//STS3032 Serial servo hardware interface layer program
class SCSerial : public SCS
{
public:
	SCSerial();
	SCSerial(u8 End);
	SCSerial(u8 End, u8 Level);
protected:
	int writeSCS(unsigned char *nDat, int nLen);// Output the nLen byte
	int writeSCS(unsigned char bDat);//Output 1 byte
	void rFlushSCS();//
	void wFlushSCS();//
public:
	unsigned long IOTimeOut;//Input/output timeout
	HardwareSerial *pSerial;//Serial port pointer
	int Err;
public:
	virtual int getErr(){  return Err;  }
};

//SMS/STS Series serial servo application layer program
class SMS_STS : public SCSerial
{
public:
	virtual void SyncWritePosEx(u8 ID[], u8 IDN, s16 Position[], u16 Speed[], u8 ACC[]);//Write multiple servo position instructions simultaneously
};

#endif
