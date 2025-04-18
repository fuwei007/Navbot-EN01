/*
 * SCS.h
 * 飞特串行舵机通信层协议程序
 * 日期: 2019.12.18
 * 作者: 
 */

#ifndef _SCS_H
#define _SCS_H

#include "INST.h"

class SCS{
public:
	SCS();
	SCS(u8 End);
	SCS(u8 End, u8 Level);
	int genWrite(u8 ID, u8 MemAddr, u8 *nDat, u8 nLen);//普通写指令
	int regWrite(u8 ID, u8 MemAddr, u8 *nDat, u8 nLen);//异步写指令
	int RegWriteAction(u8 ID = 0xfe);//异步写执行指令
	void syncWrite(u8 ID[], u8 IDN, u8 MemAddr, u8 *nDat, u8 nLen);//同步写指令
	int writeByte(u8 ID, u8 MemAddr, u8 bDat);//写1个字节
	int writeWord(u8 ID, u8 MemAddr, u16 wDat);//写2个字节
	int Read(u8 ID, u8 MemAddr, u8 *nData, u8 nLen);//读指令
	int readByte(u8 ID, u8 MemAddr);//读1个字节
	int readWord(u8 ID, u8 MemAddr);//读2个字节
	int Ping(u8 ID);//Ping指令
	int syncReadPacketTx(u8 ID[], u8 IDN, u8 MemAddr, u8 nLen);//同步读指令包发送
	int syncReadPacketRx(u8 ID, u8 *nDat);//同步读返回包接收，成功返回内存字节数，失败返回0
	int syncReadRxPacketToByte();//解码一个字节
	int syncReadRxPacketToWrod(u8 negBit=0);//解码两个字节，negBit为方向为，negBit=0表示无方向
public:
	u8 Level;//舵机返回等级
	u8 End;//处理器大小端结构
	u8 Error;//舵机状态
	u8 syncReadRxPacketIndex;
	u8 syncReadRxPacketLen;
	u8 *syncReadRxPacket;
protected:
	virtual int writeSCS(unsigned char *nDat, int nLen) = 0;
	virtual int readSCS(unsigned char *nDat, int nLen) = 0;
	virtual int writeSCS(unsigned char bDat) = 0;
	virtual void rFlushSCS() = 0;
	virtual void wFlushSCS() = 0;
protected:
	void writeBuf(u8 ID, u8 MemAddr, u8 *nDat, u8 nLen, u8 Fun);
	void Host2SCS(u8 *DataL, u8* DataH, u16 Data);//1个16位数拆分为2个8位数
	u16	SCS2Host(u8 DataL, u8 DataH);//2个8位数组合为1个16位数
	int	Ack(u8 ID);//返回应答
	int checkHead();//帧头检测
};
#endif
