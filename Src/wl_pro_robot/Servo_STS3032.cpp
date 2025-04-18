
//STS3032 Servo synchronous control command
#include "Servo_STS3032.h"
#include <stddef.h>

//STS3032 Serial servo communication layer protocol program
SCS::SCS()
{
	Level = 1;//All instructions except broadcast instructions return responses
	Error = 0;
}
SCS::SCS(u8 End)
{
	Level = 1;
	this->End = End;
	Error = 0;
}
SCS::SCS(u8 End, u8 Level)
{
	this->Level = Level;
	this->End = End;
	Error = 0;
}
//One 16-digit number is split into two 8-digit numbers
//DataL is ata low level and DataH is ata high level
void SCS::Host2SCS(u8 *DataL, u8* DataH, u16 Data)
{
	if(End){
		*DataL = (Data>>8);
		*DataH = (Data&0xff);
	}else{
		*DataH = (Data>>8);
		*DataL = (Data&0xff);
	}
}
//Synchronous write instruction
//Servo ID[] array, IDN array length, MemAddr memory table address, write data, write length
void SCS::syncWrite(u8 ID[], u8 IDN, u8 MemAddr, u8 *nDat, u8 nLen)
{
	rFlushSCS();
	u8 mesLen = ((nLen+1)*IDN+4);
	u8 Sum = 0;
	u8 bBuf[7];
	bBuf[0] = 0xff;
	bBuf[1] = 0xff;
	bBuf[2] = 0xfe;
	bBuf[3] = mesLen;
	bBuf[4] = INST_SYNC_WRITE;
	bBuf[5] = MemAddr;
	bBuf[6] = nLen;
	writeSCS(bBuf, 7);

	Sum = 0xfe + mesLen + INST_SYNC_WRITE + MemAddr + nLen;
	u8 i, j;
	for(i=0; i<IDN; i++){
		writeSCS(ID[i]);
		writeSCS(nDat+i*nLen, nLen);
		Sum += ID[i];
		for(j=0; j<nLen; j++){
			Sum += nDat[i*nLen+j];
		}
	}
	writeSCS(~Sum);
	wFlushSCS();
}

//STS3032 Serial servo hardware interface layer program
SCSerial::SCSerial()
{
	IOTimeOut = 10;
	pSerial = NULL;
}
SCSerial::SCSerial(u8 End):SCS(End)
{
	IOTimeOut = 10;
	pSerial = NULL;
}
SCSerial::SCSerial(u8 End, u8 Level):SCS(End, Level)
{
	IOTimeOut = 10;
	pSerial = NULL;
}
int SCSerial::writeSCS(unsigned char *nDat, int nLen)
{
	if(nDat==NULL){
		return 0;
	}
	return pSerial->write(nDat, nLen);
}
int SCSerial::writeSCS(unsigned char bDat)
{
	return pSerial->write(&bDat, 1);
}
void SCSerial::rFlushSCS()
{
	while(pSerial->read()!=-1);
}
void SCSerial::wFlushSCS()
{
}

//Application layer program of SMS_STS series serial servo
void SMS_STS::SyncWritePosEx(u8 ID[], u8 IDN, s16 Position[], u16 Speed[], u8 ACC[])
{
    u8 offbuf[7*IDN];
    for(u8 i = 0; i<IDN; i++){
		if(Position[i]<0){
			Position[i] = -Position[i];
			Position[i] |= (1<<15);
		}
		u16 V;
		if(Speed){
			V = Speed[i];
		}else{
			V = 0;
		}
		if(ACC){
			offbuf[i*7] = ACC[i];
		}else{
			offbuf[i*7] = 0;
		}
        Host2SCS(offbuf+i*7+1, offbuf+i*7+2, Position[i]);
        Host2SCS(offbuf+i*7+3, offbuf+i*7+4, 0);
        Host2SCS(offbuf+i*7+5, offbuf+i*7+6, V);
    }
    syncWrite(ID, IDN, SMS_STS_ACC, offbuf, 7);
}

