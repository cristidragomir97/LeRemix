/*
 * SCS.h
 * communication layer for waveshare serial bus servo
 * date: 2019.12.18 
 */

#ifndef _SCS_H
#define _SCS_H

#include "INST.h"

class SCS{
public:
	SCS();
	SCS(u8 End);
	SCS(u8 End, u8 Level);
	int genWrite(u8 ID, u8 MemAddr, u8 *nDat, u8 nLen); // general write
	int regWrite(u8 ID, u8 MemAddr, u8 *nDat, u8 nLen); // write asynchronously
	int RegWriteAction(u8 ID = 0xfe); // trigger command for regWrite()
	void syncWrite(u8 ID[], u8 IDN, u8 MemAddr, u8 *nDat, u8 nLen); // write synchronously
	int writeByte(u8 ID, u8 MemAddr, u8 bDat); // write 1 byte
	int writeWord(u8 ID, u8 MemAddr, u16 wDat); // write 2 byte
	int Read(u8 ID, u8 MemAddr, u8 *nData, u8 nLen); // read command
	int readByte(u8 ID, u8 MemAddr); // read 1 byte
	int readWord(u8 ID, u8 MemAddr); // read 2 byte
	int Ping(u8 ID); // Ping command
	int syncReadPacketTx(u8 ID[], u8 IDN, u8 MemAddr, u8 nLen); // read synchronously command send
	int syncReadPacketRx(u8 ID, u8 *nDat); // read synchronously command receive, return the number of byte when succeed, return 0 when failed
	int syncReadRxPacketToByte(); // decode one byte
	int syncReadRxPacketToWrod(u8 negBit=0); // decode 2 byte, negBit is the direction, 0 as none.
public:
	u8 Level; // the level of the servo return
	u8 End; // processor endian structure
	u8 Error; // the status of servo
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
	void Host2SCS(u8 *DataL, u8* DataH, u16 Data); // one 16-digit number split into two 8-digit numbers
	u16	SCS2Host(u8 DataL, u8 DataH); // combination of two 8-digit numbers into one 16-digit number
	int	Ack(u8 ID); // return response
	int checkHead(); // Frame header detection
};
#endif
