/*
 * SMS_STS.h
 * application layer for waveshare ST servos.
 * date: 2023.6.11 
 */

#ifndef _SMS_STS_H
#define _SMS_STS_H

//memory table definition
//-------EPROM(read only)--------
#define SMS_STS_MODEL_L 3
#define SMS_STS_MODEL_H 4

//-------EPROM(read & write)--------
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

//-------SRAM(read & write)--------
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

//-------SRAM(read only)--------
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
	virtual int WritePosEx(u8 ID, s16 Position, u16 Speed, u8 ACC = 0);//general write for single servo
	virtual int RegWritePosEx(u8 ID, s16 Position, u16 Speed, u8 ACC = 0);//position write asynchronously for single servo(call RegWriteAction to action)
	virtual void SyncWritePosEx(u8 ID[], u8 IDN, s16 Position[], u16 Speed[], u8 ACC[]);//write synchronously for multi servos
	virtual int WheelMode(u8 ID);//speed loop mode
	virtual int WriteSpe(u8 ID, s16 Speed, u8 ACC = 0);//speed loop mode ctrl command
	virtual int EnableTorque(u8 ID, u8 Enable);//torque ctrl command
	virtual int unLockEprom(u8 ID);//eprom unlock
	virtual int LockEprom(u8 ID);//eprom locked
	virtual int CalibrationOfs(u8 ID);//set middle position
	virtual int FeedBack(int ID);//servo information feedback
	virtual int ReadPos(int ID);//read position
	virtual int ReadSpeed(int ID);//read speed
	virtual int ReadLoad(int ID);//read motor load(0~1000, 1000 = 100% max load)
	virtual int ReadVoltage(int ID);//read voltage
	virtual int ReadTemper(int ID);//read temperature
	virtual int ReadMove(int ID);//read move mode
	virtual int ReadCurrent(int ID);//read current
	virtual int ReadMode(int ID);//read working mode
private:
	u8 Mem[SMS_STS_PRESENT_CURRENT_H-SMS_STS_PRESENT_POSITION_L+1];
};

#endif