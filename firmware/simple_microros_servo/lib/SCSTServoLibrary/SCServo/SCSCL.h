/*
 * SCSCL.h
 * application layer for waveshare serial bus servo
 * date: 2023.6.23 
 */

#ifndef _SCSCL_H
#define _SCSCL_H

//memory table definition
//-------EPROM(read only)--------
#define SCSCL_VERSION_L 3
#define SCSCL_VERSION_H 4

//-------EPROM(read & write)--------
#define SCSCL_ID 5
#define SCSCL_BAUD_RATE 6
#define SCSCL_MIN_ANGLE_LIMIT_L 9
#define SCSCL_MIN_ANGLE_LIMIT_H 10
#define SCSCL_MAX_ANGLE_LIMIT_L 11
#define SCSCL_MAX_ANGLE_LIMIT_H 12
#define SCSCL_CW_DEAD 26
#define SCSCL_CCW_DEAD 27

//-------SRAM(read & write)--------
#define SCSCL_TORQUE_ENABLE 40
#define SCSCL_GOAL_POSITION_L 42
#define SCSCL_GOAL_POSITION_H 43
#define SCSCL_GOAL_TIME_L 44
#define SCSCL_GOAL_TIME_H 45
#define SCSCL_GOAL_SPEED_L 46
#define SCSCL_GOAL_SPEED_H 47
#define SCSCL_LOCK 48

//-------SRAM(read & write)--------
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
	virtual int WritePos(u8 ID, u16 Position, u16 Time, u16 Speed);//general write for single servo
	virtual int WritePosEx(u8 ID, s16 Position, u16 Speed, u8 ACC);//position command for single servo
	virtual int RegWritePos(u8 ID, u16 Position, u16 Time, u16 Speed = 0);//position write asynchronously for single servo(call RegWriteAction to action)
	virtual void SyncWritePos(u8 ID[], u8 IDN, u16 Position[], u16 Time[], u16 Speed[]);//write synchronously for multi servos
	virtual int PWMMode(u8 ID);//output PWM mode
	virtual int WritePWM(u8 ID, s16 pwmOut);//output PWM mode command
	virtual int EnableTorque(u8 ID, u8 Enable);//torque ctrl command
	virtual int unLockEprom(u8 ID);//eprom unlock
	virtual int LockEprom(u8 ID);//eprom locked
	virtual int FeedBack(int ID);//servo information feedback
	virtual int ReadPos(int ID);//read position
	virtual int ReadSpeed(int ID);//read speed
	virtual int ReadLoad(int ID);//read motor load(0~1000, 1000 = 100% max load)
	virtual int ReadVoltage(int ID);//read voltage
	virtual int ReadTemper(int ID);//read temperature
	virtual int ReadMove(int ID);//read move mode
	virtual int ReadCurrent(int ID);//read current
	virtual int ReadMode(int ID);//read working mode
	virtual int CalibrationOfs(u8 ID);//set middle position(placeholder, can not be used)
	virtual int ReadInfoValue(int ID, int AddInput);//read servo type
private:
	u8 Mem[SCSCL_PRESENT_CURRENT_H-SCSCL_PRESENT_POSITION_L+1];
};

#endif