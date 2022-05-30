#ifndef _TRACKER_H_
#define _TRACKER_H_

#include <numeric>

typedef unsigned char uchar;

struct accData{
	short wordL;
	short wordH;
	int AngleX;
	int AngleY;
	int result;
};

struct GetInfoResult {
	short wordL;
	short wordH;
	int result;
	float fresult;
};


void tracker_init();
void tracker_deinit();

void BLDCMotor_MoveToAngle(uchar motorId, float angle, short direction, uchar torque, float speed);
void BLDCMotor_MoveByAngle(uchar motorId, float angle, uchar torque, float speed);
void BLDCMotor_MoveContinuous(uchar motorId, uchar direction, uchar torque, float speed);
void BLDCMotor_IdleTorque(uchar motorId, uchar torque, unsigned short torquems);
void BLDCMotor_MoveStop(uchar motorId);
void BLDCMotor_DefaultTimerInterval(short interval);
void BLDCMotor_SetPolePairs(uchar motorId, uchar polePairs);
void BLDCMotor_Reset();
void BLDCMotor_Reboot();
//0 - electrical position; 1 - current Torque; 2 - lastMotionTime - reading of the time since the last movement of the motors
GetInfoResult BLDCMotor_GetInfo(uchar motorId, uchar info, bool zprint);

accData readAccelerometer();
float getAngleX();
float getAngleY();

GetInfoResult GetPWMEncoderAngle(uchar encoderId, bool zprint);

void sleepms(int ms);

#endif // _TRACKER_H_