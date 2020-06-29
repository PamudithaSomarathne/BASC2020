#ifndef Pinconfig_h
#define Pinconfig_h
#endif

#define leftMotorIn1 0
#define leftMotorIn2 1
#define leftMotorEnable 4
#define leftMotorPWM 5
#define leftMotorEncoderA 2
#define leftMotorEncoderB 3

#define rightMotorIn1 6
#define rightMotorIn2 7
#define rightMotorEnable 8
#define rightMotorPWM 9
#define rightMotorEncoderA 10
#define rightMotorEncoderB 11

int leftMotorPinOut[6] = {leftMotorIn1,leftMotorIn2,leftMotorEnable,leftMotorPWM,leftMotorEncoderA,leftMotorEncoderB};
int rightMotorPinOut[6] = {rightMotorIn1,rightMotorIn2,rightMotorEnable,rightMotorPWM,rightMotorEncoderA,rightMotorEncoderB};