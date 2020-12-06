#ifndef Pinconfig_h
#define Pinconfig_h
#endif

#define leftMotorIn1 33
#define leftMotorIn2 32
#define leftMotorEnable 31
#define leftMotorPWM 7
#define leftMotorEncoderA 2
#define leftMotorEncoderB 3

#define rightMotorIn1 30
#define rightMotorIn2 29
#define rightMotorEnable 28
#define rightMotorPWM 6
#define rightMotorEncoderA 18
#define rightMotorEncoderB 19

int leftMotorPinOut[6] = {leftMotorIn1,leftMotorIn2,leftMotorEnable,leftMotorPWM,leftMotorEncoderA,leftMotorEncoderB};
int rightMotorPinOut[6] = {rightMotorIn1,rightMotorIn2,rightMotorEnable,rightMotorPWM,rightMotorEncoderA,rightMotorEncoderB};
