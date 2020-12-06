#ifndef Pinconfig_h
#define Pinconfig_h

#define TOFLF_S 37
#define TOFLB_S 36
#define TOFRF_S 39
#define TOFRB_S 38

#define leftMotorIn1 30
#define leftMotorIn2 29
#define leftMotorEnable 28
#define leftMotorPWM 7
#define leftMotorEncoderA 2
#define leftMotorEncoderB 3

#define rightMotorIn1 33
#define rightMotorIn2 32
#define rightMotorEnable 31
#define rightMotorPWM 6
#define rightMotorEncoderA 18
#define rightMotorEncoderB 19

int leftMotorPinOut[6] = {leftMotorIn1,leftMotorIn2,leftMotorEnable,leftMotorPWM,leftMotorEncoderA,leftMotorEncoderB};
int rightMotorPinOut[6] = {rightMotorIn1,rightMotorIn2,rightMotorEnable,rightMotorPWM,rightMotorEncoderA,rightMotorEncoderB};

#endif
