#ifndef Control_h
#define Control_h

#define baseSpeed 100

#include "PinConfig.h"
#include "IIC.h"
#include "Motors.h"
#include "PID.h"
#include "ColorSense.h"

MOTCON robot = MOTCON(leftMotorPinOut,rightMotorPinOut,baseSpeed);

void setupAll(){
  intiIIC();
}

void lineFollow(){
  
}

void wallFollow(){
  
}

void circleNavigation(){
  
}

void boxManipulate(){
  
}

void dashLineFollow(){
  
}

void rampNavigation(){
  
}

void pillerDetection(){
  
}

void gateArea(){
  
}

#endif
