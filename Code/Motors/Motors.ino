#include "Pinconfig.h"
#include "Motors.h"

#define normSpeed = 100;

MOTCON robot = MOTCON(leftMotorPinOut,rightMotorPinOut);

void setup() {
  Serial.begin(9600);
  Serial.println("Init successful");
}

void loop() {
}
