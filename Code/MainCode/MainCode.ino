#include "PinConfig.h"
#include "Motors.h"

MOTCON robot = MOTCON(leftMotorPinOut,rightMotorPinOut,200);

void setup() {
  Serial.begin(9600);
}

void loop() {
  robot.forwardDistance(21);
  delay(1000);
  robot.reverseDistance(21);
  delay(1000);
  robot.turnLeft();
  delay(1000);
  robot.turnRight();
  delay(1000);
  robot.turnBackLeft();
  delay(1000);
  robot.turnBackRight();
  delay(2000);
}
