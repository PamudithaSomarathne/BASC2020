#include "Motors.h"

MOTOR left = MOTOR(12,11,10,9,2,3,100);

void setup() {
  left.turnMotorOn();
}

void loop() {
  left.moveDistance(-1675);
  delay(1000);
}
