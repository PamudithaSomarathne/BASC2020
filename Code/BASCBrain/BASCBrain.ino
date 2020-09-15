#include "Motors.h"
#include "Pinconfig.h"
#include "OLED.h"

OLED disp = OLED();

void setup(){
  disp.printText();
  delay(2000);
  disp.clearDisp();
}

void loop(){
  
}
