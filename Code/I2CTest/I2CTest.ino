#include "IIC.h"

int d1=0,d2=0,d3=0,d4=0;
TOF_t dis;

void setup(){
  initIIC();
}

void loop(){
  dis = readTOF(); d1 = dis.LF; d2 = dis.LB; d3 = dis.RF; d4 = dis.RB;
  printEightInt(d1,d2,d3,d4,readMPU());
  delay(20);
}
