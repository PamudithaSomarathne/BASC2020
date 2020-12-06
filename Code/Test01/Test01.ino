#include "PinConfig.h"
#include "IIC.h"
#include "Motors.h"

TOF_t dist;
int thres=100;
float last_error;
unsigned int sensorValues[16];
MOTCON robot = MOTCON(leftMotorPinOut, rightMotorPinOut, 100);

void setup(){
  initIIC();
  Serial.begin(115200);
  //robot.motorsOn();
}

void loop(){
  //PIDline();
  PIDwallLeft();
}

void PIDwallLeft(){
  int base_speed=70;
  int max_change=60;

  int follow_wall=1;
  float p_wall=0.1;
  float i_wall=0;
  float d_wall=10;
  while(follow_wall==1){
    dist = readTOF();
    int error = (dist.left - 150);
    Serial.println(error);
      float propotional = error*p_wall;
      float derivative = (error-last_error) * d_wall;
      float intergrative =(intergrative+error) * i_wall;
      float correction = propotional + derivative + intergrative;
      
      if(correction> max_change)
      {
        correction=max_change;
      }
      if(correction<-1*max_change){
        correction=-1*max_change;
      }
      robot.moveForward(base_speed-correction, base_speed+correction );
      last_error=error;
  }
}

void PIDwallRight(){
  int base_speed=70;
  int max_change=60;

  int follow_wall=1;
  float p_wall=0.1;
  float i_wall=0;
  float d_wall=10;
  while(follow_wall==1){
    dist = readTOF();
    int error = (dist.right - 150);
    Serial.println(error);
      float propotional = error*p_wall;
      float derivative = (error-last_error) * d_wall;
      float intergrative =(intergrative+error) * i_wall;
      float correction = propotional + derivative + intergrative;
      
      if(correction> max_change)
      {
        correction=max_change;
      }
      if(correction<-1*max_change){
        correction=-1*max_change;
      }
      robot.moveForward(base_speed+correction, base_speed-correction );
      last_error=error;
  }
}

/*int read_sensor(){  
  sensorValues[0]=analogRead(A8)>thres ? 1:0;
  sensorValues[1]=analogRead(A9)>thres ? 1:0;
  sensorValues[2]=analogRead(A10)>thres ? 1:0;
  sensorValues[3]=analogRead(A11)>thres ? 1:0;
  sensorValues[4]=analogRead(A12)>thres ? 1:0;
  sensorValues[5]=analogRead(A13)>thres ? 1:0;
  sensorValues[6]=analogRead(A14)>thres ? 1:0;
  sensorValues[7]=analogRead(A15)>thres ? 1:0;
  sensorValues[8]=analogRead(A0)>thres ? 1:0;
  sensorValues[9]=analogRead(A1)>thres ? 1:0;
  sensorValues[10]=analogRead(A2)>thres ? 1:0;
  sensorValues[11]=analogRead(A3)>thres ? 1:0;
  sensorValues[12]=analogRead(A4)>thres ? 1:0;
  sensorValues[13]=analogRead(A5)>thres ? 1:0;
  sensorValues[14]=analogRead(A6)>thres ? 1:0;
  sensorValues[15]=analogRead(A7)>thres ? 1:0;
  
  /*Serial.print(sensorValues[0]); Serial.print(' ');
  Serial.print(sensorValues[1]); Serial.print(' ');
  Serial.print(sensorValues[2]); Serial.print(' ');
  Serial.print(sensorValues[3]); Serial.print(' ');
  Serial.print(sensorValues[4]); Serial.print(' ');
  Serial.print(sensorValues[5]); Serial.print(' ');
  Serial.print(sensorValues[6]); Serial.print(' ');
  Serial.print(sensorValues[7]); Serial.print(' ');
  Serial.print(sensorValues[8]); Serial.print(' ');
  Serial.print(sensorValues[9]); Serial.print(' ');
  Serial.print(sensorValues[10]); Serial.print(' ');
  Serial.print(sensorValues[11]); Serial.print(' ');
  Serial.print(sensorValues[12]); Serial.print(' ');
  Serial.print(sensorValues[13]); Serial.print(' ');
  Serial.print(sensorValues[14]); Serial.print(' ');
  Serial.print(sensorValues[15]);Serial.println();
  
  float posi;
  int sensor_sum = (sensorValues[0] +sensorValues[1] +sensorValues[2] +sensorValues[3] +sensorValues[4] +sensorValues[5] + sensorValues[6] +sensorValues[7]+sensorValues[8] +sensorValues[9] +sensorValues[10] +sensorValues[11] +sensorValues[12] +sensorValues[13] + sensorValues[14] +sensorValues[15]);
  switch(sensor_sum){
    case 0:
      posi = 84;
      break; //all sensors are on white
    case 16:
      posi = 86;
      break; //all sensors are on black - Reached end or on start
    default:
      posi = (sensorValues[0]*10 + sensorValues[1]*20 + sensorValues[2]*30 + sensorValues[3]*40 + sensorValues[4]*50 + sensorValues[5]*60 + sensorValues[6]*70 + sensorValues[7]*80+sensorValues[8]*90 + sensorValues[9]*100 + sensorValues[10]*110 + sensorValues[11]*120 + sensorValues[12]*130 + sensorValues[13]*140 + sensorValues[14]*150 + sensorValues[15]*160)/ sensor_sum;
      break;
    }
    return posi;  
}

void PIDline(){
  int base_speed=80;
  int max_change=70;

  int follow_line=1;
  float p_line=1;
  float i_line=0;
  float d_line=0;
  while(follow_line==1){
    int error=(read_sensor() - 85);    //need to change
    //Serial.println(error);
    if (error==-1 || error==1)
    {
      if(error==1)      //reached the end - or perpedicular to line somehow
      {
        follow_line=0;
        robot.stopRobot();            
        break;
      }
      if(error==-1)       //line lost
      {
        robot.reverseRobot();
        follow_line=1;
      }
    }
    else
    {
      float propotional = error*p_line;
      float derivative = (error-last_error) * d_line;
      float intergrative =(intergrative+error) * i_line;
      float correction = propotional + derivative + intergrative;
      
      if(correction> max_change)
      {
        correction=max_change;
      }
      if(correction<-1*max_change){
        correction=-1*max_change;
      }
      robot.moveForward(base_speed+correction, base_speed-correction );
      last_error=error;
      }
    }
}*/
