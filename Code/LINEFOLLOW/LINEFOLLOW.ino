
#define sensors 8 //number of sensors used in raykha
#define TimeOut //waits 2500 us for sensor outputs to go to low
#define EmitterPin   3     // emitter is controlled by digital pin 2
//raykha((unsigned char[]){D0,D1,D2,D3,D4,D5,D6,D7},Num_sensors,TimeOut,EmitterPin);

//robot_position= 5;
unsigned int sensorValues[8]={};

int thres=100;
const int lp=2,le=4,li1=5,li2=6,ri2=9,ri1=8,re=7,rp=3;
float last_error;
void stopRobot(){
  analogWrite(lp,0);
  analogWrite(rp,0);  
}

void moveRobot(int leftVal, int rightVal){
  digitalWrite(li1,LOW);
  digitalWrite(li2,HIGH);
  digitalWrite(ri1,LOW);
  digitalWrite(ri2,HIGH);
  analogWrite(lp,abs(leftVal));
  analogWrite(rp,abs(rightVal));
}

void reverseRobot(){
  digitalWrite(li2,LOW);
  digitalWrite(li1,HIGH);
  digitalWrite(ri2,LOW);
  digitalWrite(ri1,HIGH);
  analogWrite(lp,70);
  analogWrite(rp,70);
}

void setup() {
  pinMode(lp,OUTPUT);
  pinMode(le,OUTPUT);
  pinMode(li1,OUTPUT);
  pinMode(li2,OUTPUT);
  pinMode(ri2,OUTPUT);
  pinMode(ri1,OUTPUT);
  pinMode(re,OUTPUT);
  pinMode(rp,OUTPUT);

  Serial.begin(9600);
  
  digitalWrite(le,HIGH);
  digitalWrite(re,HIGH);
}
int read_sensor(){
  unsigned int sensorValues[8];
  
  sensorValues[0]=analogRead(A7)>thres ? 1:0;
  sensorValues[1]=analogRead(A6)>thres ? 1:0;
  sensorValues[2]=analogRead(A5)>thres ? 1:0;
  sensorValues[3]=analogRead(A4)>thres ? 1:0;
  sensorValues[4]=analogRead(A3)>thres ? 1:0;
  sensorValues[5]=analogRead(A2)>thres ? 1:0;
  sensorValues[6]=analogRead(A1)>thres ? 1:0;
  sensorValues[7]=analogRead(A0)>thres ? 1:0;

  /*Serial.print(sensorValues[0]); Serial.print(' ');
  Serial.print(sensorValues[1]); Serial.print(' ');
  Serial.print(sensorValues[2]); Serial.print(' ');
  Serial.print(sensorValues[3]); Serial.print(' ');
  Serial.print(sensorValues[4]); Serial.print(' ');
  Serial.print(sensorValues[5]); Serial.print(' ');
  Serial.print(sensorValues[6]); Serial.print(' ');
  Serial.print(sensorValues[7]); Serial.println();*/
  
  float posi;
  int sensor_sum = (sensorValues[0] +sensorValues[1] +sensorValues[2] +sensorValues[3] +sensorValues[4] +sensorValues[5] +sensorValues[6] +sensorValues[7]);
  switch(sensor_sum){
    case 0:
      posi = 44;
      break; //all sensors are on white
    case 8:
      posi = 46;
      break; //all sensors are on black - Reached end or on start
    default:
      posi = (sensorValues[0]*10 + sensorValues[1]*20 + sensorValues[2]*30 + sensorValues[3]*40 + sensorValues[4]*50 + sensorValues[5]*60 + sensorValues[6]*70 + sensorValues[7]*80)/ sensor_sum;
      break;

    }
    //Serial.println(posi);
    return posi;
    
}

void loop() {
  PID_line();
  
}
void PID_line(){
  int base_speed=80;
  int max_change=70;

  int follow_line=1;
  float p_line=1;
  float i_line=0;
  float d_line=0;
  while(follow_line==1){
    int error=(read_sensor() - 45);
    Serial.println(error);
    Serial.println();
    if (error==-1 || error==1)
    {
      if(error==1)      //reached the end - or perpedicular to line somehow
      {
        follow_line=0;
        stopRobot();            
        break;
      }
      if(error==-1)       //line lost
      {
        reverseRobot();
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
      moveRobot(base_speed+correction, base_speed-correction );
      last_error=error;
        
      }
    }
  }
