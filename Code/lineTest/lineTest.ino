int l3,l2,l1,l0,r0,r1,r2,r3,line,thres=100,sp=60;
const int lp=2,le=4,li1=5,li2=6,ri2=9,ri1=8,re=7,rp=3;

void moveRobot(int leftVal, int rightVal){
  analogWrite(lp,abs(leftVal));
  analogWrite(rp,abs(rightVal));
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

  digitalWrite(li1,LOW);
  digitalWrite(li2,HIGH);
  digitalWrite(ri1,LOW);
  digitalWrite(ri2,HIGH);
}

void loop() {
  l3=analogRead(A7)>thres ? 1:0;
  l2=analogRead(A6)>thres ? 1:0;
  l1=analogRead(A5)>thres ? 1:0;
  l0=analogRead(A4)>thres ? 1:0;
  r0=analogRead(A3)>thres ? 1:0;
  r1=analogRead(A2)>thres ? 1:0;
  r2=analogRead(A1)>thres ? 1:0;
  r3=analogRead(A0)>thres ? 1:0;

  /*Serial.print(l3); Serial.print(' ');
  Serial.print(l2); Serial.print(' ');
  Serial.print(l1); Serial.print(' ');
  Serial.print(l0); Serial.print(' ');
  Serial.print(r0); Serial.print(' ');
  Serial.print(r1); Serial.print(' ');
  Serial.print(r2); Serial.print(' ');
  Serial.print(r3); Serial.println();*/

  line = r1+r2+r3+r0-l1-l2-l3-l0;
  Serial.println(line);
  
  if(line>0){
    moveRobot(sp,0);
  }
  else if(line<0){
    moveRobot(0,sp);
  }
  else{
    moveRobot(sp,sp);
  }
  
}
