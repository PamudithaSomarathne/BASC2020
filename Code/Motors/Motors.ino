class MOTOR{
    private:
        int in1,in2,en,pwm,enA,enB;
        int pos, baseSpeed;
        void static isr0(){pos++;}

    public:
        MOTOR(int i1, int i2, int i3, int i4, int i5, int i6){
            in1 = i1; in2 = i2; en = i3;
            pwm = i4; enA = i5; enB = i6;
            pinMode(in1,OUTPUT);
            pinMode(in2,OUTPUT);
            pinMode(en,OUTPUT);
            pinMode(pwm,OUTPUT);
            pinMode(enA,INPUT);
            pinMode(enB,INPUT);
            attachInterrupt(digitalPinToInterrupt(enA),isr0,CHANGE);
        }

        void turnMotorOn(){
            digitalWrite(in1,LOW);
            digitalWrite(in2,LOW);
            analogWrite(pwm,0);
            digitalWrite(en,HIGH);
        }

        void turnMotorOff(){
            digitalWrite(en,LOW);
            digitalWrite(in1,LOW);
            digitalWrite(in2,LOW);
            analogWrite(pwm,0);
        }
        
        void move(int speed){
            if (speed>=0){
                digitalWrite(in1,HIGH);
                digitalWrite(in2,LOW);
                analogWrite(pwm,speed);
            }
            else {
                digitalWrite(in1,LOW);
                digitalWrite(in2,HIGH);
                analogWrite(pwm,(-speed));
            }
        }

        void moveDistance(int dist){
            pos=0
            if (dist>=0){
                digitalWrite(in1,HIGH);
                digitalWrite(in2,LOW);
                analogWrite(pwm,baseSpeed);
                while (pos<dist);
                analogWrite(pwm,0);
            }
            else {
                dist=(-dist)
                digitalWrite(in1,LOW);
                digitalWrite(in2,HIGH);
                analogWrite(pwm,baseSpeed);
                while (pos<dist);
                analogWrite(pwm,0);
            }
        }
};

MOTOR motor = MOTOR(8,9,10,11,2,3);

void setup(){
    motor.turnMotorOn();
}

void loop(){

    motor.move(100);
    delay(1000);
    motor.move(0);
    delay(1000);
    motor.move(-100);
    delay(1000);
    motor.move(0);
    delay(1000);
}
