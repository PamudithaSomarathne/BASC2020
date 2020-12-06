#ifndef Motors_h
#define Motors_h

#include "Arduino.h"

#define ninety 100
#define back 200

/*typedef struct {
  uint8_t enA;
  uint8_t enB;
  uint8_t state;
  int64_t pos;
} encoder_t;*/

class MOTOR {
  public:
    MOTOR(int i1, int i2, int i3, int i4, int i5, int i6, int i7) {
      baseSpeed = i7;
      in1 = i1; in2 = i2; en = i3;  pwm = i4;
      //encoder.enA = i5; encoder.enB = i6;
      pinMode(in1, OUTPUT);
      pinMode(in2, OUTPUT);
      pinMode(en, OUTPUT);
      pinMode(pwm, OUTPUT);
      //pinMode(encoder.enA, INPUT_PULLUP);
      //pinMode(encoder.enB, INPUT_PULLUP);
      //encoder.pos = 0;
      //delayMicroseconds(2000);
      //encoder.state = (digitalRead(encoder.enA) << 1) + digitalRead(encoder.enB);
      //attach_interrupt(i5, &encoder);
    }

  private:
    int in1, in2, en, pwm, baseSpeed;
    //encoder_t encoder;
    
  public:
    //static encoder_t *encRef;

    /*inline int32_t read() {
      noInterrupts();
      int32_t ret = encoder.pos;
      interrupts();
      return ret;
    }*/

    void turnMotorOn() {
      digitalWrite(in1, LOW);
      digitalWrite(in2, LOW);
      analogWrite(pwm, 0);
      digitalWrite(en, HIGH);
    }

    void turnMotorOff() {
      digitalWrite(en, LOW);
      digitalWrite(in1, LOW);
      digitalWrite(in2, LOW);
      analogWrite(pwm, 0);
    }

    void forward(int speed) {
        digitalWrite(in1, HIGH);
        digitalWrite(in2, LOW);
        analogWrite(pwm, speed);
    }

    void reverse() {
      digitalWrite(in1,LOW);
      digitalWrite(in2,HIGH);
      analogWrite(pwm, baseSpeed);
    }

    /*void moveDistance(int dist) {
      encoder.pos = 0;
      if (dist >= 0) {
        digitalWrite(in1, HIGH);
        digitalWrite(in2, LOW);
        analogWrite(pwm, baseSpeed);
        while (encoder.pos < dist){read();}
        analogWrite(pwm, 0);
      }
      else {
        digitalWrite(in1, LOW);
        digitalWrite(in2, HIGH);
        analogWrite(pwm, baseSpeed);
        while (encoder.pos > dist){read();}
        analogWrite(pwm, 0);
      }
    }
  
      public:
        static void updateEnc(encoder_t *enc) {
          int s = enc->state & 3;
          s |= (8 * digitalRead(enc->enA) + 4 * digitalRead(enc->enB));
          switch (s) {
            case 1: enc->pos++; break;
            case 2: enc->pos--; break;
            case 3: enc->pos += 2; break;
            case 4: enc->pos--; break;
            case 6: enc->pos -= 2; break;
            case 7: enc->pos++; break;
            case 8: enc->pos++; break;
            case 9: enc->pos -= 2; break;
            case 11: enc->pos--; break;
            case 12: enc->pos += 2; break;
            case 13: enc->pos--; break;
            case 14: enc->pos++; break;
            default: break;
          }
          enc->state = s >> 2;
        }
      private:
        static void attach_interrupt(uint8_t pin, encoder_t *s) {
          switch(pin){
          case 2: encRef = s; attachInterrupt(digitalPinToInterrupt(2), isr0, CHANGE); attachInterrupt(digitalPinToInterrupt(3), isr0, CHANGE); break;
          case 18: encRef = s; attachInterrupt(digitalPinToInterrupt(18), isr1, CHANGE); attachInterrupt(digitalPinToInterrupt(19), isr0, CHANGE); break;
          }
        }
    
        static void isr0() {
          updateEnc(encRef);
        };
        
        static void isr1() {
          updateEnc(encRef);
        };*/
};

class MOTCON {
  private:
    MOTOR left, right;
  public:
    MOTCON(int l[6], int r[6], int baseSpeed): left(l[0], l[1], l[2], l[3], l[4], l[5], baseSpeed), right(r[0], r[1], r[2], r[3], r[4], r[5], baseSpeed) {
      left.turnMotorOn(); right.turnMotorOn();
    }

    void moveForward(int leftSpeed, int rightSpeed){
      left.forward(leftSpeed);
      right.forward(rightSpeed);
    }

    /*void turnLeft() {
      left.moveDistance(-ninety);
      right.moveDistance(ninety);
    }

    void turnRight() {
      left.moveDistance(ninety);
      right.moveDistance(-ninety);
    }

    void turnBack() {
      left.moveDistance(back);
      right.moveDistance(-back);
    }

    void moveDistance(int dist) {
      left.moveDistance(dist);
      right.moveDistance(dist);
    }*/

    void reverseRobot(){
      left.reverse();
      right.reverse();
    }

    void stopRobot(){
      left.forward(0);
      right.forward(0);
    }

    void motorsOff(){
      left.turnMotorOff();
      right.turnMotorOff();
    }

    void motorsOn(){
      left.turnMotorOn();
      right.turnMotorOn();
    }

};

#endif
