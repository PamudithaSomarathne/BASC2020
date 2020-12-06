#ifndef Motors_h
#define Motors_h

#include "Arduino.h"

#define ninety 20
#define back 40

typedef struct {
  uint8_t enA;
  uint8_t enB;
  uint8_t state;
  int64_t pos;
} encoder_t;

class MOTOR {
  public:
    MOTOR(uint8_t i1, uint8_t i2, uint8_t i3, uint8_t i4, uint8_t i5, uint8_t i6, uint8_t i7) {
      baseSpeed = i7;
      in1 = i1; in2 = i2; en = i3;  pwm = i4;
      encoder.enA = i5; encoder.enB = i6;
      pinMode(in1, OUTPUT);
      pinMode(in2, OUTPUT);
      pinMode(en, OUTPUT);
      pinMode(pwm, OUTPUT);
      pinMode(encoder.enA, INPUT_PULLUP);
      pinMode(encoder.enB, INPUT_PULLUP);
      encoder.pos = 0;
      delayMicroseconds(2000);
      encoder.state = (digitalRead(encoder.enA) << 1) + digitalRead(encoder.enB);
      attach_interrupt(i5, &encoder);
    }

  private:
    int in1, in2, en, pwm, baseSpeed;
    encoder_t encoder;
    
  public:
    static encoder_t *encRefLeft;
    static encoder_t *encRefRight;

    inline int32_t read() {
      noInterrupts();
      int32_t ret = encoder.pos;
      interrupts();
      return ret;
    }

    inline void write(int32_t p) {
    noInterrupts();
    encoder.pos = p;
    interrupts();
  }

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

    void forwardSp(uint8_t speed) {
    	digitalWrite(in1, HIGH);
    	digitalWrite(in2, LOW);
    	analogWrite(pwm, speed);
    }

    void reverseSp(uint8_t speed) {
      digitalWrite(in1, LOW);
      digitalWrite(in2, HIGH);
      analogWrite(pwm, speed);
    }

    void forwardDist() {
      encoder.pos = 0;
      digitalWrite(in1, HIGH);
      digitalWrite(in2, LOW);
      analogWrite(pwm, baseSpeed);
      while (encoder.pos < 16){read();}
      analogWrite(pwm, 0);
    }

    void reverseDist() {
      encoder.pos = 0;
      digitalWrite(in1, LOW);
      digitalWrite(in2, HIGH);
      analogWrite(pwm, baseSpeed);
      while (encoder.pos > (-16)){read();}
      analogWrite(pwm, 0);
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
          case 2: encRefLeft = s; attachInterrupt(digitalPinToInterrupt(2), isr0, CHANGE); break;
          case 18: encRefRight = s; attachInterrupt(digitalPinToInterrupt(18), isr1, CHANGE); break;
          }
        }
    
        static void isr0() {
          updateEnc(encRefLeft);
        }

        static void isr1() {
          updateEnc(encRefRight);
        };
};

class MOTCON {
  private:
    MOTOR left, right;
  public:
    MOTCON(int l[6], int r[6], int bs): left(l[0], l[1], l[2], l[3], l[4], l[5], bs), right(r[0], r[1], r[2], r[3], r[4], r[5], bs) {
      left.turnMotorOn(); right.turnMotorOn();
    }

    void forwardSpeed(uint8_t leftSp, uint8_t rightSp){
    	left.forwardSp(leftSp);
    	right.forwardSp(rightSp);
    }

    void reverseSpeed(uint8_t leftSp, uint8_t rightSp){
    	left.reverseSp(leftSp);
    	right.reverseSp(rightSp);
    }

    void turnLeft() {
      for (int i=0; i<ninety; i++){
        left.reverseDist();
        right.forwardDist();
      }
    }

    void turnRight() {
      for (int i=0; i<ninety; i++){
        left.forwardDist();
        right.reverseDist();
      }
    }

    void turnBackLeft() {
      for (int i=0; i<back; i++){
        left.reverseDist();
        right.forwardDist();
      }
    }

    void turnBackRight() {
      for (int i=0; i<back; i++){
        left.forwardDist();
        right.reverseDist();
      }
    }

    void readPosition(){
      Serial.print(left.read());Serial.print("\t");
      Serial.println(right.read());
    }

    void forwardDistance(int dist) {
      while (dist>0){
        left.forwardDist();
        right.forwardDist();
        dist--;
      }
    }

    void reverseDistance(int dist) {
      while (dist>0){
        left.reverseDist();
        right.reverseDist();
        dist--;
      }
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
