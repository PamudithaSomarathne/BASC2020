#include "Arduino.h"

class ENCODER {
    private:
        int pos;

    public:
        ENCODER(int pin0, int pin1){
            pinMode(pin0,INPUT);
            pinMode(pin1,INPUT);
            attachInterrupt(digitalPinToInterrupt(pin0),ISR0,CHANGE);
            attachInterrupt(digitalPinToInterrupt(pin1),ISR1,CHANGE);
            pos=0;
        }


}