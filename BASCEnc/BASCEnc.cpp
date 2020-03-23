#include "Arduino.h"
#include "BASCEnc.h"

/*
	void BASCEnc::read(){
	// Output the current rotary count
	}
	
*/




BASCEnc::BASCEnc(int in1, int in2){

	inA = in1;
	inB = in2;

}

void BASCEnc::init(){

	pinMode(inA,INPUT);
	pinMode(inB,INPUT);
	attachInterrupt(0,ISR,CHANGE);
	attachInterrupt(1,ISR,CHANGE);

}

void BASCEnc::ISR(){
	if (digitalRead(inA) == digitalRead(inB)){count++;}
	else {count--;}
}