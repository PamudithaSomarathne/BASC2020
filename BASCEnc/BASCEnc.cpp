#include "Arduino.h"
#include "BASCEnc.h"

/*
	Getting input from the interupts

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

}