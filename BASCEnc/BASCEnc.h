#ifndef BASCEnc_h
#define BASCEnc_h
#include "Arduino.h"

class BASCEnc {

	private:

	int inA;
	int inB;
	volatile int count = 0;

	public:

		BASCEnc(int in1, int in2);
		void init();
		void ISR();

};

#endif