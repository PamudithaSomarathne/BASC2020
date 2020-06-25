  #include "Arduino.h"

  /*
    Connect Encoder 1 to pins 2 and 3
    Connect Encoder 2 to pins 18 and 19
  */

typedef struct {
  volatile uint8_t * inA_register;
  volatile uint8_t * inB_register;
  uint8_t inA_bitmask;
  uint8_t inB_bitmask;
  uint8_t state;
  int32_t position;
} Encstate_t;

class ENC {

  #define getBaseReg(p) (portInputRegister(digitalPinToPort(p)))
  #define getBitmask(p) (digitalPinToBitMask(p))
  #define pinRead(b,m) (((*(b))&(m)) ? 1 : 0)
  
  private:
    Encstate_t enc;
    uint8_t intsUsed;
    
  public:
    ENC(uint8_t encNumber){
      pinMode(2,INPUT);
      digitalWrite(2, HIGH);
      pinMode(3,INPUT);
      digitalWrite(3, HIGH);
      enc.inA_register = getBaseReg(2);
      enc.inA_bitmask = getBitmask(2);
      enc.inB_register = getBaseReg(3);
      enc.inB_bitmask = getBitmask(3);
      enc.position = 0;
      delayMicroseconds(2000);
      uint8_t s = 0;
      if (pinRead(enc.inA_register, enc.inA_bitmask)) s |= 1;
      if (pinRead(enc.inB_register, enc.inB_bitmask)) s |= 2;
      enc.state = s;
      intArgs[0] = enc.state;
      intArgs[1] = enc.state;
      attachInterrupt(0,isr0,CHANGE);
      attachInterrupt(1,isr1,CHANGE);
      intsUsed = 2;
    }
    static Encstate_t * intArgs[2];
    static void isr0(void) {update(intArgs[0]);}
    static void isr1(void) {update(intArgs[1]);} 
   
    static void update(Encstate_t *arg){
      asm volatile (
      "ld r30, X+"    "\n\t"
      "ld r31, X+"    "\n\t"
      "ld r24, Z"     "\n\t"  // r24 = pin1 input
      "ld r30, X+"    "\n\t"
      "ld r31, X+"    "\n\t"
      "ld r25, Z"     "\n\t"  // r25 = pin2 input
      "ld r30, X+"    "\n\t"  // r30 = pin1 mask
      "ld r31, X+"    "\n\t"  // r31 = pin2 mask
      "ld r22, X"     "\n\t"  // r22 = state
      "andi r22, 3"     "\n\t"
      "and  r24, r30"   "\n\t"
      "breq L%=1"     "\n\t"  // if (pin1)
      "ori  r22, 4"     "\n\t"  //  state |= 4
    "L%=1:" "and  r25, r31"   "\n\t"
      "breq L%=2"     "\n\t"  // if (pin2)
      "ori  r22, 8"     "\n\t"  //  state |= 8
    "L%=2:" "ldi  r30, lo8(pm(L%=table))" "\n\t"
      "ldi  r31, hi8(pm(L%=table))" "\n\t"
      "add  r30, r22"   "\n\t"
      "adc  r31, __zero_reg__"  "\n\t"
      "asr  r22"      "\n\t"
      "asr  r22"      "\n\t"
      "st X+, r22"    "\n\t"  // store new state
      "ld r22, X+"    "\n\t"
      "ld r23, X+"    "\n\t"
      "ld r24, X+"    "\n\t"
      "ld r25, X+"    "\n\t"
      "ijmp"        "\n\t"
    "L%=table:"       "\n\t"
      "rjmp L%=end"     "\n\t"  // 0
      "rjmp L%=plus1"   "\n\t"  // 1
      "rjmp L%=minus1"    "\n\t"  // 2
      "rjmp L%=plus2"   "\n\t"  // 3
      "rjmp L%=minus1"    "\n\t"  // 4
      "rjmp L%=end"     "\n\t"  // 5
      "rjmp L%=minus2"    "\n\t"  // 6
      "rjmp L%=plus1"   "\n\t"  // 7
      "rjmp L%=plus1"   "\n\t"  // 8
      "rjmp L%=minus2"    "\n\t"  // 9
      "rjmp L%=end"     "\n\t"  // 10
      "rjmp L%=minus1"    "\n\t"  // 11
      "rjmp L%=plus2"   "\n\t"  // 12
      "rjmp L%=minus1"    "\n\t"  // 13
      "rjmp L%=plus1"   "\n\t"  // 14
      "rjmp L%=end"     "\n\t"  // 15
    "L%=minus2:"        "\n\t"
      "subi r22, 2"     "\n\t"
      "sbci r23, 0"     "\n\t"
      "sbci r24, 0"     "\n\t"
      "sbci r25, 0"     "\n\t"
      "rjmp L%=store"   "\n\t"
    "L%=minus1:"        "\n\t"
      "subi r22, 1"     "\n\t"
      "sbci r23, 0"     "\n\t"
      "sbci r24, 0"     "\n\t"
      "sbci r25, 0"     "\n\t"
      "rjmp L%=store"   "\n\t"
    "L%=plus2:"       "\n\t"
      "subi r22, 254"   "\n\t"
      "rjmp L%=z"     "\n\t"
    "L%=plus1:"       "\n\t"
      "subi r22, 255"   "\n\t"
    "L%=z:" "sbci r23, 255"   "\n\t"
      "sbci r24, 255"   "\n\t"
      "sbci r25, 255"   "\n\t"
    "L%=store:"       "\n\t"
      "st -X, r25"    "\n\t"
      "st -X, r24"    "\n\t"
      "st -X, r23"    "\n\t"
      "st -X, r22"    "\n\t"
    "L%=end:"       "\n"
    : : "x" (arg) : "r22", "r23", "r24", "r25", "r30", "r31");
    }

    inline int32_t read(){
      if (intsUsed<2){
        noInterrupts();
        update(&enc);
      } else {
        noInterrupts();
      }
      int32_t ret = enc.position;
      interrupts();
      return ret;
    }

    inline void write(int32_t p){
      noInterrupts();
      enc.position = p;
      interrupts();
    }
};

ENC en1(1);

void setup() {
}

void loop() {
}
