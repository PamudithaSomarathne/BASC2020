#define CS_M_S0 6
#define CS_M_S1 5
#define CS_M_S2 4
#define CS_M_S3 3
#define CS_M_OUT 8

#define SEL_RED_M  \
   digitalWrite(CS_M_S2,LOW);digitalWrite(CS_M_S3,LOW)
#define SEL_GREEN_M \
   digitalWrite(CS_M_S2,HIGH);digitalWrite(CS_M_S3,HIGH)
#define SEL_BLUE_M \
   digitalWrite(CS_M_S2,LOW);digitalWrite(CS_M_S3,HIGH)
#define SEL_CLEAR_M \
   digitalWrite(CS_M_S2,HIGH);digitalWrite(CS_M_S3,LOW)
#define TWO_PER_M \  
   digitalWrite(CS_M_S0,LOW);digitalWrite(CS_M_S1,HIGH);// reduce the frequency to 2%
#define HUN_PER_M \
   digitalWrite(CS_M_S0,HIGH);digitalWrite(CS_M_S1,HIGH);
/////////////////////////////TO BE CALLIBERATED FOR BETTER RESULTS/////////////////////////////////

int whiteColourLimit_M =100;
int blackColourLimit_M = 1000;

//////////////////////////////////////////////////////////////////////////////////////////////////

int clrFilterVal,redFilterVal,greenFilterVal,blueFilterVal;

unsigned long get_CS_M_reading() {
  unsigned long colourVal;
  noInterrupts();
  colourVal = pulseIn(CS_M_OUT,HIGH,20000); // 2000us=2ms  2Hz min.
  interrupts();
  return colourVal;
}

uint16_t getColourReading_M(void) {
   unsigned long val;

    SEL_CLEAR_M;
    clrFilterVal = val = get_CS_M_reading();

    SEL_RED_M;
    redFilterVal = val = get_CS_M_reading();
    
    SEL_GREEN_M;
    greenFilterVal = val = get_CS_M_reading();
    
    SEL_BLUE_M;
    blueFilterVal = val = get_CS_M_reading();

    Serial.print("CLEAR: "); Serial.print(clrFilterVal);
    Serial.print(" RED: "); Serial.print(redFilterVal);
    Serial.print(" GREEN: "); Serial.print(greenFilterVal);
    Serial.print(" BLUE: "); Serial.print(blueFilterVal);
    Serial.print(" \n");
    
}

int detectColourOnce_M(){
  getColourReading_M();
  int totalColourValue = redFilterVal+greenFilterVal+blueFilterVal;

  if(totalColourValue<whiteColourLimit_M)
    return  4; // white colour
  if(totalColourValue>blackColourLimit_M)
    return  5; // white colour
    
  if(redFilterVal < blueFilterVal && redFilterVal < greenFilterVal)
    return 0; // red
  if(greenFilterVal < blueFilterVal && greenFilterVal < redFilterVal)
    return 1; // green
  if(blueFilterVal < redFilterVal && blueFilterVal+230 < greenFilterVal)
    return 2; // blue
  
  return 3; // confusion means green

}

int detectColour_M() { // 0 red , 1 green , 2 blue , 4 white , 5 black 
   delay(300);
   int colourCount[6]={0,0,0,0,0,0};

   // CAN SHORTEN THE CODE while true if >5 return
   while(true){ //  we get average values to make sure accurate reading
      int thisCol =detectColourOnce_M();
      colourCount[thisCol]++;  
      if(colourCount[thisCol]>=5){
        return thisCol;
      }
   }
   return 5;
}

void setup(){
  Serial.begin(9600);
}

void loop(){
  TWO_PER_M;
  Serial.println(detectColourOnce_M());
  HUN_PER_M;
  Serial.println(detectColourOnce_M());
  delay(3000);
}
