#ifndef IIC_h
#define IIC_h

char __value[4];

typedef struct {
	int LF=0;
	int LB=0;
	int RF=0;
	int RB=0;
} TOF_t;

#define OLED_ADDRESS 0x3C
#define TOFLF_ADDRESS 0x30
#define TOFLB_ADDRESS 0x31
#define TOFRF_ADDRESS 0x32
#define TOFRB_ADDRESS 0x33
#define MPU_ADDRESS 0x68

#define TOFLF_S 6
#define TOFLB_S 7
#define TOFRF_S 5
#define TOFRB_S 4

#include <Wire.h>
#include <Adafruit_VL53L0X.h>
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

Adafruit_SSD1306 display = Adafruit_SSD1306();

Adafruit_VL53L0X loxLF = Adafruit_VL53L0X();
Adafruit_VL53L0X loxLB = Adafruit_VL53L0X();
Adafruit_VL53L0X loxRF = Adafruit_VL53L0X();
Adafruit_VL53L0X loxRB = Adafruit_VL53L0X();

VL53L0X_RangingMeasurementData_t measureLF;
VL53L0X_RangingMeasurementData_t measureLB;
VL53L0X_RangingMeasurementData_t measureRF;
VL53L0X_RangingMeasurementData_t measureRB;

TOF_t out;

int16_t gyro_z;

/*void buzz(){
  
}*/

void initMPU(){
  Wire.beginTransmission(MPU_ADDRESS);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);
}

void initTOF(){
	pinMode(TOFLF_S,OUTPUT);
	pinMode(TOFLB_S,OUTPUT);
	pinMode(TOFRF_S,OUTPUT);
	pinMode(TOFRB_S,OUTPUT);

	digitalWrite(TOFLF_S,LOW);
	digitalWrite(TOFLB_S,LOW);
	digitalWrite(TOFRF_S,LOW);
	digitalWrite(TOFRB_S,LOW);
	delay(10);
	digitalWrite(TOFLF_S,HIGH);
	delay(10);
  if (!loxLF.begin(TOFLF_ADDRESS)) for(;;);
  delay(10);
  digitalWrite(TOFLB_S,HIGH);
	delay(10);
  if (!loxLB.begin(TOFLB_ADDRESS)) for(;;);
  delay(10);
  digitalWrite(TOFRF_S,HIGH);
	delay(10);
  if (!loxRF.begin(TOFRF_ADDRESS)) for(;;);
  delay(10);
	digitalWrite(TOFRB_S,HIGH);
	delay(10);
  if (!loxRB.begin(TOFRB_ADDRESS)) for(;;);
  delay(10);
}

void initIIC(){
  Wire.begin();
  if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDRESS)) for(;;);
  display.display();
  delay(1000);
  initMPU();
  delay(1000);
  initTOF();
  delay(1000);
  display.setTextColor(SSD1306_WHITE);
  display.clearDisplay();
  display.setTextSize(1);
}

void printReady() {
  display.clearDisplay();
  display.setCursor(0,0);
  display.cp437(true);
  display.write("OLED Display Ready");
  display.display();
  delayMicroseconds(2000);
}

void printLineFollow() {
  display.clearDisplay();
  display.setCursor(0,0);
  display.cp437(true);
  display.setTextSize(2);
  display.write("Line\nFollow");
  display.display();
  display.setTextSize(1);
  delayMicroseconds(2000);
}

void printWallFollow() {
  display.clearDisplay();
  display.setCursor(0,0);
  display.cp437(true);
  display.setTextSize(2);
  display.write("Wall\nFollow");
  display.display();
  display.setTextSize(1);
  delayMicroseconds(2000);
}

void printBoxPick() {
  display.clearDisplay();
  display.setCursor(0,0);
  display.cp437(true);
  display.setTextSize(2);
  display.write("Box Pick");
  display.display();
  display.setTextSize(1);
  delayMicroseconds(2000);
}

void printRampNav() {
  display.clearDisplay();
  display.setCursor(0,0);
  display.cp437(true);
  display.setTextSize(2);
  display.write("Ramp\nNavigation");
  display.display();
  display.setTextSize(1);
  delayMicroseconds(2000);
}

void printPillerDetect() {
  display.clearDisplay();
  display.setCursor(0,0);
  display.cp437(true);
  display.setTextSize(2);
  display.write("Piller\nDetection");
  display.display();
  display.setTextSize(1);
  delayMicroseconds(2000);
}

void printGateArea() {
  display.clearDisplay();
  display.setCursor(0,0);
  display.cp437(true);
  display.setTextSize(2);
  display.write("Gate\nNavigation");
  display.display();
  display.setTextSize(1);
  delayMicroseconds(2000);
}

void printRunAll() {
  display.clearDisplay();
  display.setCursor(0,0);
  display.cp437(true);
  display.setTextSize(2);
  display.write("Run All");
  display.display();
  display.setTextSize(1);
  delayMicroseconds(2000);
}

void printEightInt(int a0,int a1=0,int a2=0,int a3=0,int a4=0,int a5=0,int a6=0,int a7=0) {
  display.clearDisplay();
  display.setTextSize(1);
  display.setCursor(0,0);
  display.cp437(true);
  sprintf(__value,"%3d",a0);
  display.write(__value);
  display.setCursor(32,0);
  sprintf(__value,"%3d",a1);
  display.write(__value);
  display.setCursor(64,0);
  sprintf(__value,"%3d",a2);
  display.write(__value);
  display.setCursor(96,0);
  sprintf(__value,"%3d",a3);
  display.write(__value);
  display.setCursor(0,16);
  sprintf(__value,"%3d",a4);
  display.write(__value);
  display.setCursor(32,16);
  sprintf(__value,"%3d",a5);
  display.write(__value);
  display.setCursor(64,16);
  sprintf(__value,"%3d",a6);
  display.write(__value);
  display.setCursor(96,16);
  sprintf(__value,"%3d",a7);
  display.write(__value);
  display.display();
  display.setTextSize(1);
  delay(10);
}

void clearDisp() {
  display.clearDisplay();
  display.display();
}

TOF_t readTOF(){
  loxLF.rangingTest(&measureLF, false);
  loxLB.rangingTest(&measureLB, false);
  loxRF.rangingTest(&measureRF, false);
  loxRB.rangingTest(&measureRB, false);
  out.LF = measureLF.RangeMilliMeter;
  out.LB = measureLB.RangeMilliMeter;
  out.RF = measureRF.RangeMilliMeter;
  out.RB = measureRB.RangeMilliMeter;
  return out;
}

int readMPU(){
  Wire.beginTransmission(MPU_ADDRESS);
  Wire.write(0x47);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDRESS, 1*2, true);
  gyro_z = Wire.read()<<8 | Wire.read();
  gyro_z = gyro_z/10;
  return (int)(gyro_z);
}

#endif
