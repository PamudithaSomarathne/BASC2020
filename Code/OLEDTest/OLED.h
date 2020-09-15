#ifndef OLED_h
#define OLED_h

char __value[4];

#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

Adafruit_SSD1306 display(128,32,&Wire);

void initOLED(){
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) for(;;);
  display.display();
  delay(1000);
  display.setTextColor(SSD1306_WHITE);
  display.clearDisplay();
  display.setTextSize(1);
}

void printText() {
  display.clearDisplay();
  display.setCursor(0,0);
  display.cp437(true);
  display.write("Text goes here");
  display.display();
  delay(1000);
}

void printEightInt(int a0,int a1,int a2,int a3,int a4,int a5,int a6,int a7) {
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
  delay(5);
}

void clearDisp() {
  display.clearDisplay();
  display.display();
}

#endif
