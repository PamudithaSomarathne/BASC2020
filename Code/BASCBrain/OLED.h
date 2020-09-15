#ifndef OLED_h
#define OLED_h

#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64

class OLED:{
	private:
		Adafruit_SSD1306 display;
	public:
		OLED(): display(SCREEN_WIDTH,SCREEN_WIDTH,&Wire){
			if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) for(;;);
			display.display();
			delay(1000);
			display.clearDisplay();
			display.textColor(SSD1306_WHITE);
			display.setTextSize(1);
		}

		void printText() {
			display.clearDisplay();
			display.setCursor(0,0);
			display.cp437(true);
			display.Write("Text Goes Here");
			display.display();
			delay(1000);
		}

		void clearDisp() {
			display.clearDisplay();
		}
}

#endif
