#include <SPI.h>
#include "Adafruit_GFX.h"
#include "Adafruit_RA8875.h"
#define RA8875_INT 3
#define RA8875_CS 10
#define RA8875_RESET 9


///////////
#if ARDUINO >= 100
 #include "Arduino.h"
 #include "Print.h"
#else
 #include "WProgram.h"
#endif

#ifdef __AVR
  #include <avr/pgmspace.h>
#elif defined(ESP8266)
  #include <pgmspace.h>
#endif
///////////


Adafruit_RA8875 tft = Adafruit_RA8875(RA8875_CS, RA8875_RESET);
uint16_t tx, ty;
int32_t currentpage = 0; 

// Touch screen cal structs
 struct Point
{
   int32_t x;
   int32_t y;
} tsPoint_t;

typedef struct //Matrix
{
  int32_t An,
          Bn,
          Cn,
          Dn,
          En,
          Fn,
          Divider ;
} tsMatrix_t;

void setup() 
{
  Serial.begin(9600);
  Serial.println("RA8875 start");

  /* Initialise the display using 'RA8875_800x480' */
  if (!tft.begin(RA8875_800x480)) {
    Serial.println("RA8875 Not Found!");
    while (1);
  }

  tft.displayOn(true);
  tft.GPIOX(true);      // Enable TFT - display enable tied to GPIOX
  tft.PWM1config(true, RA8875_PWM_CLK_DIV1024); // PWM output for backlight
  tft.PWM1out(255);
  tft.fillScreen(RA8875_BLACK);


  /*draw our homescreen rectangles */
   tft.graphicsMode();
   //tft.fillRect(x, y, w, h, color); x, y => top left of rectangle
   tft.fillRect(10, 10, 240, 140, RA8875_RED); /*time*/
   tft.fillRect(270, 10, 260, 140, RA8875_GREEN); /*temperature*/
   tft.fillRect(550, 10, 240, 140, RA8875_BLUE); /*settings*/
   tft.fillRect(10, 170, 140, 140, RA8875_MAGENTA); /*rooms*/
   tft.fillRect(170, 170, 140, 140, RA8875_MAGENTA);
   tft.fillRect(330, 170, 140, 140, RA8875_MAGENTA);
   tft.fillRect(490, 170, 140, 140, RA8875_MAGENTA);
   tft.fillRect(10, 330, 140, 140, RA8875_MAGENTA);
   tft.fillRect(170, 330, 140, 140, RA8875_MAGENTA);
   tft.fillRect(330, 330, 140, 140, RA8875_MAGENTA);
   tft.fillRect(490, 330, 140, 140, RA8875_MAGENTA);
   tft.fillRect(650, 170, 140, 300, RA8875_YELLOW); /*information*/
   /* Switch to text mode */  
  tft.textMode();
  tft.textSetCursor(20, 18); /*time*/
  tft.textEnlarge(2);
  tft.textTransparent(RA8875_WHITE);
  tft.textWrite("2:24 PM");
  tft.textSetCursor(80, 110); /*date*/
  tft.textEnlarge(1);
  tft.textTransparent(RA8875_WHITE);
  tft.textWrite("DD/MM/YEAR");

  tft.textSetCursor(270, 50); /*temperature*/
  tft.textEnlarge(2);
  tft.textTransparent(RA8875_WHITE);
  tft.textWrite("Outside.....");

  tft.touchEnable(1);
}

void loop() 
{
  /*homescreen*/
  if (currentpage == 0) {
    if(tft.touched()){
      tft.touchRead(uint16_t *x, uint16_t *y);
      if ((x>=10) && (x<=250) && (y>=10) && (y<=150)) {
        currentpage == 1;
        tft.fillScreen(RA8875_BLACK);
      }
    }
  }
}
