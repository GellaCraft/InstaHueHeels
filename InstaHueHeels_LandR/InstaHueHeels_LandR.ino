/*********************************************************************
 Insta-Hue Heels - Shoes
 Learn more about the project at: https://www.gellacraft.com/instahueheels

 This code works for both the left and right shoes.
 
 It also uses example code from SparkFun's RFM69HCW breakout board
 https://learn.sparkfun.com/tutorials/rfm69hcw-hookup-guide

 LEDs are controlled using Adafruit's NeoPixel library examples
 For more information on using NeoPixels, check out https://learn.adafruit.com/adafruit-neopixel-uberguide?view=all
 
 MIT license, check LICENSE for more information
 All text above, and the splash screen below must be included in
 any redistribution
 
*********************************************************************/

/*=========================================================================
  NEOPIXEL MATRIX ARRAYS
  
  I used arrays for storing pixel addresses. You may also want to leverage Adafruit's NeoMatrix 
  Library for your project. Your matrix may have a different origin than mine.
  read more about NeoPixel matrices in Adafruit's guide:
  https://learn.adafruit.com/adafruit-neopixel-uberguide?view=all#neopixel-matrices

-------------------------------------------------------------------------*/


int col1[8] = {0,   1,  2,  3,  4,  5,  6,  7};
int col2[8] = {15, 14, 13, 12, 11, 10,  9,  8};
int col3[8] = {16, 17, 18, 19, 20, 21, 22, 23};
int col4[8] = {31, 30, 29, 28, 27, 26, 25, 24};
int col5[8] = {32, 33, 34, 35, 36, 37, 38, 39};
int col6[8] = {47, 46, 45, 44, 43, 42, 41, 40};
int col7[8] = {48, 49, 50, 51, 52, 53, 54, 55};
int col8[8] = {63, 62, 61, 60, 59, 58, 57, 56};

#include <RFM69.h>
#include <SPI.h>

// Addresses for this node both the bracelet and shoes must be set to the same network ID
// The bracelet is set to 1 for sending

#define NETWORKID     0   // Must be the same for all nodes
#define MYNODEID      2   // My node ID, make sure this matches the TONODEID in the Bracelet code
#define TONODEID      1   // Destination node ID (broadcast)

// RFM69 frequency, uncomment the frequency of your module:
//#define FREQUENCY   RF69_433MHZ
#define FREQUENCY     RF69_915MHZ

// Create a library object for our RFM69HCW module:
RFM69 radio;

#include <Adafruit_NeoPixel.h>

#define PIN 6 // Pin pixels are connected to
#define NUMPIXELS 64 // 8x8 NeoPixel matrix has 64 LEDs                                                                                                                                                                                      //Number of pixels in your project
#define BRIGHTNESS 10 // Brightness of your pixels, keep 10 for NeoPixel Matrix to avoid high current draw

// Parameter 1 = number of pixels in pixel
// Parameter 2 = Arduino pin number (most are valid)
// Parameter 3 = pixel type flags, add together as needed:
// NEO_KHZ800  800 KHz bitstream (most NeoPixel products w/WS2812 LEDs)
// NEO_GRB     Pixels are wired for GRB bitstream (most NeoPixel products)
Adafruit_NeoPixel pixel = Adafruit_NeoPixel(NUMPIXELS, PIN, NEO_GRB + NEO_KHZ800);

//Color and mode variables
unsigned char mode;
unsigned int R;
unsigned int G;
unsigned int B;

void setup()
{
  // Open a serial port so we can send keystrokes to the module: 
  Serial.begin(9600);
  Serial.print("Node ");
  Serial.print(MYNODEID,DEC);
  Serial.println(" ready");  

  // Initialize the RFM69HCW:
  
  radio.initialize(FREQUENCY, MYNODEID, NETWORKID);
  radio.setHighPower(); // Always use this for RFM69HCW
  
  pixel.begin(); // This initializes the NeoPixel library.
  pixel.setBrightness(BRIGHTNESS); // Set global brightness
  pixel.show(); // Initialize all pixels to 'off'
}

void loop()
{
    // Upon receiving a message the following variables are ready to use from the bracelet: mode, R, G, B
    if (ReceiveMessage())
    {
    // Use the received colors to set the matrix color
    colorWipeDown(pixel.Color(R, G, B), 50);
    }
}

/*------------------------------------------------
  Radio Receive Function
-------------------------------------------------*/
boolean ReceiveMessage()
{
  unsigned char receivebuffer[7];

  if (radio.receiveDone() == false)
    return(false);
  else // Got one!
  {
    // Print out the information:
    
    Serial.print("received from node ");
    Serial.print(radio.SENDERID, DEC);
    Serial.print(": [");

    // The actual message is contained in the DATA array,
    // and is DATALEN bytes in size:
    
    for (byte i = 0; i < radio.DATALEN; i++)
    {
      Serial.print((unsigned char)radio.DATA[i],HEX);
      Serial.print(" ");
    }
    
    // RSSI is the "Receive Signal Strength Indicator",
    // smaller numbers mean higher power.
    
    Serial.print("], RSSI ");
    Serial.println(radio.RSSI);

    // Extract the data

    for (byte i = 0; i < radio.DATALEN; i++)
      receivebuffer[i] = (unsigned char)radio.DATA[i];

    // Set global variables
    
    mode = receivebuffer[0];
    R = (unsigned int)(receivebuffer[1]*256+receivebuffer[2]);
    G = (unsigned int)(receivebuffer[3]*256+receivebuffer[4]);
    B = (unsigned int)(receivebuffer[5]*256+receivebuffer[6]);

    Serial.print("mode: "); Serial.print((unsigned char)mode,DEC);
    Serial.print(" R: "); Serial.print(R,DEC);
    Serial.print(" G: "); Serial.print(G,DEC);
    Serial.print(" B: "); Serial.println(B,DEC);
    
    return(true);
  }
}

/*------------------------------------------------
 Custom NeoPixel Function
 
 This is a custom colorwipe function I wrote to wipe the matrix
 from top to bottom using arrays. You may also use the NeoMatrix
 library to do similar functions
-------------------------------------------------*/

void colorWipeDown(uint32_t c, uint8_t wait) {
  for(uint16_t i=0; i<8; i++) {
    pixel.setPixelColor(col1[i], c);
    pixel.setPixelColor(col2[i], c);
    pixel.setPixelColor(col3[i], c);
    pixel.setPixelColor(col4[i], c);
    pixel.setPixelColor(col5[i], c);
    pixel.setPixelColor(col6[i], c);
    pixel.setPixelColor(col7[i], c);
    pixel.setPixelColor(col8[i], c);
    pixel.show();
    delay(wait);
  }
}

/*------------------------------------------------
 Adafruit Neopixel Example Functions
-------------------------------------------------*/

// Sets all LEDs to off, but DOES NOT update the display;
// call pixel.show() to actually turn them off after this.
void clearLEDs()
{
  for (int i=0; i<NUMPIXELS; i++)
  {
    pixel.setPixelColor(i, 0);
  }
}

// Input a value 0 to 255 to get a color value.
// The colours are a transition r - g - b - back to r.
uint32_t Wheel(byte WheelPos) {
  WheelPos = 255 - WheelPos;
  if(WheelPos < 85) {
    return pixel.Color(255 - WheelPos * 3, 0, WheelPos * 3);
  }
  if(WheelPos < 170) {
    WheelPos -= 85;
    return pixel.Color(0, WheelPos * 3, 255 - WheelPos * 3);
  }
  WheelPos -= 170;
  return pixel.Color(WheelPos * 3, 255 - WheelPos * 3, 0);
}

// Fill the dots one after the other with a color, reverse of normal colorwipe
void colorWipeRev(uint32_t c, uint8_t wait) {
  for(uint16_t i= NUMPIXELS-1; i >= 0; i--) {
    pixel.setPixelColor(i, c);
    pixel.show();
    delay(wait);
  }
}

// Fill the dots one after the other with a color
void colorWipe(uint32_t c, uint8_t wait) {
  for(uint16_t i=0; i<pixel.numPixels(); i++) {
    pixel.setPixelColor(i, c);
    pixel.show();
    delay(wait);
  }
}

void rainbow(uint8_t wait) {
  uint16_t i, j;

  for(j=0; j<256; j++) {
    for(i=0; i<pixel.numPixels(); i++) {
      pixel.setPixelColor(i, Wheel((i+j) & 255));
    }
    pixel.show();
    delay(wait);
  }
}

// Slightly different, this makes the rainbow equally distributed throughout
void rainbowCycle(uint8_t wait) {
  uint16_t i, j;

  for(j=0; j<256*5; j++) { // 5 cycles of all colors on wheel
    for(i=0; i< pixel.numPixels(); i++) {
      pixel.setPixelColor(i, Wheel(((i * 256 / pixel.numPixels()) + j) & 255));
    }
    pixel.show();
    delay(wait);
  }
}

//Theatre-style crawling lights.
void theaterChase(uint32_t c, uint8_t wait) {
  for (int j=0; j<10; j++) {  //do 10 cycles of chasing
    for (int q=0; q < 3; q++) {
      for (int i=0; i < pixel.numPixels(); i=i+3) {
        pixel.setPixelColor(i+q, c);    //turn every third pixel on
      }
      pixel.show();

      delay(wait);

      for (int i=0; i < pixel.numPixels(); i=i+3) {
        pixel.setPixelColor(i+q, 0);        //turn every third pixel off
      }
    }
  }
}

//Theatre-style crawling lights with rainbow effect
void theaterChaseRainbow(uint8_t wait) {
  for (int j=0; j < 256; j++) {     // cycle all 256 colors in the wheel
    for (int q=0; q < 3; q++) {
      for (int i=0; i < pixel.numPixels(); i=i+3) {
        pixel.setPixelColor(i+q, Wheel( (i+j) % 255));    //turn every third pixel on
      }
      pixel.show();

      delay(wait);

      for (int i=0; i < pixel.numPixels(); i=i+3) {
        pixel.setPixelColor(i+q, 0);        //turn every third pixel off
      }
    }
  }
}


