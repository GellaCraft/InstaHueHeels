/*********************************************************************
 Insta-Hue Heels - Bracelet
 Learn more about the project at: https://www.gellacraft.com/instahueheels
 
 This code borrows from the Controller example for Adafruit's nRF51822 based Bluefruit LE modules
 https://learn.adafruit.com/bluefruit-le-connect-for-ios?view=all

 It also uses example code from SparkFun's RFM69HCW breakout board
 https://learn.sparkfun.com/tutorials/rfm69hcw-hookup-guide

 LEDs are controlled using Adafruit's NeoPixel library examples
 For more information on using NeoPixels, check out https://learn.adafruit.com/adafruit-neopixel-uberguide?view=all
 
 MIT license, check LICENSE for more information
 All text above, and the splash screen below must be included in
 any redistribution

 Hardware notes:
 * Make sure your Flora Bluefruit is set to CMD mode with the switch on board
*********************************************************************/

//Bluefruit Libraries
#include "BluefruitConfig.h"
#include "Adafruit_BLE.h"
#include "Adafruit_BluefruitLE_UART.h"

//RFM69 Libraries
#include <RFM69.h>
#include <string.h>
#include <Arduino.h>
#include <SPI.h>

//Neopixel Library
#include <Adafruit_NeoPixel.h>

#if SOFTWARE_SERIAL_AVAILABLE
  #include <SoftwareSerial.h>
#endif

/*=========================================================================
  NEOPIXEL SETUP
-------------------------------------------------------------------------*/

#define PIN         6
#define NUMPIXELS   16 // This project uses the Adafruit 16 NePixel Ring sku#
#define BRIGHTNESS  10 // Brightness of your pixels, keep 10 for NeoPixel Matrix to avoid high current draw

// Parameter 1 = number of pixels in pixel
// Parameter 2 = Arduino pin number (most are valid)
// Parameter 3 = pixel type flags, add together as needed:
// NEO_KHZ800  800 KHz bitstream (most NeoPixel products w/WS2812 LEDs)
// NEO_GRB     Pixels are wired for GRB bitstream (most NeoPixel products)
Adafruit_NeoPixel pixel = Adafruit_NeoPixel(NUMPIXELS, PIN, NEO_GRB + NEO_KHZ800);

/*=========================================================================
  NAVIGATION SWITCH SETUP
  The switch functionality is not set up in this code yet, stay tuned for
  future updates to use the switch as a menu selector
-------------------------------------------------------------------------*/

#define upButton A2 
#define selectButton A1
#define downButton A0

// Switch variables
int downState; 
int lastDownState;

int selectState;
int lastSelectState;

int upState;
int lastUpState;

// Menu variables
int menuPosition = 0; // Menu off
int menuState = 1;

/*=========================================================================
  APPLICATION SETTINGS

  FACTORYRESET_ENABLE  Perform a factory reset when running this sketch

  MINIMUM_FIRMWARE_VERSION  Minimum firmware version to have some new features
  MODE_LED_BEHAVIOUR        LED activity, valid options are
                            "DISABLE" or "MODE" or "BLEUART" or
                            "HWUART"  or "SPI"  or "MANUAL"
  -----------------------------------------------------------------------*/
  #define FACTORYRESET_ENABLE         1
  #define MINIMUM_FIRMWARE_VERSION    "0.6.6"
  #define MODE_LED_BEHAVIOUR          "MODE"
/*=========================================================================*/

// Use Hardware Serial
// There are a lot of other options in Adafruit's example code, but the Flora Bluefruit
// needs to be UART so this is the only one I am showing
Adafruit_BluefruitLE_UART ble(BLUEFRUIT_HWSERIAL_NAME, BLUEFRUIT_UART_MODE_PIN);

// A small helper - error trap
void error(const __FlashStringHelper*err) {
  Serial.println(err);
  while (1);
}

// function prototypes over in packetparser.cpp
uint8_t readPacket(Adafruit_BLE *ble, uint16_t timeout);
float parsefloat(uint8_t *buffer);
void printHex(const uint8_t * data, const uint32_t numBytes);

// the packet buffer
extern uint8_t packetbuffer[];

/*=========================================================================
  RFM69 SETUP
-----------------------------------------------------------------------*/

// Addresses for this node both the bracelet and shoes must be set to the same network ID
// The bracelet is set to 1 for sending
// The destination is the shoes, make sure to set MYNODEID in the shoe code to 2

#define NETWORKID     0   // Must be the same for all nodes
#define MYNODEID      1   // My node ID this is 1 for the sender
#define TONODEID      2   // Destination node ID (broadcast), make the shoes are 

// RFM69 frequency, uncomment the frequency of your module:
//#define FREQUENCY   RF69_433MHZ
#define FREQUENCY     RF69_915MHZ

// Create a library object for our RFM69HCW module:
RFM69 radio;

//Color and mode variables for RFM69 sending and Bluetooth pass through
unsigned char mode;
uint8_t R;
uint8_t G;
uint8_t B;


/**************************************************************************/
/**************************************************************************/
void setup(void)
{

/*-----------------------------------------------------------------------
    RFM69 SETUP
 -----------------------------------------------------------------------*/
  // Open a serial port so we can send keystrokes to the module:
  //Serial.begin(9600);

  // Set Up RFM69HCW:
  Serial.print("Node ");
  Serial.print(MYNODEID,DEC);
  Serial.println(" ready");  

  // Initialize the RFM69HCW:
  radio.initialize(FREQUENCY, MYNODEID, NETWORKID);
  radio.setHighPower(); // Always use this for RFM69HCW

/*-----------------------------------------------------------------------
    BLUETOOTH SETUP
 -----------------------------------------------------------------------*/
  Serial.begin(115200);

  /* Initialise the module */
  Serial.print(F("Initialising the Bluefruit LE module: "));

  if ( !ble.begin(VERBOSE_MODE) )
  {
    error(F("Couldn't find Bluefruit, make sure it's in Command mode & check wiring?"));
  }
  Serial.println( F("OK!") );

  if ( FACTORYRESET_ENABLE )
  {
    /* Perform a factory reset to make sure everything is in a known state */
    Serial.println(F("Performing a factory reset: "));
    if ( ! ble.factoryReset() ){
      error(F("Couldn't factory reset"));
    }
  }

  /* Disable command echo from Bluefruit */
  ble.echo(false);

  Serial.println("Requesting Bluefruit info:");
  /* Print Bluefruit information */
  ble.info();

  Serial.println(F("Please use Adafruit Bluefruit LE app to connect in Controller mode"));
  Serial.println(F("Then activate/use the sensors, color picker, game controller, etc!"));
  Serial.println();

  ble.verbose(false);  // debug info is a little annoying after this point!

  /* Wait for connection */
  while (! ble.isConnected()) {
      delay(500);
  }

  Serial.println(F("******************************"));

  // LED Activity command is only supported from 0.6.6
  if ( ble.isVersionAtLeast(MINIMUM_FIRMWARE_VERSION) )
  {
    // Change Mode LED Activity
    Serial.println(F("Change LED activity to " MODE_LED_BEHAVIOUR));
    ble.sendCommandCheckOK("AT+HWModeLED=" MODE_LED_BEHAVIOUR);
  }

  // Set Bluefruit to DATA mode
  Serial.println( F("Switching to DATA mode!") );
  ble.setMode(BLUEFRUIT_MODE_DATA);

  Serial.println(F("******************************"));

/*-----------------------------------------------------------------------
    SWITCH SETUP
 -----------------------------------------------------------------------*/
 
  pinMode(upButton,INPUT_PULLUP);
  pinMode(selectButton,INPUT_PULLUP);
  pinMode(downButton,INPUT_PULLUP);

/*-----------------------------------------------------------------------
    NEOPIXEL SETUP
 -----------------------------------------------------------------------*/
 
  pixel.begin(); // This initializes the NeoPixel library.
  for(uint8_t i=0; i<NUMPIXELS; i++) {
  pixel.setPixelColor(i, pixel.Color(0,0,0)); // Set to 'Black'/OFF
  }
  pixel.setBrightness(BRIGHTNESS); // Set global brightness
  pixel.show();

}

/**************************************************************************/
/**************************************************************************/
void loop(void)
{
  /*
  // Read buttons - this is for menu integration, not necessary for Bluetooth connection
  // The menu system is not implemented yet, full functionality will be in a future iteration
  downState = digitalRead(downButton);
  selectState = digitalRead(selectButton);
  upState = digitalRead(upButton);
  */
  
  // Wait for new data to arrive from Bluetooth app
  uint8_t len = readPacket(&ble, BLE_READPACKET_TIMEOUT);

  // Got a packet! 
  if (len >0) {
  
  // Store color values
  if (packetbuffer[1] == 'C') {
    uint8_t R = packetbuffer[2];
    uint8_t G = packetbuffer[3];
    uint8_t B = packetbuffer[4];
    
    //Print the received values
    Serial.print ("RGB #");
    if (R < 0x10) Serial.print("0");
    Serial.print(R, HEX);
    if (G < 0x10) Serial.print("0");
    Serial.print(G, HEX);
    if (B < 0x10) Serial.print("0");
    Serial.println(B, HEX);

    //Set the pixels on the bracelet's NeoPixel ring to the received values using the colorWipe function
    colorWipe(pixel.Color(R, G, B), 50);  
    
    //Broadcast values through the RFM69 module
    SendMessage(mode, R, G, B);

    }
  }
  // checkMenu(); //This is for a future integration using the navigation switch
  delay(50);  // Delay a little bit to avoid bouncing

} // End of Loop!


/*-----------------------------------------------------------------------
    Radio Send Function
 -----------------------------------------------------------------------*/

void SendMessage(char mode, unsigned int R, unsigned int G, unsigned int B)
{
  unsigned char sendbuffer[8];
  int sendlength = 7;

  sendbuffer[0] = mode;
  sendbuffer[1] = highByte(R);
  sendbuffer[2] = lowByte(R);
  sendbuffer[3] = highByte(G);
  sendbuffer[4] = lowByte(G);
  sendbuffer[5] = highByte(B);
  sendbuffer[6] = lowByte(B);
  
  Serial.print("sending to node ");
  Serial.print(TONODEID, DEC);
  Serial.print(": [");
  for (byte i = 0; i < sendlength; i++)
  {
    Serial.print((unsigned char)sendbuffer[i],HEX);
    Serial.print(" ");
  }
  Serial.println("]");
      
  radio.send(TONODEID, sendbuffer, sendlength);
}

/*-----------------------------------------------------------------------
    Show Mode Function
 -----------------------------------------------------------------------*/

void showMode(int mode) {
    if (mode == 0) { //bluetooth
    Serial.println("Selected mode 0");
    colorWipe(pixel.Color(R, G, B), 50);  // Red
  }

   if (mode == 4) {
    Serial.println("Selected mode 4");
    theaterChase(pixel.Color(R,G,B), 50);
  }

  if (mode == 8) {
    Serial.println("Selected mode 8");
    theaterChaseRainbow(50);
  }
  
  if (mode == 12) {
    Serial.println("Selected mode 12");
  }

}

/*-----------------------------------------------------------------------
  Neopixel Animations and Functions
  If you change/edit these, make sure to do the same changes to the shoes
  code to ensure that they match
 -----------------------------------------------------------------------*/

// Fill the dots one after the other with a color
void colorWipe(uint32_t c, uint8_t wait) {
  for(uint16_t i=0; i<pixel.numPixels(); i++) {
    pixel.setPixelColor(i, c);
    pixel.show();
    if (digitalRead(downButton) == LOW || digitalRead(upButton) == LOW) return;
    delay(wait);
  }
}

// Sets all LEDs to off, but DOES NOT update the display;
// call pixel.show() to actually turn them off after this.
void clearLEDs()
{
  for (int i=0; i<NUMPIXELS; i++)
  {
    pixel.setPixelColor(i, 0);
  }
}

void rainbow(uint8_t wait) {
  uint16_t i, j;
    while(1)
  {
  for(j=0; j<256; j++) {
    for(i=0; i<pixel.numPixels(); i++) {
      pixel.setPixelColor(i, Wheel((i+j) & 255));
    }
    pixel.show();
    if (digitalRead(downButton) == LOW || digitalRead(upButton) == LOW) return;
    delay(wait);
  }
  }
}

// Slightly different, this makes the rainbow equally distributed throughout
void rainbowCycle(uint8_t wait) {
  uint16_t i, j;
    while(1)
  {
  for(j=0; j<256*5; j++) { // 5 cycles of all colors on wheel
    for(i=0; i< pixel.numPixels(); i++) {
      pixel.setPixelColor(i, Wheel(((i * 256 / pixel.numPixels()) + j) & 255));
    }
    pixel.show();
    if (digitalRead(downButton) == LOW || digitalRead(upButton) == LOW) return;
    delay(wait);
  }
  }
}

//Theatre-style crawling lights.
void theaterChase(uint32_t c, uint8_t wait) {
  while(1){
//  for (int j=0; j<10; j++) {  //do 10 cycles of chasing
    for (int q=0; q < 3; q++) {
      for (uint16_t i=0; i < pixel.numPixels(); i=i+3) {
        pixel.setPixelColor(i+q, c);    //turn every third pixel on
      }
      pixel.show();
      if (digitalRead(downButton) == LOW || digitalRead(upButton) == LOW) return;
      delay(wait);

      for (uint16_t i=0; i < pixel.numPixels(); i=i+3) {
        pixel.setPixelColor(i+q, 0);        //turn every third pixel off
      }
    }
  }
}

//Theatre-style crawling lights with rainbow effect
void theaterChaseRainbow(uint8_t wait) {

  while(1)
  {
    for (int j=0; j < 256; j++) {     // cycle all 256 colors in the wheel
      for (int q=0; q < 3; q++) {
        for (uint16_t i=0; i < pixel.numPixels(); i=i+3) {
          pixel.setPixelColor(i+q, Wheel( (i+j) % 255));    //turn every third pixel on
        }
        pixel.show();
        if (digitalRead(downButton) == LOW || digitalRead(upButton) == LOW) return; 
        delay(wait);
  
        for (uint16_t i=0; i < pixel.numPixels(); i=i+3) {
          pixel.setPixelColor(i+q, 0);        //turn every third pixel off
        }
      }
    }
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

