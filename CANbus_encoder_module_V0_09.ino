/*  Project: Farmbot CANbus Encoder Module
 *  Author: Phobos Industries - Andrew Filmer
 *  Date: 01/10/18
 *  Version: 0.09
 *  Dependencies:
 *  // CAN library:
 *  // Copyright (c) Sandeep Mistry. All rights reserved.
 *  // Licensed under the MIT license. See LICENSE file in the project root for full license information.
 *  // https://github.com/sandeepmistry/arduino-CAN
 *  
 *  // # Adafruit NeoPixel Library [![Build Status](https://travis-ci.org/adafruit/Adafruit_NeoPixel.svg?branch=master)](https://travis-ci.org/adafruit/Adafruit_NeoPixel)
 *  // https://github.com/adafruit/Adafruit_NeoPixel
 *  
 *  // Encoder Library, for measuring quadrature encoded signals
 *  // http://www.pjrc.com/teensy/td_libs_Encoder.html
 *  // Copyright (c) 2011,2013 PJRC.COM, LLC - Paul Stoffregen <paul@pjrc.com>
 */

/*  Change Log:
 *  V0.09-
 *    Reduced the value returned to master by a factor of 4 (position/4)
 *    Changed negative value sending from managed to simple
 *  V0.08-
 *    Fixed 'error' mode bug. Will now clear warning and error mode when initialised again by host.
 *  V0.07-
 *    Moved warning code into its own function
 *    LED sequence for in sync but not initialised
 *    Request for initialisation if resent after master is already online
 *  V0.06-
 *    More online debug options - feedback of encoder while connected via serial
 *    Unify time keeping code methods
 *    Delay warning repeat feature
 *  V0.05-
 *    Warning message added when drift of encoder is detected and master is not aware
 *    Set up some basic message types this remote can send back to the master
 *    Removed old redundant code and functions
 *  V0.04-
 *    Fixed bug in CAN_decode - if message is not RTR, then proceed, otherwise ignore (unwanted) intructions
 *    Fixed bug in CAN_decode - old while loop before reading an instruction data packet
 *    Synchronisation when new set position code is received
 *    Better debuging code for master to remote troubleshooting
 *  V0.03-
 *    Circuit board complete - reallocation of pins to suit
 *    Addition of Neo Pixel for visual feedback
 *    Neo pixel status changing code
 *    Staggering of local address to offset from address 0 (broadcast). Range now starts from 0x10 and increments via dip switch selection
 *    Fixed local address generating code
 *  V0.02-
 *    Build CAN functionality and basic reporting functions
 *  V0.01-
 *    Set up initial paramenters and basic strucure
 */

/* Program flow:
 * Read encoder
 * Update local encoder position
 * Respond to position request
 * Update LED status
 */

//#define DEBUG_RANDOM 1
//#define DEBUG_ENCODER 1

#include <Encoder.h>
#include <CAN.h>
#include <Adafruit_NeoPixel.h>

// Inputs
const int encoderPinA = 2; // Interrupt for encoder A channel
const int encoderPinB = 3; // Interrupt for encoder B channel
const int chipSelectPin = 10;  // SPI SS
const int MOSIpin = 11; // SPI MOSI
const int MISOpin = 12; // SPI MISO
const int SCKpin = 13;  // SPI SCK
const int address1Pin = 17;  // Address select header 0x01
const int address2Pin = 16;  // Address select header 0x02
const int address3Pin = 15;  // Address select header 0x04
const int address4Pin = 14;  // Address select header 0x08

// Outputs
int indicatorPin = 9;  // LED to indicate connection, initialisation and error status

// Constants
const uint8_t broadcastAddress = 0;
const char terminator = ';';
const int maxDrift = 1000;  // Value to determine the maximum allowable drift without an update from the host
const uint32_t minPosition = -2147483648; // Minimum postion to set the encoder to
const uint32_t maxPosition = 2147483647; // Maximum postion to set the encoder to
const uint8_t localAddressOffset = 0x10;
const uint8_t LEDBrightness = 25;

// Variables
int localAddress = 0;
long position = 0;
bool CANfailed = false; // Has CAN been properly initiated
bool initialised = false; // Has the module been initialised by the master
bool synchronised = false;  // Has the master requested the position since an update
bool errorMode = false; // Master has not responded to repeated warnings
uint8_t packet[4]; // Prepared bytes to transmit back to master (long data type)
uint32_t lastPosition = 0;  // Last position as sent to the master
uint32_t lastPositionSerial = 0; // Last position as sent via serial
char data[8]; // Storage buffer for new message decoding
int dataIndex = 0;  // Indexing for data decoding
char instruction; // Instruction interpretation
int32_t dataValue = 0; // Value obtained from instruction message
uint8_t warningNumber = 0;
uint8_t warningLimit = 5;

// Messages
const char initialise = 'I';
const char uninitialised = 'U';
const char OK = 'K';
const char warning = 'W';
const char error = 'E';
const char setPosition = 'S';
const char reset = 'R';

// Time keeping
long _time;
long lastLEDFlash;
const int LEDFlashInterval = 400; // 0.4 seconds
long lastWarning = 0;
int warningInterval = 500;

// Indicator LED variables
uint32_t colour1;
uint32_t colour2;
uint32_t colour3;
uint32_t colour4;
enum sequence {
  first,
  second,
  third,
  fourth
};

// Objects
Encoder rotaryEncoder(encoderPinA, encoderPinB);
Adafruit_NeoPixel statusLED = Adafruit_NeoPixel(1, indicatorPin, NEO_GRB + NEO_KHZ800);
sequence LEDsequence = first;

// NeoPixel colours
const uint32_t red = statusLED.Color(255, 0, 0);  // Red
const uint32_t amber = statusLED.Color(255, 100, 0);  // Amber
const uint32_t orange = statusLED.Color(255, 165, 0);  // Orange
const uint32_t yellow = statusLED.Color(255, 255, 0);  // Yellow
const uint32_t lime = statusLED.Color(127, 255, 0);  // Lime
const uint32_t green = statusLED.Color(0, 255, 0);  // Green
const uint32_t blue = statusLED.Color(0, 0, 255);  // Blue
const uint32_t cyan = statusLED.Color(0, 255, 255);  // Cyan
const uint32_t purple = statusLED.Color(128, 0, 128);  // Purple
const uint32_t pink = statusLED.Color(255, 105, 180);  // Pink
const uint32_t off = statusLED.Color(0, 0, 0);  // Off

// DEBUG comms
long currentTime = 0;
long previousTime = 0;
int debugInterval = 2000;
long random32bit = 0;

void setup() {
  // Serial for DEBUG
  Serial.begin(115200);
  
  //Declare pins
  pinMode(address1Pin, INPUT_PULLUP);
  pinMode(address2Pin, INPUT_PULLUP);
  pinMode(address3Pin, INPUT_PULLUP);
  pinMode(address4Pin, INPUT_PULLUP);
  
  // Create local address via header pins
  localAddress = localAddressOffset + 
                (!digitalRead(address4Pin) * 8) + 
                (!digitalRead(address3Pin) * 4) + 
                (!digitalRead(address2Pin) * 2) + 
                !digitalRead(address1Pin);

  Serial.print("Local address: ");
  Serial.println(localAddress);

  // Neo Pixel
  statusLED.begin();
  statusLED.setBrightness(LEDBrightness); // Up to 255
  statusLED.show();
  // Set boot colour to purple
  colour1 = purple;
  colour2 = purple;
  colour3 = purple;
  colour4 = purple;
  LEDaction();

  // Start the CAN bus at 500 kbps
  if (!CAN.begin(500E3)) {
    Serial.println("Starting CAN failed!");
    CANfailed = true;
  }
}

void loop() {
  // Read encoder
  position = rotaryEncoder.read();

  // Read CAN bus
  int packetSize = CAN.parsePacket();
  
  if(packetSize)
  {
    decodeCAN();
  }

  // Check if we have drifted far without an update from the master
  if((abs)(position - lastPosition) > maxDrift)
  {
    CANwarning();
  }

  // LED status
  if(CANfailed)
  {
    // Bad startup
    colour1 = red;
    colour2 = red;
    colour3 = red;
    colour4 = off;
  }
  else if(errorMode)
  {
    // Bad running
    colour1 = red;
    colour2 = off;
    colour3 = red;
    colour4 = off;
  }
  else if(initialised && synchronised)
  {
    // All OK
    colour1 = green;
    colour2 = off;
    colour3 = off;
    colour4 = off;
    
  }
  else if(!initialised && !synchronised)
  {
    // Boot sequence. Master has not initialised
    colour1 = amber;
    colour2 = amber;
    colour3 = off;
    colour4 = off;
  }
  else if(initialised && !synchronised)
  {
    // Out of sync with master - possible comms failure
    colour1 = red;
    colour2 = off;
    colour3 = orange;
    colour4 = off;
  }
  else if(!initialised && synchronised)
  {
    // Not initialised but master already asking for updates
    colour1 = yellow;
    colour2 = off;
    colour3 = blue;
    colour4 = off;
  }
  else
  {
    // Unknown status
    colour1 = green;
    colour2 = yellow;
    colour3 = red;
    colour4 = blue;
  }
  
  // Light LED
  LEDaction();

  // Debug comms
  #ifdef DEBUG_RANDOM
    _time = millis();
    if(_time - previousTime >= debugInterval)
    {
      // Save time
      previousTime = _time;
      
      // Determine which type of random message to send
      bool coinFlip = random(0,2);
      if(coinFlip)
      {
        // Send random 32bit number
        random32bit = random();
    
        Serial.print("Random 32bit: ");
        Serial.println(random32bit);
    
        // Build message
        buildMessage(random32bit);
    
        CAN.beginPacket(localAddress);
        CAN.write(packet, 4);
        CAN.endPacket();
      }
      else
      {
        // Send random character
        int character = random(65,123);

        Serial.print("Random character: ");
        Serial.println((char)character);

        CAN.beginPacket(localAddress);
        CAN.write(character);
        CAN.endPacket();
      }
  
      
    }
  #endif

  // Debug encoder
  #ifdef DEBUG_ENCODER
    _time = millis();
    if(_time - previousTime >= debugInterval)
    {
      // Save time
      previousTime = _time;

      // Only send if position has changed
      if(position != lastPositionSerial)
      {
        Serial.print("Position: ");
        Serial.println(position);

        // Update last position
        lastPositionSerial = position;
      }
      
    }
  #endif

}
