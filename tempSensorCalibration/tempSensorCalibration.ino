/*
  SD card datalogger
  This example shows how to log data from an analog sensors
  to an SD card using the SD library.
  The circuit:
 ** analog sensors on analog ins 0, 1, and 2
 ** SD card attached to SPI bus as follows:
 ** MOSI - pin 11
 ** MISO - pin 12
 ** CLK - pin 13
 ** CS - pin 4 (for MKRZero SD: SDCARD_SS_PIN)

  created  24 Nov 2010
  modified 9 Apr 2012
  by Tom Igoe Revised S hunt Nov 2021

  This example code is in the public domain.
*/

#include "RTClib.h" //Library Real Time Clock
#include <Wire.h> // Used for I2C implementation
#include <Adafruit_GFX.h>
#include "Adafruit_LEDBackpack.h"
#include <SPI.h>
#include <SD.h>

Adafruit_7segment matrix = Adafruit_7segment();  //new Matrix object.
RTC_PCF8523 rtc;

int buttonPin = 8;
int clearLed = 9;

const int chipSelect = 10;            //SD Shield
const unsigned int NUM_AVG = 100;
const int DIODEpin = A0;
const int LM35pin = A1;
const float V_ref = 3.2979;
const float dvdt = 0.01;
const float c = 2.758164927;    //Y intercept voltage-- voltage for Zero degrees 'C'
const float  m = -0.014873019;            //m of the line voltage v temp--ie senstivity 'M'

unsigned long previousMillis = 0;
const long logInterval = 1000;

void setup() {

  // Open serial communications and wait for port to open:
  Serial.begin(9600);
  analogReference(EXTERNAL);

  pinMode(buttonPin, INPUT_PULLUP);
  pinMode(clearLed, OUTPUT);

#ifndef __AVR_ATtiny85__
  Serial.begin(9600);
  Serial.println("7 Segment Backpack Test");

#endif
  int power = A2;  //Power the 7 segment module
  int gnd   = A3;
  pinMode(power, OUTPUT);
  pinMode(gnd, OUTPUT);
  digitalWrite(power, HIGH);
  digitalWrite(gnd, LOW);
  delay(10);

  matrix.begin(0x70); //Initialise the 7 semement display via I2C

  // pinMode(in, INPUT_PULLUP);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }

  Serial.print("Initializing SD card...");

  // see if the card is present and can be initialized:
  if (!SD.begin(chipSelect)) {
    Serial.println("Card failed, or not present");
    // don't do anything more:
    while (1);
  }
  Serial.println("card initialized.");

  if (digitalRead(buttonPin) == LOW)
  {
    SD.remove("Group100.txt");//Removes any previous logging to file.
    Serial.print("Clearing SD_Card");
    digitalWrite(clearLed, HIGH);
    delay(5000);
    digitalWrite(clearLed, LOW);
  }

  if (! rtc.begin()) {
    Serial.println("Couldn't find RTC");
    while (1);
  }

  if (! rtc.initialized()) {
    Serial.println("RTC is NOT running!");
    // following line sets the RTC to the date & time this sketch was compiled

  }
  rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
}

void loop() {

  float diodeVoltage = get_reading_v(DIODEpin);
  float diodeTemperature = (diodeVoltage - c) / m;

  float referenceTemperature = get_reading_v(LM35pin) / dvdt;


  matrix.print(diodeTemperature, 1);  //Send diode sensor value to display
  matrix.writeDisplay();
  matrix.setBrightness(5);

  //Serial.print("Diode Temp:  ");
  //Serial.print(diodeTemperature,1);

  //Serial.print("   Reference Temperature:  ");
  //Serial.println (referenceTemperature,1);

  unsigned long currentMillis = millis();

  if (currentMillis - previousMillis >= logInterval) {
    // save the last time you blinked the LED
    previousMillis = currentMillis;

    DateTime now = rtc.now();    //Get real-time

    // make a string for assembling the data to log:
    String dataString = "";

    // read three sensors and append to the string:

    //dataString += String(now.year(), DEC);
    //dataString += String('/');

    dataString += String(now.day(), DEC);
    dataString += String('/');
    dataString += String(now.month(), DEC);
    dataString += String(" - ");
    dataString += String(now.hour(), DEC);
    dataString += String(':');
    dataString += String(now.minute(), DEC);
    dataString += String(':');
    dataString += String(now.second(), DEC);
    dataString += String(" ");
    dataString += String(" ");

    dataString += String("-  Diode Temp -   ");
    dataString += String(diodeTemperature, 2);

    dataString += String(" -   Diode Voltage  -   ");
    dataString += String(diodeVoltage, 6);

    dataString += String(" -  Reference Temp  -   ");
    dataString += String(referenceTemperature, 2);

    dataString += "";

    // open the file. note that only one file can be open at a time,
    // so you have to close this one before opening another.
    File dataFile = SD.open("GroupX.txt", FILE_WRITE);

    // if the file is available, write to it:
    if (dataFile) {
      dataFile.println(dataString);
      dataFile.close();
      // print to the serial port too:
      // Serial.print("Temp ");
      Serial.println(dataString);
    }
    // if the file isn't open, pop up an error:
    else {
      Serial.println("error opening datalog.txt");
    }

  }
  //delay(1000);  //Set the display interval units ms
}
float average_reading(int pin) {
  unsigned long average = 0;
  for (int i = 0; i < NUM_AVG; i++) {
    average += analogRead(pin);
    delay(1);
  }
  return ((float)average) / NUM_AVG;
}
float convertToVolts(float reading) {
  return reading / 1023.0 * V_ref;
}
float get_reading_v(int pin) {
  return convertToVolts(average_reading(pin));
}
