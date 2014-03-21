/*
********************************************************************

Copyright 2014, Jean-François Omhover (jf.omhover@gmail.com) and Nanosatisfi, Inc. (www.nanosatisfi.com)

********************************************************************
Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
********************************************************************
*/

#include <Arduino.h>
#include <Wire.h>
#include <EEPROM.h>
#include <OnboardCommLayer.h>
#include <SAT_Mag.h>


// **************
// *** CONFIG ***
// **************
// NOTE : comment/uncomment the following lines depending on your needs

//#define LOOP_ONKEYPRESSED     // loops only when a key is received on Serial
#define LOOP_DELAY  1000      // delay applied at each loop() [if LOOP_ONKEYPRESSED is commented]

// NOTE : choose below one of the output modes below
//#define OUTPUT_READABLE
#define OUTPUT_CSV

// NOTE : below are parameters for the CSV output
//#define OUTPUT_CSV_SEPARATOR  ';'    // semicolon
//#define OUTPUT_CSV_SEPARATOR  ','    // comma
#define OUTPUT_CSV_SEPARATOR  '\t'  // tab, to be copy-pasted into excel directly


// **********************
// *** SENSOR CLASSES ***
// **********************

// connect SCL to analog 5
// connect SDA to analog 4
// connect VDD to 3.3V DC
// connect GROUND to common ground
// ADDR can be connected to ground, or vdd or left floating to change the i2c address
SAT_Mag mag;


// *************
// *** SETUP ***
// *************

void setup(void) {
  Serial.begin(9600);
  Serial.println("Starting...");
  
  OBCL.begin();             // setups the communication via I2C (sensors)

  mag.configMag();          // turn the MAG3110 on
  
#ifdef OUTPUT_CSV
  // Printing the headers of the CSV format
  Serial.print("RAW_X");
  Serial.print(OUTPUT_CSV_SEPARATOR);
  Serial.print("RAW_Y");
  Serial.print(OUTPUT_CSV_SEPARATOR);
  Serial.print("RAW_Z");
  Serial.print(OUTPUT_CSV_SEPARATOR);
  Serial.print("OFFSET_X");
  Serial.print(OUTPUT_CSV_SEPARATOR);
  Serial.print("OFFSET_Y");
  Serial.print(OUTPUT_CSV_SEPARATOR);
  Serial.print("OFFSET_Z");
  Serial.print(OUTPUT_CSV_SEPARATOR);
  Serial.print("HEADING");
  Serial.println();
#endif
}


// ************
// *** LOOP ***
// ************

void loop(void) {
#ifdef LOOP_ONKEYPRESSED
  if (Serial.available() <= 0) {
    return;
  } else {
    delay(100); // to catch a whole line it several chars are sent in the same time
    while(Serial.available() > 0) { Serial.read(); };
  }
#else
  delay(LOOP_DELAY);
#endif  


  int16_t mag_x = mag.readx();
  int16_t mag_y = mag.ready();
  int16_t mag_z = mag.readz();
  float offset_x = mag.x_value();
  float offset_y = mag.y_value();
  float offset_z = mag.z_value();
  int heading = mag.getHeading(offset_x,offset_y,offset_z);  
  
#ifdef OUTPUT_READABLE
  Serial.print("raw X value: ");    Serial.println(mag_x);
  Serial.print("raw Y value: ");    Serial.println(mag_y);
  Serial.print("raw Z value: ");    Serial.println(mag_z);
  Serial.print("offset X value: "); Serial.println(offset_x);
  Serial.print("offset Y value: "); Serial.println(offset_y);
  Serial.print("offset Z value: "); Serial.println(offset_z);
  Serial.print("heading: ");        Serial.println(heading);
#endif
#ifdef OUTPUT_CSV
  Serial.print(mag_x);
  Serial.print(OUTPUT_CSV_SEPARATOR);
  Serial.print(mag_y);
  Serial.print(OUTPUT_CSV_SEPARATOR);
  Serial.print(mag_z);
  Serial.print(OUTPUT_CSV_SEPARATOR);
  Serial.print(offset_x);
  Serial.print(OUTPUT_CSV_SEPARATOR);
  Serial.print(offset_y);
  Serial.print(OUTPUT_CSV_SEPARATOR);
  Serial.print(offset_z);
  Serial.print(OUTPUT_CSV_SEPARATOR);
  Serial.print(heading);
  Serial.println();
#endif
}

