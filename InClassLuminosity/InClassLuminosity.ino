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
#include <SAT_Lum.h>


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

// NOTE : you can activate two luminosity sensors (like on the ArduSat)
#define USE_LUM_2  // to (also) activate the second luminosity sensor


// **********************
// *** SENSOR CLASSES ***
// **********************

// TSL2561 Wiring Instructions
// connect SCL to analog 5
// connect SDA to analog 4
// connect VDD to 3.3V DC
// connect GROUND to common ground
// to change the i2c address, ADDR can be connected to ground (0x29), or vdd (0x49) or left floating (0x39)

SAT_Lum tslA(1);  // TSL2561 #1 bottomplate camera (0x29)

#ifdef USE_LUM_2
SAT_Lum tslB(2);  // TSL2561 #2 bottomplate slit (0x39)
#endif


// *************
// *** SETUP ***
// *************

void setup(void) {
  Serial.begin(9600);
  Serial.println("Starting...");
  
  OBCL.begin();             // setups the communication via I2C (sensors)
  
  if (tslA.begin()) {
    Serial.println("Found sensor A");
  } else {
    Serial.println("No sensor A?");
//    while (1);
  }
#ifdef USE_LUM_2
  if (tslB.begin()) {
    Serial.println("Found sensor B");
  } else {
    Serial.println("No sensor B?");
//    while (1);
  }
#endif

  // You can change the gain on the fly, to adapt to brighter/dimmer light situations
  //tslA.setGain(SAT_Lum_GAIN_0X);         // set no gain (for bright situtations)
  tslA.setGain(SAT_Lum_GAIN_16X);      // set 16x gain (for dim situations)

  // Changing the integration time gives you a longer time over which to sense light
  // longer timelines are slower, but are good in very low light situtations!
  tslA.setTiming(SAT_Lum_INTEGRATIONTIME_13MS);  // shortest integration time (bright light)
  //tslA.setTiming(SAT_Lum_INTEGRATIONTIME_101MS);  // medium integration time (medium light)
  //tslA.setTiming(SAT_Lum_INTEGRATIONTIME_402MS);  // longest integration time (dim light)

#ifdef USE_LUM_2
  // You can change the gain on the fly, to adapt to brighter/dimmer light situations
  //tslB.setGain(SAT_Lum_GAIN_0X);         // set no gain (for bright situtations)
  tslB.setGain(SAT_Lum_GAIN_16X);      // set 16x gain (for dim situations)

  // Changing the integration time gives you a longer time over which to sense light
  // longer timelines are slower, but are good in very low light situtations!
  tslB.setTiming(SAT_Lum_INTEGRATIONTIME_13MS);  // shortest integration time (bright light)
  //tslB.setTiming(SAT_Lum_INTEGRATIONTIME_101MS);  // medium integration time (medium light)
  //tslB.setTiming(SAT_Lum_INTEGRATIONTIME_402MS);  // longest integration time (dim light)
#endif

  // Now we're ready to get readings!

#ifdef OUTPUT_CSV
  // Printing the headers of the CSV format
  Serial.print("LUM-A-Visible");
  Serial.print(OUTPUT_CSV_SEPARATOR);
  Serial.print("LUM-A-Infrared");
  Serial.print(OUTPUT_CSV_SEPARATOR);
  Serial.print("LUM-A-FullSpectrum");
  Serial.print(OUTPUT_CSV_SEPARATOR);
  Serial.print("LUM-A-Lux");
#ifdef USE_LUM_2
  Serial.print(OUTPUT_CSV_SEPARATOR);
  Serial.print("LUM-B-Visible");
  Serial.print(OUTPUT_CSV_SEPARATOR);
  Serial.print("LUM-B-Infrared");
  Serial.print(OUTPUT_CSV_SEPARATOR);
  Serial.print("LUM-B-FullSpectrum");
  Serial.print(OUTPUT_CSV_SEPARATOR);
  Serial.print("LUM-B-Lux");
#endif /* USE_LUM_2 */
  Serial.println();
#endif /* OUTPUT_CSV */
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

  // More advanced data read example. Read 32 bits with top 16 bits IR, bottom 16 bits full spectrum
  // This can take 13-402 milliseconds!
  uint32_t lumA = tslA.getFullLuminosity();
  uint16_t lumA_ir = lumA >> 16;
  uint16_t lumA_full = lumA & 0xFFFF;
  uint16_t lumA_visible = lumA_full - lumA_ir;

#ifdef OUTPUT_READABLE
  Serial.print("LUM-A-Visible: "); Serial.println(lumA_visible);
  Serial.print("LUM-A-IR: ");      Serial.println(lumA_ir);
  Serial.print("LUM-A-Full: ");    Serial.println(lumA_full);
  Serial.print("LUM-A-Lux: ");     Serial.println(tslA.calculateLux(lumA_full, lumA_ir));
#endif  /* OUTPUT_READABLE */
#ifdef OUTPUT_CSV
  Serial.print(lumA_visible);
  Serial.print(OUTPUT_CSV_SEPARATOR);
  Serial.print(lumA_ir);
  Serial.print(OUTPUT_CSV_SEPARATOR);
  Serial.print(lumA_full);
  Serial.print(OUTPUT_CSV_SEPARATOR);
  Serial.print(tslA.calculateLux(lumA_full, lumA_ir));
#ifndef USE_LUM_2
  Serial.println();
#endif
#endif  /* OUTPUT_CSV */

#ifdef USE_LUM_2
  // More advanced data read example. Read 32 bits with top 16 bits IR, bottom 16 bits full spectrum
  // This can take 13-402 milliseconds!
  uint32_t lumB = tslB.getFullLuminosity();
  uint16_t lumB_ir = lumB >> 16;
  uint16_t lumB_full = lumB & 0xFFFF;
  uint16_t lumB_visible = lumB_full - lumB_ir;

#ifdef OUTPUT_READABLE
  Serial.print("LUM-B-Visible: "); Serial.println(lumB_visible);
  Serial.print("LUM-B-IR: ");      Serial.println(lumB_ir);
  Serial.print("LUM-B-Full: ");    Serial.println(lumB_full);
  Serial.print("LUM-B-Lux: ");     Serial.println(tslB.calculateLux(lumB_full, lumB_ir));
#endif  /* OUTPUT_READABLE */
#ifdef OUTPUT_CSV
  Serial.print(OUTPUT_CSV_SEPARATOR);
  Serial.print(lumB_visible);
  Serial.print(OUTPUT_CSV_SEPARATOR);
  Serial.print(lumB_ir);
  Serial.print(OUTPUT_CSV_SEPARATOR);
  Serial.print(lumB_full);
  Serial.print(OUTPUT_CSV_SEPARATOR);
  Serial.print(tslB.calculateLux(lumB_full, lumB_ir));
  Serial.println();
#endif  /* OUTPUT_CSV */
#endif  /* USE_LUM_2 */

}

