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

Last changed : Feb 19, 2014

********************************************************************
*/

// *************************
// *** EXPERIMENT CONFIG ***
// *************************

//#define DEBUG_MODE              // prints the values via Serial
//#define ARDUSAT_PERIOD 5552     // approx from supposed TLEs
#define PULL_DELAY 27000        // data is pooled every PULL_DELAY milliseconds
                                // considering there is 15 bytes of data per pulling
                                // that should bring approx 682 packets of sensor values (in the 10ko limit)
                                // 27 secs should cover approx 3.3 periods around earth

// DEBUG_MODE CONFIG
// Choose one of the two possibilities below :
#define DEBUG_OUTPUT_SD                   // [CHOOSE ONE] output data on SD file 
//#define DEBUG_OUTPUT_SERIAL             // [CHOOSE ONE] output data on Serial port 

// NOTE : configuration of the debug mode for writing on SD card
#define DEBUG_SD_CS  4                    // chip select : pin of the chip select on your arduino config (uno ethernet = 4)
#define DEBUG_SD_FILENAME "COLDSNAP.BIN"  // filename : name of the file writen on SD


// ***************
// *** HEADERS ***
// ***************

#include <Arduino.h>
#include <Wire.h>
#include <EEPROM.h>
#include <OnboardCommLayer.h>
#include <SAT_Temp.h>


// **********************
// *** STORAGE HEADER ***
// **********************

#ifndef DEBUG_MODE
#include <SAT_AppStorage.h>
#endif

// DEBUG_MODE switch
#if defined(DEBUG_MODE)
#if defined(DEBUG_OUTPUT_SERIAL)
#include <SAT_AppStorageEMU.h>  // experiment data channel (via Serial, see library at https://github.com/jfomhover/ArduSat-utils)
#elif defined(DEBUG_OUTPUT_SD)
#include <SD.h>
#include <SAT_AppStorageEMUSD.h>   // experiment data channel (via SD, see library at https://github.com/jfomhover/ArduSat-utils)
#endif
#endif  // DEBUG_MODE



// *********************
// *** SDK INSTANCES ***
// *********************

#ifndef DEBUG_MODE
  SAT_AppStorage store;       // experiment data channel (via comm)
#endif

#if defined(DEBUG_MODE)
#if defined(DEBUG_OUTPUT_SERIAL)
  SAT_AppStorageEMU store;    // experiment data channel (via Serial, see library at https://github.com/jfomhover/ArduSat-utils )
#elif defined(DEBUG_OUTPUT_SD)
  SAT_AppStorageEMUSD store;    // experiment data channel (via Serial, see library at https://github.com/jfomhover/ArduSat-utils )
#endif
#endif  // DEBUG_MODE

SAT_Temp tmp_sensor1(1);    // supposedly "payload"
SAT_Temp tmp_sensor2(2);    // unknown position
SAT_Temp tmp_sensor3(3);    // supposedly "bottomplate"
SAT_Temp tmp_sensor4(4);    // unknown position


// ***********************
// *** DATA STRUCTURES ***
// ***********************

#define PACKET_HEADER_CHUNK  '#'
#define DATATYPE_MS              0x0001
#define DATATYPE_SAT_TMP1        0x0010
#define DATATYPE_SAT_TMP2        0x0020
#define DATATYPE_SAT_TMP3        0x0040
#define DATATYPE_SAT_TMP4        0x0080

#define PACKET_SIZE 15 // don't trust sizeof() sometimes

// structured packet of data, easy to decode after reception (see https://github.com/jfomhover/ArduSat-utils for syntax)
struct _dataChunk {
  char header;
  uint16_t datatypes;
  uint32_t ms;
  int16_t tmp1;
  int16_t tmp2;
  int16_t tmp3;
  int16_t tmp4;
} data;

// erasing all values in the data chunk
void resetChunk() {
  byte * t_ptr = (byte*)&data;
  for (int i=0; i<PACKET_SIZE; i++)
    t_ptr[i] = 0x00;
}


// *************************
// *** PULLING FUNCTIONS ***
// *************************

// pulling values from the sensors and filling the data structure
void pullValues() {
  data.header = PACKET_HEADER_CHUNK;
  data.datatypes = DATATYPE_MS | DATATYPE_SAT_TMP1 | DATATYPE_SAT_TMP2 | DATATYPE_SAT_TMP3 | DATATYPE_SAT_TMP4;
  data.ms = millis();
  data.tmp1 = tmp_sensor1.getRawTemp();
  data.tmp2 = tmp_sensor2.getRawTemp();
  data.tmp3 = tmp_sensor3.getRawTemp();
  data.tmp4 = tmp_sensor4.getRawTemp();
}


// ******************
// *** MAIN SETUP ***
// ******************

void setup() {

#if defined(DEBUG_MODE)
  Serial.begin(9600);
#if defined(DEBUG_OUTPUT_SERIAL)
  store.init( true ); // debug : outputing verbose lines on Serial
#elif defined(DEBUG_OUTPUT_SD)
  store.init( true,               // debug mode : outputs strings on Serial
              DEBUG_SD_CS,                  
              false,               // append : true if data sent is appended at the end of the existing file)
              DEBUG_SD_FILENAME);   
#endif
#endif  // DEBUG_MODE

  OBCL.begin();    // begins the communication via I2C
}


// *****************
// *** MAIN LOOP ***
// *****************
 
signed long int previousMs = 0;
signed long int nextMs = 0;
int dataSent = 0;

void loop() {
  resetChunk();      // zeroes the data structure

  nextMs = PULL_DELAY-(millis()-previousMs);  // "(not so) intelligent delay" : just the ms needed to have a regular timing, takes into account the delay of all the functions in the loop
  previousMs += PULL_DELAY;
  
  if (nextMs > 0)
    delay(nextMs); //wait for next pull
  else
    previousMs += PULL_DELAY; // we missed a beat, skip one to get back on regular track

  pullValues();   // pull the values needed

  byte * t_ptr = (byte *)&data;
  store.send(t_ptr, 0, PACKET_SIZE);   // sends data into the communication file and queue for transfer
                                       // WARNING : introduces a 100ms delay

  dataSent += PACKET_SIZE;
  
/*
  if ((dataSent + PACKET_SIZE) > 10240) {
    // TODO : close experiments, clean mode ?
    // my understanding is : that will be done automatically when we reach the 10kb limit
  }
*/
}

