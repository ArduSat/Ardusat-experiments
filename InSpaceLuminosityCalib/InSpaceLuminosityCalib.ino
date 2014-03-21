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

    Description :  sketch that measures luminosity (2 sensors) under the 6 possible parameters (gain, integration time)
                   throws the raw values and the parameters values using format recommended by SAT_DataLib
                   https://github.com/jfomhover/ArduSat-utils
    Last Changed : Feb. 19, 2014

********************************************************************
*/


#include <Arduino.h>
#include <Wire.h>
#include <EEPROM.h>
#include <OnboardCommLayer.h>
#include <SAT_Lum.h>


// *************************
// *** EXPERIMENT CONFIG ***
// *************************

//#define DEBUG_MODE              // stores the values via Serial + SD file for debug
//#define ARDUSAT_PERIOD 5552     // approx from supposed TLEs
#define PULL_DELAY 140000       // data is pooled every PULL_DELAY milliseconds
                                // considering there is 6*20 bytes of data per pulling
                                // that should bring approx 85 packets of sensor values (in the 10ko limit)
                                // a delay of 140 seconds should cover approx 2.2 rotations around earth

// DEBUG_MODE CONFIG
// Choose one of the two possibilities below :
#define DEBUG_OUTPUT_SD                   // [CHOOSE ONE] output data on SD file 
//#define DEBUG_OUTPUT_SERIAL             // [CHOOSE ONE] output data on Serial port 

// NOTE : configuration of the debug mode for writing on SD card
#define DEBUG_SD_CS  4                    // chip select : pin of the chip select on your arduino config (uno ethernet = 4)
#define DEBUG_SD_FILENAME "LUMCALIB.BIN"  // filename : name of the file writen on SD


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

SAT_Lum tslA(1);  // TSL2561 #1 bottomplate camera (0x29)
SAT_Lum tslB(2);  // TSL2561 #2 bottomplate slit (0x39)


// ***********************
// *** DATA STRUCTURES ***
// ***********************

#define PACKET_HEADER_CHUNK  '#'
#define DATATYPE_MS              0x0001
#define DATATYPE_SAT_LUM1        0x0002
#define DATATYPE_SAT_LUM2        0x0004
#define DATATYPE_USERDEFINED1    0x2000


// structured packet of data, easy to decode after reception (see https://github.com/jfomhover/ArduSat-utils for syntax)
struct _dataChunk {
  char header;
  uint16_t datatypes;
  uint32_t ms;
  uint16_t tsl_one_values[2];  // DATATYPE_SAT_LUM1
  uint16_t tsl_two_values[2];  // DATATYPE_SAT_LUM2
  byte userdefinedblock[5];    // DATATYPE_USERDEFINED1
} data;

#define PACKET_SIZE sizeof(struct _dataChunk)

// erasing all values in the data chunk
void resetChunk() {
  byte * t_ptr = (byte*)&data;
  for (int i=0; i<PACKET_SIZE; i++)
    t_ptr[i] = 0x00;
}


// ************************
// *** SENSOR FUNCTIONS ***
// ************************

// SETUP OF THE SENSORS NEEDED (constructors mainly)
void setupSensors() {
  tslA.begin();  // TODO : error checking ?
  tslB.begin();  // TODO : error checking ?
}

byte config_switch = 0;
tsl2561Gain_t config_gain = SAT_Lum_GAIN_0X;
tsl2561IntegrationTime_t config_integration = SAT_Lum_INTEGRATIONTIME_13MS;

void setupConfig() {
  switch(config_switch) {
    case 0:
      config_gain = SAT_Lum_GAIN_0X;
      config_integration = SAT_Lum_INTEGRATIONTIME_13MS;
      break;
    case 1:
      config_gain = SAT_Lum_GAIN_0X;
      config_integration = SAT_Lum_INTEGRATIONTIME_101MS;
      break;
    case 2:
      config_gain = SAT_Lum_GAIN_0X;
      config_integration = SAT_Lum_INTEGRATIONTIME_402MS;
      break;
    case 3:
      config_gain = SAT_Lum_GAIN_16X;
      config_integration = SAT_Lum_INTEGRATIONTIME_13MS;
      break;
    case 4:
      config_gain = SAT_Lum_GAIN_16X;
      config_integration = SAT_Lum_INTEGRATIONTIME_101MS;
      break;
    case 5:
      config_gain = SAT_Lum_GAIN_16X;
      config_integration = SAT_Lum_INTEGRATIONTIME_402MS;
      break;
  };
};

// pulling values from the sensors and filling the data structure
void pullValues() {
  data.header = PACKET_HEADER_CHUNK;
  data.datatypes = DATATYPE_MS | DATATYPE_SAT_LUM1 | DATATYPE_SAT_LUM2 | DATATYPE_USERDEFINED1;
  
  setupConfig();
  tslA.setGain(config_gain);
  tslA.setTiming(config_integration);
  tslB.setGain(config_gain);
  tslB.setTiming(config_integration);

  data.ms = millis();
  uint32_t lumA = tslA.getFullLuminosity();
  uint16_t lumA_ir = lumA >> 16;
  uint16_t lumA_full = lumA & 0xFFFF;
  uint16_t lumA_visible = lumA_full - lumA_ir;

  data.tsl_one_values[0] = lumA_ir;
  data.tsl_one_values[1] = lumA_full;

  uint32_t lumB = tslB.getFullLuminosity();
  uint16_t lumB_ir = lumB >> 16;
  uint16_t lumB_full = lumB & 0xFFFF;
  uint16_t lumB_visible = lumB_full - lumB_ir;

  data.tsl_two_values[0] = lumB_ir;
  data.tsl_two_values[1] = lumB_full;
  
  data.userdefinedblock[0] = 2; // DATATYPE_USERDEFINED_HEX2; // will display in binary format on the decoder
  data.userdefinedblock[1] = config_gain;
  data.userdefinedblock[2] = config_integration;
  data.userdefinedblock[3] = 0x00;
  data.userdefinedblock[4] = 0x00;
};


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

  OBCL.begin();
  
  setupSensors();
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

  // IN THIS PARTICULAR EXPERIMENT
  // pull and send 6 times in a row (each config)
  for (config_switch=0; config_switch<6; config_switch++) {
    pullValues();   // pull the values needed

    byte * t_ptr = (byte *)&data;
    store.send(t_ptr, 0, PACKET_SIZE);   // sends data into the communication file and queue for transfer
                                       // WARNING : introduces a 100ms delay

    dataSent += PACKET_SIZE;
  }
  
/*
  if ((dataSent + PACKET_SIZE) > 10240) {
    // TODO : close experiments, clean mode ?
    // my understanding is : that will be done automatically when we reach the 10kb limit
  }
*/
}

