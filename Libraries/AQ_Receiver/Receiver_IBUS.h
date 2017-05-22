/*
 An Open Source Arduino based multicopter.
 
 This program is free software: you can redistribute it and/or modify 
 it under the terms of the GNU General Public License as published by 
 the Free Software Foundation, either version 3 of the License, or 
 (at your option) any later version. 
 
 This program is distributed in the hope that it will be useful, 
 but WITHOUT ANY WARRANTY; without even the implied warranty of 
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the 
 GNU General Public License for more details. 
 
 You should have received a copy of the GNU General Public License 
 along with this program. If not, see <http://www.gnu.org/licenses/>. 
 */

#ifndef _AEROQUAD_RECEIVER_IBUS_H_
#define _AEROQUAD_RECEIVER_IBUS_H_

#if defined (__AVR_ATmega1280__) || defined(__AVR_ATmega2560__) || defined (AeroQuadSTM32) || defined (DuePi)

#if !defined (AeroQuadSTM32) && !defined (DuePi)
  #include "Arduino.h"
  #include "pins_arduino.h"
  #include <AQMath.h>
  #include "GlobalDefined.h"
#endif
#include "Receiver.h"

#define IBUS_SYNCBYTE_0 0x20
#define IBUS_SYNCBYTE_1 0x40
#define IBUS_CHKSUM 0xFFFF  
#define FAILSAFELIMIT 1020                          // When all the 6 channels below this value assume failsafe
#define IBUS_BUFFSIZE 32                            // Max iBus packet size (2 byte header (0x20, 0x40), 14 channels x 2 bytes, 2 byte checksum (0xFF, 0XFF))
  
#define SERIAL_SBUS Serial3  

static unsigned int rcChannel[18] = {XAXIS,YAXIS,THROTTLE,ZAXIS,MODE,AUX1,AUX2,AUX3,AUX4,AUX5,10,11,12,13,14,15,16,17};
static unsigned int sbusIndex = 0;
static uint16_t rcValue[LASTCHANNEL];
static boolean failsafe = false;
static uint8_t ibusIndex = 0;
static uint8_t ibus[IBUS_BUFFSIZE] = {0};
static uint16_t chksum, rxsum;
//mode 2 default, Roll/XAXIS: 0, Pitch/YAXIS: 1, Yaw/ZAXIS: 2, Throttle: 3, Mode: 4
static uint8_t  RxPitch = YAXIS;
static uint8_t  RxThrottle = THROTTLE;


void initializeReceiver(int nbChannel = 10) {
    Serial3.begin(115200);
    #if defined Mode_1
		RxPitch = THROTTLE;
    	RxThrottle = YAXIS;
    #endif
    initializeReceiverParam(nbChannel);
}

bool readIBUS() {
	while(Serial3.available() > 0){
      uint8_t rx = Serial3.read();
      //  for start 0x20  as 0x40 packet
      if (ibusIndex == 0 && rx != IBUS_SYNCBYTE_0) {
       continue;
      }
      if (ibusIndex == 1 && rx != IBUS_SYNCBYTE_1) {
       ibusIndex = 0;
       continue;
      } 
    if (ibusIndex < IBUS_BUFFSIZE) ibus[ibusIndex] = rx;
    ibusIndex++;
    if (ibusIndex == IBUS_BUFFSIZE){
      ibusIndex = 0;
      chksum = IBUS_CHKSUM;
      for (uint8_t i = 0; i < 30; i++){
        chksum -= ibus[i];
      }
      rxsum = ibus[30] + (ibus[31] << 8);
      if (chksum == rxsum){
        rcChannel[XAXIS] = (ibus[ 3] << 8) + ibus[ 2];           // roll
        rcChannel[RxPitch] = (ibus[ 5] << 8) + ibus[ 4];         // pitch  
        rcChannel[RxThrottle]  = (ibus[ 7] << 8) + ibus[ 6];     // throttle
        rcChannel[ZAXIS] = (ibus[ 9] << 8) + ibus[ 8];           // yaw
        rcChannel[MODE] = (ibus[11] << 8) + ibus[10];
        rcChannel[AUX1] = (ibus[13] << 8) + ibus[12];
        rcChannel[AUX2] = (ibus[15] << 8) + ibus[14];
        rcChannel[AUX3] = (ibus[17] << 8) + ibus[16];
        rcChannel[AUX4] = (ibus[19] << 8) + ibus[18];
        rcChannel[AUX5] = (ibus[21] << 8) + ibus[20];           
        if (rcChannel[XAXIS] < FAILSAFELIMIT && rcChannel[YAXIS] < FAILSAFELIMIT &&
            rcChannel[THROTTLE] < FAILSAFELIMIT && rcChannel[ZAXIS] < FAILSAFELIMIT &&
            rcChannel[MODE] < FAILSAFELIMIT && rcChannel[AUX1] < FAILSAFELIMIT ) {
          failsafe = true;
          return false;             //  Error  
        }else{
          return true;             // OK 
        }
      }else{
        return false;               // Checksum error 
      }
    }
  }
  return false;                  // NO data
} 

bool getChannelValue(byte channel) {
	if (readIBUS()){ 
		for(byte channel = 0; channel < LASTCHANNEL; channel++) {
			receiverCommand[channel] = rcChannel[channel];	 
		}
		return true;               // OK RX data
	}
	return false;
}

void setChannelValue(byte channel, int value) {
}

#endif
#endif
