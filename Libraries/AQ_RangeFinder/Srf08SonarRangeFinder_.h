/*
  AeroQuad v3.0.1 - Mod. Radioelf 2017
  www.AeroQuad.com
  Copyright (c) 2012 Ted Carancho.  All rights reserved.
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

#ifndef _AEROQUAD_SRF08_SONAR_RANGE_FINDER_H_
#define _AEROQUAD_SRF08_SONAR_RANGE_FINDER_H_

#include "RangeFinder.h"

#define RANGE_SRF08 0x8C                                                // 6mts, scope =(range  x43mm) + 43mm), max 11mts 0xFF
#define GAIN_SRF08 0x31                                                 // Analog gain 1025 (default), min 0->94, max 31->1025 (31*70us = 2170us)
#define CMS 0x51                                                        // Centimeter measure

int reading =0;
bool SRF08State;
//address I2C SRF08
// 0xE0, 0xE2, 0xE4, 0xE6, 0xE8, 0xEA, 0xEC, 0xEE, 0xF0, 0xF2, 0xF4, 0xF6, 0xF8, 0xFA, 0xFC o 0xFE -> 8bits 
// 0x70, 0x71, 0x72, 0x73, 0x74, 0x75, 0x76, 0x77, 0x78, 0x79, 0x7A, 0x7B, 0x7C, 0x7D, 0x7E o 0x7F -> 7bits + W/R
uint8_t SRF_ADDRESS = 0x71;      
uint8_t rangerWaitCycles = 3;                                           // 4*20 =80ms

// get command
void SRF08_Command(uint8_t Command){
  Wire.beginTransmission(uint8_t (SRF_ADDRESS));
  Wire.write(uint8_t(0x00));                                            // Dirección interna 0x00 (registro de comandos)
  Wire.write(Command);                                                  // Enviar comando a ejecutar
  Wire.endTransmission(); 
  //delayMicroseconds(70000);                                           // Temporización de 70mS, necesario >65ms                                  
}
// get config. gain
void Conf_Gain(uint8_t gain){
  Wire.beginTransmission(uint8_t (SRF_ADDRESS)); 
  Wire.write(uint8_t(0x01));                         
  Wire.write(uint8_t(gain));                 
  Wire.endTransmission();   
}
// get config. range
void Conf_Range(uint8_t range){
  Wire.beginTransmission(uint8_t (SRF_ADDRESS)); 
  Wire.write(uint8_t(0x02));                         
  Wire.write(uint8_t(range));                 
  Wire.endTransmission();   
}
// change address I2C
void changeAddress(uint8_t NEW_ADDRESS){  
  Wire.beginTransmission(uint8_t(SRF_ADDRESS));  
  Wire.write(uint8_t(0x00));                                 
  Wire.write(uint8_t(0xA0));  
  Wire.endTransmission();  
 
  Wire.beginTransmission(uint8_t(SRF_ADDRESS));  
  Wire.write(uint8_t(0x00));  
  Wire.write(uint8_t(0xAA));  
  Wire.endTransmission();  
 
  Wire.beginTransmission(uint8_t(SRF_ADDRESS));  
  Wire.write(uint8_t(0x00));  
  Wire.write(uint8_t(0xA5));  
  Wire.endTransmission();  
 
  Wire.beginTransmission(uint8_t(SRF_ADDRESS));  
  Wire.write(uint8_t(0x00)); 
  //Wire.write(SRF_ADDRESS);                                            // address 8bits 
  Wire.write(SRF_ADDRESS *2);                                           // pasamos de 7bits a 8bits
  Wire.endTransmission();  
  SRF_ADDRESS = NEW_ADDRESS;
  delayMicroseconds(100000);                                        
}
// read mesure
int RX_SRF08(){                                                                                
  Wire.beginTransmission(uint8_t(SRF_ADDRESS));             
  Wire.write(uint8_t(0x02));                           
  Wire.endTransmission();
  
  Wire.requestFrom(uint8_t(SRF_ADDRESS), uint8_t (2));                  // peticion 2 bytes 
  while(Wire.available() < 2);                     
   reading = Wire.read();  
   reading =  reading << 8;    
   reading |= Wire.read(); 
   return (reading);                                
}
// read LDR, 0x00 min, 0xF8 max
uint8_t getLight(){  
  SRF08_Command(CMS);  
  delayMicroseconds(70000);                            
  Wire.beginTransmission(uint8_t(SRF_ADDRESS));
  Wire.write(uint8_t(0x01));                           
  Wire.endTransmission();
  
  Wire.requestFrom(uint8_t (SRF_ADDRESS),uint8_t (1));                  // peticion 1 byte
  while(Wire.available() < 0);  
  byte LDR = Wire.read();                  
  return(round (LDR/2.55));                                             // Returns % LDR
}
// read software version
uint8_t getSoft(){                                     
  Wire.beginTransmission(uint8_t(SRF_ADDRESS));             
  Wire.write(uint8_t(0x00));                                 
  Wire.endTransmission();
  
  Wire.requestFrom(uint8_t(SRF_ADDRESS),uint8_t (1));                   // peticion 1 byte
  while(Wire.available() < 0);                                         
  return(Wire.read());                               
}

void inititalizeRangeFinders() {
  uint8_t version = getSoft();
  if (version !=255 && version !=0){	
	if (GAIN_SRF08 !=0x31){
	 Conf_Gain(GAIN_SRF08);
	 delayMicroseconds(70000);
	}
	Conf_Range(RANGE_SRF08);
	SRF08State = true;
  }else{
    SRF08State = false;
	return;
   }
}

void updateRangeFinders() {                                             // call 20 ms->50hz
  if (rangerWaitCycles == 4) SRF08_Command(CMS);                        // 80ms, run read
  if (rangerWaitCycles !=0) {
	  rangerWaitCycles--;
	  return;
  }
  rangerWaitCycles = 8;                                                 // 160ms, 6.25 Hz update data
	
  // int range = RX_SRF08()*10;                                         // mm
  int range = RX_SRF08();                                               // Cm
  rangeFinderRange[0] = float (range/100.0);                            // Cm to Mts 

}
#endif 









