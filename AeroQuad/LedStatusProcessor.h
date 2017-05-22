/*
  AeroQuad v3.0 - Febuary 2012
  www.AeroQuad.com
  Copyright (c) 2011 Ted Carancho.  All rights reserved.
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

// Led Status Processor controls the LED:s on the shield according to vehicle status

#ifndef _AQ_LedProcessor_H_
#define _AQ_LedProcessor_H_

uint8_t flashingLedState = 0; // this counter increments by one at 10Hz

void processLedStatus() {

  //
  // process ready state light in case we use GPS
  //
  #if defined (UseGPS)
    if (haveAGpsLock()) {
      if (isHomeBaseInitialized()) {
        digitalWrite(LED_GREEN_PIN, HIGH);
      }
      else {
        digitalWrite(LED_GREEN_PIN, (flashingLedState & 4));
      }
    }
    else { 
      digitalWrite(LED_GREEN_PIN, (flashingLedState & 2));
    }
  #endif
  
  //
  // process ready state light in case we use Batt monitor
  //
  #if defined (BattMonitor)
    if (batteryAlarm) {
    #ifdef DuePi
        alarm_battery++;
      }else{
        alarm_battery =0;
      }
      if (alarm_battery >100){
    #endif
      } else if (batteryWarning) {
        digitalWrite(LED_BLUE_PIN, (flashingLedState & 15)==0);
        #if defined BUZZER       
          if (cicles_buzzer ==0) buzzer(500, 0);                          
        #endif
      } else { 
        #ifdef DuePi
          digitalWrite(LED_BLUE_PIN, HIGH);
        #else
          digitalWrite(LED_BLUE_PIN, LOW);
        #endif
        #if defined BUZZER
          if (cicles_buzzer ==256) cicles_buzzer =0;
        #endif
      }
  #endif  

  //
  // process mode light
  //
  if (flightMode == ATTITUDE_FLIGHT_MODE) {
    #ifdef DuePi
     digitalWrite(LED_WHITE_PIN , LOW);
    #else
     digitalWrite(LED_WHITE_PIN , HIGH);
    #endif 
  }
  else {
    #ifdef DuePi
      digitalWrite(LED_WHITE_PIN , HIGH);
    #else
      digitalWrite(LED_WHITE_PIN , LOW);
    #endif 
  }

  flashingLedState++;
}

#endif
