/*
  AeroQuad v3.0 - March 2011
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

#include <Wire.h>

#include <AQMath.h>
#include <Device_I2C.h>
#include <Gyroscope_ITG3200.h>
#include <GlobalDefined.h>

#define ENABL_PIN 31

unsigned long timer;


void setup()
{
  pinMode(ENABL_PIN, OUTPUT);
  digitalWrite(ENABL_PIN, HIGH); // habilitamos placa DronPi
  Serial.begin(115200);
  Wire.begin();
  delay (250);
  Serial.print("Gyroscope library test (ITG3200) direccion I2C:0X");
  Serial.println (ITG3200_ADDRESS, HEX);
  Serial.print("Respuesta del ITG3200:0x");
  byte OK = (readWhoI2C(ITG3200_ADDRESS));
  Serial.println(OK, HEX);
  if ((OK & ITG3200_IDENTITY_MASK) == ITG3200_IDENTITY) {
    Serial.println("Respuesta correcta");
  }else{
    Serial.println("!Error!");
    while (true){}
  }
  
  initializeGyro();
  calibrateGyro();
  delay (5000);
  timer = millis();
}

void loop(void) 
{
  if((millis() - timer) > 10) // 100Hz
  {
    timer = millis();
    measureGyro();
    
    Serial.print("Roll: ");
    Serial.print(degrees(gyroRate[XAXIS]));
    Serial.print(" Pitch: ");
    Serial.print(degrees(gyroRate[YAXIS]));
    Serial.print(" Yaw: ");
    Serial.print(degrees(gyroRate[ZAXIS]));
    Serial.print(" Heading: ");
    Serial.print(degrees(gyroHeading));
    Serial.println();
  }
}

