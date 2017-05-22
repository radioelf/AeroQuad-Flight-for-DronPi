/*
  Radioelf    
  
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

// Magnetic flux density [μT] Min.-4912  Max.4912 (-49,12 a 49,12 Gauss)

#ifndef _AEROQUAD_MAGNETOMETER_MPU9250_H_
#define _AEROQUAD_MAGNETOMETER_MPU9250_H_

#include "Compass.h"
#include <SensorsStatus.h>

#include "Arduino.h"
float mRes;
void initializeMagnetometer() {
	if (wai == 0x71){
	  delayMicroseconds(10000);                             // Power up delay **
  mpu.calib_mag();
  vehicleState |= MAG_DETECTED;
	mpu.calib_acc();
  }else{
    mpu.init(true);
      delayMicroseconds(10000);                             // Power up delay **
  mpu.calib_mag();
  }
  //mRes = 10.*4912./8190.;                   				// Proper scale to return milliGauss for 14 bits (default)
  mRes = 10.*4912./32760.0;                   				// Proper scale to return milliGauss for 16 bits
  delayMicroseconds(10000);                               	// Power up delay **
  mpu.calib_mag();	
 
  measureMagnetometer(0.0, 0.0);              				// Assume 1st measurement at 0 degrees roll and 0 degrees pitch
}

void measureMagnetometer(float roll, float pitch) {
  mpu.read_mag();
   
  rawMag[YAXIS] =  mpu.mag_data_raw[0];
  rawMag[ZAXIS] =  mpu.mag_data_raw[1];
  rawMag[XAXIS] =  mpu.mag_data_raw[2];

  measuredMagX = rawMag[XAXIS] + magBias[XAXIS];
  measuredMagY = rawMag[YAXIS] + magBias[YAXIS];
  measuredMagZ = rawMag[ZAXIS] + magBias[ZAXIS];
  
  measuredMag[XAXIS] = measuredMagX * mRes;
  measuredMag[YAXIS] = measuredMagY * mRes;
  measuredMag[ZAXIS] = measuredMagZ * mRes;
  
  const float cosRoll =  cos(roll);
  const float sinRoll =  sin(roll);
  const float cosPitch = cos(pitch);
  const float sinPitch = sin(pitch);

  const float magX = (float)measuredMagX * cosPitch + 
                     (float)measuredMagY * sinRoll * sinPitch + 
                     (float)measuredMagZ * cosRoll * sinPitch;
           
  const float magY = (float)measuredMagY * cosRoll - 
                     (float)measuredMagZ * sinRoll;

  const float tmp  = sqrt(magX * magX + magY * magY);
   
  hdgX = magX / tmp;
  hdgY = -magY / tmp;
}

#endif
