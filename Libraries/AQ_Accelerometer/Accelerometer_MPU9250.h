/*
  AeroQuad v3.0 - May 2011
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

#ifndef _AEROQUAD_ACCELEROMETER_MPU9250_COMMON_H_
#define _AEROQUAD_ACCELEROMETER_MPU9250_COMMON_H_

#include <Accelerometer.h>
#include <SensorsStatus.h>

//float accelSample_MPU[3] = {0,0,0};

void initializeAccel() {
  if (wai == 0x71){
	mpu.calib_acc();
	vehicleState |= ACCEL_DETECTED;
  }
}

/*                                 READ ACCELEROMETER
 * usage: call this function to read accelerometer data. Axis represents selected axis:
 * 0 -> X axis
 * 1 -> Y axis
 * 2 -> Z axis
 */
void measureAccel() {
  mpu.read_acc();

  //meterPerSecSec[XAXIS] = mpu.accel_data[0] * accelScaleFactor[XAXIS] + runTimeAccelBias[XAXIS];
  //meterPerSecSec[YAXIS] = mpu.accel_data[1] * accelScaleFactor[YAXIS] + runTimeAccelBias[YAXIS];
  //meterPerSecSec[ZAXIS] = mpu.accel_data[2] * accelScaleFactor[ZAXIS] + runTimeAccelBias[ZAXIS];
  meterPerSecSec[XAXIS] = mpu.accel_data_raw[0] * accelScaleFactor[XAXIS] + runTimeAccelBias[XAXIS];
  meterPerSecSec[YAXIS] = mpu.accel_data_raw[1] * accelScaleFactor[YAXIS] + runTimeAccelBias[YAXIS];
  meterPerSecSec[ZAXIS] = mpu.accel_data_raw[2] * accelScaleFactor[ZAXIS] + runTimeAccelBias[ZAXIS];

}

void measureAccelSum() {
  mpu.read_acc();
  //accelSample[XAXIS] += mpu.accel_data[0];
  //accelSample[YAXIS] += mpu.accel_data[1];
  //accelSample[ZAXIS] += mpu.accel_data[2];
  accelSample[XAXIS] += mpu.accel_data_raw[0];
  accelSample[YAXIS] += mpu.accel_data_raw[1];
  accelSample[ZAXIS] += mpu.accel_data_raw[2];

  accelSampleCount++;
}

void evaluateMetersPerSec() {
  for (byte axis = XAXIS; axis <= ZAXIS; axis++) {
    meterPerSecSec[axis] = (accelSample[axis] / accelSampleCount) * accelScaleFactor[axis] + runTimeAccelBias[axis];//

  	accelSample[axis] = 0.0;
  	
  }
  accelSampleCount = 0;
}

void computeAccelBias() { // SAMPLECOUNT->400
  for (int samples = 0; samples < SAMPLECOUNT; samples++) {
    measureAccelSum();
    delayMicroseconds(2500);
  }

  for (byte axis = 0; axis < 3; axis++) {
    meterPerSecSec[axis] = (float(accelSample[axis])/SAMPLECOUNT) * accelScaleFactor[axis];
    accelSample[axis] = 0;
  }
  accelSampleCount = 0;

  runTimeAccelBias[XAXIS] = -meterPerSecSec[XAXIS];
  runTimeAccelBias[YAXIS] = -meterPerSecSec[YAXIS];
  runTimeAccelBias[ZAXIS] = -9.8065 - meterPerSecSec[ZAXIS];

  accelOneG = abs(meterPerSecSec[ZAXIS] + runTimeAccelBias[ZAXIS]);
}


#endif
