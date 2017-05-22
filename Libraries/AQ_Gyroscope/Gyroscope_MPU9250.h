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

#ifndef _AEROQUAD_GYROSCOPE_MPU9250_COMMON_H_
#define _AEROQUAD_GYROSCOPE_MPU9250_COMMON_H_

float gyroRaw[3] = {0.0 ,0.0, 0.0};

#include <Gyroscope.h>
#include <SensorsStatus.h>
//#include <Platform_MPU9250.h>

#define GYRO_CALIBRATION_TRESHOLD 25

void initializeGyro(uint8_t wai) {
  //float rangeGyro = 2*1000.0;
  float rangeGyro = 250.0; // Valor inferior para detener la oscilación del marco
  gyroScaleFactor = radians(rangeGyro/65536.0);   //16-bit value (0-65536),
  //gyroScaleFactor =MPU9250G_1000dps; // 0.030487804878 dps/LSB

  if (wai == 0x71){
	if (calibrateGyro()){
	  vehicleState |= GYRO_DETECTED;
	 return;
	}
	if (calibrateGyro()){
	  vehicleState |= GYRO_DETECTED;
	 return;
	}
	if (calibrateGyro()){
	  vehicleState |= GYRO_DETECTED;
	}
  }
}

void gyroUpdateHeading()
{
  long int currentTime = micros();
  if (gyroRate[ZAXIS] > (float)radians(1.0) || gyroRate[ZAXIS] < (float)radians(-1.0)) {
    gyroHeading += gyroRate[ZAXIS] * ((currentTime - gyroLastMesuredTime) / 1000000.0);
  }
  gyroLastMesuredTime = currentTime;
}

void measureGyro() {
/*                                 READ GYROSCOPE
 * usage: call this function to read gyroscope data. Axis represents selected axis:
 * 0 -> X axis
 * 1 -> Y axis
 * 2 -> Z axis
 */

  mpu.read_gyro();
  float gyroADC[3];
  //gyroADC[XAXIS] = (gyroRaw[XAXIS] = mpu.gyro_data[0])  - gyroZero[XAXIS];
  //gyroADC[YAXIS] = gyroZero[YAXIS] - (gyroRaw[YAXIS]= mpu.gyro_data[1]);
  //gyroADC[ZAXIS] = gyroZero[ZAXIS] - (gyroRaw[ZAXIS]= mpu.gyro_data[2]);
  gyroADC[XAXIS] = (gyroRaw[XAXIS] = mpu.gyro_data_raw[0])  - gyroZero[XAXIS];
  gyroADC[YAXIS] = gyroZero[YAXIS] - (gyroRaw[YAXIS]= mpu.gyro_data_raw[1]);
  gyroADC[ZAXIS] = gyroZero[ZAXIS] - (gyroRaw[ZAXIS]= mpu.gyro_data_raw[2]);
   
  for (byte axis = 0; axis <= ZAXIS; axis++) {
    gyroRate[axis] = float (gyroADC[axis] * gyroScaleFactor);
  }
	gyroUpdateHeading();
}

void measureGyroSum() { 
  mpu.read_gyro();

  //gyroSample[XAXIS] += (gyroRaw[XAXIS]= mpu.gyro_data[0]);
  //gyroSample[YAXIS] += (gyroRaw[YAXIS]= mpu.gyro_data[1]);
  //gyroSample[ZAXIS] += (gyroRaw[ZAXIS]= mpu.gyro_data[2]);
  gyroSample[XAXIS] += (gyroRaw[XAXIS]= mpu.gyro_data_raw[0]);
  gyroSample[YAXIS] += (gyroRaw[YAXIS]= mpu.gyro_data_raw[1]);
  gyroSample[ZAXIS] += (gyroRaw[ZAXIS]= mpu.gyro_data_raw[2]);
  
  gyroSampleCount++;
}

void evaluateGyroRate() { //100Hz
  float gyroADC[3];

  gyroADC[XAXIS] = (gyroSample[XAXIS] / gyroSampleCount) - gyroZero[XAXIS];
  gyroADC[YAXIS] = gyroZero[YAXIS] - (gyroSample[YAXIS] / gyroSampleCount);
  gyroADC[ZAXIS] = gyroZero[ZAXIS] - (gyroSample[ZAXIS] / gyroSampleCount);

  gyroSample[XAXIS] = 0;
  gyroSample[YAXIS] = 0;
  gyroSample[ZAXIS] = 0;
  gyroSampleCount = 0;

  for (byte axis = 0; axis <= ZAXIS; axis++) {
    gyroRate[axis] = float (gyroADC[axis]) * gyroScaleFactor;
    //gyroRate[axis] = gyroADC[axis];
  }
  gyroUpdateHeading();
}

boolean calibrateGyro() {
  int findZero[FINDZERO];
  int diff = 0; 
  for (byte axis = 0; axis < 3; axis++) {
    for (int i=0; i<FINDZERO; i++) {
      mpu.read_gyro();
      if(axis == XAXIS) {
    	findZero[i] = mpu.gyro_data[0];
    	//findZero[i] = mpu.gyro_data_raw[0];
      } 
	  else if(axis == YAXIS) {
    	findZero[i] = mpu.gyro_data[1];
    	//findZero[i] = mpu.gyro_data_raw[1];
      } 
	  else {
    	findZero[i] = mpu.gyro_data[2];
    	//findZero[i] = mpu.gyro_data_raw[2];
      }
      delayMicroseconds(10000);                                         // 49*10ms measurements (490ms)
    }
    #if defined watchdog
		watchdogReset();                                                // Reset watchdog
    #endif
    int tmp = findMedianIntWithDiff(findZero, FINDZERO, &diff);
	if (diff <= GYRO_CALIBRATION_TRESHOLD) { 
	  gyroZero[axis] = tmp;
	} 
	else {
		watchdogReset();                                                // Reset watchdog
		return false; //Calibration failed.
	}
  }
  return true;        //Calibration successfull.
}

#endif
