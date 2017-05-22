/*
  AeroQuad v3.0.1 - February 2012
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

// parts of this code were taken from AN520, an early version of fabio's library and the AQ BMP085 code

#ifndef _AQ_BAROMETRIC_SENSOR_MS5611_SPI
#define _AQ_BAROMETRIC_SENSOR_MS5611_SPI

#include "BarometricSensor.h"
#include <AQMath.h>

//comand SPI
#define CMD_MS5611_RESET 0x1E
#define CMD_CONVERT_D1_OSR256  0x40
#define CMD_CONVERT_D1_OSR512  0x42
#define CMD_CONVERT_D1_OSR1024 0x44
#define CMD_CONVERT_D1_OSR2048 0x46
#define CMD_CONVERT_D1_OSR4096 0x48                               // Maximun resolution
#define CMD_CONVERT_D2_OSR256  0x50
#define CMD_CONVERT_D2_OSR512  0x52
#define CMD_CONVERT_D2_OSR1024 0x54
#define CMD_CONVERT_D2_OSR2048 0x56
#define CMD_CONVERT_D2_OSR4096 0x58                               // Maximun resolution
#define CMD_MS5611_ADC 0x00
#define CMD_MS5611_PROM_Setup 0xA0
#define CMD_MS5611_PROM_C1 0xA2
#define CMD_MS5611_PROM_C2 0xA4
#define CMD_MS5611_PROM_C3 0xA6
#define CMD_MS5611_PROM_C4 0xA8
#define CMD_MS5611_PROM_C5 0xAA
#define CMD_MS5611_PROM_C6 0xAC
#define CMD_MS5611_PROM_CRC 0xAE

// D1 and D2 result size (bytes)
#define MS561101BA_D1D2_SIZE       3

// OSR (Over Sampling Ratio) constants
#define MS561101BA_OSR_256         0x00
#define MS561101BA_OSR_512         0x02
#define MS561101BA_OSR_1024        0x04
#define MS561101BA_OSR_2048        0x06
#define MS561101BA_OSR_4096        0x08

long MS5611lastRawTemperature;
long MS5611lastRawPressure;
int64_t MS5611_sens=0;
int64_t MS5611_offset=0;
unsigned short MS5611Prom[8];

uint32_t D1, D2;
uint8_t  MS5611_State;

float temperatura =0;//NULL;
float presion = 0;//NULL;

long rawPressure         = 0;
long rawTemperature      = 0;
long rawPressureSum      = 0;
byte pressureCount       = 0;
float pressureFactor     = 1/5.255;
boolean isReadPressure   = false;
byte rawPressureSumCount = 0;
bool _updated =false;

uint8_t MS5611_SPI_read(byte reg)
{
  byte dump;
  uint8_t return_value;
  digitalWrite(CS_BARO_PIN, LOW);
  dump = SPI.transfer(reg);
  return_value = SPI.transfer(0);
  digitalWrite(CS_BARO_PIN, HIGH);
  return(return_value);
}
uint16_t MS5611_SPI_read_16bits(byte reg)
{
  byte dump,byteH,byteL;
  uint16_t return_value;
  digitalWrite(CS_BARO_PIN, LOW);
  dump = SPI.transfer(reg);
  byteH = SPI.transfer(0);
  byteL = SPI.transfer(0);
  digitalWrite(CS_BARO_PIN, HIGH);
  return_value = ((uint16_t)byteH<<8) | (byteL);
  return(return_value);
}

uint32_t MS5611_SPI_read_24bits(byte reg)
{
  byte dump,byteH,byteM,byteL;
  uint32_t return_value;
  digitalWrite(CS_BARO_PIN, LOW);
  dump = SPI.transfer(reg);
  byteH = SPI.transfer(0);
  byteM = SPI.transfer(0);
  byteL = SPI.transfer(0);
  digitalWrite(CS_BARO_PIN, HIGH);
  return_value = (((uint32_t)byteH)<<16) | (((uint32_t)byteM)<<8) | (byteL);
  return(return_value);
}

uint32_t MS5611_SPI_read_ADC()
{
  byte dump, byteH, byteM, byteL;
  uint32_t return_value;
  digitalWrite(CS_BARO_PIN, LOW);
  dump = SPI.transfer(0x00);
  byteH = SPI.transfer(0);
  byteM = SPI.transfer(0);
  byteL = SPI.transfer(0);
  digitalWrite(CS_BARO_PIN, HIGH);
  return_value = (((uint32_t)byteH)<<16) | (((uint32_t)byteM)<<8) | (byteL);
  return(return_value);
}

void MS5611_SPI_write(byte reg)
{
  byte dump;
  digitalWrite(CS_BARO_PIN, LOW);
  dump = SPI.transfer(reg);
  digitalWrite(CS_BARO_PIN, HIGH);
}

//**************************************************************************
// taken from AN520
unsigned char MS5611crc4(unsigned short n_prom[])
{
	unsigned short n_rem = 0;               // crc reminder
	unsigned short crc_read;            	// original value of the crc
	crc_read  = n_prom[7];               	//save read CRC
	n_prom[7] = (0xFF00 & (n_prom[7])); 	//CRC byte is replaced by 0
	for (int cnt = 0; cnt < 16; cnt++) {   	// operation is performed on bytes
	    // choose LSB or MSB
		if (cnt%2 == 1) {
			n_rem ^= (n_prom[cnt>>1]) & 0x00FF;
		} 
		else {
			n_rem ^= n_prom[cnt>>1] >> 8;
		}
		for (int n_bit = 8; n_bit > 0; n_bit--) {
			if (n_rem & (0x8000)) {
				n_rem = (n_rem << 1) ^ 0x3000;
			} 
			else {
				n_rem = (n_rem << 1);
			}
		}
	}
	n_rem = (n_rem >> 12) & 0xF; // // final 4-bit reminder is CRC code
	n_prom[7] = crc_read; // restore the crc_read to its original place
	return (n_rem);
}

bool MS5611readPROM()
{
	MS5611Prom[0] = MS5611_SPI_read_16bits(CMD_MS5611_PROM_Setup);
	MS5611Prom[1] = MS5611_SPI_read_16bits(CMD_MS5611_PROM_C1);
	MS5611Prom[2] = MS5611_SPI_read_16bits(CMD_MS5611_PROM_C2);
	MS5611Prom[3] = MS5611_SPI_read_16bits(CMD_MS5611_PROM_C3);
	MS5611Prom[4] = MS5611_SPI_read_16bits(CMD_MS5611_PROM_C4);
	MS5611Prom[5] = MS5611_SPI_read_16bits(CMD_MS5611_PROM_C5);
	MS5611Prom[6] = MS5611_SPI_read_16bits(CMD_MS5611_PROM_C6);
	MS5611Prom[7] = MS5611_SPI_read_16bits(CMD_MS5611_PROM_CRC);

	MS5611_SPI_write(CMD_CONVERT_D2_OSR4096);
	temperatura = 0;
	presion = 0;
  
	int crc     = MS5611crc4(MS5611Prom);//crc <-6
	
	int crcProm = MS5611Prom[7] & 0xf;

	if(crc == crcProm) {
		return 1;
	}
	return 0;
}

void MS5611reset()
{
	MS5611_SPI_write(CMD_MS5611_RESET);
}
// Calcular la temperatura y la presión compensada en unidades reales (grados centigrados*100, mbar*100)
void calculate()
{
  int32_t dT;
  int64_t TEMP;  // 64 bits
  int64_t OFF1;
  int64_t SENS;
  int64_t P;

  // Formulas from manufacturer datasheet
  // as per data sheet some intermediate results require over 32 bits, therefore
  // we define parameters as 64 bits to prevent overflow on operations
  // sub -20c temperature compensation is not included
  dT = D2-((long)MS5611Prom[5]*256);
  TEMP = 2000 + ((int64_t)dT * MS5611Prom[6])/8388608;
  OFF1 = (int64_t)MS5611Prom[2] * 65536 + ((int64_t)MS5611Prom[4] * dT ) / 128;
  SENS = (int64_t)MS5611Prom[1] * 32768 + ((int64_t)MS5611Prom[3] * dT) / 256;

  if (TEMP < 2000){   // second order temperature compensation
    int64_t T2 = (((int64_t)dT)*dT) >> 31;
    int64_t Aux_64 = (TEMP-2000)*(TEMP-2000);
    int64_t OFF2 = (5*Aux_64)>>1;
    int64_t SENS2 = (5*Aux_64)>>2;
    TEMP = TEMP - T2;
    OFF1 = OFF1 - OFF2;
    SENS = SENS - SENS2;
  }
  P = (D1*SENS/2097152 - OFF1)/32768;
  	rawTemperature = TEMP;
	rawPressure = P;
}

// maquina de 4 estados
uint8_t ReadMS5611()
{
  uint8_t result = 0;
  if (MS5611_State == 1){                       // estado 1
    
      D2 =MS5611_SPI_read_ADC();          
      MS5611_State++;
      MS5611_SPI_write(CMD_CONVERT_D1_OSR4096);  // Comando para leer la presión  
    
  }else{
    if (MS5611_State == 5){                      // estado 5  
        D1 = MS5611_SPI_read_ADC();
        calculate();
        MS5611_State = 1;                         // Comienzar de nuevo desde el estado = 1
        MS5611_SPI_write(CMD_CONVERT_D2_OSR4096); // Comando para leer la temperatura
        result = 1;                               // Nueva lectura de presión
      
    }else{                                        // estados 2, 3, 4 
        D1 = MS5611_SPI_read_ADC();
        calculate();
        MS5611_State++;
        MS5611_SPI_write(CMD_CONVERT_D1_OSR4096);  // Comando para leer la presión
        result = 1;                                // Nueva lectura de presión   
    }
  }
  return(result);
}

unsigned long readRawTemperature()
{
  // see datasheet page 7 for formulas
  //MS5611lastRawTemperature = MS5611readConversion();
  //int64_t dT     = MS5611lastRawTemperature - (((long)MS5611Prom[5]) << 8);
  //MS5611_offset  = (((int64_t)MS5611Prom[2]) << 16) + ((MS5611Prom[4] * dT) >> 7);
  //MS5611_sens    = (((int64_t)MS5611Prom[1]) << 15) + ((MS5611Prom[3] * dT) >> 8);

  return rawTemperature;
}
float readTemperature()
{
  //return ((1<<5)*2000 + (((MS5611lastRawTemperature - ((int64_t)MS5611Prom[5] << 8)) * MS5611Prom[6]) >> (23-5))) / ((1<<5) * 100.0);
  return rawTemperature;
}

float readRawPressure()
{
 // MS5611lastRawPressure = MS5611readConversion();

 // return (((( MS5611lastRawPressure * MS5611_sens) >> 21) - MS5611_offset) >> (15-5)) / ((float)(1<<5));
  return rawPressure;
}

// Return altitude using the standard 1013.25 mbar at sea level reference
float get_altitude()
{
	float tmp_float;
	float Altitude;
	tmp_float = ((presion) / 101325.0);
	tmp_float = pow(tmp_float, 0.190295);
	Altitude = 44330.0 * (1.0 - tmp_float);
	return (Altitude);
}

bool baroGroundUpdateDone = false;
unsigned long baroStartTime;
//*********************************************************************
void initializeBaro() 
{
  baroStartTime = micros();
  baroGroundAltitude = 0;
   
  MS5611reset();			// reset the device to populate its internal PROM registers
  delay(4); // some safety time

  if(MS5611readPROM()) vehicleState |= BARO_DETECTED;
  MS5611_SPI_write(CMD_CONVERT_D2_OSR4096); // setup up next measure() for temperature

  measureGroundBaro();//Medir la presión inicial del suelo (muestras múltiples)
  measureGroundBaro();

  baroAltitude = baroGroundAltitude; //igualamos la alturas
}
//leectura para obtener la media
void measureBaroSum(){
	ReadMS5611();
	if (MS5611_State ==4){
	  presion = rawPressure/100.0;
	  rawPressureSum = rawPressureSum + rawPressure;
	  rawPressureSumCount++;
	}
}

void measureBaro(){
	ReadMS5611();
  if (MS5611_State ==4){
	  presion = rawPressure/100.0;
  }
}

bool MS5611_first_read = true;

void evaluateBaroAltitude() {
  if (rawPressureSumCount < 5) return; // it may occur at init time that no pressure has been read yet!

  presion =float (rawPressureSum / rawPressureSumCount)/100.0;// obtenemos la media
  rawPressureSum = 0.0;
  rawPressureSumCount =0;
  baroRawAltitude = get_altitude();

 // baroRawAltitude = 44330 * (1 - pow(presion/101325.0, pressureFactor)); // returns absolute baroAltitude in meters

  if(MS5611_first_read) {       // primera lectura
    baroAltitude = baroRawAltitude;
    MS5611_first_read = false;
  } 
  else {
    baroAltitude = filterSmooth(baroRawAltitude, baroAltitude, baroSmoothFactor);
  }

  // set ground altitude after a delay, so sensor has time to heat up
  if(!baroGroundUpdateDone){
	if((micros()-baroStartTime) > 1000000) {
	  baroGroundAltitude = baroAltitude;
	  baroGroundUpdateDone = true;
	}
  }
}
#endif
