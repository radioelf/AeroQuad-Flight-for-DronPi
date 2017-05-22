
#ifndef __AP_BARO_MS5611_SPI_H__
#define __AP_BARO_MS5611_SPI_H__

//#include "AP_Baro.h"

class AP_Baro_MS5611 
{
  public:
  // Constructor
  AP_Baro_MS5611(uint8_t cs) {
	  MS5611_CS_PIN = cs;
	  }  
	  
  void init();
  void MS5611_SPI_write(byte reg);
  uint32_t MS5611_SPI_read_ADC();
  uint16_t MS5611_SPI_read_16bits(byte reg);
  uint8_t MS5611_SPI_read(byte reg);

  /* AP_Baro public interface: */
  uint8_t read();
  int32_t get_pressure();     // in mbar*100 units
  int16_t get_temperature();  // in celsius degrees * 100 units
  float get_altitude();        // in meter units

  int32_t get_raw_pressure();
  int32_t get_raw_temp();

  private:
  uint8_t MS5611_CS_PIN;
  int16_t Temp;
  int32_t Press;
  int32_t Alt;

  int32_t _raw_press;
  int32_t _raw_temp;
  // Internal calibration registers
  uint16_t C1,C2,C3,C4,C5,C6;
  uint32_t D1,D2;
  void calculate();
  uint8_t MS5611_Ready();
  long MS5611_timer;
  uint8_t MS5611_State;
};

#endif //  __AP_BARO_MS5611_H__
