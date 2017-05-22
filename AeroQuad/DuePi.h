#ifndef _DUEPI_
#define _DUEPI_

  #define LED_GREEN_PIN 7                                               // PC23 PIN SAM3X 134, DronPi PIN-> 18, Max. output current 15mA
  #define LED_BLUE_PIN 8                                                // PC22 PIN SAM3X 133, DronPi PIN-> 22, Max. output current 15mA
  #define LED_L_PIN  13                                                 // PB27 PIN SAM3X 68,  DronPi PIN-> N/C,Max. output current 3mA!!
  #define PPM_PIN 15                                                    // PD5  PIN SAM3X 18,  DronPi PIN-> 7,  Max. output current 15mA
  #define TXD_PIN 16                                                    // PA13 PIN SAM3X 6,   DronPi PIN-> 8,  Max. output current 3mA!!
  #define RXD_PIN 17                                                    // PA12 PIN SAM3X 5,   DronPi PIN-> 10, Max. output current 3mA!!
  #define PWM_EN_PIN 18                                                 // PA11 PIN SAM3X 4,   DronPi PIN-> 13, Max. output current 3mA!!
  #define INT_CAN_PIN 19                                                // PA10 PIN SAM3X 3,   DronPi PIN-> 15, Max. output current 3mA!!
  #define SDA_PIN 20                                                    // PB12 PIN SAM3X 86,  DronPi PIN-> 3,  Max. output current 3mA!!
  #define SCL_PIN 21                                                    // PB13 PIN SAM3X 87,  DronPi PIN-> 5,  Max. output current 3mA!!
  #define INT_MPU_PIN 22                                                // PB26 PIN SAM3X 1,   DronPi PIN-> 16, Max. output current 3mA!!
  #define RTS_PIN 23                                                    // PA14 PIN SAM3X 7,   DronPi PIN-> 12, Max. output current 15mA
  #define CTS_PIN 24                                                    // PA15 PIN SAM3X 8,   DronPi PIN-> 11, Max. output current 15mA
  #define ENABL_PIN 31                                                  // PA7  PIN SAM3X 26,  DronPi PIN-> 1,  Max. output current 15mA
  #define ON_OFF_PIN 48                                                 // PC15 PIN SAM3X 97,  DronPi PIN-> 40, Max. output current 15mA
  #define AUX5_PIN 49                                                   // PC14 PIN SAM3X 96,  DronPi PIN-> 32, Max. output current 15mA
  #define AUX1_PIN 50                                                   // PC13 PIN SAM3X 95,  DronPi PIN-> 31, Max. output current 15mA
  #define AUX2_PIN 51                                                   // PC12 PIN SAM3X 94,  DronPi PIN-> 33, Max. output current 15mA
  #define CS_CAN_PIN 52                                                 // PB21 PIN SAM3X 92,  DronPi PIN-> 38, Max. output current 3mA!!
  #define AUX4_PIN 65                                                   // PB20 PIN SAM3X 91,  DronPi PIN-> 37, Max. output current 3mA!!
  #define BUZZER_PIN 66                                                 // PB15 PIN SAM3X 76,  DronPi PIN-> 29, Max. output current 3mA!!
  #define AUX3_PIN 67                                                   // PC16 PIN SAM3X 77,  DronPi PIN-> 35, Max. output current 3mA!!
  #define SDA1_PIN 70                                                   // PA17 PIN SAM3X 9,   DronPi PIN-> 27, Max. output current 3mA!!
  #define SCL1_PIN 71                                                   // PA18 PIN SAM3X 70,  DronPi PIN-> 28, Max. output current 15mA  
  #define LED_WHITE_PIN 72                                              // PC30 PIN SAM3X 103, DronPi PIN-> 36, Max. output current 15mA                                           
  #define MISO_PIN 74                                                   // PA25 PIN SAM3X 108, DronPi PIN-> 21, Max. output current 15mA
  #define MOSI_PIN 75                                                   // PA26 PIN SAM3X 109, DronPi PIN-> 19, Max. output current 15mA
  #define SPCK_PIN 76                                                   // PA27 PIN SAM3X 110, DronPi PIN-> 23, Max. output current 15mA
  #define CS_BARO_PIN 77                                                // PA28 PIN SAM3X 111, DronPi PIN-> 24, Max. output current 15mA (PIN 77/10)
  #define CS_MPU_PIN 87                                                 // PA29 PIN SAM3X 112, DronPi PIN-> 26, Max. output current 15mA (PIN 87/4)

  uint8_t NO_user_PIN[46] = {2,3,5,6,9,11,12,14,25,26,27,28,29,30,32,33,34,35,36,37,38,39,40,41,42,43,44,45,46,47,53,54,55,56,57,58,59,60,61,62,63,64,68,69,73,78};

#define BUZZER 
//#define watchdog

#define MOTOR1 0
#define MOTOR2 1
#define MOTOR3 2
#define MOTOR4 3
#define MOTOR5 4
#define MOTOR6 5
#define MOTOR7 6
#define MOTOR8 7
#define MINCOMMAND 1000
#define MAXCOMMAND 2000

enum NB_Motors{
  FOUR_Motors = 4,
  SIX_Motors = 6,
  EIGHT_Motors = 8
};
 // EEPROM HAT 24AA32
 int addess_HAT = 0x50;                                       // I2C address EEPROM 24AA32
 String trama = "";
 uint8_t Ini_EEPROM =200;                                     // Data eeprom 200-4000,reserved for EEPROM HAT -> 0 a 199
//#define EEPROM_USES_16BIT_WORDS

 uint8_t wai =0;                                              // sensor MPU9250->0x71 OK
 bool preArmer = false;
 bool cicle =false;
 unsigned long CriticalSensorsMPU =1000;
 uint8_t alarm_battery =0;

#if defined watchdog
  void watchdogSetup(void)
  {
  watchdogEnable(1250);                                       // Enable watchdog. 1250ms
  }
#endif
/********************************************************************
  EEPROM HATs 24AA32 0x00-0xFFF (4095bytes = 4Kbytes), I2C1
*********************************************************************/
void writeEEPROMI2C(unsigned int eeaddress, uint8_t data ) 
{
  eeaddress = (eeaddress + Ini_EEPROM);
  noInterrupts();
  Wire1.beginTransmission(addess_HAT);
  Wire1.write((byte)(eeaddress >> 8));                        // MSB
  Wire1.write((byte)(eeaddress & 0xFF));                      // LSB
  Wire1.write(data);
  Wire1.endTransmission();
  interrupts();
  delayMicroseconds (5000);                                   // needs >5ms for page write
}
//------------------------------------------------------------------ 
uint8_t readEEPROMI2C(unsigned int eeaddress ) 
{
  uint8_t rdata = 0xFF;
  eeaddress = (eeaddress + Ini_EEPROM); 
  noInterrupts(); 
  Wire1.beginTransmission(addess_HAT);
  Wire1.write((byte)(eeaddress >> 8));                         // MSB
  Wire1.write((byte)(eeaddress & 0xFF));                       // LSB
  Wire1.endTransmission();
  Wire1.requestFrom(addess_HAT,1);
  if (Wire1.available()) rdata = Wire1.read();
  interrupts();
  delayMicroseconds (1000);                                    // needs >1ms for read
  return rdata;
} 
//------------------------------------------------------------------ 
double read_double_eeprom(unsigned int eeaddress)
{
  double dato = 0.0;
  uint8_t* p = (uint8_t*)(void*)&dato;
  for (int i = 0; i < sizeof(dato); i++)
    *p++ = readEEPROMI2C(eeaddress++);
  return dato;
}
//------------------------------------------------------------------ 
void write_double_eeprom(unsigned int eeaddress, double data)
{
  //const uint8_t* p = (const uint8_t*)(const void*)&data;
  uint8_t* p = (uint8_t*)(void*)&data;
  for (int i = 0; i < sizeof(data); i++){
    writeEEPROMI2C(eeaddress++, *p++);
  }
}
//------------------------------------------------------------------ 
//****************************ADC1115*******************************
// Arduino Due 3V3 SDA D4 pin 7, SCL D5 pin 8
/*            R1                R2
 *   Int    |-----|           |-----|
 * ----->---| 10K |-----*-----| 20K |-------*GND
 *          |-----|     |     |-----|
 *                      |          
 *                      V
 *                 Int ADC ADS1115
 *  Vout =  R2/(R1+R2)/Vint
 *  Vout = (200000/30000)/5 = 3.33V ->1811->5V 65535 
 */
//float multiplo = 1.85069F;                                 // ADS1115  +/- 4.096V gain 1x (16-bit) 
uint8_t     CHANNEL_V = 0;
uint8_t     CHANNEL_I = 1;
  // La ganacia del la entrada ADC se pueden cambiar a través de la siguiente
  // funciones, pero se tiene que tener cuidado de no superar nunca VDD + 0,3 V,
  // La configuración de estos valores de forma incorrecta puede destruir el ADC!
  //                                                                ADS1015  ADS1115
  //                                                                -------  -------
  // ads.setGain(GAIN_TWOTHIRDS);  // 2/3x gain +/- 6.144V  1 bit = 3mV      0.1875mV (default)
  // ads.setGain(GAIN_ONE);        // 1x gain   +/- 4.096V  1 bit = 2mV      0.125mV
  // ads.setGain(GAIN_TWO);        // 2x gain   +/- 2.048V  1 bit = 1mV      0.0625mV
  // ads.setGain(GAIN_FOUR);       // 4x gain   +/- 1.024V  1 bit = 0.5mV    0.03125mV
  // ads.setGain(GAIN_EIGHT);      // 8x gain   +/- 0.512V  1 bit = 0.25mV   0.015625mV
  // ads.setGain(GAIN_SIXTEEN);    // 16x gain  +/- 0.256V  1 bit = 0.125mV  0.0078125mV
#define GAIN GAIN_ONE
//------------------------------------------------------------------ 
//***********************Buzzer*************************************
#if defined BUZZER
int cicles_buzzer =0;
boolean ON_OFF =false;

// ISR Timer3
void buzzer_on_off(){
  ON_OFF =!ON_OFF;
  if (cicles_buzzer !=0){
    digitalWrite(BUZZER_PIN, ON_OFF? HIGH : LOW);
    if (digitalRead(BUZZER_PIN) && cicles_buzzer < 255) cicles_buzzer--; // >255 continued
  }else{
    digitalWrite(BUZZER_PIN, HIGH);                          // fin ISR Timer3
    Timer3.detachInterrupt();
    Timer3.stop();
    ON_OFF = false;
  }
}
// ON ISR Timer3 ->ms  
void buzzer(unsigned long duration, uint8_t cicles){
  Timer3.attachInterrupt(buzzer_on_off).start(duration *1000); //ms
  cicles_buzzer = cicles;
  digitalWrite(BUZZER_PIN, LOW);
  if (cicles == 0){
    cicles_buzzer =256;
  }else{
    cicles_buzzer = cicles;
  } 
}
#endif

#endif
