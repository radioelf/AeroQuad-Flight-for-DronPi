/*
FRONT_LEFT  MOTOR1
FRONT_RIGHT MOTOR2
REAR_RIGHT  MOTOR3
REAR_LEFT   MOTOR4
       CW  0....Front....0 CCW
           ......***......    
           ......***......    
           ......***......    
      CCW  0....Back.....0  CW
*/

#ifndef _AEROQUAD_MOTORS_PCA9685_H_
#define _AEROQUAD_MOTORS_PCA9685_H_


#define PCA9685_RA_MODE1           0x00
#define PCA9685_RA_MODE2           0x01
#define PCA9685_RA_LED0_ON_L       0x06
#define PCA9685_RA_LED0_ON_H       0x07
#define PCA9685_RA_LED0_OFF_L      0x08
#define PCA9685_RA_LED0_OFF_H      0x09
#define PCA9685_RA_ALL_LED_ON_L    0xFA
#define PCA9685_RA_ALL_LED_ON_H    0xFB
#define PCA9685_RA_ALL_LED_OFF_L   0xFC
#define PCA9685_RA_ALL_LED_OFF_H   0xFD
#define PCA9685_RA_PRE_SCALE       0xFE

#define PCA9685_MODE1_RESTART_BIT  (1 << 7)
#define PCA9685_MODE1_EXTCLK_BIT   (1 << 6)
#define PCA9685_MODE1_AI_BIT       (1 << 5)
#define PCA9685_MODE1_SLEEP_BIT    (1 << 4)
#define PCA9685_MODE1_SUB1_BIT     (1 << 3)
#define PCA9685_MODE1_SUB2_BIT     (1 << 2)
#define PCA9685_MODE1_SUB3_BIT     (1 << 1)
#define PCA9685_MODE1_ALLCALL_BIT  (1 << 0)

#define PCA9685_MODE2_INVRT_BIT    (1 << 4)
#define PCA9685_MODE2_OCH_BIT      (1 << 3)
#define PCA9685_MODE2_OUTDRV_BIT   (1 << 2)
#define PCA9685_MODE2_OUTNE1_BIT   (1 << 1)
#define PCA9685_MODE2_OUTNE0_BIT   (1 << 0)
#define PCA9685_ALL_LED_OFF_H_SHUT (1 << 4)
#define PCA9685_INTERNAL_CLOCK (1.04f * 25000000.f)
#define PCA9685_EXTERNAL_CLOCK 24576000.f

#define ESC_Frec 400                                         // blheli ESC->20-500hz, 400hz->2.5ms

#define direccion 0x40

#ifdef triConfig
  #define numberOfMotors  3
#endif
#if defined quadXConfig || defined quadPlusConfig ||  defined quadY4Config
  #define numberOfMotors  4
#endif
#if defined hexXConfig || defined hexPlusConfig || defined hexY6Config
  #define numberOfMotors  6
#endif
#if defined octoX8Config || defined octoPlusConfig || defined octoXConfig
  #define numberOfMotors  8
#endif 
#define MOTOR1 0
#define MINCOMMAND 1000                                      // ms
#define MAXCOMMAND 2000                                      // ms
uint16_t MinimPulse = round(4096 / (1000.f / ESC_Frec)) - 1;
int motorCommand[numberOfMotors] = {0};  
uint16_t frecuencia =50;
float frequency;
bool x = false;

//************************************************************************
void write_register_bytes(int regAddress, byte data0, byte data1, byte data2, byte data3){
  Wire.beginTransmission(direccion);
  Wire.write(regAddress);
  Wire.write(data0);
  Wire.write(data1);
  Wire.write(data2);
  Wire.write(data3);
  Wire.endTransmission();
}
void write_register(int regAddress, byte data){
  Wire.beginTransmission(direccion);
  Wire.write(regAddress);
  Wire.write(data);
  Wire.endTransmission();
}
word read_register(int regAddress) {
  word returnword = 0x00;
  Wire.beginTransmission(direccion);
  Wire.write(regAddress);
  Wire.endTransmission();
  Wire.requestFrom((int)direccion, 1);
  // esperamos e bytes
  while (Wire.available()) {
    returnword |= Wire.read(); 
  }
  return returnword;
}
// Desactivar el modo de reposo e iniciar las salidas
void restart() {
    write_register ( PCA9685_RA_MODE1, PCA9685_MODE1_SLEEP_BIT);
    write_register (PCA9685_RA_MODE1, (PCA9685_MODE1_SLEEP_BIT  |  PCA9685_MODE1_EXTCLK_BIT));
    write_register (PCA9685_RA_MODE1, (PCA9685_MODE1_RESTART_BIT | PCA9685_MODE1_EXTCLK_BIT| PCA9685_MODE1_AI_BIT));
}
// La salida se desactiban si estamos en sleep (sleep OFF)
void sleep() {
  byte MODE1 =read_register(PCA9685_RA_MODE1);
  write_register(PCA9685_RA_MODE1, (MODE1 | PCA9685_MODE1_SLEEP_BIT));
  delayMicroseconds (1000);
}
//lee el valor prescale actual y calcula la fecuencia 
float getFrequency() {
    uint8_t data =read_register(PCA9685_RA_PRE_SCALE);
    return 24576000.f / 4096.f / (data + 1);
}
//Calcula el valor prescale basado en la frecuencia especificada y lo escribimos en el dispositivo.
void setFrequency(float frecuencia) {
    sleep();
    delayMicroseconds (1000);
    uint8_t prescale = roundf(24576000.f / 4096.f / frecuencia)  - 1;
    write_register( PCA9685_RA_PRE_SCALE, prescale);
    frequency = getFrequency();
    restart();
}  
void Init_PCA9685() {
  write_register(PCA9685_RA_MODE2, B00000100);               // defecto PCA9685_RA_MODE2
  frequency = getFrequency();                                // lee el valor prescale almacenado en PCA9685 y calcula la frecuencia basada en él
  write_register (PCA9685_RA_MODE1, (PCA9685_MODE1_AI_BIT));
  digitalWrite(PWM_EN_PIN, LOW);                             // pin PWM_EN_PIN a 0 (habilitado)
  restart();                                                 // el reinicio se realiza para habilitar el reloj
}

bool testPCA9685() {
    if (read_register(PCA9685_RA_PRE_SCALE))
        return true;
    else
return false;
}
// Establece el offset de inicio del pulso y su longitud del canal indicado. Channel(0-15), Offset (0-4095), Length (0-4095)
void setPWM(uint8_t channel, uint16_t offset, uint16_t length) {
    uint8_t data[4] = {0, 0, 0, 0};
    if(length == 0) {
        data[3] = 0x10;
    } else if(length >= 4096) {
        data[1] = 0x10;
    } else {
        data[0] = offset & 0xFF;
        data[1] = offset >> 8;
        data[2] = length & 0xFF;
        data[3] = length >> 8;
    }
    write_register_bytes ((PCA9685_RA_LED0_ON_L + 4 * channel), data[0], data[1], data[2], data[3]);
    //write_register((PCA9685_RA_LED0_ON_L + 4 * channel), data[0]);
    //write_register((PCA9685_RA_LED0_ON_H + 4 * channel), data[1]);
    //write_register((PCA9685_RA_LED0_OFF_L + 4 * channel), data[2]);
    //write_register((PCA9685_RA_LED0_OFF_H + 4 * channel), data[3]);
}
void inverter(bool ON_OFF){
  byte data =read_register(PCA9685_RA_MODE2);
  if (ON_OFF){
    bitSet (data, 4);                         // invertir salida
  }else{
    bitClear(data ,4);                        // NO invertir salida
  }
  write_register(PCA9685_RA_MODE2, data);
}
// Establece la longitud del impulso del canal indicado, Channel (0-15), Length (0-4095)
void setPWM_(uint8_t channel, uint16_t length) {
    setPWM(channel, 0, length);
}

// Establece la longitud en milisegundos del impulso del canal indicado.Channel (0-15). Length en milisegundos
void setPWMmS(uint8_t channel, float length_mS) {
    setPWM_(channel, round((length_mS * 4096.f) / (1000.f / frequency)));
}

// Establece la duración del impulso del canal en microsegundos. Channel number (0-15). Length en microsegundos
void setPWMuS(uint8_t channel, float length_uS) {
    setPWM_(channel, round((length_uS * 4096.f) / (1000000.f / frequency)));
}

// Establece el offset de inicio del pulso y su longitud de todos los canales. Offset (0-4095), Length (0-4095)
void setAllPWM(uint16_t offset, uint16_t length) {
    uint8_t data[4] = {0, 0, 0, 0};   // = {offset & 0xFF, offset >> 8, length & 0xFF, length >> 8};
    data [0] = offset & 0xFF;
    data [1] = offset >> 8;
    data [2] = length & 0xFF;
    data [3] = length >> 8;
    
    write_register_bytes((PCA9685_RA_ALL_LED_ON_L), data[0], data[1], data[2], data[3]);
    //write_register((PCA9685_RA_ALL_LED_ON_L), data[0]);
    //write_register((PCA9685_RA_ALL_LED_ON_H), data[1]);
    //write_register((PCA9685_RA_ALL_LED_OFF_L), data[2]);
    //write_register((PCA9685_RA_ALL_LED_OFF_H), data[3]);
}

// Establece la longitud del pulso para todos los canales. Length (0-4095)
void setAllPWM(uint16_t length) {
    setAllPWM(0, length);
}

// Establece la longitud del pulso en milisegundos para todos los canales. Length en milisegundos
void setAllPWMmS(float length_mS) {
    setAllPWM(round((length_mS * 4096.f) / (1000.f / frequency)));
}

// Establece la longitud del pulso en milisegundos para todos los canales. Length en microsegundos
void setAllPWMuS(float length_uS) {
    setAllPWM(round((length_uS * 4096.f) / (1000000.f / frequency)));
}
//*********************************************************************
void initializeMotors(NB_Motors numbers) {
  delayMicroseconds (5000);
  Init_PCA9685();
  sleep();
  setFrequency(ESC_Frec);                                    // configuramos frecuencia 
  delayMicroseconds (5000);
  //inverter(0);
  setAllPWM(0);                                              // todos los canales a 0
}
// gestion de los motores validos 
void writeMotors() {
    for (byte motor = MOTOR1; motor < numberOfMotors; motor++){
    setPWMuS(motor, motorCommand[motor]);
	}
}
// gestion de los motores validos ->command
void commandAllMotors(int command) {
  for (byte motor = MOTOR1; motor < numberOfMotors; motor++){
    setPWMuS(motor, command);
  }
}
// pulsos en los motores validos
void pulseMotors(byte nbPulse) {
  for (byte i = 0; i < nbPulse; i++) {
	for (byte motor = MOTOR1; motor < numberOfMotors; motor++){
    setPWMuS(motor, MINCOMMAND + 100);
    for (byte x = 0; x < 25; x++) {
      delayMicroseconds (10000);
    }
    setPWMuS(motor, MINCOMMAND);
    for (byte x = 0; x < 25; x++) {
      delayMicroseconds (10000);
    }
	}
  }
} 
#endif
