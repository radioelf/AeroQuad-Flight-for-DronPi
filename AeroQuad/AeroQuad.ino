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

/****************************************************************************
   Before flight, select the different user options for your AeroQuad by
   editing UserConfiguration.h.

   If you need additional assistance go to http://www.aeroquad.com/forum.php
   or talk to us live on IRC #aeroquad
*****************************************************************************/

/* Mod. by Radioelf for board DuePi (arduino Due)
 *  https://github.com/brianc118/MPU9250 sample rate to 1 kHz (Mod. file MPU9250.cpp)
 *  https://github.com/adafruit/Adafruit_ADS1X15 (Mod. file Adafruit_ADS1015.cpp for mode CONTINUOUS)
 *  https://github.com/ivanseidel/DueTimer 
 *  http://radioelf.blogspot.com.es/

-MS5611 Barometric pressure sensor (Bus SPI)
-MPU9250 Gyroscope/Accelerometer/Internal compass/9-AXIS (Bus SPI) (-48,00 a 48,00 Gauss)
-MCP2515 CAN Bus (Bus SPI) Not implemented
-ADS1115 ADC 16bits (Bus I2C) address 0x48, 
-PCA9685 PWM (Bus I2c) address 0x40 y 0x70(ALLCALL)
-SRF08 sonar 3-600Cm
-BUZZER

 */
#include "UserConfiguration.h" // Edit this file first before uploading to the AeroQuad

//
// Define Security Checks
//

#if defined(UseGPSNMEA) || defined(UseGPS_UBLOX) || defined(UseGPSMTK) || defined(UseGPS406)
 #define UseGPS
#endif 

#if defined(UseGPSNavigator) && !defined(AltitudeHoldBaro)
  #error "GpsNavigation NEED AltitudeHoldBaro defined"
#endif

#if defined(AutoLanding) && (!defined(AltitudeHoldBaro) || !defined(AltitudeHoldRangeFinder))
  #error "AutoLanding NEED AltitudeHoldBaro and AltitudeHoldRangeFinder defined"
#endif

#if (defined(ReceiverSBUS) || defined(ReceiverIBUS)) && defined(SlowTelemetry)
  #error "Receiver SWBUS and SlowTelemetry are in conflict for Seria2, they can't be used together"
#endif

#if defined (CameraTXControl) && !defined (CameraControl)
  #error "CameraTXControl need to have CameraControl defined"
#endif 

#include <Wire.h>
#include <GlobalDefined.h>
#include <AQMath.h>

#ifdef DuePi
  #if !defined(ARDUINO_SAM_DUE)
    #error  "Error: Invalid board!!" 
  #endif
  #include <SPI.h>                                                     
  #include <DueTimer.h>                                                 // Timer3-> buzzer https://github.com/ivanseidel/DueTimer                                                
  #include "DuePi.h"   
#else
 #include <EEPROM.h>
#endif

#include "AeroQuad.h"
#include "PID.h"
#include <FourtOrderFilter.h>

#ifdef BattMonitor
  #include <BatteryMonitorTypes.h>
#endif

//********************************************************
//********************************************************
//********* PLATFORM SPECIFIC SECTION ********************
//********************************************************
//********************************************************
#ifdef AeroQuad_v1
  #define LED_GREEN_PIN 13
  #define LED_BLUE_PIN 12
  #define LED_WHITE_PIN  12

  // Gyroscope declaration
  #include <Gyroscope_IDG_IDZ500.h>

  // Accelerometer declaration
  #include <Accelerometer_ADXL500.h>

  // Receiver declaration
  #define RECEIVER_328P

  // Motor declaration
  #define MOTOR_PWM

  // unsupported in v1
  #undef AltitudeHoldBaro
  #undef AltitudeHoldRangeFinder
  #undef HeadingMagHold
  #undef BattMonitor
  #undef BattMonitorAutoDescent
  #undef POWERED_BY_VIN        
  #undef CameraControl
  #undef OSD
  #undef UseGPS
  #undef UseGPSNavigator

  /**
   * Put AeroQuad_v1 specific initialization need here
   */
  void initPlatform() {
    setGyroAref(aref);
  }
  
  // called when eeprom is initialized
  void initializePlatformSpecificAccelCalibration() {
    // Accel Cal
    accelScaleFactor[XAXIS] = 1.0;
    runTimeAccelBias[XAXIS] = 0.0;
    accelScaleFactor[YAXIS] = 1.0;
    runTimeAccelBias[YAXIS] = 0.0;
    accelScaleFactor[ZAXIS] = 1.0;
    runTimeAccelBias[ZAXIS] = 0.0;
  }

  /**
   * Measure critical sensors
   */
  void measureCriticalSensors() {
    measureGyro();
    measureAccel();
  }
#endif

#ifdef AeroQuad_v1_IDG
  #define LED_GREEN_PIN 13
  #define LED_BLUE_PIN 12
  #define LED_WHITE_PIN  12

  // Gyroscope declaration
  #include <Gyroscope_IDG_IDZ500.h>

  // Accelerometer declaration
  #include <Accelerometer_ADXL500.h>

  // Receiver declaration
  #define RECEIVER_328P

  // Motor declaration
  #define MOTOR_PWM

  // unsupported in v1
  #undef AltitudeHoldBaro
  #undef AltitudeHoldRangeFinder
  #undef HeadingMagHold
  #undef BattMonitor
  #undef BattMonitorAutoDescent
  #undef POWERED_BY_VIN        
  #undef CameraControl
  #undef OSD
  #undef UseGPS
  #undef UseGPSNavigator

  /**
   * Put AeroQuad_v1_IDG specific initialization need here
   */
  void initPlatform() {
    setGyroAref(aref);
  }
  
    // called when eeprom is initialized
  void initializePlatformSpecificAccelCalibration() {
    // Accel Cal
    accelScaleFactor[XAXIS] = 1.0;
    runTimeAccelBias[XAXIS] = 0.0;
    accelScaleFactor[YAXIS] = 1.0;
    runTimeAccelBias[YAXIS] = 0.0;
    accelScaleFactor[ZAXIS] = 1.0;
    runTimeAccelBias[ZAXIS] = 0.0;
  }

  /**
   * Measure critical sensors
   */
  void measureCriticalSensors() {
    measureGyro();
    measureAccel();
  }
#endif

#ifdef AeroQuad_v18
  #define LED_GREEN_PIN 13
  #define LED_BLUE_PIN 12
  #define LED_WHITE_PIN  12

  #include <Device_I2C.h>

  // Gyroscope declaration
  #include <Gyroscope_ITG3200.h>

  // Accelerometer declaration
  #include <Accelerometer_BMA180.h>

  // Receiver declaration
  #define RECEIVER_328P

  // Motor declaration
  #define MOTOR_PWM_Timer

  // heading mag hold declaration
  #ifdef HeadingMagHold
    #define HMC5843
  #endif

  // Battery Monitor declaration
  #ifdef BattMonitor
    #define BattDefaultConfig DEFINE_BATTERY(0, 0, 15, 0.9, BM_NOPIN, 0, 0)
  #else
    #undef BattMonitorAutoDescent
    #undef POWERED_BY_VIN        
  #endif

  #undef AltitudeHoldBaro
  #undef AltitudeHoldRangeFinder
  #undef CameraControl
  #undef OSD
  #undef UseGPS
  #undef UseGPSNavigator

  /**
   * Put AeroQuad_v18 specific initialization need here
   */
  void initPlatform() {

    pinMode(LED_BLUE_PIN, OUTPUT);
    digitalWrite(LED_BLUE_PIN, LOW);
    pinMode(LED_WHITE_PIN , OUTPUT);
    digitalWrite(LED_WHITE_PIN , LOW);

    Wire.begin();
    TWBR = 12;
  }
  
  // called when eeprom is initialized
  void initializePlatformSpecificAccelCalibration() {
    // Kenny default value, a real accel calibration is strongly recommended
    accelScaleFactor[XAXIS] = 0.0047340002;
    accelScaleFactor[YAXIS] = -0.0046519994;
    accelScaleFactor[ZAXIS] = -0.0046799998;
  }

  /**
   * Measure critical sensors
   */
  void measureCriticalSensors() {
    measureGyroSum();
    measureAccelSum();
  }

#endif

#ifdef AeroQuad_Mini
  #define LED_GREEN_PIN 13
  #define LED_BLUE_PIN 12
  #define LED_WHITE_PIN  12

  #include <Device_I2C.h>

  // Gyroscope declaration
  #define ITG3200_ADDRESS_ALTERNATE
  #include <Gyroscope_ITG3200.h>

  // Accelerometer declaration
  #include <Accelerometer_ADXL345.h>

  // Receiver declaration
  #define RECEIVER_328P

  // Motor declaration
  #if defined(quadXConfig) || defined(quadPlusConfig) || defined(quadY4Config)
    #define MOTOR_PWM_Timer
  #else
    #define MOTOR_PWM
  #endif    

  // heading mag hold declaration
  #ifdef HeadingMagHold
    #define HMC5843
  #endif
  
  // Battery Monitor declaration
  #ifdef BattMonitor
    #define BattDefaultConfig DEFINE_BATTERY(0, 0, 15.0, 0.53, BM_NOPIN, 0, 0)
  #else
    #undef BattMonitorAutoDescent
    #undef POWERED_BY_VIN        
  #endif

  // unsupported in mini
  #undef AltitudeHoldBaro
  #undef AltitudeHoldRangeFinder  
  #undef CameraControl
  #undef OSD
  #undef UseGPS
  #undef UseGPSNavigator

  /**
   * Put AeroQuad_Mini specific initialization need here
   */
  void initPlatform() {

    pinMode(LED_BLUE_PIN, OUTPUT);
    digitalWrite(LED_BLUE_PIN, LOW);
    pinMode(LED_WHITE_PIN , OUTPUT);
    digitalWrite(LED_WHITE_PIN , LOW);

    Wire.begin();
    TWBR = 12;
  }

  // called when eeprom is initialized
  void initializePlatformSpecificAccelCalibration() {
    // Kenny default value, a real accel calibration is strongly recommended
    accelScaleFactor[XAXIS] = 0.0371299982;
    accelScaleFactor[YAXIS] = -0.0374319982;
    accelScaleFactor[ZAXIS] = -0.0385979986;
  }

  /**
   * Measure critical sensors
   */
  void measureCriticalSensors() {
    measureGyroSum();
    measureAccelSum();
  }
#endif

#ifdef AeroQuadMega_v1
  #define LED_GREEN_PIN 13
  #define LED_BLUE_PIN 4
  #define LED_WHITE_PIN  31

  // Special thanks to Wilafau for fixes for this setup
  // http://aeroquad.com/showthread.php?991-AeroQuad-Flight-Software-v2.0&p=11466&viewfull=1#post11466
  // Gyroscope declaration
  #include <Gyroscope_IDG_IDZ500.h>

  // Accelerometer declaration
  #include <Accelerometer_ADXL500.h>

  // Reveiver declaration
  #define OLD_RECEIVER_PIN_ORDER
  #define RECEIVER_MEGA

  // Motor declaration
  #define MOTOR_PWM

  // unsupported on mega v1
  #undef AltitudeHoldBaro
  #undef AltitudeHoldRangeFinder  
  #undef HeadingMagHold
  #undef BattMonitor
  #undef BattMonitorAutoDescent
  #undef POWERED_BY_VIN        
  #undef CameraControl
  #undef OSD

  /**
   * Put AeroQuadMega_v1 specific initialization need here
   */
  void initPlatform() {
    setGyroAref(aref);
  }
  
    // called when eeprom is initialized
  void initializePlatformSpecificAccelCalibration() {
    // Accel Cal
    accelScaleFactor[XAXIS] = 1.0;
    runTimeAccelBias[XAXIS] = 0.0;
    accelScaleFactor[YAXIS] = 1.0;
    runTimeAccelBias[YAXIS] = 0.0;
    accelScaleFactor[ZAXIS] = 1.0;
    runTimeAccelBias[ZAXIS] = 0.0;
  }

  /**
   * Measure critical sensors
   */
  void measureCriticalSensors() {
    if (deltaTime >= 10000) {
      measureGyro();
      measureAccel();
    }
  }
#endif

#ifdef AeroQuadMega_v2
  #define LED_GREEN_PIN 13
  #define LED_BLUE_PIN 4
  #define LED_WHITE_PIN  31

  #include <Device_I2C.h>

  // Gyroscope declaration
  #include <Gyroscope_ITG3200.h>

  // Accelerometer declaration
  #include <Accelerometer_BMA180.h>

  // Receiver Declaration
  #define RECEIVER_MEGA

  // Motor declaration
  #define MOTOR_PWM_Timer

  // heading mag hold declaration
  #ifdef HeadingMagHold
    #include <Compass.h>
//    #define SPARKFUN_5883L_BOB
    #define HMC5843
  #endif

  // Altitude declaration
  #ifdef AltitudeHoldBaro    
    #define BMP085 
  #endif
  #ifdef AltitudeHoldRangeFinder
    #define XLMAXSONAR 
  #endif

  // Battery Monitor declaration
  #ifdef BattMonitor
    #ifdef POWERED_BY_VIN
      #define BattDefaultConfig DEFINE_BATTERY(0, 0, 15.0, 0, BM_NOPIN, 0, 0) // v2 shield powered via VIN (no diode)
    #else
      #define BattDefaultConfig DEFINE_BATTERY(0, 0, 15.0, 0.82, BM_NOPIN, 0, 0) // v2 shield powered via power jack
    #endif
  #else
    #undef BattMonitorAutoDescent
    #undef POWERED_BY_VIN        
  #endif

  #ifdef OSD
    #define MAX7456_OSD
  #endif  
  
  #ifndef UseGPS
    #undef UseGPSNavigator
  #endif

  /**
   * Put AeroQuadMega_v2 specific initialization need here
   */
  void initPlatform() {

    pinMode(LED_BLUE_PIN, OUTPUT);
    digitalWrite(LED_BLUE_PIN, LOW);
    pinMode(LED_WHITE_PIN , OUTPUT);
    digitalWrite(LED_WHITE_PIN , LOW);

    // pins set to INPUT for camera stabilization so won't interfere with new camera class
    pinMode(33, INPUT); // disable SERVO 1, jumper D12 for roll
    pinMode(34, INPUT); // disable SERVO 2, jumper D11 for pitch
    pinMode(35, INPUT); // disable SERVO 3, jumper D13 for yaw
    pinMode(43, OUTPUT); // LED 1
    pinMode(44, OUTPUT); // LED 2
    pinMode(45, OUTPUT); // LED 3
    pinMode(46, OUTPUT); // LED 4
    digitalWrite(43, HIGH); // LED 1 on
    digitalWrite(44, HIGH); // LED 2 on
    digitalWrite(45, HIGH); // LED 3 on
    digitalWrite(46, HIGH); // LED 4 on

    Wire.begin();
    TWBR = 12;
  }
  
  // called when eeprom is initialized
  void initializePlatformSpecificAccelCalibration() {
    // Kenny default value, a real accel calibration is strongly recommended
    accelScaleFactor[XAXIS] = 0.0046449995;
    accelScaleFactor[YAXIS] = -0.0047950000;
    accelScaleFactor[ZAXIS] = -0.0047549996;
    #ifdef HeadingMagHold
      magBias[XAXIS]  = 60.000000;
      magBias[YAXIS]  = -39.000000;
      magBias[ZAXIS]  = -7.500000;
    #endif
  }

  /**
   * Measure critical sensors
   */
  void measureCriticalSensors() {
    measureGyroSum();
    measureAccelSum();
  }
#endif

#ifdef AeroQuadMega_v21
  #define LED_GREEN_PIN 13
  #define LED_BLUE_PIN 4
  #define LED_WHITE_PIN  31

  #include <Device_I2C.h>

  // Gyroscope declaration
  #define ITG3200_ADDRESS_ALTERNATE
  #include <Gyroscope_ITG3200_9DOF.h>

  // Accelerometer declaration
  #include <Accelerometer_ADXL345_9DOF.h>

  // Receiver Declaration
  #define RECEIVER_MEGA

  // Motor declaration
  #define MOTOR_PWM_Timer

  // heading mag hold declaration
  #ifdef HeadingMagHold
    #include <Compass.h>
    #define SPARKFUN_9DOF_5883L
  #endif

  // Altitude declaration
  #ifdef AltitudeHoldBaro
    #define BMP085
  #endif
  #ifdef AltitudeHoldRangeFinder
    #define XLMAXSONAR 
  #endif


  // Battery Monitor declaration
  #ifdef BattMonitor
    #ifdef POWERED_BY_VIN
      #define BattDefaultConfig DEFINE_BATTERY(0, 0, 15.0, 0, BM_NOPIN, 0, 0) // v2 shield powered via VIN (no diode)
    #else
      #define BattDefaultConfig DEFINE_BATTERY(0, 0, 15.0, 0.82, BM_NOPIN, 0, 0) // v2 shield powered via power jack
    #endif
  #else
    #undef BattMonitorAutoDescent
    #undef POWERED_BY_VIN        
  #endif

  #ifdef OSD
    #define MAX7456_OSD
  #endif  
  
  #ifndef UseGPS
    #undef UseGPSNavigator
  #endif

  /**
   * Put AeroQuadMega_v21 specific initialization need here
   */
  void initPlatform() {

    pinMode(LED_BLUE_PIN, OUTPUT);
    digitalWrite(LED_BLUE_PIN, LOW);
    pinMode(LED_WHITE_PIN , OUTPUT);
    digitalWrite(LED_WHITE_PIN , LOW);

    // pins set to INPUT for camera stabilization so won't interfere with new camera class
    pinMode(33, INPUT); // disable SERVO 1, jumper D12 for roll
    pinMode(34, INPUT); // disable SERVO 2, jumper D11 for pitch
    pinMode(35, INPUT); // disable SERVO 3, jumper D13 for yaw
    pinMode(43, OUTPUT); // LED 1
    pinMode(44, OUTPUT); // LED 2
    pinMode(45, OUTPUT); // LED 3
    pinMode(46, OUTPUT); // LED 4
    digitalWrite(43, HIGH); // LED 1 on
    digitalWrite(44, HIGH); // LED 2 on
    digitalWrite(45, HIGH); // LED 3 on
    digitalWrite(46, HIGH); // LED 4 on

    Wire.begin();
    TWBR = 12;
  }
  
  // called when eeprom is initialized
  void initializePlatformSpecificAccelCalibration() {
    // Kenny default value, a real accel calibration is strongly recommended
    accelScaleFactor[XAXIS] = 0.0365570020;
    accelScaleFactor[YAXIS] = 0.0363000011;
    accelScaleFactor[ZAXIS] = -0.0384629964;
    #ifdef HeadingMagHold
      magBias[XAXIS]  = 1.500000;
      magBias[YAXIS]  = 205.500000;
      magBias[ZAXIS]  = -33.000000;
    #endif
  }

  /**
   * Measure critical sensors
   */
  void measureCriticalSensors() {
    measureGyroSum();
    measureAccelSum();
  }
#endif

#ifdef ArduCopter
  #define LED_GREEN_PIN 37
  #define LED_BLUE_PIN 35
  #define LED_WHITE_PIN  36

  #include <APM_ADC.h>
  #include <APM_RC.h>
  #include <Device_I2C.h>

  // Gyroscope declaration
  #include <Gyroscope_APM.h>

  // Accelerometer Declaration
  #include <Accelerometer_APM.h>

  // Receiver Declaration
  #define RECEIVER_APM

  // Motor Declaration
  #define MOTOR_APM

  // heading mag hold declaration
  #ifdef HeadingMagHold
    #define HMC5843
  #endif
  #ifdef AltitudeHoldRangeFinder
    #define XLMAXSONAR 
  #endif


  // Altitude declaration
  #ifdef AltitudeHoldBaro
    #define BMP085
  #endif

  // Battery monitor declaration
  #ifdef BattMonitor
    #define BattDefaultConfig DEFINE_BATTERY(0, 0, 13.35, 0.31, BM_NOPIN, 0, 0)
  #else
    #undef BattMonitorAutoDescent
    #undef POWERED_BY_VIN        
  #endif

  #undef CameraControl
  #undef OSD
  #ifndef UseGPS
    #undef UseGPSNavigator
  #endif

  /**
   * Put ArduCopter specific initialization need here
   */
  void initPlatform() {
    pinMode(LED_BLUE_PIN, OUTPUT);
    pinMode(LED_WHITE_PIN , OUTPUT);
    pinMode(LED_GREEN_PIN, OUTPUT);

    initializeADC();
    initRC();

    Wire.begin();
    TWBR = 12;
  }
  
  // called when eeprom is initialized
  void initializePlatformSpecificAccelCalibration() {
    // Accel Cal
    accelScaleFactor[XAXIS] = 1.0;
    runTimeAccelBias[XAXIS] = 0.0;
    accelScaleFactor[YAXIS] = 1.0;
    runTimeAccelBias[YAXIS] = 0.0;
    accelScaleFactor[ZAXIS] = 1.0;
    runTimeAccelBias[ZAXIS] = 0.0;
    #ifdef HeadingMagHold
      magBias[XAXIS]  = 0.0;
      magBias[YAXIS]  = 0.0;
      magBias[ZAXIS]  = 0.0;
    #endif
  }

  /**
   * Measure critical sensors
   */
  void measureCriticalSensors() {
    evaluateADC();
    measureGyroSum();
    measureAccelSum();
  }
#endif

#ifdef AeroQuad_Wii
  #define LED_GREEN_PIN 13
  #define LED_BLUE_PIN 12
  #define LED_WHITE_PIN  12

  #include <Device_I2C.h>

  // Platform Wii declaration
  #include <Platform_Wii.h>

  // Gyroscope declaration
  #include <Gyroscope_Wii.h>

  // Accelerometer declaration
  #include <Accelerometer_WII.h>

  // Receiver declaration
  #define RECEIVER_328P

  // Motor declaration
  #define MOTOR_PWM

  // heading mag hold declaration
  // unsupported on mega v1
  #undef AltitudeHoldBaro
  #undef AltitudeHoldRangeFinder  
  #undef HeadingMagHold
  #undef BattMonitor
  #undef BattMonitorAutoDescent
  #undef POWERED_BY_VIN        
  #undef CameraControl
  #undef OSD
  #undef UseGPS
  #undef UseGPSNavigator

  /**
   * Put AeroQuad_Wii specific initialization need here
   */
  void initPlatform() {
     Wire.begin();

     #if defined(AeroQuad_Paris_v3)
       initializeWiiSensors(true);
     #else
       initializeWiiSensors();
     #endif
  }
  
  // called when eeprom is initialized
  void initializePlatformSpecificAccelCalibration() {
    // Accel Cal
    accelScaleFactor[XAXIS] = 1.0;
    runTimeAccelBias[XAXIS] = 0.0;
    accelScaleFactor[YAXIS] = 1.0;
    runTimeAccelBias[YAXIS] = 0.0;
    accelScaleFactor[ZAXIS] = 1.0;
    runTimeAccelBias[ZAXIS] = 0.0;
  }

  /**
   * Measure critical sensors
   */
  void measureCriticalSensors() {
    if (deltaTime >= 10000) {
      readWiiSensors();
      measureGyro();
      measureAccel();
    }
  }
#endif

#ifdef AeroQuadMega_Wii
  #define LED_GREEN_PIN 13
  #define LED_BLUE_PIN 4
  #define LED_WHITE_PIN  31

  #include <Device_I2C.h>

  // Platform Wii declaration
  #include <Platform_Wii.h>

  // Gyroscope declaration
  #include <Gyroscope_Wii.h>

  // Accelerometer declaration
  #include <Accelerometer_WII.h>

  // Receiver declaration
  #define RECEIVER_MEGA

  // Motor declaration
  #define MOTOR_PWM

  // heading mag hold declaration
  #ifdef HeadingMagHold
    #include <Compass.h>
    #define HMC5843
  #endif

  // Altitude declaration
  #ifdef AltitudeHoldBaro
    #define BMP085
  #endif
  #ifdef AltitudeHoldRangeFinder
    #define XLMAXSONAR 
  #endif

  // Battery monitor declaration
  #ifdef BattMonitor
    #define BattDefaultConfig DEFINE_BATTERY(0, 0, 15.0, 0.9, BM_NOPIN, 0, 0)
  #else
    #undef BattMonitorAutoDescent
    #undef POWERED_BY_VIN        
  #endif

  #ifdef OSD
    #define MAX7456_OSD
  #endif
  
  #undef UseGPS        // Wii not enough stable to use gps
  #undef UseGPSNavigator

  /**
   * Put AeroQuadMega_Wii specific initialization need here
   */
  void initPlatform() {
    Wire.begin();
    initializeWiiSensors();
  }
  
  // called when eeprom is initialized
  void initializePlatformSpecificAccelCalibration() {
    // Accel Cal
    accelScaleFactor[XAXIS] = 1.0;
    runTimeAccelBias[XAXIS] = 0.0;
    accelScaleFactor[YAXIS] = 1.0;
    runTimeAccelBias[YAXIS] = 0.0;
    accelScaleFactor[ZAXIS] = 1.0;
    runTimeAccelBias[ZAXIS] = 0.0;
    #ifdef HeadingMagHold
      magBias[XAXIS]  = 0.0;
      magBias[YAXIS]  = 0.0;
      magBias[ZAXIS]  = 0.0;
    #endif
  }

  /**
   * Measure critical sensors
   */
  void measureCriticalSensors() {
    if (deltaTime >= 10000) {
      readWiiSensors();
      measureGyro();
      measureAccel();
    }
  }
#endif

#ifdef AeroQuadMega_CHR6DM
  #define LED_GREEN_PIN 13
  #define LED_BLUE_PIN 4
  #define LED_WHITE_PIN  31

  #include <Device_I2C.h>
  #include <Platform_CHR6DM.h>
  CHR6DM chr6dm;

  // Gyroscope declaration
  #include <Gyroscope_CHR6DM.h>

  // Accelerometer declaration
  #include <Accelerometer_CHR6DM.h>

  // Receiver declaration
  #define RECEIVER_MEGA

  // Motor declaration
  #define MOTOR_PWM

  // Kinematics declaration
  #include "Kinematics_CHR6DM.h"

  // Compass declaration
  #define HeadingMagHold
  #define COMPASS_CHR6DM
  #include <Magnetometer_CHR6DM.h>

  #ifdef OSD
    #define MAX7456_OSD
  #endif

  // Altitude declaration
  #ifdef AltitudeHoldBaro
    #define BMP085
  #endif
  #ifdef AltitudeHoldRangeFinder
    #define XLMAXSONAR 
  #endif

  // Battery monitor declaration
  #ifdef BattMonitor
    #define BattDefaultConfig DEFINE_BATTERY(0, 0, 13.35, 0.9, BM_NOPIN, 0, 0)
  #else
    #undef BattMonitorAutoDescent
    #undef POWERED_BY_VIN        
  #endif
  
  #ifndef UseGPS
    #undef UseGPSNavigator
  #endif

  /**
   * Put AeroQuadMega_CHR6DM specific initialization need here
   */
  void initPlatform() {
    Serial1.begin(BAUD);
    PORTD = B00000100;

    Wire.begin();

    chr6dm.resetToFactory();
    chr6dm.setListenMode();
    chr6dm.setActiveChannels(CHANNEL_ALL_MASK);
    chr6dm.requestPacket();

    gyroChr6dm = &chr6dm;
    accelChr6dm = &chr6dm;
    kinematicsChr6dm = &chr6dm;
    compassChr6dm = &chr6dm;
  }
  
  // called when eeprom is initialized
  void initializePlatformSpecificAccelCalibration() {
    // Accel Cal
    accelScaleFactor[XAXIS] = 1.0;
    runTimeAccelBias[XAXIS] = 0.0;
    accelScaleFactor[YAXIS] = 1.0;
    runTimeAccelBias[YAXIS] = 0.0;
    accelScaleFactor[ZAXIS] = 1.0;
    runTimeAccelBias[ZAXIS] = 0.0;
  }


  /**
   * Measure critical sensors
   */
  void measureCriticalSensors() {
    if (deltaTime >= 10000) {
      chr6dm.read();
      measureGyro();
      measureAccel();
    }
  }
#endif

#ifdef APM_OP_CHR6DM
  #define LED_GREEN_PIN 37
  #define LED_BLUE_PIN 35
  #define LED_WHITE_PIN  36

  #include <Device_I2C.h>
  #include <Platform_CHR6DM.h>
  CHR6DM chr6dm;

  // Gyroscope declaration
  #include <Gyroscope_CHR6DM.h>

  // Accelerometer declaration
  #include <Accelerometer_CHR6DM.h>

  // Receiver declaration
  #define RECEIVER_APM

  // Motor declaration
  #define MOTOR_APM

  // Kinematics declaration
  #include "Kinematics_CHR6DM.h"

  // Compass declaration
  #define HeadingMagHold
  #define COMPASS_CHR6DM
  #include <Magnetometer_CHR6DM.h>

  // Altitude declaration
  #ifdef AltitudeHoldBaro
    #define BMP085
  #endif
  #ifdef AltitudeHoldRangeFinder
    #define XLMAXSONAR 
  #endif
  

  // Battery monitor declaration
  #ifdef BattMonitor
    #define BattDefaultConfig DEFINE_BATTERY(0, 0, 13.35, 0.31, BM_NOPIN, 0, 0)
  #else
    #undef BattMonitorAutoDescent
    #undef POWERED_BY_VIN        
  #endif

  #undef CameraControl
  #undef OSD
  
  #ifndef UseGPS
    #undef UseGPSNavigator
  #endif

  /**
   * Put APM_OP_CHR6DM specific initialization need here
   */
  void initPlatform() {
    pinMode(LED_BLUE_PIN, OUTPUT);
    pinMode(LED_WHITE_PIN , OUTPUT);
    pinMode(LED_GREEN_PIN, OUTPUT);

    Serial1.begin(BAUD);
    PORTD = B00000100;

    Wire.begin();

    chr6dm.resetToFactory();
    chr6dm.setListenMode();
    chr6dm.setActiveChannels(CHANNEL_ALL_MASK);
    chr6dm.requestPacket();

    gyroChr6dm = &chr6dm;
    accelChr6dm = &chr6dm;
    kinematicsChr6dm = &chr6dm;
//    tempKinematics.setGyroscope(&gyroSpecific);
    compassChr6dm = &chr6dm;
  }

  // called when eeprom is initialized
  void initializePlatformSpecificAccelCalibration() {
    // Accel Cal
    accelScaleFactor[XAXIS] = 1.0;
    runTimeAccelBias[XAXIS] = 0.0;
    accelScaleFactor[YAXIS] = 1.0;
    runTimeAccelBias[YAXIS] = 0.0;
    accelScaleFactor[ZAXIS] = 1.0;
    runTimeAccelBias[ZAXIS] = 0.0;
  }

  /**
   * Measure critical sensors
   */
  void measureCriticalSensors() {
    if (deltaTime >= 10000) {
      chr6dm.read();
      measureGyro();
      measureAccel();
    }
  }
#endif
//**********************DuePi*****************************
#ifdef DuePi
/* http://radioelf.blogspot.com.es/
-MS5611 Barometric pressure sensor (Bus SPI)
-GPS M8N
-EEPROM 24AA32
-PCA9685 Generator PWM (Bus I2c 0x40 and 0x70(ALLCALL)) 
-ADS1115 ADC 16bits (Bus I2C 0x48) 
-MPU9250 Gyroscope / Accelerometer/9-AXIS (Bus SPI) 
-HMC5883L, external compass, (I2C 0x1E) (M8N)
-MCP2515 CAN Bus (Bus SPI) NOT used
*/
  #include <Device_I2C.h>                                               // pin20->SDA, pin21->SCL

  // Altitude declaration
  #ifdef AltitudeHoldBaro
    #define MS5611_SPI                                                 // libraries/AQ_BarometricSensor/BarometricSensor_MS5611_spi.h
  #endif
    #ifdef AltitudeHoldRangeFinder
    #define SRF08SONAR 
  #endif

  // Accelerometer, Gyroscope and compass declaration
  #ifdef MPU6050
    #include <Platform_MPU6000.h>
    #include <Gyroscope_MPU6000.h> 
    #include <Accelerometer_MPU6000.h>
  #else
    #include <MPU9250.h>                                                 //  https://github.com/brianc118/MPU9250
    #define SPI_CLOCK 8000000                                            // 8MHz clock  SPI
    MPU9250 mpu(SPI_CLOCK, CS_MPU_PIN, BITS_DLPF_CFG_42HZ, BITS_DLPF_CFG_42HZ);// 8Mhz, pin 87->PA29, low pass filter 42hz, Acc Data rate 1Khz
    #include <Gyroscope_MPU9250.h> 
    #include <Accelerometer_MPU9250.h> 
  #endif
 
  #ifdef HeadingMagHold
    #include <Compass.h>
    #define HMC5883L_M8N
    //#define Magnetometer_MPU9250
  #endif

  // Receiver Declaration
  #if !(defined(ReceiverSBUS) || defined(ReceiverIBUS))
    #define RECEIVER_DUE
  #endif

  // Motor declaration
  #define PCA9685
  
  // Battery Monitor declaration
  #ifdef BattMonitor
    #ifdef POWERED_BY_VIN
      #define BattDefaultConfig DEFINE_BATTERY(0, 0, 15.0, 0, BM_NOPIN, 0, 0) // v2 shield powered via VIN (no diode)
    #else
                              //DEFINE_BATTERY(CELLS,VPIN,VSCALE,VBIAS,CPIN,CSCALE,CBIAS) 
      #define BattDefaultConfig DEFINE_BATTERY(3, 0, 02.06, 0.0, 1, 29.5, 0) // v2 shield powered via power jack
    #endif
  #else
    #undef BattMonitorAutoDescent
    #undef POWERED_BY_VIN        
  #endif

  #ifdef OSD
    #define MAX7456_OSD
  #endif  
  
  #ifdef UseGPS
    //#define UseGPSNavigator
    #undef UseGPSNavigator
  #endif

//Defines so the device can do a self reset
#define SYSRESETREQ    (1<<2)
#define VECTKEY        (0x05fa0000UL)
#define VECTKEY_MASK   (0x0000ffffUL)
#define AIRCR          (*(uint32_t*)0xe000ed0cUL) // fixed arch-defined address
#define REQUEST_EXTERNAL_RESET (AIRCR=(AIRCR&VECTKEY_MASK)|VECTKEY|SYSRESETREQ)

  /**
   * Put DuePi specific initialization need here
   */
  void initPlatform() {
    #if defined watchdog
      watchdogReset();                                                // Reset watchdog
    #endif
    for (uint8_t x = 0; x < (sizeof(NO_user_PIN)/sizeof(NO_user_PIN[0])); x++){ 
      pinMode(NO_user_PIN[x], OUTPUT);                                // All pins NOT used as output
      digitalWrite(NO_user_PIN[x], LOW);                              // low->0
    }

    pinMode(AUX1_PIN, OUTPUT);
    digitalWrite(AUX1_PIN, LOW);
    pinMode(AUX2_PIN, OUTPUT);
    digitalWrite(AUX2_PIN, LOW);
    pinMode(AUX3_PIN, OUTPUT);
    digitalWrite(AUX3_PIN, LOW);
    pinMode(AUX4_PIN, OUTPUT);
    digitalWrite(AUX4_PIN, LOW);
    pinMode(AUX5_PIN, OUTPUT);
    digitalWrite(AUX5_PIN, LOW);
    pinMode(LED_GREEN_PIN, OUTPUT);                                  // Led Status
    digitalWrite(LED_GREEN_PIN, LOW);
    pinMode(LED_BLUE_PIN, OUTPUT);                                   // status battery
    digitalWrite(LED_BLUE_PIN, LOW);
    pinMode(LED_WHITE_PIN, OUTPUT);
    digitalWrite(LED_WHITE_PIN, LOW);                                // Mode switch for Acro or Stable
    pinMode(LED_L_PIN, OUTPUT);                                      
    digitalWrite(LED_L_PIN, LOW);                                    // mode standby

    #if defined BUZZER
      pinMode(BUZZER_PIN, OUTPUT);                                   // PIN 66-PB15, Max. output current 3mA!!
      digitalWrite(BUZZER_PIN, LOW);
      #if defined watchdog
        watchdogReset();                                             // Reset watchdog
      #endif
      delayMicroseconds (10000);
      digitalWrite(BUZZER_PIN, HIGH);
    #endif
    pinMode(ON_OFF_PIN, INPUT);                                      // pin ON/OFF as standby
    
    pinMode(CS_BARO_PIN, OUTPUT);
    digitalWrite(CS_BARO_PIN, HIGH);
    pinMode(CS_CAN_PIN, OUTPUT);
    digitalWrite(CS_CAN_PIN, HIGH);
    pinMode(CS_MPU_PIN, OUTPUT);
    digitalWrite(CS_MPU_PIN, HIGH);
    pinMode(INT_MPU_PIN, INPUT);
    pinMode(PWM_EN_PIN, OUTPUT);
    digitalWrite(PWM_EN_PIN, HIGH);                                   // pin PWM_EN_PIN a 1 (Disabled)
  
    //SPI.setDataMode(SPI_MODE3);
    //SPI.setBitOrder(MSBFIRST);
    SPI.setClockDivider(84);                                          // 84Mhz /84->1Mhz
    SPI.begin ();

    Wire.setClock(400000);                                            // 400kHz I2C clock. 
    Wire.begin();                                                     // pinPB12->SDA, pinPB13->SDL

    #ifdef MPU6050
      initializeMPU6000Sensors();
    #else   
      mpu.init(true, true);
      wai = mpu.whoami();                                            // read response MPU9250     
    #endif

    #if defined watchdog
      watchdogReset();                                               // Reset watchdog
    #endif
  }
  
  // called when eeprom is initialized
  void initializePlatformSpecificAccelCalibration() {
    // Kenny default value, a real accel calibration is strongly recommended
    accelScaleFactor[XAXIS] = 0.0365570020;
    accelScaleFactor[YAXIS] = 0.0363000011;
    accelScaleFactor[ZAXIS] = -0.0384629964;
    #ifdef HeadingMagHold
      magBias[XAXIS]  = 1.500000;
      magBias[YAXIS]  = 205.500000;
      magBias[ZAXIS]  = -33.000000;
    #endif

  }

  /**
   * Measure critical sensors
   */
  void measureCriticalSensors() {
    #if defined AK8963FASTMODE
      measureGyroSum();
      measureAccelSum();
    #else
      measureGyro();
      measureAccel();
    #endif
  }
#endif
//********END**************DuePi***********END************

//********************************************************
//********************************************************
//********* HARDWARE GENERALIZATION SECTION **************
//********************************************************
//********************************************************

#ifdef AeroQuadSTM32
  #include "AeroQuad_STM32.h"
#endif

#ifndef ADC_NUMBER_OF_BITS
  #if defined DuePi
    #define ADC_NUMBER_OF_BITS 16 // ADC1115
  #else
    #define ADC_NUMBER_OF_BITS 10 // default to 10bit ADC (AVR)
  #endif
#endif

//********************************************************
//****************** KINEMATICS DECLARATION **************
//********************************************************
#include "Kinematics.h"
#if defined(AeroQuadMega_CHR6DM) || defined(APM_OP_CHR6DM)
  // CHR6DM have it's own kinematics, so, initialize in it's scope
#else
  #include "Kinematics_ARG.h"  
#endif

//********************************************************
//******************** RECEIVER DECLARATION **************
//********************************************************
#if defined(ReceiverHWPPM)
  #include <Receiver_HWPPM.h>
#elif defined(ReceiverPPM)
  #include <Receiver_PPM.h>
#elif defined(AeroQuad_Mini) && (defined(hexPlusConfig) || defined(hexXConfig) || defined(hexY6Config))
  #include <Receiver_PPM.h>
#elif defined(RemotePCReceiver)
  #include <Receiver_RemotePC.h>
#elif defined(ReceiverSBUS)
  #include <Receiver_SBUS.h>
#elif defined(ReceiverIBUS)
  #include <Receiver_IBUS.h>
#elif defined(RECEIVER_328P)
  #include <Receiver_328p.h>
#elif defined(RECEIVER_MEGA)
  #include <Receiver_MEGA.h>
#elif defined(RECEIVER_DUE)
  #include <Receiver_APM_Due.h>
#elif defined(RECEIVER_APM)
  #include <Receiver_APM.h>
#elif defined(RECEIVER_STM32PPM)
  #include <Receiver_STM32PPM.h>  
#elif defined(RECEIVER_STM32)
  #include <Receiver_STM32.h>  
#endif

#if defined(UseAnalogRSSIReader) 
  #include <AnalogRSSIReader.h>
#elif defined(UseEzUHFRSSIReader)
  #include <EzUHFRSSIReader.h>
#elif defined(UseSBUSRSSIReader)
  #include <SBUSRSSIReader.h>
#endif

//********************************************************
//********************** MOTORS DECLARATION **************
//********************************************************
#if defined(triConfig)
  #if defined (MOTOR_STM32)
    #define MOTORS_STM32_TRI
    #include <Motors_STM32.h>    
  #else
    #include <Motors_Tri.h>
  #endif
#elif defined(MOTOR_PWM)
  #include <Motors_PWM.h>
#elif defined(MOTOR_PWM_Timer)
  #include <Motors_PWM_Timer.h>
#elif defined(MOTOR_APM)
  #include <Motors_APM.h>
#elif defined(MOTOR_I2C)
  #include <Motors_I2C.h>
#elif defined(MOTOR_STM32)
  #include <Motors_STM32.h> 
#elif defined(PCA9685)
  #include <Motors_PCA9685.h> 
#endif

//********************************************************
//******* HEADING HOLD MAGNETOMETER DECLARATION **********
//********************************************************
#if defined(HMC5843)
  #include <HeadingFusionPrcessorMARG.h>
  #include <Magnetometer_HMC5843.h>
#elif defined(SPARKFUN_9DOF_5883L) || defined(SPARKFUN_5883L_BOB) || defined(HMC5883L) || defined(HMC5883L_M8N) || defined(Magnetometer_MPU9250)
  #include <HeadingFusionProcessorMARG.h>
  #if defined (Magnetometer_MPU9250)
    #include <Magnetometer_MPU9250.h>
  #else
    #include <Magnetometer_HMC5883L.h>
  #endif
#elif defined(COMPASS_CHR6DM)
#endif
//********************************************************
//******* ALTITUDE HOLD BAROMETER DECLARATION ************
//********************************************************
#if defined(BMP085)
  #include <BarometricSensor_BMP085.h>
#elif defined(MS5611)
 #include <BarometricSensor_MS5611.h>      // I2C
#elif defined(MS5611_SPI) 
  #include <BarometricSensor_MS5611_SPI.h> // SPI
#endif
#if defined(XLMAXSONAR)
  #include <MaxSonarRangeFinder.h>
#endif 
#if defined(SRF08SONAR)
  #include <Srf08SonarRangeFinder.h>
#endif 
//********************************************************
//*************** BATTERY MONITOR DECLARATION ************
//********************************************************
#ifdef BattMonitor  
  #if defined DuePi
    #include <Adafruit_ADS1015.h>    
    Adafruit_ADS1115 ads1115;                           // 16 bits
    #include <BatteryMonitor_ADS.h>
    #include <BatteryMonitor.h>
  #else
    #include <BatteryMonitor.h>
  #endif
  #ifndef BattCustomConfig
    #define BattCustomConfig BattDefaultConfig
  #endif
  struct BatteryData batteryData[] = {BattCustomConfig};
#endif
//********************************************************
//************** CAMERA CONTROL DECLARATION **************
//********************************************************
// used only on mega for now
#if defined(CameraControl_STM32)
  #include <CameraStabilizer_STM32.h>
#elif defined(CameraControl)
  #include <CameraStabilizer_Aeroquad.h>
#endif

#if defined (CameraTXControl)
  #include <CameraStabilizer_TXControl.h>
#endif

//********************************************************
//******** FLIGHT CONFIGURATION DECLARATION **************
//********************************************************
#if defined(quadXConfig)
  #include "FlightControlQuadX.h"
#elif defined(quadPlusConfig)
  #include "FlightControlQuadPlus.h"
#elif defined(hexPlusConfig)
  #include "FlightControlHexPlus.h"
#elif defined(hexXConfig)
  #include "FlightControlHexX.h"
#elif defined(triConfig)
  #include "FlightControlTri.h"
#elif defined(quadY4Config)
  #include "FlightControlQuadY4.h"
#elif defined(hexY6Config)
  #include "FlightControlHexY6.h"
#elif defined(octoX8Config)
  #include "FlightControlOctoX8.h"
#elif defined(octoXConfig)
  #include "FlightControlOctoX.h"
#elif defined(octoPlusConfig)
  #include "FlightControlOctoPlus.h"
#endif

//********************************************************
//****************** GPS DECLARATION *********************
//********************************************************
#if defined(UseGPS) || defined(UseGPS_UBLOX)
  #if !defined(HeadingMagHold)
    #error We need the magnetometer to use the GPS
  #endif 
  #include <GpsAdapter.h>
  #include "GpsNavigator.h"
#endif

//********************************************************
//****************** OSD DEVICE DECLARATION **************
//********************************************************
#ifdef MAX7456_OSD     // only OSD supported for now is the MAX7456
  #include <Device_SPI.h>
  #include "OSDDisplayController.h"
  #include "MAX7456.h"
#endif

#if defined(SERIAL_LCD)
  #include "SerialLCD.h"
#endif

#ifdef OSD_SYSTEM_MENU
  #if !defined(MAX7456_OSD) && !defined(SERIAL_LCD)
    #error "Menu cannot be used without OSD or LCD"
  #endif
  #include "OSDMenu.h"
#endif

//********************************************************
//****************** SERIAL PORT DECLARATION *************
//********************************************************
#if defined(WirelessTelemetry) 
  #if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
    #define SERIAL_PORT Serial3
  #else    // force 328p to use the normal port
    #define SERIAL_PORT Serial
  #endif
#else  
  #if defined(SERIAL_USES_USB)   // STM32 Maple
    #define SERIAL_PORT SerialUSB
    #undef BAUD
    #define BAUD
  #else
    #define SERIAL_PORT Serial
  #endif
#endif  

#ifdef SlowTelemetry
  #include <AQ_RSCode.h>
#endif

#ifdef SoftModem
  #include <AQ_SoftModem.h>
#endif

// Include this last as it contains objects from above declarations
#include "AltitudeControlProcessor.h"
#include "FlightControlProcessor.h"
#include "FlightCommandProcessor.h"
#include "HeadingHoldProcessor.h"
#include "DataStorage.h"

#if defined(UseGPS) || defined(BattMonitor)
  #include "LedStatusProcessor.h"
#endif  
#if defined(MavLink)
  #include "MavLink.h"
#else
  #include "SerialCom.h"
#endif

/*******************************************************************
 * Main setup function, called one time at bootup
 * initialize all system and sub system of the
 * Aeroquad
 ******************************************************************/
void setup() {
  #if defined watchdog
    watchdogReset();                                                 // Reset watchdog  
  #endif
  SERIAL_BEGIN(BAUD);
  pinMode(LED_GREEN_PIN, OUTPUT);
  digitalWrite(LED_GREEN_PIN, LOW);
  initCommunication();  
  #ifdef DuePi
    Wire1.begin();                                                   // pin70->SDA1, pin71->SCL1 (EEPROM 24AA32)   
    pinMode(ENABL_PIN, OUTPUT);
    digitalWrite(ENABL_PIN, HIGH);                                   // ON DronPi 
  #endif
  #if defined watchdog 
    watchdogReset();                                                 // Reset watchdog                               
  #endif
  readEEPROM();                                                      // defined in DataStorage.h
  #if defined watchdog
    watchdogReset();                                                 // Reset watchdog 
  #endif
  boolean firstTimeBoot = false;

  if (readFloat(SOFTWARE_VERSION_ADR) != float (SOFTWARE_VERSION)) { // If we detect the wrong soft version, we init all parameters
    #if defined watchdog
      watchdogReset();                                               // Reset watchdog 
    #endif
    initializeEEPROM();
    #if defined watchdog
      watchdogReset();                                               // Reset watchdog 
    #endif
    writeEEPROM();
    #if defined watchdog
      watchdogReset();                                               // Reset watchdog 
    #endif
    firstTimeBoot = true;
  }
  initPlatform();
  #if defined(quadXConfig) || defined(quadPlusConfig) || defined(quadY4Config) || defined(triConfig)
     initializeMotors(FOUR_Motors);
  #elif defined(hexPlusConfig) || defined(hexXConfig) || defined(hexY6Config)
     initializeMotors(SIX_Motors);
  #elif defined(octoX8Config) || defined(octoXConfig) || defined(octoPlusConfig)
     initializeMotors(EIGHT_Motors);
  #endif
  initializeReceiver();  
  #if defined watchdog
    watchdogReset();                                                // Reset watchdog                                          
  #endif
  initReceiverFromEEPROM();
  #if defined watchdog
    watchdogReset();                                                // Reset watchdog 
  #endif
  // Initialize sensors
  // If sensors have a common initialization routine
  // insert it into the gyro class because it executes first
  #ifdef DuePi
    #if !defined(MPU6050)
      initializeGyro(wai); 
      mpu.calib_acc();
      mpu.calib_mag();
     #endif 
    #if defined watchdog
      watchdogReset();                                              // Reset watchdog
    #endif
  #else
    initializeGyro();                                               // defined in Gyro.h
    while (!calibrateGyro());                                       // this make sure the craft is still befor to continue init process
  #endif
  initializeAccel();                                                // defined in Accel.h
  #if defined watchdog
    watchdogReset();                                                // Reset watchdog
  #endif
  #if defined(Magnetometer_MPU9250)
    initializeMagnetometer();
  #endif
  if (firstTimeBoot) {
    computeAccelBias();
    writeEEPROM();
    #if defined watchdog
      watchdogReset();                                              // Reset watchdog
    #endif  
  }
  setupFourthOrder();
  #if defined watchdog
    watchdogReset();                                                // Reset watchdog
  #endif
  initSensorsZeroFromEEPROM();
  #if defined watchdog
    watchdogReset();                                                // Reset watchdog
  #endif
  // Integral Limit for attitude mode
  // This overrides default set in readEEPROM()
  // Set for 1/2 max attitude command (+/-0.75 radians)
  // Rate integral not used for now
  PID[ATTITUDE_XAXIS_PID_IDX].windupGuard = 0.375;
  PID[ATTITUDE_YAXIS_PID_IDX].windupGuard = 0.375;
  
  // Flight angle estimation
  initializeKinematics(); // kinematicsAngle[XAXIS and kinematicsAngle[YAXIS} ->0.0
  #ifdef HeadingMagHold
    vehicleState |= HEADINGHOLD_ENABLED;
    initializeMagnetometer();
    initializeHeadingFusion();
  #endif
  #if defined watchdog
    watchdogReset();                                               // Reset watchdog
  #endif
  // Optional Sensors
  #ifdef AltitudeHoldBaro
    initializeBaro();
    vehicleState |= ALTITUDEHOLD_ENABLED;
  #endif
  #ifdef AltitudeHoldRangeFinder
    inititalizeRangeFinders();
    #ifdef SRF08SONAR 
      if (SRF08State)
        vehicleState |= RANGE_ENABLED;
    #else
      vehicleState |= RANGE_ENABLED;
    #endif
    PID[SONAR_ALTITUDE_HOLD_PID_IDX].P = PID[BARO_ALTITUDE_HOLD_PID_IDX].P*2;
    PID[SONAR_ALTITUDE_HOLD_PID_IDX].I = PID[BARO_ALTITUDE_HOLD_PID_IDX].I;
    PID[SONAR_ALTITUDE_HOLD_PID_IDX].D = PID[BARO_ALTITUDE_HOLD_PID_IDX].D;
    PID[SONAR_ALTITUDE_HOLD_PID_IDX].windupGuard = PID[BARO_ALTITUDE_HOLD_PID_IDX].windupGuard;
  #endif
  #if defined watchdog
    watchdogReset();                                            // Reset watchdog
  #endif
  #ifdef BattMonitor
    #if defined DuePi
      ads1115.setGain(GAIN);
    #endif
    initializeBatteryMonitor(sizeof(batteryData) / sizeof(struct BatteryData), batteryMonitorAlarmVoltage);
    vehicleState |= BATTMONITOR_ENABLED;
  #endif
  
  #if defined(CameraControl)
    initializeCameraStabilization();
    vehicleState |= CAMERASTABLE_ENABLED;
  #endif
  #if defined watchdog
    watchdogReset();                                            // Reset watchdog
  #endif
  #if defined(MAX7456_OSD)
    initializeSPI();
    initializeOSD();
  #endif
  
  #if defined(SERIAL_LCD)
    InitSerialLCD();
  #endif
  #if defined watchdog
    watchdogReset();                                          // Reset watchdog
  #endif
  #if defined(BinaryWrite) || defined(BinaryWritePID)
    #ifdef OpenlogBinaryWrite
      binaryPort = &Serial1;
      binaryPort->begin(115200);
      delay(500);
      #if defined watchdog
        watchdogReset();                                      // Reset watchdog 
      #endif
    #else
     binaryPort = &Serial;
    #endif
  #endif
  
  #if defined(UseGPS)
    initializeGps();
  #endif 

  #ifdef SlowTelemetry
     initSlowTelemetry();
  #endif
  #ifdef DuePi
    measureGyro();
    measureAccel();
  #endif
  previousTime = micros();
  digitalWrite(LED_GREEN_PIN, HIGH);
  safetyCheck = 0;
  #if defined watchdog
    watchdogReset();                                      // Reset watchdog
  #endif
  #if defined BUZZER
    buzzer(100, 2);                                       // ms, cicles
  #endif
}

/*******************************************************************
 * 100Hz task
 ******************************************************************/
void process100HzTask() {
  G_Dt = (currentTime - hundredHZpreviousTime) / 1000000.0;
  hundredHZpreviousTime = currentTime;

  evaluateGyroRate();
  evaluateMetersPerSec();

  for (int axis = XAXIS; axis <= ZAXIS; axis++) {
    filteredAccel[axis] = computeFourthOrder(meterPerSecSec[axis], &fourthOrder[axis]); 
  }

 calculateKinematics(gyroRate[XAXIS], gyroRate[YAXIS], gyroRate[ZAXIS], filteredAccel[XAXIS], filteredAccel[YAXIS], filteredAccel[ZAXIS], G_Dt);

  
  #if defined AltitudeHoldBaro || defined AltitudeHoldRangeFinder
    zVelocity = (filteredAccel[ZAXIS] * (1 - accelOneG * invSqrt(isq(filteredAccel[XAXIS]) + isq(filteredAccel[YAXIS]) + isq(filteredAccel[ZAXIS])))) - runTimeAccelBias[ZAXIS] - runtimeZBias;
    if (!runtimaZBiasInitialized) {
      runtimeZBias = (filteredAccel[ZAXIS] * (1 - accelOneG * invSqrt(isq(filteredAccel[XAXIS]) + isq(filteredAccel[YAXIS]) + isq(filteredAccel[ZAXIS])))) - runTimeAccelBias[ZAXIS];
      runtimaZBiasInitialized = true;
    }
    estimatedZVelocity += zVelocity;
    estimatedZVelocity = (velocityCompFilter1 * zVelocity) + (velocityCompFilter2 * estimatedZVelocity);
  #endif    

  #if defined(AltitudeHoldBaro)
    measureBaroSum(); 
    if (frameCounter % THROTTLE_ADJUST_TASK_SPEED == 0) {  //  50 Hz tasks
      evaluateBaroAltitude();
    }
  #endif
        
  processFlightControl();
   
  #if defined(BinaryWrite)
    if (fastTransfer == ON) {
      // write out fastTelemetry to Configurator or openLog
      fastTelemetry();
    }
  #endif      
  
  #ifdef SlowTelemetry
    updateSlowTelemetry100Hz();
  #endif

  #if defined(UseGPS)
    updateGps();
  #endif      
  
  #if defined(CameraControl)
    moveCamera(kinematicsAngle[YAXIS],kinematicsAngle[XAXIS],kinematicsAngle[ZAXIS]);
    #if defined CameraTXControl
      processCameraTXControl();
    #endif
  #endif       

}

/*******************************************************************
 * 50Hz task
 ******************************************************************/
void process50HzTask() {
  G_Dt = (currentTime - fiftyHZpreviousTime) / 1000000.0;
  fiftyHZpreviousTime = currentTime;
  // Reads external pilot commands and performs functions based on stick configuration
  readPilotCommands(); 
  
  #if defined(UseAnalogRSSIReader) || defined(UseEzUHFRSSIReader) || defined(UseSBUSRSSIReader)
    readRSSI();
  #endif

  #ifdef AltitudeHoldRangeFinder
    #ifdef SRF08SONAR 
      if (SRF08State && motorArmed)
        updateRangeFinders();
    #else
      updateRangeFinders();
    #endif
  #endif

  #if defined(UseGPS)
    if (haveAGpsLock() && !isHomeBaseInitialized()) {
      initHomeBase();
    }
  #endif      
}

/*******************************************************************
 * 10Hz task
 ******************************************************************/
void process10HzTask1() {
  
  #if defined(HeadingMagHold)
  
    G_Dt = (currentTime - tenHZpreviousTime) / 1000000.0;
    tenHZpreviousTime = currentTime;
     
    measureMagnetometer(kinematicsAngle[XAXIS], kinematicsAngle[YAXIS]);
    
    calculateHeading();
    
  #endif
}

/*******************************************************************
 * low priority 10Hz task 2
 ******************************************************************/
void process10HzTask2() {
  G_Dt = (currentTime - lowPriorityTenHZpreviousTime) / 1000000.0;
  lowPriorityTenHZpreviousTime = currentTime;
  
  #if defined(BattMonitor)
    measureBatteryVoltage(G_Dt*1000.0);                                
  #endif
  // Listen for configuration commands and reports telemetry
  readSerialCommand();
  sendSerialTelemetry();
}

/*******************************************************************
 * low priority 10Hz task 3
 ******************************************************************/
void process10HzTask3() {
    G_Dt = (currentTime - lowPriorityTenHZpreviousTime2) / 1000000.0;
    lowPriorityTenHZpreviousTime2 = currentTime;

    #ifdef OSD_SYSTEM_MENU
      updateOSDMenu();
    #endif

    #ifdef MAX7456_OSD
      updateOSD();
    #endif
    
    #if defined(UseGPS) || defined(BattMonitor)
      processLedStatus();
    #endif
    
    #ifdef SlowTelemetry
      updateSlowTelemetry10Hz();
    #endif
}

/*******************************************************************
 * 1Hz task 
 ******************************************************************/
void process1HzTask() {
  #if defined watchdog
    watchdogReset();                                // Reset watchdog
  #endif                                
  #ifdef MavLink
    G_Dt = (currentTime - oneHZpreviousTime) / 1000000.0;
    oneHZpreviousTime = currentTime;
    
    sendSerialHeartbeat();   
  #endif
}

/*******************************************************************
 * Main loop funtions
 ******************************************************************/
void loop () {
  
  currentTime = micros();
  deltaTime = currentTime - previousTime;
  // digitalWrite(AUX5_PIN, !digitalRead(AUX5_PIN));             // out frequency loop
 
  // ================================================================
  // 1000Hz task loop 1 ms
  // ================================================================
  #if defined AK8963FASTMODE && defined (DuePi)                 // +- 80us-> 12.5 kHz  (for XT 84MHz) 
    if (deltaTime >=CriticalSensorsMPU){
      CriticalSensorsMPU = (deltaTime+1000);
      measureCriticalSensors();                                 // MPU9250 sample rate to 1ms-> 1 kHz  
    }
  #else
    measureCriticalSensors();                                    // for XT 16Mhz                                 
  #endif
  // ================================================================
  // 100Hz task loop 10 ms
  // ================================================================
  if (deltaTime >= 10000) {   
    frameCounter++;
    
    #if !defined AK8963FASTMODE && defined (DuePi)
      measureCriticalSensors();                                   // MPU9250 sample rate to 10ms-> 100 Hz 
    #endif
    #if defined AK8963FASTMODE && defined (DuePi) 
      CriticalSensorsMPU =1000;
      //measureCriticalSensors();
    #endif
    process100HzTask();

    // ================================================================
    // 50Hz task loop 20 ms
    // ================================================================
    if (frameCounter % TASK_50HZ == 0) {  //  50 Hz tasks
      process50HzTask();
    }

    // ================================================================
    // 10Hz task loop 100 ms
    // ================================================================
    if (frameCounter % TASK_10HZ == 0) {  //   10 Hz tasks
      process10HzTask1();
    }
    else if ((currentTime - lowPriorityTenHZpreviousTime) > 100000) {
      process10HzTask2();
    }
    else if ((currentTime - lowPriorityTenHZpreviousTime2) > 100000) {
      process10HzTask3();
    }
    
    // ================================================================
    // 1Hz task loop 1000 ms
    // ================================================================
    if (frameCounter % TASK_1HZ == 0) {  //   1 Hz tasks
      process1HzTask();
    }
    
    previousTime = currentTime;
  }
  
  if (frameCounter >= 100) {
      frameCounter = 0;
      #ifdef DuePi
        if (digitalRead(ON_OFF_PIN) == LOW){          // standby
          #if defined watchdog
            watchdogReset();                           // Reset watchdog
          #endif
          #if defined BUZZER
            buzzer(500 ,2);                           // ms, cicles
          #endif
          motorArmed = OFF;
          digitalWrite(ENABL_PIN, LOW);               // OFF DronPi, disabled borad DronPi
          digitalWrite(PWM_EN_PIN, HIGH);             // pin PWM_EN_PIN  1 (disabled PWM)
          digitalWrite(LED_GREEN_PIN, HIGH);
          digitalWrite(LED_BLUE_PIN, HIGH);
          digitalWrite(LED_WHITE_PIN, HIGH);
          digitalWrite(LED_L_PIN, LOW);  
          #if defined BUZZER       
            while ((digitalRead(ON_OFF_PIN) == LOW) || cicles_buzzer){
              #if defined watchdog  
                watchdogReset();
              #endif
              delay(150);
          }
          #endif
          #ifdef RECEIVER_DUE 
            detachInterrupt(PPM_PIN);
          #endif
          uint8_t pwm_LED_L =0;
          bool up_down = true; 
          while ((digitalRead(ON_OFF_PIN) == HIGH)){
            delay(15);
            #if defined watchdog
              watchdogReset();                          // Reset watchdog
            #endif
            if (pwm_LED_L ==0) delay(500);
            if (up_down){
              analogWrite(LED_L_PIN, pwm_LED_L++);
              delay(15);
              if (pwm_LED_L ==100){
                pwm_LED_L ==200;
                up_down =false;
              }
            }else{
              analogWrite(LED_L_PIN, pwm_LED_L--);
              if (pwm_LED_L ==0){ 
                up_down =true;
                delay(350);
                #if defined watchdog
                  watchdogReset();                      // Reset watchdog
                #endif
              }
            }
          }
          while ((digitalRead(ON_OFF_PIN) == LOW)){
            #if defined watchdog
              watchdogReset();
            #endif  
            }
          REQUEST_EXTERNAL_RESET;                      // reset hardware
    }
    #endif
  }
}



