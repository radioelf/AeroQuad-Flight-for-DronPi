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
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See thegyroSample
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program. If not, see <http://www.gnu.org/licenses/>.
*/

// SerialCom.pde is responsible for the serial communication for commands and telemetry from the AeroQuad
// This comtains readSerialCommand() which listens for a serial command and it's arguments
// This also contains readSerialTelemetry() which listens for a telemetry request and responds with the requested data
// For more information on each command/telemetry look at: http://aeroquad.com/content.php?117

// Includes re-write / fixes from Aadamson and ala42, special thanks to those guys!
// http://aeroquad.com/showthread.php?1461-We-have-some-hidden-warnings&p=14618&viewfull=1#post14618

#ifndef _AQ_SERIAL_COMM_
#define _AQ_SERIAL_COMM_

char queryType = 'X';
bool debug = false;
uint8_t CicleDebug =0;
  
void initCommunication() {
  // do nothing here for now
}

//***************************************************************************************************
//********************************** Serial Commands ************************************************
//***************************************************************************************************
bool validateCalibrateCommand(uint8_t command)
{
  
  if (readFloatSerial() == float(123.45)) {// use a specific float value to validate full throttle call is being sent 
    if (debug) SERIAL_PRINTLN("Running test ESC");
    motorArmed = OFF;
    calibrateESC = command;
    return true;
  }
  else {
    if (debug) SERIAL_PRINTLN("NO specific validate float value-> 123.45.");
    calibrateESC = 0;
    testCommand = 1000;
    return false;
  }
}

void readSerialPID(unsigned char PIDid) {
  struct PIDdata* pid = &PID[PIDid];
  pid->P = readFloatSerial();
  pid->I = readFloatSerial();
  pid->D = readFloatSerial();
  pid->lastError = 0;
  pid->integratedError = 0;
}

void skipSerialValues(uint8_t number) {
  for(uint8_t i=0; i<number; i++) {
    readFloatSerial();
  }
}

void readSerialCommand() {
  // Check for serial message
  if (SERIAL_AVAILABLE()) {
    queryType = SERIAL_READ();
    switch (queryType) {
    case 'A': // Receive roll and pitch rate mode PID
      if (debug and CicleDebug++ ==0){
        SERIAL_PRINTLN("Send roll and pitch PID values for rate mode.");
        SERIAL_PRINTLN("[Roll P Gain];[Roll I Gain];");
        SERIAL_PRINTLN("[Roll D Gain];[Pitch P Gain];");
        SERIAL_PRINTLN("[Pitch I Gain];[Pitch D Gain];");
        SERIAL_PRINTLN("[Rotation Speed Factor];");
      }
      readSerialPID(RATE_XAXIS_PID_IDX);
      readSerialPID(RATE_YAXIS_PID_IDX);
      rotationSpeedFactor = readFloatSerial();
      break;

    case 'B': // Receive roll/pitch attitude mode PID
      if (debug and CicleDebug++ ==0){
        SERIAL_PRINTLN("Send roll and pitch PID values for attitude mode");
        SERIAL_PRINTLN("B[Roll Accel P Gain];[Roll Accel I Gain];");
        SERIAL_PRINTLN("[Roll Accel D Gain];[Pitch Accel P Gain];");
        SERIAL_PRINTLN("[Pitch Accel I Gain];[Pitch Accel D Gain];");
        SERIAL_PRINTLN("[Roll P Gain];[Roll I Gain];[Roll D Gain];");
        SERIAL_PRINTLN("[Pitch P Gain];[Pitch I Gain];[Pitch D Gain];[Windup Guard];");
      }
      readSerialPID(ATTITUDE_XAXIS_PID_IDX);
      readSerialPID(ATTITUDE_YAXIS_PID_IDX);
      readSerialPID(ATTITUDE_GYRO_XAXIS_PID_IDX);
      readSerialPID(ATTITUDE_GYRO_YAXIS_PID_IDX);
      windupGuard = readFloatSerial(); // defaults found in setup() of AeroQuad.pde
      break;

    case 'C': // Receive yaw PID
      if (debug and CicleDebug++ ==0){
        SERIAL_PRINTLN("Send yaw and heading hold PID values");
        SERIAL_PRINTLN("C[Yaw P Gain];[Yaw I Gain];[Yaw D Gain];");
        SERIAL_PRINTLN("[Heading Hold P];[Heading Hold I];[Heading Hold D];");
        SERIAL_PRINTLN("[Heading Hold off/on (0/1)];");
      }
      readSerialPID(ZAXIS_PID_IDX);
      readSerialPID(HEADING_HOLD_PID_IDX);
      headingHoldConfig = readFloatSerial();
      break;

    case 'D': // Altitude hold PID
      if (debug and CicleDebug++ ==0){
        SERIAL_PRINTLN("Send Altitude Hold PID values");
        SERIAL_PRINTLN("D[Altitude P Gain];[Altitude I Gain];");
        SERIAL_PRINTLN("[Altitude P Gain];[Altitude I Gain];[Altitude D Gain];");
        SERIAL_PRINTLN("[Altitude P Gain];[Altitude I Gain];[Altitude D Gain];");
        SERIAL_PRINTLN("[Altitude P Gain];[Altitude I Gain];[Altitude D Gain];");
        SERIAL_PRINTLN("[Altitude Windup Guard];[Altitude Hold Bump];");
        SERIAL_PRINTLN("[Altitude Hold Panic Stick Movement];");
        SERIAL_PRINTLN("[Minimum Throttle Adjust];[Maximum Throttle Adjust];");
        SERIAL_PRINTLN("[Barometer Smooth Factor];[Z Dampening P Gain];");
        SERIAL_PRINTLN("[Z Dampening I Gain];[Z Dampening D Gain];");
        SERIAL_PRINTLN("[Altitude Windup Guard];[Altitude Hold Bump];");
        SERIAL_PRINTLN("[Altitude Hold Panic Stick Movement];");
        SERIAL_PRINTLN("[Minimum Throttle Adjust];[Maximum Throttle Adjust];");
        SERIAL_PRINTLN("[Barometer Smooth Factor];[Z Dampening P Gain];");
        SERIAL_PRINTLN("[Z Dampening I Gain];[Z Dampening D Gain];");
        SERIAL_PRINTLN("[Altitude Windup Guard];[Altitude Hold Bump];");
        SERIAL_PRINTLN("[Altitude Hold Panic Stick Movement];");
        SERIAL_PRINTLN("[Minimum Throttle Adjust];[Maximum Throttle Adjust];");
        SERIAL_PRINTLN("[Barometer Smooth Factor];[Z Dampening P Gain];");
        SERIAL_PRINTLN("[Z Dampening I Gain];[Z Dampening D Gain];");
        SERIAL_PRINTLN("[Altitude D Gain];[Altitude Windup Guard];");
        SERIAL_PRINTLN("[Altitude Hold Bump];[Altitude Hold Panic Stick Movement];");
        SERIAL_PRINTLN("[Minimum Throttle Adjust];[Maximum Throttle Adjust];");
        SERIAL_PRINTLN("[Barometer Smooth Factor];[Z Dampening P Gain];");
        SERIAL_PRINTLN("[Z Dampening I Gain];[Z Dampening D Gain];");
      }
      #if defined AltitudeHoldBaro || defined AltitudeHoldRangeFinder
        readSerialPID(BARO_ALTITUDE_HOLD_PID_IDX);
        PID[BARO_ALTITUDE_HOLD_PID_IDX].windupGuard = readFloatSerial();
        altitudeHoldBump = readFloatSerial();
        altitudeHoldPanicStickMovement = readFloatSerial();
        minThrottleAdjust = readFloatSerial();
        maxThrottleAdjust = readFloatSerial();
        #if defined AltitudeHoldBaro
          baroSmoothFactor = readFloatSerial();
        #else
          readFloatSerial();
        #endif
        readSerialPID(ZDAMPENING_PID_IDX);
      #endif
      break;

    case 'E': // Receive sensor filtering values
      if (debug and CicleDebug++ ==0){
        SERIAL_PRINTLN("Send miscellaneous config values");
        SERIAL_PRINTLN("E[AREF value];[minArmedThrottle];");
      }
      aref = readFloatSerial();
      minArmedThrottle = readFloatSerial();
      break;

    case 'F': // Receive transmitter smoothing values
      if (debug and CicleDebug++ ==0){
        SERIAL_PRINTLN("Send transmitter smoothing values");
        SERIAL_PRINTLN("F[Transmitter Factor];[Roll Smooth Factor];");
        SERIAL_PRINTLN("[Pitch Smooth Factor];[Yaw Smooth Factor];");
        SERIAL_PRINTLN("[Throttle Smooth Factor];[Mode Smooth Factor];");
        SERIAL_PRINTLN("[AUX1 Smooth Factor];[AUX2 Smooth Factor];[AUX3 Smooth Factor];");
      }
      receiverXmitFactor = readFloatSerial();
      for(uint8_t channel = XAXIS; channel<LASTCHANNEL; channel++) {
        receiverSmoothFactor[channel] = readFloatSerial();
      }
      break;

    case 'G': // Receive transmitter calibration values
      if (debug and CicleDebug++ ==0){
        SERIAL_PRINTLN("Send transmitter calibration values");
        SERIAL_PRINTLN("G[channel number 0-9];[calibration value];");
      }
      channelCal = (int)readFloatSerial();
      receiverSlope[channelCal] = readFloatSerial();
      break;

    case 'H': // Receive transmitter calibration values
      if (debug and CicleDebug++ ==0){
        SERIAL_PRINTLN("Initialize EEPROM with default values");
        SERIAL_PRINTLN("H[channel number 0-9];[offset value];");
      }
      channelCal = (int)readFloatSerial();
      receiverOffset[channelCal] = readFloatSerial();
      break;

    case 'I': // Initialize EEPROM with default values
      if (debug and CicleDebug++ ==0){
        SERIAL_PRINTLN("Initialize EEPROM with default values and calibrate sensors");
      }
      initializeEEPROM(); // defined in DataStorage.h
      writeEEPROM();
      storeSensorsZeroToEEPROM();
      calibrateGyro();
      zeroIntegralError();
      #ifdef HeadingMagHold
        initializeMagnetometer();
      #endif
      #ifdef AltitudeHoldBaro
        initializeBaro();
      #endif
      break;

    case 'J': // calibrate gyros
      if (debug and CicleDebug++ ==0) SERIAL_PRINTLN("Calibrate gyros");
      if (calibrateGyro()){
        if (debug) SERIAL_PRINTLN("Calibrate gyros OK");
      }else
        if (debug) SERIAL_PRINTLN("Calibrate gyros FAULT");
      break;

    case 'K': // Write accel calibration values
      if (debug and CicleDebug++ ==0){
        SERIAL_PRINTLN("Send accel calibration values");
        SERIAL_PRINTLN("K[Accel Scale Factor X Axis];[Run Time Accel Bias X Axis];");
        SERIAL_PRINTLN("[Accel Scale Factor Y Axis];[Run Time Accel Bias Y Axis];");
        SERIAL_PRINTLN("[Accel Scale Factor Z Axis];[Run Time Accel Bias Z Axis];");
      }
      accelScaleFactor[XAXIS] = readFloatSerial();
      readFloatSerial();
      accelScaleFactor[YAXIS] = readFloatSerial();
      readFloatSerial();
      accelScaleFactor[ZAXIS] = readFloatSerial();
      readFloatSerial();
      computeAccelBias();    
      storeSensorsZeroToEEPROM();
      break;

    case 'L': // generate accel bias
      if (debug and CicleDebug++ ==0) SERIAL_PRINTLN("Generate accel bias");
      computeAccelBias();
      #if defined(AeroQuadMega_CHR6DM) || defined(APM_OP_CHR6DM)
        calibrateKinematics();
        accelOneG = meterPerSecSec[ZAXIS];
      #endif
      storeSensorsZeroToEEPROM();
      break;

    case 'M': // calibrate magnetometer
      if (debug and CicleDebug++ ==0){
        SERIAL_PRINTLN("Send magnetometer calibration values");
        SERIAL_PRINTLN("M[Mag Bias X Axis];[Mag Bias Y Axis];[Mag Bias Z Axis]");
      }
      #ifdef HeadingMagHold
        magBias[XAXIS]  = readFloatSerial();
        magBias[YAXIS]  = readFloatSerial();
        magBias[ZAXIS]  = readFloatSerial();
        writeEEPROM();
      #else
        if (debug) SERIAL_PRINTLN("OFF Calibrate magnetometer=");
        skipSerialValues(3);
      #endif
      break;

    case 'N': // battery monitor
      if (debug and CicleDebug++ ==0){
        SERIAL_PRINTLN("Send battery monitor values");
        SERIAL_PRINTLN("N[Cell Alarm Voltage];[Throttle Target];[Going Down Time]");
      }
      #ifdef BattMonitor
        batteryMonitorAlarmVoltage = readFloatSerial();
        batteryMonitorThrottleTarget = readFloatSerial();
        batteryMonitorGoingDownTime = readFloatSerial();
        setBatteryCellVoltageThreshold(batteryMonitorAlarmVoltage);
      #else
        if (debug) SERIAL_PRINTLN("OFF Battery monitor");
        skipSerialValues(3);
      #endif
      break;

    case 'O': // define waypoints
      if (debug and CicleDebug++ ==0){
        SERIAL_PRINTLN("Send waypoints");
        SERIAL_PRINTLN("O[Waypoint number];[Waypoint Latitude];");
        SERIAL_PRINTLN("[Waypoint Longitude];[Waypoint Altitude];");
      }
      #ifdef UseGPSNavigator
        missionNbPoint = readIntegerSerial();
        waypoint[missionNbPoint].latitude = readIntegerSerial();
        waypoint[missionNbPoint].longitude = readIntegerSerial();
        waypoint[missionNbPoint].altitude = readIntegerSerial();
      #else
        for(uint8_t i = 0; i < 4; i++) {
          readFloatSerial();
        }
      #endif
      break;
    case 'P': //  read Camera values
      if (debug and CicleDebug++ ==0){
        SERIAL_PRINTLN("Send camera stabilization values");
        SERIAL_PRINTLN("P[Camera Mode(0 = off, 1 = onboard stabilisation)];[Camera Center Pitch];");
        SERIAL_PRINTLN("[Camera Center Roll];[Camera Center Yaw];[Camera Pitch Servo Sensitivity/direction];");
        SERIAL_PRINTLN("[Camera Roll Servo Sensitivity/direction];[Camera Yaw Servo Sensitivity/direction];");
        SERIAL_PRINTLN("[Camera Pitch Servo minimum value];[Camera Roll Servo minimum value];");
        SERIAL_PRINTLN("[Camera Yaw Servo minimum value];[Camera Pitch Servo maximum value];");
        SERIAL_PRINTLN("[Camera Roll Servo maximum value];[Camera Yaw Servo maximum value];");
      } 
      #ifdef CameraControl
        cameraMode = readFloatSerial();
        servoCenterPitch = readFloatSerial();
        servoCenterRoll = readFloatSerial();
        servoCenterYaw = readFloatSerial();
        mCameraPitch = readFloatSerial();
        mCameraRoll = readFloatSerial();
        mCameraYaw = readFloatSerial();
        servoMinPitch = readFloatSerial();
        servoMinRoll = readFloatSerial();
        servoMinYaw = readFloatSerial();
        servoMaxPitch = readFloatSerial();
        servoMaxRoll = readFloatSerial();
        servoMaxYaw = readFloatSerial();
        #ifdef CameraTXControl
          servoTXChannels = readFloatSerial();
        #endif
      #else
        #ifdef CameraTXControl
          skipSerialValues(14)
        #else
          if (debug) SERIAL_PRINTLN("OFF Camera Control");
          skipSerialValues(13);
        #endif
      #endif
      break;
    case 'U': // Range Finder
      if (debug and CicleDebug++ ==0){
        SERIAL_PRINTLN("Send range finder values");
        SERIAL_PRINTLN("U[maxRangeFinderRange];[minRangeFinderRange];");
      }
      #if defined (AltitudeHoldRangeFinder)
        maxRangeFinderRange = readFloatSerial();
        minRangeFinderRange = readFloatSerial();
      #else
        if (debug) SERIAL_PRINTLN("OFF Range Finder");
        skipSerialValues(2);
      #endif
      break;

    case 'V': // GPS
      if (debug and CicleDebug++ ==0){
        SERIAL_PRINTLN("Send GPS PID values");
        SERIAL_PRINTLN("V[GPS Roll P];[GPS Roll I];[GPS Roll D];");
        SERIAL_PRINTLN("[GPS Pitch P];[GPS Pitch I];[GPS Pitch D];");
        SERIAL_PRINTLN("[GPS Yaw P];[GPS Yaw I];[GPS Yaw D];");
      }
      
      #if defined (UseGPSNavigator)
        readSerialPID(GPSROLL_PID_IDX);
        readSerialPID(GPSPITCH_PID_IDX);
        readSerialPID(GPSYAW_PID_IDX);
        writeEEPROM();
      #else
        if (debug) SERIAL_PRINTLN("OFF use GPS");
        skipSerialValues(9);
      #endif
      break;

    case 'W': // Write all user configurable values to EEPROM
      if (debug and CicleDebug++ ==0) SERIAL_PRINTLN("Write user defined values to EEPROM");
      writeEEPROM(); // defined in DataStorage.h
      zeroIntegralError();
      break;

    case 'X': // Stop sending messages
      if (debug){
        SERIAL_PRINTLN("Stop sending messages");
        CicleDebug =0;
      }
      break;
      
    case 'Y': // Test buzzer
      #if defined BUZZER
        if (debug and CicleDebug++ ==0) SERIAL_PRINTLN("Test buzzer");
        buzzer(100 ,3);                           //ms, cicles
      #endif
     break;

    case '1': // Calibrate ESCS's by setting Throttle high on all channels
      if (debug and CicleDebug++ ==0) SERIAL_PRINTLN("Calibrate ESCSs by setting Throttle high on all channels");
      validateCalibrateCommand(1);
      break;

    case '2': // Calibrate ESC's by setting Throttle low on all channels
      if (debug and CicleDebug++ ==0) SERIAL_PRINTLN("Calibrate ESC's by setting Throttle low on all channels");
      validateCalibrateCommand(2);
      break;

    case '3': // Test ESC calibration
      if (debug and CicleDebug++ ==0) SERIAL_PRINTLN("Test ESC calibration");
      if (validateCalibrateCommand(3)) {
        testCommand = readFloatSerial();
      }
      break;

    case '4': // Turn off ESC calibration
      if (debug and CicleDebug++ ==0) SERIAL_PRINTLN("Turn off ESC calibration");
      if (validateCalibrateCommand(4)) {
        calibrateESC = 0;
        testCommand = 1000;
      }
      break;

    case '5': // Send individual motor commands (motor, command)
      if (debug and CicleDebug++ ==0) SERIAL_PRINTLN("Send individual motor commands");
      if (validateCalibrateCommand(5)) {
        for (uint8_t motor = 0; motor < LASTMOTOR; motor++) {
          motorConfiguratorCommand[motor] = (int)readFloatSerial();
        }
      }
      break;

    case 'Z': // fast telemetry transfer <--- get rid if this?
      if (readFloatSerial() == 1.0){
        fastTransfer = ON;
        if (debug) SERIAL_PRINTLN("Fast telemetry transfer ON");
      }
      else{
        fastTransfer = OFF;
        if (debug) SERIAL_PRINTLN("Fast telemetry transfer OFF");
      }
      break;
    }
  }
}

//***************************************************************************************************
//********************************* Serial Telemetry ************************************************
//***************************************************************************************************

void PrintValueComma(float val) {
  SERIAL_PRINT(val);
  comma();
}

void PrintValueComma(double val) {
  SERIAL_PRINT(val);
  comma();
}

void PrintValueComma(char val) {
  SERIAL_PRINT(val);
  comma();
}

void PrintValueComma(int val) {
  SERIAL_PRINT(val);
  comma();
}

void PrintValueComma(unsigned long val)
{
  SERIAL_PRINT(val);
  comma();
}

void PrintValueComma(uint8_t val)
{
  SERIAL_PRINT(val);
  comma();
}

void PrintValueComma(long int val)
{
  SERIAL_PRINT(val);
  comma();
}

void PrintPID(unsigned char IDPid)
{
  PrintValueComma(PID[IDPid].P);
  PrintValueComma(PID[IDPid].I);
  PrintValueComma(PID[IDPid].D);
}

void PrintDummyValues(uint8_t number) {
  for(uint8_t i=0; i<number; i++) {
    PrintValueComma(0);
  }
}


float getHeading()
{
  #if defined(HeadingMagHold) || defined(AeroQuadMega_CHR6DM) || defined(APM_OP_CHR6DM)
    float heading = trueNorthHeading;
    if (heading < 0) { 
      heading += (2.0 * M_PI);
    }
    return heading;
  #else
    return(gyroHeading);
  #endif
}

void sendSerialTelemetry() {
  switch (queryType) {
  case '=': // Reserved debug command to view any variable from Serial Monitor
    if (debug){
      debug = false;
      SERIAL_PRINTLN("Debug OFF");
      CicleDebug =0;
    }
    else{
      SERIAL_PRINTLN("Debug ON");
      debug = true;
    }
    queryType = 'X';
    break;

  case 'a': // Send roll and pitch rate mode PID values
    if (debug) SERIAL_PRINTLN("Read roll and pitch rate (acro) mode PID values");
    PrintPID(RATE_XAXIS_PID_IDX);
    PrintPID(RATE_YAXIS_PID_IDX);
    PrintValueComma(rotationSpeedFactor);
    SERIAL_PRINTLN();
    queryType = 'X';
    break;

  case 'b': // Send roll and pitch attitude mode PID values
    if (debug) SERIAL_PRINTLN("Read roll and pitch attitude (stable) mode PID values");
    PrintPID(ATTITUDE_XAXIS_PID_IDX);
    PrintPID(ATTITUDE_YAXIS_PID_IDX);
    PrintPID(ATTITUDE_GYRO_XAXIS_PID_IDX);
    PrintPID(ATTITUDE_GYRO_YAXIS_PID_IDX);
    SERIAL_PRINTLN(windupGuard);
    queryType = 'X';
    break;

  case 'c': // Send yaw PID values
    if (debug) SERIAL_PRINTLN("Read yaw PID values");
    PrintPID(ZAXIS_PID_IDX);
    PrintPID(HEADING_HOLD_PID_IDX);
    SERIAL_PRINTLN((int)headingHoldConfig);
    queryType = 'X';
    break;

  case 'd': // Altitude Hold
    if (debug) SERIAL_PRINTLN("Read altitude Hold");
    #if defined AltitudeHoldBaro || defined AltitudeHoldRangeFinder
      PrintPID(BARO_ALTITUDE_HOLD_PID_IDX);
      PrintValueComma(PID[BARO_ALTITUDE_HOLD_PID_IDX].windupGuard);
      PrintValueComma(altitudeHoldBump);
      PrintValueComma(altitudeHoldPanicStickMovement);
      PrintValueComma(minThrottleAdjust);
      PrintValueComma(maxThrottleAdjust);
      #if defined AltitudeHoldBaro
        PrintValueComma(baroSmoothFactor);
      #else
        PrintValueComma(0);
      #endif
      PrintPID(ZDAMPENING_PID_IDX);
    #else
      PrintDummyValues(10);
    #endif
    SERIAL_PRINTLN();
    queryType = 'X';
    break;

  case 'e': // miscellaneous config values
    if (debug) SERIAL_PRINTLN("Read Miscellaneous config values");
    PrintValueComma(aref);
    SERIAL_PRINTLN(minArmedThrottle);
    queryType = 'X';
    break;

  case 'f': // Send transmitter smoothing values
    if (debug and CicleDebug++ ==0) SERIAL_PRINTLN("Read transmitter smoothing values");
    PrintValueComma(receiverXmitFactor);
    for (uint8_t axis = XAXIS; axis < LASTCHANNEL; axis++) {
      PrintValueComma(receiverSmoothFactor[axis]);
    }
    PrintDummyValues(10 - LASTCHANNEL);
    SERIAL_PRINTLN();
    queryType = 'X';
    break;

  case 'g': // Send transmitter calibration values
    if (debug) SERIAL_PRINTLN("Read transmitter calibration values");
    for (uint8_t axis = XAXIS; axis < LASTCHANNEL; axis++) {
      Serial.print(receiverSlope[axis], 6);
      Serial.print(',');
    }
    SERIAL_PRINTLN();
    queryType = 'X';
    break;

  case 'h': // Send transmitter offset values
    if (debug) SERIAL_PRINTLN("Read transmitter offset values");
    for (uint8_t axis = XAXIS; axis < LASTCHANNEL; axis++) {
      Serial.print(receiverOffset[axis], 6);
      Serial.print(',');
    }
    SERIAL_PRINTLN();
    queryType = 'X';
    break;

  case 'i': // Send sensor data
    if (debug and CicleDebug++ ==0){ 
      SERIAL_PRINTLN("Read sensor data");
    }
    if (debug) SERIAL_PRINT("gyro X, Y, Z: ");
    for (uint8_t axis = XAXIS; axis <= ZAXIS; axis++) {  
      PrintValueComma(gyroRate[axis]);
    }
    if (debug) SERIAL_PRINT(" Accel X, Y, Z: ");
    for (uint8_t axis = XAXIS; axis <= ZAXIS; axis++) {
      PrintValueComma(filteredAccel[axis]);
    }
    if (debug) SERIAL_PRINT(" Compass X, Y, Z: ");
    for (uint8_t axis = XAXIS; axis <= ZAXIS; axis++) {
      #if defined(HeadingMagHold)
        PrintValueComma(getMagnetometerData(axis));
      #else
        PrintValueComma(0);
      #endif
    }
    SERIAL_PRINTLN();
    break;

  case 'j': // Send raw mag values
    if (debug and CicleDebug++ ==0) SERIAL_PRINTLN("Read raw mag values");
    #ifdef HeadingMagHold
      PrintValueComma(getMagnetometerRawData(XAXIS));
      PrintValueComma(getMagnetometerRawData(YAXIS));
      SERIAL_PRINTLN(getMagnetometerRawData(ZAXIS));
    #endif
    break;

  case 'k': // Send accelerometer cal values
    if (debug) SERIAL_PRINTLN("Read accelerometer cal values");
    SERIAL_PRINT(accelScaleFactor[XAXIS], 6);
    comma();
    SERIAL_PRINT(runTimeAccelBias[XAXIS], 6);
    comma();
    SERIAL_PRINT(accelScaleFactor[YAXIS], 6);
    comma();
    SERIAL_PRINT(runTimeAccelBias[YAXIS], 6);
    comma();
    SERIAL_PRINT(accelScaleFactor[ZAXIS], 6);
    comma();
    SERIAL_PRINTLN(runTimeAccelBias[ZAXIS], 6);
    queryType = 'X';
    break;

  case 'l': // Send raw accel values
    if (debug and CicleDebug++ ==0) SERIAL_PRINTLN("Read raw accel values");
    measureAccelSum();
    PrintValueComma((int)(accelSample[XAXIS]/accelSampleCount));
    accelSample[XAXIS] = 0;
    PrintValueComma((int)(accelSample[YAXIS]/accelSampleCount));
    accelSample[YAXIS] = 0;
    SERIAL_PRINTLN ((int)(accelSample[ZAXIS]/accelSampleCount));
    accelSample[ZAXIS] = 0;
    accelSampleCount = 0;
    break;

  case 'm': // Send magnetometer cal values
    if (debug) SERIAL_PRINTLN("Read magnetometer cal values");
    #ifdef HeadingMagHold
      SERIAL_PRINT(magBias[XAXIS], 6);
      comma();
      SERIAL_PRINT(magBias[YAXIS], 6);
      comma();
      SERIAL_PRINTLN(magBias[ZAXIS], 6);
    #endif
    queryType = 'X';
    break;

  case 'n': // battery monitor
    if (debug) SERIAL_PRINTLN("Read battery monitor values");
    #ifdef BattMonitor
      if (debug) SERIAL_PRINT("Alarm Voltage: ");
      PrintValueComma(batteryMonitorAlarmVoltage);
      if (debug) SERIAL_PRINT(" Throttle Target: ");
      PrintValueComma(batteryMonitorThrottleTarget);
      if (debug) SERIAL_PRINT(" Going DownTime: ");
      PrintValueComma(batteryMonitorGoingDownTime);
      if (debug){
        SERIAL_PRINTLN();
        SERIAL_PRINT("voltage: ");
        SERIAL_PRINT((float)batteryData[0].voltage/100.0); // voltage internally stored at 10mV:s
        #if ! defined (__AVR_ATmega328P__) && ! defined(__AVR_ATmegaUNO__) && ! defined(ARDUINO_SAM_DUE)
          SERIAL_PRINT(" current: ");
          SERIAL_PRINTLN((float)batteryData[0].current/100.0);
        #endif
      }
    #else
      if (debug) SERIAL_PRINTLN("OFF battery monitor");
      PrintDummyValues(3);
    #endif
    SERIAL_PRINTLN();
    queryType = 'X';
    break;

  case 'o': // send waypoints
    if (debug) SERIAL_PRINTLN("Read waypoints");
    #ifdef UseGPSNavigator
      for (uint8_t index = 0; index < MAX_WAYPOINTS; index++) {
        PrintValueComma(index);
        PrintValueComma(waypoint[index].latitude);
        PrintValueComma(waypoint[index].longitude);
        PrintValueComma(waypoint[index].altitude);
      }
    #else
      PrintDummyValues(4);
    #endif
    SERIAL_PRINTLN();
    queryType = 'X';
    break;

  case 'p': // Send Camera values
    if (debug) SERIAL_PRINTLN("Read Camera values");
    #ifdef CameraControl
      PrintValueComma(cameraMode);
      PrintValueComma(servoCenterPitch);
      PrintValueComma(servoCenterRoll);
      PrintValueComma(servoCenterYaw);
      PrintValueComma(mCameraPitch);
      PrintValueComma(mCameraRoll);
      PrintValueComma(mCameraYaw);
      PrintValueComma(servoMinPitch);
      PrintValueComma(servoMinRoll);
      PrintValueComma(servoMinYaw);
      PrintValueComma(servoMaxPitch);
      PrintValueComma(servoMaxRoll);
      PrintValueComma(servoMaxYaw);
      #ifdef CameraTXControl
        PrintValueComma(servoTXChannels);
      #endif
    #else
      #ifdef CameraTXControl
        PrintDummyValues(14);
      #else
        PrintDummyValues(13);
        SERIAL_PRINTLN();
        if (debug) SERIAL_PRINTLN("OFF CameraControl");
      #endif
    #endif
    SERIAL_PRINTLN();
    queryType = 'X';
    break;

  case 'q': // Send Vehicle State Value
    if (debug){ 
       SERIAL_PRINTLN("Read Vehicle State Value");
       SERIAL_PRINTLN(vehicleState, BIN);
    }else{
       SERIAL_PRINTLN(vehicleState);
    }
    queryType = 'X';
    break;

  case 'r': // Vehicle attitude
    if (debug and CicleDebug++ ==0) SERIAL_PRINTLN("Read Vehicle attitude");
    PrintValueComma(kinematicsAngle[XAXIS]);
    PrintValueComma(kinematicsAngle[YAXIS]);
    SERIAL_PRINTLN(getHeading());
    break;

  case 's': // Send all flight data
    if (debug and CicleDebug++ ==0) SERIAL_PRINTLN("Read all flight data");
    if (debug) SERIAL_PRINT("Status motor:");
    PrintValueComma(motorArmed);
    if (debug) SERIAL_PRINT(" XAXIS:"); // alabeo-roll
    PrintValueComma(kinematicsAngle[XAXIS]);
    if (debug) SERIAL_PRINT(" YAXIS:");  //cabeceo pitch
    PrintValueComma(kinematicsAngle[YAXIS]);
    if (debug) SERIAL_PRINT(" Heading:"); // rotacion grados
    PrintValueComma(getHeading());
    #if defined AltitudeHoldBaro || defined AltitudeHoldRangeFinder
      #if defined AltitudeHoldBaro
        if (debug) SERIAL_PRINT(" BaroAltitude:");
        PrintValueComma(getBaroAltitude());
      #elif defined AltitudeHoldRangeFinder
        if (debug) SERIAL_PRINT(" Altitude:");
        PrintValueComma(rangeFinderRange[ALTITUDE_RANGE_FINDER_INDEX] != INVALID_RANGE ? rangeFinderRange[ALTITUDE_RANGE_FINDER_INDEX] : 0.0);
      #endif
        if (debug) SERIAL_PRINT(" AltitudeHoldState:");
      PrintValueComma((int)altitudeHoldState);
    #else
      PrintValueComma(0);
      PrintValueComma(0);
    #endif

    for (uint8_t channel = 0; channel < 8; channel++) { // Configurator expects 8 values
      PrintValueComma((channel < LASTCHANNEL) ? receiverCommand[channel] : 0);
    }

    for (uint8_t motor = 0; motor < LASTMOTOR; motor++) {
      PrintValueComma(motorCommand[motor]);
    }
    PrintDummyValues(8 - LASTMOTOR); // max of 8 motor outputs supported

    #ifdef BattMonitor
      PrintValueComma((float)batteryData[0].voltage/100.0); // voltage internally stored at 10mV:s
    #else
      PrintValueComma(0);
    #endif
    PrintValueComma(flightMode);
    SERIAL_PRINTLN();
    break;

  case 't': // Send processed transmitter values
    if (debug and CicleDebug++ ==0) SERIAL_PRINTLN("Read processed transmitter values");
    for (uint8_t axis = 0; axis < LASTCHANNEL; axis++) {
      PrintValueComma(receiverCommand[axis]);
    }
    SERIAL_PRINTLN();
    break;

  case 'u': // Send range finder values
    if (debug) SERIAL_PRINTLN("Read range finder values");
    #if defined (AltitudeHoldRangeFinder)
      PrintValueComma(maxRangeFinderRange);
      SERIAL_PRINTLN(minRangeFinderRange);
    #else
      PrintValueComma(0);
      SERIAL_PRINTLN(0);
    #endif
    queryType = 'X';
    break;

  case 'v': // Send GPS PIDs
    if (debug) SERIAL_PRINTLN("Read GPS PIDs");
    #if defined (UseGPSNavigator)
      PrintPID(GPSROLL_PID_IDX);
      PrintPID(GPSPITCH_PID_IDX);
      PrintPID(GPSYAW_PID_IDX);
      queryType = 'X';
    #else
      if (debug) SERIAL_PRINTLN("OFF Use GPS Navigator");
      PrintDummyValues(9);
    #endif
    SERIAL_PRINTLN();
    queryType = 'X';
    break;
  case 'w':   
    #if defined DuePi
      Ini_EEPROM =0;
      if (debug) SERIAL_PRINTLN("Read EEPROM Hat info:");
      for (int x = 42; x < 62; x++) {                                 // We read from position 42 to 62 (EEPROM HAT->0-200)
        trama.concat((char) readEEPROMI2C(x));  
      } 
      SERIAL_PRINTLN(trama);
      trama = "";
      Ini_EEPROM =200;
      if (debug) SERIAL_PRINTLN("Read EEPROM:");
      for (int x = 0; x < 592; x++) {                                // We read from position 0->200 to 592 (data->200-4000)
        char dato =readEEPROMI2C(x);
        if (dato <= 0x0F)
          SERIAL_PRINT ("0x0");
        else
          SERIAL_PRINT ("0x");
        SERIAL_PRINT (dato, HEX);
        SERIAL_PRINT (" ");
        if (x >14 && (x+1)%16 ==0) SERIAL_PRINTLN();
      } 
    #else
      SERIAL_PRINTLN("Error: Invalid board!!");
    #endif
    queryType = 'X';
    break;
  case 'y': // send GPS info
    if (debug and CicleDebug++ ==0) SERIAL_PRINTLN("Read GPS info");
    #if defined (UseGPS)
      PrintValueComma(gpsData.state);
      PrintValueComma(gpsData.lat);
      PrintValueComma(gpsData.lon);
      PrintValueComma(gpsData.height);
      PrintValueComma(gpsData.course);
      PrintValueComma(gpsData.speed);
      PrintValueComma(gpsData.accuracy);
      PrintValueComma(gpsData.sats);
      PrintValueComma(gpsData.fixtime);
      PrintValueComma(gpsData.sentences);
      PrintValueComma(gpsData.idlecount);
      if (debug){
        SERIAL_PRINT("@");
        SERIAL_PRINTLN(gpsBaudRates[gpsData.baudrate]);
      }
    #else
      if (debug) SERIAL_PRINTLN("OFF GPS");
      PrintDummyValues(11);
    #endif    
    SERIAL_PRINTLN();
    break;
 
  case 'z': // Send all Altitude data 
    if (debug and CicleDebug++ ==0) SERIAL_PRINTLN("Read all Altitude data");
    #if defined (AltitudeHoldBaro) 
      if (debug){
        SERIAL_PRINT("Ground Altitude ");
        PrintValueComma(baroGroundAltitude);
        SERIAL_PRINT("Altitude ");
        PrintValueComma(baroAltitude);
        SERIAL_PRINTLN();
        if (baroGroundUpdateDone)SERIAL_PRINT("Altitude flight ");
      }
      PrintValueComma(getBaroAltitude()); //altura actual->baroAltitude - altura tierra->baroGroundAltitude
    #else
      PrintValueComma(0);
    #endif 
    #if defined (AltitudeHoldRangeFinder) 
      if (debug){
        SERIAL_PRINT("Sensor RangeFinder ");
      }
      SERIAL_PRINTLN(rangeFinderRange[ALTITUDE_RANGE_FINDER_INDEX]);
    #else
      SERIAL_PRINTLN(0); 
    #endif 
    break;
    
  case '$': // send BatteryMonitor voltage/current readings
    if (debug and CicleDebug++ ==0) SERIAL_PRINTLN("Read BatteryMonitor voltage/current readings");
    #if defined (BattMonitor)
      if (debug) SERIAL_PRINT("voltage: ");
      PrintValueComma((float)batteryData[0].voltage/100.0); // voltage internally stored at 10mV:s
      #if defined (BM_EXTENDED)
        if (debug) SERIAL_PRINT(" current (A): ");
        PrintValueComma((float)batteryData[0].current/100.0);
        if (debug) SERIAL_PRINT(" Capacity: ");
        PrintValueComma((float)batteryData[0].usedCapacity/1000.0);
      #else
        PrintDummyValues(2);
      #endif
    #else
      PrintDummyValues(3);
    #endif
    SERIAL_PRINTLN();
    break;
    
  case '%': // send RSSI
    if (debug and CicleDebug++ ==0) SERIAL_PRINTLN("Read Received Signal Strength Indication)");
    #if defined (UseAnalogRSSIReader) || defined (UseEzUHFRSSIReader) || defined (UseSBUSRSSIReader)
      SERIAL_PRINTLN(rssiRawValue);
    #else
      if (debug) SERIAL_PRINTLN("OFF RSSI");
      SERIAL_PRINTLN(0);
    #endif
    break;

  case 'x': // Stop sending messages
    if (debug){
      SERIAL_PRINTLN("Stop sending messages");
      CicleDebug =0;
      queryType = 'X';
    }
    break;

  case '!': // Send flight software version
    if (debug and CicleDebug++ ==0) SERIAL_PRINTLN("Read flight software version");
    SERIAL_PRINTLN(SOFTWARE_VERSION, 1);
    queryType = 'X';
    break;

  case '#': // Send configuration
    if (debug) SERIAL_PRINTLN("Read configuration");
    reportVehicleState();
    queryType = 'X';
    break;

  case '6': // Report remote commands
    if (debug) SERIAL_PRINTLN("Report remote commands");
    for (uint8_t motor = 0; motor < LASTMOTOR; motor++) {
      PrintValueComma(motorCommand[motor]);
    }
    SERIAL_PRINTLN();
    queryType = 'X';
    break;
    
#if defined DuePi
  case '0': // data RAW sensor MPU9250 
  if (debug) { 
    SERIAL_PRINTLN("Data RAW sensor MPU9250");
  }
  if (debug) SERIAL_PRINT("Accelerometer X, Y, Z: ");
  PrintValueComma((long int) mpu.accel_data_raw[0]);
  PrintValueComma((long int) mpu.accel_data_raw[1]);
  PrintValueComma((long int) mpu.accel_data_raw[2]);
  if (debug){
    SERIAL_PRINTLN();
    SERIAL_PRINT("Gyroscope X, Y, Z: ");
  }
  PrintValueComma((long int) mpu.gyro_data_raw[0]);
  PrintValueComma((long int) mpu.gyro_data_raw[1]);
  PrintValueComma((long int) mpu.gyro_data_raw[2]);
  #if defined (Magnetometer_MPU9250)
    if (debug){
      SERIAL_PRINTLN();
      SERIAL_PRINT("Compas X, Y, Z: ");
    }
    PrintValueComma((long int) mpu.mag_data_raw[0]);
    PrintValueComma((long int) mpu.mag_data_raw[1]);
    PrintValueComma((long int) mpu.mag_data_raw[2]);
    SERIAL_PRINTLN();
  #endif
  if (debug){
    SERIAL_PRINTLN();
    SERIAL_PRINT("Temperature: ");
  }
  mpu.read_temp();
  PrintValueComma((float)mpu.temperature);
  SERIAL_PRINTLN();
  break;

  case '8': // data SRF08
  if (debug) SERIAL_PRINTLN("Read SRF08 Mts");
  rangerWaitCycles =3;
  while (rangerWaitCycles !=6){
    updateRangeFinders();
    delay(20);    
  }
  SERIAL_PRINT (rangeFinderRange[0]);
  if (debug) SERIAL_PRINTLN(" Mts");
  rangerWaitCycles =3;
  queryType = 'X';
  break;
#endif

#if defined(OSD) && defined(OSD_LOADFONT)
  case '&': // fontload
    if (debug) SERIAL_PRINTLN("max7456LoadFont");
    if (OFF == motorArmed) {
      max7456LoadFont();
    }
    queryType = 'X';
    break;
#endif

  }
}

void readValueSerial(char *data, uint8_t size) {
  uint8_t index = 0;
  uint8_t timeout = 0;
  data[0] = '\0';

  do {
    if (SERIAL_AVAILABLE() == 0) {
      delayMicroseconds (1000);
      timeout++;
    } else {
      data[index] = SERIAL_READ();
      timeout = 0;
      index++;
    }
  } while ((index == 0 || data[index-1] != ';') && (timeout < 10) && (index < size-1));

  data[index] = '\0';
}


// Used to read floating point values from the serial port
float readFloatSerial() {
  char data[15] = "";

  readValueSerial(data, sizeof(data));
  return atof(data);
}

// Used to read integer values from the serial port
long readIntegerSerial() {
  char data[16] = "";

  readValueSerial(data, sizeof(data));
  
  return atol(data);
}

void comma() {
  SERIAL_PRINT(',');
}


#ifdef BinaryWrite
void printInt(int data) {
  uint8_t msb, lsb;

  msb = data >> 8;
  lsb = data & 0xff;

  binaryPort->write(msb);
  binaryPort->write(lsb);
}

void sendBinaryFloat(float data) {
  union binaryFloatType {
    uint8_t floatByte[4];
    float floatVal;
  } binaryFloat;

  binaryFloat.floatVal = data;
  binaryPort->write(binaryFloat.floatByte[3]);
  binaryPort->write(binaryFloat.floatByte[2]);
  binaryPort->write(binaryFloat.floatByte[1]);
  binaryPort->write(binaryFloat.floatByte[0]);
}

void sendBinaryuslong(unsigned long data) {
  union binaryuslongType {
    uint8_t uslongByte[4];
    unsigned long uslongVal;
  } binaryuslong;

  binaryuslong.uslongVal = data;
  binaryPort->write(binaryuslong.uslongByte[3]);
  binaryPort->write(binaryuslong.uslongByte[2]);
  binaryPort->write(binaryuslong.uslongByte[1]);
  binaryPort->write(binaryuslong.uslongByte[0]);
}

void fastTelemetry()
{
  // **************************************************************
  // ***************** Fast Transfer Of Sensor Data ***************
  // **************************************************************
  // AeroQuad.h defines the output rate to be 10ms
  // Since writing to UART is done by hardware, unable to measure data rate directly
  // Through analysis:  115200 baud = 115200 bits/second = 14400 bytes/second
  // If float = 4 bytes, then 3600 floats/second
  // If 10 ms output rate, then 36 floats/10ms
  // Number of floats written using sendBinaryFloat is 15

  if (motorArmed == ON) {
    #ifdef OpenlogBinaryWrite
       printInt(21845); // Start word of 0x5555
       sendBinaryuslong(currentTime);
        printInt((int)flightMode);
       for (uint8_t axis = XAXIS; axis <= ZAXIS; axis++) {
         sendBinaryFloat(gyroRate[axis]);
       }
       for (uint8_t axis = XAXIS; axis <= ZAXIS; axis++) {
         sendBinaryFloat(meterPerSecSec[axis]);
       }
       sendBinaryFloat(accelOneG);
       #ifdef HeadingMagHold
          sendBinaryFloat(hdgX);
          sendBinaryFloat(hdgY);
		  for (uint8_t axis = XAXIS; axis <= ZAXIS; axis++) {
		       #if defined(HeadingMagHold)
			      sendBinaryFloat(getMagnetometerData(axis));
		       #endif
          }
       #else
         sendBinaryFloat(0.0);
         sendBinaryFloat(0.0);
         sendBinaryFloat(0.0);
       #endif
        for (uint8_t axis = XAXIS; axis <= ZAXIS; axis++) {
          sendBinaryFloat(kinematicsAngle[axis]);
        }
        printInt(32767); // Stop word of 0x7FFF
    #else
       printInt(21845); // Start word of 0x5555
       for (uint8_t axis = XAXIS; axis <= ZAXIS; axis++) {
         sendBinaryFloat(gyroRate[axis]);
       }
       for (uint8_t axis = XAXIS; axis <= ZAXIS; axis++) {
         sendBinaryFloat(meterPerSecSec[axis]);
       }
       for (uint8_t axis = XAXIS; axis <= ZAXIS; axis++)
       #if defined(HeadingMagHold)
         sendBinaryFloat(getMagnetometerData(axis));
       #else
         sendBinaryFloat(0);
       #endif
       for (uint8_t axis = XAXIS; axis <= ZAXIS; axis++) {
         sendBinaryFloat(getGyroUnbias(axis));
       }
       for (uint8_t axis = XAXIS; axis <= ZAXIS; axis++) {
         sendBinaryFloat(kinematicsAngle[axis]);
       }
       printInt(32767); // Stop word of 0x7FFF
    #endif
  }
}
#endif // BinaryWrite

void printVehicleState(const char *sensorName, unsigned long state, const char *message) {
  
  SERIAL_PRINT(sensorName);
  SERIAL_PRINT(": ");
  if (!(vehicleState & state)) {
    SERIAL_PRINT("Not ");
  }
  SERIAL_PRINTLN(message);
}

void reportVehicleState() {
  // Tell Configurator how many vehicle state values to expect
  SERIAL_PRINTLN(15);
  SERIAL_PRINT("Software Version: ");
  SERIAL_PRINTLN(SOFTWARE_VERSION, 1);
  SERIAL_PRINT("Board Type: ");
  #if defined(AeroQuad_v1)
    SERIAL_PRINTLN("v1.x");
  #elif defined(AeroQuad_v1_IDG)
    SERIAL_PRINTLN("v1.x IDG");
  #elif defined(AeroQuadMega_v1)
    SERIAL_PRINTLN("Mega v1.x");
  #elif defined(AeroQuad_v18)
    SERIAL_PRINTLN("v1.8 and greater");
  #elif defined(AeroQuadMega_v2)
    SERIAL_PRINTLN("Mega v2");
  #elif defined(AeroQuadMega_v21)
    SERIAL_PRINTLN("Mega v21");
  #elif defined(AeroQuadMega_v21)
    SERIAL_PRINTLN("AutoNav");
  #elif defined(AutonavShield)
    SERIAL_PRINTLN("AutonavShield");
  #elif defined(AeroQuad_Wii)
    SERIAL_PRINTLN("Wii");
  #elif defined(AeroQuadMega_Wii)
    SERIAL_PRINTLN("Mega Wii");
  #elif defined(ArduCopter)
    SERIAL_PRINTLN("ArduCopter");
  #elif defined(AeroQuadMega_CHR6DM)
    SERIAL_PRINTLN("CHR6DM");
  #elif defined(APM_OP_CHR6DM)
    SERIAL_PRINTLN("APM w/ CHR6DM");
  #elif defined(AeroQuad_Mini)
    SERIAL_PRINTLN("Mini");
  #elif defined(DuePi)
    SERIAL_PRINTLN("DuePi");
  #elif defined(AeroQuadSTM32)
    SERIAL_PRINTLN(STM32_BOARD_TYPE);
  #endif

  SERIAL_PRINT("Flight Config: ");
  #if defined(quadPlusConfig)
    SERIAL_PRINTLN("Quad +");
  #elif defined(quadXConfig)
    SERIAL_PRINTLN("Quad X");
  #elif defined (quadY4Config)
    SERIAL_PRINTLN("Quad Y4");
  #elif defined (triConfig)
    SERIAL_PRINTLN("Tri");
  #elif defined(hexPlusConfig)
    SERIAL_PRINTLN("Hex +");
  #elif defined(hexXConfig)
    SERIAL_PRINTLN("Hex X");
  #elif defined(hexY6Config)
    SERIAL_PRINTLN("Hex Y6");
  #elif defined(octoX8Config)
    SERIAL_PRINTLN("Octo X8");
  #elif defined(octoXConfig)
    SERIAL_PRINTLN("Octo X");
  #elif defined(octoPlusConfig)
    SERIAL_PRINTLN("Octo +");
  #endif

  SERIAL_PRINT("Receiver Channels: ");
  SERIAL_PRINTLN(LASTCHANNEL);

  SERIAL_PRINT("Motors: ");
  SERIAL_PRINTLN(LASTMOTOR);

  printVehicleState("Gyroscope", GYRO_DETECTED, "Detected");
  printVehicleState("Accelerometer", ACCEL_DETECTED, "Detected");
  printVehicleState("Barometer", BARO_DETECTED, "Detected");
  printVehicleState("Magnetometer", MAG_DETECTED, "Detected");
  printVehicleState("Heading Hold", HEADINGHOLD_ENABLED, "Enabled");
  printVehicleState("Altitude Hold", ALTITUDEHOLD_ENABLED, "Enabled");
  printVehicleState("Battery Monitor", BATTMONITOR_ENABLED, "Enabled");
  printVehicleState("Camera Stability", CAMERASTABLE_ENABLED, "Enabled");
  printVehicleState("Range Detection", RANGE_ENABLED, "Enabled");
#ifdef UseGPS
  SERIAL_PRINT("GPS: ");
  SERIAL_PRINT((gpsData.state==GPS_DETECTING)?"Scanning ":"Detected ");
  if (gpsData.state != GPS_DETECTING) {
    SERIAL_PRINT(gpsTypes[gpsData.type].name);
  }
  SERIAL_PRINT("@");
  SERIAL_PRINTLN(gpsBaudRates[gpsData.baudrate]);
#else
  SERIAL_PRINTLN("GPS: Not Enabled");
#endif
}

#ifdef SlowTelemetry
  struct __attribute__((packed)) telemetryPacket {
    unsigned short  id;
    long  latitude;
    long  longitude;
    short altitude;
    short course;
    short heading;
    uint8_t  speed;
    uint8_t  rssi;
    uint8_t  voltage;
    uint8_t  current;
    unsigned short capacity;
    unsigned short gpsinfo;
    uint8_t  ecc[8];
  };

  union telemetryBuffer {
    struct telemetryPacket data;
    uint8_t   bytes[32];
  } telemetryBuffer;

  #define TELEMETRY_MSGSIZE 24
  #define TELEMETRY_MSGSIZE_ECC (TELEMETRY_MSGSIZE + 8)

  uint8_t slowTelemetryByte = 255;

  void initSlowTelemetry() {
#ifdef SoftModem
    softmodemInit();
#else
    Serial2.begin(1200);
#endif
    slowTelemetryByte = 255;
  }

  /* 100Hz task, sends data out byte by byte */
  void updateSlowTelemetry100Hz() {

    if (slowTelemetryByte < TELEMETRY_MSGSIZE_ECC ) {
      #ifdef SoftModem
        if (softmodemFreeToSend()) {
	  softmodemSendByte(telemetryBuffer.bytes[slowTelemetryByte]);
	  slowTelemetryByte++;
        }
      #else
        Serial2.write(telemetryBuffer.bytes[slowTelemetryByte]);
        slowTelemetryByte++;
      #endif
    }
    else {
      slowTelemetryByte=255;
    }
  }

  void updateSlowTelemetry10Hz() {

    if (slowTelemetryByte==255) {
      telemetryBuffer.data.id        = 0x5141; // "AQ"
      #ifdef UseGPS
        telemetryBuffer.data.latitude  = currentPosition.latitude;  // degrees/10000000
        telemetryBuffer.data.longitude = currentPosition.longitude; // degrees/10000000
        telemetryBuffer.data.course    = getCourse()/10; // degrees
        telemetryBuffer.data.speed     = getGpsSpeed()*36/1000;              // km/h
        telemetryBuffer.data.heading   = (short)(trueNorthHeading*RAD2DEG); // degrees
        telemetryBuffer.data.gpsinfo   = 0;
        telemetryBuffer.data.gpsinfo  |= (((unsigned short)((gpsData.sats<15)?gpsData.sats:15)) << 12);
      #else
        telemetryBuffer.data.latitude  = 0;
        telemetryBuffer.data.longitude = 0;
        telemetryBuffer.data.course    = 0;
        telemetryBuffer.data.speed     = 0;
        telemetryBuffer.data.heading   = 0;
        telemetryBuffer.data.gpsinfo   = 0;
      #endif

      #ifdef AltitudeHoldBaro
        telemetryBuffer.data.altitude  = (short)(getBaroAltitude()*10.0); // 0.1m->10cm
      #else
        telemetryBuffer.data.altitude  = 0;
      #endif

      #ifdef UseRSSIFaileSafe
        #ifdef RSSI_RAWVAL
          telemetryBuffer.data.rssi      = rssiRawValue/10; // scale to 0-100
        #else
          telemetryBuffer.data.rssi      = rssiRawValue;
        #endif
      #else
        telemetryBuffer.data.rssi      = 100;
      #endif

      #ifdef BattMonitor
        telemetryBuffer.data.voltage   = batteryData[0].voltage/10;  // to 0.1V
        telemetryBuffer.data.current   = batteryData[0].current/100; // to A
        telemetryBuffer.data.capacity  = batteryData[0].usedCapacity/1000; // mAh
      #else
        telemetryBuffer.data.voltage   = 0;
        telemetryBuffer.data.current   = 0;
        telemetryBuffer.data.capacity  = 0;
      #endif

       /* add ECC */
      encode_data(telemetryBuffer.bytes,24);

      /* trigger send */
      slowTelemetryByte=0;
    }
  }
#endif // SlowTelemetry

#endif // _AQ_SERIAL_COMM_
