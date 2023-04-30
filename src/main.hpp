// MIT License
//
// Copyright (c) 2020 Christian Riggenbach
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#pragma once

#include <WiFi.h>
#include <WiFiMulti.h>

#include <HTTPClient.h>

#include <AsyncUDP.h>

#include <ESPUI.h>

#include <Wire.h>

#include "average.hpp"

#include "jsonqueueselector.h"

extern JsonQueueSelector jsonQueueSelector;

extern int8_t ditherAmount; // variable gets reset upon user changing dither
extern double steerSupplyVoltage;
extern double steerMotorCurrent;
extern bool disabledBySpeedSafety;

extern uint16_t labelLoad;
extern uint16_t labelWheelAngle;
extern uint16_t labelSpeedDisableAutosteer;
extern uint16_t labelSupplyVoltage;
extern uint16_t labelSteerMotorCurrent;
extern uint16_t labelSteerEngagedFaults;
extern uint16_t labelSwitchStates;

extern uint16_t labelStatusOutput;
extern uint16_t labelStatusAdc;
extern uint16_t labelStatusCan;

extern SemaphoreHandle_t i2cMutex;

struct Diagnostics {
  double steerSupplyVoltageMin;
  double steerSupplyVoltageMax;
  int8_t steerEnabledWithNoPower;
  int8_t fuse1Shorted;
  int8_t fuse2Shorted;
};
extern Diagnostics diagnostics;
///////////////////////////////////////////////////////////////////////////
// Configuration
///////////////////////////////////////////////////////////////////////////

struct SteerConfig {

  enum class AnalogIn : uint8_t {
    None                    = 0,
    ADS1115A0Single         = 100,
    ADS1115A1Single         = 101,
    ADS1115A0A1Differential = 200
  };

  enum class SpeedUnits : int8_t {
    MilesPerHour      = 0,
    KilometersPerHour = 1
  } speedUnits = SpeedUnits::MilesPerHour;

  enum class Mode : uint8_t {
    QtOpenGuidance = 0,
    AgOpenGps = 1
  } mode = Mode::AgOpenGps;

  enum class DisengageSwitchType : uint8_t {
    Encoder = 0,
    Hydraulic = 1
  } disengageSwitchType = DisengageSwitchType::Encoder;

  char ssid[24] = "AOG hub";
  char password[24] = "password";
  char hostname[24] = "ESP32-QOG";
  uint8_t apModePin = 13;

  uint32_t baudrate = 115200;

  bool enableOTA = false;

  //set to 1  if you want to use Steering Motor + Cytron MD30C Driver
  //set to 2  if you want to use Steering Motor + IBT 2  Driver
  //set to 3  if you want to use IBT 2  Driver + PWM 2-Coil Valve
  //set to 4  if you want to use IBT 2  Driver + Danfoss Valve PVE A/H/M
  enum class OutputType : uint8_t {
    None = 0,
    SteeringMotorCytron = 1,
    SteeringMotorIBT2,
    HydraulicPwm2Coil,
    HydraulicDanfoss,
    HydraulicBangBang
  } outputType = OutputType::None;

  double pwmFrequency = 1000;
  bool invertOutput = false;
  float minAutosteerSpeed = 0.1;
  uint8_t gpioPwm = 27;
  uint8_t gpioDir = 26;
  uint8_t gpioEn = 25;

  bool allowPidOverwrite = false;
  double steeringPidKp = 20;
  double steeringPidKi = 0.5;
  double steeringPidKiMax = 128;
  double steeringPidKd = 1;
  double steeringPidAutoBangOnFactor = 2;
  double steeringPidBangOn = 40;
  double steeringPidBangOff = 0.1;
  uint8_t steeringPidMinPwm = 20;
  uint8_t steeringPidMaxPwm = 255;

  uint8_t dither = 0;

  enum class WorkswitchType : uint8_t {
    None = 0,
    Gpio,
    RearHitchPosition,
    FrontHitchPosition,
    RearPtoRpm,
    FrontPtoRpm,
    MotorRpm
  } workswitchType = WorkswitchType::None;
  uint8_t gpioWorkswitch = 2;
  uint8_t gpioWorkLED = 14;
  uint8_t gpioSteerswitch = 15;
  uint8_t gpioSteerLED = 12;
  uint8_t gpioDisengage = 23;
  uint16_t disengageFramePulses = 3;
  uint16_t disengageFrameMillis = 1000;
  double steeringShuntVoltsPerAmp = 1.0;
  double maxSteerCurrent = 5.00;
  bool workswitchActiveLow = true;
  bool steerswitchActiveLow = true;
  bool steerSwitchIsMomentary = false;
  bool hydraulicSwitchActiveLow = true;

  enum class WheelAngleSensorType : uint8_t {
    WheelAngle = 0,
    TieRodDisplacement
  } wheelAngleSensorType = WheelAngleSensorType::WheelAngle;

  SteerConfig::AnalogIn wheelAngleInput = SteerConfig::AnalogIn::None;

  bool invertWheelAngleSensor = false;
  float wheelAngleCountsPerDegree = 118;
  uint16_t wheelAnglePositionZero = 5450;
  uint16_t ackermann = 100;
  bool ackermannAboveZero = true;

  float wheelAngleOffset = 0;

  float wheelAngleFirstArmLenght = 92;
  float wheelAngleSecondArmLenght = 308;
  float wheelAngleTieRodStroke = 210;
  float wheelAngleMinimumAngle = 37;
  float wheelAngleTrackArmLenght = 165;

  uint8_t gpioSDA = 21;
  uint8_t gpioSCL = 22;
  uint32_t i2cBusSpeed = 400000;

  bool canBusEnabled = false;
  uint8_t canBusRx = 33;
  uint8_t canBusTx = 32;
  enum class CanBusSpeed : uint16_t {
    Speed250kbs = 250,
    Speed500kbs = 500
  } canBusSpeed = CanBusSpeed::Speed250kbs;

  uint8_t canBusHitchThreshold = 50;
  uint8_t canBusHitchThresholdHysteresis = 6;

  uint16_t canBusRpmThreshold = 400;
  uint16_t canBusRpmThresholdHysteresis = 100;

  float maxAutosteerSpeed = 10;
  uint8_t gpioAlarm = 4;

  uint16_t aogPortSendFrom = 5577;
  uint16_t aogPortListenTo = 8888;
  uint16_t aogPortSendTo = 9999;

  uint16_t qogPortListenTo = 1337;
  uint16_t qogPortSendTo = 1338;

  uint16_t qogChannelIdAutosteerEnable = 1000;    // in
  uint16_t qogChannelIdWorkswitch = 2000;
  uint16_t qogChannelIdSteerswitch = 2001;
  uint16_t qogChannelIdWheelAngle = 3000;
  uint16_t qogChannelIdSetpointSteerAngle = 4000; // in
  uint16_t qogChannelIdOrientation = 5000;
  uint16_t qogChannelIdGpsDataIn = 6000;          // in
  uint16_t qogChannelIdGpsDataOut = 6001;
  uint16_t qogChannelIdCanRearHitch = 7000;
  uint16_t qogChannelIdCanFrontHitch = 7001;
  uint16_t qogChannelIdCanRearPtoRpm = 7002;
  uint16_t qogChannelIdCanFrontPtoRpm = 7003;
  uint16_t qogChannelIdCanMotorRpm = 7004;
  uint16_t qogChannelIdCanWheelbasedSpeed = 7005;

  bool retainWifiSettings = true;
};
extern SteerConfig steerConfig, steerConfigDefaults;

struct Initialisation {
  SteerConfig::OutputType outputType = SteerConfig::OutputType::None;
  SteerConfig::AnalogIn wheelAngleInput = SteerConfig::AnalogIn::None;

  uint16_t portSendFrom = 5577;
  uint16_t portListenTo = 8888;
  uint16_t portSendTo = 9999;

};
extern Initialisation initialisation;


///////////////////////////////////////////////////////////////////////////
// Global Data
///////////////////////////////////////////////////////////////////////////

struct SteerSettings {
  float Ko = 0.0f;  //overall gain
  float Kp = 0.0f;  //proportional gain
  float Ki = 0.0f;//integral gain
  float Kd = 0.0f;  //derivative gain
  uint8_t minPWMValue = 10;
  int maxIntegralValue = 20; //max PWM value for integral PID component
  float wheelAngleCountsPerDegree = 118;
  uint16_t wheelAnglePositionZero = 0;

  time_t lastPacketReceived = 0;
};
extern SteerSettings steerSettings;

struct SteerSetpoints {
  uint8_t relais = 0;
  float speed = 0;
  uint16_t distanceFromLine = 32020;
  double requestedSteerAngle = 0;

  bool enabled = false;
  float receivedRoll = 0;
  double actualSteerAngle = 0;
  double wheelAngleCounts = 0;
  double wheelAngleCurrentDisplacement = 0;
  double wheelAngleRaw = 0;
  float correction = 0;

  time_t lastPacketReceived = 0;
};
extern SteerSetpoints steerSetpoints;

struct SteerMachineControl {
  uint8_t pedalControl = 0;
  float speed = 0;
  uint8_t relais = 0;
  uint8_t youTurn = 0;

  time_t lastPacketReceived = 0;
};
extern SteerMachineControl steerMachineControl;

struct SteerCanData {
  float speed;
  uint16_t motorRpm;
  uint8_t frontHitchPosition;
  uint8_t rearHitchPosition;
  uint16_t frontPtoRpm;
  uint16_t rearPtoRpm;
};
extern SteerCanData steerCanData;

///////////////////////////////////////////////////////////////////////////
// external Libraries
///////////////////////////////////////////////////////////////////////////

extern ESPUIClass ESPUI;

// extern AsyncUDP udpLocalPort;
// extern AsyncUDP udpRemotePort;
extern AsyncUDP udpSendFrom;

///////////////////////////////////////////////////////////////////////////
// Helper Classes
///////////////////////////////////////////////////////////////////////////
extern portMUX_TYPE mux;
class TCritSect {
    TCritSect() {
      portENTER_CRITICAL( &mux );
    }
    ~TCritSect() {
      portEXIT_CRITICAL( &mux );
    }
};

///////////////////////////////////////////////////////////////////////////
// Threads
///////////////////////////////////////////////////////////////////////////


///////////////////////////////////////////////////////////////////////////
// Helper Functions
///////////////////////////////////////////////////////////////////////////
extern void setResetButtonToRed();

extern void initESPUI();
extern void initIdleStats();
extern void initSensors();
extern void initCan();
extern void initAutosteer();
extern void initWiFi();
extern void initDiagnostics();
