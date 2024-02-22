// MIT License
//
// Copyright (c) 2020 Reuben Rissler
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

#include <stdio.h>
#include <string.h>

#include <CAN_config.h>

#include "main.hpp"
#include "jsonFunctions.hpp"

#include <string>       // std::string
#include <sstream>      // std::stringstream

CAN_device_t CAN_cfg;               // CAN Config
const int rx_queue_size = 10;       // Receive Queue size

constexpr time_t Timeout = 1000;

void canbusAutosteerWorker100Hz( void* z ) {
  constexpr TickType_t xFrequency = 10;
  TickType_t xLastWakeTime = xTaskGetTickCount();

  static time_t timeoutPoint;
  pid.setTimeStep( xFrequency );

  for( ;; ) {
    timeoutPoint = millis() - Timeout;

    if( steerSetpoints.speed > steerConfig.maxAutosteerSpeed ) {
      disabledBySpeedSafety = true;
      steerState = false;
      safetyAlarmLatch = true;
    } else if ( safetyAlarmLatch == false ) {
      disabledBySpeedSafety = false; // only proceed from safety disable, AFTER autosteer switch is turned off
    }

    if( steerSetpoints.enabled == true && disabledBySpeedSafety == true ) {
      digitalWrite( steerConfig.gpioAlarm, HIGH );
    } else if ( steerSetpoints.enabled == false && disabledBySpeedSafety == false ) {
      digitalWrite( steerConfig.gpioAlarm, LOW ); // turn off alarm after safety AND autosteer are off
    }

    if( steerConfig.manualSteerState == true ){
      if( steerConfig.manualPWM > 255 ){
        steerConfig.manualPWM = 255;
        ESPUI.updateNumber( manualValvePWMWidget, steerConfig.manualPWM);
      } else if( steerConfig.manualPWM < -255 ){
        steerConfig.manualPWM = -255;
        ESPUI.updateNumber( manualValvePWMWidget, steerConfig.manualPWM );
      }

      switch( steerConfig.canbusTractorType ) {
        case SteerConfig::CanBusTractorType::Claas:{
          // to be done
        }
        break;

        case SteerConfig::CanBusTractorType::ValtraMasseyChallenger: {
          // fixme
        }
        break;

        default:
          break;

      }
    }
    // check for timeout, data from AgOpenGPS, safety disable, and mininum autosteer speed
    else if( steerSetpoints.lastPacketReceived < timeoutPoint ||
        steerSetpoints.enabled == false ||
        disabledBySpeedSafety == true ||
        steerSetpoints.speed < steerConfig.minAutosteerSpeed ) {
      
      // autosteer disable fixme

      digitalWrite( steerConfig.gpioSteerLED, LOW );
    } else {

      pid.setGains( steerConfig.steeringPidKp, steerConfig.steeringPidKi, steerConfig.steeringPidKd );

      if( pid.getIntegral() > 0 && pid.getIntegral() > steerConfig.steeringPidKiMax ){
        pid.setIntegral(steerConfig.steeringPidKiMax);
      }
      if( -pid.getIntegral() > 0 && -pid.getIntegral() > steerConfig.steeringPidKiMax ){
        pid.setIntegral(-steerConfig.steeringPidKiMax);
      }

      // here comes the magic: executing the PID loop
      // the values are given by pointers, so the AutoPID gets them automaticaly
      pid.run();

      pidOutputTmp = steerConfig.invertOutput ? pidOutput : -pidOutput;

      if( pidOutputTmp < 0 ) {
        pidOutputTmp = map( pidOutputTmp, -steerConfig.steeringPidMaxPwm, 0, -steerConfig.steeringPidMaxPwm, -steerConfig.steeringPidMinPwm );
        pidOutputTmp = constrain( pidOutputTmp, -steerConfig.steeringPidMaxPwm, -steerConfig.steeringPidMinPwm );
      }

      if( pidOutputTmp > 0 ) {
        pidOutputTmp = map( pidOutputTmp, 0, steerConfig.steeringPidMaxPwm, steerConfig.steeringPidMinPwm, steerConfig.steeringPidMaxPwm );
        pidOutputTmp = constrain( pidOutputTmp, steerConfig.steeringPidMinPwm, steerConfig.steeringPidMaxPwm );
      }

      switch( steerConfig.canbusTractorType ) {
        case SteerConfig::CanBusTractorType::Claas:{
          // to be done
        }
        break;

        case SteerConfig::CanBusTractorType::ValtraMasseyChallenger: {
          // fixme
        }
        break;

        default:
          break;

      }
      digitalWrite( steerConfig.gpioSteerLED, HIGH );
    }

    static uint8_t loopCounter = 0;

    if( ++loopCounter >= 10 ) {
      loopCounter = 0;

      uint8_t data[14] = {0};

      data[0] = 0x80; // AOG specific
      data[1] = 0x81; // AOG specific
      data[2] = 0x7F; // autosteer module to AOG
      data[3] = 0xFD; // autosteer module to AOG
      data[4] = 8;    // length of data

      {
        int16_t steerAngle = steerSetpoints.actualSteerAngle * 100 ;
        data[5] = ( uint16_t )steerAngle;
        data[6] = ( uint16_t )steerAngle >> 8;
      }

      // read inputs
      {
        if( steerConfig.workswitchType != SteerConfig::WorkswitchType::None ) {
          uint16_t value = 0;
          uint16_t threshold = 0;
          uint16_t hysteresis = 0;

          switch( steerConfig.workswitchType ) {
            case SteerConfig::WorkswitchType::Gpio:
              value =  digitalRead( ( uint8_t )steerConfig.gpioWorkswitch ) ? 1 : 0;
              threshold = 1;
              hysteresis = 0;
              break;

            case SteerConfig::WorkswitchType::RearHitchPosition:
              value = steerCanData.rearHitchPosition;
              threshold = steerConfig.canBusHitchThreshold;
              hysteresis = steerConfig.canBusHitchThresholdHysteresis;
              break;

            case SteerConfig::WorkswitchType::FrontHitchPosition:
              value = steerCanData.frontHitchPosition;
              threshold = steerConfig.canBusHitchThreshold;
              hysteresis = steerConfig.canBusHitchThresholdHysteresis;
              break;

            case SteerConfig::WorkswitchType::RearPtoRpm:
              value = steerCanData.rearPtoRpm;
              threshold = steerConfig.canBusRpmThreshold;
              hysteresis = steerConfig.canBusRpmThresholdHysteresis;
              break;

            case SteerConfig::WorkswitchType::FrontPtoRpm:
              value = steerCanData.frontPtoRpm;
              threshold = steerConfig.canBusRpmThreshold;
              hysteresis = steerConfig.canBusRpmThresholdHysteresis;
              break;

            case SteerConfig::WorkswitchType::MotorRpm:
              value = steerCanData.motorRpm;
              threshold = steerConfig.canBusRpmThreshold;
              hysteresis = steerConfig.canBusRpmThresholdHysteresis;
              break;

            default:
              break;
          }

          static bool workswitchState = false;

          if( value >= threshold ) {
            workswitchState = true;
          }

          if( value < ( threshold - hysteresis ) ) {
            workswitchState = false;
          }

          if( steerConfig.workswitchActiveLow ) {
            workswitchState = ! workswitchState;
          }

          data[10] |= workswitchState ? 1 : 0;
          digitalWrite( steerConfig.gpioWorkLED, workswitchState);
        }

          if(( steerState == false || steerSetpoints.enabled == false ) && steerConfig.manualSteerState == true ){
            steerConfig.manualSteerState = false;
            ESPUI.updateSwitcher( manualValveSwitcher, false );
          }

          data[11] |= steerState ? 0 : 2;
      }
        //data[12] = 0; // PWM ?
      //add the checksum
      int CRCtoAOG = 0;
      for (byte i = 2; i < sizeof(data) - 1; i++)
      {
        CRCtoAOG = (CRCtoAOG + data[i]);
      }
      data[sizeof(data) - 1] = CRCtoAOG;

      udpSendFrom.broadcastTo( data, sizeof( data ), initialisation.portSendTo );

    }
    vTaskDelayUntil( &xLastWakeTime, xFrequency );
  }
}

void initCanbusAutosteer() {

  CAN_cfg.speed =  ( CAN_speed_t )steerConfig.canBusSpeed;;
  CAN_cfg.tx_pin_id = ( gpio_num_t )steerConfig.canBusTx;
  CAN_cfg.rx_pin_id = ( gpio_num_t )steerConfig.canBusRx;
  CAN_cfg.rx_queue = xQueueCreate(rx_queue_size, sizeof(CAN_frame_t));

  CAN_filter_t p_filter;
  p_filter.FM = Single_Mode;

  p_filter.ACR0 = 0x0C;
  p_filter.ACR1 = 0xAC;
  p_filter.ACR2 = 0x1C;
  p_filter.ACR3 = 0x13;

  p_filter.AMR0 = 0x18;
  p_filter.AMR1 = 0xEF;
  p_filter.AMR2 = 0x1C;
  p_filter.AMR3 = 0x32;
  ESP32Can.CANConfigFilter(&p_filter);

  // Init CAN Module
  ESP32Can.CANInit();

  if( steerConfig.aogPortSendFrom != 0 ) {
    initialisation.portSendFrom = steerConfig.aogPortSendFrom;
  }

  if( steerConfig.aogPortListenTo != 0 ) {
    initialisation.portListenTo = steerConfig.aogPortListenTo;
  }

  if( steerConfig.aogPortSendTo != 0 ) {
    initialisation.portSendTo = steerConfig.aogPortSendTo;
  }

  udpSendFrom.listen( initialisation.portSendFrom );

  if( udpLocalPort.listen( initialisation.portListenTo ) ) {
    udpLocalPort.onPacket( []( AsyncUDPPacket packet ) {
      uint8_t* data = packet.data();
      if( data[1] + ( data[0] << 8 ) != 0x8081 ){
          return;
      }
      uint16_t pgn = data[3] + ( data[2] << 8 );
      // see pgn.xlsx in https://github.com/farmerbriantee/AgOpenGPS/tree/master/AgOpenGPS_Dev
      switch( pgn ) {
        case 0x7FFE: {
          steerSetpoints.speed = ( float )( (data[5] | data[6] << 8))*0.1 ;
          if( ( SteerConfig::SpeedUnits )steerConfig.speedUnits == SteerConfig::SpeedUnits::MilesPerHour ) {
            steerSetpoints.speed *= 0.6213711922;
          }
          steerSetpoints.enabled = data[7];
          steerSetpoints.requestedSteerAngle = (( double ) ((( int16_t )data[8]) | (( int8_t )data[9] << 8 ))) * 0.01; //horrible code to make negative doubles work

          steerSetpoints.lastPacketReceived = millis();
        }
        break;

        case 0x7FFC: {
          steerSettings.Kp = ( float )data[2] * 1.0; // read Kp from AgOpenGPS
          steerSettings.Ki = ( float )data[3] * 0.001; // read Ki from AgOpenGPS
          steerSettings.Kd = ( float )data[4] * 1.0; // read Kd from AgOpenGPS
          steerSettings.Ko = ( float )data[5] * 0.1; // read Ko from AgOpenGPS
          steerSettings.wheelAnglePositionZero = ( int8_t )data[6]; //read steering zero offset
          steerSettings.minPWMValue = data[7]; //read the minimum amount of PWM for instant on
          steerSettings.maxIntegralValue = data[8] * 0.1; //
          steerSettings.wheelAngleCountsPerDegree = data[9]; //sent as 10 times the setting displayed in AOG

          steerSettings.lastPacketReceived = millis();
        }
        break;

        case 0x7FF6: {
          steerMachineControl.pedalControl = data[2];
          steerMachineControl.speed = ( float )data[3] / 4;
          steerMachineControl.relais = data[4];
          steerMachineControl.youTurn = data[5];

          steerMachineControl.lastPacketReceived = millis();
        }
        break;

        default:
          break;
      }
    } );
  }

  // init output
  {

    pinMode( steerConfig.gpioSteerLED, OUTPUT );
    digitalWrite( steerConfig.gpioSteerLED, LOW );

    pinMode( steerConfig.gpioAlarm, OUTPUT );
    digitalWrite( steerConfig.gpioAlarm, LOW );

  }

  xTaskCreate( canbusAutosteerWorker100Hz, "autosteerWorker", 3096, NULL, 3, NULL );
}
