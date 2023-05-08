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

#include <stdio.h>
#include <string.h>

#include <AutoPID.h>

#include "main.hpp"
#include "jsonFunctions.hpp"

#include <string>       // std::string
#include <sstream>      // std::stringstream

#include "soc/mcpwm_reg.h"
#include "soc/mcpwm_struct.h"
#include "driver/mcpwm.h"

SteerSettings steerSettings;
SteerSetpoints steerSetpoints;
SteerMachineControl steerMachineControl;

AsyncUDP udpSendFrom;
AsyncUDP udpLocalPort;
AsyncUDP udpRemotePort;

double pidOutput = 0;
double pidOutputTmp = 0;
AutoPID pid(
        &( steerSetpoints.actualSteerAngle ),
        &( steerSetpoints.requestedSteerAngle ),
        &( pidOutput ),
        -255, 255,
        steerConfig.steeringPidKp, steerConfig.steeringPidKi, steerConfig.steeringPidKd );

JsonQueueSelector jsonQueueSelector;

constexpr time_t Timeout = 1000;
volatile bool disengageState;
volatile bool disengagePrevState;
volatile bool steerState = false;
volatile bool steerChangeProcessed = true;
volatile time_t steerChangeMillis = millis();
volatile time_t disengageActivityMillis = millis();
volatile uint16_t steeringPulseCount = 0;

bool safetyAlarmLatch = false;
bool ditherDirection = false;
bool dtcAutosteerPrevious = false;
bool disabledBySpeedSafety = false;

void ditherWorkerHalfHZ( void* z ) {

  for( ;; ) {
    ditherAmount += ditherDirection ? -1 : 1 ;
    if( abs( ditherAmount ) >= steerConfig.dither ){
      ditherDirection = !ditherDirection;
    }
    vTaskDelay( pdMS_TO_TICKS( 500 ) );
  }
  vTaskDelete( NULL );
}

void autosteerWorker100Hz( void* z ) {
  constexpr TickType_t xFrequency = 10;
  TickType_t xLastWakeTime = xTaskGetTickCount();

  pid.setTimeStep( xFrequency );

  QueueHandle_t queue = xQueueCreate( 16, sizeof( json* ) );
  jsonQueueSelector.addQueue( steerConfig.qogChannelIdAutosteerEnable, queue );
  jsonQueueSelector.addQueue( steerConfig.qogChannelIdSetpointSteerAngle, queue );

  for( ;; ) {
    time_t timeoutPoint = millis() - Timeout;

    if( steerConfig.mode == SteerConfig::Mode::QtOpenGuidance ) {
      while( uxQueueMessagesWaiting( queue ) ) {
        json* j = nullptr;

        if( xQueueReceive( queue, &j, 0 ) == pdTRUE ) {
          uint16_t channelId = j->at( "channelId" );

          if( channelId == steerConfig.qogChannelIdAutosteerEnable ) {
            if( j->contains( "state" ) ) {
              steerSetpoints.enabled = j->at( "state" );
              steerSetpoints.lastPacketReceived = millis();
            }
          }

          if( channelId == steerConfig.qogChannelIdSetpointSteerAngle ) {
            if( j->contains( "number" ) ) {
              steerSetpoints.requestedSteerAngle = j->at( "number" );
              steerSetpoints.lastPacketReceived = millis();
            }
          }

          delete j;
        }
      }
    }

    if( steerSetpoints.speed > steerConfig.maxAutosteerSpeed ) {
      disabledBySpeedSafety = true;
      steerState = false;
      safetyAlarmLatch = true;
    } else if ( safetyAlarmLatch == false ) {
      disabledBySpeedSafety = false; // only proceed from safety disable, AFTER autosteer switch is turned off
    }

    if( steerSetpoints.enabled == true ){
      if(  dtcAutosteerPrevious == false && steerSupplyVoltage < 14500 ){  // 10.8 volts
          diagnostics.steerEnabledWithNoPower += 1;

          Control* labelSteerEngagedFaultsHandle = ESPUI.getControl( labelSteerEngagedFaults );
          String str;
          str.reserve( 30 );
          str = "Number of faults: ";
          str += ( int8_t ) diagnostics.steerEnabledWithNoPower;
          str += "\nFault active since startup: Yes";
          labelSteerEngagedFaultsHandle->value = str;
          labelSteerEngagedFaultsHandle->color = ControlColor::Alizarin;
          ESPUI.updateControlAsync( labelSteerEngagedFaultsHandle );
          saveDiagnostics();
      }
    }
    dtcAutosteerPrevious = steerSetpoints.enabled;

    if( steerSetpoints.enabled == true && disabledBySpeedSafety == true ) {
      digitalWrite( steerConfig.gpioAlarm, HIGH );
    } else if ( steerSetpoints.enabled == false && disabledBySpeedSafety == false ) {
      digitalWrite( steerConfig.gpioAlarm, LOW ); // turn off alarm after safety AND autosteer are off
    }

    // check for timeout, data from AgOpenGPS, safety disable, and mininum autosteer speed
    if( steerSetpoints.lastPacketReceived < timeoutPoint ||
        steerSetpoints.enabled == false ||
        disabledBySpeedSafety == true ||
        steerSetpoints.speed < steerConfig.minAutosteerSpeed ) {
      
      switch( initialisation.outputType ) {
        case SteerConfig::OutputType::HydraulicDanfoss: {
          ledcWrite( 0, 128 );
          ledcWrite( 1, 0 );
          ledcWrite( 2, 0 );
        }
        break;

        default: {
          ledcWrite( 0, 0 );
          ledcWrite( 1, 0 );
          ledcWrite( 2, 0 );
        }
        break;
      }

      digitalWrite( steerConfig.gpioSteerLED, LOW );
    } else {

      diagnostics.steerSupplyVoltageMax = max( steerSupplyVoltage, diagnostics.steerSupplyVoltageMax );
      diagnostics.steerSupplyVoltageMin = min( steerSupplyVoltage, diagnostics.steerSupplyVoltageMin );

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
      if( pidOutput ) {

        pidOutputTmp = steerConfig.invertOutput ? pidOutput : -pidOutput;

        if( pidOutputTmp < 0 ) {
          pidOutputTmp = map( pidOutputTmp, -steerConfig.steeringPidMaxPwm, 0, -steerConfig.steeringPidMaxPwm, -steerConfig.steeringPidMinPwm );
          pidOutputTmp = constrain( pidOutputTmp, -steerConfig.steeringPidMaxPwm, -steerConfig.steeringPidMinPwm );
        }

        if( pidOutputTmp > 0 ) {
          pidOutputTmp = map( pidOutputTmp, 0, steerConfig.steeringPidMaxPwm, steerConfig.steeringPidMinPwm, steerConfig.steeringPidMaxPwm );
          pidOutputTmp = constrain( pidOutputTmp, steerConfig.steeringPidMinPwm, steerConfig.steeringPidMaxPwm );
        }

        pidOutputTmp -= ditherAmount; // only valid for Hydraulic Pwm 2 Coil, don't increase PWM output above 255

        switch( initialisation.outputType ) {
          case SteerConfig::OutputType::SteeringMotorIBT2:
          case SteerConfig::OutputType::HydraulicPwm2Coil: {
            ledcWrite( 0, 0 );
            if( pidOutputTmp >= 0 ) {
              ledcWrite( 1, pidOutputTmp );
              ledcWrite( 2, 0 );
            }
            if( pidOutputTmp < 0 ) {
              ledcWrite( 1, 0 );
              ledcWrite( 2, -pidOutputTmp );
            }
          }
          break;

          case SteerConfig::OutputType::SteeringMotorCytron: {
            if( pidOutputTmp >= 0 ) {
              ledcWrite( 1, 255 );
            } else {
              ledcWrite( 0, 255 );
              pidOutputTmp = -pidOutputTmp;
            }

            ledcWrite( 0, pidOutputTmp );
            ledcWrite( 2, 255 );
          }
          break;

          case SteerConfig::OutputType::HydraulicDanfoss: {
            ledcWrite( 2, 255 );
            uint8_t lowRange = 255 - steerConfig.steeringPidMaxPwm;
            pidOutputTmp = map( pidOutputTmp, -steerConfig.steeringPidMaxPwm, steerConfig.steeringPidMaxPwm, lowRange, steerConfig.steeringPidMaxPwm );
            ledcWrite( 0, pidOutputTmp );
          }
          break;

          default:
            break;
        }

      }
      digitalWrite( steerConfig.gpioSteerLED, HIGH );
    }

    static uint8_t loopCounter = 0;

    if( ++loopCounter >= 10 ) {
      loopCounter = 0;

      uint8_t data[14] = {0};

      if( steerConfig.mode == SteerConfig::Mode::AgOpenGps ) {
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

          if( steerConfig.mode == SteerConfig::Mode::QtOpenGuidance ) {
            sendStateTransmission( steerConfig.qogChannelIdWorkswitch, workswitchState );
          }

          if( steerConfig.mode == SteerConfig::Mode::AgOpenGps ) {
            data[10] |= workswitchState ? 1 : 0;
          }
          digitalWrite( steerConfig.gpioWorkLED, workswitchState);
        }

          if( steerChangeProcessed == false && ( millis() - steerChangeMillis ) > 200 ) {
            if( digitalRead( ( uint8_t )steerConfig.gpioSteerswitch) == steerConfig.steerswitchActiveLow) {
              steerChangeProcessed = true;
              steerState = ! steerState;
              if( steerState == false ) {
                safetyAlarmLatch = false;
              }
            }
          }

          if( steerConfig.disengageSwitchType == SteerConfig::DisengageSwitchType::Hydraulic ){
            if( digitalRead( steerConfig.gpioDisengage ) != steerConfig.hydraulicSwitchActiveLow ){
              steerState = false;
            }
          }
          else if( millis() - disengageActivityMillis < steerConfig.disengageFrameMillis ) {
            if( ( steeringPulseCount / 2 ) >= steerConfig.disengageFramePulses ) { // divide by two to compensate for LOW and HIGH
              steerState = false;
              steeringPulseCount = 0;
            }
          } else { steeringPulseCount = 0; }

          if( steerConfig.mode == SteerConfig::Mode::QtOpenGuidance ) {
            sendStateTransmission( steerConfig.qogChannelIdSteerswitch, steerState );
          }

          if( steerConfig.mode == SteerConfig::Mode::AgOpenGps ) {
            data[11] |= steerState ? 0 : 2;
          }
      }
        //data[12] = 0; // PWM ?
      //add the checksum
      int CRCtoAOG = 0;
      for (byte i = 2; i < sizeof(data) - 1; i++)
      {
        CRCtoAOG = (CRCtoAOG + data[i]);
      }
      data[sizeof(data) - 1] = CRCtoAOG;

      if( steerConfig.mode == SteerConfig::Mode::AgOpenGps ) {
        udpSendFrom.broadcastTo( data, sizeof( data ), initialisation.portSendTo );
      }

      {
        switch( steerConfig.outputType ) {
          case SteerConfig::OutputType::SteeringMotorIBT2: {
            Control* labelStatusOutputHandle = ESPUI.getControl( labelStatusOutput );
            String str;
            str.reserve( 30 );
            str = "IBT2 Motor, SetPoint: ";
            str += ( float )steerSetpoints.requestedSteerAngle;
            str += "°,\ntimeout: ";
            str += ( bool )( steerSetpoints.lastPacketReceived < timeoutPoint ) ? "Yes" : "No" ;
            str += ", enabled: ";
            str += ( bool )steerSetpoints.enabled ? "Yes" : "No" ;
            str += ", output: ";
            str += ( float )pidOutputTmp ;
            labelStatusOutputHandle->value = str;
            labelStatusOutputHandle->color = ControlColor::Emerald;
            ESPUI.updateControlAsync( labelStatusOutputHandle );
          }
          break;

          case SteerConfig::OutputType::SteeringMotorCytron: {
            Control* labelStatusOutputHandle = ESPUI.getControl( labelStatusOutput );
            String str;
            str.reserve( 30 );
            str = "Cytron Motor, SetPoint: ";
            str += ( float )steerSetpoints.requestedSteerAngle;
            str += "°\ntimeout: ";
            str += ( bool )( steerSetpoints.lastPacketReceived < timeoutPoint ) ? "Yes" : "No" ;
            str += ", enabled: ";
            str += ( bool )steerSetpoints.enabled ? "Yes" : "No" ;
            str += ", output: ";
            str += ( float )pidOutputTmp ;
            labelStatusOutputHandle->value = str;
            labelStatusOutputHandle->color = ControlColor::Emerald;
            ESPUI.updateControlAsync( labelStatusOutputHandle );
          }
          break;

          case SteerConfig::OutputType::HydraulicPwm2Coil: {
            Control* labelStatusOutputHandle = ESPUI.getControl( labelStatusOutput );
            String str;
            str.reserve( 30 );
            str = "IBT2 Hydraulic PWM 2 Coil, SetPoint: ";
            str += ( float )steerSetpoints.requestedSteerAngle;
            str += "°,\ntimeout: ";
            str += ( bool )( steerSetpoints.lastPacketReceived < timeoutPoint ) ? "Yes" : "No" ;
            str += ", enabled: ";
            str += ( bool )steerSetpoints.enabled ? "Yes" : "No" ;
            str += ", output: ";
            str += ( float )pidOutputTmp ;
            labelStatusOutputHandle->value = str;
            labelStatusOutputHandle->color = ControlColor::Emerald;
            ESPUI.updateControlAsync( labelStatusOutputHandle );
          }
          break;

          case SteerConfig::OutputType::HydraulicDanfoss: {
            Control* labelStatusOutputHandle = ESPUI.getControl( labelStatusOutput );
            String str;
            str.reserve( 30 );
            str = "IBT2 Hydraulic Danfoss, SetPoint: ";
            str += ( float )steerSetpoints.requestedSteerAngle;
            str += "°,\ntimeout: ";
            str += ( bool )( steerSetpoints.lastPacketReceived < timeoutPoint ) ? "Yes" : "No" ;
            str += ", enabled: ";
            str += ( bool )steerSetpoints.enabled ? "Yes" : "No" ;
            str += ", output: ";
            str += ( float )pidOutputTmp ;
            labelStatusOutputHandle->value = str;
            labelStatusOutputHandle->color = ControlColor::Emerald;
            ESPUI.updateControlAsync( labelStatusOutputHandle );
          }
          break;

          case SteerConfig::OutputType::HydraulicBangBang: {
            Control* labelStatusOutputHandle = ESPUI.getControl( labelStatusOutput );
            String str;
            str.reserve( 30 );
            str = "IBT2 Hydraulic Bang Bang, SetPoint: ";
            str += ( float )steerSetpoints.requestedSteerAngle;
            str += "°,\ntimeout: ";
            str += ( bool )( steerSetpoints.lastPacketReceived < timeoutPoint ) ? "Yes" : "No" ;
            str += ", enabled: ";
            str += ( bool )steerSetpoints.enabled ? "Yes" : "No" ;
            str += ", output: ";
            str += ( float )pidOutputTmp ;
            labelStatusOutputHandle->value = str;
            labelStatusOutputHandle->color = ControlColor::Emerald;
            ESPUI.updateControlAsync( labelStatusOutputHandle );
          }
          break;

          default:
            break;

        }
      }

    }
    vTaskDelayUntil( &xLastWakeTime, xFrequency );
  }
}

void IRAM_ATTR steerswitchMomentaryIsr() {
    // interrupt service routine for a momentary steer switch
    // the rest of the debounce gets calculated in void autosteerWorker100Hz()
    steerChangeMillis = millis();
    steerChangeProcessed = false;
}

void IRAM_ATTR steerswitchMaintainedIsr() {
    // interrupt service routine for a maintained steer switch
    // no debounce needed for maintained switches
    if( digitalRead( ( uint8_t )steerConfig.gpioSteerswitch) == steerConfig.steerswitchActiveLow){
        steerState = false;
        safetyAlarmLatch = false;
    } else {
        steerState = true;
    }
}

void IRAM_ATTR disengageIsr() {
    // interrupt service routine for the steering wheel
    // due to a problem in ESP32, we have to actually check for a change in the input state
    disengageState = digitalRead( ( uint8_t ) steerConfig.gpioDisengage );
    if( disengagePrevState != disengageState ){
      disengagePrevState = disengageState;
      if( steeringPulseCount == 0 ){
        disengageActivityMillis = millis();
      }
      steeringPulseCount += 1;
    }
}

void initAutosteer() {
  if( steerConfig.mode == SteerConfig::Mode::QtOpenGuidance ) {
    if( steerConfig.qogPortListenTo != 0 ) {
      initialisation.portListenTo = steerConfig.qogPortListenTo;
    }

    if( steerConfig.qogPortSendTo != 0 ) {
      initialisation.portSendTo = steerConfig.qogPortSendTo;
    }

    if( udpLocalPort.listen( initialisation.portListenTo ) ) {
      udpLocalPort.onPacket( []( AsyncUDPPacket packet ) {
        std::vector<uint8_t> v;
        v.reserve( packet.length() );

        for( uint16_t i = 0; i < packet.length(); ++i ) {
          v.push_back( packet.data()[i] );
        }

        json* j = new json;

        try {
          *j = json::from_cbor( v );

          if( j->is_object() ) {
            if( j->contains( "channelId" ) ) {
              // valid data -> reset timeout
              steerSetpoints.lastPacketReceived = millis();

              uint16_t channelId = j->at( "channelId" );

              if( jsonQueueSelector.isValidChannelId( channelId ) ) {
                xQueueSend( jsonQueueSelector.getQueue( channelId ), &j, 0 );
                return;
              }
            }
          }
        } catch( json::exception& e ) {
          // output exception information
          Serial.print( "message: " );
          Serial.println( e.what() );
          Serial.print( "exception id: " );
          Serial.print( e.id );
          Serial.print( ", packet.length(): " );
          Serial.println( packet.length() );
          Serial.write( v.data(), v.size() );
          Serial.println();
        }

        delete j;
      } );
    }
  }

  if( steerConfig.mode == SteerConfig::Mode::AgOpenGps ) {
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
            steerSetpoints.requestedSteerAngle = ( int16_t )( ( data[9] << 8 ) | data[8] ) / 100;

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
  }

  // init output
  {
    pinMode( steerConfig.gpioPwm, OUTPUT );
    ledcSetup( 0, steerConfig.pwmFrequency, 8 );
    ledcAttachPin( steerConfig.gpioPwm, 0 );
    ledcWrite( 0, 0 );

    pinMode( steerConfig.gpioDir, OUTPUT );
    ledcSetup( 1, steerConfig.pwmFrequency, 8 );
    ledcAttachPin( steerConfig.gpioDir, 1 );
    ledcWrite( 1, 0 );

    pinMode( steerConfig.gpioEn, OUTPUT );
    ledcSetup( 2, steerConfig.pwmFrequency, 8 );
    ledcAttachPin( steerConfig.gpioEn, 2 );
    ledcWrite( 2, 0 );

    pinMode( steerConfig.gpioSteerLED, OUTPUT );
    digitalWrite( steerConfig.gpioSteerLED, LOW );

    pinMode( steerConfig.gpioAlarm, OUTPUT );
    digitalWrite( steerConfig.gpioAlarm, LOW );

    switch( steerConfig.outputType ) {
      case SteerConfig::OutputType::SteeringMotorIBT2: {
        initialisation.outputType = SteerConfig::OutputType::SteeringMotorIBT2;
      }
      break;

      case SteerConfig::OutputType::SteeringMotorCytron: {
        initialisation.outputType = SteerConfig::OutputType::SteeringMotorCytron;
      }
      break;

      case SteerConfig::OutputType::HydraulicPwm2Coil: {
        initialisation.outputType = SteerConfig::OutputType::HydraulicPwm2Coil;
      }
      break;

      case SteerConfig::OutputType::HydraulicDanfoss: {
        initialisation.outputType = SteerConfig::OutputType::HydraulicDanfoss;
      }
      break;

      case SteerConfig::OutputType::HydraulicBangBang: {
        initialisation.outputType = SteerConfig::OutputType::HydraulicBangBang;
      }
      break;

      default:
        break;

    }
  }

  pinMode( steerConfig.gpioWorkswitch, INPUT_PULLUP );
  pinMode( steerConfig.gpioWorkLED, OUTPUT );

  // use interrupt callbacks to simpify steer state tracking,
  // whichever callback happens last (steering wheel or steer switch) gets priority
  pinMode( steerConfig.gpioSteerswitch, INPUT_PULLUP );
  if( steerConfig.steerSwitchIsMomentary ){
      attachInterrupt( steerConfig.gpioSteerswitch, steerswitchMomentaryIsr, CHANGE);
  } else {
      attachInterrupt( steerConfig.gpioSteerswitch, steerswitchMaintainedIsr, CHANGE);
  }

  pinMode( steerConfig.gpioDisengage, INPUT_PULLUP );
  if ( steerConfig.disengageSwitchType == SteerConfig::DisengageSwitchType::Encoder ){
    attachInterrupt( steerConfig.gpioDisengage, disengageIsr, CHANGE);
  }

  xTaskCreate( autosteerWorker100Hz, "autosteerWorker", 3096, NULL, 3, NULL );
  if( steerConfig.outputType == SteerConfig::OutputType::HydraulicPwm2Coil ) {
    xTaskCreate( ditherWorkerHalfHZ, "ditherWorkerHalfHZ", 1024, NULL, 1, NULL );
  }
}
