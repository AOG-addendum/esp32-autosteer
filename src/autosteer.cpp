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

SteerSettings steerSettings;
SteerSetpoints steerSetpoints;
SteerMachineControl steerMachineControl;

AsyncUDP udpSendFrom;
AsyncUDP udpLocalPort;
AsyncUDP udpRemotePort;

double pidOutput = 0;
AutoPID pid(
        &( steerSetpoints.actualSteerAngle ),
        &( steerSetpoints.requestedSteerAngle ),
        &( pidOutput ),
        -255, 255,
        steerConfig.steeringPidKp, steerConfig.steeringPidKi, steerConfig.steeringPidKd );

JsonQueueSelector jsonQueueSelector;

constexpr time_t Timeout = 1000;
volatile bool steerState = false;
volatile bool steerChangeProcessed = true;
volatile time_t steerChangeMillis = millis();
volatile time_t steeringWheelActivityMillis = millis();
volatile uint16_t steeringPulseCount = 0;

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

    // check for timeout and data from AgOpenGPS
    if( steerSetpoints.lastPacketReceived < timeoutPoint ||
        ( steerConfig.mode == SteerConfig::Mode::AgOpenGps && steerSetpoints.distanceFromLine == 32020 ) ||
        ( steerConfig.mode == SteerConfig::Mode::QtOpenGuidance && steerSetpoints.enabled == false )/* ||
         steerSetpoints.speed < 1*/
      ) {
      if( steerConfig.mode == SteerConfig::Mode::AgOpenGps ) {
        steerSetpoints.enabled = false;
      }

      switch( initialisation.outputType ) {
        case SteerConfig::OutputType::HydraulicDanfoss: {
          ledcWrite( 0, 128 );
          ledcWrite( 1, 0 );
        }
        break;

        default: {
          ledcWrite( 0, 0 );
          ledcWrite( 1, 0 );
        }
        break;
      }

      if( steerConfig.gpioEn != SteerConfig::Gpio::None ) {
        digitalWrite( ( uint8_t )steerConfig.gpioEn, LOW );
      }
      if( steerConfig.gpioSteerLED != SteerConfig::Gpio::None ) {
        digitalWrite( ( uint8_t )steerConfig.gpioSteerLED, LOW );
      }
    } else {
      if( steerConfig.mode == SteerConfig::Mode::AgOpenGps ) {
        steerSetpoints.enabled = true;
      }

      pid.setGains( steerConfig.steeringPidKp, steerConfig.steeringPidKi, steerConfig.steeringPidKd );

      if( steerConfig.steeringPidAutoBangOnFactor ) {
        pid.setBangBang( ( ( double )0xFF / steerSettings.Kp ) * steerConfig.steeringPidAutoBangOnFactor, steerConfig.steeringPidBangOff );
      } else {
        pid.setBangBang( steerConfig.steeringPidBangOn, steerConfig.steeringPidBangOff );
      }

      // here comes the magic: executing the PID loop
      // the values are given by pointers, so the AutoPID gets them automaticaly
      pid.run();

//         Serial.print( "actualSteerAngle: " );
//         Serial.print( steerSetpoints.actualSteerAngle );
//         Serial.print( ", requestedSteerAngle: " );
//         Serial.print( steerSetpoints.requestedSteerAngle );
//         Serial.print( "pidOutput: " );
//         Serial.println( pidOutput );

      if( pidOutput ) {

        double pidOutputTmp = steerConfig.invertOutput ? pidOutput : -pidOutput;

        if( pidOutputTmp < 0 && pidOutputTmp > -steerConfig.steeringPidMinPwm ) {
          pidOutputTmp = -steerConfig.steeringPidMinPwm;
        }

        if( pidOutputTmp > 0 && pidOutputTmp < steerConfig.steeringPidMinPwm ) {
          pidOutputTmp = steerConfig.steeringPidMinPwm;
        }

        switch( initialisation.outputType ) {
          case SteerConfig::OutputType::SteeringMotorIBT2:
          case SteerConfig::OutputType::HydraulicPwm2Coil: {
            if( pidOutputTmp >= 0 ) {
              ledcWrite( 0, pidOutputTmp );
              ledcWrite( 1, 0 );
            }

            if( pidOutputTmp < 0 ) {
              ledcWrite( 0, 0 );
              ledcWrite( 1, -pidOutputTmp );
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

            if( steerConfig.gpioEn != SteerConfig::Gpio::None ) {
              digitalWrite( ( uint8_t )steerConfig.gpioEn, HIGH );
            }
          }
          break;

          case SteerConfig::OutputType::HydraulicDanfoss: {

            // go from 25% on: max left, 50% on: center, 75% on: right max
            if( pidOutputTmp >  250 ) {
              pidOutputTmp =  250;
            }

            if( pidOutputTmp < -250 ) {
              pidOutputTmp = -250;
            }

            pidOutputTmp /= 4;
            pidOutputTmp += 128;
            ledcWrite( 0, pidOutputTmp );
          }
          break;

          case SteerConfig::OutputType::HydraulicBangBang: {
            if( pidOutputTmp > 0 ) {
              ledcWrite( 0, 255 );
              ledcWrite( 1, 0 );
              vTaskDelayUntil( &xLastWakeTime, pidOutputTmp );
              ledcWrite( 0, 0 );
            }

            if( pidOutputTmp < 0 ) {
              ledcWrite( 0, 0 );
              ledcWrite( 1, 255 );
              vTaskDelayUntil( &xLastWakeTime, -pidOutputTmp );
              ledcWrite( 1, 0 );
            }
            else {
              ledcWrite( 0, 0 );
              ledcWrite( 1, 0 );
            }
          }
          break;

          default:
            break;
        }

        if( steerConfig.gpioEn != SteerConfig::Gpio::None ) {
          digitalWrite( ( uint8_t )steerConfig.gpioEn, HIGH );
        }
        if( steerConfig.gpioSteerLED != SteerConfig::Gpio::None ) {
          digitalWrite( ( uint8_t )steerConfig.gpioSteerLED, HIGH );
        }
      } else {
        ledcWrite( 0, 0 );
        ledcWrite( 1, 0 );
      }
    }

    if( steerConfig.mode == SteerConfig::Mode::QtOpenGuidance ) {

    }

    static uint8_t loopCounter = 0;

    if( ++loopCounter >= 10 ) {
      loopCounter = 0;

      if( initialisation.outputType != SteerConfig::OutputType::None ) {
        uint8_t data[10] = {0};

        if( steerConfig.mode == SteerConfig::Mode::AgOpenGps ) {
          data[0] = 0x7F;
          data[1] = 0xFD;

          {
            int16_t steerAngle = steerSetpoints.actualSteerAngle * 100;
            data[2] = ( uint16_t )steerAngle >> 8;
            data[3] = ( uint16_t )steerAngle;
          }

          {
            uint16_t heading;

            if( initialisation.imuType != SteerConfig::ImuType::None ) {
              heading = ( float )steerImuInclinometerData.heading * 16;
            } else {
              heading = 9999;
            }

            data[4] = heading >> 8;
            data[5] = heading;
          }

          {
            int16_t roll;

            if( initialisation.inclinoType != SteerConfig::InclinoType::None ) {
              roll = steerImuInclinometerData.roll * 16;

              if( steerConfig.invertRoll ) {
                roll = -roll;
              }
            } else {
              roll = 9999;
            }

            data[6] = ( uint16_t )roll >> 8;
            data[7] = ( uint16_t )roll;
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
              data[8] |= workswitchState ? 1 : 0;
            }
            if( steerConfig.gpioWorkLED != SteerConfig::Gpio::None ) {
              digitalWrite( ( uint8_t )steerConfig.gpioWorkLED, workswitchState);
            }
          }

            if( steerChangeProcessed == false && ( millis() - steerChangeMillis ) > 200 ) {
              if( digitalRead( ( uint8_t )steerConfig.gpioSteerswitch) == steerConfig.steerswitchActiveLow) {
                steerChangeProcessed = true;
                steerState = ! steerState;
              }
            }

            if( millis() - steeringWheelActivityMillis < steerConfig.steeringWheelFrameMillis ) {
              if( ( steeringPulseCount / 2 ) > steerConfig.steeringWheelFramePulses ) { // divide by two to compensate for LOW and HIGH
                steerState = false;
                steeringPulseCount = 0;
              }
            } else { steeringPulseCount = 0; }

            if( steerConfig.mode == SteerConfig::Mode::QtOpenGuidance ) {
              sendStateTransmission( steerConfig.qogChannelIdSteerswitch, steerState );
            }

            if( steerConfig.mode == SteerConfig::Mode::AgOpenGps ) {
              data[8] |= steerState ? 0 : 2;
            }
        }

        if( steerConfig.mode == SteerConfig::Mode::AgOpenGps ) {
          udpSendFrom.broadcastTo( data, sizeof( data ), initialisation.portSendTo );
        }

      } else {
        if( ( steerConfig.mode == SteerConfig::Mode::AgOpenGps ) &&
            ( initialisation.inclinoType !=  SteerConfig::InclinoType::None || initialisation.imuType != SteerConfig::ImuType::None ) ) {
          uint8_t data[10] = {0};
          data[0] = 0x7F;
          data[1] = 0xEE;

          {
            uint16_t heading;

            if( initialisation.imuType != SteerConfig::ImuType::None ) {
              heading = ( float )steerImuInclinometerData.heading * 16;
            } else {
              heading = 9999;
            }

            data[4] = heading >> 8;
            data[5] = heading;
          }

          {
            uint16_t roll;

            if( initialisation.inclinoType != SteerConfig::InclinoType::None ) {
              roll = steerImuInclinometerData.roll * 16;
            } else {
              roll = 9999;
            }

            data[6] = roll >> 8;
            data[7] = roll;
          }

          udpSendFrom.broadcastTo( data, sizeof( data ), initialisation.portSendTo );
        }
      }

      {
        switch( steerConfig.outputType ) {
          case SteerConfig::OutputType::SteeringMotorIBT2: {
            Control* labelStatusOutputHandle = ESPUI.getControl( labelStatusOutput );
            String str;
            str.reserve( 30 );
            str = "IBT2 Motor, SetPoint: ";
            str += ( float )steerSetpoints.requestedSteerAngle;
            str += "°, timeout: ";
            str += ( bool )( steerSetpoints.lastPacketReceived < timeoutPoint );
            str += ", enabled: ";
            str += ( bool )steerSetpoints.enabled;
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
            str += "°, timeout: ";
            str += ( bool )( steerSetpoints.lastPacketReceived < timeoutPoint );
            str += ", enabled: ";
            str += ( bool )steerSetpoints.enabled;
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
            str += "°, timeout: ";
            str += ( bool )( steerSetpoints.lastPacketReceived < timeoutPoint );
            str += ", enabled: ";
            str += ( bool )steerSetpoints.enabled;
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
            str += "°, timeout: ";
            str += ( bool )( steerSetpoints.lastPacketReceived < timeoutPoint );
            str += ", enabled: ";
            str += ( bool )steerSetpoints.enabled;
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
            str += "°, timeout: ";
            str += ( bool )( steerSetpoints.lastPacketReceived < timeoutPoint );
            str += ", enabled: ";
            str += ( bool )steerSetpoints.enabled;
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
    } else {
        steerState = true;
    }
}

void IRAM_ATTR steeringWheelIsr() {
    // interrupt service routine for the steering wheel
    if( steeringPulseCount == 0 ){
        steeringWheelActivityMillis = millis();
    }
    steeringPulseCount += 1;
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
        uint16_t pgn = data[1] + ( data[0] << 8 );

        // see pgn.xlsx in https://github.com/farmerbriantee/AgOpenGPS/tree/master/AgOpenGPS_Dev
        switch( pgn ) {
          case 0x7FFE: {
            steerSetpoints.relais = data[2];
            steerSetpoints.speed = ( float )data[3] / 4;
            steerSetpoints.distanceFromLine = data[5] + ( data[4] << 8 );
            steerSetpoints.requestedSteerAngle = ( int16_t )( data[7] + ( data[6] << 8 ) ) / 100;

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

    // if no inclinometer is configured, try to receive the value from the net
    if( initialisation.inclinoType == SteerConfig::InclinoType::None ) {
      if( udpRemotePort.listen( initialisation.portSendTo ) ) {
        udpRemotePort.onPacket( []( AsyncUDPPacket packet ) {
          uint8_t* data = packet.data();
          uint16_t pgn = data[1] + ( data[0] << 8 );

          // see pgn.xlsx in https://github.com/farmerbriantee/AgOpenGPS/tree/master/AgOpenGPS_Dev
          switch( pgn ) {
            case 0x7FFD: {
              uint16_t rollInteger = data[7] + ( data[6 << 8] );

              if( rollInteger != 9999 ) {
                steerSetpoints.receivedRoll = ( float )rollInteger / 16;
              }
            }
            break;

            case 0x7FEE: {
              uint16_t rollInteger = data[7] + ( data[6 << 8] );

              if( rollInteger != 9999 ) {
                steerSetpoints.receivedRoll = ( float )rollInteger / 16;
              }
            }
            break;

            default:
              break;
          }
        } );
      }
    }
  }

  // init output
  {
    if( steerConfig.gpioPwm != SteerConfig::Gpio::None ) {
      pinMode( ( uint8_t )steerConfig.gpioPwm, OUTPUT );
      ledcSetup( 0, 1000/*steerConfig.pwmFrequency*/, 8 );
      ledcAttachPin( ( uint8_t )steerConfig.gpioPwm, 0 );
      ledcWrite( 0, 0 );
    }

    if( steerConfig.gpioDir != SteerConfig::Gpio::None ) {
      pinMode( ( uint8_t )steerConfig.gpioDir, OUTPUT );
      ledcSetup( 0, 1000/*steerConfig.pwmFrequency*/, 8 );
      ledcAttachPin( ( uint8_t )steerConfig.gpioDir, 1 );
      ledcWrite( 1, 0 );
    }

    if( steerConfig.gpioEn != SteerConfig::Gpio::None ) {
      pinMode( ( uint8_t )steerConfig.gpioEn, OUTPUT );
      digitalWrite( ( uint8_t )steerConfig.gpioEn, LOW );
    }

    if( steerConfig.gpioSteerLED != SteerConfig::Gpio::None ) {
      pinMode( ( uint8_t )steerConfig.gpioSteerLED, OUTPUT );
      digitalWrite( ( uint8_t )steerConfig.gpioSteerLED, LOW );
    }

    switch( steerConfig.outputType ) {
      case SteerConfig::OutputType::SteeringMotorIBT2: {
        Control* labelStatusOutputHandle = ESPUI.getControl( labelStatusOutput );

        if( steerConfig.gpioPwm != SteerConfig::Gpio::None &&
            steerConfig.gpioDir != SteerConfig::Gpio::None &&
            steerConfig.gpioEn  != SteerConfig::Gpio::None ) {
          labelStatusOutputHandle->value = "Output configured";
          labelStatusOutputHandle->color = ControlColor::Emerald;
          ESPUI.updateControlAsync( labelStatusOutputHandle );

          initialisation.outputType = SteerConfig::OutputType::SteeringMotorIBT2;
        } else {
          {
            labelStatusOutputHandle->value = "GPIOs not correctly defined";
            labelStatusOutputHandle->color = ControlColor::Carrot;
            ESPUI.updateControlAsync( labelStatusOutputHandle );
          }
        }
      }
      break;

      case SteerConfig::OutputType::SteeringMotorCytron: {
        Control* labelStatusOutputHandle = ESPUI.getControl( labelStatusOutput );

        if( steerConfig.gpioPwm != SteerConfig::Gpio::None &&
            steerConfig.gpioDir != SteerConfig::Gpio::None ) {
          labelStatusOutputHandle->value = "Output configured";
          labelStatusOutputHandle->color = ControlColor::Emerald;
          ESPUI.updateControlAsync( labelStatusOutputHandle );

          initialisation.outputType = SteerConfig::OutputType::SteeringMotorCytron;
        } else {
          {
            labelStatusOutputHandle->value = "GPIOs not correctly defined";
            labelStatusOutputHandle->color = ControlColor::Carrot;
            ESPUI.updateControlAsync( labelStatusOutputHandle );
          }
        }
      }
      break;

      case SteerConfig::OutputType::HydraulicPwm2Coil: {
        Control* labelStatusOutputHandle = ESPUI.getControl( labelStatusOutput );

        if( steerConfig.gpioPwm != SteerConfig::Gpio::None &&
            steerConfig.gpioDir != SteerConfig::Gpio::None ) {
          labelStatusOutputHandle->value = "Output configured";
          labelStatusOutputHandle->color = ControlColor::Emerald;
          ESPUI.updateControlAsync( labelStatusOutputHandle );

          initialisation.outputType = SteerConfig::OutputType::HydraulicPwm2Coil;
        } else {
          {
            labelStatusOutputHandle->value = "GPIOs not correctly defined";
            labelStatusOutputHandle->color = ControlColor::Carrot;
            ESPUI.updateControlAsync( labelStatusOutputHandle );
          }
        }
      }
      break;

      case SteerConfig::OutputType::HydraulicDanfoss: {
        Control* labelStatusOutputHandle = ESPUI.getControl( labelStatusOutput );

        if( steerConfig.gpioPwm != SteerConfig::Gpio::None &&
            steerConfig.gpioEn != SteerConfig::Gpio::None ) {
          labelStatusOutputHandle->value = "Output configured";
          labelStatusOutputHandle->color = ControlColor::Emerald;
          ESPUI.updateControlAsync( labelStatusOutputHandle );

          initialisation.outputType = SteerConfig::OutputType::HydraulicDanfoss;
        } else {
          {
            labelStatusOutputHandle->value = "GPIOs not correctly defined";
            labelStatusOutputHandle->color = ControlColor::Carrot;
            ESPUI.updateControlAsync( labelStatusOutputHandle );
          }
        }
      }
      break;

      case SteerConfig::OutputType::HydraulicBangBang: {
        Control* labelStatusOutputHandle = ESPUI.getControl( labelStatusOutput );

        if( steerConfig.gpioPwm != SteerConfig::Gpio::None &&
            steerConfig.gpioEn != SteerConfig::Gpio::None ) {
          labelStatusOutputHandle->value = "Output configured";
          labelStatusOutputHandle->color = ControlColor::Emerald;
          ESPUI.updateControlAsync( labelStatusOutputHandle );

          initialisation.outputType = SteerConfig::OutputType::HydraulicBangBang;
        } else {
          {
            labelStatusOutputHandle->value = "GPIOs not correctly defined";
            labelStatusOutputHandle->color = ControlColor::Carrot;
            ESPUI.updateControlAsync( labelStatusOutputHandle );
          }
        }
      }
      break;

      default:
        break;

    }
  }

  if( steerConfig.gpioWorkswitch != SteerConfig::Gpio::None ) {
    pinMode( ( uint8_t )steerConfig.gpioWorkswitch, INPUT_PULLUP );
  }
  if( steerConfig.gpioWorkLED != SteerConfig::Gpio::None ) {
    pinMode( ( uint8_t )steerConfig.gpioWorkLED, OUTPUT );
  }

  // use interrupt callbacks to simpify steer state tracking,
  // whichever callback happens last (steering wheel or steer switch) gets priority
  if( steerConfig.gpioSteerswitch != SteerConfig::Gpio::None ) {
    pinMode( ( uint8_t )steerConfig.gpioSteerswitch, INPUT_PULLUP );
    if( steerConfig.steerSwitchIsMomentary ){
        attachInterrupt( ( uint8_t )steerConfig.gpioSteerswitch, steerswitchMomentaryIsr, CHANGE);
    } else {
        attachInterrupt( ( uint8_t )steerConfig.gpioSteerswitch, steerswitchMaintainedIsr, CHANGE);
    }
  }

  if( steerConfig.steeringWheelEncoder != SteerConfig::Gpio::None ) {
    pinMode( ( uint8_t )steerConfig.steeringWheelEncoder, INPUT_PULLUP );
    attachInterrupt( ( uint8_t )steerConfig.steeringWheelEncoder, steeringWheelIsr, CHANGE);
  }

  xTaskCreate( autosteerWorker100Hz, "autosteerWorker", 3096, NULL, 3, NULL );
}
