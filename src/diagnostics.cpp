
#include <stdio.h>
#include <string.h>

#include "main.hpp"
#include "jsonFunctions.hpp"

void diagnosticWorker1Hz( void* z ) {
  vTaskDelay( 2000 );
  constexpr TickType_t xFrequency = 1000;
  TickType_t xLastWakeTime = xTaskGetTickCount();
  for( ;; ) {

    {
      Control* labelSafetyDisableAutosteerHandle = ESPUI.getControl( labelSafetyDisableAutosteer );
      String str;
      str.reserve( 30 );
      str = "Speed: ";
      str += ( float )steerSetpoints.speed;
      if( ( SteerConfig::SpeedUnits )steerConfig.speedUnits == SteerConfig::SpeedUnits::MilesPerHour ) {
        str += " MPH";
      } else {
        str += " KPH";
      }
      str += "\nEnable autosteer timed out: ";
      str += ( bool )safety.AOGEnableAutosteerTimeout ? "Yes" : "No" ;
      str += "\nDisabled by max speed: ";
      str += ( bool )safety.autosteerDisabledByMaxEngageSpeed ? "Yes" : "No" ;
      str += "\nDisabled by min speed: ";
      str += ( bool )steerSetpoints.speed < steerConfig.minAutosteerSpeed ? "Yes" : "No" ;
      str += "\nDisabled by steering wheel: ";
      str += ( bool )machine.disengagedBySteeringWheel ? "Yes" : "No" ;
      if( safety.AOGEnableAutosteerTimeout || 
          safety.autosteerDisabledByMaxEngageSpeed || 
          machine.disengagedBySteeringWheel ){
        labelSafetyDisableAutosteerHandle->color = ControlColor::Alizarin;
      } else {
        labelSafetyDisableAutosteerHandle->color = ControlColor::Emerald;
      }
      ESPUI.updateLabel( labelSafetyDisableAutosteer, str );
    }
    {
      String str;
      str.reserve( 30 );
      str = ( uint16_t ) machine.steerSupplyVoltage ;
      str += " counts; ";
      if( steerConfig.adsGain == SteerConfig::ADSGain::GAIN_TWOTHIRDS ){
        str += ( double ) ( ( double ) ( machine.steerSupplyVoltage * 3.03 ) * 0.0001875 ); // .1875 mv per bit; 10k/3.3k = 3.03
        str += " volts\n";
        str += ( double ) ( ( double ) ( diagnostics.steerSupplyVoltageMin * 3.03 ) * 0.0001875 );
        str += " volts min while steering\n";
        str += ( double ) ( ( double ) ( diagnostics.steerSupplyVoltageMax * 3.03 ) * 0.0001875 );
        str += " volts max while steering";
      } else if( steerConfig.adsGain == SteerConfig::ADSGain::GAIN_ONE ){
        str += ( double ) ( ( double ) ( machine.steerSupplyVoltage * 4.545 ) * 0.000125 ); // .125 mv per bit; 10k/2.2k = 4.545
        str += " volts\n";
        str += ( double ) ( ( double ) ( diagnostics.steerSupplyVoltageMin * 4.545 ) * 0.000125 );
        str += " volts min while steering\n";
        str += ( double ) ( ( double ) ( diagnostics.steerSupplyVoltageMax * 4.545 ) * 0.000125 );
        str += " volts max while steering";
      } else {
        str += "N/A";
      }
      ESPUI.updateLabel( labelSupplyVoltage, str );
    }
    {
      String str;
      str.reserve( 30 );
      str = "Current: ";
      str += ( uint16_t ) machine.steerMotorCurrent;
      str += "\nOverlimit: ";
      str += ( machine.steerMotorCurrent > steerConfig.maxSteerCurrent ) ? "Yes" : "No";
      ESPUI.updateLabel( labelSteerMotorCurrent, str );
    }
    {
      String str;
      str.reserve( 30 );
      if( steerConfig.wheelAngleInput == SteerConfig::AnalogIn::CanbusValtraMasseyChallenger ){
        str = "Canbus WAS: ";
        str += ( float )steerSetpoints.actualSteerAngle;
        str += "°";
      } else if( steerConfig.wheelAngleSensorType == SteerConfig::WheelAngleSensorType::TieRodDisplacement ) {
        str += ( float )steerSetpoints.actualSteerAngle;
        str += "°, Raw ";
        str += ( float )steerSetpoints.wheelAngleRaw;
        str += "°, Displacement ";
        str += ( float )steerSetpoints.wheelAngleCurrentDisplacement;
        str += "mm\n";
      } else {
        str += "A/D count: ";
        str += ( int )steerSetpoints.wheelAngleCounts;
        str += ", Raw: ";
        str += ( float )steerSetpoints.wheelAngleRaw;
        str += "°\nActual: ";
        str += ( float )steerSetpoints.actualSteerAngle;
        str += "°, SetPoint: ";
        str += ( float )steerSetpoints.requestedSteerAngle;
        str += "°\n";
      }
      if( steerConfig.adsGain == SteerConfig::ADSGain::GAIN_TWOTHIRDS ){
        str += ( float )( steerSetpoints.wheelAngleCounts * 0.0001875 );
        str += " volts from WAS";
      }
      else if( steerConfig.adsGain == SteerConfig::ADSGain::GAIN_ONE ){
        str += ( float )( steerSetpoints.wheelAngleCounts * 1.589 * 0.000125 ); // Sensor - 3.3K - ADS - 5.6K - Gnd
        str += " volts from WAS";
      }
      ESPUI.updateLabel( labelWheelAngle, str );
    }
    {
      String str;
      str.reserve( 30 );
      if( steerConfig.steerSwitchIsMomentary == true ){
        str = "Momentary steer switch: ";
      } else {
        str = "Maintained steer switch: ";
      }
      str += ( bool )( digitalRead( steerConfig.gpioSteerswitch ) == steerConfig.steerswitchActiveLow ) ? "Off" : "On" ;
      if( steerConfig.workswitchType > SteerConfig::WorkswitchType::Gpio ){ // Canbus function
        str += "\nCanbus work function: ";
        str += ( bool ) workswitchState ? "Off" : "On" ;
      } else {
        str += "\nWork switch: ";
        str += ( bool )( digitalRead( steerConfig.gpioWorkswitch ) == steerConfig.workswitchActiveLow ) ? "Off" : "On" ;
      }
      switch( steerConfig.disengageSwitchType ) {
        case SteerConfig::DisengageSwitchType::Encoder: {
          str += "\nEncoder on steering wheel: ";
          str += ( bool )digitalRead( steerConfig.gpioDisengage ) ? "Off" : "On" ;
          str += " / ";
          str += machine.handwheelPulseCount;
          str += " counts";
        }
        break;

        case SteerConfig::DisengageSwitchType::Hydraulic: {
          str += "\nHydraulic disengage switch: ";
          str += ( bool )( digitalRead( steerConfig.gpioDisengage ) == steerConfig.hydraulicSwitchActiveLow ) ? "Off" : "On" ;
        }
        break;

        case SteerConfig::DisengageSwitchType::JDVariableDuty: {
          str += "\nDeere variable duty encoder: ";
          str += ( uint16_t )( abs( machine.DeereDutyAverage - machine.DeereDutyCycle ) );
        }
      }
      ESPUI.updateLabel( labelSwitchStates, str );
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
          str += ( bool )( steerSetpoints.lastPacketReceived < safety.timeoutPoint ) ? "Yes" : "No" ;
          str += ", enabled: ";
          str += ( bool )steerSetpoints.enabled ? "Yes" : "No" ;
          str += ", output: ";
          str += ( uint8_t )machine.valveOutput ;
          labelStatusOutputHandle->color = ControlColor::Emerald;
          ESPUI.updateLabel( labelStatusOutput, str );
        }
        break;

        case SteerConfig::OutputType::SteeringMotorCytron: {
          Control* labelStatusOutputHandle = ESPUI.getControl( labelStatusOutput );
          String str;
          str.reserve( 30 );
          str = "Cytron Motor, SetPoint: ";
          str += ( float )steerSetpoints.requestedSteerAngle;
          str += "°\ntimeout: ";
          str += ( bool )( steerSetpoints.lastPacketReceived < safety.timeoutPoint ) ? "Yes" : "No" ;
          str += ", enabled: ";
          str += ( bool )steerSetpoints.enabled ? "Yes" : "No" ;
          str += ", output: ";
          str += ( uint8_t )machine.valveOutput;
          labelStatusOutputHandle->color = ControlColor::Emerald;
          ESPUI.updateLabel( labelStatusOutput, str );
        }
        break;

        case SteerConfig::OutputType::HydraulicPwm2Coil: {
          Control* labelStatusOutputHandle = ESPUI.getControl( labelStatusOutput );
          String str;
          str.reserve( 30 );
          str = "IBT2 Hydraulic PWM 2 Coil, SetPoint: ";
          str += ( float )steerSetpoints.requestedSteerAngle;
          str += "°,\ntimeout: ";
          str += ( bool )( steerSetpoints.lastPacketReceived < safety.timeoutPoint ) ? "Yes" : "No" ;
          str += ", enabled: ";
          str += ( bool )steerSetpoints.enabled ? "Yes" : "No" ;
          str += ",\n output: ";
          str += ( uint8_t )machine.valveOutput ;
          str += ", dither: ";
          str += ( float )ditherAmount ;
          labelStatusOutputHandle->color = ControlColor::Emerald;
          ESPUI.updateLabel( labelStatusOutput, str );
        }
        break;

        case SteerConfig::OutputType::HydraulicDanfoss: {
          Control* labelStatusOutputHandle = ESPUI.getControl( labelStatusOutput );
          String str;
          str.reserve( 30 );
          str = "IBT2 Hydraulic Danfoss, SetPoint: ";
          str += ( float )steerSetpoints.requestedSteerAngle;
          str += "°,\ntimeout: ";
          str += ( bool )( steerSetpoints.lastPacketReceived < safety.timeoutPoint ) ? "Yes" : "No" ;
          str += ", enabled: ";
          str += ( bool )steerSetpoints.enabled ? "Yes" : "No" ;
          str += ", output: ";
          str += ( uint8_t )machine.valveOutput ;
          labelStatusOutputHandle->color = ControlColor::Emerald;
          ESPUI.updateLabel( labelStatusOutput, str );
        }
        break;

        case SteerConfig::OutputType::HydraulicBangBang: {
          Control* labelStatusOutputHandle = ESPUI.getControl( labelStatusOutput );
          String str;
          str.reserve( 30 );
          str = "IBT2 Hydraulic Bang Bang, SetPoint: ";
          str += ( float )steerSetpoints.requestedSteerAngle;
          str += "°,\ntimeout: ";
          str += ( bool )( steerSetpoints.lastPacketReceived < safety.timeoutPoint ) ? "Yes" : "No" ;
          str += ", enabled: ";
          str += ( bool )steerSetpoints.enabled ? "Yes" : "No" ;
          str += ", output: ";
          str += ( uint8_t )machine.valveOutput ;
          labelStatusOutputHandle->color = ControlColor::Emerald;
          ESPUI.updateLabel( labelStatusOutput, str );
        }
        break;

        default:
          break;

        }
    }
    vTaskDelayUntil( &xLastWakeTime, xFrequency );
  }
}

void initDiagnostics() {
  xTaskCreate( diagnosticWorker1Hz, "diagnosticWorker", 3096, NULL, 3, NULL );
}
