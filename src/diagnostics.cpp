
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
      Control* labelSpeedDisableAutosteerHandle = ESPUI.getControl( labelSpeedDisableAutosteer );
      String str;
      str.reserve( 30 );
      str = "Speed: ";
      str += ( float )steerSetpoints.speed;
      if( ( SteerConfig::SpeedUnits )steerConfig.speedUnits == SteerConfig::SpeedUnits::MilesPerHour ) {
        str += " MPH";
      } else {
        str += " KPH";
      }
      str += "\nDisabled by max speed: ";
      str += ( bool )disabledBySpeedSafety ? "Yes" : "No" ;
      str += "\nDisabled by min speed: ";
      str += ( bool )steerSetpoints.speed < steerConfig.minAutosteerSpeed ? "Yes" : "No" ;
      labelSpeedDisableAutosteerHandle->color = ( bool )disabledBySpeedSafety ? ControlColor::Alizarin : ControlColor::Emerald;
      ESPUI.updateLabel( labelSpeedDisableAutosteer, str );
    }
    {
      String str;
      str.reserve( 30 );
      str = ( uint16_t ) steerSupplyVoltage ;
      str += " counts; ";
      str += ( double ) ( ( double ) steerSupplyVoltage * 0.003 * 0.248 ); // 3 mv per bit; 10k/3.3k = 0.248
      str += " volts\n";
      str += ( double ) ( ( double ) diagnostics.steerSupplyVoltageMin * 0.003 * 0.248 );
      str += " volts min while steering\n";
      str += ( double ) ( ( double ) diagnostics.steerSupplyVoltageMax  * 0.003 * 0.248 );
      str += " volts max while steering";
      labelSupplyVoltageHandle->value = str;
      ESPUI.updateControlAsync( labelSupplyVoltageHandle );
    }
    {
      String str;
      str.reserve( 30 );
      str = "Current: ";
      str += ( double ) steerMotorCurrent ;
      str += "\nOverlimit: N/A";
      ESPUI.updateLabel( labelSteerMotorCurrent, str );
    }
    {
      String str;
      str.reserve( 30 );
      if( steerConfig.wheelAngleSensorType == SteerConfig::WheelAngleSensorType::TieRodDisplacement ) {
        str += ( float )steerSetpoints.actualSteerAngle;
        str += "°, Raw ";
        str += ( float )steerSetpoints.wheelAngleRaw;
        str += "°, Displacement ";
        str += ( float )steerSetpoints.wheelAngleCurrentDisplacement;
        str += "mm";
      } else {
        str += "A/D count: ";
        str += ( int )steerSetpoints.wheelAngleCounts;
        str += ", Raw: ";
        str += ( float )steerSetpoints.wheelAngleRaw;
        str += "°\nActual: ";
        str += ( float )steerSetpoints.actualSteerAngle;
        str += "°, SetPoint: ";
        str += ( float )steerSetpoints.requestedSteerAngle;
        str += "°";
      }
    }
    {
      String str;
      str.reserve( 30 );
      str = "Autosteer switch: ";
      str += ( bool )( digitalRead( steerConfig.gpioSteerswitch ) == steerConfig.steerswitchActiveLow ) ? "Off" : "On" ;
      str += "\nWork switch: ";
      str += ( bool )( digitalRead( steerConfig.gpioWorkswitch ) == steerConfig.workswitchActiveLow ) ? "Off" : "On" ;
      if( steerConfig.disengageSwitchType == SteerConfig::DisengageSwitchType::Encoder ){
        str += "\nDisengage on steering wheel: ";
        str += ( bool )digitalRead( steerConfig.gpioDisengage ) ? "Off" : "On" ;
      }
      else if( steerConfig.disengageSwitchType == SteerConfig::DisengageSwitchType::Hydraulic ){
        str += "\nHydraulic disengage switch: ";
        str += ( bool )( digitalRead( steerConfig.gpioDisengage ) == steerConfig.hydraulicSwitchActiveLow ) ? "Off" : "On" ;
      }
      ESPUI.updateLabel( labelSwitchStates, str );
    }
    {
      String str;
      str.reserve( 30 );
      str = "ADS1115 initialized, ";
      if( steerConfig.adsGain == SteerConfig::ADSGain::GAIN_TWOTHIRDS ){
        str += " 6.144 volts max,\n";
        str += ( float )( steerSetpoints.wheelAngleCounts / 65536 ) * 6.144;
        str += " volts from WAS, no divider";
      }
      else if( steerConfig.adsGain == SteerConfig::ADSGain::GAIN_ONE ){
        str += " 4.096 volts max,\n";
        str += ( steerSetpoints.wheelAngleCounts / 65536 ) * ( 4.096 * 0.629 ); // Sensor - 3.3K - ADS - 5.6K - Gnd
        str += " volts from WAS, 3.3k-5.6k divider";
      }
      else if( steerConfig.adsGain == SteerConfig::ADSGain::GAIN_TWO ){
        str += " 2.048 volts max";
      }
      else if( steerConfig.adsGain == SteerConfig::ADSGain::GAIN_FOUR ){
        str += " 1.024 volts max";
      }
      else if( steerConfig.adsGain == SteerConfig::ADSGain::GAIN_EIGHT ){
        str += " 0.512 volts max";
      }
      else if( steerConfig.adsGain == SteerConfig::ADSGain::GAIN_SIXTEEN ){
        str += " 0.256 volts max";
      }
      handle->value = str;
      handle->color = ControlColor::Emerald;
      initialisation.wheelAngleInput = steerConfig.wheelAngleInput;
      ESPUI.updateControlAsync( handle );
    }
    vTaskDelayUntil( &xLastWakeTime, xFrequency );
  }
}

void initDiagnostics() {
  xTaskCreate( diagnosticWorker1Hz, "diagnosticWorker", 3096, NULL, 3, NULL );
}
