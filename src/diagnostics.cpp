
#include <stdio.h>
#include <string.h>

#include "main.hpp"
#include "jsonFunctions.hpp"

void diagnosticWorker10Hz( void* z ) {
  vTaskDelay( 2000 );
  constexpr TickType_t xFrequency = 100;
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
      labelSpeedDisableAutosteerHandle->value = str;
      labelSpeedDisableAutosteerHandle->color = ( bool )disabledBySpeedSafety ? ControlColor::Alizarin : ControlColor::Emerald;
      ESPUI.updateControlAsync( labelSpeedDisableAutosteerHandle );
    }
    {
      Control* labelSupplyVoltageHandle = ESPUI.getControl( labelSupplyVoltage );
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
      Control* labelSteerMotorCurrentHandle = ESPUI.getControl( labelSteerMotorCurrent );
      String str;
      str.reserve( 30 );
      str = "Current: ";
      str += ( double ) steerMotorCurrent ;
      str += "\nOverlimit: N/A";
      labelSteerMotorCurrentHandle->value = str;
      ESPUI.updateControlAsync( labelSteerMotorCurrentHandle );
    }
    {
      Control* handle = ESPUI.getControl( labelWheelAngle );
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
      handle->value = str;
      ESPUI.updateControlAsync( handle );
    }
    {
      Control* labelSwitchStatesHandle = ESPUI.getControl( labelSwitchStates );
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
      labelSwitchStatesHandle->value = str;
      ESPUI.updateControlAsync( labelSwitchStatesHandle );
    }
    vTaskDelayUntil( &xLastWakeTime, xFrequency );
  }
}

void initDiagnostics() {
  xTaskCreate( diagnosticWorker10Hz, "diagnosticWorker", 3096, NULL, 3, NULL );
}
