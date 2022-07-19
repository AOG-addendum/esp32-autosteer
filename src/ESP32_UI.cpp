
#include <stdio.h>

#include <ESPUI.h>

#include "main.hpp"
#include "jsonFunctions.hpp"

SteerConfig steerConfig, steerConfigDefaults;

int8_t ditherAmount = 0;
uint16_t labelLoad;
uint16_t labelWheelAngle;
uint16_t buttonReset;

uint16_t labelWheelAngleDisplacement;

uint16_t labelStatusOutput;
uint16_t labelStatusAdc;
uint16_t labelStatusCan;
uint16_t labelBuildDate;
uint16_t labelStatusSafety;

void setResetButtonToRed() {
  ESPUI.getControl( buttonReset )->color = ControlColor::Alizarin;
  ESPUI.updateControlAsync( buttonReset );
}

void addAnalogInputADS1115( uint16_t parent ) {
  ESPUI.addControl( ControlType::Option, "ADS1115 A0 Single", String( ( uint8_t )SteerConfig::AnalogIn::ADS1115A0Single ), ControlColor::Alizarin, parent );
  ESPUI.addControl( ControlType::Option, "ADS1115 A1 Single", String( ( uint8_t )SteerConfig::AnalogIn::ADS1115A1Single ), ControlColor::Alizarin, parent );
  ESPUI.addControl( ControlType::Option, "ADS1115 A0 Differential", String( ( uint8_t )SteerConfig::AnalogIn::ADS1115A0A1Differential ), ControlColor::Alizarin, parent );}

void initESPUI ( void ) {

  labelLoad = ESPUI.addControl( ControlType::Label, "Load:", "", ControlColor::Turquoise );
  labelWheelAngle = ESPUI.addControl( ControlType::Label, "Wheel Angle:", "0°", ControlColor::Emerald );

  buttonReset = ESPUI.addControl( ControlType::Button, "Store the Settings", "Apply", ControlColor::Emerald, Control::noParent,
  []( Control * control, int id ) {
    if( id == B_UP ) {
      saveConfig();
    }
  } );

  buttonReset = ESPUI.addControl( ControlType::Button, "If this turns red, you have to", "Apply & Reboot", ControlColor::Emerald, Control::noParent,
  []( Control * control, int id ) {
    if( id == B_UP ) {
      saveConfig();
      SPIFFS.end();
      ESP.restart();
    }
  } );

  uint16_t tabConfigurations;

  // Status Tab
  {
    uint16_t tab = ESPUI.addControl( ControlType::Tab, "Status", "Status" );

    labelStatusOutput = ESPUI.addControl( ControlType::Label, "Output:", "No Output configured", ControlColor::Turquoise, tab );
    labelStatusAdc = ESPUI.addControl( ControlType::Label, "ADC:", "No ADC configured", ControlColor::Turquoise, tab );
    labelStatusCan = ESPUI.addControl( ControlType::Label, "CAN:", "No CAN BUS configured", ControlColor::Turquoise, tab );
    String buildDate = String(__DATE__);
    buildDate += String(" ");
    buildDate += String(__TIME__);
    labelBuildDate = ESPUI.addControl( ControlType::Label, "Build date :", buildDate, ControlColor::Turquoise, tab );
    labelStatusSafety = ESPUI.addControl( ControlType::Label, "Speed Safety:", "Not started", ControlColor::Turquoise, tab );
  }

  // Network Tab
  {
    uint16_t tab = ESPUI.addControl( ControlType::Tab, "Network", "Network" );

    {
      uint16_t sel = ESPUI.addControl( ControlType::Select, "Mode*", String( ( int )steerConfig.mode ), ControlColor::Wetasphalt, tab,
      []( Control * control, int id ) {
        steerConfig.mode = ( SteerConfig::Mode )control->value.toInt();
        setResetButtonToRed();
      } );
      ESPUI.addControl( ControlType::Option, "QtOpenGuidance", "0", ControlColor::Alizarin, sel );
      ESPUI.addControl( ControlType::Option, "AgOpenGps", "1", ControlColor::Alizarin, sel );
    }

    {
      uint16_t baudrate = ESPUI.addControl( ControlType::Select, "Baudrate Serial", String( steerConfig.baudrate ), ControlColor::Peterriver, tab,
      []( Control * control, int id ) {
        uint32_t baudrate = control->value.toInt();
        steerConfig.baudrate = baudrate;
        Serial.updateBaudRate( baudrate );
      } );
      ESPUI.addControl( ControlType::Option, "4800", "4800", ControlColor::Alizarin, baudrate );
      ESPUI.addControl( ControlType::Option, "9600", "9600", ControlColor::Alizarin, baudrate );
      ESPUI.addControl( ControlType::Option, "19200", "19200", ControlColor::Alizarin, baudrate );
      ESPUI.addControl( ControlType::Option, "38400", "38400", ControlColor::Alizarin, baudrate );
      ESPUI.addControl( ControlType::Option, "57600", "57600", ControlColor::Alizarin, baudrate );
      ESPUI.addControl( ControlType::Option, "115200", "115200", ControlColor::Alizarin, baudrate );
      ESPUI.addControl( ControlType::Option, "230400", "230400", ControlColor::Alizarin, baudrate );
      ESPUI.addControl( ControlType::Option, "460800", "460800", ControlColor::Alizarin, baudrate );
      ESPUI.addControl( ControlType::Option, "921600", "921600", ControlColor::Alizarin, baudrate );
    }

    ESPUI.addControl( ControlType::Text, "SSID*", String( steerConfig.ssid ), ControlColor::Wetasphalt, tab,
    []( Control * control, int id ) {
      control->value.toCharArray( steerConfig.ssid, sizeof( steerConfig.ssid ) );
      setResetButtonToRed();
    } );
    ESPUI.addControl( ControlType::Text, "Password*", String( steerConfig.password ), ControlColor::Wetasphalt, tab,
    []( Control * control, int id ) {
      control->value.toCharArray( steerConfig.password, sizeof( steerConfig.password ) );
      setResetButtonToRed();
    } );
    ESPUI.addControl( ControlType::Text, "Hostname*", String( steerConfig.hostname ), ControlColor::Wetasphalt, tab,
    []( Control * control, int id ) {
      control->value.toCharArray( steerConfig.hostname, sizeof( steerConfig.hostname ) );
      setResetButtonToRed();
    } );

    ESPUI.addControl( ControlType::Switcher, "OTA Enabled*", steerConfig.enableOTA ? "1" : "0", ControlColor::Wetasphalt, tab,
    []( Control * control, int id ) {
      steerConfig.enableOTA = control->value.toInt() == 1;
      setResetButtonToRed();
    } );

    if( steerConfig.mode == SteerConfig::Mode::QtOpenGuidance ) {
      ESPUI.addControl( ControlType::Number, "Port to send to*", String( steerConfig.qogPortSendTo ), ControlColor::Wetasphalt, tab,
      []( Control * control, int id ) {
        steerConfig.qogPortSendTo = control->value.toInt();
        setResetButtonToRed();
      } );
      ESPUI.addControl( ControlType::Number, "Port to listen to*", String( steerConfig.qogPortListenTo ), ControlColor::Wetasphalt, tab,
      []( Control * control, int id ) {
        steerConfig.qogPortListenTo = control->value.toInt();
        setResetButtonToRed();
      } );
    }

    if( steerConfig.mode == SteerConfig::Mode::AgOpenGps ) {
      ESPUI.addControl( ControlType::Number, "Port to send from*", String( steerConfig.aogPortSendFrom ), ControlColor::Wetasphalt, tab,
      []( Control * control, int id ) {
        steerConfig.aogPortSendFrom = control->value.toInt();
        setResetButtonToRed();
      } );

      ESPUI.addControl( ControlType::Number, "Port to send to*", String( steerConfig.aogPortSendTo ), ControlColor::Wetasphalt, tab,
      []( Control * control, int id ) {
        steerConfig.aogPortSendTo = control->value.toInt();
        setResetButtonToRed();
      } );
      ESPUI.addControl( ControlType::Number, "Port to listen to*", String( steerConfig.aogPortListenTo ), ControlColor::Wetasphalt, tab,
      []( Control * control, int id ) {
        steerConfig.aogPortListenTo = control->value.toInt();
        setResetButtonToRed();
      } );
    }
  }

  // CAN Bus
  {
    uint16_t tab = ESPUI.addControl( ControlType::Tab, "CAN Bus/J1939", "CAN Bus/J1939" );

    ESPUI.addControl( ControlType::Switcher, "CAN Bus Enabled*", steerConfig.canBusEnabled ? "1" : "0", ControlColor::Wetasphalt, tab,
    []( Control * control, int id ) {
      steerConfig.canBusEnabled = control->value.toInt() == 1;
      setResetButtonToRed();
    } );

    {
      uint16_t sel = ESPUI.addControl( ControlType::Select, "Bus Speed*", String( ( int )steerConfig.canBusSpeed ), ControlColor::Wetasphalt, tab,
      []( Control * control, int id ) {
        steerConfig.canBusSpeed = ( SteerConfig::CanBusSpeed )control->value.toInt();
        setResetButtonToRed();
      } );
      ESPUI.addControl( ControlType::Option, "250kB/s", "250", ControlColor::Alizarin, sel );
      ESPUI.addControl( ControlType::Option, "500kB/s", "500", ControlColor::Alizarin, sel );
    }
  }

  // Switches/Buttons Tab
  {
    uint16_t tab = ESPUI.addControl( ControlType::Tab, "Work- and Steerswitch", "Work- and Steerswitch" );

    uint16_t sel = ESPUI.addControl( ControlType::Select, "Workswitch Type", String( ( int )steerConfig.workswitchType ), ControlColor::Peterriver, tab,
      []( Control * control, int id ) {
        steerConfig.workswitchType = ( SteerConfig::WorkswitchType )control->value.toInt();
      } );
      ESPUI.addControl( ControlType::Option, "None", "0", ControlColor::Alizarin, sel );
      ESPUI.addControl( ControlType::Option, "Gpio", "1", ControlColor::Alizarin, sel );

    if( steerConfig.canBusEnabled ) {
      {
        ESPUI.addControl( ControlType::Option, "Rear Hitch Position (from Can Bus)", "2", ControlColor::Alizarin, sel );
        ESPUI.addControl( ControlType::Option, "Front Hitch Position (from Can Bus)", "3", ControlColor::Alizarin, sel );
        ESPUI.addControl( ControlType::Option, "Rear Pto Rpm (from Can Bus)", "4", ControlColor::Alizarin, sel );
        ESPUI.addControl( ControlType::Option, "Front Pto Rpm (from Can Bus)", "5", ControlColor::Alizarin, sel );
        ESPUI.addControl( ControlType::Option, "Motor Rpm (from Can Bus)", "6", ControlColor::Alizarin, sel );
      }
      {
        uint16_t num = ESPUI.addControl( ControlType::Number, "Hitch Threshold", String( steerConfig.canBusHitchThreshold ), ControlColor::Peterriver, tab,
        []( Control * control, int id ) {
          steerConfig.canBusHitchThreshold = control->value.toInt();
        } );
        ESPUI.addControl( ControlType::Min, "Min", "0", ControlColor::Peterriver, num );
        ESPUI.addControl( ControlType::Max, "Max", "100", ControlColor::Peterriver, num );
        ESPUI.addControl( ControlType::Step, "Step", "1", ControlColor::Peterriver, num );
      }
      {
        uint16_t num = ESPUI.addControl( ControlType::Number, "Hitch Threshold Hysteresis", String( steerConfig.canBusHitchThresholdHysteresis ), ControlColor::Peterriver, tab,
        []( Control * control, int id ) {
          steerConfig.canBusHitchThresholdHysteresis = control->value.toInt();
        } );
        ESPUI.addControl( ControlType::Min, "Min", "0", ControlColor::Peterriver, num );
        ESPUI.addControl( ControlType::Max, "Max", "100", ControlColor::Peterriver, num );
        ESPUI.addControl( ControlType::Step, "Step", "1", ControlColor::Peterriver, num );
      }

      {
        uint16_t num = ESPUI.addControl( ControlType::Number, "RPM Threshold", String( steerConfig.canBusHitchThreshold ), ControlColor::Peterriver, tab,
        []( Control * control, int id ) {
          steerConfig.canBusHitchThreshold = control->value.toInt();
        } );
        ESPUI.addControl( ControlType::Min, "Min", "0", ControlColor::Peterriver, num );
        ESPUI.addControl( ControlType::Max, "Max", "3500", ControlColor::Peterriver, num );
        ESPUI.addControl( ControlType::Step, "Step", "1", ControlColor::Peterriver, num );
      }
      {
        uint16_t num = ESPUI.addControl( ControlType::Number, "RPM Threshold Hysteresis", String( steerConfig.canBusHitchThresholdHysteresis ), ControlColor::Peterriver, tab,
        []( Control * control, int id ) {
          steerConfig.canBusHitchThresholdHysteresis = control->value.toInt();
        } );
        ESPUI.addControl( ControlType::Min, "Min", "0", ControlColor::Peterriver, num );
        ESPUI.addControl( ControlType::Max, "Max", "1000", ControlColor::Peterriver, num );
        ESPUI.addControl( ControlType::Step, "Step", "1", ControlColor::Peterriver, num );
      }
    }

    {
      ESPUI.addControl( ControlType::Switcher, "Workswitch Active Low", steerConfig.workswitchActiveLow ? "1" : "0", ControlColor::Peterriver, tab,
      []( Control * control, int id ) {
        steerConfig.workswitchActiveLow = control->value.toInt() == 1;
      } );
    }

    {
      ESPUI.addControl( ControlType::Switcher, "Autosteer Switch Active Low", steerConfig.steerswitchActiveLow ? "1" : "0", ControlColor::Peterriver, tab,
      []( Control * control, int id ) {
        steerConfig.steerswitchActiveLow = control->value.toInt() == 1;
      } );
    }

    {
      ESPUI.addControl( ControlType::Switcher, "Autosteer Switch is Momentary*", steerConfig.steerSwitchIsMomentary ? "1" : "0", ControlColor::Wetasphalt, tab,
      []( Control * control, int id ) {
        steerConfig.steerSwitchIsMomentary = control->value.toInt() == 1;
        setResetButtonToRed();
      } );
    }

    {
      uint16_t num = ESPUI.addControl( ControlType::Number, "Steering Wheel Pulses per Frame", String( steerConfig.steeringWheelFramePulses ), ControlColor::Peterriver, tab,
      []( Control * control, int id ) {
        steerConfig.steeringWheelFramePulses = control->value.toInt();
      } );
      ESPUI.addControl( ControlType::Min, "Min", "1", ControlColor::Peterriver, num );
      ESPUI.addControl( ControlType::Max, "Max", "1000", ControlColor::Peterriver, num );
      ESPUI.addControl( ControlType::Step, "Step", "10", ControlColor::Peterriver, num );
  }

    {
      uint16_t num = ESPUI.addControl( ControlType::Number, "Steering Wheel Millis per Frame", String( steerConfig.steeringWheelFrameMillis ), ControlColor::Peterriver, tab,
      []( Control * control, int id ) {
        steerConfig.steeringWheelFrameMillis = control->value.toInt();
      } );
      ESPUI.addControl( ControlType::Min, "Min", "1", ControlColor::Peterriver, num );
      ESPUI.addControl( ControlType::Max, "Max", "10000", ControlColor::Peterriver, num );
      ESPUI.addControl( ControlType::Step, "Step", "1000", ControlColor::Peterriver, num );
    }
  }

  // Wheel Angle Sensor Tab
  {
    uint16_t tab = ESPUI.addControl( ControlType::Tab, "Wheel Angle Sensor", "Wheel Angle Sensor" );

    {
      uint16_t sel = ESPUI.addControl( ControlType::Select, "Wheel Angle Sensor*", String( ( int )steerConfig.wheelAngleInput ), ControlColor::Wetasphalt, tab,
      []( Control * control, int id ) {
        steerConfig.wheelAngleInput = ( SteerConfig::AnalogIn )control->value.toInt();
        setResetButtonToRed();
      } );
      ESPUI.addControl( ControlType::Option, "None", "0", ControlColor::Alizarin, sel );
      addAnalogInputADS1115( sel );
    }

    {
      uint16_t sel = ESPUI.addControl( ControlType::Select, "Wheel Angle Sensor Type*", String( ( int )steerConfig.wheelAngleSensorType ), ControlColor::Wetasphalt, tab,
      []( Control * control, int id ) {
        steerConfig.wheelAngleSensorType = ( SteerConfig::WheelAngleSensorType )control->value.toInt();
        setResetButtonToRed();
      } );
      ESPUI.addControl( ControlType::Option, "Direct Wheel Angle", "0", ControlColor::Alizarin, sel );
      ESPUI.addControl( ControlType::Option, "Two Arms connected to tie rod", "1", ControlColor::Alizarin, sel );
    }

    {
      uint16_t num = ESPUI.addControl( ControlType::Number, "Wheel Angle Sensor Center", String( steerConfig.wheelAnglePositionZero ), ControlColor::Peterriver, tab,
      []( Control * control, int id ) {
        steerConfig.wheelAnglePositionZero = control->value.toInt();
      } );
      ESPUI.addControl( ControlType::Min, "Min", "0", ControlColor::Peterriver, num );
      ESPUI.addControl( ControlType::Max, "Max", "26000", ControlColor::Peterriver, num );
      ESPUI.addControl( ControlType::Step, "Step", "1", ControlColor::Peterriver, num );
    }

    {
      uint16_t num = ESPUI.addControl( ControlType::Number, "Wheel Angle Counts per Degree", String( steerConfig.wheelAngleCountsPerDegree ), ControlColor::Peterriver, tab,
      []( Control * control, int id ) {
        steerConfig.wheelAngleCountsPerDegree = control->value.toFloat();
      } );
      ESPUI.addControl( ControlType::Min, "Min", "0", ControlColor::Peterriver, num );
      ESPUI.addControl( ControlType::Max, "Max", "250", ControlColor::Peterriver, num );
      ESPUI.addControl( ControlType::Step, "Step", "0.1", ControlColor::Peterriver, num );
    }

    ESPUI.addControl( ControlType::Switcher, "Invert Wheel Angle Sensor", steerConfig.invertWheelAngleSensor ? "1" : "0", ControlColor::Peterriver, tab,
    []( Control * control, int id ) {
      steerConfig.invertWheelAngleSensor = control->value.toInt() == 1;
    } );

    {
      uint16_t num = ESPUI.addControl( ControlType::Number, "Wheel Angle Offset", String( steerConfig.wheelAngleOffset ), ControlColor::Peterriver, tab,
      []( Control * control, int id ) {
        steerConfig.wheelAngleOffset = control->value.toFloat();
      } );
      ESPUI.addControl( ControlType::Min, "Roll Min", "-80", ControlColor::Peterriver, num );
      ESPUI.addControl( ControlType::Max, "Roll Max", "80", ControlColor::Peterriver, num );
      ESPUI.addControl( ControlType::Step, "Roll Step", "0.1", ControlColor::Peterriver, num );
    }

    if( steerConfig.wheelAngleSensorType == SteerConfig::WheelAngleSensorType::TieRodDisplacement ) {
      {
        uint16_t num = ESPUI.addControl( ControlType::Number, "1. Arm connect to sensor (mm)", String( steerConfig.wheelAngleFirstArmLenght ), ControlColor::Peterriver, tab,
        []( Control * control, int id ) {
          steerConfig.wheelAngleFirstArmLenght = control->value.toFloat();
        } );
        ESPUI.addControl( ControlType::Min, "Min", "0", ControlColor::Peterriver, num );
        ESPUI.addControl( ControlType::Max, "Max", "500", ControlColor::Peterriver, num );
        ESPUI.addControl( ControlType::Step, "Step", "1", ControlColor::Peterriver, num );
      }

      {
        uint16_t num = ESPUI.addControl( ControlType::Number, "2. Arm connect to tie rod (mm)", String( steerConfig.wheelAngleSecondArmLenght ), ControlColor::Peterriver, tab,
        []( Control * control, int id ) {
          steerConfig.wheelAngleSecondArmLenght = control->value.toFloat();
        } );
        ESPUI.addControl( ControlType::Min, "Min", "0", ControlColor::Peterriver, num );
        ESPUI.addControl( ControlType::Max, "Max", "500", ControlColor::Peterriver, num );
        ESPUI.addControl( ControlType::Step, "Step", "1", ControlColor::Peterriver, num );
      }

      {
        uint16_t num = ESPUI.addControl( ControlType::Number, "Tie rod stroke (mm)", String( steerConfig.wheelAngleTieRodStroke ), ControlColor::Peterriver, tab,
        []( Control * control, int id ) {
          steerConfig.wheelAngleTieRodStroke = control->value.toFloat();
        } );
        ESPUI.addControl( ControlType::Min, "Min", "0", ControlColor::Peterriver, num );
        ESPUI.addControl( ControlType::Max, "Max", "500", ControlColor::Peterriver, num );
        ESPUI.addControl( ControlType::Step, "Step", "1", ControlColor::Peterriver, num );
      }

      {
        uint16_t num = ESPUI.addControl( ControlType::Number, "Minimum Angle of wheel angle sensor", String( steerConfig.wheelAngleMinimumAngle ), ControlColor::Peterriver, tab,
        []( Control * control, int id ) {
          steerConfig.wheelAngleMinimumAngle = control->value.toFloat();
        } );
        ESPUI.addControl( ControlType::Min, "Min", "0", ControlColor::Peterriver, num );
        ESPUI.addControl( ControlType::Max, "Max", "180", ControlColor::Peterriver, num );
        ESPUI.addControl( ControlType::Step, "Step", "1", ControlColor::Peterriver, num );
      }

      {
        uint16_t num = ESPUI.addControl( ControlType::Number, "Lenght of Track Arm (mm)", String( steerConfig.wheelAngleTrackArmLenght ), ControlColor::Peterriver, tab,
        []( Control * control, int id ) {
          steerConfig.wheelAngleTrackArmLenght = control->value.toFloat();
        } );
        ESPUI.addControl( ControlType::Min, "Min", "0", ControlColor::Peterriver, num );
        ESPUI.addControl( ControlType::Max, "Max", "500", ControlColor::Peterriver, num );
        ESPUI.addControl( ControlType::Step, "Step", "1", ControlColor::Peterriver, num );
      }
    }

    {
      uint16_t num = ESPUI.addControl( ControlType::Number, "Ackermann (%)", String( steerConfig.ackermann ), ControlColor::Peterriver, tab,
      []( Control * control, int id ) {
        steerConfig.ackermann = control->value.toFloat();
      } );
      ESPUI.addControl( ControlType::Min, "Min", "0", ControlColor::Peterriver, num );
      ESPUI.addControl( ControlType::Max, "Max", "200", ControlColor::Peterriver, num );
      ESPUI.addControl( ControlType::Step, "Step", "1", ControlColor::Peterriver, num );
    }

    {
      uint16_t sel = ESPUI.addControl( ControlType::Select, "Ackermann - Wheel with WAS has smaller radius", String( ( int )steerConfig.ackermannAboveZero ), ControlColor::Peterriver, tab,
      []( Control * control, int id ) {
        steerConfig.ackermannAboveZero = control->value.toInt() == 1;
      } );
      ESPUI.addControl( ControlType::Option, "when Actual degrees are below zero", "0", ControlColor::Alizarin, sel );
      ESPUI.addControl( ControlType::Option, "when Actual degrees are above zero", "1", ControlColor::Alizarin, sel );
    }
  }

  // Steering Tab
  {
    uint16_t tab = ESPUI.addControl( ControlType::Tab, "Steering", "Steering" );

    {
      uint16_t sel = ESPUI.addControl( ControlType::Select, "Output Type*", String( ( int )steerConfig.outputType ), ControlColor::Wetasphalt, tab,
      []( Control * control, int id ) {
        steerConfig.outputType = ( SteerConfig::OutputType )control->value.toInt();
        setResetButtonToRed();
      } );
      ESPUI.addControl( ControlType::Option, "None", "0", ControlColor::Alizarin, sel );
      ESPUI.addControl( ControlType::Option, "Motor: Cytron MD30C", "1", ControlColor::Alizarin, sel );
      ESPUI.addControl( ControlType::Option, "Motor: IBT 2", "2", ControlColor::Alizarin, sel );
      ESPUI.addControl( ControlType::Option, "Hydraulic: IBT 2 + PWM 2-Coil Valve", "3", ControlColor::Alizarin, sel );
      ESPUI.addControl( ControlType::Option, "Hydraulic: IBT 2 + Danfoss Valve PVE A/H/M", "4", ControlColor::Alizarin, sel );
      ESPUI.addControl( ControlType::Option, "Hydraulic: IBT 2 + Bang Bang Valve", "5", ControlColor::Alizarin, sel );
    }

    ESPUI.addControl( ControlType::Switcher, "Invert Output", steerConfig.invertOutput ? "1" : "0", ControlColor::Peterriver, tab,
    []( Control * control, int id ) {
      steerConfig.invertOutput = control->value.toInt() == 1;
    } );

    {
      uint16_t num = ESPUI.addControl( ControlType::Number, "PWM Frequency*", String( steerConfig.pwmFrequency, 2 ), ControlColor::Wetasphalt, tab,
      []( Control * control, int id ) {
        steerConfig.pwmFrequency = control->value.toDouble();
        setResetButtonToRed();
      } );
      ESPUI.addControl( ControlType::Min, "Min", "1", ControlColor::Peterriver, num );
      ESPUI.addControl( ControlType::Max, "Max", "4000", ControlColor::Peterriver, num );
      ESPUI.addControl( ControlType::Step, "Step", "1", ControlColor::Peterriver, num );
    }

    {
      uint16_t num = ESPUI.addControl( ControlType::Number, "Dither (Hydraulic Pwm 2 Coil only)", String( steerConfig.dither ), ControlColor::Peterriver, tab,
      []( Control * control, int id ) {
        steerConfig.dither = control->value.toInt();
        ditherAmount = 0;
      } );
      ESPUI.addControl( ControlType::Min, "Min", "0", ControlColor::Peterriver, num );
      ESPUI.addControl( ControlType::Max, "Max", "100", ControlColor::Peterriver, num );
      ESPUI.addControl( ControlType::Step, "Step", "1", ControlColor::Peterriver, num );
    }
  }

  // Steering PID Tab
  {
    uint16_t tab = ESPUI.addControl( ControlType::Tab, "Steering PID", "Steering PID" );

    {
      uint16_t num = ESPUI.addControl( ControlType::Number, "PID Kp", String( steerConfig.steeringPidKp, 4 ), ControlColor::Peterriver, tab,
      []( Control * control, int id ) {
        steerConfig.steeringPidKp = control->value.toDouble();
      } );
      ESPUI.addControl( ControlType::Min, "Min", "0", ControlColor::Peterriver, num );
      ESPUI.addControl( ControlType::Max, "Max", "50", ControlColor::Peterriver, num );
      ESPUI.addControl( ControlType::Step, "Step", "0.1", ControlColor::Peterriver, num );
    }
    {
      uint16_t num = ESPUI.addControl( ControlType::Number, "PID Ki", String( steerConfig.steeringPidKi, 4 ), ControlColor::Peterriver, tab,
      []( Control * control, int id ) {
        steerConfig.steeringPidKi = control->value.toDouble();
      } );
      ESPUI.addControl( ControlType::Min, "Min", "0", ControlColor::Peterriver, num );
      ESPUI.addControl( ControlType::Max, "Max", "50", ControlColor::Peterriver, num );
      ESPUI.addControl( ControlType::Step, "Step", "0.01", ControlColor::Peterriver, num );
    }
    {
      uint16_t num = ESPUI.addControl( ControlType::Number, "PID Ki Max", String( steerConfig.steeringPidKiMax, 2 ), ControlColor::Peterriver, tab,
      []( Control * control, int id ) {
        steerConfig.steeringPidKiMax = control->value.toDouble();
      } );
      ESPUI.addControl( ControlType::Min, "Min", "0", ControlColor::Peterriver, num );
      ESPUI.addControl( ControlType::Max, "Max", "255", ControlColor::Peterriver, num );
      ESPUI.addControl( ControlType::Step, "Step", "1.00", ControlColor::Peterriver, num );
    }
    {
      uint16_t num = ESPUI.addControl( ControlType::Number, "PID Kd", String( steerConfig.steeringPidKd, 4 ), ControlColor::Peterriver, tab,
      []( Control * control, int id ) {
        steerConfig.steeringPidKd = control->value.toDouble();
      } );
      ESPUI.addControl( ControlType::Min, "Min", "0", ControlColor::Peterriver, num );
      ESPUI.addControl( ControlType::Max, "Max", "50", ControlColor::Peterriver, num );
      ESPUI.addControl( ControlType::Step, "Step", "0.01", ControlColor::Peterriver, num );
    }
    {
      uint16_t num = ESPUI.addControl( ControlType::Number, "Automatic Bang On Factor (multiple of saturation with Kp, 0 to turn off)", String( steerConfig.steeringPidAutoBangOnFactor, 4 ), ControlColor::Peterriver, tab,
      []( Control * control, int id ) {
        steerConfig.steeringPidAutoBangOnFactor = control->value.toDouble();
      } );
      ESPUI.addControl( ControlType::Min, "Min", "0", ControlColor::Peterriver, num );
      ESPUI.addControl( ControlType::Max, "Max", "10", ControlColor::Peterriver, num );
      ESPUI.addControl( ControlType::Step, "Step", "0.1", ControlColor::Peterriver, num );
    }
    {
      uint16_t num = ESPUI.addControl( ControlType::Number, "Turn Output on if error is greater (BangOn)", String( steerConfig.steeringPidBangOn, 4 ), ControlColor::Peterriver, tab,
      []( Control * control, int id ) {
        steerConfig.steeringPidBangOn = control->value.toDouble();
      } );
      ESPUI.addControl( ControlType::Min, "Min", "0", ControlColor::Peterriver, num );
      ESPUI.addControl( ControlType::Max, "Max", "50", ControlColor::Peterriver, num );
      ESPUI.addControl( ControlType::Step, "Step", "0.01", ControlColor::Peterriver, num );
    }
    {
      uint16_t num = ESPUI.addControl( ControlType::Number, "Turn Output off if error is smaller (BangOff)", String( steerConfig.steeringPidBangOff, 4 ), ControlColor::Peterriver, tab,
      []( Control * control, int id ) {
        steerConfig.steeringPidBangOff = control->value.toDouble();
      } );
      ESPUI.addControl( ControlType::Min, "Min", "0", ControlColor::Peterriver, num );
      ESPUI.addControl( ControlType::Max, "Max", "50", ControlColor::Peterriver, num );
      ESPUI.addControl( ControlType::Step, "Step", "0.01", ControlColor::Peterriver, num );
    }
    {
      uint16_t num = ESPUI.addControl( ControlType::Number, "Minimum PWM", String( steerConfig.steeringPidMinPwm ), ControlColor::Peterriver, tab,
      []( Control * control, int id ) {
        steerConfig.steeringPidMinPwm = control->value.toInt();
      } );
      ESPUI.addControl( ControlType::Min, "Min", "0", ControlColor::Peterriver, num );
      ESPUI.addControl( ControlType::Max, "Max", "255", ControlColor::Peterriver, num );
      ESPUI.addControl( ControlType::Step, "Step", "1", ControlColor::Peterriver, num );
    }
  }

  // Sensors Tab
  {
    uint16_t tab = ESPUI.addControl( ControlType::Tab, "Sensors", "Sensors" );

    {
      uint16_t num = ESPUI.addControl( ControlType::Number, "I2C Bus Speed*", String( steerConfig.i2cBusSpeed ), ControlColor::Wetasphalt, tab,
      []( Control * control, int id ) {
        steerConfig.i2cBusSpeed = control->value.toInt();
      } );
      ESPUI.addControl( ControlType::Min, "Min", "10000", ControlColor::Peterriver, num );
      ESPUI.addControl( ControlType::Max, "Max", "5000000", ControlColor::Peterriver, num );
      ESPUI.addControl( ControlType::Step, "Step", "1000", ControlColor::Peterriver, num );
    }

  }

  // Safety Tab
  {
    uint16_t tab = ESPUI.addControl( ControlType::Tab, "Safety", "Safety" );

    {
      uint16_t num = ESPUI.addControl( ControlType::Number, "Max autosteer speed", String( steerConfig.maxAutosteerSpeed ), ControlColor::Peterriver, tab,
      []( Control * control, int id ) {
        steerConfig.maxAutosteerSpeed = control->value.toFloat();
      } );
      ESPUI.addControl( ControlType::Min, "Min", "0", ControlColor::Peterriver, num );
      ESPUI.addControl( ControlType::Max, "Max", "30", ControlColor::Peterriver, num );
      ESPUI.addControl( ControlType::Step, "Step", "0.5", ControlColor::Peterriver, num );
    }
    {
      uint16_t sel = ESPUI.addControl( ControlType::Select, "Speed units", String( ( int )steerConfig.speedUnits ), ControlColor::Peterriver, tab,
      []( Control * control, int id ) {
        steerConfig.speedUnits =  ( SteerConfig::SpeedUnits )control->value.toInt();
      } );
      ESPUI.addControl( ControlType::Option, "MPH", "0", ControlColor::Alizarin, sel );
      ESPUI.addControl( ControlType::Option, "KPH", "1", ControlColor::Alizarin, sel );
    }
  }

  if( steerConfig.mode == SteerConfig::Mode::QtOpenGuidance ) {

    // Channels Tab
    {
      uint16_t tab = ESPUI.addControl( ControlType::Tab, "Channels", "Channels" );

      {
        uint16_t num = ESPUI.addControl( ControlType::Number, "Channel ID Autosteer Enable", String( steerConfig.qogChannelIdAutosteerEnable ), ControlColor::Peterriver, tab,
        []( Control * control, int id ) {
          steerConfig.qogChannelIdAutosteerEnable = control->value.toInt();
        } );
        ESPUI.addControl( ControlType::Min, "Min", "0", ControlColor::Peterriver, num );
        ESPUI.addControl( ControlType::Max, "Max", "65535", ControlColor::Peterriver, num );
        ESPUI.addControl( ControlType::Step, "Step", "1", ControlColor::Peterriver, num );
      }
      {
        uint16_t num = ESPUI.addControl( ControlType::Number, "Channel ID Workswitch", String( steerConfig.qogChannelIdWorkswitch ), ControlColor::Peterriver, tab,
        []( Control * control, int id ) {
          steerConfig.qogChannelIdWorkswitch = control->value.toInt();
        } );
        ESPUI.addControl( ControlType::Min, "Min", "0", ControlColor::Peterriver, num );
        ESPUI.addControl( ControlType::Max, "Max", "65535", ControlColor::Peterriver, num );
        ESPUI.addControl( ControlType::Step, "Step", "1", ControlColor::Peterriver, num );
      }
      {
        uint16_t num = ESPUI.addControl( ControlType::Number, "Channel ID Steerswitch", String( steerConfig.qogChannelIdSteerswitch ), ControlColor::Peterriver, tab,
        []( Control * control, int id ) {
          steerConfig.qogChannelIdWorkswitch = control->value.toInt();
        } );
        ESPUI.addControl( ControlType::Min, "Min", "0", ControlColor::Peterriver, num );
        ESPUI.addControl( ControlType::Max, "Max", "65535", ControlColor::Peterriver, num );
        ESPUI.addControl( ControlType::Step, "Step", "1", ControlColor::Peterriver, num );
      }

      {
        uint16_t num = ESPUI.addControl( ControlType::Number, "Channel ID Wheel Angle", String( steerConfig.qogChannelIdWheelAngle ), ControlColor::Peterriver, tab,
        []( Control * control, int id ) {
          steerConfig.qogChannelIdWheelAngle = control->value.toInt();
        } );
        ESPUI.addControl( ControlType::Min, "Min", "0", ControlColor::Peterriver, num );
        ESPUI.addControl( ControlType::Max, "Max", "65535", ControlColor::Peterriver, num );
        ESPUI.addControl( ControlType::Step, "Step", "1", ControlColor::Peterriver, num );
      }

      {
        uint16_t num = ESPUI.addControl( ControlType::Number, "Channel ID Steer Angle Setpoint", String( steerConfig.qogChannelIdSetpointSteerAngle ), ControlColor::Peterriver, tab,
        []( Control * control, int id ) {
          steerConfig.qogChannelIdSetpointSteerAngle = control->value.toInt();
        } );
        ESPUI.addControl( ControlType::Min, "Min", "0", ControlColor::Peterriver, num );
        ESPUI.addControl( ControlType::Max, "Max", "65535", ControlColor::Peterriver, num );
        ESPUI.addControl( ControlType::Step, "Step", "1", ControlColor::Peterriver, num );
      }

      {
        uint16_t num = ESPUI.addControl( ControlType::Number, "Channel ID Orientation", String( steerConfig.qogChannelIdOrientation ), ControlColor::Peterriver, tab,
        []( Control * control, int id ) {
          steerConfig.qogChannelIdOrientation = control->value.toInt();
        } );
        ESPUI.addControl( ControlType::Min, "Min", "0", ControlColor::Peterriver, num );
        ESPUI.addControl( ControlType::Max, "Max", "65535", ControlColor::Peterriver, num );
        ESPUI.addControl( ControlType::Step, "Step", "1", ControlColor::Peterriver, num );
      }

      {
        uint16_t num = ESPUI.addControl( ControlType::Number, "Channel ID GPS Data In", String( steerConfig.qogChannelIdGpsDataIn ), ControlColor::Peterriver, tab,
        []( Control * control, int id ) {
          steerConfig.qogChannelIdGpsDataIn = control->value.toInt();
        } );
        ESPUI.addControl( ControlType::Min, "Min", "0", ControlColor::Peterriver, num );
        ESPUI.addControl( ControlType::Max, "Max", "65535", ControlColor::Peterriver, num );
        ESPUI.addControl( ControlType::Step, "Step", "1", ControlColor::Peterriver, num );
      }
      {
        uint16_t num = ESPUI.addControl( ControlType::Number, "Channel ID GPS Data Out", String( steerConfig.qogChannelIdGpsDataOut ), ControlColor::Peterriver, tab,
        []( Control * control, int id ) {
          steerConfig.qogChannelIdGpsDataOut = control->value.toInt();
        } );
        ESPUI.addControl( ControlType::Min, "Min", "0", ControlColor::Peterriver, num );
        ESPUI.addControl( ControlType::Max, "Max", "65535", ControlColor::Peterriver, num );
        ESPUI.addControl( ControlType::Step, "Step", "1", ControlColor::Peterriver, num );
      }

      if( steerConfig.mode == SteerConfig::Mode::QtOpenGuidance && steerConfig.canBusEnabled ) {
        {
          uint16_t num = ESPUI.addControl( ControlType::Number, "Channel ID CAN: Rear Hitch", String( steerConfig.qogChannelIdCanRearHitch ), ControlColor::Peterriver, tab,
          []( Control * control, int id ) {
            steerConfig.qogChannelIdCanRearHitch = control->value.toInt();
          } );
          ESPUI.addControl( ControlType::Min, "Min", "0", ControlColor::Peterriver, num );
          ESPUI.addControl( ControlType::Max, "Max", "65535", ControlColor::Peterriver, num );
          ESPUI.addControl( ControlType::Step, "Step", "1", ControlColor::Peterriver, num );
        }
        {
          uint16_t num = ESPUI.addControl( ControlType::Number, "Channel ID CAN: Front Hitch", String( steerConfig.qogChannelIdCanFrontHitch ), ControlColor::Peterriver, tab,
          []( Control * control, int id ) {
            steerConfig.qogChannelIdCanFrontHitch = control->value.toInt();
          } );
          ESPUI.addControl( ControlType::Min, "Min", "0", ControlColor::Peterriver, num );
          ESPUI.addControl( ControlType::Max, "Max", "65535", ControlColor::Peterriver, num );
          ESPUI.addControl( ControlType::Step, "Step", "1", ControlColor::Peterriver, num );
        }
        {
          uint16_t num = ESPUI.addControl( ControlType::Number, "Channel ID CAN: Rear RPM", String( steerConfig.qogChannelIdCanRearPtoRpm ), ControlColor::Peterriver, tab,
          []( Control * control, int id ) {
            steerConfig.qogChannelIdCanRearPtoRpm = control->value.toInt();
          } );
          ESPUI.addControl( ControlType::Min, "Min", "0", ControlColor::Peterriver, num );
          ESPUI.addControl( ControlType::Max, "Max", "65535", ControlColor::Peterriver, num );
          ESPUI.addControl( ControlType::Step, "Step", "1", ControlColor::Peterriver, num );
        }
        {
          uint16_t num = ESPUI.addControl( ControlType::Number, "Channel ID CAN: Front RPM", String( steerConfig.qogChannelIdCanFrontPtoRpm ), ControlColor::Peterriver, tab,
          []( Control * control, int id ) {
            steerConfig.qogChannelIdCanFrontPtoRpm = control->value.toInt();
          } );
          ESPUI.addControl( ControlType::Min, "Min", "0", ControlColor::Peterriver, num );
          ESPUI.addControl( ControlType::Max, "Max", "65535", ControlColor::Peterriver, num );
          ESPUI.addControl( ControlType::Step, "Step", "1", ControlColor::Peterriver, num );
        }
        {
          uint16_t num = ESPUI.addControl( ControlType::Number, "Channel ID CAN: Motor RPM", String( steerConfig.qogChannelIdCanMotorRpm ), ControlColor::Peterriver, tab,
          []( Control * control, int id ) {
            steerConfig.qogChannelIdCanMotorRpm = control->value.toInt();
          } );
          ESPUI.addControl( ControlType::Min, "Min", "0", ControlColor::Peterriver, num );
          ESPUI.addControl( ControlType::Max, "Max", "65535", ControlColor::Peterriver, num );
          ESPUI.addControl( ControlType::Step, "Step", "1", ControlColor::Peterriver, num );
        }
        {
          uint16_t num = ESPUI.addControl( ControlType::Number, "Channel ID CAN: Wheel-based Speed", String( steerConfig.qogChannelIdCanWheelbasedSpeed ), ControlColor::Peterriver, tab,
          []( Control * control, int id ) {
            steerConfig.qogChannelIdCanWheelbasedSpeed = control->value.toInt();
          } );
          ESPUI.addControl( ControlType::Min, "Min", "0", ControlColor::Peterriver, num );
          ESPUI.addControl( ControlType::Max, "Max", "65535", ControlColor::Peterriver, num );
          ESPUI.addControl( ControlType::Step, "Step", "1", ControlColor::Peterriver, num );
        }
      }
    }
  }

  // Default Configurations Tab
  {
    uint16_t tab = ESPUI.addControl( ControlType::Tab, "Configurations", "Configurations" );
    ESPUI.addControl( ControlType::Label, "Attention:", "These Buttons here reset the whole config. This affects the WIFI too, if not configured otherwise below. You have to press \"Apply & Reboot\" above to actualy store them.", ControlColor::Carrot, tab );

    ESPUI.addControl( ControlType::Label, "OTA Update:", "<a href='/update'>Update</a>", ControlColor::Carrot, tab );

    ESPUI.addControl( ControlType::Label, "Download the config:", "<a href='config.json'>Configuration</a>", ControlColor::Carrot, tab );

    ESPUI.addControl( ControlType::Label, "Upload the config:", "<form method='POST' action='/upload-config' enctype='multipart/form-data'><input name='f' type='file'><input type='submit'></form>", ControlColor::Carrot, tab );

    // onchange='this.form.submit()'
    {
      ESPUI.addControl( ControlType::Switcher, "Retain WIFI settings", steerConfig.retainWifiSettings ? "1" : "0", ControlColor::Peterriver, tab,
      []( Control * control, int id ) {
        steerConfig.retainWifiSettings = control->value.toInt() == 1;
      } );
    }
    {
      ESPUI.addControl( ControlType::Button, "Set Settings To Default*", "Defaults", ControlColor::Wetasphalt, tab,
      []( Control * control, int id ) {
        char ssid[24], password[24], hostname[24];

        if( steerConfig.retainWifiSettings ) {
          memcpy( ssid, steerConfig.ssid, sizeof( ssid ) );
          memcpy( password, steerConfig.password, sizeof( password ) );
          memcpy( hostname, steerConfig.hostname, sizeof( hostname ) );
        }

        steerConfig = steerConfigDefaults;

        if( steerConfig.retainWifiSettings ) {
          memcpy( steerConfig.ssid, ssid, sizeof( ssid ) );
          memcpy( steerConfig.password, password, sizeof( password ) );
          memcpy( steerConfig.hostname, hostname, sizeof( hostname ) );
        }

        setResetButtonToRed();
      } );
    }

    tabConfigurations = tab;

  }


  ESPUI.server->on( "/config.json", HTTP_GET, []( AsyncWebServerRequest * request ) {
    request->send( SPIFFS, "/config.json", "application/json", true );
  } );

  // upload a file to /upload-config
  ESPUI.server->on( "/upload-config", HTTP_POST, []( AsyncWebServerRequest * request ) {
    request->send( 200 );
  }, [tabConfigurations]( AsyncWebServerRequest * request, String filename, size_t index, uint8_t* data, size_t len, bool final ) {
    if( !index ) {
      request->_tempFile = SPIFFS.open( "/config.json", "w" );
    }

    if( request->_tempFile ) {
      if( len ) {
        request->_tempFile.write( data, len );
      }

      if( final ) {
        request->_tempFile.close();
        setResetButtonToRed();
        String str( "/#tab" );
        str += tabConfigurations;
        request->redirect( str );
      }
    }
  } );

}
