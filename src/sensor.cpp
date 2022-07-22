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

#include <Adafruit_Sensor.h>

#include <Adafruit_ADS1015.h>

#include <ESPUI.h>

#include "main.hpp"
#include "jsonFunctions.hpp"

#include "average.hpp"
#include "ringbuffer.hpp"

Adafruit_ADS1115 ads = Adafruit_ADS1115( 0x48 );

volatile uint16_t samplesPerSecond;

// http://www.schwietering.com/jayduino/filtuino/index.php?characteristic=bu&passmode=lp&order=2&usesr=usesr&sr=100&frequencyLow=5&noteLow=&noteHigh=&pw=pw&calctype=float&run=Send
//Low pass butterworth filter order=2 alpha1=0.05
class  FilterBuLp2_3 {
  public:
    FilterBuLp2_3() {
      v[0] = 0.0;
      v[1] = 0.0;
    }
  private:
    float v[3];
  public:
    float step( float x ) { //class II
      v[0] = v[1];
      v[1] = v[2];
      v[2] = ( 2.008336556421122521e-2 * x )
             + ( -0.64135153805756306422 * v[0] )
             + ( 1.56101807580071816339 * v[1] );
      return
              ( v[0] + v[2] )
              + 2 * v[1];
    }
} wheelAngleSensorFilter;


void sensorWorker100HzPoller( void* z ) {
  vTaskDelay( 2000 );
  constexpr TickType_t xFrequency = 10;
  TickType_t xLastWakeTime = xTaskGetTickCount();

  for( ;; ) {

    if( steerConfig.wheelAngleInput != SteerConfig::AnalogIn::None ) {
      float wheelAngleTmp = 0;

      switch( ( uint8_t )steerConfig.wheelAngleInput ) {
        case( uint8_t )SteerConfig::AnalogIn::ADS1115A0Single ...( uint8_t )SteerConfig::AnalogIn::ADS1115A1Single: {
          if( xSemaphoreTake( i2cMutex, 1000 ) == pdTRUE ) {
            wheelAngleTmp = ads.readADC_SingleEnded(
                                    ( uint8_t )steerConfig.wheelAngleInput - ( uint8_t )SteerConfig::AnalogIn::ADS1115A0Single );
            xSemaphoreGive( i2cMutex );
          }
        }
        break;

        case( uint8_t )SteerConfig::AnalogIn::ADS1115A0A1Differential: {
          if( xSemaphoreTake( i2cMutex, 1000 ) == pdTRUE ) {
            wheelAngleTmp = ads.readADC_Differential_0_1();
            xSemaphoreGive( i2cMutex );
          }
        }
        break;

        default:
          break;
      }

      {
        steerSetpoints.wheelAngleCounts = wheelAngleTmp;
        wheelAngleTmp -= steerConfig.wheelAnglePositionZero;
        wheelAngleTmp /= steerConfig.wheelAngleCountsPerDegree;

        steerSetpoints.wheelAngleRaw = wheelAngleTmp;

        if( steerConfig.wheelAngleSensorType == SteerConfig::WheelAngleSensorType::TieRodDisplacement ) {
          if( steerConfig.wheelAngleFirstArmLenght != 0 && steerConfig.wheelAngleSecondArmLenght != 0 &&
              steerConfig.wheelAngleTrackArmLenght != 0 && steerConfig.wheelAngleTieRodStroke != 0 ) {

            auto getDisplacementFromAngle = []( float angle ) {
              // a: 2. arm, b: 1. arm, c: abstand drehpunkt wineklsensor und anschlagpunt 2. arm an der spurstange
              // gegenwinkel: winkel zwischen 1. arm und spurstange
              double alpha = PI - radians( angle );

              // winkel zwischen spurstange und 2. arm
              double gamma = PI - alpha - ( asin( steerConfig.wheelAngleFirstArmLenght * sin( alpha ) / steerConfig.wheelAngleSecondArmLenght ) );

              // auslenkung
              return steerConfig.wheelAngleSecondArmLenght * sin( gamma ) / sin( alpha );
            };

            steerSetpoints.wheelAngleCurrentDisplacement = getDisplacementFromAngle( wheelAngleTmp );

            double relativeDisplacementToStraightAhead =
                    // real displacement
                    steerSetpoints.wheelAngleCurrentDisplacement -
                    // calculate middle of displacement -
                    ( getDisplacementFromAngle( steerConfig.wheelAngleMinimumAngle ) + ( steerConfig.wheelAngleTieRodStroke / 2 ) );

            wheelAngleTmp = degrees( asin( relativeDisplacementToStraightAhead / steerConfig.wheelAngleTrackArmLenght ) );
          }
        }

        if( steerConfig.invertWheelAngleSensor ) {
          wheelAngleTmp *= ( float ) -1;
        }

        wheelAngleTmp -= steerConfig.wheelAngleOffset;

        if (wheelAngleTmp > 0 && steerConfig.ackermannAboveZero == true) {
          wheelAngleTmp = ( wheelAngleTmp * steerConfig.ackermann) / 100;
        }
        else if (wheelAngleTmp < 0 && steerConfig.ackermannAboveZero == false){
          wheelAngleTmp = ( wheelAngleTmp * steerConfig.ackermann) / 100;
        }

        wheelAngleTmp = wheelAngleSensorFilter.step( wheelAngleTmp );
        steerSetpoints.actualSteerAngle = wheelAngleTmp;

        if( steerConfig.mode == SteerConfig::Mode::QtOpenGuidance ) {
          static uint8_t loopCounter = 0;

          if( ++loopCounter >= 10 ) {
            sendNumberTransmission( steerConfig.qogChannelIdWheelAngle, wheelAngleTmp );
          }
        }
      }
    }

    {
      static uint8_t loopCounter = 0;

      if( loopCounter++ > 99 ) {
        loopCounter = 0;
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
      }
    }

    vTaskDelayUntil( &xLastWakeTime, xFrequency );
  }
}

void initSensors() {

  // initialise ads1115 everytime, even if not available (no answer in the init -> just sending)
  {
    ads.setGain( GAIN_TWOTHIRDS );   // 2/3x gain +/- 6.144V  1 bit = 3mV      0.1875mV (default)
    // ads.setGain(GAIN_ONE);        // 1x gain   +/- 4.096V  1 bit = 2mV      0.125mV
    // ads.setGain(GAIN_TWO);        // 2x gain   +/- 2.048V  1 bit = 1mV      0.0625mV
    // ads.setGain(GAIN_FOUR);       // 4x gain   +/- 1.024V  1 bit = 0.5mV    0.03125mV
    // ads.setGain(GAIN_EIGHT);      // 8x gain   +/- 0.512V  1 bit = 0.25mV   0.015625mV
    // ads.setGain(GAIN_SIXTEEN);    // 16x gain  +/- 0.256V  1 bit = 0.125mV  0.0078125mV

    ads.begin();
    ads.setSPS( ADS1115_DR_860SPS );

    Control* handle = ESPUI.getControl( labelStatusAdc );
    handle->value = "ADC1115 initialized";
    handle->color = ControlColor::Emerald;
    initialisation.wheelAngleInput = steerConfig.wheelAngleInput;
    ESPUI.updateControlAsync( handle );
  }

  xTaskCreate( sensorWorker100HzPoller, "sensorWorker100HzPoller", 4096, NULL, 6, NULL );
}
