

#include "main.hpp"
#if defined(ESP32)
#include "esp_wifi.h"
#include <WiFi.h>
#else
#include <ESP8266WiFi.h>
#endif

IPAddress softApIP( 192, 168, 1, 1 );
String apName;
bool WiFiWasConnected = false;

void WiFiStationGotIP(WiFiEvent_t event, WiFiEventInfo_t info){
    IPAddress myIP = WiFi.localIP();
    if( myIP[3] != 89 ){
        myIP[3] = 89;
        IPAddress gwIP = WiFi.gatewayIP();
        if (!WiFi.config(myIP, gwIP, IPAddress( 255, 255, 255, 0 ), gwIP)) {
          Serial.println("STA Failed to configure");
        }
        Serial.println("Switching off AP, station only");
        WiFi.softAPdisconnect (true);
        WiFi.mode( WIFI_MODE_STA );
    }
    if( steerConfig.apModePin != SteerConfig::Gpio::None ) {
        digitalWrite( ( int )steerConfig.apModePin, HIGH );
    }
    WiFiWasConnected = true;
}

void WiFiStationDisconnected(WiFiEvent_t event, WiFiEventInfo_t info){
    if( steerConfig.apModePin != SteerConfig::Gpio::None ) {
        digitalWrite( ( int )steerConfig.apModePin, LOW );
    }
    if( WiFiWasConnected == true ){
      WiFi.disconnect(true);
      if( !WiFi.config(INADDR_NONE, INADDR_NONE, INADDR_NONE) ) {
        Serial.println("STA Failed to unset configuration");
      }
      delay(2000);
      WiFi.begin( steerConfig.ssid, steerConfig.password );
    }
    Serial.println("reconnecting");
}

void WiFiStationConnected(WiFiEvent_t event, WiFiEventInfo_t info){
  WiFi.config(INADDR_NONE, INADDR_NONE, INADDR_NONE);
  #if defined(ESP32)
    WiFi.setHostname( steerConfig.hostname );
  #else
    WiFi.hostname( steerConfig.hostname );
  #endif
}

void WiFiAPStaConnected(WiFiEvent_t event, WiFiEventInfo_t info){
    Serial.println("Switching off station mode, AP only");
    WiFi.disconnect(true);
    delay(100);
    WiFi.mode( WIFI_MODE_AP );
}

void initWiFi( void ){
  WiFi.config(INADDR_NONE, INADDR_NONE, INADDR_NONE);
  delay(250);
  #if defined(ESP32)
    WiFi.onEvent(WiFiStationConnected, SYSTEM_EVENT_STA_CONNECTED);
    WiFi.onEvent(WiFiStationDisconnected, SYSTEM_EVENT_STA_DISCONNECTED);
    WiFi.onEvent(WiFiStationGotIP, SYSTEM_EVENT_STA_GOT_IP);
    WiFi.onEvent(WiFiAPStaConnected, SYSTEM_EVENT_AP_STACONNECTED);
  #endif
    // try to connect to existing network
    WiFi.begin( steerConfig.ssid, steerConfig.password );
    Serial.print( "\n\nTry to connect to existing network \"" );
    Serial.print( steerConfig.ssid );
    Serial.print( "\" with password \"" );
    Serial.print( steerConfig.password );
    Serial.println( "\"" );

    uint8_t timeout = 5;
    // Wait for connection, 2.5s timeout
    do {
    delay( 500 );
    Serial.print( "." );
    timeout--;
    } while( timeout && WiFi.status() != WL_CONNECTED );
    // not connected -> create hotspot
    if( WiFi.status() != WL_CONNECTED ) {
        WiFi.disconnect(true);
        Serial.print( "\n\nCreating hotspot" );

        if( steerConfig.apModePin != SteerConfig::Gpio::None ) {
          digitalWrite( ( int )steerConfig.apModePin, LOW );
        }

        apName = String( "Machine steer " );
        apName += WiFi.macAddress();
        apName.replace( ":", "" );

        WiFi.mode( WIFI_MODE_AP );
        WiFi.softAP( apName.c_str() );
        while (!SYSTEM_EVENT_AP_START) { // wait until AP has started
            delay(100);
            Serial.print(".");
        }
        WiFi.softAPConfig( softApIP, softApIP, IPAddress( 255, 255, 255, 0 ) );
        //WiFi.begin( steerConfig.ssid, steerConfig.password );
    }
}
