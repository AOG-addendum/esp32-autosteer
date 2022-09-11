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

#include "main.hpp"


// ESP32 pollutes the env with macros for all possible binary values starting with "B" (pe "B1 = 1", "B1111=7"...)
// undef them, as json.hpp defines a template argument with B1
#undef B1
#include <../lib/json/json.hpp>
using json = nlohmann::json;

#pragma once

extern void loadSavedDiagnostics();
extern void saveDiagnostics();

extern void loadSavedConfig();
extern void saveConfig();

extern json loadJsonFromFile( const char* fileName );
extern void saveJsonToFile( const json& json, const char* fileName );

extern void parseJsonToDiagnostics( json& json, Diagnostics& diagnostics );
extern json parseDiagnosticsToJson( const Diagnostics& diagnostics );

extern void parseJsonToSteerConfig( json& json, SteerConfig& config );
extern json parseSteerConfigToJson( const SteerConfig& config );

extern void sendStateTransmission( uint16_t channelId, bool state );
extern void sendNumberTransmission( uint16_t channelId, double number );
