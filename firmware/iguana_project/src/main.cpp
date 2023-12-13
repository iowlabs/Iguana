/*
# SPDX-FileCopyrightText: 2023 iowlabs <contacto@iowlabs.com>
#
# SPDX-License-Identifier: GPL-3.0-or-later.txt
*/

/*
Project: Iguana
DESCRIPTION: This code is for Iguana sensor station for small agriculture.
It features both temperature and humidity sensors to monitor 

Last Author: Rickss

Features: Capacitive Soil Moisture Sensor, DS18B20 waterproof temperature sensor 
and SHT31 Temperature & Humidity Sensor. 1'3 OLED Screen.

Connections (USED PIN MAP):
  -----------------------------

Comunication:
  -----------------------------

LIBS:
	sparkfun/SparkFun Qwiic RTC RV8803 Arduino Library @ ^1.2.8
	paulstoffregen/OneWire@^2.3.7
	milesburton/DallasTemperature@^3.11.0
	adafruit/Adafruit SHT31 Library@^2.2.2
	adafruit/Adafruit BusIO@^1.14.3
	adafruit/Adafruit Unified Sensor@^1.1.13
	bblanchon/ArduinoJson@^6.21.3
	adafruit/Adafruit SH110X @ ^2.1.8
    adafruit/Adafruit GFX Library @ ^1.11.5
	fastled/FastLED@^3.6.0
	sandeepmistry/LoRa @ ^0.8.0
	hieromon/AutoConnect @ ^1.4.2

TODOLIST:
-	EXAMPLES FILES
-	CHECK STATUS and connectivity
- 	HANDLE ERRORS
- 	ADD RGB FEATUREs
- 	ADD PUMP Features
-   ADD AUTOMATIC Functions
-   SET SAMPLE TIME options
-   ADD TIMERs
-	less RGB brightness
*/

#include <Arduino.h>
#include "iowIguana.h"

iowIguana iguana = iowIguana();
uint8_t status;
String output;

void setup()
{
	Serial.begin(115200);
  	iguana.activateSTH();
  	iguana.activateSoilTemp();
  	iguana.activateSoilMoisture();
	status = iguana.begin();
	printlnd(status);

  	delay(200);

  	iguana.iowLogo();
  	delay(3000);
  	iguana.showLogo();
  	delay(3000);

	Serial.println("Setup Ready");
}

void loop()
{
  iguana.readSensors();
  output = iguana.pubData();
  Serial.println(output);
  //iguana.saveData();
  iguana.showData(2500);

}
