/*
Projecto: PIguana
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
	paulstoffregen/OneWire@^2.3.7
	milesburton/DallasTemperature@^3.11.0
	adafruit/Adafruit SHT31 Library@^2.2.2
	adafruit/Adafruit BusIO@^1.14.3
	adafruit/Adafruit Unified Sensor@^1.1.13
	adafruit/DHT sensor library@^1.4.4
	bblanchon/ArduinoJson@^6.21.3

To Do:

*/

//-------------
//  LIBS
//-------------

#include <Arduino.h>
#include <DallasTemperature.h>
#include <OneWire.h>
#include <Wire.h>
#include <SPI.h>
#include <DHT.h>
#include "Adafruit_SHT31.h"
#include <ArduinoJson.h>

//-------------
//  DEFINES
//-------------

// Version (mayor, minor, patch, build)
#define VERSION    "IGN-v.0.1.1" //Iguana Sensor Station
#define ID         "IGN01"

#define uS_TO_S_FACTOR 1000000  /* Conversion factor for micro seconds to seconds */
#define TIME_TO_SLEEP  5        /* Time ESP32 will go to sleep (in seconds) */

#define AOUT_PIN 27 //For Capacitive Soil Moisture Sensor

//-------------
// INSTANCES
//-------------

//JSON to SEND
char json_tx[1024];

// DS18B20 waterproof temperature sensor 
const int oneWireBus = 26;
OneWire oneWire(oneWireBus);
DallasTemperature sensors(&oneWire);

Adafruit_SHT31 sht31 = Adafruit_SHT31();
DHT sensorTH (4,DHT22);

// Capacitive Soil Sensor Calibration Parameters
const int AirValue = 2468;   
const int WaterValue = 986;  
int soilmoisture = 0;

float soiltemperature = 0;
float SHT31temperatureC = 0;
float SHT31humidity = 0;

//Sampling
int NSamples = 1000;

//Functions
void publishData()
{
  StaticJsonDocument<256> doc_tx;

  doc_tx["Tsoil"] = (int)(soiltemperature* 100 + 0.5) / 100.0;
  doc_tx["Hsoil"] = (int)(soilmoisture* 100 + 0.5) / 100.0;
  doc_tx["T"] = (int)(SHT31temperatureC* 100 + 0.5) / 100.0;
  doc_tx["H"] = (int)(SHT31humidity* 100 + 0.5) / 100.0;

  String json;
  serializeJson(doc_tx, json);
  Serial.println(json);
}

void setup() {
  Serial.begin(115200);
  sht31.begin(0x44);
  delay(2000); //2 Seconds to initiate serial and sensors

  esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR);

  //CODE AFTER THIS LINE
  sensors.requestTemperatures();
  soiltemperature = sensors.getTempCByIndex(0);
  SHT31temperatureC = sht31.readTemperature();
  SHT31humidity = sht31.readHumidity();
  
  int soilsensor = 0;
  for(int i =0; i<NSamples;i++){
    soilsensor += analogRead(AOUT_PIN);
  }
  soilsensor = soilsensor/NSamples; 
  soilsensor = map(soilsensor, AirValue, WaterValue, 61.3, 100);
  soilmoisture = constrain(soilsensor, 0, 100);
  
  /*
  Serial.println("=== Temperature Sensor ===");
  Serial.print(soiltemperature);
  Serial.println(" ºC");
  Serial.println("=== Capacitive Soil Sensor ===");
  Serial.print("Moisture value: ");
  Serial.print(soilmoisture); 
  Serial.println (" %");
  Serial.println("===== SHT31 =====");
  Serial.print("Temperature: ");
  Serial.print(SHT31temperatureC);
  Serial.println(" °C");
  Serial.print("Humidity: ");
  Serial.print(SHT31humidity);
  Serial.println (" %");
  Serial.println("");
  */

  publishData();
  delay(2000);

  //END CODE HERE
  delay(1000);
  Serial.flush(); 
  esp_deep_sleep_start();
}

void loop() {
}

