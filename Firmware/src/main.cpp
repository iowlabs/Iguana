#include <Arduino.h>
#include <DallasTemperature.h>
#include <OneWire.h>
#include <Wire.h>
#include <SPI.h>
#include "Adafruit_SHT31.h"

// put function declarations here:

const int oneWireBus = 26;
#define AOUT_PIN 27

const int AirValue = 2600;   
const int WaterValue = 1124;  
int soilmoisturepercent = 0;

OneWire oneWire(oneWireBus);
DallasTemperature sensors(&oneWire);

Adafruit_SHT31 sht31 = Adafruit_SHT31();


void setup() {
    // Start the Serial Monitor
  Serial.begin(115200);
  sht31.begin(0x44);
  // Start the DS18B20 sensor
  sensors.begin();
}

void loop() {
  // put your main code here, to run repeatedly:
  int value = analogRead(AOUT_PIN); // read the analog value from sensor
  value = map(value, AirValue, WaterValue, 54.2, 100);
  soilmoisturepercent = constrain(value, 0, 100);

  sensors.requestTemperatures(); 
  float temperatureC = sensors.getTempCByIndex(0);
  float temperatureF = sensors.getTempFByIndex(0);

  float shtemperatureC = sht31.readTemperature();
  float shhumidity = sht31.readHumidity();

  Serial.println("Temperature Sensor");
  Serial.print(temperatureC);
  Serial.println("ºC");
  Serial.print(temperatureF);
  Serial.println("ºF");
  Serial.print("");
  Serial.println("Capacitive Soil Sensor");
  Serial.print("Moisture value: ");
  Serial.println(soilmoisturepercent);
  Serial.println("SHT31");
  Serial.print("Temperature: ");
  Serial.print(shtemperatureC);
  Serial.println(" °C");
  Serial.print("Moisture value: ");
  Serial.println(shhumidity);
  delay(2000);
}