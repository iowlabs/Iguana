/*
Projecto: Iguana
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

//-------------
//  LIBS
//-------------

#include <Arduino.h>
#include <DallasTemperature.h>
#include <OneWire.h>
#include <Wire.h>
#include <SPI.h>
#include "Adafruit_SHT31.h"
#include <ArduinoJson.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SH110X.h>

//-------------
//  DEFINES
//-------------

// Version (mayor, minor, patch, build)
#define VERSION    "IGN-v.0.1.1" //Iguana Sensor Station
#define ID         "IGN01"

#define uS_TO_S_FACTOR 1000000  /* Conversion factor for micro seconds to seconds */
#define TIME_TO_SLEEP  5        /* Time ESP32 will go to sleep (in seconds) */

#define AOUT_PIN 27 //For Capacitive Soil Moisture Sensor

//Voltage Enable
#define V_EN 33

#define SCREEN_ADDRESS 0x3C
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define OLED_RESET     -1 // Reset pin # (or -1 if sharing Arduino reset pin)


#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define OLED_RESET     -1 // Reset pin # (or -1 if sharing Arduino reset pin)


//-------------
// INSTANCES
//-------------

Adafruit_SH1106G display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

//JSON to SEND
char json_tx[1024];


// DS18B20 waterproof temperature sensor 
const int oneWireBus = 32;
OneWire oneWire(oneWireBus);
DallasTemperature sensors(&oneWire);

Adafruit_SHT31 sht31 = Adafruit_SHT31();
// Capacitive Soil Sensor Calibration Parameters
const int AirValue = 2468;   
const int WaterValue = 986;  
int soilmoisture = 0;

float soiltemperature = 0;
float SHT31temperatureC = 0;
float SHT31humidity = 0;

//Sampling
int NSamples = 1000;


const unsigned char logo_iowlabs [] PROGMEM = {
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x01, 0xfc, 0x03, 0xf0, 0x0f, 0xc0, 0x3f, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x01, 0xfc, 0x07, 0xf8, 0x0f, 0xe0, 0x3f, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x01, 0xfc, 0x07, 0xf8, 0x1f, 0xe0, 0x3f, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x01, 0xfc, 0x07, 0xf8, 0x1f, 0xe0, 0x3f, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x3f, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfc, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x7f, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0xff, 0xe3, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0xff, 0xe1, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x1e, 0x00, 0x00, 0x00, 0x1e, 0x00, 0x00, 0x00,
	0xff, 0xe3, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x1e, 0x00, 0x00, 0x00, 0x1e, 0x00, 0x00, 0x00,
	0xff, 0xf7, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x1e, 0x00, 0x00, 0x00, 0x1e, 0x00, 0x00, 0x00,
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x1e, 0x00, 0x00, 0x00, 0x1e, 0x00, 0x00, 0x00,
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x1e, 0x00, 0x00, 0x00, 0x1e, 0x00, 0x00, 0x00,
	0xff, 0xe3, 0xfc, 0x07, 0xe3, 0xf0, 0xfc, 0x7f, 0x1e, 0x00, 0x00, 0xc0, 0x1e, 0x60, 0x01, 0xc0,
	0xff, 0xe1, 0xf0, 0x01, 0xe3, 0xf0, 0xfc, 0x7f, 0x1e, 0x00, 0x07, 0xf8, 0x1f, 0xf8, 0x0f, 0xf8,
	0xff, 0xe1, 0xe0, 0x40, 0xe1, 0xf0, 0x7c, 0x7f, 0x1e, 0x00, 0x0f, 0xfc, 0x1f, 0xfc, 0x1f, 0xfe,
	0xff, 0xe1, 0xe1, 0xf0, 0xf1, 0xe0, 0x78, 0x7f, 0x1e, 0x00, 0x0f, 0xfe, 0x1f, 0xfe, 0x1f, 0xfe,
	0xff, 0xe1, 0xc3, 0xf8, 0x71, 0xe0, 0x78, 0xff, 0x1e, 0x00, 0x06, 0x1e, 0x1f, 0x1e, 0x3e, 0x1c,
	0xff, 0xe1, 0xc3, 0xf8, 0x70, 0xe0, 0x38, 0xff, 0x1e, 0x00, 0x00, 0x0e, 0x1e, 0x0e, 0x3c, 0x00,
	0xff, 0xe1, 0xc3, 0xf8, 0x78, 0xc2, 0x30, 0xff, 0x1e, 0x00, 0x00, 0x0e, 0x1e, 0x0e, 0x3f, 0x00,
	0xff, 0xe1, 0xc3, 0xf8, 0x78, 0xc6, 0x31, 0xff, 0x1e, 0x00, 0x07, 0xfe, 0x1e, 0x0e, 0x1f, 0xf8,
	0xff, 0xe1, 0xc3, 0xf8, 0x78, 0x46, 0x31, 0xff, 0x1e, 0x00, 0x0f, 0xfe, 0x1e, 0x0e, 0x1f, 0xfc,
	0xff, 0xe1, 0xc3, 0xf8, 0x7c, 0x47, 0x01, 0xff, 0x1e, 0x00, 0x1f, 0xfe, 0x1e, 0x0e, 0x07, 0xfe,
	0xff, 0xe1, 0xc3, 0xf8, 0x7c, 0x0f, 0x03, 0xff, 0x1e, 0x00, 0x1e, 0x0e, 0x1e, 0x0e, 0x00, 0x1e,
	0xff, 0xe1, 0xe1, 0xf0, 0xfc, 0x0f, 0x03, 0xff, 0x1e, 0x00, 0x1c, 0x0e, 0x1e, 0x0e, 0x10, 0x1e,
	0xff, 0xe1, 0xe0, 0x40, 0xfe, 0x0f, 0x83, 0xff, 0x1e, 0x00, 0x1e, 0x0e, 0x1e, 0x1e, 0x1c, 0x1e,
	0xff, 0xe1, 0xf0, 0x01, 0xfe, 0x0f, 0x87, 0xff, 0x1f, 0xff, 0x9f, 0xfe, 0x1f, 0xfe, 0x3f, 0xfe,
	0xff, 0xe3, 0xfc, 0x07, 0xff, 0x1f, 0x87, 0xff, 0x1f, 0xff, 0x9f, 0xfe, 0x1f, 0xfc, 0x3f, 0xfc,
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x1f, 0xff, 0x8f, 0xfe, 0x1f, 0xfc, 0x1f, 0xf8,
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x7f, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x3f, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfe, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x01, 0xfc, 0x07, 0xf8, 0x1f, 0xe0, 0x3f, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x01, 0xfc, 0x07, 0xf8, 0x1f, 0xe0, 0x3f, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x01, 0xfc, 0x07, 0xf8, 0x0f, 0xe0, 0x3f, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x01, 0xfc, 0x03, 0xf0, 0x0f, 0xe0, 0x3f, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};


//Functions
void publishData(){
  StaticJsonDocument<256> doc_tx;

  doc_tx["Tsoil"] = (int)(soiltemperature* 100 + 0.5) / 100.0;
  doc_tx["Hsoil"] = (int)(soilmoisture* 100 + 0.5) / 100.0;
  doc_tx["T"] = (int)(SHT31temperatureC* 100 + 0.5) / 100.0;
  doc_tx["H"] = (int)(SHT31humidity* 100 + 0.5) / 100.0;

  String json;
  serializeJson(doc_tx, json);
  Serial.println(json);
}

/*
void showiowLogo()
{
  display.clearDisplay(); // Clear the buffer
  display.drawBitmap(0, 0, logo_iowlabs, 128, 64,1,0);
  display.display();
}
*/
void setup() {
  Serial.begin(115200);
  Wire.begin();
  pinMode(V_EN,OUTPUT);
  digitalWrite(V_EN,HIGH);
  delay(100);

  sht31.begin(0x44);
  
  display.display();
  delay(2000); // Pause for 2 seconds
  // Clear the buffer
  display.clearDisplay();

  delay(2000);

  //2 Seconds to initiate serial and sensors

  //esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR);

  //CODE AFTER THIS LINE
 
  //END CODE HERE
  /*
  delay(1000);
  Serial.flush(); 
  esp_deep_sleep_start();
  */
}

void loop() {
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

}

