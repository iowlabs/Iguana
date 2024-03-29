/*
# SPDX-FileCopyrightText: 2023 iowlabs <contacto@iowlabs.com>
#
# SPDX-License-Identifier: GPL-3.0-or-later.txt
*/

/*
Project: Iguana
DESCRIPTION: This code is for Iguana sensor station for small agriculture.
It features both temperature and humidity sensors to monitor

Last Author:
WAC@IOWLABS
CRISTOBAL@Iowlabs
RICKS@IOWLABS


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
#include <PubSubClient.h>


// Parametros de MQTT  USER DEFINE
#define MQTT_PORT         1883
#define MQTT_USER         "iowlabs"
#define MQTT_PASSWORD     "!iow_woi!"
#define MQTT_PUBLISH_CH   "Iguana"
#define MQTT_RECEIVER_CH  "Iguana/rx"

//Wifi and mqtt client
WiFiClient wifiClient;
PubSubClient mqtt(wifiClient);

// Conexión a una red y servidor de MQTT
const char* ssid      	= "iownwater";
const char* password  	= "temp3_NL156$";
const char* mqtt_server = "35.223.234.244";

iowIguana iguana = iowIguana();
uint8_t status;
String output;
int  mqtt_try       = 0; // Cantidad de intentos para conectarse
bool mqtt_connected = false;
bool wifi_status = false;

void    publishMqtt(char *serialData);
void    setupWiFi();
bool    reconnect();



void setup()
{
	Serial.begin(115200);
  	iguana.activateSTH();
  	iguana.activateSoilTemp();
  	iguana.activateSoilMoisture();
	iguana.activateRS485();

	status = iguana.begin();
	printlnd(status);

  	delay(200);

  	iguana.iowLogo();
  	delay(3000);
  	iguana.showLogo();
  	delay(3000);


	setupWiFi();
   	mqtt.setServer(mqtt_server,MQTT_PORT);

	Serial.println("Setup Ready");
}

void loop()
{
  iguana.readSensors();
  output = iguana.pubData();
  Serial.println(output);
  //iguana.saveData();
  //iguana.showData(2500);
  publishMqtt((char*) output.c_str());
  delay(60000);
}


void setupWiFi()
{
  delay(10);
  printd("Connecting to ");
  printlnd(ssid);
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    digitalWrite(LED,HIGH);
    delay(500);
    digitalWrite(LED,LOW);
    printd(".");

  }
  printlnd("WiFi connected");
  printd("IP address: ");
  printlnd(WiFi.localIP());
  digitalWrite(LED,HIGH);
}

// Funcion para reconectar a la red en caso que se pierda la conexion
bool reconnect()
{
    while (!mqtt.connected())
    {
        printd("Attempting MQTT connection...");
        if (mqtt.connect(ID, MQTT_USER , MQTT_PASSWORD))
        {
            printlnd("connected");
            mqtt.subscribe(MQTT_RECEIVER_CH);
            mqtt_try = 0;
            return true;
        }
        else
        {
            printd("failed, rc=");
            printlnd(mqtt.state());
            printlnd(" try again in 5 seconds");
            delay(5000);
            mqtt_try += 1;
            if(mqtt_try>=3)
            {
              mqtt_try = 0;
              return false;
            }
        }
    }

    return false;
}


void publishMqtt(char *payload)
{

  if(!mqtt.connected())
  {
   	reconnect();
	mqtt.publish(MQTT_PUBLISH_CH, "hola");

  }
  mqtt.publish(MQTT_PUBLISH_CH, payload);

}
