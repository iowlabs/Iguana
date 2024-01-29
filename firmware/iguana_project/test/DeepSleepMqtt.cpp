/*
Project: Iguana

Description: This code implements periodic Deep Sleep routines to optimize power
             consumption, in addition to supporting MQTT publishing.

Last Author: Catalina Sierra

Libraries:
	sparkfun/SparkFun Qwiic RTC RV8803 Arduino Library @ ^1.2.8
	paulstoffregen/OneWire@^2.3.7
	milesburton/DallasTemperature@^3.11.0
	adafruit/Adafruit SHT31 Library@^2.2.2
	adafruit/Adafruit BusIO@^1.14.3
	adafruit/Adafruit Unified Sensor@^1.1.13
	bblanchon/ArduinoJson@^6.21.3
	adafruit/Adafruit SH110X @ ^2.1.8
    adafruit/Adafruit GFX Library @ ^1.11.5
	knolleary/PubSubClient@^2.8
	fastled/FastLED@^3.6.0
	sandeepmistry/LoRa @ ^0.8.0
*/

#include <Arduino.h>
#include "iowIguana.h"
#include <PubSubClient.h>
#include "esp_sleep.h"
#include "esp_wifi.h"

// Parametros de MQTT  USER DEFINE
#define MQTT_PORT         1883
#define MQTT_USER         "iowlabs"
#define MQTT_PASSWORD     "!iow_woi!"
#define MQTT_PUBLISH_CH   "Iguana2"
#define MQTT_RECEIVER_CH  "Iguana/rx"

// Definitions for deepsleep
#define uS_TO_S_FACTOR 1000000     /* Conversion factor for micro seconds to seconds */
#define TIME_TO_SLEEP 20           /* Time ESP32 will go to sleep (in seconds) */

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
String msg = "Iguanita";
int  mqtt_try       = 0;
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

	iguana.begin();
	printlnd(status);

	delay(200);
  	iguana.iowLogo();
  	delay(3000);
  	iguana.showLogo();
  	delay(3000);

	setupWiFi();
  	mqtt.setServer(mqtt_server,MQTT_PORT);
	Serial.println("Setup Ready");

  	iguana.readSensors();
  	output = iguana.pubData();
  	Serial.println(output);
  	iguana.saveData();
  	iguana.showData(2500);

  	if (!mqtt.connected())
	{
    	reconnect();}
  		mqtt.loop();
  		delay(100);
  		mqtt.publish("Iguana2",  output.c_str());
  		Serial.println("Iguanita sent");
  		delay(3000);
  		mqtt.unsubscribe("Iguana2");
  		mqtt.disconnect();
  		WiFi.disconnect();
  		esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR); //go to sleep
  		Serial.println("Setup ESP32 to Deep sleep for " + String(TIME_TO_SLEEP) + " Seconds");
  		Serial.println("Going to sleep now...zzZzzZzz");
  		esp_deep_sleep_start();
	}

void loop()
{

}

void setupWiFi()
{
  delay(20);
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.begin(ssid, password);

  int count=0;
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    digitalWrite(LED,HIGH);
    delay(500);
    digitalWrite(LED,LOW);
    count++;
    Serial.print(".");
    	if (count > 20)
		{
    		Serial.println("");
    		Serial.println("Something bad happened, trying to reset");
    		mqtt.disconnect();
    		WiFi.disconnect();
    		ESP.restart();
    	}
    }
    Serial.println("");
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

  if(!mqtt.connected()){
   	reconnect();
  }

  mqtt.publish(MQTT_PUBLISH_CH, payload);
}
