#include <Arduino.h>
#include "iowIguana.h"
#include <PubSubClient.h>
#include <string>
#include <iostream>

// WiFi and MQTT parameters
#define MQTT_PORT        1883
#define wifi_ssid        "iownwater"        
#define wifi_password    "temp3_NL156$"    
#define mqtt_server      "35.223.234.244"
#define mqtt_user        "iowlabs"
#define mqtt_password    "!iow_woi!"
#define publish_topic    "esp32/iguana"
#define sleeptime_topic  "esp32/sleeptime"
#define uS_TO_S_FACTOR   1000000 

// RTC fast memory (8 kB SRAM) variables
RTC_DATA_ATTR int  TIME_TO_SLEEP_prev = 0;
RTC_DATA_ATTR int  TIME_TO_SLEEP      = 20; 
unsigned      long currentTime;

//WiFi and MQTT client
WiFiClient wifiClient;
PubSubClient client(wifiClient);

iowIguana iguana   = iowIguana();
String    output;

void setupWiFi();
void reconnect();
void publishMqtt(char *serialData);
void callback(char* topic, byte* message, unsigned int length);

void setup() {
	Serial.begin(115200);
  iguana.activateSTH();
  iguana.activateSoilTemp();
  iguana.activateSoilMoisture();
	iguana.activateRS485();

	iguana.begin();
  delay(200);
  iguana.iowLogo();
  delay(3000);
  iguana.showLogo();
  delay(3000);

	setupWiFi();
  client.setServer(mqtt_server,MQTT_PORT);
  client.setCallback(callback);

  if (!client.connected()) reconnect();
  Serial.println("Setup Ready: WiFi, MQTT and Iguana sensors");

  // Iguana sensor reading
  iguana.readSensors();
  output = iguana.pubData();
  iguana.saveData();
  iguana.showData(2500);

  // MQTT listening for 5 seconds to receive commands
  currentTime = millis();
  while (millis() - currentTime < 2000) client.loop();
  
  // MQTT publishing
  client.loop();
  delay(100);
  client.publish(publish_topic,  output.c_str());
  Serial.printf("Message published on topic: %s. Message: %s\n", publish_topic, output.c_str());
  delay(1000);

  // Deep Sleep
  client.unsubscribe(sleeptime_topic);
  client.disconnect();
  WiFi.disconnect();
  esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR); //go to sleep
  Serial.println("Setup ESP32 to Deep sleep for " + String(TIME_TO_SLEEP) + " Seconds");
  Serial.println("Going to sleep now...zzZzzZzz");
  esp_deep_sleep_start();
}

// Callback function to process MQTT messages
void callback(char* topic, byte* message, unsigned int length) {
  Serial.printf("Message arrived on topic: %s. Message: ",topic);
  String messageTemp;
  int messageR;
  
  for (int i = 0; i < length; i++) {
    Serial.print((char)message[i]);
    messageTemp += (char)message[i];
  }
  Serial.println();

  if (String(topic) == sleeptime_topic) {
    TIME_TO_SLEEP_prev = TIME_TO_SLEEP;
    TIME_TO_SLEEP = std::stoi(messageTemp.c_str());

    if (TIME_TO_SLEEP != TIME_TO_SLEEP_prev){
      Serial.printf("Changing Sleep time from %d to %d seconds\n", TIME_TO_SLEEP_prev, TIME_TO_SLEEP);
    }
    else{
      Serial.println("Change discarded since it's already incorporated");
    }
  }
}

void loop() {
  // client.loop();
  // iguana.readSensors();
  // output = iguana.pubData();
  // iguana.saveData();
  // iguana.showData(2500);
  // publishMqtt((char*) output.c_str());
  // Serial.printf("Message published on topic: %s. Message: %s\n", publish_topic, output.c_str());
  // delay(20000);
}

void setupWiFi() {
  delay(20);
  Serial.println();
  Serial.printf("Connecting to %s \n", wifi_ssid);

  WiFi.begin(wifi_ssid, wifi_password);

  int count=0;
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    digitalWrite(LED,HIGH);
    delay(500);
    digitalWrite(LED,LOW);
    count++;
    Serial.print(".");
    if (count > 20) {
      Serial.println("");
      Serial.println("Something bad happened, trying to reset\n");
      client.disconnect();
      WiFi.disconnect();
      ESP.restart();
    }
    }
    Serial.println("");
    digitalWrite(LED,HIGH);
    Serial.println("WiFi is OK ");
    Serial.print("=> ESP32 new IP address is: ");
    Serial.println(WiFi.localIP());
}

// Funcion para reconectar a la red en caso que se pierda la conexion
void reconnect()
{
    while (!client.connected()) {
    
    Serial.print("Connecting to MQTT broker ...");
    if (client.connect("ESP32Client", mqtt_user, mqtt_password)) {
      Serial.println("OK");
      client.setCallback(callback);
      client.subscribe(sleeptime_topic);
      client.loop();
      delay(500);
    } else {
      Serial.print("[Error] Not connected: "+String(client.state())+". Wait 5 seconds before retry.");
      delay(5000);
    }
  }
}


void publishMqtt(char *payload)
{

  if(!client.connected()){
   	reconnect();
  }

  client.publish(publish_topic, payload);
}
