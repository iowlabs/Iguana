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
#define publish_topic    "esp32/wakeup"
#define MQTT_PUBLISH_CH  "esp32/wakeup"
#define sleeptime_topic  "esp32/sleeptime"
#define uS_TO_S_FACTOR   1000000 




//-------------
//  DEFINES
//-------------



// Version (mayor, minor, patch, build)
#define VERSION    "MQTT-v.0.1.0.b.0" //
#define ID         "n01_test"

// SAMPLING
#define TIMER_F_SCLK        10000   // TIMER SOURCE CLOCK. IN kHz
#define TIMER_DEFAULT_TS    3      // TIMER DEFAULT PERIOD. IN SECONDS

// Debug terminal baudrate
#define BR_DEBUG  9600

// Operation modes
#define IDLE      0
#define RUNNING   1
#define SLAVE     2
#define TEST      3

//MACROS FOR CONTROL SERIAL PRINTS
#define DEBUG     1 // Comment this line to turn off serial prints
#if DEBUG
#define printd(s) {Serial.print((s));}
#define printlnd(s) {Serial.println((s));}
#else
#define printd(s)
#define printlnd(s)
#endif

//List of pre-defined responses
#define RESPONSE_OK     "OK"
#define RESPONSE_ERROR  "ERROR"
#define RESPONSE_ERROR_JSON  "ERROR_INPUT_JSON"
#define RESPONSE_UPDATE "ERROR_UPDATE"
#define RESPONSE_WRONG_CHANNEL "ERROR_WRONG_CHANNEL"
#define RESPONSE_ERROR_CMD "ERROR_CMD"


//default file name
#define DATALOGGER 		1
#define FILE_NAME       "/log.txt"




// RTC fast memory (8 kB SRAM) variables
RTC_DATA_ATTR int  TIME_TO_SLEEP_prev = 0;
RTC_DATA_ATTR int  TIME_TO_SLEEP      = 20; 
RTC_DATA_ATTR int  ESCUCHAR           = 0; // 1: escuchar, 0: no escuchar
unsigned      long currentTime;

//WiFi and MQTT client
WiFiClient wifiClient;
PubSubClient mqtt(wifiClient);
WiFiClientSecure cliente;

iowIguana iguana   = iowIguana();
//String    output;










const char* URL_fw_Bin = "https://raw.githubusercontent.com/Iowlabs/Iguana/main/firmware/OTA_bin_file/firmware.bin";

static const char cacert[] PROGMEM = R"EOF(
-----BEGIN CERTIFICATE-----
MIIDQTCCAimgAwIBAgITBmyfz5m/jAo54vB4ikPmljZbyjANBgkqhkiG9w0BAQsF
ADA5MQswCQYDVQQGEwJVUzEPMA0GA1UEChMGQW1hem9uMRkwFwYDVQQDExBBbWF6
b24gUm9vdCBDQSAxMB4XDTE1MDUyNjAwMDAwMFoXDTM4MDExNzAwMDAwMFowOTEL
MAkGA1UEBhMCVVMxDzANBgNVBAoTBkFtYXpvbjEZMBcGA1UEAxMQQW1hem9uIFJv
b3QgQ0EgMTCCASIwDQYJKoZIhvcNAQEBBQADggEPADCCAQoCggEBALJ4gHHKeNXj
ca9HgFB0fW7Y14h29Jlo91ghYPl0hAEvrAIthtOgQ3pOsqTQNroBvo3bSMgHFzZM
9O6II8c+6zf1tRn4SWiw3te5djgdYZ6k/oI2peVKVuRF4fn9tBb6dNqcmzU5L/qw
IFAGbHrQgLKm+a/sRxmPUDgH3KKHOVj4utWp+UhnMJbulHheb4mjUcAwhmahRWa6
VOujw5H5SNz/0egwLX0tdHA114gk957EWW67c4cX8jJGKLhD+rcdqsq08p8kDi1L
93FcXmn/6pUCyziKrlA4b9v7LWIbxcceVOF34GfID5yHI9Y/QCB/IIDEgEw+OyQm
jgSubJrIqg0CAwEAAaNCMEAwDwYDVR0TAQH/BAUwAwEB/zAOBgNVHQ8BAf8EBAMC
AYYwHQYDVR0OBBYEFIQYzIU07LwMlJQuCFmcx7IQTgoIMA0GCSqGSIb3DQEBCwUA
A4IBAQCY8jdaQZChGsV2USggNiMOruYou6r4lK5IpDB/G/wkjUu0yKGX9rbxenDI
U5PMCCjjmCXPI6T53iHTfIUJrU6adTrCC2qJeHZERxhlbI1Bjjt/msv0tadQ1wUs
N+gDS63pYaACbvXy8MWy7Vu33PqUXHeeE6V/Uq2V8viTO96LXFvKWlJbYK8U90vv
o/ufQJVtMVT8QtPHRh8jrdkPSHCa2XV4cdFyQzR1bldZwgJcJmApzyMZFo6IQ6XU
5MsI+yMRQ+hDKXJioaldXgjUkK642M4UwtBV8ob2xJNDd2ZhwLnoQdeXeGADbkpy
rqXRfboQnoZsG4q5WTP468SQvvG5
-----END CERTIFICATE-----
)EOF";
 
 
// Copy contents from XXXXXXXX-certificate.pem.crt here ▼
static const char clientCert[] PROGMEM = R"KEY(
-----BEGIN CERTIFICATE-----
MIIDWTCCAkGgAwIBAgIUKBFqJ1Flviff0y415UFaxZqrqrUwDQYJKoZIhvcNAQEL
BQAwTTFLMEkGA1UECwxCQW1hem9uIFdlYiBTZXJ2aWNlcyBPPUFtYXpvbi5jb20g
SW5jLiBMPVNlYXR0bGUgU1Q9V2FzaGluZ3RvbiBDPVVTMB4XDTI0MDEwNjE5NDgx
OVoXDTQ5MTIzMTIzNTk1OVowHjEcMBoGA1UEAwwTQVdTIElvVCBDZXJ0aWZpY2F0
ZTCCASIwDQYJKoZIhvcNAQEBBQADggEPADCCAQoCggEBAPEORuEWFWXJAvMsiMVH
eBgUVRrQOg4oOEyhtS9CRsICu/Xr3f9v1X0sLWpNreeDmArrFLPWuvNQzhr4y5KD
o4EoyT3eopQ3XM18buRyI16no1rx76K4B/Wx60ijpgsgplR43l0hWVn1HUZYocwT
AitSFpxmjABhBQHKqhEA3ongvhHKAtNGpljzD2Ld9oGRI6O2r+PAtNFATZtN59Zt
VUwR0GD8k96kVL3j46aS6Jw8BCY/075w/BoUchXBf+PNkdv/0aHjO0wI7ZJwRvgW
Ry7WXtbSrRYWHZPhzTV6PJ82E6HeTIuXKyIyK/DGph/VWAUCFFAXl6hOfydFrrO4
QkcCAwEAAaNgMF4wHwYDVR0jBBgwFoAUtJ/+FDOOO7dTDgat2dP5M2rFpzEwHQYD
VR0OBBYEFIN9WTmUoTTSxoJwte5dantfDe1SMAwGA1UdEwEB/wQCMAAwDgYDVR0P
AQH/BAQDAgeAMA0GCSqGSIb3DQEBCwUAA4IBAQAYyV5IDGhfbEqJvuF7mX93ACzy
fOZDFjrLny3a7D88Pgjkv+RTAZ6WvlHggOrZIaZwNt4A4fuy8ivDUlPrdVtv8z02
uziPpqit5zELjsWNISBHSY8EGNXz+SlCRLQCHBC59Ln5tgc5HqShh0S4mA6JcOrp
LjiK4ehhz3FRgkFLG+ZND7XE7nMXdtmLWBEgQa00zE/XrAr7AbUd8BnUgGQb16dv
hsgeBJNumHV8G+fs+q/jiuHo2NmzvPb+P36Gx7SM/xCpe9aQ6B7OgxskVqESm20m
csV043UZUZN3LtSBUX1i6Gfnm7MoieY9c+OjkrULZNl597oMh06gBxp/xGYK
-----END CERTIFICATE-----

 
)KEY";
 
 
// Copy contents from  XXXXXXXX-private.pem.key here ▼
static const char clientKey[] PROGMEM = R"KEY(
-----BEGIN RSA PRIVATE KEY-----
MIIEpAIBAAKCAQEA8Q5G4RYVZckC8yyIxUd4GBRVGtA6Dig4TKG1L0JGwgK79evd
/2/VfSwtak2t54OYCusUs9a681DOGvjLkoOjgSjJPd6ilDdczXxu5HIjXqejWvHv
orgH9bHrSKOmCyCmVHjeXSFZWfUdRlihzBMCK1IWnGaMAGEFAcqqEQDeieC+EcoC
00amWPMPYt32gZEjo7av48C00UBNm03n1m1VTBHQYPyT3qRUvePjppLonDwEJj/T
vnD8GhRyFcF/482R2//RoeM7TAjtknBG+BZHLtZe1tKtFhYdk+HNNXo8nzYTod5M
i5crIjIr8MamH9VYBQIUUBeXqE5/J0Wus7hCRwIDAQABAoIBAQDcXbjePFoDWuef
dq7x6owXNz816/Wmy43sQEhqk6Qr6GP8RUwFIC+C5sf9zlWUt/Beytl6Lmtp1A0X
CyrPHhhzEt4l+UmXRaVJFOArZ0x+U7I6icIQdzyPl78IANLiiU6FGpnteXc9jZ7d
mFkRPq9U+w5AnF6QWiFBqgwezzShxVSxzBBXpJWsbZvDCagPZ7pE1KhmEFTJ0F3t
G4SEg3qSYAe6wgwZourKX4PlC84Qrz3aDYtBpnyPpQ37ooN8mqw5JuMy6mGbF7j5
5ecZTznNLjJvOVCPHTV5sSx7urU6mszRxdjhDm4BtHe0T7Debp3g8RFqQc8R1HEj
E80acsFBAoGBAP2g79onFj85rAduImsSTROTdHCKHIGVVn41DKZFLVtoljV8RP2K
T0yVz2fnqJF5Oju1vJ2avV5emXj9AxN4BPGZOBJHWQoPFQjT4CR6/wVzAH/DFTkZ
ey0clKp/lBh7yFSXqVvwW4C4M8vmsKyaKXukKgpvVW0V9WSY5pE06DKlAoGBAPNP
PyCxKKeWftFT+pDUwetL+1zfgDFYQ3l5FXXMsendoFfhxQ91u815ocVkQxn/XXgj
Eo3kUWooUEm7hf2Im7SVbC+Afmn1kmtImcUrz0mK/L1ZWxLwI0cOtKajRb+MRO8D
kFXLTEho0iK+v4CoTSd7NC/o8t8//C4t8/vDhal7AoGALNSFkMnX5NUM00NkUgiq
Fwu5SSsgXsdGwogd9EXF/qI96rZMe4Q3ABJ61qLb6NrEsEkvTaxMTKHr/ra6FHbi
aNHsV07FLkH8NdM9izUyXoQ1Ul/pIG6glxi2LmzegHVgs5MbHGUfx2DHxwK/t8KI
EpKx26ZSMzXuahmcp43yfQECgYAO74Nph1Z+ix93zBbqgxTwU7a3acTdwpRJxaQK
mikluQQZwPMu0sRXharnihMQanubX1PJQAW7ZYQgMpG0EYm1WSLN+65G5Ns7GLbP
wrOzyQNgwDZ2ACW6O+3c+NLUQlEM9wQKMDObkkur9FefFzW9QbPfaQZXq/7X+twq
TUNMNwKBgQDWCd6ilx3Y5Ox3uoP5XiWLTynKScJfNo2Zu4um7RBAfKXnAsBAkVEj
ii4o/JdMfJ3xB73ghQm2lvR0T6MEHv8ifloYS+0toDX1egOxCZx2GPZodk6OPKrK
C+s9/oHRb6GULl0hpKca1LcwHc+25CcUmEUjVHdVqCTsM60O/iS9hQ==
-----END RSA PRIVATE KEY-----
 
)KEY";


//-------------
// INSTANCES
//-------------

// define the instances of the used sensors

//timers
hw_timer_t * timer = NULL;    //timer to adquisition of sensors
//RTC
RV8803 rtc;
//SD
//USD_IOW uSD_card(SD_CS);



//-------------
// VARIABLES
//-------------

// CMD recived
const char* rcvd_id;
const char* cmd;
int arg = 0;
const char* response = RESPONSE_OK;
//JSON to SEND
char json_tx[1024];


int mode        = RUNNING;
int sample_time = TIMER_DEFAULT_TS;

//Timer variables
int   timer_counter = int(sample_time * TIMER_F_SCLK); //VARIABLE TO SET THE MAX COUNT OF THE TIMER COUNTER
bool  timer_flag    = false;
bool  publish_flag  = false;
int   count         = 0;

// variables
float s1	= 0.0;
float s2    = 0.0;
float s3    = 0.0;

//RTC unix stamp
unsigned long timestamp = 0;

//SD
bool SD_ini;
bool SD_status;

//
uint8_t status;
String output;

//wifi and mqtt
// const char* ssid      = "iownwater";
// const char* password  = "temp3_NL156$";
// const char* mqtt_server = "35.223.234.244";

int  mqtt_try       = 0; // Cantidad de intentos para conectarse
bool mqtt_connected = false;
bool wifi_status = false;

//-------------
// FUNCTIONS
//-------------











void setupWiFi();
void reconnect();
void callback(char* topic, byte* message, unsigned int length);
void processCmd(byte* payload, unsigned int length);
String publishData();
void publishResponse();
void publishMqtt(char *payload);
void firmwareUpdate();
void writeDatalogger();
void IRAM_ATTR timerISR();


void setup() {
	Serial.begin(115200);
  iguana.activateSTH();
  iguana.activateSoilTemp();
  iguana.activateSoilMoisture();
	iguana.activateRS485();

	iguana.begin();
  delay(200);
  iguana.iowLogo();
  delay(1000);
  iguana.showLogo();
  delay(1000);

	setupWiFi();
  mqtt.setServer(mqtt_server,MQTT_PORT);
  mqtt.setCallback(callback);

  if (!mqtt.connected()) reconnect();
  Serial.println("Setup Ready: WiFi, MQTT and Iguana sensors");

  // Iguana sensor reading
  iguana.readSensors();
  //output = iguana.pubData();
  iguana.saveData();
  iguana.showData(2500);


  // MQTT listening for 5 seconds to receive commands
  currentTime = millis();
  while (millis() - currentTime < 2000) mqtt.loop();
  
  // MQTT publishing
  //mqtt.loop();

  while (ESCUCHAR == 1) {
    mqtt.loop();
    //Serial.println("despiertooooooooooooooooooo");
  }


  Serial.println("Going to sleep now OLA");
  delay(100);
  //mqtt.publish(publish_topic,  output.c_str());
  //Serial.printf("Message published on topic: %s. Message: %s\n", publish_topic, "hola".c_str());
  delay(1000);

  // Deep Sleep
  mqtt.unsubscribe(sleeptime_topic);
  mqtt.disconnect();
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
  Serial.println(messageTemp);

  
  if (ESCUCHAR == 1) {
    if (messageTemp != "DORMIR"){
    processCmd(message,length);
  }
  }

  if (messageTemp == "ESCUCHA"){
    Serial.printf("La variable ESCUCHAR es: %d", ESCUCHAR);
    ESCUCHAR = 1;
    Serial.printf("ESTOY ESCUCHANDO");
    Serial.printf("La variable ESCUCHAR es: %d", ESCUCHAR);

  } else if (messageTemp == "DORMIR"){
    Serial.printf("La variable ESCUCHAR es: %d", ESCUCHAR);
    ESCUCHAR = 0;
    Serial.printf("ESTOY dormiR");
    Serial.printf("La variable ESCUCHAR es: %d", ESCUCHAR);
  }

  Serial.println("Im alieeeeeeeeee");
}

void loop() {
  // mqtt.loop();
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
      mqtt.disconnect();
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
    while (!mqtt.connected()) {
    
    Serial.print("Connecting to MQTT broker ...");
    if (mqtt.connect("ESP32Client", mqtt_user, mqtt_password)) {
      Serial.println("OK");
      mqtt.setCallback(callback);
      mqtt.subscribe(sleeptime_topic);
      mqtt.loop();
      delay(500);
    } else {
      Serial.print("[Error] Not connected: "+String(mqtt.state())+". Wait 5 seconds before retry.");
      delay(5000);
    }
  }
}





void processCmd(byte* payload, unsigned int length)
{
	StaticJsonDocument<256> doc_rx;
    //const char* json_rx = "{\"id\":\"anw00\",\"cmd\":\"gps\",\"arg\":1}";
    DeserializationError error_rx;
    //check for error
    error_rx = deserializeJson(doc_rx, payload,length);
    if (error_rx)
    {
		printd(F("deserializeJson() failed: "));
		response = RESPONSE_ERROR_JSON;
		publishResponse();
      	printlnd(error_rx.c_str());

    }

    //parsing incoming msg

    rcvd_id = doc_rx["id"];
    if( strcmp(rcvd_id,ID)==0)
    {
		JsonObject arg_js = doc_rx["arg"];

      	cmd = doc_rx["cmd"];

      	//prossesing incoming command
      	if(strcmp(cmd,"rst")==0)
      	{
			
        	response = RESPONSE_OK;
        	publishResponse();
			    ESP.restart();
      	}
      	else if(strcmp(cmd,"v")==0)
      	{

			printlnd(VERSION);
			response = VERSION;
        	publishResponse();
      	}
      	else if(strcmp(cmd,"led")==0)
      	{
			int canal = arg_js["canal"];
			int encendido = arg_js["encendido"];
			if (canal == 1){
				if(encendido == 1){
					digitalWrite(LED,HIGH);
				}
				else{
					digitalWrite(LED,LOW);
				}
				response = RESPONSE_OK;
			}
			else if (canal == 2){
				if(encendido == 1){
					digitalWrite(LED,HIGH);
				}
				else{
					digitalWrite(LED,LOW);
				}
				response = RESPONSE_OK;
			}
			else{
				response = RESPONSE_WRONG_CHANNEL;
			}
			publishResponse();
	  	}
      	else if(strcmp(cmd,"fs")==0)
      	{
			int sample_time = arg_js["time"];
			Serial.println(sample_time);

			timerStop(timer);
        	//sample_time = int(arg);
        	if(sample_time <3){sample_time = 3;} // min sample time can be 3s
			timer_counter = int(sample_time * TIMER_F_SCLK);
			timerAttachInterrupt(timer,&timerISR,true);
			timerAlarmWrite(timer, timer_counter,true); // params: timer object, counter_limit, restart counter on top.// to get T= 1s, n=T*f_{timer source clock}
			timerStart(timer);
			response = RESPONSE_OK;
        	publishResponse();
      	}
      	else if(strcmp(cmd,"pub")==0)
      	{
			output = publishData();
  			printlnd(output);
  			publishMqtt((char*) output.c_str());
			response = RESPONSE_OK;
			publishResponse();
      	}
	  	else if(strcmp(cmd,"update")==0)
	  	{
			URL_fw_Bin = arg_js["link"];
		  	firmwareUpdate();
			

		}
      	else
      	{
        	printlnd("Command not valid");
			response = RESPONSE_ERROR_CMD;
			publishResponse();
      	}
      	cmd = "";
      	arg = 0;

    }
    else
    {
      printlnd("msg not for me");
    }
}

String publishData()
{
  StaticJsonDocument<256> doc_tx;

  doc_tx["id"]   = ID;
  doc_tx["t"]    = timestamp;//millis();
  doc_tx["s1"]   = s1;
  doc_tx["s2"]   = s2;
  doc_tx["s3"]   = s3;
  doc_tx["err"]  = RESPONSE_OK;

  String json;
  serializeJson(doc_tx, json);
  return json;
}

void publishResponse()
{
  StaticJsonDocument<256> doc_tx;

  doc_tx["id"]    = ID;
  doc_tx["cmd"]  = cmd;
  doc_tx["arg"]   = 0;
  doc_tx["resp"]  = response;

  String json;
  serializeJson(doc_tx, json);
  printlnd(json);
  publishMqtt((char*) json.c_str());

}

void publishMqtt(char *payload)
{
	if(!mqtt.connected())
 	{
    	reconnect();
  	}
  	mqtt.publish(MQTT_PUBLISH_CH, payload);
}

void firmwareUpdate(void)
{
  //WiFiClientSecure client;
  cliente.setCACert(cacert);
  cliente.setCertificate(clientCert);
  cliente.setPrivateKey(clientKey);
  //httpUpdate.setLedPin(LED_BUILTIN, LOW);
  httpUpdate.setLedPin(2, LOW);
  Serial.println("Updating firmware...");
  t_httpUpdate_return ret = httpUpdate.update(cliente, URL_fw_Bin);

  switch (ret) {
  case HTTP_UPDATE_FAILED:
    Serial.printf("HTTP_UPDATE_FAILD Error (%d): %s\n", httpUpdate.getLastError(), httpUpdate.getLastErrorString().c_str());
	response = RESPONSE_UPDATE;
	publishResponse();
    break;

  case HTTP_UPDATE_NO_UPDATES:
    Serial.println("HTTP_UPDATE_NO_UPDATES");
    break;

  case HTTP_UPDATE_OK:
    Serial.println("HTTP_UPDATE_OK");
  	Serial.println("Firmware updated!");
	response = RESPONSE_OK;
	publishResponse();

    break;
  }
}

void writeDatalogger()
{
  StaticJsonDocument<256> doc_tx;

  doc_tx["id"]   = ID;
  doc_tx["t"]    = timestamp;//millis();
  doc_tx["s1"]   = s1;
  doc_tx["s2"]   = s2;
  doc_tx["s3"]   = s3;

  String json;
  serializeJson(doc_tx, json);
  // uSD_card.appendFile(SD,FILE_NAME,json.c_str());
  // uSD_card.appendFile(SD,FILE_NAME,"\n");
}

void timerISR()
{
  timer_flag = true;
}
