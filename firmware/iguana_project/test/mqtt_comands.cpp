/*
# SPDX-FileCopyrightText: 2024 iowlabs <contacto@iowlabs.com>
#
# SPDX-License-Identifier: GPL-3.0-or-later.txt
*/


/*
  Project     : Generic sensor - mqtt
  Description : This code read data from a generic group of sensors and send it
  to a broker mqtt. It use wifi as a client.
  - A Timer trigger the data adquisition.
  - The function processCmd allow use internalfunctions triggereds by mqtt commands.
  - It count with OTA feature.
  - Every time the node get data, it store it locally on a generic SD.


  Authors:
    WAC@IOWLAB
	Cristobal@IOWLABS

  Features:
  - SD module
  - RTC implemented by RV8806 i2c RTC
  - mqtt
  - timer
  - OTA
  - Sensors: GENERICs
  Connections (USED PIN MAP):
    -----------------------------

  Comunicationmunication:
    - Json structure:
      RCVED MSG:
      {"id":"NODE_ID" "cmd":"CMD_RX","arg":1}
      SENDED MSG:
      {"id":"aq_01", "t" : 456739, "c1": 11.23 ,"c2":224.5, "f1":230.34,"f2":123.123 ,"l": 11.23,"resp":"ok"}
    - List of cmds (add cmds here)
      - RESET: reset the micro
      - v : get version.
          arg = 1   print by serial the actual version of the code
      - led:  turn of the on bord LED. used for debugging.
          arg = 0  LED off
          arg = 1  LED on
      - pub: Trigger a single sample at the recived msg time.
         arg = 1
  LIBS:
  the code need the following libraries (all of them availables in the official
  arduino repo):
  - ArduinoJson  	bblanchon/ArduinoJson @ ^6.18.0
  -					knolleary/PubSubClient@^2.8


*/

//-------------
//  LIBS
//-------------

#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <SparkFun_RV8803.h>
#include "uSD_iow.h"
#include <HTTPClient.h>
#include <HTTPUpdate.h>
#include "WiFiClientSecure.h"
//add the libraries for the used sensors


//-------------
//  DEFINES
//-------------

// PINS
#define LED       2
#define SD_CS     5


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

//MQTT
#define MQTT_PORT         1883
#define MQTT_USER         "iowlabs"
#define MQTT_PASSWORD     "!iow_woi!"
#define MQTT_PUBLISH_CH   "Iguana/response"
#define MQTT_RECEIVER_CH  "Iguana/cmd"
// OTAUPDATE
//#define URL_fw_Bin "https://raw.githubusercontent.com/Iowlabs/Iguana/main/firmware/OTA_bin_file/firmware.bin"

const char* URL_fw_Bin;

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
USD_IOW uSD_card(SD_CS);

//Wifi and mqtt client
WiFiClient wifiClient;
PubSubClient mqtt(wifiClient);
WiFiClientSecure client;

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
const char* ssid      = "iownwater";
const char* password  = "temp3_NL156$";
const char* mqtt_server = "35.223.234.244";

int  mqtt_try       = 0; // Cantidad de intentos para conectarse
bool mqtt_connected = false;
bool wifi_status = false;

//-------------
// FUNCTIONS
//-------------


void setupWiFi();
bool reconnect();

void readSensors();
void readSensor1();
void readSensor2();
void readSensor3();

void writeDatalogger();

void mqttCallback(char* topic, byte* payload, unsigned int length);
void processCmd(byte* payload, unsigned int length);
String publishData();
void publishResponse();
void publishMqtt(char *data);
void firmwareUpdate();

void IRAM_ATTR timerISR();

void setup()
{
  	pinMode(LED,OUTPUT);
  	pinMode(SD_CS, OUTPUT);

  	digitalWrite(SD_CS,1);

  	Wire.begin();
  	Serial.begin(BR_DEBUG);
  	delay(500);

	#if DATALOGGER
  		if (rtc.begin() == false)
  		{
    		printlnd("Device not found. Please check wiring.");
  		}

  		// un comment for setup time sec, min, hour, dayof the week, date, month, year
  		/*
  		if (rtc.setTime(0, 38, 18, 6, 30, 6, 2023) == false)
  		{
    		Serial.println("Something went wrong setting the time");
  		}
  		*/
  		//INIT SD

  		uSD_card.uSD_init();
  		delay(200);
	#endif

  	setupWiFi();
	mqtt.setServer(mqtt_server,MQTT_PORT);
	mqtt.setCallback(mqttCallback);

	//setup Timer
	timer = timerBegin(3, 8000, true);          // timer 3, prescaler 8000, counting up
	timerAttachInterrupt(timer,&timerISR,true); // params: timer object, pointer to ISR function anddress, mode edge (if false: level mode)
	timerAlarmWrite(timer, timer_counter,true); // params: timer object, counter_limit, restart counter on top.// to get T= 1s, n=T*f_{timer source clock}
	timerAlarmEnable(timer);                    // enable CTC mode

  	printd("setup ready");

}

void loop()
{
	if(timer_flag)
	{
    	readSensors();
		output = publishData();
		printlnd(output);
		publishMqtt((char*) output.c_str());
  	}
	mqtt.loop();
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

// Function to reconnect to the MQTT broker
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
  	}
  	mqtt.publish(MQTT_PUBLISH_CH, payload);
}

void mqttCallback(char* topic, byte* payload, unsigned int length)
{
	if(DEBUG)
  	{
    	Serial.println("-------new message from broker-----");
    	Serial.print("channel:");
    	Serial.println(topic);
    	Serial.print("data:");
    	Serial.write(payload, length);
    	Serial.println();
  	}
  	processCmd(payload,length);
}

void readSensors()
{
	readSensor1();
  	readSensor2();
  	readSensor3();
	#if DATALOGGER

  		if (rtc.updateTime() == true) //Updates the time variables from RTC
  		{
    		timestamp = rtc.getEpoch(); //Get the time in UNIX
  		}
 		//writeDatalogger();
	#else
		timsestamp =	millis();
	#endif
}

void readSensor1()
{
	s1 = random(0,100)+random(0,100)/100.0;
  	return;
}

void readSensor2()
{
	s2 = random(0,100)+random(0,100)/100.0;
  	return;
}

void readSensor3()
{
	s3 = random(0,100)+random(0,100)/100.0;
  	return;
}

/*
  get the avg of each buff and then Publish the json data structure by Lora.
  (For debug, also send the json msg on usb serial)
*/
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


/*
  this function only send a respone type of data.
  used for communication.
*/
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

/*
  Parse the received json message and procces the recived commands
*/
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

void firmwareUpdate(void)
{
  //WiFiClientSecure client;
  client.setCACert(cacert);
  client.setCertificate(clientCert);
  client.setPrivateKey(clientKey);
  //httpUpdate.setLedPin(LED_BUILTIN, LOW);
  httpUpdate.setLedPin(2, LOW);
  Serial.println("Updating firmware...");
  t_httpUpdate_return ret = httpUpdate.update(client, URL_fw_Bin);

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
  uSD_card.appendFile(SD,FILE_NAME,json.c_str());
  uSD_card.appendFile(SD,FILE_NAME,"\n");
}

void timerISR()
{
  timer_flag = true;
}
