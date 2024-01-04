/*
# SPDX-FileCopyrightText: 2023 iowlabs <contacto@iowlabs.com>
#
# SPDX-License-Identifier: GPL-3.0-or-later.txt
*/

/*
TODO-
- Display % de battery
- Implement autoconect to handle Wifi connection
- Implement calibration function for moisture sensors
- Instructions for case as de armado de caja
- Instrucciones de inicio y uso.
- Test rápidos para verificar funcionamiento
- Modo subida de datos
- Modo red local
- Modo solo pantalla
- Sleep Mode
- modo periodico.

*/

#include <Arduino.h>
#include "iowIguana.h"

iowIguana::iowIguana()
{

}

uint8_t iowIguana::begin()
{
  	pinMode(LED,OUTPUT);
  	pinMode(V_EN,OUTPUT);

  	pinMode(SD_CS, OUTPUT);
  	pinMode(RFM_CS  ,OUTPUT);
  	pinMode(RFM_RST ,OUTPUT);
  	pinMode(RFM_DIO0,OUTPUT);
  	pinMode(RFM_DIO1,OUTPUT);

	pinMode(PUMP,OUTPUT);
	pinMode(RGB,OUTPUT);
	pinMode(RS485_EN,OUTPUT);


  	digitalWrite(V_EN  ,HIGH);
  	digitalWrite(SD_CS ,HIGH);
  	digitalWrite(RFM_CS ,HIGH);
  	digitalWrite(RFM_RST,HIGH);
	digitalWrite(PUMP,LOW);
	digitalWrite(RS485_EN,LOW);

	//SERIAL_RS485(RS485_TX, RS485_RX);

	SERIAL_RS485.begin(RS485_BR);

  	Wire.begin();

	FastLED.addLeds<NEOPIXEL, RGB>(rgbLED, 1);  // GRB ordering is assumed

  	if(rtc.begin()) rtc_status = true;
  	if(iowsd.uSD_init(SD_CS)) sd_status = true;
  	if(display.begin(DISPLAY_ADDRESS,true)) display_status = true ;
  	if(lora_sel){
    	LoRa.setPins(RFM_CS, RFM_RST, RFM_DIO0);
    	if(LoRa.begin(915E6)) lora_status = true;
    	LoRa.setSyncWord(0xF3);
  	}
	if(sht_sel)
	{
		if(sensor_sht31.begin(0x44)) sht_status = true;
	}
	if(st_sel)
	{
		sensor_temp.begin();
		st_status = true;
	}
	if(rs485_sel)
	{
		SERIAL_RS485.begin(RS485_BR);
	}
	display.clearDisplay();
  	display.setRotation(0); //IF NOT INVERTED COMMENT THIS LINE
	display.setTextColor(SH110X_WHITE);
  	display.setTextSize(2);
	// display.setFont(); //If not in use, comment

  	if( rtc_status && display_status && sd_status && !(sht_sel^sht_status) && !(st_sel^st_status) )
	{
		rgbLED[0]= CRGB::Blue;
		FastLED.show();
		return STATUS_OK;
	}
  	else
	{
		rgbLED[0]= CRGB::Red;
		FastLED.show();
		return STATUS_ERROR;
	}
}

void iowIguana::iowLogo()
{
  display.clearDisplay(); // Clear the buffer
  display.drawBitmap(0, 0, logo_iowlabs, 128, 64,1,0);
  display.display();
}

void iowIguana::showLogo()
{
  display.clearDisplay(); // Clear the buffer
  display.drawBitmap(0, 0, logo_iguana, 128, 64,1,0);
  display.display();
}

void iowIguana::activateRS485()		{rs485_sel 	= true;}
void iowIguana::activateSTH()		{sht_sel 	= true;}
void iowIguana::activateSoilTemp()	{st_sel 	= true;}
void iowIguana::activateSoilMoisture(){sm_sel 	= true;}
void iowIguana::activateLoRa()		{lora_sel 	= true;}

void iowIguana::activateAll()
{
  	rs485_sel 	= true;
  	sht_sel   	= true;
  	st_sel  	= true;
  	sm_sel    	= true;
  	lora_sel    = true;
}

// This function must be conditioned for each modbus sensor
void iowIguana::readRS485()
{
	
	//TRY WITH 410 TRAMA
    digitalWrite(RS485_EN,HIGH);
    SERIAL_RS485.write(soilSensorRequest, sizeof(soilSensorRequest));
    SERIAL_RS485.flush();
    digitalWrite(RS485_EN,LOW);
    delay(100);
	
	// NUEVOOO

	unsigned long startTime = millis();
	while (SERIAL_RS485.available() < 7 && millis() - startTime < 1000)
	{
		delay(1);
	}
	
	if (SERIAL_RS485.available() >= 7) // If valid response received
	{
		// Read the response from the sensor
		byte index = 0;
		while (SERIAL_RS485.available() && index < 7)
		{
		soilSensorResponse[index] = SERIAL_RS485.read();
		Serial.print(soilSensorResponse[index], HEX); // Print the received byte in HEX format
		Serial.print(" ");
		index++;
		}
		Serial.println();
 
		// Parse and calculate the Moisture value
		int Moisture_Int = int(soilSensorResponse[3] << 8 | soilSensorResponse[4]);
		float Moisture_Percent = Moisture_Int / 10.0;
 
		Serial.print("Moisture: ");
		Serial.print(Moisture_Percent);
		Serial.println(" %RH\n");
		rs485_moisture = Moisture_Percent;
 
 
	}
	else
	{
		Serial.println("Sensor timeout or incomplete frame");
	}

}

void iowIguana::readSTH()
{
	ambient_temp 	= sensor_sht31.readTemperature();
	ambient_h 		= sensor_sht31.readHumidity();
	//ADD CHECK DATA
}

void iowIguana::readSoilTemperature()
{
	sensor_temp.requestTemperatures();
    soil_temp = sensor_temp.getTempCByIndex(0);
	//ADD CHECK DATA
	//if(soil_temp != DEVICE_DISCONNECTED_C)
	//{
	//	st_status = false;
	//	soil_temp = 0.0;
	//}
}

void iowIguana::readSoilMoisture()
{
	for(int i =0; i<N_AVG_SAMPLES;i++)
	{
	  sum_adc += analogRead(SENS_M);
	}
	soil_moisture_val = sum_adc/N_AVG_SAMPLES;
	// soil_moisture = (soil_moisture_m)*(soil_moisture_val - MOISTURE_WATER_VALUE) + 100; //EC. recta

	// if(soil_moisture < 0.0) soil_moisture = 0; //constrain min
	// if(soil_moisture > 100.0) soil_moisture = 100; //constrain MAX
	soil_moisture = soil_moisture_val;

	soil_moisture_val 	= 0;
	sum_adc 			= 0;
}

void iowIguana::readSensors()
{
	if(sht_sel){readSTH();}
	if(sm_sel ){readSoilMoisture();}
	if(rs485_sel){readRS485();}
	if(st_sel ){readSoilTemperature();}
	if (rtc.updateTime() == true) //Updates the time variables from RTC
  	{
    	timestamp = rtc.getLocalEpoch(true) - 220678400; //Get the time in UNIX
  	}
}

String iowIguana::pubData(void)
{
  	StaticJsonDocument<128> doc_tx;

  	doc_tx["id"]    = ID;
  	doc_tx["time"]  = timestamp;
  	if(sht_sel) 	doc_tx["ta"]  = ambient_temp;
  	if(sht_sel) 	doc_tx["ha"]  = ambient_h;
  	if(st_sel)  	doc_tx["ms"]  = soil_moisture;
  	if(sm_sel)  	doc_tx["ts"]  = soil_temp;
	if(rs485_sel) 	doc_tx["m"]   = rs485_moisture;

  	String json;
  	serializeJson(doc_tx, json);

  	if(lora_sel)
  	{
    	delayMicroseconds(random(1000)); // random delay for avoid collisions
    	LoRa.beginPacket();
    	LoRa.print(json);
    	LoRa.endPacket();
  	}
  	return json;
}

void iowIguana::saveData(void)
{
	StaticJsonDocument<128> doc_tx;

  	doc_tx["id"]    = ID;
  	doc_tx["time"]  = timestamp;
  	if(sht_sel) doc_tx["ta"]  = ambient_temp;
  	if(sht_sel) doc_tx["ha"]  = ambient_h;
  	if(st_sel)  doc_tx["ts"]  = soil_moisture;
  	if(sm_sel)  doc_tx["ms"]  = soil_temp;
	if(rs485_sel) 	doc_tx["m"]   = rs485_moisture;

	String json;
  	serializeJson(doc_tx, json);

  	iowsd.appendFile(SD,FILE_NAME,json.c_str());
	iowsd.appendFile(SD,FILE_NAME,"\n");
}

void iowIguana::showStatus(void)
{
 	return;
}

void iowIguana::showData(long time_interval)
{
	display.clearDisplay();

	display.setTextSize(1);
	display.setCursor(0,0);
	display.println("Iguana V.1.0.1");

	display.setTextSize(2);
	display.setCursor(16, 16);
	display.println("ST:");
	display.setCursor(52, 16);
	display.print(soil_temp);

	display.setCursor(16, 32);
	display.println("SM:");
	display.setCursor(52, 32);
	display.print(soil_moisture);
	display.display();
	delay(time_interval);


	display.clearDisplay();

	display.setTextSize(1);
	display.setCursor(0,0);
	display.println("Iguana V.1.0.1");

	display.setTextSize(2);
	display.setCursor(16, 16);
	display.println("T:");
	display.setCursor(52, 16);
	display.print(ambient_temp);

	display.setCursor(16, 32);
	display.println("H:");
	display.setCursor(52, 32);
	display.print(ambient_h);

	display.display();
	delay(time_interval);

}

void iowIguana::firmwareUpdate(void)
{
  //WiFiClientSecure client;
  client.setCACert(rootCACertificate);
  httpUpdate.setLedPin(LED_BUILTIN, LOW);
  t_httpUpdate_return ret = httpUpdate.update(client, URL_fw_Bin);

  switch (ret) {
  case HTTP_UPDATE_FAILED:
    Serial.printf("HTTP_UPDATE_FAILD Error (%d): %s\n", httpUpdate.getLastError(), httpUpdate.getLastErrorString().c_str());
    break;

  case HTTP_UPDATE_NO_UPDATES:
    Serial.println("HTTP_UPDATE_NO_UPDATES");
    break;

  case HTTP_UPDATE_OK:
    Serial.println("HTTP_UPDATE_OK");
    break;
  }
}