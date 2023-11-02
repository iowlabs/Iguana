/*
# SPDX-FileCopyrightText: 2023 iowlabs <contacto@iowlabs.com>
#
# SPDX-License-Identifier: GPL-3.0-or-later.txt
*/

#ifndef __IOWIGUANA__
#define __IOWIGUANA__

#include <Arduino.h>
#include <DallasTemperature.h>
#include <OneWire.h>
#include <Wire.h>
#include <SPI.h>
#include <DHT.h>
#include "Adafruit_SHT31.h"
#include <ArduinoJson.h>

/*-------PINs OF IGUANA BOARD------*/
#define LED          2 // on board led
#define I2C_SCL      22
#define I2C_SDA      21
#define SD_CS        5
#define PH_EN        33
#define ORP_EN       32
#define OD_EN        12
#define EC_EN        13
#define RFM_CS       15
#define RFM_RST      4
#define RFM_DIO0     17
#define RFM_DIO1     14

#define PH_ADDRESS    100
#define ORP_ADDRESS   101
#define TEMP_ADDRESS  102
#define OD_ADDRESS    103
#define EC_ADDRESS    104

#define DISPLAY_ADDRESS 0x3C
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define OLED_RESET     -1 // Reset pin # (or -1 if sharing Arduino reset pin)

#define ID              "n01"
#define RESPONSE_OK     "OK"
#define RESPONSE_ERROR  "ERROR"
#define FILE_NAME       "/log.txt"

#define STATUS_OK     0
#define STATUS_ERROR  1
#define ERROR_SD      0b00000001
#define ERROR_RTC     0b00000010
#define ERROR_OLED    0b00000100
#define ERROR_LORA    0b00001000



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



#endif