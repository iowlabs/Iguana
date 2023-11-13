/*
# SPDX-FileCopyrightText: 2023 iowlabs <contacto@iowlabs.com>
#
# SPDX-License-Identifier: GPL-3.0-or-later.txt
*/


/*
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

iowIguana iguana = iowIguana();
uint8_t status;
String output;

void setup()
{
	Serial.begin(115200);
  	iguana.activateSTH();
  	iguana.activateSoilTemp();
  	iguana.activateSoilMoisture();

	status = iguana.begin();
	printlnd(status);
  	delay(200);
  	iguana.iowLogo();
  	delay(3000);
  	iguana.showLogo();
  	delay(3000);

	Serial.println("setup ready");

}

void loop()
{
  iguana.readSensors();
  output = iguana.pubData();
  Serial.println(output);
  //iguana.saveData();
  iguana.showData(2500);

}
