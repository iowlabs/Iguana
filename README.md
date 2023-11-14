# iowLabs Iguana

![Foto real](3Dmodel/V1/photo_2.jpg)

Iguana es un sensor de LoRa para smartCitys. Es un instrumento diseñado para el monitoreo de la temperatura y humedad de suelo en tiempo real. De bajo costo y diseñado para adaptarse a diversos escenarios y diferentes tipos de reds y topologías de red inalambricas para integrarlo a redes IoT.

La placa fue diseñada para considerar un gran numero de los posibles escenarios que engloban las necesidades comunes asociadas al regadío automático el cutlivo inteligente y el monitoreo de la calidad de suelo. Tanto a nivel de aplicaciones de hogar como a niveles de aplicaciones industriales.

El sensor considera una compatibilidad con los siguientes sensores:

- Sensor de humedad de suelo.ds18b20
- Sensor de temperatura de suelo.
- Sensor de humedad y temperatura ambiente.


Ademas cuenta con una driver para una bomba, un módulo LoRa, un módulo RTC y una SD para funciones de datalogger y una pantalla OLED para visualizar la información instantanea. Otro de los atributos interesantes, es que cuenta con un conversor RS485, que permite integrar el dispositivo iguana con sensores de caracter industriales para el monitoreo de la humedad y calidad de suelo que habitualmente son compatibles con protocolos ModBus RS485.  

**IoT**,**LoRa**, **Environmental measurement**, **Soil mositure**, **datalogger**, **MODBUS RS485**.

### OSHW LICENSES


 ![OSL](LICENSES/OSL.png)  


## Hardware description

The PCB was designed in KiCad7 stable realease and additional components used can be found in the official [iowLabs](https://github.com/iowlabs/KiCad_Lib) library.

Cuenta con un microcontrolador ESP32, un módulo RF95 para integración a redes LoRa y LoRaWAN.  



Cuenta con un RTC RV8803 y un socket para tarjetas micro SD necesarios para implementar funcionalidades de data-logger.
Adicionalmente cuenta con un módulo "max485" para integrar sondas para monitoreo calidad de suelo de caracter industrial basados en protocolo de cómunicación MODBUS RS485.

Un driver sencillo permite integrar bombas pequeñas de riego para implementar funcionalidades de riego automático y control de riego remoto.

Una pantalla OLED y un LED RGB permiten visualizar información en forma real sobre los sensonres.

Cuenta también con un modulo de carga para baterías de lipo basado en el integrado TP4056 y un circuito de protección de carga. Y fácil integración con aplicaciones basadas en paneles solares.

Para administrar la energía del circuito se utiliza un conversor DC DC MT3608 y un arreglo de reguladores que alimentan la electrónica digital, los sensores y el datalogger de forma independiente. Los reguladores que alimentan las dependencias pueden ser desabilitados por medio de firmware para implementar funciones de bajo consumo.

### Dependencies
The board is based on an ESP32 microcontroller.

- USB communication through IC CH340
- Lora rfm95 module.
- microSD socket.
- RV8803 RTC module.
- max485 for RS485 communication.
- On board pump driver.
- On board battery charger compatible with 3.7V lipo bateries, based on TP4056.
- ws2812 RGB LED
- OLED display


### Schematic

![Detalle del esquemático de la placa phecda](hardware/output_files/Iguana_V1.svg)

### Layout
La tarjeta diseñada tiene dimensiones de 8x5cm. Fue diseñada en una PCB estandar de 1.6mm de dielectrico de dos capas.

| Top view | Bottom view |
| -------- | ----------- |
| ![front view of the pcb form kicad](hardware/output_files/Iguana_V1-top.svg)|![back view of the pcb form kicad](hardware/output_files/Iguana_V1-bottom.svg)|


### BOM

The bill of materials with respective references to LCSC part numbers de los componentes que componen la PCB de iguana can be found at the following [link](hardware/output_files/Iguana_board.csv).

A continuación se agrega una lista de los componentes y  las sondas y sensores que complementan el dispositivo iguana para implementar todas sus funcionalidades. Los precios entregados y los links sirven solo como referencia.


|  Componente 	| Precio ref 	| Descripcion 	| Link 		|
| ----------- 	| ----------- 	| ------------- | ---------	|
| Sensor de humedad de suelo | 				| 	Sensor de humedad de suelo capacitivo	|			|
| Sensor de temeperatura de suelo| 	| Sonda sensor ds18b20 | |
| Sensor de humedad y temperatura ambiente | | SHT31 - case outdoor IP68 | |
| Sensor de Temperatura y humedad de suelo industrial | | Sensor MODBUS 485 capacitivo, de humedad y temperatura de suelo. | |
| Pantalla OLED |  | 128x64 1.3'' | |
| Batería LiPo  |  | Batería lipo 3.7V 2500mAh.Tamaño menor a 65x55cm.  | |
| Tarjeta microSD | | 16GB | |
| Batería 2012 	| | Batería 3.3V CR2012 | |



## 3D Model

![Foto real](3Dmodel/V1/photo_1.jpg)

![Render 3D de la placa phecda](hardware/output_files/Phecda_board.png)

Hemos diseñado un casing para intengrar la electrónica. Este diseño es un case simple compuesto de dos partes diseñado para ser impreso en PLA. El objetivo principal de este case es permitir el uso del dispositivo para aplicaciones de hambientes controlados como cultivos particulares e invernaderos. Para aplicaciones de outdoor recomendamos utilizar un case comercial que cuente con una IP65 o superior.  

![Enclousure diseñado para phecda](3Dmodel/V2/full_view.png)

The design allows integrar la pantalla OLED y el LED RGB para monitorear el estatus de los sensores. Deja a disposición del usuario los conectores para la comunicación con las sondas, el conector SMA para la antena LoRa y un boton para el encendido y apagado del dispositivo.


| Cover | Base |
| -------- | ----------- |
| ![componente superior del case](3Dmodel/V2/top_v2.png)|![componente inferior del case](3Dmodel/V2/base_v2.png)|

El dispositivo considera una batería lipo 3.7V lipo de 65x54 cm para la alimentación. Esta batería puede ser cargada de forma externa por medio del conector USB o por medio de la entrada de carga.

Editable files and display plans can be found in [directorio](3Dmodel/).

The PCB design allows it to fit commercial project enclosures with IP65 standards or higher for outdoor or harsh weather applications.

## Firmware

The developed firmware represents a collection of libraries and drivers for the board's dependencies.

This collection of useful functions is implemented in the iowIguana library. However, due to the flexibility of the design, it is possible to load a wide variety of code based on other supported libraries.

The firmware was developed using PlatformIO; however, usage examples were generated for [Arduino IDE] (https://www.arduino.cc/en/software).

|  Folder  | Description |
| -----------| ----------- |
| [firmware/iguana_project](firmware/iguana_project) | The complete project compiled in its last version.|
| [firmware/examples](firmware/examples) | Arduino compatible example codes .|
| [firmware/src](firmware/src) | The source files of the library made for iguana.|

### Used libraries
Except for the uSD_iow library which is available in the official iowlabs repository, all other libraries used are available from the official Arduino and PlatformIO library manager.

- RTC 			: sparkfun/SparkFun Qwiic RTC RV8803 Arduino Library @ ^1.2.8
- Temperature sensor	: paulstoffregen/OneWire@^2.3.7
- Temperature sensor 	: milesburton/DallasTemperature@^3.11.0
- Temperature and humidity sensor : adafruit/Adafruit SHT31 Library@^2.2.2
- Temperature and humidity sensor : adafruit/Adafruit BusIO@^1.14.3
- Temperature and humidity sensor : adafruit/Adafruit Unified Sensor@^1.1.13
- ArduinoJson	:	bblanchon/ArduinoJson@^6.21.3
- OLED			: adafruit/Adafruit SH110X @ ^2.1.8
- OLED 			: adafruit/Adafruit GFX Library @ ^1.11.5
- fastLED		: fastled/FastLED@^3.6.0
- LoRa 			: sandeepmistry/LoRa @ ^0.8.0
- MQTT               : knolleary/PubSubClient@^2.8


### Available examples

| File | Description |
|---------|-------------|
| simple_version.ino  | Collect the data from the pH, ORP and temperature sensors, and displays it on the screen. Print the collected data in json format through the serial monitor. |
| low_energy_version.ino |  Adds a low-power mode to the simple_version code |
| lora_version.ino | Activate the LoRa module and send a message in json format. |
| mqtt_version.ino | This is an example of reading the data and sending it via MQTT to a pre-established broker.|

### Available functions

- uint8_t begin(void);
- void readSoilTemperature(void);
- void readSoilMoisture(void);
- void readSTH(void);
- void readRS485(void);
- void readSensors(void);
- String pubData(void);
- void activateSensors(uint8_t sens);
- void activateLoRa(void);
- void activateAll(void);
- void iowLogo(void);
- void showLogo(void);
- void showStatus(void);
- void showData(long time_interval);
- void saveData(void);
- void moistureCal(uint8_t val);
- void temperatureCal(uint8_t val);

## Instruction and usage
