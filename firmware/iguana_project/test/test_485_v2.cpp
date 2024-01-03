
#include <Arduino.h>

#define SERIAL_RS485 Serial2
#define RS485_BR 	 4800

#define LED          2 // on board led
#define I2C_SCL      22
#define I2C_SDA      21

#define SD_CS        5 //VSPI
#define SD_MISO      19
#define SD_MOSI      23
#define SD_CLK       18

#define RFM_CS       15 //HPSI
#define RFM_RST      25
#define RFM_DIO0     26
#define RFM_DIO1     27

#define RS485_TX     16 //HARDWARE SERIAL 2
#define RS485_RX     17 //HARDWARE SERIAL 2
#define RS485_EN     4
#define V_EN		 33

#define PUMP       	13
#define RGB        	14
#define SENS_TEMP  	32
#define SENS_M    	34
#define BAT_LVL    	35

//NTU/SST
//byte rs485_trama[8] = {0x01,0x03,0x00,0x00,0x00,0x02,0xC4,0xB0};//
//byte rs485_trama[8] = {0x01,0x03,0x00,0x01,0x00,0x01,0xD5,0xCA};//
byte rs485_trama[8] = {0x01,0x03,0x00,0x00,0x00,0x01,0x84,0x0A};//
byte rs485_rcv_buff [15];
byte rs485_temp_buff[4] , rs485_header_buff[3], rs485_val_buff[2];
int moisture_int 	= 0;
float moisture 	= 0.0;

void setup()
{
	Serial.begin(115200);
  	SERIAL_RS485.begin(RS485_BR, RS485_TX, RS485_RX);
	pinMode(LED,OUTPUT);
  	pinMode(V_EN,OUTPUT);
	pinMode(RS485_EN,OUTPUT);

	digitalWrite(V_EN  ,HIGH);
	digitalWrite(RS485_EN  ,LOW);

	Serial.println("Setup Ready");
}

void loop()
{
	//TRY WITH 410 TRAMA
    digitalWrite(RS485_EN,HIGH);
    SERIAL_RS485.write(rs485_trama,8);
    SERIAL_RS485.flush();
    digitalWrite(RS485_EN,LOW);
    delay(100);


  	  SERIAL_RS485.readBytes(rs485_rcv_buff,7);
  	  for (int i = 0; i<8;i++)
  	  {
  		  Serial.print(rs485_rcv_buff[i], HEX); // Print the received byte in HEX format
  		  Serial.print(",");


  	  }
    Serial.println();

    for(int i = 0; i<3 ; i++){ rs485_header_buff[i] = rs485_rcv_buff[i];}
    for(int i = 0; i<2 ; i++){ rs485_val_buff[i] 	= rs485_rcv_buff[i];}

    //CONVERT DATA TO FLOAT
    moisture_int = int(rs485_val_buff[0]<<8 | rs485_val_buff[1]);
    Serial.println(moisture_int);
    Serial.println(moisture_int/100.0);
    delay(30000);

}
