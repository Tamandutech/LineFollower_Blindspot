/***************************************************
Simple example of reading the MCP3008 analog input channels and printing
them all out.

Author: Carter Nelson
License: Public Domain
****************************************************/

#include <Adafruit_MCP3008.h>

Adafruit_MCP3008 adc;
int inverseColor = 1023;
int count = 0;
float ValorSensor0;
float ValorSensor1;
float ValorSensor2;
float ValorSensor3;
float ValorSensor4;
float ValorSensor5;
float ValorSensor6;
float ValorSensor7;
float Posicao = 0;

void setup() {
Serial.begin(9600);
while (!Serial);

Serial.println("MCP3008 simple test.");

  // Hardware SPI (specify CS, use any available digital)
  // Can use defaults if available, ex: UNO (SS=10) or Huzzah (SS=15)
  //adc.begin();
  // Feather 32u4 (SS=17) or M0 (SS=16), defaults SS not broken out, must specify
  //adc.begin(10);  

  // Software SPI (specify all, use any available digital)
  // (sck, mosi, miso, cs);
adc.begin(10, 12, 11, 2);
}

void loop() {
 ValorSensor0 = adc.readADC(7);
 ValorSensor1 = adc.readADC(6);
 ValorSensor2 = adc.readADC(5);
 ValorSensor3 = adc.readADC(4);
 ValorSensor4 = adc.readADC(3);
 ValorSensor5 = adc.readADC(2);
 ValorSensor6 = adc.readADC(1);
 ValorSensor7 = adc.readADC(0);
//Serial.println ();
 Posicao = (0*(inverseColor-ValorSensor0) + 1000*(inverseColor-ValorSensor1) + 2000*(inverseColor-ValorSensor2) + 3000*(inverseColor-ValorSensor3) + 4000*(inverseColor-ValorSensor4) + 5000*(inverseColor-ValorSensor5) + 6000*(inverseColor-ValorSensor6) + 7000*(inverseColor-ValorSensor7))/((inverseColor-ValorSensor0) + (inverseColor-ValorSensor1) + (inverseColor-ValorSensor2) + (inverseColor-ValorSensor3) + (inverseColor-ValorSensor4) + (inverseColor-ValorSensor5) + (inverseColor-ValorSensor6) + (inverseColor-ValorSensor7));
 Serial.println (Posicao);

}
