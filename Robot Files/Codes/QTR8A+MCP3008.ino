#include <Adafruit_MCP3008.h>

Adafruit_MCP3008 adc;

int numSensors = 8;//Numero de sesnores sendo utilizados
float valorSensor[8];//array de valores do sensor
float posit = 0; //posição do robô em relação a linha
bool whiteLine = true;

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
 
for(int i=0; i<numSensors; i++){
    valorSensor[i] = adc.readADC(i);
    if(whiteLine){
      valorSensor[i] = 1023-valorSensor[i]; //se a linha for branca no fundo preto
    }    
  }  
  posit = GetPosition(valorSensor);
 Serial.println (posit);
}

float GetPosition(float sensorsValues[]){//recebe o array de valores lidos
  float pos=0;
  float denom=0, numer =0;
  int num = sizeof(sensorsValues);

  for(int i=0; i<numS;i++){
    numer= i*sensorsValues[i] + numer;
    denom= sensorsValues[i] + denom;
  }
  pos = numer/denom;

  return pos; //retorna a posição em relação a linha
}


