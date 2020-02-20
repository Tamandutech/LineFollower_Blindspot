
#include <Adafruit_MCP3008.h>

Adafruit_MCP3008 adc;

//Setup do array QTR
int numSensors = 8;//Numero de sesnores sendo utilizados
float valorSensor[8];//array de valores do sensor
float posit = 0; //posição do robô em relação a linha
float MaxQTRValues[8]=  {0, 0, 0, 0, 0, 0, 0, 0};
float MinQTRValues[8]=  {1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023};; //armazenar valores máximos e mínimo para a calibração.
bool whiteLine = true;
float idealPosition = ((numSensors-1)*1000)/2;

void setup() {

Serial.begin(9600);
//------------------------------
while (!Serial);
// Software SPI (specify all, use any available digital)
// (sck, mosi, miso, cs);
adc.begin(10, 12, 11, 2);
Serial.println("start calibrate: ");
CalibrateArraySensors();
}

void loop() {
 ReadArraySensor();
 posit = GetPosition(valorSensor);
 Serial.println (posit);
}

void ReadArraySensor (){

  for(int i=0; i<numSensors; i++){
    float aux =0;
    aux = adc.readADC(i); 
    if(aux>MaxQTRValues[i]) aux = MaxQTRValues[i];
    if(aux<MinQTRValues[i]) aux = MinQTRValues[i];   
    valorSensor[i] = 1000 * ((aux-MinQTRValues[i]) /(MaxQTRValues[i]-MinQTRValues[i]));
    //Serial.print(valorSensor[i]);
    //Serial.print("    ");   
    if(whiteLine){
       valorSensor[i] = 1000-valorSensor[i]; //se a linha for branca no fundo preto
    }
    //Serial.print(valorSensor[i]);
    //Serial.print("    ");     
  }
  //Serial.println(" ");
}

float GetPosition(float sensorsValues[]){
  float pos=0;
  float denom=0, numer =0;

  for(int i=0; i<numSensors;i++){
    numer= (i*1000*sensorsValues[i]) + numer;
    denom= sensorsValues[i] + denom;

    /*Serial.print(i);
    Serial.print("  ");
    Serial.print("numer");
    Serial.print(numer);

    Serial.print("demon"); 
    Serial.println(denom); */
  }
  pos = numer/denom;

  if(pos<0) pos=0;
  if (pos>(numSensor-1*)1000) pos = (numSensor-1*)1000;

  return pos; //retorna a posição em relação a linha
}

//calibrate o array de sensores pegando os valores máximo e mínimo que cada um esta lendo
void CalibrateArraySensors() {
for(int i=0; i<200; i++){
  for(int j=0; j<numSensors; j++){ 
    float read = adc.readADC(j);

    if(read < MinQTRValues[j])   MinQTRValues[j]= read;

    if(read > MaxQTRValues[j])   MaxQTRValues[j] = read;
  }
  delay(25);
}
//------- MIN VALUES -----------------
Serial.print(" Min Values: ");
  for(int k=0; k<numSensors; k++){
    Serial.print(" ");
    Serial.print(MinQTRValues[k]);
  }
//------MAX VALUES ------------------
Serial.println(" ");
Serial.print("Max Values: ");
  for(int l=0; l<numSensors; l++){
    Serial.print(" ");
    Serial.print(MaxQTRValues[l]);
  }
 Serial.println(" ");
}
