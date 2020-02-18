#include <Adafruit_MCP3008.h>

Adafruit_MCP3008 adc;
#define LED_STATUS              13

//Setup dos sensores parada
#define stopSensor1 A4
#define stopSensor2 A5
int stopCount =0;
int stopSensorValue1 = 0, stopSensorValue2 = 0;        // valores iniciais dos sensores 1 e 2
int trigger1 =0, trigger2= 0;                        //valor que ativa a contagem dos sensores 1 e 2
int instersecs=0;                                      //valor de instersecções na pista

//setup dos sensores curva
#define curveSensor1 A6
#define curveSensor2 A7
int curvesCount =0;
int curveSensorValue1 = 0;        // valor inicial do sensor1
int curveSensorValue2 = 0;        // valor inicial do sensor2

//Setup do array QTR
int numSensors = 8;//Numero de sesnores sendo utilizados
float valorSensor[8];//array de valores do sensor
float posit = 0; //posição do robô em relação a linha
bool whiteLine = true;
float idealPosition = ((numSensors-1)*1000)/2;

//Motors Config
int STBY = 6; //standby

// Motor A,
const uint8_t pwmLeft = 3;      // ENA - Enable and PWM
const uint8_t leftForward = 5;  // IN1 - Forward Drive
const uint8_t leftReverse = 4;  // IN2 - Reverse Drive
 
// Motor B,
const uint8_t pwmRight = 9;     // ENB - Enable and PWM
const uint8_t rightForward = 7; // IN3 - Forward Drive
const uint8_t rightReverse = 8; // IN4 - Reverse Drive

//Speed configs
int rightMaxSpeed =255;; // velocidade máxima da roda direita
int leftMaxSpeed =255;// velocidade máxima da roda esquerda
int rightBaseSpeed= 150; // velocidade base da roda direita
int leftBaseSpeed =150; // velocidade base da roda esquerda
int rightMinSpeed =2; // velocidade min da roda direita
int leftMinSpeed =2;

//Temporizadores
unsigned long previousMillis = 0;  
unsigned long currentMillis=0;
const long interval = 400; 
int timeStop=32000;

//PID configs
 float last_proportional = 0;
 float proportional = 0;
 float derivative = 0;
 float integral = 0;

//-------------------------------------Setup-------------------------------------------------
void setup() {
Serial.begin(9600);

//-------setup dos motores--------------- 
pinMode(STBY, OUTPUT);
pinMode(pwmLeft, OUTPUT);
pinMode(leftForward, OUTPUT);
pinMode(leftReverse, OUTPUT);
pinMode(pwmRight, OUTPUT);
pinMode(rightForward, OUTPUT);
pinMode(rightReverse, OUTPUT);
digitalWrite(STBY, HIGH); 
//------------------------------

while (!Serial);

// Software SPI (specify all, use any available digital)
// (sck, mosi, miso, cs);
adc.begin(10, 12, 11, 2);
}


void loop() {
currentMillis = millis();//iguala o valor de current ao valor real de milis

if((currentMillis - previousMillis) >= interval){
  CountStops();
  previousMillis = currentMillis;
}

/*------ Se desejar parar por tempo, descomentar esta função
if(currentMillis >= stopTime){
  StopRobot();
}*/

for(int i=0; i<numSensors; i++){
    valorSensor[i] = adc.readADC(i);
    if(whiteLine){
      valorSensor[i] = 1023-valorSensor[i]; //se a linha for branca no fundo preto
    }    
  }  
  posit = GetPosition(valorSensor);
 Serial.println (posit);
}

//calcula a posição do robô em relação a linha
float GetPosition(float sensorsValues[]){
  float pos=0;
  float denom=0, numer =0;
  int num = sizeof(sensorsValues);

  for(int i=0; i<num;i++){
    numer= i*sensorsValues[i] + numer;
    denom= sensorsValues[i] + denom;
  }
  pos = numer/denom;

  return pos; //retorna a posição em relação a linha
}
//----conta as intersecções e faixas de parada da pista----
bool CountStops(){
  stopSensorValue1 = analogRead(stopSensor1);
  stopSensorValue2 = analogRead(stopSensor2);

  if(stopSensorValue1 > trigger1 && stopSensorValue2 > trigger2){
    stopCount++;
  }
  if(stopCount >= instersecs){
    StopRobot();
  }
}

void StopRobot(){ //para o robô 
  analogWrite(pwmLeft,0);
  analogWrite(pwmRight,0);
  digitalWrite(leftForward, HIGH);
  digitalWrite(rightForward, HIGH);
  digitalWrite(leftReverse, HIGH);
  digitalWrite(rightReverse, HIGH);
  delay(20000);
}
