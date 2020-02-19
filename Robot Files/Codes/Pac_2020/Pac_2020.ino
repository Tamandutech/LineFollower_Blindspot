#include <Adafruit_MCP3008.h>

Adafruit_MCP3008 adc;
#define Led              13

//Setup dos sensores parada
#define stopSensor1 A4
#define stopSensor2 A5
int stopCount =0;
int stopSensorValue1 = 0, stopSensorValue2 = 0;        // valores iniciais dos sensores 1 e 2
int stopTrigger1 =0, stopTrigger2= 0;                        //valor que ativa a contagem dos sensores 1 e 2
int instersecs=0;                                      //valor de instersecções na pista

//setup dos sensores curva
#define curveSensor1 A6
#define curveSensor2 A7
int curvesCount =0;
int curveSensorValue1 = 0, curveSensorValue2 = 0;;        // valor inicial dos sensores esquerdos 1 e 2
int curveTrigger1 =0, curveTrigger2= 0;

//Setup do array QTR
int numSensors = 8;//Numero de sesnores sendo utilizados
float valorSensor[8];//array de valores do sensor
float posit = 0; //posição do robô em relação a linha
float MaxQTRValues[8], MinQTRValues[8]; //armazenar valores máximos e mínimo para a calibração.
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
pinMode(Led, OUTPUT);
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
//------------------------------------------------------INICIO CALIBRAÇÃO----------------------------------
BlinkLed(3, 500); // Pisca o Led para mostrar que a calibração começou
CalibrateSensors(); // inicia rotina de calibração dos sensores
BlinkLed(5, 100); //Pisca o Led para mostrar que a calibração terminou e deve-se colocar o robô na posição
delay(000); //aguarda 2 segundos para colocar o robô.
}

//---------------------------------------------------- LOOPING CODE -------------------------------------------------------------------

void loop() {
currentMillis = millis();//iguala o valor de current ao valor real de milis

if((currentMillis - previousMillis) >= interval){
  CountStops();
  CountCurves();
  previousMillis = currentMillis;
}

/*------ Se desejar parar por tempo, descomentar esta função
if(currentMillis >= stopTime){
  StopRobot();
}*/
 posit = GetPosition(valorSensor);
 Serial.println (posit);
}

void CalibrateSensors() {
//PseudoCode
for(int i=0; i<200; i++){
  for(int j=0; j<8; j++){ 
    float read = adc.readADC(i);

    if(read < MinQTRValues[i])   MinQTRValues[i]= read;

    if(read > MaxQTRValues[i])   MaxQTRValues[i] = read;
  }
}
//------- MIN VALUES -----------------
Serial.print(" Min Values: ");
for(int k=0; k<8; k++){
  Serial.print(" ");
  Serial.print(MinQTRValues[k]);
}
//------MAX VALUES ------------------
Serial.println(" ");
Serial.print("Max Values: ");
for(int l=0; l<8; l++){
  Serial.print(" ");
  Serial.print(MaxQTRValues[l]);
}
}

void ReadArraySensor (){


  for(int i=0; i<numSensors; i++){
    int aux;
    aux = adc.readADC(i);
    if(aux>MaxQTRValues[i]) aux = MaxQTRValues[i];
    if(aux<MinQTRValues[i]) aux = MinQTRValues[i];

    valorSensor[i] = 1000 * (aux / (MaxQTRValues[i]-MinQTRValues[i]));

    if(whiteLine){
       valorSensor[i] = 1000-valorSensor[i]; //se a linha for branca no fundo preto
    }    
  }
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
void CountStops(){
  stopSensorValue1 = analogRead(stopSensor1);
  stopSensorValue2 = analogRead(stopSensor2);

  if(stopSensorValue1 > stopTrigger1 && stopSensorValue2 > stopTrigger2){
    stopCount++;
  }
  if(stopCount >= instersecs){
    StopRobot();
  }
}

void CountCurves(){ //conta os marcadores de curva
  curveSensorValue1 = analogRead(curveSensor1);
  curveSensorValue2 = analogRead(curveSensor2);

  if(curveSensorValue1 > curveTrigger1 && curveSensorValue2 > curveTrigger2){
    curvesCount++;
  }
  
}

int PIDFollow(int linhaposition) {

  float P=0, I=0, D=0;
  float Kp=0.7;//0.02  0.0385
  float Ki=0;
  float Kd=0.0;//0.1

  //CALCULO DE VALOR LIDO
  last_proportional;
  proportional = 3500-linhaposition;
  derivative=proportional-last_proportional;
  integral = integral+proportional;
  last_proportional=proportional;
  
  P= proportional*Kp;
  D = derivative* Kd;
  I = integral * Ki;
  //PID
  float power_difference = P+I+D;

  if(power_difference>120)power_difference=120;

  //*Serial.print(proportional);//
  //Serial.print("         ");//
  //Serial.print(derivative);//
  //Serial.print("         ");//
  //Serial.print(integral);//
  //Serial.print("         ");//
  /*/Serial.print(power_difference);*/
      
  int rightMotorSpeed = rightBaseSpeed + power_difference;
  int leftMotorSpeed = leftBaseSpeed - power_difference;

  if (rightMotorSpeed > rightMaxSpeed ) rightMotorSpeed = rightMaxSpeed; // prevent the motor from going beyond max speed
  if (leftMotorSpeed > leftMaxSpeed ) leftMotorSpeed = leftMaxSpeed; // prevent the motor from going beyond max speed
  if (rightMotorSpeed < rightMinSpeed) rightMotorSpeed = rightMinSpeed; // keep the motor speed positive
  if (leftMotorSpeed < leftMinSpeed) leftMotorSpeed = leftMinSpeed; // keep the motor speed positive

  
  set_motors(leftMotorSpeed, rightMotorSpeed);
}

void set_motors(int l, int r)
{
  //DIRECIONAR RODAS
  /*Serial.print("left: ");
  Serial.print(l);
  Serial.print(" right: ");
  Serial.println(r);*/
  analogWrite(pwmLeft,l);
  analogWrite(pwmRight, r);

  digitalWrite(leftForward, LOW);
  digitalWrite(rightForward, LOW);
  digitalWrite(leftReverse, HIGH);
  digitalWrite(rightReverse, HIGH);
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

void BlinkLed(int times, int wait){

  for(int i=0; i<times; i++){
  digitalWrite(Led, HIGH);
  delay(wait);
  digitalWrite(Led, LOW);
  delay(wait);
  }
}