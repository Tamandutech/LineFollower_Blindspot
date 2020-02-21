#include <Adafruit_MCP3008.h>

Adafruit_MCP3008 adc;
#define Led              13

int maxSideSensorValues[4] =  {0, 0, 0, 0};
int minSideSensorValues[4] = {1023, 1023, 1023, 1023};

//Setup dos sensores parada
#define stopSensor1 A6
#define stopSensor2 A7
int stopCount =0;
int stopSensorValue1 = 0, stopSensorValue2 = 0;        // valores iniciais dos sensores 1 e 2
int stopTrigger = 400;                                  // = 30% do valor total ativa a contagem dos sensores 1 e 2
int instersecs = 4;                                      //valor de instersecções na pista
bool stops= false;


//setup dos sensores curva
#define curveSensor1 A5
#define curveSensor2 A4
int curvesCount =0;
int curveSensorValue1 = 0, curveSensorValue2 = 0;        // valor inicial dos sensores esquerdos 1 e 2
int curveTrigger= 300;
bool curves = false;

//Setup do array QTR
int numSensors = 8;//Numero de sesnores sendo utilizados
float valorSensor[8];//array de valores do sensor
float posit = 0; //posição do robô em relação a linha
float MaxQTRValues[8] =  {0, 0, 0, 0, 0, 0, 0, 0};
float MinQTRValues[8]= {1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023}; //armazenar valores máximos e mínimo para a calibração.
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
int rightBaseSpeed= 190; // velocidade base da roda direita
int leftBaseSpeed =190; // velocidade base da roda esquerda
int rightMinSpeed =10; // velocidade min da roda direita
int leftMinSpeed =10;

//Temporizadores
unsigned long previousMillis = 0;  
unsigned long currentMillis=0;
const long interval = 150; 
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
//-------setup dos motores--------------------
pinMode(STBY, OUTPUT);
pinMode(pwmLeft, OUTPUT);
pinMode(leftForward, OUTPUT);
pinMode(leftReverse, OUTPUT);
pinMode(pwmRight, OUTPUT);
pinMode(rightForward, OUTPUT);
pinMode(rightReverse, OUTPUT);
digitalWrite(STBY, HIGH); 
//-----------SETUP DOS SENSORES------------------
pinMode(stopSensorValue1,INPUT);
pinMode(stopSensorValue2,INPUT);
pinMode(curveSensorValue1,INPUT);
pinMode(curveSensorValue2,INPUT);
//-----------------------------------------------
while (!Serial);

// Software SPI (specify all, use any available digital)
// (sck, mosi, miso, cs);
adc.begin(10, 12, 11, 2);
//------------------------------------------------------INICIO CALIBRAÇÃO----------------------------------
BlinkLed(3, 500); // Pisca o Led para mostrar que a calibração começou
CalibrateArraySensors(); // inicia rotina de calibração dos sensores
BlinkLed(5, 100); //Pisca o Led para mostrar que a calibração terminou e deve-se colocar o robô na posição
CalibrateSideSensors(4);
BlinkLed(10, 50);
delay(2000); //aguarda 2 segundos para colocar o robô.
}

//---------------------------------------------------- LOOPING CODE -------------------------------------------------------------------

void loop() {
currentMillis = millis();//iguala o valor de current ao valor real de milis

stops = CountStops();
curves = CountCurves();

  if(((stops == true) || (curves==true)) && ((currentMillis - previousMillis) >= interval)){  
    previousMillis = currentMillis;
    
    if(stops==true) stopCount++;
    
    if(stopCount >= instersecs){
    //Serial.print("  PARAAAAAAAAA: ");
    StopRobot();
    }

    if(curves==true) {
    curvesCount++;
    //Serial.print("  CurveCount: ");
    //Serial.print(curvesCount);
    }

  }

  ReadArraySensor();
  posit = GetPosition(valorSensor);
  PIDFollow(posit);
/*------ Se desejar parar por tempo, descomentar esta função
if(currentMillis >= stopTime){
  StopRobot();
}*/

 //Serial.println (posit);
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
  }

void CalibrateSideSensors(int num){

  int sensors[4];
  int reads[4] = {stopSensor1, stopSensor2, curveSensor1, curveSensor2};
  for(int i=0; i<200; i++){
    for(int j=0; j<num; j++){
      sensors[j] =  analogRead(reads[j]);
    
      if(sensors[j] < minSideSensorValues[j])   minSideSensorValues[j]= sensors[j];

      if(sensors[j] > maxSideSensorValues[j])   maxSideSensorValues[j] = sensors[j];
    }
    delay(25);
  }
  //------- MIN VALUES -----------------
  Serial.print(" Min Values: ");
    for(int k=0; k<num; k++){
      Serial.print(" ");
      Serial.print(minSideSensorValues[k]);
    }
  //------MAX VALUES ------------------
  Serial.println(" ");
  Serial.print("Max Values: ");
    for(int l=0; l<num; l++){
      Serial.print(" ");
      Serial.print(maxSideSensorValues[l]);
    }
}

void ReadArraySensor (){

  for(int i=0; i<numSensors; i++){
    float aux;
    aux = adc.readADC(i);
    if(aux>MaxQTRValues[i]) aux = MaxQTRValues[i];
    if(aux<MinQTRValues[i]) aux = MinQTRValues[i];

    valorSensor[i] = 1000 * ((aux-MinQTRValues[i]) /(MaxQTRValues[i]-MinQTRValues[i]));

    if(whiteLine){
       valorSensor[i] = 1000-valorSensor[i]; //se a linha for branca no fundo preto
    }
    //Serial.print(valorSensor[i]);
    //Serial.print("    ");     
  }
  //Serial.println(" ");
}

//calcula a posição do robô em relação a linha
float GetPosition(float sensorsValues[]){
  float pos=0;
  float denom=0, numer =0;
  //int num = sizeof(sensorsValues);

  for(int i=0; i<numSensors;i++){
    numer= (i*1000 * sensorsValues[i]) + numer;
    denom= sensorsValues[i] + denom;
  }
  pos = numer/denom;

  if(pos<0) pos=0;
  if(pos>(numSensors-1)*1000) pos = (numSensors-1)*1000;

  return pos; //retorna a posição em relação a linha
}

//----conta as intersecções e faixas de parada da pista----
bool CountStops(){
  stopSensorValue1 = analogRead(stopSensor1);
  stopSensorValue2 = analogRead(stopSensor2);

//Valores maiores ou menores que os calibrados são excluidos
  if(stopSensorValue1>maxSideSensorValues[0]) stopSensorValue1 = maxSideSensorValues[0];
  if(stopSensorValue2>maxSideSensorValues[1]) stopSensorValue2 = maxSideSensorValues[1];

  if(stopSensorValue1<minSideSensorValues[0]) stopSensorValue1 = minSideSensorValues[0];
  if(stopSensorValue2<minSideSensorValues[1]) stopSensorValue2 = minSideSensorValues[1];

  //stopSensorValue1 = 1000 * ((stopSensorValue1-minSideSensorValues[0]) /(maxSideSensorValues[0]-minSideSensorValues[0]));
  stopSensorValue1 = map(stopSensorValue1,minSideSensorValues[0],maxSideSensorValues[0], 0, 1000);
  //stopSensorValue2 = 1000 * ((stopSensorValue2-minSideSensorValues[1]) /(maxSideSensorValues[1]-minSideSensorValues[1]));
  stopSensorValue2 = map(stopSensorValue2,minSideSensorValues[1],maxSideSensorValues[1], 0, 1000);

 if(stopSensorValue1 < stopTrigger && stopSensorValue2 < stopTrigger){
     //Serial.print("  stopCount: ");
    //Serial.print(stopCount);
    return true;
  }else {
    return false;
  }
}

bool CountCurves(){ //conta os marcadores de curva
  curveSensorValue1 = analogRead(curveSensor1);
  curveSensorValue2 = analogRead(curveSensor2);

  //Valores maiores ou menores que os calibrados são excluidos
  if(curveSensorValue1>maxSideSensorValues[2]) curveSensorValue1 = maxSideSensorValues[2];
  if(curveSensorValue2>maxSideSensorValues[3]) curveSensorValue2 = maxSideSensorValues[3];

  if(curveSensorValue1<minSideSensorValues[2]) curveSensorValue1 = minSideSensorValues[2];
  if(curveSensorValue2<minSideSensorValues[3]) curveSensorValue2 = minSideSensorValues[3];

  curveSensorValue1 = map(curveSensorValue1,minSideSensorValues[2],maxSideSensorValues[2], 0, 1000);
  curveSensorValue2 = map(curveSensorValue2,minSideSensorValues[3],maxSideSensorValues[3], 0, 1000);

   if(curveSensorValue1 < curveTrigger && curveSensorValue2 < curveTrigger){
    return true;   
  }
  else{
    return false;
  }
  
}

int PIDFollow(float linhaposition) {

  float P=0, I=0, D=0;
  float Kp=0.07;//0.02  0.0385
  float Ki=0.0;
  float Kd=0.75;//0.1

  //CALCULO DE VALOR LIDO
  last_proportional;
  proportional = linhaposition-3500;
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

  digitalWrite(leftForward, HIGH);
  digitalWrite(rightForward, HIGH);
  digitalWrite(leftReverse, LOW);
  digitalWrite(rightReverse, LOW);
}

void StopRobot(){ //para o robô
  for(int i=0; i<40; i++){
  ReadArraySensor();
  posit = GetPosition(valorSensor);
  PIDFollow(posit); 
  }
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