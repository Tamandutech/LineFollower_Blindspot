#include <QTRSensors.h>
int STBY = 5; //standby
//DEFINIÇÕES DOS MOTORES, FRENTE TRÁS E PWM
// Motor A, Left Side
const uint8_t pwmLeft = 3;  // ENA - Enable and PWM
const uint8_t leftForward = 7;  // IN1 - Forward Drive
const uint8_t leftReverse = 2; // IN2 - Reverse Drive
 
// Motor B, Right Side
const uint8_t pwmRight = 11;     // ENB - Enable and PWM
const uint8_t rightForward = 9; // IN3 - Forward Drive
const uint8_t rightReverse = 8; // IN4 - Reverse Drive

int rightMaxSpeed =255;; // max speed of the robot
int leftMaxSpeed =255;// max speed of the robot
int rightBaseSpeed= 150; // this is the speed at which the motors should spin when the robot is perfectly on the line
int leftBaseSpeed =150; //145
int rightMinSpeed =2; //8
int leftMinSpeed =2;

#define NUM_SENSORS             6 //QUANTIDADE DE SENSORES
#define NUM_SAMPLES_PER_SENSOR  4
#define EMITTER_PIN             6 //PINO HABILITAR LUZ QTR
#define linhamedia              2500 //VALOR MÉDIO PARA MANTER RETO

#define LED_STATUS              13

#define Sensor_parada           10
bool parada = true;
int contador =0;
//#define Sensor_curva           12

#define Paradas                 5

unsigned long previousMillis = 0;  
const long interval = 500; 

  float last_proportional = 0;
  float proportional = 0;
  float derivative = 0;
  float integral = 0;

//PORTAS DO QTR
QTRSensorsAnalog qtra((unsigned char[]) { 1, 2, 3, 4, 5, 6},
  NUM_SENSORS, NUM_SAMPLES_PER_SENSOR, EMITTER_PIN);
unsigned int sensorValues[NUM_SENSORS];
 
void setup()
{

  //CALIBRAR QTR +- 10 SEGUNDOS, LED ASCENDE AO COMEÇAR A CALIBRAR, APAGA AO TERMINAR
  delay(500);
  pinMode(LED_STATUS, OUTPUT);
  digitalWrite(LED_STATUS, HIGH);
  for (int i = 0; i < 200; i++)
  {
    qtra.calibrate();
  }
  digitalWrite(LED_STATUS, LOW);
  Serial.begin(9600);
  /*Serial.begin(9600);
  for (int i = 0; i < NUM_SENSORS; i++)
  {
    Serial.print(qtra.calibratedMinimumOn[i]);
    Serial.print(' ');
  }
  //Serial.println();

  // print the calibration maximum values measured when emitters were on
  for (int i = 0; i < NUM_SENSORS; i++)
  {
    Serial.print(qtra.calibratedMaximumOn[i]);
    Serial.print(' ');
  }
  Serial.println();
  Serial.println();*/
  
  pinMode(Sensor_parada, INPUT);
 // pinMode(Sensor_curva, INPUT);
  pinMode(STBY, OUTPUT);
  pinMode(pwmLeft, OUTPUT);
  pinMode(leftForward, OUTPUT);
  pinMode(leftReverse, OUTPUT);
  pinMode(pwmRight, OUTPUT);
  pinMode(rightForward, OUTPUT);
  pinMode(rightReverse, OUTPUT);
     digitalWrite(STBY, HIGH); 
  delay(1000);
}


void loop()
{
  unsigned int linhaposition = qtra.readLine(sensorValues); //LEITURA QTR
    unsigned long currentMillis = millis();//igual o valor de current ao valor real de milis
  /*for (unsigned char i = 0; i < NUM_SENSORS; i++)
  {
    Serial.print(sensorValues[i]);
    Serial.print('\t');
  }
  Serial.println(linhaposition); // comment this line out if you are using raw values*/
  
    parada = digitalRead(Sensor_parada);//ler sensor de parada (sensor da direita)
    //int curva = analogRead(Sensor_curva);
    velocidade(linhaposition);
    
    if(parada==0 && (currentMillis - previousMillis) >= interval){//se ler linha branca
      contador++;//soma um na contagem de cruazmentos brancos
       previousMillis = currentMillis;
    }

    if(contador >= 30){
      delay(200);
      stop_motors();
    }

    if (currentMillis >= 34000) {
       stop_motors();
    }
    
}

int velocidade(int linhaposition) {
  //CALCULO DE VALOR LIDO
  last_proportional;
  proportional = (linhaposition-linhamedia);
  derivative=proportional-last_proportional;
  integral = integral+proportional;

  Serial.println("erro: ");  
  Serial.println(proportional);
  
  last_proportional=proportional;

  
  float Kp=0.046;//0.02  0.0385
  float Ki=0;
  float Kd=0.075;//0.1

  //PID
  float power_difference = proportional * Kp + derivative *Kd;

  //*Serial.print(proportional);//
  //Serial.print("         ");//
  //Serial.print(derivative);//
  //Serial.print("         ");//
  //Serial.print(integral);//
  //Serial.print("         ");//
  /*/Serial.print(power_difference);*/
  /*if(proportional <= 200){
     rightMaxSpeed=240;
     leftMaxSpeed=240;
     rightBaseSpeed=180;
     leftBaseSpeed=180;
      rightMinSpeed =180; //8
    leftMinSpeed =180;      
  }*/
  
  int rightMotorSpeed = rightBaseSpeed + power_difference;
  int leftMotorSpeed = leftBaseSpeed - power_difference;

  if (rightMotorSpeed > rightMaxSpeed ) rightMotorSpeed = rightMaxSpeed; // prevent the motor from going beyond max speed
  if (leftMotorSpeed > leftMaxSpeed ) leftMotorSpeed = leftMaxSpeed; // prevent the motor from going beyond max speed
  if (rightMotorSpeed < rightMinSpeed) rightMotorSpeed = rightMinSpeed; // keep the motor speed positive
  if (leftMotorSpeed < leftMinSpeed) leftMotorSpeed = leftMinSpeed; // keep the motor speed positive
  
  /*Serial.print("         ");
  Serial.println(power_difference);*/
  
  set_motors(leftMotorSpeed, rightMotorSpeed);

}

void stop_motors(){
   analogWrite(pwmLeft,40);
  analogWrite(pwmRight, 40);

  digitalWrite(leftForward, LOW);
  digitalWrite(rightForward, LOW);
  digitalWrite(leftReverse, HIGH);
  digitalWrite(rightReverse, HIGH);
  delay(1000);
  
  analogWrite(pwmLeft,0);
  analogWrite(pwmRight, 0);
  digitalWrite(leftForward, LOW);
  digitalWrite(rightForward, LOW);
  digitalWrite(leftReverse, LOW);
  digitalWrite(rightReverse, LOW);
  delay(15000);
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
