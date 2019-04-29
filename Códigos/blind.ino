#include <QTRSensors.h>
//DEFINIÇÕES DOS MOTORES, FRENTE TRÁS E PWM
          int STBY = 5; //standby do motor
          // Motor A, Left Side
          const uint8_t pwmLeft = 3;      // ENA - Enable and PWM
          const uint8_t leftForward = 7;  // IN1 - frente
          const uint8_t leftReverse = 2;  // IN2 - trás
           
          // Motor B, Right Side
          const uint8_t pwmRight = 11;     // ENB - Enable and PWM
          const uint8_t rightForward = 9; // IN3 - frente
          const uint8_t rightReverse = 8; // IN4 - trás



//DEFINIÇÕES DO QTR
          #define NUM_SENSORS             6 //QUANTIDADE DE SENSORES do qtr
          #define NUM_SAMPLES_PER_SENSOR  4
          #define EMITTER_PIN             6 //PINO HABILITAR LUZ QTR
          #define linhamedia              2500 //VALOR MÉDIO PARA MANTER RETO

#define LED_STATUS              13 //led que indica se já calibrou enquanto acesso, está calibrando

//DEFINIÇÕES DOS SENSORES LATERAIS
          #define Sensor_parada           A0 //sensor que detecta quando parar
          #define Sensor_curva            A7 //sensor que detecta as curvas
          
          #define Paradas                 5  //quantidade de marcadores a direita para saber quando parar
          #define Sensor_parada_calib     500 //valor para calibrar o sensor da direita

//OUTRAS DEFINIÇÕES / PID / COUNTER
  boolean blackLineRight = true;

  int Count_parada = 0;
  int Sensor_parada_read = 0;
  float last_proportional = 0;
  float proportional = 0;
  float derivative = 0;
  float integral = 0;

//PORTAS DO QTR
  QTRSensorsAnalog qtra((unsigned char[]) {1, 2, 3, 4, 5, 6},
  NUM_SENSORS, NUM_SAMPLES_PER_SENSOR, EMITTER_PIN);
  unsigned int sensorValues[NUM_SENSORS];
 
void setup()
{  
  //CALIBRAR QTR +- 10 SEGUNDOS, LED ASCENDE AO COMEÇAR A CALIBRAR, APAGA AO TERMINAR
  delay(500);
  pinMode(LED_STATUS, OUTPUT); //início calibrar led on
  
  pinMode(Sensor_parada, INPUT);
  //pinMode(Sensor_curva, INPUT);
  pinMode(STBY, OUTPUT);
  pinMode(pwmLeft, OUTPUT);
  pinMode(leftForward, OUTPUT);
  pinMode(leftReverse, OUTPUT);
  pinMode(pwmRight, OUTPUT);
  pinMode(rightForward, OUTPUT);
  pinMode(rightReverse, OUTPUT);
  digitalWrite(STBY, HIGH);
  
  digitalWrite(LED_STATUS, HIGH);
  for (int i = 0; i < 200; i++)
  {
    qtra.calibrate();
  }
  digitalWrite(LED_STATUS, LOW); //fim calibrar led off

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
  

  
  delay(1000);
}


void loop()
{
  
  unsigned int linhaposition = qtra.readLine(sensorValues); //LEITURA QTR
  /*for (unsigned char i = 0; i < NUM_SENSORS; i++)
  {
    Serial.print(sensorValues[i]);
    Serial.print('\t');
  }
  Serial.println(linhaposition); // comment this line out if you are using raw values*/
  if (millis() < 37000 ) {
  velocidade(linhaposition);
  } else { 
    stop_motors();
  }
  //stopCondition();
}

int velocidade(int linhaposition) {
  //CALCULO DE VALOR LIDO
  last_proportional;
  proportional = (linhaposition-linhamedia);
  derivative=proportional-last_proportional;
  integral = integral+proportional;

  //Serial.println(proportional);
  
  last_proportional=proportional;


  //VALORES DO PID
  float Kp=0.3;  //0.2  agora 0.35
  float Ki=0;
  float Kd=1.1;
  float max = 175; //170 agora 195

  //PID
  float power_difference = proportional * Kp + integral * Ki + derivative *Kd;

  /*Serial.print(proportional);
  Serial.print("         ");
  Serial.print(derivative);
  Serial.print("         ");
  Serial.print(integral);
  Serial.print("         ");
  Serial.print(power_difference);*/
  
  //DEFINIR VELOCIDADE DE CADA RODA
  const int vel_base = 180;
  if(power_difference>max)
    power_difference=max;
  if(power_difference < -max)
    power_difference=(-1*max);

  /*Serial.print("         ");
  Serial.println(power_difference);*/
  
  
  if(power_difference < 0)
  {
    set_motors(max+power_difference, max);
  } else {
    set_motors(max, max-power_difference);
  }
}


void set_motors(int l, int r)
{
  //DIRECIONAR RODAS
  /*Serial.print(l);
  Serial.print("         ");
  Serial.println(r);*/
  analogWrite(pwmLeft,l);
  analogWrite(pwmRight, r);
  digitalWrite(leftForward, HIGH);
  digitalWrite(rightForward, HIGH);
  digitalWrite(leftReverse, LOW);
  digitalWrite(rightReverse, LOW);
}

void stop_motors()
{
  //DIRECIONAR RODAS
  /*Serial.print(l);
  Serial.print("         ");
  Serial.println(r);*/
  analogWrite(pwmLeft,0);
  analogWrite(pwmRight, 0);
  digitalWrite(leftForward, LOW);
  digitalWrite(rightForward, LOW);
  digitalWrite(leftReverse, LOW);
  digitalWrite(rightReverse, LOW);
  delay(30000);
}

void stopCondition()  {
  
  Sensor_parada_read = analogRead(Sensor_parada); //LEITURA DO SENSOR LATERAL

  //SE LEITURA DO SENSOR < VALOR CALIBRADO
  if(Sensor_parada_read < Sensor_parada_calib && blackLineRight == true)
  {
    blackLineRight = false;
    Count_parada++; //ADICIONAR INDICADOR DE PARADA
  }

  //SE LEITURA DO SENSOR > VALOR CALIBRADO
  if(Sensor_parada_read > Sensor_parada_calib)
  {
    blackLineRight = true;
  }

  //SE O NÚMERO DE PARADAS CONTADAS >= PARADAS DEFINIDAS --- PARAR ROBÔ
  if(Count_parada >= Paradas) {
    set_motors(0, 0);
    delay(15000);
  }
}
