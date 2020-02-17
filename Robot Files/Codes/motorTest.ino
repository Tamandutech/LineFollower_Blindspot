int STBY = 6; //standby
//DEFINIÇÕES DOS MOTORES, FRENTE TRÁS E PWM
// Motor A, Left Side
const uint8_t pwmLeft = 3;      // ENA - Enable and PWM
const uint8_t leftForward = 5;  // IN1 - Forward Drive
const uint8_t leftReverse = 4;  // IN2 - Reverse Drive
 
// Motor B, Right Side
const uint8_t pwmRight = 9;     // ENB - Enable and PWM
const uint8_t rightForward = 7; // IN3 - Forward Drive
const uint8_t rightReverse = 8; // IN4 - Reverse Drive


void setup (){
  pinMode(STBY, OUTPUT);
  pinMode(pwmLeft, OUTPUT);
  pinMode(leftForward, OUTPUT);
  pinMode(leftReverse, OUTPUT);
  pinMode(pwmRight, OUTPUT);
  pinMode(rightForward, OUTPUT);
  pinMode(rightReverse, OUTPUT);
  digitalWrite(STBY, HIGH); 
}

void loop (){
  //Andar Frente
  analogWrite(pwmLeft,90);
  analogWrite(pwmRight, 90);
  digitalWrite(leftForward, HIGH);
  digitalWrite(rightForward, HIGH);
  digitalWrite(leftReverse, LOW);
  digitalWrite(rightReverse, LOW);
  }
  
