

const int SensorAnalog1 = A7;  // Analog input pin sensor de linha 1A
const int SensorAnalog2 = A6; // Analog input pin sensor de linha 2A

int sensor1Value = 0;        // valor inicial do sensor
int sensor2Value = 0;        // valor inicial do sensor

void setup() {
  pinMode(SensorAnalog1,INPUT);
  pinMode(SensorAnalog2,INPUT);
  // initialize serial communications at 9600 bps:
  Serial.begin(9600);
}

void loop() {

  // leitura analógica dos sensores:
  sensor1Value = analogRead(SensorAnalog1);
  sensor2Value = analogRead(SensorAnalog2);

  // printar os resultos para o Serial Monitor:
  Serial.print("sensor1 = ");
  Serial.print(sensor1Value);
  Serial.print("\t sensor2 = ");
  Serial.println(sensor2Value);

  
  delay(50);
}
