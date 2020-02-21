

#define stopSensor1 A6  // Analog input pin sensor de linha 1A
#define stopSensor2 A7 // Analog input pin sensor de linha 2A

#define curveSensor1 A4  // Analog input pin sensor de linha 1B
#define curveSensor2 A5 // Analog input pin sensor de linha 2B

int maxSideSensorValues[4] =  {0, 0, 0, 0};
int minSideSensorValues[4] = {1023, 1023, 1023, 1023};

int stopCount =0;
int stopSensorValue1 = 0, stopSensorValue2 = 0;        // valores iniciais dos sensores 1 e 2
int stopTrigger = 300;                                  // = 30% do valor total ativa a contagem dos sensores 1 e 2
int instersecs = 8; 
bool stops= false;

int curvesCount =0;
int curveSensorValue1 = 0, curveSensorValue2 = 0;        // valor inicial dos sensores esquerdos 1 e 2
int curveTrigger= 300;
bool curves = false;

//Temporizadores
unsigned long previousMillis = 0;  
unsigned long currentMillis=0;
const long interval = 400; 
int timeStop=32000;

void setup() {
  pinMode(stopSensorValue1,INPUT);
  pinMode(stopSensorValue2,INPUT);
  pinMode(curveSensorValue1,INPUT);
  pinMode(curveSensorValue2,INPUT);
  // initialize serial communications at 9600 bps:
  Serial.begin(9600);
  Serial.println("start calibrate: ");
  CalibrateSideSensors(4);
}

void loop() {

currentMillis = millis();//iguala o valor de current ao valor real de milis

  stops = CountStops();
  curves = CountCurves();
  if(((stops == true) || (curves==true)) && ((currentMillis - previousMillis) >= interval)){  
    previousMillis = currentMillis;
    if(stops==true) stopCount++;
    
    if(stopCount >= instersecs){
    Serial.print("  PARAAAAAAAAA: ");
    //StopRobot();
    }

    if(curves==true) curvesCount++;
    Serial.print("  CurveCount: ");
    Serial.print(curvesCount);
  }
  
  // leitura analógica dos sensores:
  //sensor1Value = analogRead(SensorAnalog1);
  //sensor2Value = analogRead(SensorAnalog2);

  /* printar os resultos para o Serial Monitor:
  Serial.print("sensor1 = ");
  Serial.print(sensor1Value);
  Serial.print("\t sensor2 = ");
  Serial.println(sensor2Value);*/
  
  //delay(50);
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

  /*Serial.println(" ");
  Serial.print(stopSensorValue1);
  Serial.print(" ");
  Serial.print(stopSensorValue2);*/
    
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

  Serial.println(" ");
  Serial.print(curveSensorValue1);
  Serial.print(" ");
  Serial.print(curveSensorValue2);
  
  if(curveSensorValue1 < curveTrigger && curveSensorValue2 < curveTrigger){
    return true;   
  }
  else{
    return false;
  }
  
}
