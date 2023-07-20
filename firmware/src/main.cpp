#include <Arduino.h>
#include <QTRSensors.h>

int EEP = GPIO_NUM_19;
int in1 = GPIO_NUM_15;
int in2 = GPIO_NUM_17;
int in3 = GPIO_NUM_16;
int in4 = GPIO_NUM_4;
int VLT = GPIO_NUM_18;

int P;
int I;
int D;

float Kp = 0.05;
float Ki = 0.00001;
float Kd = 0.8;

int lastError = 0;

int button = GPIO_NUM_15;
int buzzer = GPIO_NUM_21;
QTRSensors qtr;

const uint8_t SensorCount = 8;
uint16_t sensorValues[SensorCount];

void forward_movement(int speedA, int speedB);
void PID_control();

void setup() {
  // put your setup code here, to run once:
  qtr.setTypeRC();
  qtr.setSensorPins((const uint8_t[]){GPIO_NUM_13, GPIO_NUM_12, GPIO_NUM_14, GPIO_NUM_27, GPIO_NUM_26, GPIO_NUM_25, GPIO_NUM_33, GPIO_NUM_32}, SensorCount);

  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
  pinMode(EEP, OUTPUT);
  pinMode(VLT, INPUT);

  digitalWrite(EEP, HIGH);

  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);

  pinMode(button, INPUT_PULLUP);
  digitalWrite(BUILTIN_LED, LOW);
  digitalWrite(buzzer, LOW);

  while(digitalRead(button) == HIGH) {
    delay(100);
    digitalWrite(BUILTIN_LED, HIGH);
    digitalWrite(buzzer, HIGH);
    delay(100);
    digitalWrite(BUILTIN_LED, LOW);
    digitalWrite(buzzer, LOW);
  }
  digitalWrite(BUILTIN_LED, LOW);
  digitalWrite(buzzer, LOW);
  
  //10 seconds
  for (uint16_t i = 0; i < 400; i++)
  {
    qtr.calibrate();
  }

  while(digitalRead(button) == HIGH){}
  digitalWrite(buzzer, HIGH);
  delay(100);
  digitalWrite(buzzer, LOW);
  delay(1000);
}

void loop() {
  // put your main code here, to run repeatedly:
  PID_control();
}

void PID_control() {
  uint16_t positionLine = qtr.readLineBlack(sensorValues);
  int error = 3500 - positionLine;

  P = error;
  I = error + I;
  D = error - lastError;
  lastError = error; 

  int motorSpeedChange = P*Kp + I*Ki + D*Kd;

  int motorSpeedA = 100 + motorSpeedChange;
  int motorSpeedB = 100 - motorSpeedChange;

  if (motorSpeedA > 125) {
    motorSpeedA = 125;
  }
  if (motorSpeedB > 125) {
    motorSpeedB = 125;
  }
  if (motorSpeedA < -75) {
    motorSpeedA = -75;
  }
  if (motorSpeedB < -75) {
    motorSpeedB = -75;
  }
  forward_movement(motorSpeedA, motorSpeedB);
}

void forward_movement(int speedA, int speedB) {
  if (speedA < 0) {
    speedA = 0 - speedA;
    digitalWrite(in2, HIGH);
    digitalWrite(in1, LOW);
  }
  else {
    digitalWrite(in2, LOW);
    digitalWrite(in1, LOW);
  }
  if (speedB < 0) {
    speedB = 0 - speedB;
    digitalWrite(in4, HIGH);
    digitalWrite(in3, LOW);
  }
  else {
    digitalWrite(in4, LOW);
    digitalWrite(in3, LOW);
  }
  analogWrite(in1, speedA);
  analogWrite(in3, speedB);
}