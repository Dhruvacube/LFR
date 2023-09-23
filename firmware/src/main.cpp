#include <Arduino.h>
#include <QTRSensors.h>

int EEP = GPIO_NUM_19;
int in1 = GPIO_NUM_5;
int in2 = GPIO_NUM_17;
int in3 = GPIO_NUM_16;
int in4 = GPIO_NUM_4;
int ULT = GPIO_NUM_18;

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


void setup() {
  // put your setup code here, to run once:
  qtr.setTypeRC();
  qtr.setSensorPins((const uint8_t[]){GPIO_NUM_13, GPIO_NUM_12, GPIO_NUM_14, GPIO_NUM_27, GPIO_NUM_26, GPIO_NUM_25, GPIO_NUM_33, GPIO_NUM_32}, SensorCount);

  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
  pinMode(EEP, OUTPUT);
  pinMode(ULT, INPUT_PULLUP);

  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);

  digitalWrite(EEP, LOW);

  pinMode(button, INPUT_PULLUP);
  pinMode(buzzer, OUTPUT);
  pinMode(BUILTIN_LED, OUTPUT);

  digitalWrite(BUILTIN_LED, LOW);
  digitalWrite(buzzer, LOW);

  //Wait for calibration On
  while(digitalRead(button) == HIGH) {
    delay(100);
    digitalWrite(BUILTIN_LED, HIGH);
    digitalWrite(buzzer, HIGH);
    delay(100);
    digitalWrite(BUILTIN_LED, LOW);
    digitalWrite(buzzer, LOW);
  }

  digitalWrite(BUILTIN_LED, HIGH);
  digitalWrite(buzzer, LOW);

  //10 seconds
  for (uint16_t i = 0; i < 400; i++)
  {
    qtr.calibrate();
  }
  digitalWrite(BUILTIN_LED, LOW);

  //Wait for the start
  while(digitalRead(button) == HIGH){
    delay(100);
    digitalWrite(BUILTIN_LED, HIGH);
    digitalWrite(buzzer, HIGH);
    delay(100);
    digitalWrite(BUILTIN_LED, LOW);
    digitalWrite(buzzer, LOW);
  }
  digitalWrite(buzzer, HIGH);
  delay(100);
  digitalWrite(buzzer, LOW);
  delay(100);
  digitalWrite(EEP, HIGH);
  delay(1000);
  // Starts approximately after 5 secs
}

void loop() {
 
}

