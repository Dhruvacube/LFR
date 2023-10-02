#include <Arduino.h>
#include <QTRSensors.h>
#include <cstring>
#include "nokiatune.h"
#include "BluetoothSerial.h"
#include <DRV8833.h>

DRV8833 driver = DRV8833();

#define USE_PIN
const char *pin = "7842";

String device_name = "HOKAGE_Meshmerize";

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

#if !defined(CONFIG_BT_SPP_ENABLED)
#error Serial Bluetooth not available or not enabled. It is only available for the ESP32 chip.
#endif

BluetoothSerial SerialBT;

#define EEP GPIO_NUM_19
#define in2 GPIO_NUM_5
#define in1 GPIO_NUM_17
#define in4 GPIO_NUM_4
#define in3 GPIO_NUM_16
#define ULT GPIO_NUM_18

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
const uint8_t sensors[SensorCount] = {GPIO_NUM_13, GPIO_NUM_12, GPIO_NUM_14, GPIO_NUM_27, GPIO_NUM_26, GPIO_NUM_25, GPIO_NUM_33, GPIO_NUM_32};
uint16_t sensorValues[SensorCount];

void printDebugData(String data) {
  if (SerialBT.available()) {
    SerialBT.println(data);
  }
  if (Serial.available()) {
    Serial.println(data);
  }
}

void setup() {
  Serial.begin(115200);
  SerialBT.begin(device_name); //Bluetooth device name
  
  printDebugData("The device started, now you can pair it with bluetooth!");
  printDebugData("The pin is 7842");
  printDebugData("in setup()");

  driver.attachMotorA(in1, in2);
  driver.attachMotorB(in3, in4);
  pinMode(EEP, OUTPUT);
  pinMode(ULT, INPUT_PULLUP);
  printDebugData("Set DRV8833");

  digitalWrite(EEP, HIGH);
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
  digitalWrite(EEP, LOW);
  printDebugData("Pulling DRV8833 pins to low");

  pinMode(button, INPUT_PULLUP);
  pinMode(buzzer, OUTPUT);
  pinMode(BUILTIN_LED, OUTPUT);
  printDebugData("Set button, buzzer and builtin LED");

  digitalWrite(BUILTIN_LED, LOW);
  digitalWrite(buzzer, LOW);
  printDebugData("Pulling buzzer and builtin LED to low");

  qtr.setTypeRC();
  qtr.setSensorPins(sensors, SensorCount);
  printDebugData("Set IR araay");

  //Wait for calibration On
  printDebugData("Wait for calibration On (IR Senor Array))");
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
  printDebugData("Calibrating IR Senor Array for 10 seconds");
  for (uint16_t i = 0; i < 400; i++)
  {
    qtr.calibrate();
  }
  printDebugData("Calibration done");
  digitalWrite(BUILTIN_LED, LOW);

  nokiatune(buzzer);

  //Wait for the start
  printDebugData("Wait for the start");
  while(digitalRead(button) == HIGH){
    delay(100);
    digitalWrite(BUILTIN_LED, HIGH);
    delay(100);
    digitalWrite(BUILTIN_LED, LOW);
  }
  digitalWrite(buzzer, HIGH);
  delay(100);
  digitalWrite(buzzer, LOW);
  delay(100);
  digitalWrite(EEP, HIGH);
  delay(1000);

  // Starts approximately after 1 secs
  printDebugData("Starts approximately after 1 secs");
}


void forward_movement(int speedA, int speedB) {
  if (speedA < 0) {
    speedA = 0 - speedA;
  }
  if (speedB < 0) {
    speedB = 0 - speedB;
  }
  driver.motorAForward(speedA);
  driver.motorBForward(speedB);
}

void PID_control() {
  uint16_t positionLine = qtr.readLineBlack(sensorValues);
  Serial.println(positionLine);
  int error = 3500 - positionLine;

  P = error;
  I = error + I;
  D = error - lastError;
  lastError = error; 

  int motorSpeedChange = (P*Kp) + (I*Ki) + (D*Kd);

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


void loop() {
  // put your main code here, to run repeatedly:
  PID_control();
}