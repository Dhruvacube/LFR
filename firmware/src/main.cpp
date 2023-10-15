#include <Arduino.h>
#include <QTRSensors.h>
#include "nokiatune.h"
#include <DRV8833.h>
#include "BluetoothSerial.h"

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


#define button GPIO_NUM_15
#define buzzer GPIO_NUM_21

#define threshold 3500

QTRSensors qtr;

const uint8_t SensorCount = 8;
const uint8_t sensors[SensorCount] = {GPIO_NUM_13, GPIO_NUM_12, GPIO_NUM_14, GPIO_NUM_27, GPIO_NUM_26, GPIO_NUM_25, GPIO_NUM_33, GPIO_NUM_32};
uint16_t sensorValues[SensorCount];


/*

  This code is to be used with the Robojunkies Explorer Kit. More details here- https://robojunkies.com/product/explorer-robotics-kit/
  This is a Maze Solving Program like the competitions held at IIT Techfest- Meshmerize.
  Requires a black background and white line maze.

  ==>This program was tested and refined for a maze containing 90 degrees only!!!<==

  Developed by Team Robojunkies.

   Motor Right Dir- 7
   Motor Right PWM- 9
   Motor Left Dir- 8
   Motor Left PWM- 10
   RGB- 6,5,3

   Line sensor on A0,A1,A2,A3,A4
   A0-left & A4 - right

*/

bool l = 0;
bool r = 0;
bool s = 0;
bool u = 0;
int e = 0;
int paths = 0;

bool endFound = 0;

// int blackValue = 900;
// int whiteValue = 100;
// int threshold = 660;
//int threshold = (blackValue + whiteValue) * 0.5;
int FT = 250;
int P, D, I, previousError, PIDvalue, error;
int lsp = 100;
int rsp = 100;
int lfspeed = 80;
int turnspeed;
float Kp = 0.04;
float Kd = 0.05;
float Ki = 0;

String str;

void botleft ();
void linefollow ();
void checknode ();
void reposition();
void PID ();
void botstop ();
void botinchforward ();
void ledBlink();
void ledStop();
void botright ();
void botuturn ();

void setup() {
  SerialBT.begin(device_name);
  qtr.setTypeRC();
  qtr.setSensorPins(sensors, SensorCount);

  driver.attachMotorA(in1, in2);
  driver.attachMotorB(in3, in4);
  pinMode(EEP, OUTPUT);
  digitalWrite(EEP, LOW);
  pinMode(ULT, INPUT_PULLUP);

  pinMode(button, INPUT_PULLUP);
  pinMode(buzzer, OUTPUT);
  pinMode(BUILTIN_LED, OUTPUT);

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

  nokiatune(buzzer);
  //  lfspeed = 50000 / analogRead(7); //arbitrary conversion to convert analogRead to speed. Need to check if this works for all voltage levels
  turnspeed = lfspeed * 0.6;
  digitalWrite(EEP, HIGH);
}

void loop() {
  digitalWrite(BUILTIN_LED, HIGH);
    digitalWrite(buzzer, HIGH);
    delay(100);
    digitalWrite(BUILTIN_LED, LOW);
    digitalWrite(buzzer, LOW);
  while (digitalRead(button) == HIGH)
  { //Do nothing while waiting for button press
    delay(100);
    digitalWrite(BUILTIN_LED, HIGH);
    delay(100);
    digitalWrite(BUILTIN_LED, LOW);
  }
  delay(1000);
  int n= 0;
  while (endFound == 0)
  {
    linefollow();
    checknode();

    botstop();
    delay(100);

    reposition ();
    n+=1;
    SerialBT.println(n);
  }
  SerialBT.println("Inside forloop");
  for (int x = 0; x < 4; x++)
  {
    str.replace("LULUS", "U");
    str.replace("LUSUL", "U");
    str.replace("LUL", "S");
    str.replace("SUL", "R");
    str.replace("LUS", "R");
    str.replace("RUL", "U");
    SerialBT.println(str);
  }
  int endpos = str.indexOf('E');

  while (digitalRead(button) == HIGH)
  { //Do nothing while waiting for button press
    delay(100);
    digitalWrite(BUILTIN_LED, HIGH);
    delay(100);
    digitalWrite(BUILTIN_LED, LOW);
    SerialBT.println("Waiting for button press");
  }
  delay(1000);

  for (int i = 0; i <= endpos; i++)
  {
    SerialBT.println("Inside forloop");
    char node = str.charAt(i);
    paths = 0;
    while (paths < 2)
    {
      linefollow();
      checknode();
      if (paths == 1)
      {
        reposition();
      }
    }
    switch (node)
    {
      case 'L':
        botstop();
        delay(50);
        botleft();
        break;

      case 'S':
        break;

      case 'R':
        botstop();
        delay(50);
        botright();
        break;

      case 'E':
        for (int i = 0; i < 50; i++)
        {
          botinchforward ();
        }
        botstop();
        delay(1000);
        break;
    }//_________end of switch
  }//_________end of for loop

}

uint16_t returnPositionLine() {
  return qtr.readLineWhite(sensorValues);;
}

void linefollow()
{ //green () ;
  paths = 0;
  while ((returnPositionLine() < threshold ) && (returnPositionLine() > threshold ) && (returnPositionLine() < threshold+500 && returnPositionLine() > threshold-500))
  {
    PID();
    delay(50);
  }
}


void PID()
{
  int error = threshold - returnPositionLine();

  P = error;
  I = I + error;
  D = error - previousError;

  PIDvalue = (Kp * P) + (Ki * I) + (Kd * D);
  previousError = error;

  lsp = lfspeed - PIDvalue;
  rsp = lfspeed + PIDvalue;

  if (lsp > 200) {
    lsp = 200;
  }
  if (lsp < 0) {
    lsp = 0;
  }
  if (rsp > 200) {
    rsp = 200;
  }
  if (rsp < 0) {
    rsp = 0;
  }
  SerialBT.println("PID");
  SerialBT.println(returnPositionLine());
  SerialBT.println(lsp);
  SerialBT.println(rsp);
  driver.motorAForward(lsp);
  driver.motorBForward(rsp);
  delay(200);
}


void checknode ()
{
  l = 0;
  r = 0;
  s = 0;
  u = 0;
  e = 0;
  paths = 0;

  // checks whethere bot is on node and the number of exits possible


  if (returnPositionLine() < threshold) r = 1;
  if (returnPositionLine() > threshold) l = 1;
  if ((returnPositionLine() < threshold && (returnPositionLine() > threshold))) { // && (analogRead(2) > threshold)
    u = 1;
  }
  if (returnPositionLine() < threshold) {
    e = 1;
  }

  if (u == 0)
  {
    for (int i = 0; i < FT; i++)
    {
      //botinchforward ();
      PID();
      if (returnPositionLine() < threshold) r = 1;
      if (returnPositionLine() > threshold) l = 1;
    }

    for (int i = 0; i < FT; i++)
    { ledBlink();
      //botinchforward ();
      PID();
      if (returnPositionLine() < threshold+500 && returnPositionLine() > threshold-500) s = 1;
      if ((e == 1) && returnPositionLine() < threshold) e = 2;
    }
  }
  if (u == 1)
  {
    for (int i = 0; i < 50; i++)
    {
      botinchforward ();
    }
  }

  paths = l + s + r;
  SerialBT.println("Paths"+paths);
}


void reposition()
{
  ledBlink();
  if (e == 2)
  {
    str += 'E';
    endFound = 1;
    ledBlink();
    botstop();
    delay(2000);
  }
  else if (l == 1)
  {
    if (paths > 1) str += 'L';
    botleft(); //take left
  }

  else if (s == 1)
  {
    if (paths > 1) str += 'S';
  }
  else if (r == 1)
  {
    if (paths > 1) str += 'R';
    botright(); //take right
  }

  else if (u == 1)
  {
    ledBlink();
    str += 'U';
    botuturn(); //take left
  }
  ledStop();

}


void botleft ()
{
  SerialBT.println("Left");
  driver.motorAReverse(lfspeed);
  driver.motorBForward(lfspeed);
  delay(200);
  while (returnPositionLine() > threshold+500 || returnPositionLine() < threshold-500)
  {
    SerialBT.println("While Left");
    driver.motorAReverse(lfspeed);
    driver.motorBForward(lfspeed);
  }
  driver.motorAStop();
  driver.motorBStop();
  delay(50);
}

void botright ()
{
  SerialBT.println("Right");
  driver.motorAForward(lfspeed);
  driver.motorBReverse(lfspeed);
  delay(200);
  while (returnPositionLine() > threshold+500 || returnPositionLine() < threshold-500)
  {
    SerialBT.println("While Right");
    driver.motorAForward(lfspeed);
    driver.motorBReverse(lfspeed);
  }
  driver.motorAStop();
  driver.motorBStop();
  delay(50);
}

void botstraight ()
{
  SerialBT.println("Straight");
  driver.motorAForward(lfspeed);
  driver.motorBForward(lfspeed);
}

void botinchforward ()
{
  SerialBT.println("Inch Forward");
  driver.motorAForward(turnspeed);
  driver.motorBForward(turnspeed);
  delay(10);
}
void botstop ()
{
  SerialBT.println("Stop");
  driver.motorAStop();
  driver.motorBStop();
}
void botuturn ()
{
  SerialBT.println("Uturn");
  driver.motorAReverse(lfspeed);
  driver.motorBForward(lfspeed * 0.8);
  delay(400);
  while (returnPositionLine() > threshold+500 || returnPositionLine() < threshold-500)
  {
    SerialBT.println("While Uturn");
    driver.motorAReverse(lfspeed);
    driver.motorBForward(lfspeed * 0.8);
  }
  driver.motorAStop();
  driver.motorBStop();
  delay(50);
}

void ledBlink(){
  digitalWrite(BUILTIN_LED, HIGH);
  delay(100);
  digitalWrite(BUILTIN_LED, LOW);
  delay(100);
  digitalWrite(BUILTIN_LED, HIGH);
}

void ledStop(){
  delay(100);
  digitalWrite(BUILTIN_LED, LOW);
}