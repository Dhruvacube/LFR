#include "BluetoothSerial.h"
#include <QTRSensors.h>

#define USE_PIN
const char *pin = "7842";

String device_name = "HOKAGE_Meshmerize";

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

#if !defined(CONFIG_BT_SPP_ENABLED)
#error Serial Bluetooth not available or not enabled. It is only available for the ESP32 chip.
#endif

#define button GPIO_NUM_15

BluetoothSerial SerialBT;
QTRSensors qtr;

const uint8_t SensorCount = 8;
const uint8_t sensors[SensorCount] = {GPIO_NUM_13, GPIO_NUM_12, GPIO_NUM_14, GPIO_NUM_27, GPIO_NUM_26, GPIO_NUM_25, GPIO_NUM_33, GPIO_NUM_32};
uint16_t sensorValues[SensorCount];

void setup() {
  // put your setup code here, to run once:
  SerialBT.begin(device_name); //Bluetooth device name
  pinMode(BUILTIN_LED, OUTPUT);
  pinMode(button, INPUT_PULLUP);
  
  qtr.setTypeRC();
  qtr.setSensorPins(sensors, SensorCount);

  while(digitalRead(button) == HIGH) {
    delay(100);
    digitalWrite(BUILTIN_LED, HIGH);
    delay(100);
    digitalWrite(BUILTIN_LED, LOW);
  }
  digitalWrite(BUILTIN_LED, LOW);
  for (uint16_t i = 0; i < 400; i++)
  {
    qtr.calibrate();
  }

  while(digitalRead(button) == HIGH) {
    delay(100);
    digitalWrite(BUILTIN_LED, HIGH);
    delay(100);
    digitalWrite(BUILTIN_LED, LOW);
  }
  digitalWrite(BUILTIN_LED, LOW);
}

void loop() {
  // put your main code here, to run repeatedly:
  digitalWrite(BUILTIN_LED, HIGH);
  uint16_t positionLine = qtr.readLineWhite(sensorValues);
  SerialBT.println(positionLine);
  digitalWrite(BUILTIN_LED, LOW);
  delay(100);
}
