#include "BluetoothSerial.h"

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

void setup() {
  // put your setup code here, to run once:

  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
  pinMode(EEP, OUTPUT);
  pinMode(ULT, INPUT_PULLUP);
  digitalWrite(EEP, HIGH);

}

void loop() {
  // put your main code here, to run repeatedly:
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);

}
