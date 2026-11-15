#include "BluetoothSerial.h"

BluetoothSerial SerialBT;
uint8_t hc05[6] = {0x98, 0xD3, 0xB1, 0xFE, 0xE1, 0x7C};

void setup() {
  Serial.begin(115200);
  SerialBT.begin("ESP32_master", true);
  SerialBT.setPin("1234", 4);

  Serial.println("Connecting to HC-05...");
  while (!SerialBT.connect(hc05)) {
    Serial.println("Connection failed, retrying...");
    delay(1000);
  }
  Serial.println("Connected to HC-05!");
}

void loop() {
  if (Serial.available()) SerialBT.write(Serial.read());
  if (SerialBT.available()) Serial.write(SerialBT.read());
}