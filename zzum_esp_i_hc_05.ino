#include "BluetoothSerial.h"

BluetoothSerial SerialBT;
uint8_t hc05[6] = {0x98, 0xD3, 0xB1, 0xFE, 0xE1, 0x7C};

QueueHandle_t myQueue;

// Task 1: reads bytes from HC-05 and puts them into the queue
void receiveTask(void *param) {
  for (;;) {
    while (SerialBT.available()) {
      uint8_t data = SerialBT.read();
      xQueueSend(myQueue, &data, 0);
    }
    vTaskDelay(pdMS_TO_TICKS(1));
  }
}

// Task 2: takes bytes from the queue and prints them to the PC
void printTask(void *param) {
  uint8_t data;
  for (;;) {
    if (xQueueReceive(myQueue, &data, portMAX_DELAY) == pdTRUE) {
      Serial.write(data);
    }
  }
}

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

  // The queue must be created BEFORE the tasks that use it
  myQueue = xQueueCreate(512, sizeof(uint8_t));

  xTaskCreate(receiveTask, "receive", 4096, NULL, 2, NULL);
  xTaskCreate(printTask, "print", 4096, NULL, 1, NULL);
}

void loop() {
  // Test only: PC -> HC-05 (later the data will come from STM32)
  if (Serial.available()) {
    SerialBT.write(Serial.read());
  }
  vTaskDelay(pdMS_TO_TICKS(1));
}