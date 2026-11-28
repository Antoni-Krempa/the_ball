#include "BluetoothSerial.h"

BluetoothSerial SerialBT;
uint8_t hc05[6] = {0x98, 0xD3, 0xB1, 0xFE, 0xE1, 0x7C};

QueueHandle_t myQueue;
const char PACKET_END = ',';   // change to '\n' once the STM32 sends one line per sample
unsigned long lastSeq = 0;
bool firstPacket = true;
unsigned long receivedPackets = 0;
unsigned long lostPackets = 0;

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

// Called for every complete packet
void handlePacket(char *packet) {
  char *endPtr;
  unsigned long seq = strtoul(packet, &endPtr, 10);
  if (endPtr == packet) return;   // no number found, ignore

  if (!firstPacket && seq > lastSeq + 1) {
    lostPackets += seq - lastSeq - 1;
  }
  firstPacket = false;
  lastSeq = seq;
  receivedPackets++;

  Serial.printf("seq=%lu received=%lu lost=%lu\n", seq, receivedPackets, lostPackets);
}

// Task 2: builds packets from the byte stream and processes them
void parseTask(void *param) {
  uint8_t data;
  char packet[64];
  size_t len = 0;
  for (;;) {
    if (xQueueReceive(myQueue, &data, portMAX_DELAY) == pdTRUE) {
      if (data == PACKET_END) {
        packet[len] = '\0';
        if (len > 0) handlePacket(packet);
        len = 0;
      } else if (data != '\r' && data != '\n') {
        if (len < sizeof(packet) - 1) packet[len++] = data;
        else len = 0;   // packet too long, drop it
      }
    }
  }
}

// Test only: simulates the STM32 stream (with some packets missing)
void simulatorTask(void *param) {
  unsigned long n = 0;
  char buf[16];
  for (;;) {
    n++;
    if (n % 10 != 0) {   // "lose" every 10th packet
      int len = snprintf(buf, sizeof(buf), "%lu,", n);
      for (int i = 0; i < len; i++) {
        uint8_t b = buf[i];
        xQueueSend(myQueue, &b, 0);
      }
    }
    vTaskDelay(pdMS_TO_TICKS(50));
  }
}

void setup() {
  Serial.begin(115200);
//  SerialBT.begin("ESP32_master", true);
//  SerialBT.setPin("1234", 4);

//  Serial.println("Connecting to HC-05...");
//  while (!SerialBT.connect(hc05)) {
//    Serial.println("Connection failed, retrying...");
//    delay(1000);
//  }
//  Serial.println("Connected to HC-05!");

  // The queue must be created BEFORE the tasks that use it
  myQueue = xQueueCreate(512, sizeof(uint8_t));

  xTaskCreate(simulatorTask, "receive", 4096, NULL, 2, NULL);
  xTaskCreate(parseTask, "parse", 4096, NULL, 1, NULL);
}

void loop() {
//  if (!SerialBT.connected(1000)) {
//    Serial.println("Connection lost, reconnecting...");
//    while (!SerialBT.connect(hc05)) {
//      Serial.println("Connection failed, retrying...");
//      vTaskDelay(pdMS_TO_TICKS(1000));
//    }
//    Serial.println("Reconnected to HC-05!");
//  }
  vTaskDelay(pdMS_TO_TICKS(100));
}