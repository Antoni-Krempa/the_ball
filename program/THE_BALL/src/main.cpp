#include <Arduino.h>
#include <Wire.h>
#include <LSM6.h>

Uart bluetooth(PA10, PA9);  // RX, TX
LSM6 imu;

void setup() {
  bluetooth.begin(9600);

  Wire.setSDA(PB7);
  Wire.setSCL(PB6);
  Wire.begin();

  while (!imu.init()) {
    bluetooth.println("ERR: Nie wykryto IMU. Sprawdz SDA, SCL i zasilanie.");
    delay(1000);
  }

  imu.enableDefault();
}

void loop() {
  imu.read();
  delay(50);  // Czekaj na zakończenie odczytu IMU
  // CSV: czas_ms,ax,ay,az,gx,gy,gz
  bluetooth.print(millis());
  bluetooth.print(',');
  bluetooth.print(imu.a.x);
  bluetooth.print(',');
  bluetooth.print(imu.a.y);
  bluetooth.print(',');
  bluetooth.print(imu.a.z);
  bluetooth.print(',');
  bluetooth.print(imu.g.x);
  bluetooth.print(',');
  bluetooth.print(imu.g.y);
  bluetooth.print(',');
  bluetooth.println(imu.g.z);

  delay(100);
}