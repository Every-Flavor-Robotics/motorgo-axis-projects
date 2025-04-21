#include <Arduino.h>
#include <FreeRTOS.h>

#include <Adafruit_LSM6DS33.h>

#include "pins_arduino.h"  // Include our custom pins for AXIS board

const int imu_copi = 13;
const int imu_cipo = 37;
const int imu_sck = 14;
const int imu_cs = 39;

Adafruit_LSM6DS33 imu;

unsigned long last_time;

void setup()
{
  Serial.begin(115200);
  delay(2000);

  if (!imu.begin_SPI(imu_cs, imu_sck, imu_cipo, imu_copi)) {
    while (1)
    {
      Serial.println("Failed to find LSM6DS33 chip");
      delay(10);
    }
  }
  
  imu.setAccelDataRate(LSM6DS_RATE_6_66K_HZ);
  imu.setGyroDataRate(LSM6DS_RATE_6_66K_HZ);

  imu.configInt1(false, false, true); // accelerometer DRDY on INT1
  imu.configInt2(false, true, false); // gyro DRDY on INT2

  Serial.println("starting IMU Test WITH SPI!");
  last_time = micros();
}

void loop()
{
  last_time = micros();
  sensors_event_t accel;
  sensors_event_t gyro;
  sensors_event_t temp;
  imu.getEvent(&accel, &gyro, &temp);
  unsigned long now_time = micros();

  Serial.print("Acc X Data: ");
  Serial.print(accel.acceleration.x);

  Serial.print(" | Gyro X Data: ");
  Serial.print(gyro.gyro.x);

  Serial.print(" | Loop time: ");
  Serial.println(now_time - last_time);
}