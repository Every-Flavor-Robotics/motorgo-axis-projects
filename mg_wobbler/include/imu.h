

#ifndef IMU_H
#define IMU_H

// IMU includes
#include <Adafruit_AHRS.h>
#include <Adafruit_LSM6DS33.h>
#include <Arduino.h>
#include <Preferences.h>
#include <Wire.h>

// Open imu namespace
namespace Imu
{

struct gravity_vector_t
{
  float x;
  float y;
  float z;
};

// Type for saving and loading calibration parameters to/from EEPROM
const int IMU_CALIBRATION_DATA_LEN = (9 + 3 + 3 + 3) * sizeof(float);
typedef union
{
  struct __attribute__((packed))
  {
    float zero_electric_angle;

    // Eccentricity LUT
    float mag_soft_iron[9];
    float mag_hard_iron[3];
    float gyro_bias[3];
    float accel_bias[3];
  };

  uint8_t raw[IMU_CALIBRATION_DATA_LEN];
} imu_calibration_data_t;

// Helper function for calculating the CRC16 checksum
// extern uint16_t crc16_update(uint16_t crc, uint8_t a);
// extern void serial_print_motioncal(sensors_event_t &accel_event,
//                                    sensors_event_t &gyro_event,
//                                    sensors_event_t &mag_event);

extern Adafruit_LSM6DS33 lsm6ds;
// extern Adafruit_LIS3MDL lis3mdl;

class Imu
{
 public:
  Imu();

  // Init IMU sensor, with default settings. Use saved calibration if available
  void init();

  // Init IMU, with option to calibrate or not
  void init(bool should_calibrate);

  void loop();

  float get_roll();
  float get_pitch();
  float get_yaw();

  gravity_vector_t get_gravity_vector();

  // TODO: NOT IMPLEMENTED because the magdwick filter needs to be updated to
  // return this
  //   float get_pitch_rate();

  float get_raw_gyro_x();
  float get_raw_gyro_y();
  float get_raw_gyro_z();

  float get_raw_accel_x();
  float get_raw_accel_y();
  float get_raw_accel_z();

  float get_raw_mag_x();
  float get_raw_mag_y();
  float get_raw_mag_z();

 private:
  // Buffers for reading in magnetic calibration data
  float offsets[16];
  byte caldata[68];  // buffer to receive magnetic calibration data
  byte calcount = 0;
  imu_calibration_data_t calibration_data;

  // Filter
  Adafruit_Madgwick filter;
  // Last filter update time
  unsigned long last_update;

  // Event objects to store IMU data
  //   sensors_event_t accel_event, gyro_event, mag_event, temp;
  sensors_event_t accel_event, gyro_event, temp;
  float low_pass_pitch_rate = 0.0f;

  // Function for reading IMU data
  void read();

  // Function for performing IMU calibration
  // This will first do magnetic calibration using motioncal, followed by
  // accelerometer and gyroscope calibration
  void calibrate();

  // Reads and parses calibration data sent over serial from MotionCal
  bool receive_calibration();

  // Apply calibration and change units
  void apply_calibration();
};
}  // namespace Imu

#endif  // IMU_H