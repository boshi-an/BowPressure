#pragma once
#include <Wire.h>

// BNO055 I2C address and raw data register base
static constexpr uint8_t kBno055Addr       = 0x28;
static constexpr uint8_t kRegAccDataXLsb   = 0x08;  // ACC X/Y/Z: 0x08–0x0D
// Gyro X/Y/Z: 0x14–0x19  (magnetometer occupies 0x0E–0x13, skipped)

struct BnoRawSample {
  int16_t acc_x, acc_y, acc_z;
  int16_t gyr_x, gyr_y, gyr_z;
};

// Single 18-byte burst read: sets register pointer once, reads accel + mag + gyro
// in one transaction, discards the 6 magnetometer bytes in the middle.
// Returns false on I2C error.
inline bool bnoReadBurst(BnoRawSample &s) {
  Wire.beginTransmission(kBno055Addr);
  Wire.write(kRegAccDataXLsb);
  if (Wire.endTransmission(false) != 0) return false;

  if (Wire.requestFrom(kBno055Addr, (uint8_t)18) < 18) return false;

  uint8_t buf[18];
  for (uint8_t i = 0; i < 18; i++) buf[i] = Wire.read();

  // Accel: bytes 0–5
  s.acc_x = (int16_t)((buf[1]  << 8) | buf[0]);
  s.acc_y = (int16_t)((buf[3]  << 8) | buf[2]);
  s.acc_z = (int16_t)((buf[5]  << 8) | buf[4]);
  // bytes 6–11 = magnetometer, ignored
  // Gyro: bytes 12–17
  s.gyr_x = (int16_t)((buf[13] << 8) | buf[12]);
  s.gyr_y = (int16_t)((buf[15] << 8) | buf[14]);
  s.gyr_z = (int16_t)((buf[17] << 8) | buf[16]);

  return true;
}
