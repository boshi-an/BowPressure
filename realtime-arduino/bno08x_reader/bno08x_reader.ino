#include <Wire.h>

// Uses Adafruit_BNO08x library (install via Library Manager):
// - "Adafruit BNO08x"
// - "Adafruit BusIO"
//
// Target board: Arduino Mega 2560
//
// Wiring (I2C):
// - IMU SDA -> Mega SDA (D20 / SDA pin)
// - IMU SCL -> Mega SCL (D21 / SCL pin)
// - IMU 3V3/VIN -> per your breakout
// - IMU GND -> Mega GND
//
// Output format (CSV):
// IMU,t_us,ax,ay,az,gx,gy,gz,qw,qx,qy,qz

#include <Adafruit_BNO08x.h>

constexpr uint32_t SERIAL_BAUD = 230400;
constexpr uint32_t REPORT_INTERVAL_US = 5000;  // 200 Hz target reports (actual depends on sensor/config)

Adafruit_BNO08x bno08x(-1);
sh2_SensorValue_t sensorValue;

constexpr uint8_t BNO08X_I2C_ADDR = 0x4A;  // detected by i2c_scanner

struct Vec3 {
  float x = 0, y = 0, z = 0;
  bool valid = false;
};

struct Quat {
  float w = 1, x = 0, y = 0, z = 0;
  bool valid = false;
};

Vec3 accel;
Vec3 gyro;
Quat rot;

static float qToFloat(int16_t fixed, int q) {
  // Convert SH2 fixed-point to float: value = fixed / (2^q)
  return (float)fixed / (float)(1 << q);
}

void enableReports() {
  // Pick a small set of common reports.
  // If you want more/less, enable/disable here.
  bno08x.enableReport(SH2_ACCELEROMETER, REPORT_INTERVAL_US);
  bno08x.enableReport(SH2_GYROSCOPE_CALIBRATED, REPORT_INTERVAL_US);
  bno08x.enableReport(SH2_ROTATION_VECTOR, REPORT_INTERVAL_US);
}

void setup() {
  Serial.begin(SERIAL_BAUD);
  while (!Serial) {
    ;
  }

  Wire.begin();
  Wire.setClock(1000);  // 400kHz I2C

  Serial.println("Target board: Mega 2560");
  Serial.println("BNO08x init...");

  delay(1000);
  // Try the detected address first, then fallback to library default.
  if (!bno08x.begin_I2C(BNO08X_I2C_ADDR) && !bno08x.begin_I2C()) {
    Serial.print("ERROR: BNO08x not detected on I2C (tried 0x");
    Serial.print(BNO08X_I2C_ADDR, HEX);
    Serial.println(" and default)");
    while (true) {
      delay(1000);
    }
  }

  enableReports();

  for (int n = 0; n < bno08x.prodIds.numEntries; n++) {
    Serial.print("Part ");
    Serial.print(bno08x.prodIds.entry[n].swPartNumber);
    Serial.print(": Version :");
    Serial.print(bno08x.prodIds.entry[n].swVersionMajor);
    Serial.print(".");
    Serial.print(bno08x.prodIds.entry[n].swVersionMinor);
    Serial.print(".");
    Serial.print(bno08x.prodIds.entry[n].swVersionPatch);
    Serial.print(" Build ");
    Serial.println(bno08x.prodIds.entry[n].swBuildNumber);
  }

  Serial.println("BNO08x ready");
  Serial.println("IMU,t_us,ax,ay,az,gx,gy,gz,qw,qx,qy,qz");
}

void setReports(void) {
  Serial.println("Setting desired reports");
  if (! bno08x.enableReport(SH2_GAME_ROTATION_VECTOR)) {
    Serial.println("Could not enable game vector");
  }
}

void loop() {
  delay(10);
  if (bno08x.wasReset()) {
    Serial.print("sensor was reset ");
    setReports();
  }
  if (! bno08x.getSensorEvent(&sensorValue)) {
  
  }
  switch (sensorValue.sensorId) {
    case SH2_GAME_ROTATION_VECTOR:
    Serial.print("Game Rotation Vector - r: ");
    Serial.print(sensorValue.un.gameRotationVector.real);
    Serial.print(" i: ");
    Serial.print(sensorValue.un.gameRotationVector.i);
    Serial.print(" j: ");
    Serial.print(sensorValue.un.gameRotationVector.j);
    Serial.print(" k: ");
    Serial.println(sensorValue.un.gameRotationVector.k);
    break;
  }
}
