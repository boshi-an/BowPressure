#include "DFRobot_BNO055.h"
#include "Wire.h"

typedef DFRobot_BNO055_IIC BNO;

BNO* bno = nullptr;

// On-chip gyro bandwidth (BNO055 register GYR_CONFIG0). `begin()` uses ~32 Hz by default.
// Pick one: eGyrBandWidth_523, _230, _116, _47, _23, _12, _64, _32 (lower ≈ stronger LPF).
static constexpr BNO::eGyrBandWidth_t kGyroBandwidth = BNO::eGyrBandWidth_23;

/** Must run in CONFIG mode; restores NDOF fusion mode after. */
static void applyGyroBandwidth(BNO* imu) {
  if (imu == nullptr) return;
  imu->setOprMode(BNO::eOprModeConfig);
  delay(25);
  imu->setGyrBandWidth(kGyroBandwidth);
  delay(10);
  imu->setOprMode(BNO::eOprModeNdof);
  delay(50);
}

void setup() {
  Serial.begin(115200);
  while (!Serial) { delay(10); }

  Wire.begin();
  Wire.setClock(10000);

  bno = new BNO(&Wire, 0x28);
  bno->reset();

  while (bno->begin() != BNO::eStatusOK) {
    Serial.println("bno begin failed, retrying...");
    delay(2000);
  }
  applyGyroBandwidth(bno);
  Serial.println("bno begin success");
  Serial.println("t_ms,acc_x,acc_y,acc_z,gyr_x,gyr_y,gyr_z");
  Serial.println("units: acc=mg  gyr=dps");
}

void loop() {
  if (bno == nullptr) return;

  BNO::sAxisAnalog_t sAccAnalog = bno->getAxis(BNO::eAxisAcc);  // raw accel, includes gravity

  if (bno->lastOperateStatus != BNO::eStatusOK) {
    Serial.println("Read error, reinitializing...");
    bno->reset();
    delay(500);
    while (bno->begin() != BNO::eStatusOK) {
      Serial.println("Reinit failed, retrying...");
      delay(2000);
    }
    applyGyroBandwidth(bno);
    Serial.println("Reinit success");
    return;
  }

  BNO::sAxisAnalog_t sGyrAnalog = bno->getAxis(BNO::eAxisGyr);  // angular velocity in dps

  Serial.print(millis());   Serial.print(",");
  Serial.print(sAccAnalog.x); Serial.print(",");
  Serial.print(sAccAnalog.y); Serial.print(",");
  Serial.print(sAccAnalog.z); Serial.print(",");
  Serial.print(sGyrAnalog.x); Serial.print(",");
  Serial.print(sGyrAnalog.y); Serial.print(",");
  Serial.println(sGyrAnalog.z);

  // delay(100);
}
