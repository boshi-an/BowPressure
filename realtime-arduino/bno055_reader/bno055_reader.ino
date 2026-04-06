/*!
 * read_data.ino
 *
 * Download this demo to test read data from bno055
 * Data will print on your serial monitor
 *
 * Product: http://www.dfrobot.com.cn/goods-1860.html
 * Copyright   [DFRobot](http://www.dfrobot.com), 2016
 * Copyright   GNU Lesser General Public License
 *
 * version  V1.0
 * date  07/03/2019
 */

#include "DFRobot_BNO055.h"
#include "Wire.h"

typedef DFRobot_BNO055_IIC
    BNO; // ******** use abbreviations instead of full names ********

BNO* g_bno = nullptr;

// --- IMU configuration (CONFIG mode writes; see DFRobot_BNO055 / Bosch BNO055 datasheet) ---
// Full-scale ranges (library: eAccRange_t / eGyrRange_t).
static const BNO::eAccRange_t kAccRange = BNO::eAccRange_4G;
static const BNO::eGyrRange_t kGyrRange = BNO::eGyrRange_500;
// Accelerometer / gyroscope bandwidth (library maps these to chip BW bits; higher = faster
// settling, more responsive "sampling" of motion — use lower values to reduce noise).
static const BNO::eAccBandWidth_t kAccBandWidth = BNO::eAccBandWidth_125;
static const BNO::eGyrBandWidth_t kGyrBandWidth = BNO::eGyrBandWidth_116;

// Apply ACC+GYRO only mode, ranges, bandwidth, and highest-performance power settings:
// system PWR_MODE NORMAL, ACC_PWR_MODE NORMAL, GYR_PWR_MODE NORMAL (not low-power / suspend).
static void configureBno055(BNO* bno) {
  bno->setOprMode(BNO::eOprModeConfig);
  bno->setAccRange(kAccRange);
  bno->setGyrRange(kGyrRange);
  bno->setPowerMode(BNO::ePowerModeNormal);
  bno->setAccPowerMode(BNO::eAccPowerModeNormal);
  bno->setGyrPowerMode(BNO::eGyrPowerModeNormal);
  bno->setAccBandWidth(kAccBandWidth);
  bno->setGyrBandWidth(kGyrBandWidth);
  bno->setOprMode(BNO::eOprModeAccGyro);
  delay(2000);
}

// show last sensor operate status
void printLastOperateStatus(BNO::eStatus_t eStatus) {
  switch (eStatus) {
  case BNO::eStatusOK:
    Serial.println("everything ok");
    break;
  case BNO::eStatusErr:
    Serial.println("unknow error");
    break;
  case BNO::eStatusErrDeviceNotDetect:
    Serial.println("device not detected");
    break;
  case BNO::eStatusErrDeviceReadyTimeOut:
    Serial.println("device ready time out");
    break;
  case BNO::eStatusErrDeviceStatus:
    Serial.println("device internal status error");
    break;
  default:
    Serial.println("unknow status");
    break;
  }
}

void setup() {
  Serial.begin(230400);
  Serial.println("bno055_reader setup");
  g_bno = new BNO(&Wire, 0x28);
  g_bno->reset();
  while (g_bno->begin() != BNO::eStatusOK) {
    Serial.println("bno begin faild");
    printLastOperateStatus(g_bno->lastOperateStatus);
    delay(2000);
  }
  Serial.println("bno begin success");

  configureBno055(g_bno);
  Serial.println(
      "configured: ACCGYRO, 4g / 500 dps, NORMAL power; serial: raw acc (mg), gyro (dps)");
}

#define printAxisData(sAxis)                                                   \
  Serial.print(" z: ");                                                        \
  Serial.print(sAxis.z)

void loop() {
  BNO::sAxisAnalog_t sAccAnalog, sGyrAnalog;
  sAccAnalog = g_bno->getAxis(BNO::eAxisAcc);
  sGyrAnalog = g_bno->getAxis(BNO::eAxisGyr);
  Serial.print("acc (mg)  ");
  printAxisData(sAccAnalog);
  // Serial.print("gyr (dps) ");
  // printAxisData(sGyrAnalog);
  Serial.println();

  delay(10);
}
