#include <SPI.h>
#include <Wire.h>
#include "DFRobot_BNO055.h"

// Must appear before any function definitions: Arduino injects prototypes after includes,
// so these types must be visible before generated `adsSetInputMode` / `adsConfigAdc1` protos.
enum class InputMode : uint8_t {
  SingleEnded = 0,  // AINx vs AINCOM
  Differential = 1  // AINp vs AINn
};

enum class FilterMode : uint8_t {
  FIR = 0x84,
  Sinc1 = 0x04,
  Sinc2 = 0x24,
  Sinc3 = 0x44,
  Sinc4 = 0x64
};

typedef DFRobot_BNO055_IIC BNO;
BNO* g_bno = nullptr;

static const uint8_t kBnoI2cAddr = 0x28;

// --- BNO055 configuration (CONFIG mode; DFRobot_BNO055 / Bosch BNO055) ---
static const BNO::eAccRange_t kAccRange = BNO::eAccRange_4G;
static const BNO::eGyrRange_t kGyrRange = BNO::eGyrRange_500;
static const BNO::eAccBandWidth_t kAccBandWidth = BNO::eAccBandWidth_125;
static const BNO::eGyrBandWidth_t kGyrBandWidth = BNO::eGyrBandWidth_116;

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

void printLastOperateStatus(BNO::eStatus_t eStatus) {
  switch (eStatus) {
  case BNO::eStatusOK:
    Serial.println(F("everything ok"));
    break;
  case BNO::eStatusErr:
    Serial.println(F("unknow error"));
    break;
  case BNO::eStatusErrDeviceNotDetect:
    Serial.println(F("device not detected"));
    break;
  case BNO::eStatusErrDeviceReadyTimeOut:
    Serial.println(F("device ready time out"));
    break;
  case BNO::eStatusErrDeviceStatus:
    Serial.println(F("device internal status error"));
    break;
  default:
    Serial.println(F("unknow status"));
    break;
  }
}

// ===== Pin mapping (keep UNO wiring on all boards) =====
constexpr uint8_t PIN_CS = 10;   // ADS1263 CS (same as UNO wiring)
constexpr uint8_t PIN_DRDY = 2;  // DRDY to D2 (same as UNO wiring)
constexpr uint8_t PIN_RST = 4;   // ADS1263 RESET (same as UNO wiring)
#if defined(ARDUINO_AVR_MEGA2560)
constexpr uint8_t PIN_HW_SS = 53;  // Mega SPI master pin (internal requirement)
#else
constexpr uint8_t PIN_HW_SS = 10;  // UNO/R4 SPI master pin
#endif
constexpr float ADC_REF_MV = 5000.0f;  // Set to measured AVDD in mV for accuracy
constexpr uint8_t DIFF_AIN_P = 0;      // Differential + input: IN0
constexpr uint8_t DIFF_AIN_N = 1;      // Differential - input: IN1
constexpr uint32_t STREAM_BAUD = 230400;
constexpr uint32_t STREAM_STATS_INTERVAL_MS = 1000;

volatile bool g_samplePending = false;
volatile uint32_t g_pendingTimestampUs = 0;
volatile uint32_t g_droppedEdges = 0;

uint32_t g_streamCount = 0;
uint32_t g_lastStatsMs = 0;
uint32_t g_lastStatsCount = 0;

// ===== ADS1263 command/register map (subset) =====
constexpr uint8_t CMD_RESET = 0x06;
constexpr uint8_t CMD_START1 = 0x08;
constexpr uint8_t CMD_STOP1 = 0x0A;
constexpr uint8_t CMD_RDATA1 = 0x12;

constexpr uint8_t REG_MODE0 = 0x03;
constexpr uint8_t REG_MODE1 = 0x04;   // digital filter settings
constexpr uint8_t REG_MODE2 = 0x05;   // data rate in low nibble
constexpr uint8_t REG_INPMUX = 0x06;  // [7:4]=AINP [3:0]=AINN
constexpr uint8_t REG_REFMUX = 0x0F;  // use AVDD/AVSS by default

constexpr uint8_t AINCOM = 0x0A;

// DR code table from ADS1263 MODE2.DR[3:0] setting.
// This helper focuses on common rates and picks the nearest one.
uint8_t rateToDrCode(uint16_t sps) {
  struct Pair {
    uint16_t sps;
    uint8_t dr;
  };
  static const Pair table[] = {
      {2, 0x00},   {5, 0x01},    {10, 0x02},   {16, 0x03},   {20, 0x04},
      {50, 0x05},  {60, 0x06},   {100, 0x07},  {400, 0x08},  {1200, 0x09},
      {2400, 0x0A}, {4800, 0x0B}, {7200, 0x0C}, {14400, 0x0D}, {19200, 0x0E},
      {38400, 0x0F}};

  uint8_t best = table[0].dr;
  uint16_t bestDiff = 65535;
  for (const auto &item : table) {
    uint16_t diff = (item.sps > sps) ? (item.sps - sps) : (sps - item.sps);
    if (diff < bestDiff) {
      bestDiff = diff;
      best = item.dr;
    }
  }
  return best;
}

void csLow() { digitalWrite(PIN_CS, LOW); }
void csHigh() { digitalWrite(PIN_CS, HIGH); }

void adsWriteCommand(uint8_t cmd) {
  csLow();
  SPI.transfer(cmd);
  csHigh();
}

void adsWriteRegister(uint8_t reg, uint8_t value) {
  // WREG format: 010r rrrr, then count-1, then data bytes.
  csLow();
  SPI.transfer(0x40 | (reg & 0x1F));
  SPI.transfer(0x00);  // write 1 register
  SPI.transfer(value);
  csHigh();
}

uint8_t adsReadRegister(uint8_t reg) {
  // RREG format: 001r rrrr, then count-1, then read data bytes.
  csLow();
  SPI.transfer(0x20 | (reg & 0x1F));
  SPI.transfer(0x00);  // read 1 register
  uint8_t v = SPI.transfer(0x00);
  csHigh();
  return v;
}

void adsHardwareReset() {
  digitalWrite(PIN_RST, LOW);
  delay(2);
  digitalWrite(PIN_RST, HIGH);
  delay(10);
}

void adsInit() {
  pinMode(PIN_HW_SS, OUTPUT);
  digitalWrite(PIN_HW_SS, HIGH);
  pinMode(PIN_CS, OUTPUT);
  pinMode(PIN_RST, OUTPUT);
  pinMode(PIN_DRDY, INPUT_PULLUP);
  csHigh();
  digitalWrite(PIN_RST, HIGH);

  SPI.begin();
  // ADS1263 uses SPI mode 1, MSB first, <=8MHz. Use 4MHz for stability.
  SPI.beginTransaction(SPISettings(4000000, MSBFIRST, SPI_MODE1));

  adsHardwareReset();
  adsWriteCommand(CMD_RESET);
  delay(5);
}

// 1) Activate input mode and selected channel(s)
void adsSetInputMode(InputMode mode, uint8_t ainP, uint8_t ainN) {
  if (mode == InputMode::SingleEnded) {
    // Single-ended: channel vs AINCOM.
    if (ainP >= 10 || ainN >= 10) {
      Serial.println("Error: Single-ended input must be between 0 and 9");
      return;
    }
    adsWriteRegister(REG_INPMUX, ((ainP & 0x0F) << 4) | (AINCOM & 0x0F));
  } else {
    if (ainP >= 10 || ainN >= 10) {
      Serial.println("Error: Differential input must be between 0 and 9");
      return;
    }
    if (ainP == ainN) {
      Serial.println("Error: Differential input must be different");
      return;
    }
    if (ainP+1 != ainN or ainP%2 != 0) {
      Serial.println("Error: Differential input must be consecutive and paired");
      return;
    }
    // Differential: AINp vs AINn.
    adsWriteRegister(REG_INPMUX, ((ainP & 0x0F) << 4) | (ainN & 0x0F));
  }
}

bool adsWriteRegVerified(uint8_t reg, uint8_t value, const char *name) {
  adsWriteRegister(reg, value);
  delay(1);
  uint8_t got = adsReadRegister(reg);
  bool ok = (got == value);
  Serial.print(name);
  Serial.println(ok ? " success" : " unsuccess");
  if (!ok) {
    Serial.print("  expected=0x");
    Serial.print(value, HEX);
    Serial.print(" got=0x");
    Serial.println(got, HEX);
  }
  return ok;
}

// Mirrors Waveshare ADS1263_ConfigADC1 flow:
// MODE2 -> REFMUX -> MODE0 -> MODE1, each with readback verification.
void adsConfigAdc1(uint8_t gain, uint8_t drate, uint8_t delayReg, FilterMode filterMode) {
  uint8_t mode2 = 0x80;  // PGA bypassed
  mode2 |= ((gain & 0x07) << 4) | (drate & 0x0F);
  adsWriteRegVerified(REG_MODE2, mode2, "REG_MODE2");

  uint8_t refmux = 0x24;  // AVDD/AVSS as reference (matches Waveshare/Pi example)
  adsWriteRegVerified(REG_REFMUX, refmux, "REG_REFMUX");

  uint8_t mode0 = delayReg;
  adsWriteRegVerified(REG_MODE0, mode0, "REG_MODE0");

  if (FilterMode::FIR == filterMode && drate > 20) {
    Serial.println("Error: FIR filter is not supported for data rates greater than 20 SPS");
    return;
  }
  uint8_t mode1 = static_cast<uint8_t>(filterMode);
  adsWriteRegVerified(REG_MODE1, mode1, "REG_MODE1");
}

int32_t adsReadAdc1RawInline() {
  csLow();
  SPI.transfer(CMD_RDATA1);
  (void)SPI.transfer(0x00);  // status
  uint8_t b3 = SPI.transfer(0x00);
  uint8_t b2 = SPI.transfer(0x00);
  uint8_t b1 = SPI.transfer(0x00);
  uint8_t b0 = SPI.transfer(0x00);
  (void)SPI.transfer(0x00);  // checksum
  csHigh();
  return (int32_t)(((uint32_t)b3 << 24) | ((uint32_t)b2 << 16) |
                   ((uint32_t)b1 << 8) | (uint32_t)b0);
}

void onDrdyFalling() {
  uint32_t t = micros();
  if (g_samplePending) {
    ++g_droppedEdges;
  } else {
    g_pendingTimestampUs = t;
    g_samplePending = true;
  }
}

float rawToMilliVolts(int32_t raw) {
  // ADS1263 bipolar code: -0x80000000 .. +0x7FFFFFFF maps to -VREF .. +VREF.
  constexpr float ADC_FULL_SCALE_POS = 2147483647.0f;  // 0x7FFFFFFF
  return (static_cast<float>(raw) / ADC_FULL_SCALE_POS) * ADC_REF_MV;
}

float sqrtNewton(float x) {
  if (x <= 0.0f) return 0.0f;
  float r = x;
  for (uint8_t i = 0; i < 12; ++i) {
    r = 0.5f * (r + x / r);
  }
  return r;
}

void setup() {
  Serial.begin(STREAM_BAUD);
  while (!Serial) { delay(10); }

  Wire.begin();
  Wire.setClock(50000);

  g_bno = new BNO(&Wire, kBnoI2cAddr);
  g_bno->reset();

  while (g_bno->begin() != BNO::eStatusOK) {
    Serial.println(F("bno begin failed, retrying..."));
    printLastOperateStatus(g_bno->lastOperateStatus);
    delay(2000);
  }
  Serial.println(F("bno begin success"));

  configureBno055(g_bno);
  Serial.println(
      F("configured: ACCGYRO, 4g / 500 dps, NORMAL power; IMU via getAxis (acc, gyr)"));
  Wire.setClock(50000);

  adsInit();

  // Differential mode on IN0-IN1, ~100 SPS (DRDY drives sample rate; IMU read once per ADC sample).
  adsSetInputMode(InputMode::Differential, DIFF_AIN_P, DIFF_AIN_N);
  adsConfigAdc1(
      0x00,               // gain=1
      rateToDrCode(100),  // drate target
      0x00,               // conversion delay register
      FilterMode::Sinc1   // filter mode
  );

  const int drdyInterrupt = digitalPinToInterrupt(PIN_DRDY);
  if (drdyInterrupt < 0) {
    Serial.println(F("Error: PIN_DRDY does not support external interrupt"));
    while (true) {
      delay(1000);
    }
  }
  attachInterrupt(drdyInterrupt, onDrdyFalling, FALLING);
  adsWriteCommand(CMD_START1);
  delay(10);

  g_lastStatsMs = millis();

  Serial.println(F("ADS1263 + BNO055 fused stream (~100 Hz from ADC DRDY)"));
#if defined(ARDUINO_AVR_MEGA2560)
  Serial.println(F("Target board: Arduino Mega 2560"));
#else
  Serial.println(F("Target board: Arduino UNO-class"));
#endif
  Serial.print(F("ADC1 differential: IN"));
  Serial.print(DIFF_AIN_P);
  Serial.print(F(" - IN"));
  Serial.println(DIFF_AIN_N);
  Serial.print(F("Serial baud="));
  Serial.println(STREAM_BAUD);
  Serial.println(
      F("# DATA: timestamp_us,raw,mV,acc_x,acc_y,acc_z,gyr_x,gyr_y,gyr_z | acc=mg gyr=dps"));
}

void loop() {
  bool hasSample = false;
  uint32_t tsUs = 0;
  noInterrupts();
  if (g_samplePending) {
    hasSample = true;
    tsUs = g_pendingTimestampUs;
    g_samplePending = false;
  }
  interrupts();

  if (hasSample) {
    int32_t raw = adsReadAdc1RawInline();
    float mv = rawToMilliVolts(raw);

    if (g_bno == nullptr) return;

    BNO::sAxisAnalog_t sAcc = g_bno->getAxis(BNO::eAxisAcc);
    BNO::sAxisAnalog_t sGyr = g_bno->getAxis(BNO::eAxisGyr);

    ++g_streamCount;

    Serial.print(F("DATA,"));
    Serial.print(tsUs);      Serial.print(',');
    Serial.print(raw);       Serial.print(',');
    Serial.print(mv, 3);     Serial.print(',');
    Serial.print((int)round(sAcc.x)); Serial.print(',');
    Serial.print((int)round(sAcc.y)); Serial.print(',');
    Serial.print((int)round(sAcc.z)); Serial.print(',');
    Serial.print(sGyr.x, 2);  Serial.print(',');
    Serial.print(sGyr.y, 2);  Serial.print(',');
    Serial.println(sGyr.z, 2);
  }

  uint32_t nowMs = millis();
  if (nowMs - g_lastStatsMs >= STREAM_STATS_INTERVAL_MS) {
    uint32_t dCount = g_streamCount - g_lastStatsCount;
    float fps = (1000.0f * static_cast<float>(dCount)) /
                static_cast<float>(nowMs - g_lastStatsMs);
    uint32_t dropped = 0;
    noInterrupts();
    dropped = g_droppedEdges;
    interrupts();

    Serial.print(F("STATS,fps="));
    Serial.print(fps, 2);
    Serial.print(F(",count="));
    Serial.print(g_streamCount);
    Serial.print(F(",dropped_edges="));
    Serial.println(dropped);

    g_lastStatsMs = nowMs;
    g_lastStatsCount = g_streamCount;
  }
}
