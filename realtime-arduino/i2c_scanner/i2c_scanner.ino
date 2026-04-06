#include <Wire.h>

// Simple I2C scanner.
// Prints all 7-bit addresses that ACK on the bus.

constexpr uint32_t SERIAL_BAUD = 230400;

void setup() {
  Serial.begin(SERIAL_BAUD);
  while (!Serial) {
    ;
  }

  Wire.begin();
  Wire.setClock(400000);  // try 400kHz

  Serial.println("I2C scanner starting...");
  Serial.println("Scanning 7-bit addresses 0x08..0x77");
}

void loop() {
  uint8_t found = 0;

  for (uint8_t addr = 0x08; addr <= 0x77; ++addr) {
    Wire.beginTransmission(addr);
    uint8_t err = Wire.endTransmission();
    if (err == 0) {
      Serial.print("Found device at 0x");
      if (addr < 16) Serial.print("0");
      Serial.println(addr, HEX);
      ++found;
    } else if (err == 4) {
      Serial.print("Unknown error at 0x");
      if (addr < 16) Serial.print("0");
      Serial.println(addr, HEX);
    }
  }

  if (found == 0) {
    Serial.println("No I2C devices found.");
  } else {
    Serial.print("Total devices found: ");
    Serial.println(found);
  }

  Serial.println("Rescanning in 2 seconds...\n");
  delay(2000);
}
