constexpr int LED_PIN = LED_BUILTIN;
constexpr int TEST_ADC_PIN = A0;
constexpr unsigned long HEARTBEAT_MS = 1000;

unsigned long lastBeatMs = 0;
bool ledState = false;

void printBanner() {
  Serial.println("=== Arduino Self Check ===");
  Serial.print("Build date: ");
  Serial.print(__DATE__);
  Serial.print(" ");
  Serial.println(__TIME__);
  Serial.println("Checks:");
  Serial.println("  1) Serial output");
  Serial.println("  2) LED blink");
  Serial.println("  3) Main loop heartbeat");
  Serial.println("  4) Optional analog read on A0");
  Serial.println("==============================");
}

void setup() {
  pinMode(LED_PIN, OUTPUT);
  pinMode(TEST_ADC_PIN, INPUT);
  // Keep SPI hardware SS as output on Mega to avoid master-mode glitches.
  pinMode(53, OUTPUT);
  digitalWrite(53, HIGH);

  Serial.begin(115200);
  delay(300);

  printBanner();
  Serial.println("Target board: Mega 2560");
  Serial.println("Boot OK");
}

void loop() {
  const unsigned long now = millis();
  if (now - lastBeatMs >= HEARTBEAT_MS) {
    lastBeatMs = now;

    ledState = !ledState;
    digitalWrite(LED_PIN, ledState ? HIGH : LOW);

    int adcRaw = analogRead(TEST_ADC_PIN);
    Serial.print("Heartbeat ms=");
    Serial.print(now);
    Serial.print("  LED=");
    Serial.print(ledState ? "ON" : "OFF");
    Serial.print("  A0=");
    Serial.print(adcRaw);
    Serial.println();
  }
}
