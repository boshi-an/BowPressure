void setup() {
  Serial.begin(115200);
  pinMode(2, INPUT);
  attachInterrupt(digitalPinToInterrupt(2), isr, CHANGE);
  Serial.println("Listening on pin 2...");
}

volatile uint32_t edge_count = 0;

void isr() {
  edge_count++;
}

void loop() {
  static uint32_t last_count = 0;
  static uint32_t last_time  = 0;

  uint32_t now   = millis();
  uint32_t count = edge_count;

  if (now - last_time >= 500) {
    uint32_t delta = count - last_count;
    Serial.print("Edges in last 500ms: ");
    Serial.print(delta);
    Serial.print("   |   Total: ");
    Serial.println(count);
    last_count = count;
    last_time  = now;
  }
}
