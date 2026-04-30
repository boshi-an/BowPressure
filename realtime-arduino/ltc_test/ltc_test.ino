#define LTC_PIN 3
#define LTC_FPS 30
#define SAMPLE_RATE 48000

// Derived timing thresholds (microseconds)
// Half-bit period = 1000000 / (FPS * 80 * 2)
#define HALF_BIT_US (1000000 / (LTC_FPS * 80 * 2))
#define SHORT_MIN (HALF_BIT_US / 3)
#define SHORT_MAX (HALF_BIT_US * 3 / 2)
#define LONG_MAX (HALF_BIT_US * 3)

void setup() {
  Serial.begin(230400);
  pinMode(LTC_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(LTC_PIN), isr, CHANGE);

  while (!Serial) {
    delay(100);
  }

  Serial.println("Listening on pin ");
  Serial.print(LTC_PIN);
  Serial.println("...");

  Serial.print("Half-bit: ");
  Serial.print(HALF_BIT_US);
  Serial.println("us");
  Serial.print("SHORT_MAX: ");
  Serial.print(SHORT_MAX);
  Serial.println("us");
  Serial.print("LONG_MAX: ");
  Serial.print(LONG_MAX);
  Serial.println("us");
  Serial.print("SHORT_MIN: ");
  Serial.print(SHORT_MIN);
  Serial.println("us");
}

volatile uint32_t edge_count = 0;
volatile uint32_t last_edge = 0;

void isr() {
  uint32_t now = micros();
  uint32_t duration = now - last_edge;
  last_edge = now;

  Serial.print(duration);
  Serial.println();

  edge_count++;
}

void loop() {
  static uint32_t last_count = 0;
  static uint32_t last_time  = 0;

  uint32_t now   = millis();
  uint32_t count = edge_count;

  if (now - last_time >= 500) {
    uint32_t delta = count - last_count;
    // Serial.print("Edges in last 500ms: ");
    // Serial.print(delta);
    // Serial.print("   |   Total: ");
    // Serial.println(count);
    last_count = count;
    last_time  = now;
  }
}
