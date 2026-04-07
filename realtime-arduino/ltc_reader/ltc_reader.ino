#define LTC_PIN 2
#define LTC_FPS 25
#define SAMPLE_RATE 48000

// Derived timing thresholds (microseconds)
// Half-bit period = 1000000 / (FPS * 80 * 2)
#define HALF_BIT_US (1000000 / (LTC_FPS * 80 * 2))
#define SHORT_MIN (HALF_BIT_US / 3)
#define SHORT_MAX (HALF_BIT_US * 3 / 2)
#define LONG_MAX (HALF_BIT_US * 3)

void push_bit(uint8_t bit);

volatile uint32_t last_edge = 0;
volatile uint8_t half_count = 0;
volatile uint8_t bit_count = 0;
volatile uint8_t bit_buf[10] = {0};
volatile bool frame_ready = false;
volatile uint8_t frame[10] = {0};

void isr() {
  uint32_t now = micros();
  uint32_t duration = now - last_edge;
  last_edge = now;

  if (duration < SHORT_MIN || duration > LONG_MAX) {
    half_count = 0;
    return;
  }

  if (duration <= SHORT_MAX) {
    half_count++;
    if (half_count == 2) {
      half_count = 0;
      push_bit(1);
    }
  } else {
    half_count = 0;
    push_bit(0);
  }
}

void push_bit(uint8_t bit) {
  for (int i = 0; i < 9; i++)
    bit_buf[i] = (bit_buf[i] >> 1) | ((bit_buf[i + 1] & 0x01) << 7);
  bit_buf[9] = (bit_buf[9] >> 1) | (bit << 7);

  bit_count++;
  if (bit_count > 80)
    bit_count = 80;

  uint16_t sync = ((uint16_t)bit_buf[9] << 8) | bit_buf[8];
  if ((sync == 0xBFFC || sync == 0x3FFD) && bit_count == 80) {
    memcpy((void *)frame, (void *)bit_buf, 10);
    frame_ready = true;
    bit_count = 0;
    memset((void *)bit_buf, 0, 10);
  }
}

void decode_and_print() {
  uint8_t f[10];
  noInterrupts();
  memcpy(f, (void *)frame, 10);
  frame_ready = false;
  interrupts();

  uint8_t ff = (f[0] & 0x0F) + ((f[1] & 0x03) * 10);
  uint8_t ss = (f[2] & 0x0F) + ((f[3] & 0x07) * 10);
  uint8_t mm = (f[4] & 0x0F) + ((f[5] & 0x07) * 10);
  uint8_t hh = (f[6] & 0x0F) + ((f[7] & 0x03) * 10);

  char buf[12];
  snprintf(buf, sizeof(buf), "%02d:%02d:%02d:%02d", hh, mm, ss, ff);
  Serial.println(buf);
}

void setup() {
  Serial.begin(115200);
  pinMode(LTC_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(LTC_PIN), isr, CHANGE);

  Serial.println("LTC decoder ready");
  Serial.print("FPS: ");
  Serial.println(LTC_FPS);
  Serial.print("Half-bit: ");
  Serial.print(HALF_BIT_US);
  Serial.println("us");
  Serial.print("SHORT_MAX: ");
  Serial.print(SHORT_MAX);
  Serial.println("us");
  Serial.print("LONG_MAX: ");
  Serial.print(LONG_MAX);
  Serial.println("us");
}

void loop() {
  if (frame_ready) {
    decode_and_print();
  }

  // Your application code here — never blocked
}
