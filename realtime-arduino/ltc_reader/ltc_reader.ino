// LTC Timecode Decoder for Arduino UNO R4 Minima
// Signal input: Pin 2 (interrupt pin)
// Baud: 115200 on Serial monitor

#define LTC_PIN 2

// ── Biphase-mark timing constants ──────────────────────────────
// At 25fps: bit period = 1/25/80 = 500µs, half = 250µs
// We treat anything < 375µs as a "short" pulse (= 1 bit)
// and anything > 375µs as a "long"  pulse (= 0 bit boundary)
#define SHORT_MIN  80
#define SHORT_MAX  375
#define LONG_MIN   376
#define LONG_MAX   800

// ── LTC frame is 80 bits ────────────────────────────────────────
#define LTC_FRAME_BITS 80

// ── Sync word: last 16 bits of every LTC frame ──────────────────
#define LTC_SYNC_WORD 0xBFFC

// ── Shared state (ISR writes, loop reads) ───────────────────────
volatile uint64_t ltc_shift  = 0;      // 80-bit shift register (low 64 bits)
volatile uint16_t ltc_shift2 = 0;      // high 16 bits
volatile bool     ltc_ready  = false;  // frame waiting to be read
volatile uint8_t  ltc_frame[10];       // decoded frame bytes

// Snapshot for main loop (safe copy)
uint8_t frame_snap[10];

// ── ISR state ───────────────────────────────────────────────────
volatile uint32_t last_edge   = 0;
volatile bool     last_bit    = false;  // last decoded bit
volatile uint8_t  half_count  = 0;      // biphase half-cycle counter
volatile uint8_t  bit_count   = 0;
volatile uint8_t  bit_buf[10] = {0};
volatile bool     in_sync     = false;

// ── Helpers ─────────────────────────────────────────────────────
void push_bit(bool bit) {
  // Shift bit into 80-bit buffer (LSB first, as per SMPTE)
  for (int i = 9; i > 0; i--)
    bit_buf[i] = (bit_buf[i] >> 1) | ((bit_buf[i-1] & 1) << 7);
  bit_buf[0] = (bit_buf[0] >> 1) | (bit ? 0x80 : 0x00);
  bit_count++;

  if (bit_count >= LTC_FRAME_BITS) {
    // Check sync word — last 16 bits of frame
    uint16_t sync = ((uint16_t)bit_buf[9] << 8) | bit_buf[8];
    if (sync == LTC_SYNC_WORD) {
      memcpy((void*)ltc_frame, (void*)bit_buf, 10);
      ltc_ready = true;
    }
    bit_count = 0;
  }
}

// ── ISR: fires on every edge transition ─────────────────────────
void ltc_isr() {
  uint32_t now      = micros();
  uint32_t duration = now - last_edge;
  last_edge         = now;

  if (duration < SHORT_MIN || duration > LONG_MAX) {
    // Noise or too-long gap — reset
    bit_count  = 0;
    half_count = 0;
    return;
  }

  if (duration <= SHORT_MAX) {
    // Short pulse = half a biphase-mark "1" bit
    half_count++;
    if (half_count == 2) {
      half_count = 0;
      push_bit(true);
    }
  } else {
    // Long pulse = full biphase-mark "0" bit
    half_count = 0;
    push_bit(false);
  }
}

// ── Decode helpers ───────────────────────────────────────────────
uint8_t bcd2dec(uint8_t tens, uint8_t units) {
  return tens * 10 + units;
}

void decode_frame(uint8_t* f, int &hh, int &mm, int &ss, int &ff) {
  // SMPTE LTC bit layout (LSB first within each byte):
  // Byte 0: frames units [3:0], user bits [7:4]
  // Byte 1: frames tens  [1:0], drop/color [3:2], user bits [7:4]
  // Byte 2: seconds units[3:0], user bits [7:4]
  // Byte 3: seconds tens [2:0], user bit  [3],   user bits [7:4]
  // Byte 4: minutes units[3:0], user bits [7:4]
  // Byte 5: minutes tens [2:0], user bit  [3],   user bits [7:4]
  // Byte 6: hours units  [3:0], user bits [7:4]
  // Byte 7: hours tens   [1:0], flags     [3:2], user bits [7:4]

  uint8_t fr_u  = f[0] & 0x0F;
  uint8_t fr_t  = f[1] & 0x03;
  uint8_t sec_u = f[2] & 0x0F;
  uint8_t sec_t = f[3] & 0x07;
  uint8_t min_u = f[4] & 0x0F;
  uint8_t min_t = f[5] & 0x07;
  uint8_t hr_u  = f[6] & 0x0F;
  uint8_t hr_t  = f[7] & 0x03;

  ff = bcd2dec(fr_t,  fr_u);
  ss = bcd2dec(sec_t, sec_u);
  mm = bcd2dec(min_t, min_u);
  hh = bcd2dec(hr_t,  hr_u);
}

// ── Setup ────────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  pinMode(LTC_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(LTC_PIN), ltc_isr, CHANGE);
  Serial.println("LTC decoder ready — waiting for signal...");
}

// ── Loop ─────────────────────────────────────────────────────────
void loop() {
  if (ltc_ready) {
    // Safe copy — disable interrupt briefly just for memcpy
    noInterrupts();
    memcpy(frame_snap, (void*)ltc_frame, 10);
    ltc_ready = false;
    interrupts();

    int hh, mm, ss, ff;
    decode_frame(frame_snap, hh, mm, ss, ff);

    // Print as HH:MM:SS:FF
    char buf[16];
    snprintf(buf, sizeof(buf), "%02d:%02d:%02d:%02d", hh, mm, ss, ff);
    Serial.println(buf);
  }

  // Your application code goes here — loop is never blocked
}
