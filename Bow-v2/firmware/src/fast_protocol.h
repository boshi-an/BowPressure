/* Fixed 32-byte packets, all multibyte fields little endian.
 * A5 5A, version=2, type, sequence u32, read_us u32, payload[18], CRC16.
 * IMU payload: six high16 little-endian words, timestamp u16,
 * three bytes with accel low4 in bits7:4 and gyro low4 in bits3:0, reserved.
 * CRC16-CCITT-FALSE over bytes 0..29; stored little endian. */
#include <stdint.h>
/* Nibble lookup: same CRC16-CCITT-FALSE, less CPU than 8 bit loops/byte. */
static uint16_t fast_crc(const uint8_t *p, unsigned n) {
    static const uint16_t table[16] = {
        0x0000,0x1021,0x2042,0x3063,0x4084,0x50A5,0x60C6,0x70E7,
        0x8108,0x9129,0xA14A,0xB16B,0xC18C,0xD1AD,0xE1CE,0xF1EF
    };
    uint16_t crc = 0xFFFF;
    for (unsigned i = 0; i < n; ++i) {
        crc ^= (uint16_t)p[i] << 8;
        crc = (uint16_t)(crc << 4) ^ table[crc >> 12];
        crc = (uint16_t)(crc << 4) ^ table[crc >> 12];
    }
    return crc;
}
static void pack16(uint8_t *p, uint16_t v) { p[0] = v; p[1] = v >> 8; }
static void pack32(uint8_t *p, uint32_t v) {
    for (unsigned i = 0; i < 4; ++i) p[i] = v >> (8 * i);
}
