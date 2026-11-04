/* Forward-running 25fps LTC only. BMC edge intervals in microseconds.
 * Frame layout follows SMPTE LTC: LSB-first BCD, sync FC BF, 25fps parity bit59.
 * Require two valid consecutive frames before reporting lock. No heap/FP. */
#include <stdint.h>
#include <stdbool.h>
typedef struct {
    uint8_t bits[10], tc[4]; /* hours, minutes, seconds, frame */
    unsigned count, half, since_sync;
    uint32_t elapsed, previous_time, previous_code, errors;
    bool candidate, locked;
} LtcDecoder;
static void ltc_reset(LtcDecoder *d) {
    d->count = d->half = d->since_sync = 0;
    d->candidate = d->locked = false;
}
static bool ltc_interval(LtcDecoder *d, unsigned dt) {
    d->elapsed += dt;
    unsigned bit;
    if (dt >= 180 && dt <= 320) {
        if (!d->half) { d->half = 1; return false; }
        d->half = 0; bit = 1;
    } else if (dt >= 380 && dt <= 620) {
        if (d->half) { ++d->errors; ltc_reset(d); }
        bit = 0;
    } else { ++d->errors; ltc_reset(d); return false; }
    for (unsigned j = 0; j < 9; ++j)
        d->bits[j] = (d->bits[j] >> 1) | (d->bits[j+1] << 7);
    d->bits[9] = (d->bits[9] >> 1) | (bit << 7);
    if (d->count < 80) ++d->count;
    ++d->since_sync;
    if (d->candidate && d->since_sync > 80) { d->candidate = d->locked = false; }
    if (d->count < 80 || d->bits[8] != 0xFC || d->bits[9] != 0xBF) return false;
    unsigned parity = 0;
    for (unsigned j = 0; j < 10; ++j)
        for (unsigned k = 0; k < 8; ++k) parity ^= (d->bits[j] >> k) & 1;
    unsigned f = (d->bits[0]&15) + 10*(d->bits[1]&3);
    unsigned s = (d->bits[2]&15) + 10*(d->bits[3]&7);
    unsigned m = (d->bits[4]&15) + 10*(d->bits[5]&7);
    unsigned h = (d->bits[6]&15) + 10*(d->bits[7]&3);
    if (parity || (d->bits[1]&4) || (d->bits[0]&15)>9 ||
        (d->bits[2]&15)>9 || (d->bits[4]&15)>9 || (d->bits[6]&15)>9 ||
        f>=25 || s>=60 || m>=60 || h>=24) {
        ++d->errors; ltc_reset(d); return false;
    }
    uint32_t code = ((h*60 + m)*60+s)*25+f;
    uint32_t expected = d->previous_code + 1;
    if (expected == 2160000) expected = 0;
    uint32_t period = d->elapsed - d->previous_time;
    bool ok = d->candidate && d->since_sync == 80 && code == expected && period>=38000 && period<=42000;
    if (d->candidate && !ok) ++d->errors;
    d->previous_time = d->elapsed; d->previous_code = code;
    d->candidate = true; d->locked = ok; d->since_sync = 0;
    d->tc[0]=h; d->tc[1]=m; d->tc[2]=s; d->tc[3]=f;
    return ok;
}
