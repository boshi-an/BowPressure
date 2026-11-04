#include <stdio.h>
#include "../src/ltc_decode.h"
int main(void) {
    LtcDecoder d = {0};
    unsigned dt;
    while (scanf("%u", &dt) == 1) {
        if (ltc_interval(&d, dt))
            printf("%02u:%02u:%02u:%02u\n", d.tc[0], d.tc[1], d.tc[2], d.tc[3]);
    }
    return 0;
}
