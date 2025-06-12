/* simple serialization of output bits to stdout for debugging purposes */
#include "bits8.h"
#include <stdio.h>
#include <stdint.h>


void bits8(uint8_t num) {
    char bits[9];
    for (int i = 7; i >= 0; i--) {
        if (((num >> i) & 1) == 1) {
            bits[7-i]='1';
        } else {
            bits[7-i]='0';
        }
    }
    bits[8]=0;
    printf("%s",bits);
}

