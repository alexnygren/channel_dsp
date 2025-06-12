

#ifndef __ROTARY__
#define __ROTARY__

#include <ctype.h>
#include "hardware/pio.h"

typedef void (*callback)();


typedef struct rotary_encoder {
    uint sm;
    PIO pio;
    uint offset;
} rotary_encoder;



rotary_encoder* init_rotary_pio(PIO pio, uint8_t rotary_clock_pin, callback on_up, callback on_down);

#endif

