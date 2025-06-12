
/* Machine API
   Implements the API for controlling the hardware and provides
   common types
*/


#include <stdio.h>
#include <string.h>
#include <math.h>
#include <unistd.h>
#include <stdlib.h>
#include <stdbool.h>
#include "hardware/clocks.h"
#include "hardware/dma.h"
#include "hardware/i2c.h"
#include "hardware/irq.h"
#include "hardware/pio.h"
#include "hardware/adc.h"
#include "hardware/flash.h"
#include "pico/sync.h"
#include "pico/multicore.h"
#include "pico/stdlib.h"
#include "pico/float.h"

#include "common.h"

machine *current_state;

float __not_in_flash_func(map_range)(float n, float from_low, float from_high, float to_low, float to_high) {
    return (to_low + (((n- from_low)/ (from_high- from_low))* (to_high- to_low)));
}

void set_machine_state(machine *machine_state) {
    current_state = machine_state;
}

struct machine_state_structure *get_machine_state() {
    return current_state;
}

// shorthand helper
char *str_alloc(int len) {
    return (char *) calloc(len,sizeof(char));
}


// return the signal strength in negative gain from 0dB (pressure)

float signal_dB(int32_t amp) {
    return 0-(20*log10f(MAX_AMPLITUDE/amp));
}

// converts to a decibel relative to 1, where 1 is 0db

float ratio_to_dB(float relative_amplitude) {
    return (20*log10f(relative_amplitude));
}

// return the sample value for a provided dB in pressure

int32_t dB_to_sample(float dB) {

    float abs_dB = dB;
    if (dB < 0) {
	abs_dB = abs_dB * -1;
    }
    return (int32_t) (MAX_AMPLITUDE / powf(10.0, (float) (abs_dB / 20.0)));
}



float dB_to_ratio(float dB) {

    if (dB >= 0) {
	 return powf(10,(float) (dB / 20));
    }
    return -1.0 * ((float) 0.0 - ( powf(10.0, (float) (dB / 20.0))));
    
}

uint64_t now_ms() {
    return to_ms_since_boot(get_absolute_time());
}

void request_channel_status() {
    char buf[20];
    if (uart_is_writable(uart1)) {
	sprintf(buf,"s\n");
	uart_puts(uart1,buf);
    }
}

const char *on_off(uint32_t value) {
    if (value==0) return "OFF";
    return "ON";
		   
}

void set_logging(bool on) {
    current_state->log_activity = on;
}




