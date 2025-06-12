/* PCM1780 Configuration Driver for RP2040
   This uses the 3 wire mode serial control
   A. Nygren 2025-2
*/
// commands for 3-wire control of the pcm1780

#include <unistd.h>
#include <stdint.h>
#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "pcm1780.h"
#include "bits8.h"

void cpause_1780() {
    sleep_us(2);
}

#define HIGH 1
#define LOW 0
#define set_clock(v) gpio_put(PCM1780_MC_PIN,v)
#define data_bit(v) gpio_put(PCM1780_MD_PIN,v)
#define set_write_latch(v) gpio_put(PCM1780_MS_PIN, v)

void write_byte_1780(uint8_t b) {
    for(int i = 7; i >= 0; i--) {
	set_clock(LOW);
	cpause_1780();
	if (((b >> i) & 1) == 1) {
	    data_bit(HIGH);
	} else {
	    data_bit(LOW);
	}
	cpause_1780();
	set_clock(HIGH);
	cpause_1780();
	cpause_1780();
    }
}


void setup_serial_to_pcm1780() {
    gpio_init(PCM1780_MC_PIN);
    gpio_set_dir(PCM1780_MC_PIN,GPIO_OUT);
    gpio_pull_down(PCM1780_MC_PIN);
    set_clock(HIGH);
    gpio_init(PCM1780_MS_PIN);
    gpio_set_dir(PCM1780_MS_PIN,GPIO_OUT);
    gpio_pull_down(PCM1780_MS_PIN);
    set_write_latch(HIGH);
    gpio_init(PCM1780_MD_PIN);
    gpio_set_dir(PCM1780_MD_PIN,GPIO_OUT);
    gpio_pull_down(PCM1780_MD_PIN);
    data_bit(LOW);

}

void serial_set_pcm1780(uint8_t reg, uint8_t value, bool log) {

    if (log) {
	printf("pcm1780: sending: ");
	bits8(reg);
	printf(" ");
	bits8(value);
	printf("\n");
    }
    
    // start the clock and set it low...
    set_clock(LOW);
    // set the write latch to low...
    set_write_latch(LOW);

    // write our register address and value...
    write_byte_1780(reg);
    write_byte_1780(value);

    // set the latch high to store...
    
    set_clock(LOW);
    cpause_1780();
    set_write_latch(HIGH);
    cpause_1780();
    set_clock(HIGH);    
}




