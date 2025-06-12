/* PCM3060 Configuration Driver for RP2040
   This uses the 3 wire mode serial control
   A. Nygren 2025-2

   Enables and uses 3-wire SPI control of the PCM3060
   */

#include <unistd.h>
#include <stdint.h>
#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "pcm3060.h"
#include "bits8.h"

void cpause() {
    sleep_us(2);
}

#define HIGH 1
#define LOW 0
#define set_clock(v) gpio_put(PCM3060_MC_PIN,v)
#define data_bit(v) gpio_put(PCM3060_MD_PIN,v)
#define set_write_latch(v) gpio_put(PCM3060_MS_PIN, v)

void write_byte(uint8_t b) {
    for(int i = 7; i >= 0; i--) {
	set_clock(LOW);
	cpause();
	if (((b >> i) & 1) == 1) {
	    data_bit(HIGH);
	} else {
	    data_bit(LOW);
	}
	cpause();
	set_clock(HIGH);
	cpause();
	cpause();
    }
}
// keep the RST pin high, which enables the PCM3060 to work
void init_pcm3060() {
    gpio_init(PCM3060_RESET);
    gpio_set_dir(PCM3060_RESET, GPIO_OUT);
    gpio_pull_up(PCM3060_RESET);
    gpio_put(PCM3060_RESET,HIGH);
}



void setup_serial_to_pcm3060() {

    printf("Setting up PCM3060: GPIOs: Reset: %d  MC: %d  MD: %d  MS: %d\n",PCM3060_RESET, PCM3060_MC_PIN, PCM3060_MD_PIN, PCM3060_MS_PIN);
    
    gpio_init(PCM3060_MC_PIN);
    gpio_set_dir(PCM3060_MC_PIN,GPIO_OUT);
    gpio_pull_down(PCM3060_MC_PIN);
    set_clock(HIGH);
    gpio_init(PCM3060_MS_PIN);
    gpio_set_dir(PCM3060_MS_PIN,GPIO_OUT);
    gpio_pull_down(PCM3060_MS_PIN);
    set_write_latch(HIGH);
    gpio_init(PCM3060_MD_PIN);
    gpio_set_dir(PCM3060_MD_PIN,GPIO_OUT);
    gpio_pull_down(PCM3060_MD_PIN);
    data_bit(LOW);

}

void serial_set_pcm3060(uint8_t reg, uint8_t value, bool log) {

    if (log) {
	printf("Sending: ");
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
    write_byte(reg);
    write_byte(value);

    // set the latch high to store...
    
    set_clock(LOW);
    cpause();
    set_write_latch(HIGH);
    cpause();
    set_clock(HIGH);    
}




