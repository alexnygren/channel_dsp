// Driver for WS2812B lights

#include <stdio.h>
#include <stdlib.h>
#include "pico/stdlib.h"
#include <math.h>
#include "hardware/clocks.h"
#include "hardware/pio.h"
#include "ws2812_driver.h"
#include "ws2812_driver.pio.h"


void put_pixel(ws2812b *inst, uint32_t pixel_grb) {
    pio_sm_put_blocking(inst->pio, inst->sm, pixel_grb << 8u);
}

uint32_t urgb_u32(uint8_t r, uint8_t g, uint8_t b) {
    return
            ((uint32_t) (r) << 8) |
            ((uint32_t) (g) << 16) |
            (uint32_t) (b);
}



ws2812b *init_ws2812b(PIO pio_inst, uint8_t gpio_pin, uint8_t num_pixels) {

    ws2812b *inst = malloc(sizeof(ws2812b));
    inst->pio = pio_inst;
    inst->sm = pio_claim_unused_sm(inst->pio, true);
    inst->offset = pio_add_program(inst->pio, &ws2812_driver_program);
    inst->gpio_pin = gpio_pin;
    inst->num_pixels = num_pixels;
    //pio_inst->sm[inst->sm].clkdiv = (uint32_t) (33 * (1 << 16));
    ws2812_driver_program_init(pio_inst, inst->sm, inst->offset, inst->gpio_pin,800000,false);
    printf("ws2812b: offset=%d sm=%d  pin=%d\n",inst->offset,inst->sm,inst->gpio_pin);
    return inst;

}



