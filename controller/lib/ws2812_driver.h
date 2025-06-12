


#ifndef __WS2812B_DRIVER__
#define __WS2812B_DRIVER__


typedef struct ws2812b {
    PIO pio;
    uint sm;
    uint offset;
    uint8_t gpio_pin;
    uint8_t num_pixels;
} ws2812b;

ws2812b *init_ws2812b(PIO pio_inst, uint8_t gpio_pin,uint8_t num_pixels);

uint32_t urgb_u32(uint8_t r, uint8_t g, uint8_t b);

void put_pixel(ws2812b *inst, uint32_t pixel_grb);


#endif
