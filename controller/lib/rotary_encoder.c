
#include <unistd.h>
#include <stdlib.h>
#include <stdio.h>
#include "rotary_encoder.h"
#include "hardware/gpio.h"
#include "hardware/pio.h"
#include "hardware/irq.h"
#include "hardware/timer.h"
#include "rotary.pio.h"


rotary_encoder* init_rotary_pio(PIO pio, uint8_t rotary_clock_pin, callback on_up, callback on_down) {
    
    rotary_encoder *re = calloc(1,sizeof(rotary_encoder));

    re->pio = pio;
    re->offset = pio_add_program(re->pio, &rotary_program);
    re->sm = pio_claim_unused_sm(re->pio, true);

    printf("re->offset: %d   re->sm=%d\n",re->offset,re->sm);
    
    //Enable irq interrupts for up and down actions.
    pio_set_irq0_source_enabled(re->pio, pis_interrupt0, true);
    irq_add_shared_handler(PIO1_IRQ_0, on_up, 0);
    irq_set_enabled(PIO1_IRQ_0, true);
    pio_set_irq1_source_enabled(re->pio, pis_interrupt1, true);
    irq_add_shared_handler(PIO1_IRQ_1, on_down, 0);
    irq_set_enabled(PIO1_IRQ_1, true);

    rotary_program_init(re->pio, re->sm, re->offset, rotary_clock_pin);
    printf("re: enabled pio on gpio: %d\n",rotary_clock_pin);
    return re;
}
