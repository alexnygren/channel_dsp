/* ADS1115 ADC driver
 * adapted from https://github.com/gavinlyonsrepo/ADS1x15_PICO/tree/main
 * MIT License
 *
 */



#ifndef __ADS1115__
#define __ADS1115__

#include <unistd.h>
#include <stdint.h>
#include "hardware/i2c.h"
#include "hardware/irq.h"



// chip instance structure

typedef struct ADS1115 {
    uint8_t  addr;
    uint16_t timeout_us;
    i2c_inst_t *i2c;
} ADS1115;

// instatiation
ADS1115 *initialize_ads1115(i2c_inst_t *i2c_bus, uint8_t address, uint16_t timeout_us);

// control functions

int write_adc_register(ADS1115 *adc, uint8_t reg, uint16_t value);
int16_t read_adc_register(ADS1115 *adc, uint8_t reg);
void start_adc_reading(ADS1115 *adc, uint8_t config_mode);


#endif
