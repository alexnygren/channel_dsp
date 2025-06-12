


#ifndef __MCP4728__
#define __MCP4728__
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <unistd.h>
#include <stdlib.h>
#include "hardware/i2c.h"



typedef struct mcp4728_t {
    i2c_inst_t *i2c;
    uint8_t addr;
    bool restart_mode;
} mcp4728_t;

#define MCP4728_ADDRESS 0b01100000

mcp4728_t *init_mcp4728(i2c_inst_t *i2c, uint8_t addr, bool restart_mode);

void set_dac(mcp4728_t *inst, uint16_t c0, uint16_t c1, uint16_t c2, uint16_t c3, bool log_packet);



#endif

