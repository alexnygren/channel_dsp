/* MCP4728 Driver for RP2040
   A. Nygren 2025-2-15
*/

#include "mcp4728.h"
#include "bits8.h"



mcp4728_t *init_mcp4728(i2c_inst_t *i2c, uint8_t addr, bool restart_mode) {
    mcp4728_t *inst = calloc(1,sizeof(mcp4728_t));
    inst->i2c = i2c;
    inst->addr = addr;
    inst->restart_mode = restart_mode;
    return inst;
}

void swap_bytes(uint16_t word16, uint8_t* packet) {
    uint8_t swapper = 0;
    memcpy(packet,(uint8_t *) &word16,2);
    swapper = packet[0];
    packet[0] = packet[1];
    packet[1] = swapper;
}

void set_dac(mcp4728_t *inst, uint16_t c0, uint16_t c1, uint16_t c2, uint16_t c3, bool log_packet) {

    uint8_t packet[8];
    
    uint16_t timeout = 10000;  // four milliseconds - long timeout, should be 200ms to execute packet run
    size_t avail = 0;
    int sent = 0;
    if (c0 > 4095) {
	c0 = 4095;
    }
    if (c1 > 4095) {
	c1 = 4095;
    }
    if (c2 > 4095) {
	c2 = 4095;
    }
    if (c3 > 4095) {
	c3 = 4095;
    }
    

    swap_bytes(c0, packet);
    swap_bytes(c1, packet+2);
    swap_bytes(c2, packet+4);
    swap_bytes(c3, packet+6);
   
    
    if (log_packet) {
	timeout = 10000;
	printf("\naddress: 0x%x  ",inst->addr);
	bits8(inst->addr);
	printf("\nset channel values: %d %d %d %d\n",c0,c1,c2,c3);
	printf("\ndata packet:\n");
	for (int i = 0; i < 8; i+=2) {
	    printf("%2d: ",i);
	    bits8(packet[i]);
	    printf(" ");
	    bits8(packet[i+1]);
	    printf("  ");
	    printf("%d %d\n",packet[i],packet[i+1]);
	};
	printf("\n");	
    }

    uint64_t start_time = time_us_64();
    avail = i2c_get_write_available(inst->i2c); 
    if (avail > 2) {
	sent = i2c_write_timeout_us(inst->i2c,inst->addr,packet,8,inst->restart_mode,timeout);
	
	if (log_packet && (sent == PICO_ERROR_GENERIC)) {
	    printf("set_dac: NO DEV/ACK\n");
	} else if (sent == PICO_ERROR_TIMEOUT) {
	    printf("set_dac: DT\n");
	} else if ((sent > -1) && (sent != 8)) {
	    printf("set_dac: incorrect packet size: %d\n",sent);
	} 
	
	uint64_t dtime = time_us_64() - start_time;
	if (log_packet) {
	    printf("avail buffer: %d\n",avail); 
	    printf("sent: %d bytes in %llu microseconds.\n",sent,dtime);
	}
	if (dtime > 5000) {
	    printf("set_dac: DELAY: %llu\n",dtime);
	}
    } else {
	if (log_packet) printf("QFull\n");
    }
}
