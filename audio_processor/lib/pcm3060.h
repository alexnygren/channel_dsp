/* PCM3060 Configuration Driver for RP2040
   This uses the 3 wire mode serial control
   A. Nygren 2025-2
*/
// commands for 3-wire control of the pcm3060

#ifndef __PCM3060__
#define __PCM3060__

// define the GPIO PINs for the protocol
// 11 20 2
#ifndef PCM3060_MS_PIN
#define PCM3060_MS_PIN 13
#endif
#ifndef PCM3060_MC_PIN
#define PCM3060_MC_PIN 12
#endif
#ifndef PCM3060_MD_PIN
#define PCM3060_MD_PIN 14
#endif
#ifndef PCM3060_RESET
#define PCM3060_RESET 17
#endif

// enable the pcm3060 chip.  This must be called prior to the others
void init_pcm3060();

// setup the GPIO pins
void setup_serial_to_pcm3060();

// send an encoded command to the PCM3060
void serial_set_pcm3060(uint8_t reg, uint8_t value, bool log);



#endif


