/* PCM1780 Configuration Driver for RP2040
   This uses the 3 wire mode serial control
   A. Nygren 2025-2
*/
// commands for 3-wire control of the pcm3060

#ifndef __PCM1780__
#define __PCM1780__

// define the GPIO PINs for the protocol

#ifndef PCM1780_MS_PIN
#define PCM1780_MS_PIN 21
#endif
#ifndef PCM1780_MC_PIN
#define PCM1780_MC_PIN 20
#endif
#ifndef PCM1780_MD_PIN
#define PCM1780_MD_PIN 2
#endif

// setup the GPIO pins
void setup_serial_to_pcm1780();

// send an encoded command to the PCM1780
void serial_set_pcm1780(uint8_t reg, uint8_t value, bool log);


#endif


