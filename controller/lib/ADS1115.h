#ifndef __ADS1115__
#define __ADS1115__
// Register Pointers
uint16_t REG_POINTER_MASK = 0x03;			/**< Register pointer mask */
uint16_t REG_POINTER_CONVERT = 0x00;		/**< Conversion register */
uint16_t REG_POINTER_CONFIG = 0x01;		/**< Configuration register */
uint16_t REG_POINTER_LOWTHRESH = 0x02; /**< Low threshold register */
uint16_t REG_POINTER_HITHRESH = 0x03;	/**< High threshold register */

uint16_t REG_CONFIG_OS_MASK = 0x8000;		/**< OS Mask */
uint16_t REG_CONFIG_OS_SINGLE = 0x8000;	/**< Write: Set to start a single conversion */
uint16_t REG_CONFIG_OS_BUSY = 0x0000;		/**< R/W: Bit=0 when conversion is in progress */
uint16_t REG_CONFIG_OS_NOTBUSY = 0x8000; /**< R/W: Bit=1 when device is not performing a conversion */
uint16_t REG_CONFIG_MUX_MASK = 0x7000;		/**< Mux Mask */
// Register Configs
uint16_t REG_CONFIG_PGA_MASK = 0x0E00;		/**< PGA Mask */
uint16_t REG_CONFIG_PGA_6_144V = 0x0000; /**< +/-6.144V range = Gain 2/3 */
uint16_t REG_CONFIG_PGA_4_096V = 0x0200; /**< +/-4.096V range = Gain 1 */
uint16_t REG_CONFIG_PGA_2_048V = 0x0400; /**< +/-2.048V range = Gain 2 (default) */
uint16_t REG_CONFIG_PGA_1_024V = 0x0600; /**< +/-1.024V range = Gain 4 */
uint16_t REG_CONFIG_PGA_0_512V = 0x0800; /**< +/-0.512V range = Gain 8 */
uint16_t REG_CONFIG_PGA_0_256V = 0x0A00; /**< +/-0.256V range = Gain 16 */
uint16_t REG_CONFIG_MODE_MASK = 0x0100;	 /**< Mode Mask */
uint16_t REG_CONFIG_MODE_CONTIN = 0x0000; /**< Continuous conversion mode */
uint16_t REG_CONFIG_MODE_SINGLE = 0x0100; /**< Power-down single-shot mode (default) */

uint16_t REG_CONFIG_RATE_MASK = 0x00E0;		/**< Data Rate Mask */
uint16_t REG_CONFIG_CMODE_MASK = 0x0010;		/**< CMode Mask */
uint16_t REG_CONFIG_CMODE_TRAD = 0x0000;		/**< Traditional comparator with hysteresis (default) */
uint16_t REG_CONFIG_CMODE_WINDOW = 0x0010; /**< Window comparator */
uint16_t REG_CONFIG_CPOL_MASK = 0x0008;		/**< CPol Mask */
uint16_t REG_CONFIG_CPOL_ACTVLOW = 0x0000; /**< ALERT/RDY pin is low when active (default) */
uint16_t REG_CONFIG_CPOL_ACTVHI = 0x0008;	/**< ALERT/RDY pin is high when active */

uint16_t REG_CONFIG_CLAT_MASK = 0x0004;	 /**< Determines if ALERT/RDY pin latches once asserted */
uint16_t REG_CONFIG_CLAT_NONLAT = 0x0000; /**< Non-latching comparator (default) */
uint16_t REG_CONFIG_CLAT_LATCH = 0x0004;	 /**< Latching comparator */

uint16_t REG_CONFIG_CQUE_MASK = 0x0003;	/**< CQue Mask */
uint16_t REG_CONFIG_CQUE_1CONV = 0x0000; /**< Assert ALERT/RDY after one conversion */
uint16_t REG_CONFIG_CQUE_2CONV = 0x0001; /**< Assert ALERT/RDY after two conversions */
uint16_t REG_CONFIG_CQUE_4CONV = 0x0002; /**< Assert ALERT/RDY after four conversions */
uint16_t REG_CONFIG_CQUE_NONE = 0x0003;	/**< Disable the comparator and put ALERT/RDY in high state (default) */

// Data Rates for ADS1015
uint16_t RATE_ADS1015_128SPS = 0x0000;	 /**< 128 samples per second */
uint16_t RATE_ADS1015_250SPS = 0x0020;	 /**< 250 samples per second */
uint16_t RATE_ADS1015_490SPS = 0x0040;	 /**< 490 samples per second */
uint16_t RATE_ADS1015_920SPS = 0x0060;	 /**< 920 samples per second */
uint16_t RATE_ADS1015_1600SPS = 0x0080; /**< 1600 samples per second (default) */
uint16_t RATE_ADS1015_2400SPS = 0x00A0; /**< 2400 samples per second */
uint16_t RATE_ADS1015_3300SPS = 0x00C0; /**< 3300 samples per second */

// Data Rates for ADS1115
uint16_t RATE_ADS1115_8SPS = 0x0000;		/**< 8 samples per second */
uint16_t RATE_ADS1115_16SPS = 0x0020;	/**< 16 samples per second */
uint16_t RATE_ADS1115_32SPS = 0x0040;	/**< 32 samples per second */
uint16_t RATE_ADS1115_64SPS = 0x0060;	/**< 64 samples per second */
uint16_t RATE_ADS1115_128SPS = 0x0080; /**< 128 samples per second (default) */
uint16_t RATE_ADS1115_250SPS = 0x00A0; /**< 250 samples per second */
uint16_t RATE_ADS1115_475SPS = 0x00C0; /**< 475 samples per second */
uint16_t RATE_ADS1115_860SPS = 0x00E0; /**< 860 samples per second */


#endif
