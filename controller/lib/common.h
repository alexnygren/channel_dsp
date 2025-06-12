#pragma once

/* Machine API
   Implements the common types and functions used to represent
   and control the hardware implementation
 */

#include <stdint.h>
#include <stdbool.h>
#include "hardware/clocks.h"
#include "pico/stdlib.h"
#include "pico/float.h"
#include "ssd1306_i2c_driver.h"

// 24 BIT signed max amplitude (as absolute)
#define MAX_AMPLITUDE 2147483647

// the number of neo-pixels on the controller board
#define AMPLITUDE_PIXEL_COUNT 28  // 20 for the dev card


// limits
#define COMP_LOW_TH_LIMIT -40
#define COMP_HIGH_TH_LIMIT 0

#define GATE_LOW_TH_LIMIT -99
#define GATE_HIGH_TH_LIMIT -3

// clamp defines - make sure that values are between min and max

#define min(X, Y) ((X) < (Y) ? (X) : (Y))
#define max(X, Y) ((X) > (Y) ? (X) : (Y))
#define clamp(V, X, Y) min(Y,max(X,V))  



// define our callback type
typedef void (*callback)();

typedef void (*callback1)(uint16_t);

typedef void (*callback1f)(float);

// various utilities and conversions


float signal_dB(int32_t amp);
int32_t dB_to_sample(float dB);

float ratio_to_dB(float relative_amplitude);
float dB_to_ratio(float dB);

float __not_in_flash_func(map_range)(float n, float from_low, float from_high, float to_low, float to_high);
				    

// for different LED bar graph color schemes

typedef struct colorscheme {
    uint8_t *amplitude;
    uint8_t *peak_amp;
    uint8_t *overload;
} colorscheme;


// structure for representing the state of the system
struct machine_state_structure {
    bool ready;           // ready is false during initial startup, then is turned true and the control loops entered
    bool compressor_on;   // if true, the compression is on, otherwise the compression (cgain and mgain) is bypassed
    uint8_t compressor_on_pin; // which GPIO pin to set high to show compression is on (for LED)
    bool muted;            // is muting on for the channel?
    float input_trim_gain; // pre compression gain applied to the signal
    float channel_gain;    // post compression gain as applied post compression
    float send1_gain;      // gain for send 1
    float send2_gain;      // gain for send 2
    uint8_t run_mode;      // run_mode is 0 (placeholdeer for flexibility, may not be used
    bool log_activity;     // log activity to the console or not  
    absolute_time_t uptime_milliseconds; // the count of milliseconds since system boot used for tracking millisecond tasks
    uint8_t display_on;    // whether to draw information to the display 
    uint8_t display_addr;  // the i2c address of the SSD1306 display
    uint8_t display_level; 
    uint8_t gate_active;       // whether the gate is part of the signal chain or bypassed (1 = on, 0 = off and bypassed)
    uint8_t gate_open;   // whether the gate is currently open or not
    float gate_threshold_dB; // the threshold at which to open the noise gate
    int32_t gate_threshold_sample; // the threshold at which to open the noise gate
    float gate_reduction_dB;    // the reduction in dB when the noise gate is closed
    float gate_attack_ms;   // the time to open the gate once the threshold is reached
    float gate_hold_ms;    // time to keep gate open once the signal falls below the threshold
    float gate_release_ms;  // time to reach the maximum attenuation of the signal after it falls below the threshold
    float gate_gain;        // the current gain of the gate for reporting purposes, updated every ms 
    int32_t threshold_sample; // the computed sample value of the dB threshold to engage compression
    float threshold_dB;    // calculated for user interaction purposes    
    float compression_gain;  // 1 is unity, 0 is mute (aka cgain)
    float attack_rate_ms; // how quickly in milliseconds to engage the compressin 
    float release_rate_ms; // how quickly to release the compression
    uint32_t min_steps;    // when attack is set to 0, the minimum number of cycles (steps) to take to reach the target gain (can't be 0 to avoid div/0)
    float peak_amp_l;      // the last peak amplitude for display purposes (as a sample value)
    float peak_amp_r;
    float current_input_amp_l; // current input amplitude left as a 1-255 (reporting purposes only) (not in dB)
    float current_input_amp_r; // right side 
    float output_amp_l;    // the computed output value at the cycle instance 1-255 (not in dB)
    float output_amp_r;    // right side
    float output_peak_amp_l;
    float output_peak_amp_r;
    float signal_amp_dB;   //  the current dB level of the compressor input signal
    float peak_amp_dB;     // thecurrent peak input level in dB
    float makeup_db;          // the multiplier for the compression makeup gain (mgain)
    float ratio;           // how strong the compression is to be applied (compression dB reduction/input dB level) post thrshold
    float balance;         // how strong the left and right channels are (-1.0 left unity gain, right mute, 0=both unity, 1=left mute, right unity)
    float output_mix;      // the level of original input signal to compressed signal on output (0 - only input signal, 0.5 - 50% mix, 1 - only compressed signal)
    bool slider_adc_initialized; // true once the slider adc has been initialized..
    float slider_percent;  // the current adc reading of the slider from 0 to 1.x where 1.0 is calibrated to 0db on the meter - this is the linear position
    float slider_db; // the decibel value of the slider    
    float slider_target_position; // if the system gain is changed, adjust the slider to this target.  This should ultimately match with slider_percent.
    bool slider_fault_mode; // if the slider can't reach it's goal, this is set to high and should be reset prior to further use.
    uint64_t core_1_cycle_time_us; // the time through the main core 1 processing loop
    uint64_t core_0_cycle_time_us; // the time through the main core 1 processing loop
    uint64_t display_refresh_time_us; // the microseconds it takes to refresh the display
    SSD_i2c_display *display; // pointer to the display structure instance
    uint32_t *button_state; // state of the panel buttons
    uint8_t num_buttons;    // number of buttons
    char *channel_name;    // the name of the channel
    uint8_t channel_number;  // the assigned number of the channel
    uint64_t slider_update_ms;
    uint64_t last_external_update_ms; // the ms since boot when the structure was updated from a remote source, such as the DSP, or the system controller
};

typedef struct machine_state_structure machine;


// define the global instance which will be referenced
// across the subsystems

void set_machine_state(machine *machine_state);

extern machine *current_state;

char *str_alloc(int len);

void request_channel_status();

void set_logging(bool on);

uint64_t now_ms();

const char *on_off(uint32_t value);





