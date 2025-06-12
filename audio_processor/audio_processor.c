/* Channel Audio Processor
 *
 * audio_processor is free software: you can redistribute it and/or modify it under
 * the terms of the GNU General Public License, version 3 as published by the
 * Free Software Foundation.
 *
 * This software is distributed in the hope that it will
 * be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this code. If not, see <https://www.gnu.org/licenses/>.
 */

/*
 6 is data out
 7 is data in
 8 is bit clock or serial clock 
 9 is LR_CLOCK/word clock
 10 is sys clock or master clock
*/


#include <stdio.h>
#include <string.h>
#include <math.h>
#include <unistd.h>
#include <stdlib.h>

#include "hardware/clocks.h"
#include "hardware/dma.h"
#include "hardware/i2c.h"
#include "hardware/irq.h"
#include "hardware/pio.h"
#include "pico/sync.h"
#include "pico/multicore.h"
#include "i2s.h"
#include "pico/stdlib.h"
#include "pico/float.h"
#include "moving_average.h"
#include "pcm3060.h"
#include "bits8.h"
#include "mcp4728.h"

/* Defines for the specific hardware configuration
 * When DSP_PCB is set, the config is for the manufactured DSP PCB
 * otherwise we are using the dev pcb with external circuitry
 */

#define DSP_PCB REV5

#ifdef DSP_PCB

#define I2C_PORT_I2S i2c1
#define I2C_SDA  6
#define I2C_SCL  7

#define I2C_PORT_STD i2c0
#define I2C_SDA0 4
#define I2C_SCL0 5

#define HEARTBEAT 15

// uart serial coms to controller
#define TX_TO_CONTROLLER 20
#define RX_FROM_CONTROLLER 21
#define MUTE_GPIO 11


#endif


#ifndef DSP_PCB

#define I2C_PORT_I2S i2c1
#define I2C_SDA  6
#define I2C_SCL  7

#define I2C_PORT_STD i2c0
#define I2C_SDA0 4
#define I2C_SCL0 5

// uart serial coms to controller
#define TX_TO_CONTROLLER 24
#define RX_FROM_CONTROLLER 25


#define MUTE_GPIO 22
#define HEARTBEAT 26

#endif

// for initializing the second core as auxillary driver 

#define MULTICORE true
#define CORE1_INIT_FLAG 32767

// how big our command buffer is - must be larger than 10

#define ENTRY_SIZE 500

// define our callback type
typedef void (*callback)();


#define MAX_AMPLITUDE 2147483647

// the common denominator used in sending levels from the dsp to the controller
#define COMMON_RANGE 8192


// command buffer for storing received commands

char *cmd_buffer;
uint64_t pause_count = 100000;

// text buffer for display of text
char *text_buffer = 0;

// send buffer for sending status to controller
char *send_buffer;

// structure for representing the state of the system
struct machine_state_structure {
    bool compressor_on;   // if true, the compression is on, otherwise the compression (cgain and mgain) is bypassed
    bool muted;            // is muting on for the channel?
    float input_trim_gain; // pre compression gain applied to the signal
    float channel_gain;    // post compression gain as applied post compression
    float channel_gain_raw; // non-averaged gain for the channel
    float send1_gain;      // gain for send 1
    float send2_gain;      // gain for send 2
    uint8_t run_mode;      // run_mode is 0 (placeholdeer for flexibility, may not be used
    bool log_activity;     // log activity to the console or not  
    absolute_time_t uptime_milliseconds; // the count of milliseconds since system boot used for tracking millisecond tasks
    uint8_t gate_active;       // whether the gate is part of the signal chain or bypassed (1 = on, 0 = off and bypassed)
    uint8_t gate_open;   // whether the gate is currently open or not
    float gate_threshold_dB; // the threshold at which to open the noise gate
    int32_t gate_threshold_sample; // the threshold at which to open the noise gate
    float gate_reduction_dB;    // the reduction in dB when the noise gate is closed
    uint16_t gate_attack_ms;   // the time to open the gate once the threshold is reached
    uint16_t gate_hold_ms;    // time to keep gate open once the signal falls below the threshold
    uint16_t gate_release_ms;  // time to reach the maximum attenuation of the signal after it falls below the threshold
    float gate_gain;        // the current gain of the gate for reporting purposes, updated every ms 
    int32_t threshold_sample; // the computed sample value of the dB threshold to engage compression
    float threshold_dB;    // calculated for user interaction purposes    
    float compression_gain;  // 1 is unity, 0 is mute (aka cgain)
    uint16_t attack_rate_ms; // how quickly in milliseconds to engage the compressin 
    uint16_t release_rate_ms; // how quickly to release the compression
    uint32_t min_steps;    // when attack is set to 0, the minimum number of cycles (steps) to take to reach the target gain (can't be 0 to avoid div/0)
    int32_t peak_amp_l;      // the last peak amplitude for display purposes (as a sample value)
    int32_t peak_amp_r;    // right side peak
    int32_t current_input_amp_l; // current input amplitude (as a sample value)
    int32_t current_input_amp_r; // right side 
    int32_t output_amp;    // the computed output sample value at the cycle instance
    int32_t output_amp_l;
    int32_t output_amp_r;
    float signal_amp_dB;   //  the current dB level of the compressor input signal
    float peak_amp_dB;     // thecurrent peak input level in dB
    uint32_t output_peak_amp_l; // the last peak amplitude for display purposes (sample value)
    uint32_t output_peak_amp_r; // right side
    float makeup_dB;       // the signal amplitude gain make up in the compressor
    float makeup;          // the ratio for the compression makeup gain (mgain)
    float ratio;           // how strong the compression is to be applied (compression dB reduction/input dB level) post thrshold
    float balance;         // how strong the left and right channels are (-1.0 left unity gain, right mute, 0=both unity, 1=left mute, right unity)
    float output_mix;      // the level of original input signal to compressed signal on output (0 - only input signal, 0.5 - 50% mix, 1 - only compressed signal)
    mcp4728_t* vca_dac;    // the quad dac driver chip for the VCAs
    uint32_t cycle_time_us;
};

struct machine_state_structure machine_state = { .compressor_on = true,\
						 .muted = false,\
						 .input_trim_gain = 1,\
						 .channel_gain = 1.0,\
						 .send1_gain = 0.0,\
						 .send2_gain = 0.0,\
						 .run_mode = 0,\
						 .log_activity = false,\
						 .uptime_milliseconds = 0,\
						 .gate_active = 0,\
						 .gate_open = 0,\
						 .gate_threshold_dB = -72,\
						 .gate_threshold_sample = 0,\
						 .gate_attack_ms = 2,\
						 .gate_hold_ms = 60,\
						 .gate_release_ms = 800,\
						 .gate_gain = 1.0,\
						 .threshold_sample = 0,\
						 .threshold_dB = 0,\
						 .attack_rate_ms = 1,\
						 .release_rate_ms = 225,\
						 .compression_gain = 1.0,\
						 .ratio = 30.0,\
						 .balance = 1.0,\
						 .min_steps = 20,\
						 .peak_amp_l = 0,\
						 .peak_amp_r = 0,\
						 .current_input_amp_l = 0,\
						 .current_input_amp_r = 0,\
						 .output_amp = 0,\
						 .output_peak_amp_l = 0,\
						 .output_peak_amp_r = 0,\
						 .signal_amp_dB = 0,\
						 .peak_amp_dB = 0,\
						 .makeup = 1.0,\
						 .makeup_dB = 0,
						 .output_mix = 1.0,\
						  .vca_dac = 0,\
						 .cycle_time_us = 0 };



static __attribute__((aligned(8))) pio_i2s i2s;


// gain average
average *gain_avg;


float __not_in_flash_func(map_range)(float n, float from_low, float from_high, float to_low, float to_high) {
    return (to_low + (((n- from_low)/ (from_high- from_low))* (to_high- to_low)));
}

// clamp defines - make sure that values are between min and max

#define min(X, Y) ((X) < (Y) ? (X) : (Y))
#define max(X, Y) ((X) > (Y) ? (X) : (Y))
#define clamp(V, X, Y) min(Y,max(X,V))  

// return the signal strength in negative gain from 0dB (pressure)

float signal_dB(int32_t amp) {
    return 0-(20*log10f(MAX_AMPLITUDE/amp));
}

// return the sample value for a provided dB in pressure

int32_t dB_to_sample(float dB) {

    float abs_dB = dB;
    if (dB < 0) {
	abs_dB = abs_dB * -1;
    }
    return (int32_t) (MAX_AMPLITUDE / powf(10.0, (float) (abs_dB / 20.0)));
}

// converts to a decibel relative to 1, where 1 is 0db

float ratio_to_dB(float relative_amplitude) {
    return (20*log10f(relative_amplitude));
}

// converts a ratio where 1 is 0db to a decibel value

float dB_to_ratio(float dB) {

    if (dB >= 0) {
	 return powf(10,(float) (dB / 20));
    }
    return -1.0 * ((float) 0.0 - ( powf(10.0, (float) (dB / 20.0))));
    
}

uint64_t now_ms() {
    return to_ms_since_boot(get_absolute_time());
}


void compute_threshold_for_dB(float dB) {
    
    machine_state.threshold_dB = dB;
    machine_state.threshold_sample = dB_to_sample(dB);
    
}


//static mutex_t mutex;      // control access to the current state

int32_t neg_gain = 1;  // range from below 0 (positive gain) to 65536, 0 is unity gain, 65536 is silence, 
int32_t divs = 0;
int32_t peak_amp_l = 0;
int32_t peak_amp_r = 0;
int32_t peak_amp_output_l = 0;
int32_t peak_amp_output_r = 0;
int32_t neg_peak_amp = 0;

uint32_t call_count = 0;

uint32_t over_limit_time = 0;
uint32_t output_limit = MAX_AMPLITUDE - 1000;
uint32_t output_limit_neg = 0 - (MAX_AMPLITUDE - 1000);

/* These global values represent the various gains to be applied
 * in the current instant within the audio processing function
 * based on the settings in the machine_state structure.
 */

float calculated_net_gain = 1.0;
float balance_l_gain = 1.0;
float balance_r_gain = 1.0;
float comp_output_mix = 1.0;
float raw_output_mix = 0.0;



// channel audio processing function 



static void process_audio(const int32_t* input, int32_t* output, size_t num_frames) {
    int32_t word;
    float wordf;
    float input_word;
    float output_word;
    int32_t abs_word;
    int32_t abs_output_word;
    bool mixed = (machine_state.output_mix < 1.0);
    float trim_gain = machine_state.input_trim_gain;
    
    // copy the input to the output
    for (size_t i = 0; i < num_frames * 2; i++) {
	
	word = input[i];  // get the current word
	wordf = int2float(word);

	// apply any channel level input trim gain to the signal...
	// prior to measurement and processing

	wordf = wordf * trim_gain;

	// collect the original input word for output signal mixing
	input_word = wordf; 
	
	// get the absolute value and see if it greater than current peak
	
	if (word < 0) {
	    abs_word = word * -1;
	} else {
	    abs_word = word;
	}
	
	// assert(abs_word >= 0);

	// get a normalized magnitude of the current word, which can be positive or negative
	wordf = wordf * calculated_net_gain;

	// mix in the original signal if set to..

	if (mixed) {
	    wordf = (comp_output_mix * wordf) + (raw_output_mix * input_word);
	}

	//unfiltered_word = wordf;
	
	// apply channel balance
	// convert to a signed int32 and place in the output buffer
	
	if (i%2 == 0) {
	    if (abs_word > peak_amp_l) peak_amp_l = abs_word;
	    wordf = wordf*balance_l_gain;
	    output_word = float2int(wordf);	    
	    output[i] = output_word;

	    if (output_word < 0) {
		abs_output_word = output_word * -1;
	    } else {
		abs_output_word = output_word;
	    }
	    if (abs_output_word > peak_amp_output_l) {
		peak_amp_output_l = abs_output_word;
	    } 	    
	} else {
	    if (abs_word > peak_amp_r) peak_amp_r = abs_word;
	    
	    wordf = wordf*balance_r_gain;
	    output_word = float2int(wordf);	    	    
	    output[i] = output_word;

	    if (output_word < 0) {
		abs_output_word = output_word * -1;
	    } else {
		abs_output_word = output_word;
	    }
	    if (abs_output_word > peak_amp_output_r) {
		peak_amp_output_r = abs_output_word;
	    }
	}		
    }
    call_count++;
}


static void dma_i2s_in_handler(void) {
    /* We're double buffering using chained TCBs. By checking which buffer the
     * DMA is currently reading from, we can identify which buffer it has just
     * finished reading (the completion of which has triggered this interrupt).
     */
    if (*(int32_t**)dma_hw->ch[i2s.dma_ch_in_ctrl].read_addr == i2s.input_buffer) {
        // It is inputting to the second buffer so we can overwrite the first
        process_audio(i2s.input_buffer, i2s.output_buffer, AUDIO_BUFFER_FRAMES);
    } else {
        // It is currently inputting the first buffer, so we write to the second
        process_audio(&i2s.input_buffer[STEREO_BUFFER_SIZE], &i2s.output_buffer[STEREO_BUFFER_SIZE], AUDIO_BUFFER_FRAMES);
    }
    dma_hw->ints0 = 1u << i2s.dma_ch_in_data;  // clear the IRQ
}


 
/*
Compression processing logic
-- theory of operation ----

target_gain is the current instant computed gain that should be
applied to the signal to conform with the threshold setting.

if it is 1, no compression should be applied
if < 1, compression should follow
 
rgain is the current state of compression applied to the signal.
rgain follows target_gain by being pushed and pulled incrementally
toward target_gain.  It is necessary because if we just adjusted the
signal strength based on the instantaneous target_gain, it would result
in clicks and pops as the signal would be adjusted too quickly and the
transients too large. rgain essentially imposes a slew rate via a
moving average to mitigate and soften the transient changes applied 
to the signal.

Since the compression has an attack and release rate, we need to
reach the target gain over a number of cycles.  Therefore, based on
the current time required to complete a cycle of the loop (cycle_time),
the number of steps to decrement gain (attack) or increase gain (release),
is computed.  Then rgain is adjusted by the computed step value, and
it either decreases the gain (compress), or increments the gain back toward
1.0 (no compression).

Once rgain is computed on the signal, a makeup constant is applied,
which should be considered the post-compression gain.  

*/

void run_compression() {

    
    uint64_t ctime = 0;  // current microsecond 
    uint64_t start_time; // cycle start time 
    
    int32_t local_peak_amp = 1;  // greatest of left or right
    int32_t local_peak_amp_l = 1;
    int32_t local_peak_amp_r = 1;

    int32_t current_input_amp_l = 1;
    int32_t current_input_amp_r = 1;
    int32_t current_input_amp = 1; // greatest of left or right

    int32_t current_output_amp_r = 1;
    int32_t current_output_amp_l = 1;;
    
    int32_t local_output_peak_amp = 1;
    int32_t local_output_peak_amp_l = 1;
    int32_t local_output_peak_amp_r = 1;

    // time tracking for peak indicators 
    uint32_t last_input_peak_time = 0;
    uint32_t last_input_peak_time_l = 0;
    uint32_t last_input_peak_time_r = 0;
    uint32_t last_output_peak_time = 0;
    uint32_t last_output_peak_time_l = 0;
    uint32_t last_output_peak_time_r = 0;
    
    
    float rgain = 1;  // relative gain applied to the signal based on the threshold
    float mgain = 1;
    int32_t delta;
    int32_t threshold;
    uint16_t attack_rate = machine_state.attack_rate_ms;
    uint16_t release_rate = machine_state.release_rate_ms;
    
    float gain_ratio = 1;
    float target_gain = 1; // where we want to be at any given moment based on the input
    float attack_steps = 500;  
    float release_steps = 1000;
    float delta_per_step = 0.001;
    float cycle_time = 0;
    int64_t calculated_output_value;
    float ratio_coef = 1.0;  // 0 - 1, where 1 ~= threshold:1 (aka limiter) and 0 is lighter compression post threshold
    float mute_gain = 1.0; // 1.0 if not muted, muted = 0;
    average *rgain_avg = initialize_average(20);

    average *cycle_time_avg = initialize_average(100);

    bool over_limit = false;
    int32_t over_limit_value = MAX_AMPLITUDE-1000;


    float gate_attack_steps = 500;
    float gate_release_steps = 10000;

    float mute_gain_step = 0.01; // so it doesn't snap or click going on and off
    
    float gate_gain = 0;
    uint32_t gate_below_threshold_time = 0;
    int32_t gate_threshold = machine_state.gate_threshold_sample;
    bool gate_below_threshold = true;
    
    printf("starting dsp loop\n");
    
    threshold = machine_state.threshold_sample;

    while(1) {
	// time accounting for rate calculations
	start_time = time_us_64();
	machine_state.uptime_milliseconds = to_ms_since_boot(get_absolute_time());
	
	// collect the value of the input peak amplitude at the current moment
	current_input_amp_l = peak_amp_l;
	current_input_amp_r = peak_amp_r;

	current_input_amp = max(current_input_amp_l,current_input_amp_r); // take the greater of left or right
	if (current_input_amp > local_peak_amp) {
	    local_peak_amp = current_input_amp;
	    last_input_peak_time = 0;
	}
	if (current_input_amp_l > local_peak_amp_l) {	    
	    local_peak_amp_l = current_input_amp_l;
	    last_input_peak_time_l = 0;
	}
	if (current_input_amp_r > local_peak_amp_r) {
	    local_peak_amp_r = current_input_amp_r;
	    last_input_peak_time_r = 0;
	}
	// collect current output situation
        
	if (peak_amp_output_l > local_output_peak_amp_l) {
	    local_output_peak_amp_l=peak_amp_output_l;
	    last_output_peak_time_l = 0;
	}
	if (peak_amp_output_r > local_output_peak_amp_r) {
	    local_output_peak_amp_r=peak_amp_output_r;
	    last_output_peak_time_r = 0;
	}

	// record for the machine state
	machine_state.peak_amp_l = local_peak_amp_l;
	machine_state.peak_amp_r = local_peak_amp_r;
	machine_state.output_peak_amp_l = local_output_peak_amp_l;
	machine_state.output_peak_amp_r = local_output_peak_amp_r;
	machine_state.current_input_amp_l = current_input_amp_l;
	machine_state.current_input_amp_r = current_input_amp_r;
	
	// is the gate active
	if (machine_state.gate_active) {
	    if ((current_input_amp_l >= gate_threshold)||(current_input_amp_r >= gate_threshold)) {
		gate_below_threshold_time = 0; // reset our timer for when we fall below
		gate_gain = max(0, min(1.0, gate_gain + (1 / gate_attack_steps)));
		gate_below_threshold = false;
		machine_state.gate_open = 1;
	    } else if ((current_input_amp_l < gate_threshold)&&(current_input_amp_r < gate_threshold)) {
		gate_below_threshold = true;
		// only decrement once we are passed the hold time		
		if ((gate_below_threshold_time > machine_state.gate_hold_ms) && (gate_gain > 0)) {
		    machine_state.gate_open = 0;
		    gate_gain = max(0, min(1.0, gate_gain - (1 / gate_release_steps)));		    
		}
	    }
	    // and apply the current gain..experiment: allow compressor to still be active
	    // while the gate is closed, so when it opens it may already be where it needs
	    // to be...
	    // current_input_amp = (int32_t) floorf((float) current_input_amp * gate_gain);
	} else {
	    gate_gain = 1;
	}
      
	if (machine_state.compressor_on) {

	    // calculate the current steps nee
	
	    // determine if the current amplitude is greater than our threshold
	    delta = current_input_amp - threshold;

	    // do the compression gain calculations...
	    // if gain_ratio is positive, the signal amplitude is past the threshold
	    // otherwise if it is negative, the amplitude is below the threshold	    
	    
	    gain_ratio = (float) ((float) current_input_amp / (float) threshold);
	    
	    // ratio will be handled by adding a coefficient to the log addition
	    if (gain_ratio > 1.0) gain_ratio = gain_ratio + (ratio_coef * log10f(gain_ratio));
	    // if gain ratio is below 1, then rgain is below 1
	    if (gain_ratio < 1.0) gain_ratio = 1;

	    // target gain is the reciprocal of the current gain multiple
	    // target is the instantaneous target value to adjust gain to
	    target_gain = 1.0 / gain_ratio;

	    target_gain = clamp(target_gain,0.0,1.0);

	    // apply a slew rate using a stepped approach and a moving average
	    // to smooth the transient changes applied to the final compression
	    // gain...
	    
	    if (delta > 0) {
		//  it's past our current threshold, adjust gain accordingly...
		// depending on our attack rate	    
		delta_per_step = (rgain - target_gain) / attack_steps;
		/*if (delta_per_step <= 0.0001) {
		    delta_per_step = 0.0001;
		    }*/
	    } else {	    
		// we're below so implement the release steps..
		// rgain should be below target_gain, so we have a negative step delta..
		delta_per_step = (rgain - target_gain) / release_steps;
		if (delta_per_step >= -0.0001) {
		    delta_per_step = -0.0001;
		} 
	    } 
	    //if (rgain < 1.0) {
	    // adjust rgain based on the timing and the difference to target_gain
	    rgain = rgain - delta_per_step;
	    //}
	    // this is the compressor gain portion, which we don't want higher than 1
	    if (rgain > 1.0) {
		rgain = 1.0;
	    } else if (rgain < 0.0) {
		rgain = 0;
	    }

	    // smooth rgain out so it is not sharply adjusted...
	    
	    rgain = (float) moving_average(rgain_avg, rgain, false);
	    machine_state.compression_gain = rgain;

	    // makeup applies a gain for the compressed value 
	    // channel_gain provides the final output gain applied to the channel
	    
	    // mgain = rgain * machine_state.makeup * machine_state.channel_gain * gate_gain;
	    
	    // do final gain with message to VCA DACS so don't include channel_gain here
	    // this is because there are 3 different output gain settings, channel, send 1 and
	    // send 2.
	    mgain = rgain * machine_state.makeup * gate_gain;
	    	
	  
	    // register our computed output amplitude
	    calculated_output_value = (int32_t) floorf(mgain * (float) current_input_amp);
	    if (calculated_output_value > over_limit_value) {
		over_limit_time = 0;
		calculated_output_value = over_limit_value;
		mgain = 0.9;
	    } else if (calculated_output_value < -over_limit_value) {
		over_limit_time = 0;
		calculated_output_value = -over_limit_value;
		mgain = 0.9;
	    }
	    // is mute on?  If so apply and send to the processing loop  
	    // set the overall negative gain for the processing loop by setting calculated_net_gain
	    calculated_net_gain = mgain * mute_gain;  //
	    machine_state.output_amp = (int32_t) calculated_output_value;
	    if (machine_state.output_amp > local_output_peak_amp) {	       
		local_output_peak_amp = machine_state.output_amp;
		last_output_peak_time = 0;	       
		//		if (machine_state.output_peak_amp > 2000000) printf("\n%ld %f %ld %ld\n",machine_state.output_peak_amp, mgain, delta, neg_gain);
	    }	    	    
	    
	} else {
	    
	    // compressor off so just pass on the value as received
	    // only modulated by the current channel_gain value
	    machine_state.compression_gain = 1.0;
	    calculated_net_gain = mute_gain * machine_state.channel_gain * gate_gain;  // TODO: BUG - are we calculating channel gain twice due to VCA
	    mgain = 1.0;
	    rgain = 1.0;
	    machine_state.output_amp = (int32_t) (machine_state.channel_gain * (float) current_input_amp);
	    if (machine_state.output_amp > local_output_peak_amp) {
		local_output_peak_amp = machine_state.output_amp;
		last_output_peak_time = 0;  // reset our counter...
	    }
	}
	// check if it is a new millisecond 
	if (ctime < machine_state.uptime_milliseconds) {

	    // Do the db calculations 
	    machine_state.signal_amp_dB = signal_dB(current_input_amp); 
	    machine_state.peak_amp_dB = signal_dB(local_peak_amp);
	    	  
	    // update any settings
	    threshold = machine_state.threshold_sample; // update threshold_sample if needed	    
	    attack_rate = machine_state.attack_rate_ms;		
	    
	    if (attack_rate == 0) attack_steps = machine_state.min_steps;  // avoid divide by 0 and enough to avoid chop
	    else attack_steps = round((attack_rate*1000)/cycle_time);
	    
	    release_rate = machine_state.release_rate_ms;	    
	    if (release_rate == 0) release_steps = machine_state.min_steps;	    
	    else release_steps = round((release_rate*1000)/cycle_time);

	    gate_threshold = machine_state.gate_threshold_sample;
	    gate_attack_steps = round((machine_state.gate_attack_ms*1000)/cycle_time);
	    gate_release_steps = round((machine_state.gate_release_ms*1000)/cycle_time);
	    machine_state.gate_gain = gate_gain;
	    if (gate_below_threshold) {
		gate_below_threshold_time++;  // increment our milisecond counter;
	    }

	    if (machine_state.muted > 0) {
		mute_gain = max(0.0, mute_gain-mute_gain_step);		
	    } else {
	        mute_gain = min(1.0, mute_gain+mute_gain_step);	      
	    }
	    
	    if (machine_state.log_activity && machine_state.uptime_milliseconds % 200 == 0) {
		printf("M%d C%d G%d%d  %5.2fdB  IN[L %5.2fdB/%5.2fdB] [R %5.2fdB/%5.2fdB]   OUT[L %5.2fdB/%4.2fdB] [R %5.2fdB/%4.2fdB]      \r",
		       machine_state.muted,
		       machine_state.compressor_on,
		       machine_state.gate_active,
		       machine_state.gate_open,
		       signal_dB((int32_t) roundf(machine_state.compression_gain*(float)MAX_AMPLITUDE)),
		       signal_dB((int32_t) machine_state.current_input_amp_l),
		       signal_dB((int32_t) machine_state.peak_amp_l),
		       signal_dB((int32_t) machine_state.current_input_amp_r),
		       signal_dB((int32_t) machine_state.peak_amp_r),
		       signal_dB((int32_t) machine_state.output_amp_l),
		       signal_dB((int32_t) machine_state.output_peak_amp_l),
		       signal_dB((int32_t) machine_state.output_amp_r),
		       signal_dB((int32_t) machine_state.output_peak_amp_r)); 
		
		       
 		/*printf("%d  %4.2fdB/%4.2fdB   %3ldus  %.3f/%4.2fdB  [%.0f %.0f] NGG=%1.3f  %6.0f GR=%2.3f TG=%2.3f RG=%2.3f MG=%2.3f DPS=%1.5f     \r", \
			   machine_state.compressor_on,\
			   machine_state.peak_amp_dB,\
			   machine_state.signal_amp_dB,\
			   machine_state.cycle_time_us,\
			   machine_state.compression_gain,\
			   signal_dB((int32_t) roundf(machine_state.compression_gain*(float)MAX_AMPLITUDE)),\
			   attack_steps,\
			   release_steps,\
		           gate_gain,\
			   floor(machine_state.uptime_milliseconds/1000),\
			   gain_ratio,\
			   target_gain,\
			   rgain,\
		           mgain,\
		           delta_per_step); */
	    }
	    last_input_peak_time++;
	    last_output_peak_time++;
	    last_input_peak_time_l++;
	    last_input_peak_time_r++;
	    last_output_peak_time_l++;
	    last_output_peak_time_r++;
	    over_limit_time++;
	    if (machine_state.muted == 0) {
		if (over_limit && over_limit_time > 750) {
		    gpio_put(MUTE_GPIO,0);
		    over_limit = false;
		}
		if (over_limit_time < 750) {
		    gpio_put(MUTE_GPIO,1);
		    over_limit = true;
		}
	    }
	}
	// handle the fall back of the peak amplitude, normalized to cycle time
	// only fall back after the delay period...
	if (last_input_peak_time > 750) {
	    local_peak_amp-=(1000 * machine_state.cycle_time_us);
	    if (local_peak_amp < 0) {
		local_peak_amp = 0;	    
	    }
	}
	if (last_input_peak_time_l > 750) {
	    local_peak_amp_l-=(1000 * machine_state.cycle_time_us);	    
	    if (local_peak_amp_l < 0) {
		local_peak_amp_l = 0;
	    }
	    
	}
	if (last_input_peak_time_r > 750) {
	    local_peak_amp_r-=(1000 * machine_state.cycle_time_us);
	    if (local_peak_amp_r < 0) {
		local_peak_amp_r = 0;
	    }	    
	}
	
	if (last_output_peak_time > 750) {	    
	    local_output_peak_amp-=(1000 * machine_state.cycle_time_us);
	    if (local_output_peak_amp < 0) {
		local_output_peak_amp = 0;	   
		
	    }
	}
	if (last_output_peak_time_l > 750) {	    
	    local_output_peak_amp_l-=(1000 * machine_state.cycle_time_us);
	    if (local_output_peak_amp_l < 0) {
		local_output_peak_amp_r = 0;	   		
	    }
	}
	if (last_output_peak_time_r > 750) {	    
	    local_output_peak_amp_r-=(1000 * machine_state.cycle_time_us);
	    if (local_output_peak_amp_r < 0) {
		local_output_peak_amp_r = 0;	   		
	    }
	}
	
	// let the global machine structure know the current peak once it has been validated

	machine_state.output_peak_amp_l = local_peak_amp_l;
	machine_state.output_peak_amp_r = local_peak_amp_r;
	
	// rapidly diminish the current input amplitude
	current_input_amp-=(15000*machine_state.cycle_time_us);
	if (current_input_amp < 0) {
	    current_input_amp = 0;
	}
	current_input_amp_l-=(15000*machine_state.cycle_time_us);
	if (current_input_amp_l < 0) {
	    current_input_amp_l = 0;
	}
	current_input_amp_r-=(15000*machine_state.cycle_time_us);
	if (current_input_amp_r < 0) {
	    current_input_amp_r = 0;
	}
	current_output_amp_l-=(15000*machine_state.cycle_time_us);
	if (current_output_amp_l < 0) {
	    current_output_amp_l = 0;
	}
	current_output_amp_r-=(15000*machine_state.cycle_time_us);
	if (current_output_amp_r < 0) {
	    current_output_amp_r = 0;
	}
	
	// let the audio processing loop know the current setting for peak_amp
	peak_amp_l = current_input_amp_l;
	peak_amp_r = current_input_amp_r;

	peak_amp_output_l = current_output_amp_l;
	peak_amp_output_r = current_output_amp_r;
	
	// time accounting
	ctime = machine_state.uptime_milliseconds;
	cycle_time = moving_average(cycle_time_avg, (float) (time_us_64() - start_time),false);
	machine_state.cycle_time_us = cycle_time;
	
    }
}

// send the current activity state of the audio channel
void send_activity() {
        
    if (uart_is_writable(uart1)) {
	int input_amp_l = round(map_range((float) machine_state.current_input_amp_l,0,MAX_AMPLITUDE,0,COMMON_RANGE));
	int input_amp_r = round(map_range((float) machine_state.current_input_amp_r,0,MAX_AMPLITUDE,0,COMMON_RANGE));
		
	int peak_amp_l = round(map_range((float) machine_state.peak_amp_l,0,MAX_AMPLITUDE,0,COMMON_RANGE));
	int peak_amp_r = round(map_range((float) machine_state.peak_amp_r,0,MAX_AMPLITUDE,0,COMMON_RANGE));

	int output_amp_l = round(map_range((float) machine_state.output_amp_l,0,MAX_AMPLITUDE,0,COMMON_RANGE));
	int output_amp_r = round(map_range((float) machine_state.output_amp_r,0,MAX_AMPLITUDE,0,COMMON_RANGE));

	int output_peak_amp_l = round(map_range((float) machine_state.output_peak_amp_l,0,MAX_AMPLITUDE,0,COMMON_RANGE));
	int output_peak_amp_r = round(map_range((float) machine_state.output_peak_amp_r,0,MAX_AMPLITUDE,0,COMMON_RANGE));
	
	int cgain = round(map_range((float) machine_state.compression_gain,0,1,0,65535));

	
	sprintf(send_buffer,"A%d %d %d %d %d %d %d %d %d %d %d\n",input_amp_l,input_amp_r,peak_amp_l, peak_amp_r,output_amp_l,output_amp_r,output_peak_amp_l,output_peak_amp_r,cgain, machine_state.muted, machine_state.gate_open);
	uart_puts(uart1,send_buffer);
    }
}

void send_status() {
    if (uart_is_writable(uart1)) {
	sprintf(send_buffer,"C%d %d %d %f %f %f %f %f %f %f %f %f\n",machine_state.compressor_on, machine_state.attack_rate_ms, machine_state.release_rate_ms, machine_state.threshold_dB, machine_state.makeup_dB, machine_state.ratio, machine_state.output_mix, machine_state.input_trim_gain, machine_state.balance, machine_state.channel_gain, machine_state.send1_gain, machine_state.send2_gain);
	uart_puts(uart1,send_buffer);
    }
}


void set_logging(bool on) {
    machine_state.log_activity = on;
}

void clear_entry(char *buf) {
  for (unsigned int i = 0; i < ENTRY_SIZE; i++) {
    buf[i] = 0;
  }
}

void calc_output_mix (float mix) {
    comp_output_mix = min(1.0,max(0,mix));
    raw_output_mix = min(1.0,max(0,1 - mix));
}


uint16_t calc_gain(float g) {
    uint16_t out = clamp((uint16_t) (g*4096.0),0,4095);
    return out;
}



void set_channel_gain() {
    
    if (machine_state.channel_gain != machine_state.channel_gain_raw) {
	set_dac(machine_state.vca_dac,
		calc_gain(machine_state.channel_gain),
		calc_gain(machine_state.send1_gain),
		calc_gain(machine_state.send2_gain),
		calc_gain(machine_state.channel_gain),
		false);  // logging true
    }
}



void send_bus_command(char *buf) {

    char *target;
    char *reg;
    char *value;
    char *nil = 0;
    target = strtok(buf," ");
    reg = strtok(nil, " ");
    value = strtok(nil, " ");

    printf("Target: %s  register: %s  value: %s\n",target,reg,value);
    serial_set_pcm3060(atoi(reg),atoi(value),true);

}


void heartbeat() {
    
    static uint64_t last_check = 0;
    static uint8_t heartbeat_state = 0;
    
    if (machine_state.uptime_milliseconds>(last_check+20)) {
	last_check = machine_state.uptime_milliseconds;	
	switch(heartbeat_state) {
	case 0:
	    gpio_put(HEARTBEAT,1);
	    break;
	case 2:
	    gpio_put(HEARTBEAT,0);
	    break;
	case 8:
	    gpio_put(HEARTBEAT,1);
	    break;
	case 9:
	    gpio_put(HEARTBEAT,0);
	    break;
	}
	heartbeat_state++;
	heartbeat_state = heartbeat_state%70;
    }
}

// transpond with the controller and the console

char handle_command_io() {
    // check for controller command...

    if (uart_is_readable(uart1)) {
	return uart_getc(uart1);
    }
    heartbeat();
    machine_state.channel_gain = moving_average(gain_avg,machine_state.channel_gain_raw,false);
    set_channel_gain();
    static uint64_t last_check = 0;
    if (machine_state.uptime_milliseconds>(last_check+49)) {	
	send_activity();	
	last_check = machine_state.uptime_milliseconds;
    }
    // check stdio...
    int rval = 0;    
    rval = stdio_getchar_timeout_us(10000);
    if (rval == PICO_ERROR_TIMEOUT) {
	return 0;
    }
    return (char) rval;
}

// listen for control events on the UARTS

char *command_listener() {
  char dur_buffer[20];
  unsigned int pos = 0;
  char log_state = machine_state.log_activity;
  char c;
  clear_entry(cmd_buffer);
  while(1) {
    // proto_poll(); // check for commands from the control processor
      c = handle_command_io();
      if (c==0) continue;
      /*int rval = stdio_getchar_timeout_us(50000);
    if (rval ==PICO_ERROR_TIMEOUT) {
	send_state();
	continue;
    } else {
	c = rval;	
	}*/
    if ((machine_state.log_activity == true) && (pos == 0)) {
        set_logging(0);
	printf("\n%c",c);
    } else {
	printf("%c",c);
    } 
    switch (c) {
      case '\r':
      case '\n':
	if (pos > 0) {
	  dur_buffer[pos]=0;
	  memcpy(cmd_buffer, dur_buffer,pos);
	  set_logging(log_state);
	  return cmd_buffer;
	} else {
          set_logging(log_state);
	  return cmd_buffer;  // will be blank
	}
	break;
    case 27:  // escape - cancel command
	clear_entry(cmd_buffer);
	set_logging(log_state);
	return cmd_buffer; 
    default:
      dur_buffer[pos] = c;
      pos++;
      if (pos>=20) {
	printf("\ninvalid entry.\n");
	clear_entry(cmd_buffer);
	set_logging(log_state);
	return cmd_buffer;
      }
    }
  }
  printf("\nERROR: fall through in read_text\n");
  set_logging(log_state);
  return cmd_buffer;
}
void show_help() {
    printf("The command entry format is: <COMMAND LETTER>[ARGUMENTS]\n");
    printf("where command letter is one of: \n");
    printf("  s - show current settings.\n");
    printf("compression:\n");
    printf("  a - set attack time in milliseconds\n");
    printf("  r - set release time in milliseconds\n");
    printf("  t - set threshold in dB\n");
    printf("  m - set make up ratio\n");
    printf("  R - set ratio [1 to 30] dB compression for every 1dB of signal\n");
    printf("  O - output mix [0 - no compressed signal to 1 - only compressed]\n");
    printf("  c - set compression on/off\n");
    printf("noise gate:\n");
    printf("  G - gate on/off\n");
    printf("  N - set gate threshold dB\n");
    printf("  A - set attack time in milliseconds\n");
    printf("  E - set release time in milliseconds\n");
    printf("  H - set gate hold time in milliseconds\n");
    printf("channel:\n");
    printf("  b - set channel stereo balance [-1.0 to 1.0]\n");
    printf("  g - set channel gain (applied post compression) [0.0 to 2.0]\n");
    printf("  1 - set send 1 gain (applied post compression) [0.0 to 1.0]\n");
    printf("  2 - set send 2 gain (applied post compression) [0.0 to 1.0]\n");
    printf("  T - set channel input gain (trim)\n");
    printf("  M - set channel muted [0 - not muted, 1 - muted]\n\n");
    printf("  L - set low pass ratio [0 - no filter, 1 - fully filtered\n");
    printf("  h - set high pass ratio [0 - no filter, 1 - fully filtered\n");
    printf("  l - log current state to the console on/off\n");
    printf("  S - set minimum permissible cycle steps for slew\n");    
    printf("  C - clear the screen.\n");
    printf("  ? - print this menu.\n\n");
    printf("examples: set attack to 2ms:      a2\n");
    printf("          set threshold to -18dB: t18   or:  t-18\n");
    printf("          set a 2.5:1 make up:    m2.5\n\n");
}

char *on_off(uint32_t value) {
    if (value == 0) return "off";
    else return "on";
}

void output_settings() {
    printf("Channel Settings\n");
    printf("gain:              %1.5f  %4.3fdB\n",machine_state.channel_gain,signal_dB((int32_t) roundf(machine_state.channel_gain * (float) MAX_AMPLITUDE)));    
    printf("muted:              %s\n",on_off(machine_state.muted));
    printf("compression:        %s\n",on_off(machine_state.compressor_on));
    printf("balance:            %2.4f\n",machine_state.balance);
    printf("\nCompressor\n");
    printf("attack:      %3dms  release:    %3dms\n",machine_state.attack_rate_ms, machine_state.release_rate_ms);
    printf("threshold:   %fdB   makeup:     %.3fdB\n",machine_state.threshold_dB, machine_state.makeup_dB);
    printf("ratio:       %2.3f:1\n",machine_state.ratio);
    printf("output mix:  %.0f compressed\n",roundf(comp_output_mix*100));

    printf("\n");
}

void handle_command(char cmd, char* args) {
    float f; 
    int32_t i;
    switch(cmd) {
    case 'a':
	i = clamp(atoi(args),0,500);
	machine_state.attack_rate_ms = i;
	printf("attack rate = %dms\n",machine_state.attack_rate_ms);
	break;
    case 'r':
	i = clamp(atoi(args),0,500);
	machine_state.release_rate_ms = i;
	printf("release rate = %dms\n",machine_state.release_rate_ms);
	break;
    case 't':
	f = atof(args);
	if (f < 0) {
	    f = f * -1;
	}
	f = min(64,f);	
	compute_threshold_for_dB(0-f);
	printf("threshold dB = %3.2fdB\n",machine_state.threshold_dB);
	printf("threshold sample = %ld\n",machine_state.threshold_sample);
	break;
    case 'c':
	i = clamp(atoi(args),0,1);
	machine_state.compressor_on = (uint8_t) i;
	if (machine_state.compressor_on) {
	    printf("compressor is now on.\n");
	} else {
	    printf("compressor is now off.\n");
	}
	break;
    case 'b':
	machine_state.balance = max(-1,min(1,atof(args)));
	printf("balance is: %2.4f\n",machine_state.balance);
	balance_l_gain = 1.0 + machine_state.balance;
	balance_r_gain = 1.0 - machine_state.balance;
	break;
    case 'R':
	f = max(min(30, atof(args)),0);
	machine_state.ratio = f;
	printf("ratio = %2.2f:1\n",machine_state.ratio);
	break;
    case 'l':
	if (machine_state.log_activity) {
	    machine_state.log_activity = false;
	    printf("logging is off\n");
	} else {
	    machine_state.log_activity = true;
	    printf("logging is on\n");
	}
	break;
    case 'S':
	i = clamp(atoi(args),1,1000);
	machine_state.min_steps = i;
	printf("minimum transition steps = %ld\n",machine_state.min_steps);
	break;
    case 'm':
	f = clamp(atof(args),0,24);
	machine_state.makeup_dB = f;
	machine_state.makeup = dB_to_ratio(machine_state.makeup_dB);
	printf("makeup_dB = %.3f  ratio=%.3f\n",machine_state.makeup_dB,machine_state.makeup);
	break;
    case 'C':
	printf("\033[2J");
	break;
    case 'g':
	f = clamp(atof(args),0.0,1.0);
	machine_state.channel_gain_raw = f;
	printf("g=%1.3f\n",machine_state.channel_gain_raw);
	set_channel_gain();
	break;
    case '1':
	f = clamp(atof(args),0.0,1.0);
	machine_state.send1_gain = f;
	set_channel_gain();
	printf("send 1 gain = %1.3f\n",machine_state.send1_gain);
	break;
    case '2':
	f = clamp(atof(args),0.0,1.0);
	machine_state.send2_gain = f;
	set_channel_gain();
	printf("send 2 gain = %1.3f\n",machine_state.send2_gain);
	break;
    case '3':
	i = clamp(atoi(args),0,100000);
	float orig = machine_state.channel_gain;
	machine_state.channel_gain = 0;
	set_channel_gain();
	float step_diff = (float) 1.0/(float)i;
	for(int step = 0; step < i; step++) {
	    machine_state.channel_gain+=step_diff;
	    set_channel_gain();
	}
	printf("...pausing..\n");
	sleep_us(100000);
	printf("decrementing...\n");
	for(int step = 0; step < i; step++) {
	    machine_state.channel_gain-=step_diff;
	    set_channel_gain();
	}
	machine_state.channel_gain = orig;
	set_channel_gain();
	break;
    case 'G':
	i = clamp(atoi(args),0,1);
	machine_state.gate_active = (uint8_t) i;
	if (machine_state.gate_active) {
	    printf("noise gate is now on.\n");
	} else {
	    printf("noise gate is now off.\n");
	}
	break;
    case 'A':
	i = clamp(atoi(args),0,3000);
	machine_state.gate_attack_ms = i;
	printf("noise gate attack time = %dms\n",machine_state.gate_attack_ms);
	break;
    case 'E':
	i = clamp(atoi(args),0,3000);
	machine_state.gate_release_ms = i;
	printf("noise gate release time = %dms\n",machine_state.gate_release_ms);
	break;
    case 'N':

	f = clamp(atof(args),-100,100);
	if (f < 0) {
	    f = f * -1;
	}
	
	machine_state.gate_threshold_dB = f;
	machine_state.gate_threshold_sample = dB_to_sample(f);
	printf("noise gate threshold arg = %2.2f  dB = %2.2f  sample = %lu\n",f, machine_state.gate_threshold_dB, machine_state.gate_threshold_sample);
	break;
    case 'H':
	i = clamp(atoi(args),0,3000);
	machine_state.gate_hold_ms = i;
	printf("noise gate hold time = %dms\n",machine_state.gate_hold_ms);
	break;
    case 's':
	send_status();
	output_settings();	
	break;
    case 'O':
	f = clamp(atof(args),0.0,1.0);
	machine_state.output_mix = f;
	// do the precomputation here for the processor loop
	calc_output_mix(f);
	printf("compressed mix: %2.3f   raw mix: %2.3f\n", comp_output_mix, raw_output_mix);
	break;
    case 'M':
	i = clamp(atoi(args),0,1);
	machine_state.muted = i;
	if (i == 0) {
	    gpio_put(MUTE_GPIO,0);
	} else {
	    gpio_put(MUTE_GPIO,1);
	}
	break;
    case 'T':
	f = clamp(atof(args),0.1,3);
	machine_state.input_trim_gain = f;
	break;
    case 'W':
	i = clamp(atoi(args),0,255);
	serial_set_pcm3060(0x40,i,true);	
	break;
    case '#':	
	send_bus_command(args);
	break;
    case 'B':
	i = atoi(args);
	printf("0x%lx   ",i);
	bits8(i);
	printf("\n");
	break;
    case '?':
	show_help();
	break;
    default:	
	uart_puts(uart1,"Eunknown command\n");
	printf("unknown command: %c\nType ? for command menu.\n",cmd);
    }
}

void control_loop() {
    char cmd;
    char *args;
    int cmd_len;
    char *termcodes = (char *)calloc(50,sizeof(char));
    char *rows = 0;
    char *cols = 0;
    uint8_t idx = 0;
    uint64_t last_second = 0;
    
    printf("\033[s\033[999;999H\033[6n\033[u");
    while(1) {
	termcodes[idx] = stdio_getchar_timeout_us(1000000);
	if (termcodes[idx] == PICO_ERROR_TIMEOUT) {
	    termcodes[idx] = 0;
	    break;
	} else if (termcodes[idx]=='R') {
	    break;
	}
	idx++;
	if (idx >= 20) {
	    break;
	}
	if (last_second != round(machine_state.uptime_milliseconds/1000)) {
	    last_second = round(machine_state.uptime_milliseconds/1000);
	    if ((last_second%2)==0) {
		gpio_put(HEARTBEAT,1);
	    } else {
		gpio_put(HEARTBEAT,1);
	    }	    
	}
	
    }
    if (idx > 2) {
	rows = termcodes+2;
	for(uint8_t i = 2; i < idx; i++) {
	    if (termcodes[i] == ';') {
		termcodes[i] = 0;
		cols = termcodes+i+1;
	    } else if (termcodes[i] == 'R') {
		*(termcodes+i) = 0;
	    }
	}
	if (rows && cols) {
	    printf("term: cols x rows: %d x %d\n",atoi(cols),atoi(rows));
	} else {
	    printf("term: didn't get terminal information.\n");
	}
	printf("\n");
    } 	
    printf("\n");
    printf("\nEnter ? for help.\n");
    while(1) {
	if (machine_state.run_mode == 0) {
	    printf("\n-> "); // print a prompt to the UART0
	}
	cmd_buffer = command_listener();
	cmd_len = strlen(cmd_buffer);
	if (cmd_len > 0) {
	    cmd = cmd_buffer[0];
	    if (cmd_len > 1) args = cmd_buffer+1;
	    else args = "";
	    printf("-> [%c] [%s]\n",cmd,args);
	    handle_command(cmd,args);
	}
    }
}


void core1_init() {

    multicore_fifo_push_blocking(CORE1_INIT_FLAG);
    uint32_t resp = multicore_fifo_pop_blocking();
    stdio_init_all();
    cmd_buffer = (char *) calloc(ENTRY_SIZE,sizeof(char));
    if (resp != CORE1_INIT_FLAG) {
	printf("\nERROR: core 1 didn't receive the flag value from core 0 for initializing sequence.");
    } else {
	i2c_init(I2C_PORT_STD, 80 * 1000);	
	gpio_set_function(I2C_SDA0, GPIO_FUNC_I2C);
	gpio_set_function(I2C_SCL0, GPIO_FUNC_I2C);
	gpio_pull_up(I2C_SDA0);
	gpio_pull_up(I2C_SCL0);

	gain_avg = initialize_average(20);

	gpio_init(HEARTBEAT);
	gpio_set_dir(HEARTBEAT, GPIO_OUT);
	gpio_pull_down(HEARTBEAT);

	
	gpio_set_slew_rate(I2C_SDA0, GPIO_SLEW_RATE_FAST);
	gpio_set_slew_rate(I2C_SCL0, GPIO_SLEW_RATE_FAST);

	
	// initialize uart
	
	gpio_set_function(TX_TO_CONTROLLER, UART_FUNCSEL_NUM(uart1, TX_TO_CONTROLLER));
	gpio_set_function(RX_FROM_CONTROLLER, UART_FUNCSEL_NUM(uart1, RX_FROM_CONTROLLER));
	uart_init(uart1,115200);
	
	setup_serial_to_pcm3060();

	// wait for initialization
	sleep_us(100000);
	// setup the pcm3060..
	// 129 = mode control register reset + single ended output
	serial_set_pcm3060(0x40, 129,true);
	
	// 127 = reset, but keep differential output
	//serial_set_pcm3060(0x40,0b10000000,true);
	
	// set the DAC to use the clock to the ADC
	serial_set_pcm3060(0x43, 0b10000000,true); // 128 - 0b10000000
	
	// setup the pcm1780..
	// register 20 (0x14), with b100 (4) for i2s format0
	//serial_set_pcm1780(20, 4, true);
	
	 
	machine_state.vca_dac = init_mcp4728(I2C_PORT_STD,MCP4728_ADDRESS,false);
	set_channel_gain();
	
	control_loop();
       
    }
}


int setup() {

    text_buffer = (char *) calloc(1024,sizeof(char));
    send_buffer = (char *) calloc(ENTRY_SIZE,sizeof(char));
    
    
    // I2C Initialisation. Using it at 100Khz.
    i2c_init(I2C_PORT_I2S, 100 * 1000);

    gpio_set_function(I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL, GPIO_FUNC_I2C);
    gpio_set_pulls(I2C_SDA, true, false);
    gpio_set_pulls(I2C_SCL, true, false);

    
    gpio_set_drive_strength(I2C_SDA, GPIO_DRIVE_STRENGTH_12MA);
    gpio_set_drive_strength(I2C_SCL, GPIO_DRIVE_STRENGTH_12MA);
    gpio_set_slew_rate(I2C_SDA, GPIO_SLEW_RATE_FAST);
    gpio_set_slew_rate(I2C_SCL, GPIO_SLEW_RATE_FAST);

    // bring up the PCM3060 into the enabled state..
    init_pcm3060();
    
    
    gpio_init(MUTE_GPIO);
    gpio_set_dir(MUTE_GPIO,GPIO_OUT);
    gpio_pull_down(MUTE_GPIO);

    gpio_put(MUTE_GPIO,1);
    sleep_ms(250);
    gpio_put(MUTE_GPIO,0);



    
    // Note: it is usually best to configure the codec here, and then enable it
    //       after starting the I2S clocks, below.
    // default threshold is -12dB
    compute_threshold_for_dB(-12);
    
    if (MULTICORE) {
	multicore_launch_core1(core1_init);
	uint32_t resp = multicore_fifo_pop_blocking();
	if (resp != CORE1_INIT_FLAG) {
	    printf("\nERROR: core 0 didn't receive the initialized flag from core 1\n");
	    return 1;
	} else {
	    printf("core1 is active.\n");
	}
	multicore_fifo_push_blocking(CORE1_INIT_FLAG);
    }
    printf("initializing i2s...\n");
    // main channel and clocks on pio0
    i2s_program_start_synched(pio0, &i2s_config_default, dma_i2s_in_handler, &i2s);
    // sends on pio1
    //    i2s_program_start_sends(pio1, &i2s_config_default, dma_i2s_send_handler, &i2s);
    return 0;
}

int main() {
    // Set a 132.000 MHz system clock to more evenly divide the audio frequencies
    set_sys_clock_khz(132000, true);
    stdio_init_all();
    printf("\n\nAudio Channel DSP\n");
    printf("System Clock: %lu\n", clock_get_hz(clk_sys));

    // set up the environment...
    setup();
    // and run the main loop
    run_compression();
    
    return 0;
}
