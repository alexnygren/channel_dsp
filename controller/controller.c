/* Channel Controller
 *
 * channel_controller is free software: you can redistribute it and/or modify it under
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



#define SSD1306_WIDTH 128
#define SSD1306_HEIGHT 64

#include <stdio.h>
#include <string.h>
#include <math.h>
#include <unistd.h>
#include <stdlib.h>
#include <stdbool.h>
#include "hardware/clocks.h"
#include "hardware/dma.h"
#include "hardware/i2c.h"
#include "hardware/irq.h"
#include "hardware/pio.h"
#include "hardware/adc.h"
#include "hardware/flash.h"
#include "pico/sync.h"
#include "pico/multicore.h"
#include "pico/stdlib.h"
#include "pico/float.h"
#include "common.h"
#include "ui.h"
#include "moving_average.h"
#include "bits8.h"

#include "ads1115.h"

// rotary encoder
#include "rotary_encoder.h"




// ws2812b neo-pixel driver
#include "ws2812_driver.h"

// I2C defines
// This system uses I2C1 on GPIO6 (SDA) and GPIO7 (SCL) running at 100KHz.
// Connect the codec I2C control to this. 


/* Hardware GPIO Mappings --------------------
 */

#define I2C_SDA0 4
#define I2C_SCL0 5

#define I2C_SDA1 26
#define I2C_SCL1 27

#define ROTARY_CLK_PIN 6
#define ROTARY_DT_PIN 7

// uart serial coms to audio_processor
#define TX_TO_DSP 8
#define RX_FROM_DSP 9

// button definitions
#define NUM_BUTTONS 11
#define SWITCH_I 13
#define SWITCH_J 14
#define SWITCH_R 3

#define MUX_SEL_A 29
#define MUX_SEL_B 28
#define MUX_READ_A 10
#define MUX_READ_B 25



#define SLIDER_UP 2
#define SLIDER_DOWN 11

#define LED_GPIO 12

#define RESET_DSP 24 // in schematic it is 16

// WS2812 Addressable LED GPIO
#define ADDR_LED_GPIO 20

// color decay rate
#define LED_DECAY_RATE 0.9
#define LED_AVG_SIZE 20

// color modifiers
#define IN_COMP_HIGH 8
#define IN_COMP_LOW 4

// signals when adc is ready


// for initializing the second core as auxillary driver 

#define MULTICORE true
#define CORE1_INIT_FLAG 32767

// how big our command buffer is - must be larger than 10

#define ENTRY_SIZE 1024


// the display pointer

SSD_i2c_display *display;


// display updates is the position in the linked list that
// new updates are appended to.  display_pos is the position
// in the list of the transfer function to the display.  Once
// an area in the list has been transfered to the display, the
// area is free'd and the transfer function moves to the next
// element until next is nil.




// the common denominator used in receiving levels from the dsp
#define COMMON_RANGE 8192;

// two 32-bit arrays to use as modulators of the LED brightness

// the current level of the leds 
float *led_levels;
// the current level of the compressor, when mapped to the LEDS
float *comp_gain;

// the current average  of the leds
average **avg_led_rgb;

// and a driver instance...
ws2812b *amplitude_inst = 0;

colorscheme default_colors;


// define an ADS1115 instance for the slider ADC
ADS1115 *ads;

float slider_divisor = 5588.0;
float gain_range = 1.185;
int16_t raw_val;

int16_t slider_position;
int16_t slider_position_locked;

// lookup structure for delta movements
typedef struct delta_to_ms {
    float delta;
    uint8_t ms;
    struct delta_to_ms *next;
    struct delta_to_ms *prior;
} delta_to_ms;



delta_to_ms *delta_list_start = 0;  // capture the start

volatile bool adc_ready = false;
#define ADC_CONFIG_REG 0x01
#define ADC_CONVERT_REG 0x00
average *adc_avg;

uint32_t *buttons;

// rotary encoder instance
rotary_encoder *re;



// DSP receive header
void check_dsp();

// command buffer for storing received commands...

char *cmd_buffer;

// and from the DSP card on uart1.
char *recv_buffer;

// text buffer for display of text
char *text_buffer = 0;

typedef struct dsp_xmit_queue {
    char *message;
    struct dsp_xmit_queue *next;
    struct dsp_xmit_queue *prior;
} dsp_xmit_queue;

dsp_xmit_queue *dsp_queue;

struct machine_state_structure machine_state = { .ready = false,      
		      .compressor_on = true,
		      .compressor_on_pin = 0,
		      .muted = false,
		      .input_trim_gain = 1,
		      .channel_gain = 1.0,
		      .send1_gain = 0.0,
		      .send2_gain = 0.0,
		      .run_mode = 0,
		      .log_activity = false,
		      .uptime_milliseconds = 0,
		      .display_on = true,
		      .display_addr = SSD1306_I2C_ADDR_DEFAULT,
		      .display_level = 1,
		      .gate_active = 0,
		      .gate_open = 0,
		      .gate_threshold_dB = -72,
		      .gate_threshold_sample = 0,
		      .gate_attack_ms = 2,
		      .gate_hold_ms = 60,
		      .gate_release_ms = 800,
		      .gate_gain = 1.0,
		      .threshold_sample = 0,
		      .threshold_dB = 0,
		      .attack_rate_ms = 1,
		      .release_rate_ms = 225,
		      .compression_gain = 1.0,
		      .ratio = 30.0,
		      .balance = 1.0,
		      .min_steps = 20,
		      .peak_amp_l = 0,
		      .peak_amp_r = 0,
		      .current_input_amp_l = 0,
		      .current_input_amp_r = 0,
		      .output_amp_l = 0,
		      .output_amp_r = 0,
		      .output_peak_amp_l = 0,
		      .output_peak_amp_r = 0,
		      .signal_amp_dB = 0,
		      .peak_amp_dB = 0,
		      .makeup_db = 1.0,
		      .output_mix = 1.0,
		      .slider_percent = 0.0,
		      .slider_db = 0.0,
		      .slider_adc_initialized = false,
		      .display_refresh_time_us = 0,
		      .display = 0,
		      .num_buttons = NUM_BUTTONS,
		      .button_state = 0,
		      .core_1_cycle_time_us = 0,
		      .channel_name = "Channel",
		      .channel_number = 0,
		      .last_external_update_ms = 0,
		      .slider_update_ms = 0 };


void mark_state_updated() {
    current_state->last_external_update_ms = to_ms_since_boot(get_absolute_time());
}

void mark_slider_state_updated() {
    current_state->slider_update_ms = to_ms_since_boot(get_absolute_time());
}

void process_led_levels() {

    uint8_t total_amp_l = (uint8_t) map_range(current_state->current_input_amp_l,0,1,0,AMPLITUDE_PIXEL_COUNT/2);
    uint8_t total_amp_r = (uint8_t) map_range(current_state->current_input_amp_r,0,1,0,AMPLITUDE_PIXEL_COUNT/2);
    
    uint8_t peak_amp_l = (uint8_t) map_range(current_state->peak_amp_l,0,1,0,AMPLITUDE_PIXEL_COUNT/2);
    uint8_t peak_amp_r = (uint8_t) map_range(current_state->peak_amp_r,0,1,0,AMPLITUDE_PIXEL_COUNT/2);

    uint8_t no_amplitude_color = 0;  // this is assigned to the blue element of the pixel

    uint8_t compressor_level = (uint8_t) map_range(current_state->compression_gain,0,1,0,(AMPLITUDE_PIXEL_COUNT/2)+1);
    
    uint8_t gate_closed = ((current_state->gate_active) && (!current_state->gate_open));
    uint8_t gate_pixel = AMPLITUDE_PIXEL_COUNT/2;
    uint8_t in_comp = 0;
    uint8_t in_comp_l = 0;
    if (current_state->compressor_on) {
	no_amplitude_color = 2;  // amplitude to assign a deep background blue when the compressor is on...	
    } else {
	no_amplitude_color = 0;
    }
    if (total_amp_l > AMPLITUDE_PIXEL_COUNT/2) {
	total_amp_l = AMPLITUDE_PIXEL_COUNT/2;
    }
    
    // go through the array and adjust values;
    // left channel 
    for(uint8_t i = 1; i <= AMPLITUDE_PIXEL_COUNT/2; i++) {
	uint8_t idx = i - 1;
	uint8_t idx3 = idx*3;
	if (i>=compressor_level) {
	    in_comp=IN_COMP_HIGH;
	    in_comp_l = IN_COMP_LOW;
	    comp_gain[idx]=1;
	} else {
	    comp_gain[idx]=0;
	    in_comp = in_comp_l = 0;
	}
	if (i<=total_amp_l) {
	    led_levels[idx]=1;
	    //color = urgb_u32(amp_rgb[0],amp_rgb[1],amp_rgb[2]);
	    //color = urgb_u32(0,16,0);
	    // green
	    moving_average(avg_led_rgb[idx3],0+in_comp,false);
	    moving_average(avg_led_rgb[idx3+1],min(avg_led_rgb[idx3+1]->value+16+in_comp,32+in_comp),false);
	    //moving_average(avg_led_rgb[idx3+1],16+in_comp,false);
	    moving_average(avg_led_rgb[idx3+2],0+in_comp,false);
	} else if (i<=peak_amp_l) {
	    //color = urgb_u32(peak_rgb[0],peak_rgb[1],peak_rgb[2]);
	    //color = urgb_u32(6,3,0);
	    // orange 
	    moving_average(avg_led_rgb[idx3],24+in_comp,false);
	    moving_average(avg_led_rgb[idx3+1],12+in_comp_l,false);
	    moving_average(avg_led_rgb[idx3+2],in_comp_l,false);	    
	    led_levels[idx]=led_levels[idx]*0.95;
	} else {
	   
	    led_levels[idx]=led_levels[idx]*0.5;	    
	    //color = 0;

	     if (i == gate_pixel && gate_closed) {
		 moving_average(avg_led_rgb[idx3], 10.0,false);
		 moving_average(avg_led_rgb[idx3+1],((float)avg_led_rgb[idx3+1]->value*0.9),false);	    
		 moving_average(avg_led_rgb[idx3+2], 10.0,false);
	     } else {
		 if (in_comp && current_state->compression_gain < 0.95) {		    
		     avg_led_rgb[idx3]->value = in_comp_l;    // reset the avg to be at this value at the tip
		     avg_led_rgb[idx3+1]->value = in_comp_l;
		     avg_led_rgb[idx3+2]->value = in_comp;
		     
		     //moving_average(avg_led_rgb[idx3], in_comp_l,false);  // a grey should be where the compressor is active
		     //moving_average(avg_led_rgb[idx3+1], in_comp_l,false);	    
		     //moving_average(avg_led_rgb[idx3+2], in_comp,false);	    
		 } else {
		     moving_average(avg_led_rgb[idx3], ((float)avg_led_rgb[idx3]->value*LED_DECAY_RATE), false);
		     moving_average(avg_led_rgb[idx3+1], ((float)avg_led_rgb[idx3+1]->value*LED_DECAY_RATE), false);
		     moving_average(avg_led_rgb[idx3+2], no_amplitude_color, false);	    // default deep blue
		 }
		 
	     }
	}
	
    }
    // start at 0, but keep our index value in the average array
    // right channel
    uint8_t amp = 0;
    for(uint8_t i = AMPLITUDE_PIXEL_COUNT/2+1; i <= AMPLITUDE_PIXEL_COUNT; i++) {
	amp++;
	uint8_t idx = i - 1;
	uint8_t idx3 = idx*3;

	if (amp>=compressor_level) {
	    in_comp=IN_COMP_HIGH;
	    in_comp_l = IN_COMP_LOW;
	} else {
	    in_comp = in_comp_l =0;	    
	}
	if (amp<=total_amp_r) {
	    led_levels[idx]=1;
	    moving_average(avg_led_rgb[idx3],0+in_comp,false);
	    moving_average(avg_led_rgb[idx3+1],min(avg_led_rgb[idx3+1]->value+16+in_comp,32+in_comp),false);
	    moving_average(avg_led_rgb[idx3+2],0+in_comp,false);
	} else if (amp<=peak_amp_r) {
	    moving_average(avg_led_rgb[idx3],24+in_comp,false);
	    moving_average(avg_led_rgb[idx3+1],12+in_comp_l,false);
	    moving_average(avg_led_rgb[idx3+2],in_comp_l,false);	    
	    led_levels[idx]=led_levels[idx]*0.95;
	} else {
	    led_levels[idx]=led_levels[idx]*0.5;	    
	    //color = 0;
	    if (in_comp && current_state->compression_gain < 0.95) {		
		    avg_led_rgb[idx3]->value = in_comp_l;    // reset the avg to be at this value at the tip
		    avg_led_rgb[idx3+1]->value = in_comp_l;
		    avg_led_rgb[idx3+2]->value = in_comp;
		
		//moving_average(avg_led_rgb[idx3], in_comp_l,false);  // a grey should be where the compressor is active
		//moving_average(avg_led_rgb[idx3+1], in_comp_l,false);	    
		//moving_average(avg_led_rgb[idx3+2], in_comp,false);	    	       
	    } else {		
		moving_average(avg_led_rgb[idx3], ((float)avg_led_rgb[idx3]->value*LED_DECAY_RATE), false);
		moving_average(avg_led_rgb[idx3+1], ((float)avg_led_rgb[idx3+1]->value*LED_DECAY_RATE), false);
		//	    moving_average(avg_led_rgb[idx3+2],(uint32_t) ((float)avg_led_rgb[idx3+1]->value*0.9),false);
		moving_average(avg_led_rgb[idx3+2], no_amplitude_color,false);	    // default deep blue
	    }
	}
    }
}

// send data to neopixel LED array

void send_amp() {
    uint32_t color;
    // go through the array and send values.  Since we need to be super reactive with
    // current amplitude values, override the average to show the actual current amplitude.
    if (current_state->display_level) {
	for(uint8_t i = 1; i <= AMPLITUDE_PIXEL_COUNT; i++) {
	    uint8_t idx = i - 1;
	    uint8_t idx3 = idx*3;
	    if (led_levels[idx] >=1) {
		color = urgb_u32(0,18,0);
	    } else {
		color = urgb_u32(avg_led_rgb[idx3]->value,avg_led_rgb[idx3+1]->value,avg_led_rgb[idx3+2]->value);
	    }
	    put_pixel(amplitude_inst,color);
	}
    }
    
}

// each character 6x12
#define ROW_HEIGHT 12
#define COL_WIDTH 6

#define set_stime stime = time_us_64()
#define record_stime update_times[i]=time_us_64() - stime; stime = time_us_64();i++
// check update timing



void update_display() {

    if (current_state->display_on == 0) return;
    ui_update(); // do whatever ui updates might need to be done - 
    flush_to_display(display);
    
}




void refresh_display() {
    
    uint64_t stime = time_us_64();
    ui_update();
    current_state->display_refresh_time_us = time_us_64() - stime;
}

void send_to_dsp(char *text) {

    if (text==NULL) return; // no empty messages - this would be an error

    // if empty queue    
    if (dsp_queue == NULL) {
	dsp_queue = (dsp_xmit_queue *) calloc(1,sizeof(dsp_xmit_queue));	
	dsp_queue->next = 0;
	dsp_queue->prior = 0;
    } else {
	dsp_queue->next = (dsp_xmit_queue *) calloc(1,sizeof(dsp_xmit_queue));
	dsp_queue->next->prior=dsp_queue;
	dsp_queue=dsp_queue->next;
	dsp_queue->next = 0;
	
    }
    dsp_queue->message = str_alloc(strlen(text));
    strcpy(dsp_queue->message,text);
}

void send_dsp_queue() {
    static uint64_t last_send = 0;
    if (dsp_queue == 0) return;  // nothing in the queue, so return
    
    if (uart_is_writable(uart1) && to_ms_since_boot(get_absolute_time()) > last_send+10) {
	uint16_t num_updates = 1;
	uint16_t update_count = 1;
	dsp_xmit_queue *sent = NULL;          // for the element we just transferred
	dsp_xmit_queue *current = dsp_queue;  // grab the list...
	dsp_queue = NULL;                        // and snip off the old list in current
	// record when we last sent
	last_send = to_ms_since_boot(get_absolute_time());
	while(current->prior != NULL) {
	    current = current->prior;
	    num_updates++;
	}
	// we have current at the first element queued for transmision..
	// loop through and send throuth the uart
	while(current != NULL && update_count <= num_updates) {
	    current->prior = NULL;
	    uart_puts(uart1,current->message);
	    sent = current;
	    current=current->next;
	    current->prior = NULL;
	    free(sent->message);
	    free(sent);
	    update_count++;
	}
	
    }
}

void set_gate_on(float state) {
    if (state != 0) state = 1;
    else state = 0;
    current_state->gate_active = (uint8_t) state;
    char buf[20];
    printf("%s: %.0f->%d\n",__FUNCTION__, state, current_state->gate_active);
    sprintf(buf,"G%d\n",current_state->gate_active);
    send_to_dsp(buf);
}

void set_gate_threshold(float db) {
    current_state->gate_threshold_dB = clamp(db,-100,0);
    char buf[20];
    printf("%s: %0.4fn->%0.4f\n",__FUNCTION__, db, current_state->gate_threshold_dB);
    sprintf(buf,"N%.4f\n",current_state->gate_threshold_dB);
    send_to_dsp(buf);
}


void set_gate_attack(float ms) {
    float attack_rate = ms;
    current_state->gate_attack_ms = clamp(attack_rate,0,3000);
    char buf[20];
    sprintf(buf,"A%d\n",(int) current_state->gate_attack_ms);
    send_to_dsp(buf);
}


void set_gate_hold(float ms) {
    float hold_rate = ms;
    current_state->gate_hold_ms = clamp(hold_rate,0,3000);
    char buf[20];
    sprintf(buf,"H%d\n",(int) current_state->gate_hold_ms);
    send_to_dsp(buf);
}


void set_gate_release(float ms) {
    float release_rate = ms;
    current_state->gate_release_ms = clamp(release_rate,0,3000);
    char buf[20];
    sprintf(buf,"E%d\n",(int) current_state->gate_release_ms);
    send_to_dsp(buf);
}

void set_compressor_on(float state) {
    if (state != 0) state = 1;
    else state = 0;
    current_state->compressor_on = (bool) state;
    char buf[20];
    printf("%s: %.0f->%d\n",__FUNCTION__, state, current_state->compressor_on);
    sprintf(buf,"c%d\n",current_state->compressor_on);
    send_to_dsp(buf);
}


void set_compressor_threshold(float db) {    
    current_state->threshold_dB = clamp(db,-40,0);
    char buf[20];
    printf("%s: %0.4fn->%0.4f\n",__FUNCTION__, db, current_state->threshold_dB);
    sprintf(buf,"t%.4f\n",current_state->threshold_dB);
    send_to_dsp(buf);
}


void set_compressor_makeup(float db) {

    current_state->makeup_db = clamp(db,0,18);
    printf("%s: %0.4fdB\n",__FUNCTION__, current_state->makeup_db);
    char buf[20];
    sprintf(buf,"m%.4f\n",current_state->makeup_db);
    send_to_dsp(buf);
}


void set_compressor_attack(float ms) {
    float attack_rate = ms;
    current_state->attack_rate_ms = clamp(attack_rate,0,250);
    char buf[20];
    sprintf(buf,"a%d\n",(int) current_state->attack_rate_ms);
    send_to_dsp(buf);
}


void set_compressor_release(float ms) {
    float rate = (uint16_t) ms;
    current_state->release_rate_ms = clamp(rate,0,1000);
    char buf[20];
    sprintf(buf,"r%d\n",(int) current_state->release_rate_ms);
    send_to_dsp(buf);
}

float slider_velocity = 0;

void calc_slider_db() {
    float sp = current_state->slider_percent;
    if (sp > 1.0) {
	current_state->slider_db = ratio_to_dB(map_range(sp,1,gain_range,1,2));
    } else {
	current_state->slider_db = ratio_to_dB(current_state->slider_percent);
    }
    //printf("slider: %.4fdB  lin=%.4f raw=%d vel=%.3f\n",current_state->slider_db,sp, slider_position, slider_velocity);

}

void on_slider_movement() {
    char buf[20];
    static uint64_t last_send = 0;
    calc_slider_db();
    
    // throttle so we aren't overwhelming with changes...    
    if (uart_is_writable(uart1) && to_ms_since_boot(get_absolute_time()) > last_send+20) {	
	last_send = now_ms();
	sprintf(buf,"g%.3f\n",current_state->slider_percent);	
	send_to_dsp(buf);
	mark_slider_state_updated();
	//mark_state_updated();
    }
}




void adc_ready_callback(uint gpio, uint32_t events) {
    adc_ready = true;
}



void check_adc() {
    if (adc_ready && current_state->slider_adc_initialized) {
	adc_ready = false;
	static uint64_t num_checks = 0;
	static uint64_t last_move = 0;
    
	int16_t pos = read_adc_register(ads,0);
	raw_val= pos;
	/*if (pos > 32767) {
	    pos = 0;
	    }*/
        
	pos = pos >> 2; // shift right to reduce read noise..
	slider_position = (int16_t) moving_average(adc_avg,pos,false);       
	slider_velocity = slider_position - slider_position_locked;
	if (abs(slider_position - slider_position_locked) > 4) {
	    slider_position_locked = slider_position;
	    last_move = num_checks;
	    current_state->slider_percent = (float) slider_position/slider_divisor;
	    if (num_checks > 20) on_slider_movement();
	} else if ((slider_position_locked != slider_position) && ((num_checks - last_move) < 3)) {
	    current_state->slider_percent = (float) slider_position/slider_divisor;
	    if (num_checks > 12) on_slider_movement();
	}    
	num_checks++;
    }
}


void start_adc() {
    printf("ADS1115: initializing..\n");
    start_adc_reading(ads, 0);
    current_state->slider_adc_initialized = true;
}



uint64_t motor_on = 0;
uint64_t pulse_time = 0;
uint64_t pulse_start = 0;
uint16_t cycle_count = 0;
uint16_t pulse_time_coef = 80;
uint32_t max_cycle_count = 20000;


void move_up(uint8_t pulse_length) {
    gpio_put(SLIDER_DOWN,0);
    gpio_put(SLIDER_UP,1);
    motor_on = current_state->uptime_milliseconds;
    pulse_start = motor_on;
    pulse_time = pulse_length;
    // printf("U:p %lld\n",pulse_time);
}

void move_down(uint8_t pulse_length) {
    gpio_put(SLIDER_UP,0);
    gpio_put(SLIDER_DOWN,1);
    motor_on = current_state->uptime_milliseconds;
    pulse_start = motor_on;
    pulse_time = pulse_length;
    // printf("D:p %lld\n",pulse_time);
}

void motor_off() {
    gpio_put(SLIDER_UP,0);
    gpio_put(SLIDER_DOWN,0);
    //if (pulse_time != (current_state->uptime_milliseconds - pulse_start)) {
    //	printf("O:p: %lld  ms:  %llu  %d\n",pulse_time, current_state->uptime_milliseconds - pulse_start,cycle_count);
    //}
    motor_on = 0;
    pulse_start = 0;
    pulse_time = 0; 
}

void brake(uint8_t pulse_length) {
    gpio_put(SLIDER_UP,1);
    gpio_put(SLIDER_DOWN,1);
    motor_on = current_state->uptime_milliseconds;
    pulse_start = motor_on;
    pulse_time = pulse_length;        
}

void coast(uint8_t pulse_length) {
    motor_off();
    pulse_start = current_state->uptime_milliseconds;
    pulse_time = pulse_length;
    //printf("C:p: %lld \n",pulse_time);
}


void set_slider_position(float pos) {
    current_state->slider_fault_mode = false;
    pos = clamp(pos,0,1);
    printf("Slider target position: %f\n",pos);    
    current_state->slider_target_position = pos;
    pulse_start = current_state->uptime_milliseconds;
    cycle_count = 1; // enable slider movement
}
/* Lisp:
  (defun ms_to_pos (ms)
    (* 1.535 (- (Math.exp (/ ms 100)) 1)))
 */
// use to roughly calculate approximate motor on times to position movement
float ms_to_delta (uint8_t ms) {
    return ((exp(((float) ms / 100.0))-1) * 1.3);
}

void build_initial_movement_table() {
    delta_to_ms *delta_list = (delta_to_ms *) calloc(1,sizeof(delta_to_ms));
    delta_list_start = delta_list;  // capture the start
    delta_to_ms *prior_delta_to_ms = 0;
    // build delta_to_ms list    
    for (uint8_t i = 1; i <= 50; i++) {
	delta_list->ms = i;
	delta_list->delta = ms_to_delta(i);
	delta_list->prior = prior_delta_to_ms;
	if (prior_delta_to_ms != NULL) {
	    prior_delta_to_ms->next = delta_list;
	}
	prior_delta_to_ms = delta_list;
	delta_list = (delta_to_ms *) calloc(1,sizeof(delta_to_ms));
    }
    // last delta_list
    delta_list->next = 0;
}

void show_movement_list() {
    delta_to_ms *delta_list = delta_list_start;
    if (delta_list != 0) {
	printf("DELTA   MS\n");
	while(delta_list->next != 0) {
	    printf("  %.2f   %d\n",delta_list->delta,delta_list->ms); 
	    delta_list = delta_list->next;
	}
    }
}

uint movement_list_size() {
    delta_to_ms *delta_list = delta_list_start;
    uint c = 0;
    if (delta_list != 0) {
	while(delta_list->next != 0) {
	    c++;
	    delta_list = delta_list->next;
	}
    }
    return c;
 }

uint serialize_movement_list(uint8_t *storage) {
    delta_to_ms *delta_list = delta_list_start;
    char buf[30];
    uint c = 0;
    if (delta_list != 0) {
        sprintf(buf,"%.3f %d,",delta_list->delta,delta_list->ms);
	memcpy(storage+c,(uint8_t *)buf,strlen(buf));
	c=c+strlen(buf);
	while(delta_list->next != 0) {	    
	    delta_list = delta_list->next;
	}
    }
    return c;
}


delta_to_ms *get_delta_pos(float delta) {
    delta_to_ms *delta_list = delta_list_start;   
    if (delta_list != NULL && delta > 0) {
	do {
	    if (delta_list->delta == delta) {
		return delta_list;  // exact match
	    } else if (delta_list->delta > delta) {
		return delta_list->prior; // return the one prior
	    }
	    delta_list=delta_list->next;
	} while(delta_list != NULL);	
    }
    return NULL;
}

delta_to_ms *get_delta_ms(uint8_t ms) {
    delta_to_ms *delta_list = delta_list_start;   
    if (delta_list != NULL && ms > 0) {
	do {
	    if (delta_list->ms == ms) {
		return delta_list;  // exact match
	    } else if (delta_list->ms > ms) {
		return delta_list->prior; // return the one prior
	    }
	    delta_list=delta_list->next;
	} while(delta_list != NULL);	
    }
    return NULL;
}


void update_delta_list(float delta, uint8_t ms) {
    delta_to_ms *delta_list = get_delta_ms(ms);
    delta_to_ms *next;
    delta_to_ms *new_elem;
    if (delta < 0.001) {
	return;
    }
    if (delta_list) {
	next = delta_list->next;
	if (delta_list->ms == ms) {
	    delta_list->delta = delta;	   
	    return;
	} else if (delta_list->delta == delta) {
	    delta_list->delta = delta;
	} else if (next && next->ms > ms) {
	    // we need to slot in the value between delta_list and next
	    new_elem = (delta_to_ms *) calloc(1,sizeof(delta_to_ms));
	    new_elem->ms = ms;
	    new_elem->delta = delta;
	    new_elem->next = next;
	    new_elem->prior = delta_list;	    
	    delta_list->next = new_elem;
	}
    }
}

uint8_t get_ms_for_delta(float delta) {
    if (delta == 0) {
	return 0;
    } else {
	delta_to_ms *elem = get_delta_pos(delta);
	if (elem != NULL) {
	    return elem->ms;
	}
    }
    return 0;
}

void slider_to_zero() {
    uint8_t ms = 0;
    uint8_t count = 0;
    while (current_state->slider_percent > 0.0) {
	ms = get_ms_for_delta(current_state->slider_percent); // get the current position and apply the ms to return to 0
	gpio_put(SLIDER_DOWN,1);
	sleep_ms(ms);
	gpio_put(SLIDER_DOWN,0);
	sleep_ms(100);
	count++;
	if (count > 5) break;
    }
    printf("0 [%.3f]\n",current_state->slider_percent);    
}

volatile bool do_calibration = false;
volatile uint8_t calibration_state = 0;

void start_calibration() {
    calibration_state = 1;
    do_calibration = true;
}

void calibrate_slider() {
    static uint8_t ms = 1;
    switch (calibration_state) {
    case 1:	
	calibration_state = 2;
	set_slider_position(0.0);	
	break;    
    case 3:	
	calibration_state = 4;
	pulse_start = current_state->uptime_milliseconds;
	cycle_count = 1;
	move_up(ms);	
	break;
    case 5:
	if (abs(slider_velocity) < 5) {
	    calibration_state = 6;
	} 
	break;
    case 6:	
	update_delta_list(current_state->slider_percent, ms);
	ms++;
	calibration_state = 1;      
    }
    if (ms > 51) {
	motor_off();
	ms = 1; // reset
	calibration_state = 0;
	do_calibration = false;
	show_movement_list();
    }
}

// Adjust to the current target position to the current gain
// core 0 - manage slider motor
// core 1 - slider value status
void align_slider() {
    // diff will be positive if the slider is higher than the target position
    // diff will be negative if the slider is lower than the target position
    float diff = current_state->slider_percent - current_state->slider_target_position;
    float adiff = diff;
    uint64_t ctime = current_state->uptime_milliseconds;   
    static uint64_t last_ctime = 0;
    uint64_t cdur = ctime - pulse_start;
    if (cdur < 0) cdur = 0;
    if (adiff < 0) adiff *=-1;
    //if ((cycle_count > 1) && (ctime > (last_ctime + 1))) {
    //	printf("high delay: %llu\n",ctime - last_ctime);
    //}
    // if (ctime == last_ctime) return; // same millisecond ..wait for the next
    // if we are in a fault do nothing
    if (current_state->slider_fault_mode == true) {
	motor_off(); // ensure that the motor pins are pulled low to stop current from the motor
	last_ctime = ctime;
	return;
    }
    if ((motor_on > 0) && (ctime - motor_on) > 500) {
	printf("motor: fault mode set.\n");
	motor_off(); 
	current_state->slider_fault_mode = true;
	last_ctime = ctime;
	return;
    } else if (cycle_count > max_cycle_count) {
	motor_off();
	printf("motor: unable to attain target: diff is: %f cycle time: %llu\n",diff, ctime - last_ctime);
	cycle_count = 0;	
    } else if (cycle_count > 0) {
	// if we are here, we are on the hunt!
	//if (cycle_count == 1) {
	//    printf("target tracking to: %.3f from delta: %.3f\n",current_state->slider_target_position, diff);
	//}
	cycle_count++;
	if (pulse_time > 0 && cdur >= pulse_time) {
	    if (do_calibration && calibration_state == 4) {
		motor_off();
		cycle_count = 0;
		calibration_state = 5;
		last_ctime = ctime;
		return;
	    }
	    //if (slider_velocity > 40) {		
	    //move_down(1);		
	    //} else if (slider_velocity < 40) {
	    //	move_up(1);	    
	    if (motor_on > 0) {
		coast(20); // sample to where we are;		
	    } else {
		pulse_time = 0;
	    }
	} else if (pulse_time > 0 && cdur < pulse_time) {
	    // we are still in an action so let it continue...
	    last_ctime = ctime;
	    return;
	}
	if (pulse_time == 0 && abs(slider_velocity) < 4) {
	    if (adiff < 0.01) {
		motor_off(); // just in case..
		printf("target reached: cycles: %d diff: %.3f\n",cycle_count,diff);
		current_state->slider_target_position = current_state->slider_percent;
		cycle_count = 0;
		if (do_calibration && calibration_state == 2 && current_state->slider_percent < 0.1) {
		    calibration_state = 3;
		    last_ctime = 0;
		    return;
		}	    
	    } else {
		if (adiff < 0.02) {
		    if (diff > 0) {
			move_down(2);
		    } else {
			move_up(2);
		    }
		} else {
		    if (diff > 0) {
			move_down(get_ms_for_delta(adiff));
		    } else {
			move_up(get_ms_for_delta(adiff));
		    }
		}
	    } 
	}
    }
    last_ctime = ctime;
}


//static mutex_t mutex;      // control access to the current state




void clear_entry(char *buf) {
  for (unsigned int i = 0; i < ENTRY_SIZE; i++) {
    buf[i] = 0;
  }
}

void save_config() {
    size_t save_image_size = (((10*movement_list_size()+1024)/FLASH_PAGE_SIZE) + 1) * FLASH_PAGE_SIZE;
    uint size=0;
    uint8_t* save_image = (uint8_t *) calloc(save_image_size,sizeof(uint8_t));
    size=serialize_movement_list(save_image);
    printf("save_config: size=%d  image_size: %d\n",size,save_image_size);
    free(save_image);
}







absolute_time_t last_display_check = 0;

// handles the output to display, leds, etc..
// core 0
void core0_run_loop() {
    printf("%s: started.\n",__FUNCTION__);
    while(true) {
	uint64_t stime = time_us_64();
	current_state->uptime_milliseconds = to_ms_since_boot(get_absolute_time()); // update our millisecond  counter 	
	if (current_state->uptime_milliseconds > last_display_check) {
		// every millisecond
	    check_adc();
	    check_dsp();
		// every 5 milliseconds
	    if (current_state->uptime_milliseconds % 5 == 0) {
		process_led_levels();
		send_amp();
		adc_ready = true;
	    }
	    // every 10 milliseconds
	    if (current_state->uptime_milliseconds % 20 == 0) {
		if (do_calibration) {
		    calibrate_slider();		    
		}		
		 
	    }	    
	}
	last_display_check = current_state->uptime_milliseconds;	
	current_state->core_0_cycle_time_us = time_us_64() - stime;
        align_slider();
	sleep_us(60);
    }    
}

// check buttons and mulitplexed buttons

void check_buttons() {
    static uint8_t mux_addr = 0;
    uint8_t mux_read_target = MUX_READ_A;
    uint8_t dir_button = 0;
    if (mux_addr>3) mux_read_target = MUX_READ_B;
    if (mux_addr > 7) {
	// directly wired buttons
	switch(mux_addr) {
	case 8:
	    dir_button = SWITCH_I;
	    break;
	case 9:
	    dir_button = SWITCH_J;
	    break;
	case 10:
	    dir_button = SWITCH_R;
	    break;
	}
	if (gpio_get(dir_button) == 0) {
	    // target button is pressed
	    if (buttons[mux_addr] == 0) {
		button_event(mux_addr,PRESS,0);
		printf("button press: %d\n",mux_addr);
	    }
	    buttons[mux_addr]++;	    
	} else {
	    if (buttons[mux_addr] > 0) {
		button_event(mux_addr,RELEASE,buttons[mux_addr]);
		printf("button release: %d\n",mux_addr);
	    }
	    buttons[mux_addr] = 0;
	}
    } else {
	// multiplexed buttons
	switch(mux_addr%4) {
	case 0:
	    gpio_put(MUX_SEL_A,0);
	    gpio_put(MUX_SEL_B,0);
	    break;
	case 1:
	    gpio_put(MUX_SEL_A,1);
	    gpio_put(MUX_SEL_B,0);
	    break;
	case 2:
	    gpio_put(MUX_SEL_A,0);
	    gpio_put(MUX_SEL_B,1);
	    break;
	case 3:
	    gpio_put(MUX_SEL_A,1);
	    gpio_put(MUX_SEL_B,1);
	    break;
	}
	if (gpio_get(mux_read_target) == 0) {
	    if (buttons[mux_addr]==0) {
		button_event(mux_addr,PRESS,0);
		printf("button press: %d\n",mux_addr);	    
	    }
	    buttons[mux_addr]++;
	} else {
	    if (buttons[mux_addr] > 0) {
		button_event(mux_addr,RELEASE,buttons[mux_addr]);
		printf("button release: %d\n",mux_addr);
	    }
	    buttons[mux_addr] = 0;	 
	}
    }
    mux_addr++;
    mux_addr = mux_addr % NUM_BUTTONS;   // cycle through NUM_BUTTONS
    
}

// rotary encoder callbacks



void rotary_up() {
    rotary_event(1);
    pio_interrupt_clear(re->pio, 0);
}

void rotary_down() {
    rotary_event(0);
    pio_interrupt_clear(re->pio, 1);
}

int int_arg(char *str) {
    return atoi(strtok(str, " "));
}

float float_arg(char *str) {
    return atof(strtok(str, " "));
}


void update_dsp_settings(char *args) {
    current_state->compressor_on = int_arg(args);
    current_state->attack_rate_ms = int_arg(NULL);
    current_state->release_rate_ms = int_arg(NULL);
    current_state->threshold_dB = float_arg(NULL);
    current_state->makeup_db = float_arg(NULL);
    current_state->ratio = float_arg(NULL);
    current_state->output_mix = float_arg(NULL);

    current_state->input_trim_gain = float_arg(NULL);
    current_state->balance = float_arg(NULL);
    current_state->channel_gain = float_arg(NULL);
    current_state->send1_gain = float_arg(NULL);
    current_state->send2_gain = float_arg(NULL);
    mark_state_updated();
    //set_ui_needs_update();
}

void process_activity(char *args) {
    char *amp_l;
    char *amp_r;
    char *output_amp_l;
    char *output_amp_r;
    char *peak_l;
    char *peak_r;
    char *output_peak_amp_l;
    char *output_peak_amp_r;
    char *comp_gain;
    char *gate_open;
    char *muted;
    char *start = args;
    uint16_t i = 0;
    for (i = 0; i <= 100;i++) { // only go the first 100, if we pass it, ignore this packet
	if (args[i]=='A') {
	    start=args+i;
	    break;
	}
    }
    if (i== 100) {
	printf("NOPE\n");
	return;
    }
    strncpy(text_buffer,start,ENTRY_SIZE-1);
    amp_l = strtok(text_buffer, " ");
    amp_r = strtok(NULL, " ");	    
    peak_l = strtok(NULL, " ");
    peak_r = strtok(NULL, " ");
    output_amp_l = strtok(NULL, " ");
    output_amp_r = strtok(NULL, " ");
    output_peak_amp_l = strtok(NULL, " ");
    output_peak_amp_r = strtok(NULL, " ");
    comp_gain = strtok(NULL, " ");
    muted = strtok(NULL, " ");
    gate_open = strtok(NULL, " ");
    current_state->current_input_amp_l = (float) atof(amp_l)/COMMON_RANGE;
    current_state->current_input_amp_r = (float) atof(amp_r)/COMMON_RANGE;
    current_state->peak_amp_l = (float) atof(peak_l)/COMMON_RANGE;
    current_state->peak_amp_r = (float) atof(peak_r)/COMMON_RANGE;
    current_state->output_amp_l = (float) atof(output_amp_l)/COMMON_RANGE;
    current_state->output_amp_r = (float) atof(output_amp_r)/COMMON_RANGE;
    current_state->output_peak_amp_l = (float) atof(output_peak_amp_l)/COMMON_RANGE;
    current_state->output_peak_amp_r = (float) atof(output_peak_amp_r)/COMMON_RANGE;
    current_state->compression_gain = atof(comp_gain) / 65535.0;
    current_state->muted = (bool) atoi(muted);
    if (current_state->muted == true) {
	gpio_put(LED_GPIO,1);
	printf("??: %s\n",args);
    } else {
	gpio_put(LED_GPIO,0);
    }
    current_state->gate_open = atoi(gate_open);
}


void check_dsp() {
    // any updates queued?
    send_dsp_queue();
    static uint8_t idx = 0;
    char *args = recv_buffer+1;
    char command;
    while (uart_is_readable(uart1)) {
	char c = uart_getc(uart1);
	if (c=='\n') {
	    // process received data	    
	    recv_buffer[idx]=0;	    
	    idx = 0;
	    command = recv_buffer[0];
	    switch (command) {
	    case 'A':
		process_activity(args);
		break;		
	    case 'E':
		printf("error: %s\n",args);
		break;
	    case 'C':  // channel settings
		update_dsp_settings(args);
		break;
	    }
	} else {
	    recv_buffer[idx]=c;
	    idx=(idx+1)%256;    
	}
    }
}


// listen for control events on the UARTS
// if a command is received, return with the command,
// otherwise check for incoming DSP data every millisecond

// core 1

char *core1_run_loop() {
  char dur_buffer[20];
  unsigned int pos = 0;
  char log_state = current_state->log_activity;
  char c;
  uint64_t dcheck = now_ms();
  clear_entry(cmd_buffer);
  while(1) {
      
    int rval = stdio_getchar_timeout_us(1000);
    if (rval ==PICO_ERROR_TIMEOUT) {	
	// check_dsp();
	// check_adc();
	check_buttons();
	if ((now_ms() - dcheck) > 20) {
	    dcheck = now_ms();	    
	    update_display();
	    current_state->core_1_cycle_time_us = now_ms() - dcheck;
	}
	continue;
    } else {
	c = rval;
	//if (uart_is_writable(uart1)) {
	//    uart_putc(uart1,c);
	//}
    }
    if ((current_state->log_activity == true) && (pos == 0)) {
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
    printf("  l - log current state to the console on/off\n");
    printf("  S - set minimum permissible cycle steps for slew\n");    
    printf("  C - clear the screen.\n");
    printf("  _ - set slider position [0.0 to 1.0]\n");
    printf("  * - show core 1 cycle time\n");
    printf("  ? - print this menu.\n");
    printf("Esc - clear command line entry.\n\n");
    printf("examples: set attack to 2ms:      a2\n");
    printf("          set threshold to -18dB: t18   or:  t-18\n");
    printf("          set a 2.5:1 make up:    m2.5\n\n");

}

void handle_command(char cmd, char* args) {
    float f; 
    int32_t i;
    uint8_t ms;
    char *arg;
    uint8_t row;
    uint8_t col;
    switch(cmd) {
    case '?':
	show_help();
	break;
    case 'l':
	if (current_state->log_activity) {
	    current_state->log_activity = false;
	    printf("logging is off\n");
	} else {
	    current_state->log_activity = true;
	    printf("logging is on\n");
	}
	break;
    case 's':
	request_channel_status();
	break;
    case 'X':
	i = atoi(args);
	current_state->display_level = i;
	current_state->display_on = i;
	refresh_display();
	break;
    case 'a':
	i = atoi(args);
	if (i>0) {
	    write_adc_register(ads, 0x01, i); // write to the config register	    
	} else {
	    check_adc();
	}		
	break;
    case '*':
	printf("\nCore 1 cycle time (microseconds): %llu\n",current_state->core_1_cycle_time_us);
	break;
    case 'P':
	i = atoi(args);
	pulse_time_coef = clamp(i,1,140);
	printf("pulse time coef: %d\n",pulse_time_coef);
	break;
    case 'M':
	i = atoi(args);
	max_cycle_count = max(1,i);
	printf("max cycle count: %ld\n",max_cycle_count);
	break;
    case 'R':
	printf("Reboot DSP..\n");
	gpio_put(RESET_DSP,1);
	sleep_us(5000);
	gpio_put(RESET_DSP,0);
	printf("Done.\n");
	break;
    case 'G':
	gain_range = clamp(atof(args),1,3);
	printf("Gain range set to: %.3f\n",gain_range);
	break;
    case 'D':
	printf("Old divisor: %f\n",slider_divisor);
	slider_divisor = clamp(atof(args),4096.0,12000.0);
	printf("New divisor: %f\n",slider_divisor);
	break;
    case '_':
	f = atof(args);
	if (f > 1) {
	    f = f / 100;  // if the entry is a percentage, divide it by 100
	}
	set_slider_position(f);
	break;
    case '-':
	f = atof(args);
	f = clamp(f,0,1);
	ms = get_ms_for_delta(f);	
	printf("delta: %f -> ms: %d\n",f,ms);
	ms = clamp(ms,0,50);
	if (ms > 0) {
	    gpio_put(SLIDER_DOWN,1);
	    sleep_ms(ms);
	    gpio_put(SLIDER_DOWN,0);
	}
	break;
    case '+':
	f = atof(args);
	f = clamp(f,0,1);
	ms = get_ms_for_delta(f);
	printf("delta: %f -> ms: %d\n",f,ms);
	ms = clamp(ms,0,50);
	if (ms > 0) {
	    gpio_put(SLIDER_UP,1);
	    sleep_ms(ms);
	    gpio_put(SLIDER_UP,0);
	}
	break;
    case 'C':
	start_calibration();
	break;
    case '>':
	i = atoi(args);
	i = clamp(i,0,400);
	gpio_put(SLIDER_UP,1);
	sleep_ms(i);
	gpio_put(SLIDER_UP,0);
	break;
    case '<':
	i = atoi(args);
        i = clamp(i,0,400);
	gpio_put(SLIDER_DOWN,1);
	sleep_ms(i);
	gpio_put(SLIDER_DOWN,0);
	break;
    case 't':
	arg = strtok(args," ");
	row = atoi(arg);
	col = atoi(strtok(NULL," "));
	char *text = strtok(NULL," ");
	printf("text_at: row=%d col=%d text=%s\n",row,col,text);
	text_at(row,col,text);
	break;
    case 'c':
	clear_display(display);       
	break;
    case 'e':
	arg = strtok(args," ");
	row = atoi(arg);
	col = atoi(strtok(NULL," "));
	uint8_t num = atoi(strtok(NULL," "));
	clear_text(row,col,num);
	break;
    case 'E':
	i = clamp(atoi(args),0,3);
	display->color = i;
	printf("text color: %d\n",display->color);
	break;    	
    case 'd':
	dump_display();
	break;
    default:	
	printf("\nType ? for command menu.\n");
    }
}

// core 1 - handles term io and display
void control_loop() {
    char cmd;
    char *args;
    int cmd_len;
    char *termcodes = (char *)calloc(50,sizeof(char));
    char *rows = 0;
    char *cols = 0;
    uint8_t idx = 0;
    printf("\033[s\033[999;999H\033[6n\033[u");
    while(1) {
	termcodes[idx] = stdio_getchar_timeout_us(100000);
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
    request_channel_status();
    printf("\n");
    printf("\nEnter ? for help.\n");
    while(1) {
	if (current_state->run_mode == 0) {
	    printf("\n-> "); // print a prompt to the UART0
	}
	cmd_buffer = core1_run_loop();
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
    
    
    if (resp != CORE1_INIT_FLAG) {
	printf("\nERROR: core 1 didn't receive the flag value from core 0 for initializing sequence.");
    } else {
	printf("core1: initialized - starting up.\n");
	i2c_init(i2c0, 100 * 1000);	
	printf("core1: i2c initialized.\n");
	gpio_set_function(I2C_SDA0, GPIO_FUNC_I2C);
	gpio_set_function(I2C_SCL0, GPIO_FUNC_I2C);

	gpio_pull_up(I2C_SDA0);
	gpio_pull_up(I2C_SCL0);

	i2c_init(i2c1, 100*1000);
	gpio_set_function(I2C_SDA1, GPIO_FUNC_I2C);
	gpio_set_function(I2C_SCL1, GPIO_FUNC_I2C);
	gpio_pull_up(I2C_SDA1);
	gpio_pull_up(I2C_SCL1);



	gpio_init(MUX_SEL_A);
	gpio_set_dir(MUX_SEL_A, GPIO_OUT);
	gpio_pull_down(MUX_SEL_A);

	gpio_init(MUX_SEL_B);
	gpio_set_dir(MUX_SEL_B, GPIO_OUT);
	gpio_pull_down(MUX_SEL_B);
	
	gpio_init(MUX_READ_A);
	gpio_set_dir(MUX_READ_A, GPIO_IN);

	gpio_init(MUX_READ_B);
	gpio_set_dir(MUX_READ_B, GPIO_IN);
	

	//gpio_set_drive_strength(I2C_SDA0, GPIO_DRIVE_STRENGTH_12MA);
	//gpio_set_drive_strength(I2C_SCL0, GPIO_DRIVE_STRENGTH_12MA);
	//gpio_set_slew_rate(I2C_SDA0, GPIO_SLEW_RATE_FAST);
	// gpio_set_slew_rate(I2C_SCL0, GPIO_SLEW_RATE_FAST);
    
	
	printf("core1: initializing display...\n");
	display = SSD1306_initialize(i2c0,current_state->display_addr);
	current_state->display = display;
	printf("core1: refreshing display.\n");
	clear_display(display);
	printf("core1: display refresh time: %llu (us)\n",current_state->display_refresh_time_us);
	//show_grid();
	

	buttons = current_state->button_state;
	
	// start up the adc for the gain slider
	start_adc();

	// start the ui system
	ui_initialize();
	

	
	gpio_init(RESET_DSP);
	gpio_set_dir(RESET_DSP, GPIO_OUT);
	gpio_pull_down(RESET_DSP);
	//adc_ready = false;
       
	//gpio_set_irq_enabled_with_callback(ADC_READY_PIN, GPIO_IRQ_EDGE_FALL, false, &adc_ready_callback);

	gpio_init(LED_GPIO);
	gpio_set_dir(LED_GPIO,GPIO_OUT);
	gpio_pull_down(LED_GPIO);
	gpio_put(LED_GPIO,1);


	// setup inter-system serial communication
	gpio_set_function(TX_TO_DSP, UART_FUNCSEL_NUM(uart1, TX_TO_DSP));
	gpio_set_function(RX_FROM_DSP, UART_FUNCSEL_NUM(uart1, RX_FROM_DSP));
	uart_init(uart1,115200);
	// wait for the LED testing
	while(current_state->ready==false) {
	    sleep_us(1000);
	}
	gpio_put(LED_GPIO,0);


	//printf("core1: configuring gpio interrupt..\n");
	//gpio_set_irq_enabled(ADC_READY_PIN,GPIO_IRQ_EDGE_FALL, true);
	//gpio_set_irq_callback(&adc_ready_callback);
	printf("core1: entering control loop\n");
	clear_framebuffer();
	inverse_text(3,4,"Cymbolix");
	current_state->threshold_dB=-12;
	
	control_loop();       
    }
}


int setup() {
    
    // assign the current_state pointer to the current machine state 
    set_machine_state(&machine_state);
    
    // allocations for globals...
    text_buffer = (char *) calloc(1024,sizeof(char));
    recv_buffer = (char *) calloc(ENTRY_SIZE,sizeof(char));    
    led_levels = (float *) calloc(AMPLITUDE_PIXEL_COUNT*3, sizeof(float));
    comp_gain = (float *) calloc(AMPLITUDE_PIXEL_COUNT*3, sizeof(float));
    cmd_buffer = (char *) calloc(ENTRY_SIZE,sizeof(char));

    dsp_queue = 0;
    
    build_initial_movement_table();
    //show_movement_list();
    
    // setup the ADS1115 ADC for the slider position sensing...
    ads = initialize_ads1115(i2c1,0x48,40000);

    //allocate button states

    machine_state.button_state = (uint32_t *) calloc(NUM_BUTTONS,sizeof(uint32_t));
    
    
    printf("core0: setting up pio for rotary encoder\n");
    
    // setup rotary controller, assumes two sequential GPIO pins, lower one is clock	
    re = init_rotary_pio(pio1, ROTARY_CLK_PIN, (callback) rotary_up, (callback) rotary_down);

    
    uint8_t a[3]= {0,16,0}; //urgb_u32(0,16,0);
    uint8_t p[3]= {6,3,0}; //  = urgb_u32(6,3,0);
    uint8_t o[3]= {20,0,0};
    avg_led_rgb = initialize_averages(AMPLITUDE_PIXEL_COUNT*3+1, LED_AVG_SIZE);
    
    default_colors.amplitude = a;
    default_colors.peak_amp = p;
    default_colors.overload = o;

    adc_avg = initialize_average(20);
    amplitude_inst = init_ws2812b(pio0,ADDR_LED_GPIO,AMPLITUDE_PIXEL_COUNT);
    
    // I2C Initialisation. Using it at 100Khz.
    // Note: it is usually best to configure the codec here, and then enable it
    //       after starting the I2S clocks, below.
    // default threshold is -12dB
    
    if (MULTICORE) {
	printf("core0: starting core1..\n");
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

    // initialize the motorized slider controls

    machine_state.slider_target_position = machine_state.channel_gain;
    cycle_count = 0;
    
    gpio_init(SLIDER_UP);
    gpio_set_dir(SLIDER_UP,GPIO_OUT);
    gpio_pull_down(SLIDER_UP);

    gpio_init(SLIDER_DOWN);
    gpio_set_dir(SLIDER_DOWN,GPIO_OUT);
    gpio_pull_down(SLIDER_DOWN);

    // setup rotary controller, assumes two sequential GPIO pins, lower one is clock	
    //re = init_rotary_pio(pio1, ROTARY_CLK_PIN, (callback) rotary_up, (callback) rotary_down);

    
    return 0;
}

int main() {
    set_sys_clock_khz(132000, true); 
    stdio_init_all();
    machine_state.ready = false;
    printf("\n\nChannel Controller\n");
    printf("System Clock: %lu\n", clock_get_hz(clk_sys));

    // set up the environment...
    setup();
    // and run the main loop in core 0

    // perform test light pattern - ensure we have enough power
    // pixels should be a white (all rgb on)
    for(int i = 0; i < AMPLITUDE_PIXEL_COUNT; i++) {
	for (int j = 0; j < AMPLITUDE_PIXEL_COUNT; j++) {
	    if (j==i) put_pixel(amplitude_inst,urgb_u32(40,40,40));
	    else put_pixel(amplitude_inst,0);		
	}
	sleep_ms(40);
    }
    for(int i = AMPLITUDE_PIXEL_COUNT-1; i >=0; i--) {
	for (int j = 0; j < AMPLITUDE_PIXEL_COUNT; j++) {
	    if (j==i) put_pixel(amplitude_inst,urgb_u32(40,40,40));
	    else put_pixel(amplitude_inst,0);		
	}
	sleep_ms(40);
    }	
    for(int i = 0; i < AMPLITUDE_PIXEL_COUNT; i++) {
	put_pixel(amplitude_inst,0);
    }

    
    machine_state.ready = true;
    	
    
    core0_run_loop();	
    
    return 0;
}
