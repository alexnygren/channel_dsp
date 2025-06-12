
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <math.h>
#include <unistd.h>
#include <stdlib.h>
#include "common.h"
#include "hardware/i2c.h"
#include "ssd1306_i2c_driver.h"
#include "ui.h"
#include "channel.h"
#include "controller.h"

typedef enum {
    REFRESH_ALL = 0,
    VALUE_UPDATED = 1,    
    CURRENT = 3,
} PAGE_STATE;

PAGE_STATE p_state = REFRESH_ALL;

uint16_t button_offset = 0;

uint64_t last_update_time = 0;
uint64_t last_press_time = 0;
bool in_display_update = false;

float multiplier = 1;

uint16_t buttons_to_pages[11];

#define METER_SHIFT 1


bool check_if_need_full_refresh() {
    if (current_state->last_external_update_ms+10 > last_update_time) {
	//printf("%s: p_state is now REFRESH_ALL.\n",__FUNCTION__);
	p_state = REFRESH_ALL;
	last_update_time=now_ms();
	return true;
    }
    return false;
}

void render_compressor_state() {
    if (current_state==NULL) return;
    
    if (in_display_update == true) {
	return;
    }
    in_display_update = true;
    button_offset = 30;
    check_if_need_full_refresh();
    if ((p_state == VALUE_UPDATED) || (p_state == REFRESH_ALL)) {
	printf("%s: refreshing display.\n",__FUNCTION__);
	clear_framebuffer();	
	text_at(0,0,"Compressor");
	draw_line(disp,0,12,SSD1306_WIDTH-1,12,true);
	text_at(1,0,"Thr:");
	text_at(2,0,"Att:");
	text_at(3,0,"Mkp:");
	text_at(4,0,"Rel:");
	char txt[20];
	sprintf(txt,"%.1fdB",current_state->threshold_dB);
	text_at(1,6,txt);
	sprintf(txt,"%.1dms",(int) current_state->attack_rate_ms);
	text_at(2,6,txt);
	sprintf(txt,"%.1fdB",current_state->makeup_db);
	text_at(3,6,txt);
	sprintf(txt,"%.1dms",(int) current_state->release_rate_ms);	
	text_at(4,6,txt);
	if (current_state->compressor_on) {
	    inverse_text(0,16,"ON");
	} else text_at(0,16,"OFF");
	p_state = CURRENT;
    }
    in_display_update = false;

    return;
}


void render_gate_state() {
    if (current_state==NULL) return;

    if (in_display_update == true) {
	return;
    }
    in_display_update = true;
    button_offset = 20;
    check_if_need_full_refresh();
    
    if ((p_state == VALUE_UPDATED) || (p_state == REFRESH_ALL)) {
	p_state = CURRENT;
	//printf("%s: refreshing display.\n",__FUNCTION__);
	clear_framebuffer();	
	text_at(0,0,"Gate");
	draw_line(disp,0,12,SSD1306_WIDTH-1,12,true);

	text_at(1,0,"Thr:");
	text_at(2,0,"Att:");
	text_at(3,0,"Hld::");
	text_at(4,0,"Rel:");
	char txt[20];
	sprintf(txt,"%.1fdB",current_state->gate_threshold_dB);
	text_at(1,6,txt);
	sprintf(txt,"%.1dms",(int) current_state->gate_attack_ms);
	text_at(2,6,txt);
	sprintf(txt,"%.1dms",(int) current_state->gate_hold_ms);
	text_at(3,6,txt);
	sprintf(txt,"%.1fdB",current_state->gate_release_ms);
	text_at(4,6,txt);
	if (current_state->gate_active) {
	    inverse_text(0,16,"ON");
	} else text_at(0,16,"OFF");
	
    }
    in_display_update = false;

    return;
}




void render_channel_state() {
    if (current_state==NULL) return;
    if (in_display_update == true) {
	return;
    }
    in_display_update = true;
    char txt[26];
    check_if_need_full_refresh();
    if (current_state->slider_update_ms > last_update_time) {
	sprintf(txt,"%.1fdB  %.2f",current_state->slider_db,current_state->slider_percent);
	clear_text(2,8,13);
	text_at(2,8,txt);
	last_update_time = now_ms();
    }
    if ((p_state == VALUE_UPDATED) || (p_state == REFRESH_ALL)) {
	//printf("%s: refreshing display.\n",__FUNCTION__);
	clear_framebuffer();	
	text_at(0,0,"Channel");
	draw_line(disp,0,12,SSD1306_WIDTH-1,12,true);

	sprintf(txt,"%d",current_state->channel_number);
	text_at(0,9,txt);    
	if (current_state->gate_active) inverse_text(1,0,"Gate");
	else text_at(1,0,"Gate");
	if (current_state->compressor_on) inverse_text(1,6,"Comp");
	else text_at(1,6,"Comp");
	if (current_state->muted) inverse_text(1,12,"Mute");
	else text_at(1,12,"Mute");
	text_at(2,0,"Gain:");
	sprintf(txt,"%.1fdB  %.2f",current_state->slider_db,current_state->slider_percent);
	text_at(2,8,txt);
	text_at(3,0,"Trim:");
	sprintf(txt,"%.2fdB",current_state->input_trim_gain);	    
	text_at(3,8,txt);
	p_state = CURRENT;
    }
    in_display_update = false;


}

void render_control() {

    if (active_page == NULL) {
	printf("%s: no active page, cannot render.\n",__FUNCTION__);
	p_state = CURRENT;
	return;
    }
    if (!active_page->initialized) {
	printf("%s: not initialized!\n",__FUNCTION__);
	return;
    }

    if (p_state == CURRENT) {
	return;
    }
    if (in_display_update == true) {
	return;
    }
    in_display_update = true;
    page_control *ctl = NULL;
    if (active_page->active_control) {
	ctl = active_page->active_control;	
    }
    if (ctl==NULL) {
	printf("%s: active page: %s has no active control set. cannot render.\n",__FUNCTION__,active_page->name);
	p_state = CURRENT;
	return;  // nothing to render
    }
    printf("%s: active control: %s  p_state: %d val: %f mval: %f\n",__FUNCTION__,ctl->name,p_state, ctl->value, *(ctl->machine_value));
    // if the machine's current state has been updated..
    // externally within the last 10ms or more, set our state
    if (check_if_need_full_refresh()) {
	printf("%s: system update: refreshing: %s\n",__FUNCTION__,ctl->name);	
    }
    
    if (p_state == REFRESH_ALL) {
	// clear the frame buffer
	clear_framebuffer();	
	//inverse_text(0,0,current_state->channel_name);
	last_update_time = now_ms();	
	text_at(0,0,active_page->name);
		    
	ctl->value_updated = true;
	ctl->graph_updated = true;
	ctl->render(ctl);    
    } else if (p_state == VALUE_UPDATED) {
	last_update_time = now_ms();
	ctl->value_updated = true;
	ctl->render(ctl);
    }
    // UI page state should be current with the system state
    p_state = CURRENT;
    in_display_update = false;
}




// on the rotary interupt just update the value
// and the change flag, the ui will be updated
// asynchronously.

void on_rotary_up() {
    if (active_page && active_page->active_control) {	
	page_control *ctl = active_page->active_control;
	uint32_t *buttons = get_button_state();
	printf("up: %s\n",ctl->name);
	if (buttons[10]>0)  multiplier=10;
	else multiplier=1;
	*(ctl->machine_value) = min(*(ctl->machine_value)+(ctl->adjust_increment*multiplier),ctl->high_limit);
	p_state = VALUE_UPDATED;
    }   
}

void on_rotary_down() {
   if (active_page && active_page->active_control) {
	page_control *ctl = active_page->active_control;
	uint32_t *buttons = get_button_state();
	if (buttons[10]>0)  multiplier=10;
	else multiplier=1;
	printf("down: %s\n",ctl->name);
	*(ctl->machine_value) = max(*(ctl->machine_value)-(ctl->adjust_increment*multiplier),ctl->low_limit);
	//*(ctl->machine_value) -= ctl->adjust_increment;
	p_state = VALUE_UPDATED;
    }
}


void set_active_page(ui_page *page) {
    if (page) {
	printf("%s: going to page: %s [%d]\n",__FUNCTION__,page->name,page->id);	    
	active_page = page;
	p_state = REFRESH_ALL;
    } else {
	printf("%s: null page passed.\n",__FUNCTION__);
    }
}



void on_button_release(uint16_t button_idx) {

    // the rotary encoder, when pressed, and when this event is not triggered
    // by its release, acts as a kind of a shift key, which, in combination
    // with other buttons, turns on/off the compressor and gate and other
    // special functions

    uint32_t *buttons = get_button_state();
    
    if (button_idx != 10 && buttons[10]>0) {
	switch(button_idx) {
	case 5:
	    set_gate_on(!current_state->gate_active);
	    break;
	case 6:
	    set_compressor_on(!current_state->compressor_on);	
	    break;
	}
	p_state = REFRESH_ALL;
	multiplier = 2;
	return;
    }
    
    if (multiplier == 1) {
	if (button_idx < 4) {  // top row buttons
	    button_idx+=button_offset;
	}
	ui_page *goto_page = page_by_id(button_idx);    
	set_active_page(goto_page);
    }
    multiplier = 1;
}

ui_page *setup_control_page(ui_page *page, page_control *ctl) {
    page->active_control = ctl;
    page->render = render_control;
    page->on_rotary_up = on_rotary_up;
    page->on_rotary_down = on_rotary_down;
    page->on_button_release = on_button_release;
    page->active_control = (void *) ctl;
    return page;
}

ui_page *initialize_channel_pages() {
    p_state = REFRESH_ALL;
    
    ui_page *last_page = 0;
    ui_page *page = create_page("Channel");
    ui_page *start_page = page;    // channel is the opening page

    button_offset = 0;
    page_control *ctl;
    page->render = render_channel_state;
    page->on_button_release = on_button_release;
    page->initialized = true;
    page->id = 10;   // associate with the press of the rotary encoder 
    last_page = page;

    page = create_page("Gate");
    last_page->next = page;
    page->prior = last_page;
    page->render = render_gate_state;
    page->on_button_release = on_button_release;
    page->id = 5;
    page->initialized = true;
    page->prior = last_page;
    last_page = page;
        
    page = create_page("Compressor");
    last_page->next = page;
    page->prior = last_page;
    page->render = render_compressor_state;
    page->on_button_release = on_button_release;
    page->id = 6;    // first button on the bottom [5 6 7 8]
    page->initialized = true;
    page->prior = last_page;
    last_page = page;
    
    // create the control pages

    page = create_page("Gate Threshold");
    page->id = 20;
    last_page->next = page;
    page->prior = last_page;  
    ctl = horizontal_db_meter(12,&current_state->gate_threshold_dB,GATE_LOW_TH_LIMIT,GATE_HIGH_TH_LIMIT,"Gate Threshold",0.2);
    ctl->sync = set_gate_threshold;
    set_db_meter_display_range(ctl,-20,0); // 20dB of visible range
    shift_db_meter_display_range(ctl,0,true); // center to the value between min_range
    setup_control_page(page,ctl);
    page->initialized = true;
    last_page = page;

    page = create_page("Gate Attack");
    page->id = 21;
    last_page->next = page;
    page->prior = last_page;  
    ctl = millisecond_control(12, &current_state->gate_attack_ms,0,1000,"Gate Attack",1);
    ctl->sync = set_gate_attack;
    set_millis_display_range(ctl,0,100);
    shift_millis_display_range(ctl,0,true);
    setup_control_page(page,ctl);
    page->initialized = true;
    last_page = page;

    page = create_page("Gate Hold");
    page->id = 22;
    last_page->next = page;
    page->prior = last_page;  
    ctl = millisecond_control(12, &current_state->gate_hold_ms,1,3000,"Gate Hold",10);
    ctl->sync = set_gate_hold;
    set_millis_display_range(ctl,0,250);
    shift_millis_display_range(ctl,0,true);
    setup_control_page(page,ctl);
    page->initialized = true;
    last_page = page;

    page = create_page("Gate Release");
    page->id = 23;
    last_page->next = page;
    page->prior = last_page;  
    ctl = millisecond_control(12, &current_state->gate_attack_ms,1,3000,"Gate Release",1);
    ctl->sync = set_gate_release;
    set_millis_display_range(ctl,0,100);
    shift_millis_display_range(ctl,0,true);
    setup_control_page(page,ctl);
    page->initialized = true;
    last_page = page;


    page = create_page("Compressor Threshold");
    page->id = 30;
    last_page->next = page;
    page->prior = last_page;  
    ctl = horizontal_db_meter(12,&current_state->threshold_dB,COMP_LOW_TH_LIMIT,COMP_HIGH_TH_LIMIT,"Compressor Threshold",0.2);
    ctl->sync = set_compressor_threshold;
    set_db_meter_display_range(ctl,-20,0); // 20dB of visible range
    shift_db_meter_display_range(ctl,0,true); // center to the value between min_range
    setup_control_page(page,ctl);
    page->initialized = true;
    last_page = page;
        
    page = create_page("Compressor Attack");
    page->id = 31;
    last_page->next = page;
    page->prior = last_page;  
    ctl = millisecond_control(12, &current_state->attack_rate_ms,0,250,"Compressor Attack",1);
    ctl->sync = set_compressor_attack;
    set_millis_display_range(ctl,0,100);
    shift_millis_display_range(ctl,0,true);
    setup_control_page(page,ctl);
    page->initialized = true;
    last_page = page;
    
    
    page = create_page("Compressor Makeup");
    page->id = 32;
    last_page->next = page;
    page->prior = last_page;  
    ctl = horizontal_db_meter(12,&current_state->makeup_db,0,24,"Compressor Makeup",0.2);
    ctl->sync = set_compressor_makeup;
    set_db_meter_display_range(ctl,0,20); // 20dB of visible range
    shift_db_meter_display_range(ctl,0,true); // center to the value between min_range
    setup_control_page(page,ctl);
    page->initialized = true;
    last_page = page;

    page = create_page("Compressor Release");
    page->id = 33;
    last_page->next = page;
    page->prior = last_page;  
    ctl = millisecond_control(12, &current_state->release_rate_ms,10,1000,"Compressor Release",10);
    ctl->sync = set_compressor_release;
    set_millis_display_range(ctl,0,250);
    shift_millis_display_range(ctl,0,true);
    setup_control_page(page,ctl);    
    page->initialized = true;
    last_page = page;
    
    
    
    active_page = page;
    printf("%s: initialized.\n",__FUNCTION__);
    p_state = REFRESH_ALL;

    return start_page;
}

