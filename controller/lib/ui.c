

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
#include "monospaced_10.h"
#include "bits8.h"

display_update *display_updates = NULL;  // global pointer to a linked list of updates

SSD_i2c_display *disp;


ui_page *pages = 0;

ui_page *active_page;

uint32_t q_id = 100;


void queue_to_display(struct render_area* area) {
        
    if (display_updates == NULL) {
	display_updates = (display_update *) calloc(sizeof(display_update),1);
	display_updates->prior = NULL;
	display_updates->next = NULL;
    } else {
	display_update *prior = display_updates;
	// create the next display update
	display_updates->next = (display_update *) calloc(sizeof(display_update),1);
	// move there...
	display_updates=display_updates->next;
	display_updates->prior = prior;
	display_updates->next = NULL;
    }
    
    
    display_updates->area = (struct render_area *) calloc(sizeof(struct render_area),1);
    display_updates->area->id = area->id;
    display_updates->area->start_col = area->start_col;
    display_updates->area->end_col = area->end_col;
    display_updates->area->start_page = area->start_page;
    display_updates->area->end_page = area->end_page;

    
}

void enqueue_display_operation(struct render_area *update_area) {
    if (update_area->start_page > update_area->end_page) {
	update_area->start_page = 0;
    }
    if (update_area->start_col > 120) {
	update_area->start_col = 120;
    }
    update_area->start_col = clamp(update_area->start_col,0,120);
    update_area->start_page = clamp(update_area->start_page,0,7);
    update_area->end_page = clamp(update_area->end_page,1,7);    
    queue_to_display(update_area);
   
}



void flush_to_display() {

    if (display_updates != NULL) {
	// if we have display_updates, find the first one	
	display_update *current = display_updates;
	display_update *transfered = NULL;
	uint16_t num_updates = 1;
	uint16_t update_count = 1;
	bool max_mode = true;
	// break the chain, use current from now on.  any new requests
	// will be in a new list.
	display_updates = NULL;
	
	struct render_area max_area = {
           start_col: current->area->start_col,
	   end_col: current->area->end_col,
	   start_page: current->area->start_page,
	   end_page: current->area->end_page
	};

	while(current->prior != NULL) {	    
	    current = current->prior;
	    if (max_mode) {
		if (current->area->start_col < max_area.start_col) max_area.start_col = current->area->start_col;
		if (current->area->end_col > max_area.end_col) max_area.end_col = min(SSD1306_WIDTH-1,current->area->end_col);
		if (current->area->start_page < max_area.start_page) max_area.start_page = current->area->start_page;
		if (current->area->end_page > max_area.end_page) max_area.end_page = current->area->end_page;
	    }
	    num_updates++;
	}

	
	
	// current should now be at the first element in the list
	// now move forward, detach and transfer
	if (max_mode) {
	    
	    ssd_render(disp,&max_area);
	    
	}
	while(current != NULL && update_count <= num_updates) {	    
	    // get the area and update the display
	    // first, remove the back pointer 
	    current->prior = NULL;
	    // transfer the area to the display
	    //printf("fl: sc=%d,ec=%d,sp=%d,ep=%d  id: %ld\n",current->area->start_col,current->area->end_col,current->area->start_page,current->area->end_page,current->area->id);
	    if (!max_mode) ssd_render(disp, current->area);
	    transfered = current;
	    // move current foward if possible
	    current=current->next;
	    current->prior = NULL;

	    // free the transfered elements 
	    free(transfered->area);	    

	    free(transfered);
	    update_count++;
	}

    }
    
    
}

void pq(struct render_area *area,const char *text) {
    printf("area: [%s] sc=%d,ec=%d,sp=%d,ep=%d  id: %ld\n",text,area->start_col,area->end_col,area->start_page,area->end_page, area->id);
}

void clear_framebuffer() {
    memset(disp->buf,0,SSD1306_BUF_LEN);
    struct render_area update_area = {
    start_col: 0,
    end_col: SSD1306_WIDTH,
    start_page: 0,
    end_page: SSD1306_NUM_PAGES,
    id: ++q_id
    };
    //pq(&update_area,__FUNCTION__);
    enqueue_display_operation(&update_area);
}

void text_at(uint8_t row, uint8_t column, const char *str) {
    row = clamp(row,0,5);
    uint8_t y = row*ROW_HEIGHT;
    uint8_t page = (y / 8);
    column = clamp(column,0,(int)SSD1306_WIDTH / COL_WIDTH);
    write_text(disp,column*COL_WIDTH,y ,str);
    struct render_area update_area = {
    start_col : max(0, (column * COL_WIDTH) - 2),
      end_col : min(SSD1306_WIDTH-COL_WIDTH,(column * COL_WIDTH)+(strlen(str) * COL_WIDTH)),
      start_page : page, // TODO: convert from ROWS to PAGES
      end_page : page+2,
      id: ++q_id
    };
    //pq(&update_area,__FUNCTION__);
    enqueue_display_operation(&update_area);
     //printf("text_at: r=%d c=%d t='%s' sp=%d ep=%d\n",row,column,str,update_area.start_page, update_area.end_page);
    //ssd_render(display,&update_area);
}

void clear_text(uint8_t row, uint8_t column, uint8_t num_chars) {
    row = clamp(row,0,5);
    column = column*COL_WIDTH;
    column = clamp(column,0,SSD1306_WIDTH);
    uint8_t y = row*ROW_HEIGHT;    
    erase_area(column, y, num_chars*COL_WIDTH,13);
}

void clear_text_old(uint8_t row, uint8_t column, uint8_t num_chars) {
    row = clamp(row,0,5);
    column = clamp(column,0,SSD1306_WIDTH / COL_WIDTH);
    uint8_t y = row*ROW_HEIGHT;
    uint8_t page = (y / 8);
    int byte_index = (page * SSD1306_WIDTH)+(column*COL_WIDTH);
    uint8_t y_bit_offset = y % 8;
    uint8_t page_mask = 255;
    //page_mask |= 1 << y_bit_offset;
    if (y_bit_offset == 0) {
	page_mask = 0;
    } else {
	for (int i = 7; i>=y_bit_offset; i--) {
	    page_mask &= 0 << i;
	}
    }
    // printf("clear_text: y = %d, page=%d, y_bit_offset = %d  ",y, page, y_bit_offset);
    // printf("\n");
    // first page
    for (uint c = 0; c < num_chars*COL_WIDTH; c++) {
	disp->buf[byte_index+c]=disp->buf[byte_index+c] & page_mask;
    }
    
    if (page < 7) {	
	page_mask = 255;
	y+=ROW_HEIGHT-1;
	y_bit_offset = (y % 8);
	uint8_t page_a = (y / 8);
	byte_index = (page_a * SSD1306_WIDTH)+(column*COL_WIDTH);
	if (y_bit_offset == 0) {
	    page_mask = 0;
	} else {
	// page_mask |= 1 << (7 - y_bit_offset);
	    for (int i = 0; i <= 7; i++) {
		if (i <= y_bit_offset) {
		    page_mask &= 0 << i;
		}
	    }
	}
	//printf("clear_text: y = %d, page=%d, y_bit_offset = %d  ",y, page_a, y_bit_offset);
	//printf("\n");
	for (uint c = 0; c < num_chars*COL_WIDTH; c++) {
	    disp->buf[byte_index+c]=disp->buf[byte_index+c] & page_mask;
	}
    }
       
    struct render_area update_area = {
      start_col : max(0, (column * COL_WIDTH) - 2),
      end_col : min(SSD1306_WIDTH-1,(column * COL_WIDTH)+(num_chars * COL_WIDTH)),
      start_page : page,
      end_page : min(page+1,7),
      id: ++q_id
    };
    //pq(&update_area,__FUNCTION__);
    enqueue_display_operation(&update_area);
}


void inverse_text(uint8_t row, uint8_t column, char *str) {
    row = clamp(row,0,5);
    column = clamp(column,0,SSD1306_WIDTH / COL_WIDTH);
    uint8_t y = row*ROW_HEIGHT;
    uint8_t page = (y / 8);
    uint8_t num_chars = strlen(str);
    OLEDDISPLAY_COLOR orig_color = disp->color;
    int byte_index = (page * SSD1306_WIDTH)+(column*COL_WIDTH);
    uint8_t y_bit_offset = y % 8;
    uint8_t page_mask = 0;
    //page_mask |= 1 << y_bit_offset;
    if (y_bit_offset == 0) {
	page_mask = 255;
    } else {
	for (int i = 7; i>=y_bit_offset; i--) {
	    page_mask |= 1 << i;
	}
    }
    // first page
    for (uint c = 0; c < (num_chars*COL_WIDTH)+1; c++) {
	disp->buf[byte_index+c]=disp->buf[byte_index+c] | page_mask;
    }
    
    if (page < 7) {
	
	page_mask = 0;
	y+=ROW_HEIGHT-1;
	y_bit_offset = (y % 8);
	uint8_t page_a = (y / 8);
	byte_index = (page_a * SSD1306_WIDTH)+(column*COL_WIDTH);
	if (y_bit_offset == 0) {
	    page_mask = 255;
	} else {
	    for (int i = 0; i <= 7; i++) {
		if (i <= y_bit_offset) {
		    page_mask |= 1 << i;
		}
	    }
	}
	for (uint c = 0; c < (num_chars*COL_WIDTH)+1; c++) {
	    disp->buf[byte_index+c]=disp->buf[byte_index+c] | page_mask;
	}
    }
  
    
    struct render_area update_area = {
      start_col : max(0, (column * COL_WIDTH) - 2),
      end_col : min(SSD1306_WIDTH-1,(column * COL_WIDTH)+(num_chars * COL_WIDTH)+1),
      start_page : page,
      end_page : min(page+1,7),
      id: ++q_id
    };
    //pq(&update_area,__FUNCTION__);
    enqueue_display_operation(&update_area);

    disp->color = 2;
    text_at(row, column,str);
    disp->color = orig_color;
    
}

// cache the bitmask by bit offset 
uint8_t north[8]={
    0b00000000,
    0b00000001,
    0b00000011,
    0b00000111,
    0b00001111,
    0b00011111,
    0b00111111,
    0b01111111 };

uint8_t south[8]={
    0b11111111,
    0b11111110,
    0b11111100,
    0b11111000,
    0b11110000,
    0b11100000,
    0b11000000,
    0b10000000 };


void erase_area(uint8_t x, uint8_t y, uint8_t width, uint8_t height) {
    x = clamp(x,0,127);
    y = clamp(y,0,63);
    
    if (x+width > SSD1306_WIDTH) {
	width = SSD1306_WIDTH - x;
    }
    if (y+height > SSD1306_HEIGHT) {
	height = SSD1306_HEIGHT - y;
    }
    uint8_t y_start = y;
    uint8_t start_page = (y_start/8);
    uint8_t bits_offset = (y_start % 8);
    uint8_t start_mask = north[bits_offset];
    uint8_t current_page = start_page;
    uint8_t end_mask = south[(y_start + height) % 8];    
    uint8_t end_page = ((y_start+height)/8);
    int byte_index = (start_page * SSD1306_WIDTH)+x;
    uint8_t lines_remaining = height;
    //printf("%s: y_start: %d  sp: %d  h: %d ep: %d  sm:",__FUNCTION__,y_start,start_page,height, end_page);
    //bits8(start_mask);
    //printf("  em:");
    //bits8(end_mask);
    //printf("\n");
    // if our height is less than 8, then or together and apply to framebuffer
    if (end_page == start_page) {
	uint8_t page_mask = start_mask|end_mask;
	for (uint c = 0; c < width; c++) {
	    disp->buf[byte_index+c]=disp->buf[byte_index+c] & page_mask;
	}
	lines_remaining = 0;
    } else {	
	// the height spans multiple pages, so apply the first mask to start
	for (uint c = 0; c < width; c++) {
	    disp->buf[byte_index+c]=disp->buf[byte_index+c] & start_mask;
	}
	// align lines_remaining and y to the start of the next page..
	lines_remaining-=(8-bits_offset);
	y+=(8-bits_offset);
	current_page = (y / 8);
	// current page should be one greater than the start page
	// update our byte index to align to the current page and offset
	byte_index = (current_page * SSD1306_WIDTH)+x;
	//printf("%s: + cp:%d y: %d lr:%d bi:%d\n",__FUNCTION__,current_page,y,lines_remaining,byte_index);
	
	while (lines_remaining>SSD1306_PAGE_HEIGHT) {
	    for (uint c = 0; c < width; c++) {
		disp->buf[byte_index+c]= 0;  // whole page = 0
	    }
	    lines_remaining-=SSD1306_PAGE_HEIGHT;
	    y+=SSD1306_PAGE_HEIGHT;
	    byte_index = (current_page * SSD1306_WIDTH)+x;
	}
	if (lines_remaining>0) {
	    // apply the end mask to the end page
	    byte_index = (end_page * SSD1306_WIDTH)+x;
	    //printf("%s: F cp:%d y: %d lr:%d bi:%d  ep:%d\n",__FUNCTION__,current_page,y,lines_remaining,byte_index,end_page);	    
	    for (uint c = 0; c < width; c++) {
		disp->buf[byte_index+c]=disp->buf[byte_index+c] & end_mask;
	    }    
	}
	
    }
    // now create the update structure
    struct render_area update_area = {
      start_col : max(0, x),
      end_col : min(SSD1306_WIDTH-1,x+width),
      start_page : start_page,
      end_page : min(end_page+1,7),
      id: ++q_id,
    };
    //pq(&update_area,__FUNCTION__);
    enqueue_display_operation(&update_area);
    
}




void dump_display() {
    printf("\nFrame Buffer:\n");
    const int bytes_per_row = SSD1306_WIDTH;
    int page_num;
    for (int y = 0; y < SSD1306_HEIGHT; y++) {
	page_num = (int) floorf(y / 8);
	printf("%3d: %d %.4d: ",y, page_num,page_num*bytes_per_row);
	for (int x = 0; x < SSD1306_WIDTH; x++) {
	    int byte_idx = (y / 8) * bytes_per_row + x;
	    uint8_t byte = disp->buf[byte_idx];
	    if (((byte >> (y % 8)) & 1) == 1) {
		printf("*");
	    } else {
		if (x%COL_WIDTH==0 && y%8 ==0) {
		    printf(":");
		} else if (x%COL_WIDTH==0) {
		    printf(",");
		} else {
		    printf(".");
		}
	    }
	}       
	printf(" %d\n",(page_num*bytes_per_row)+bytes_per_row-1);
    }
    printf("\n");
}

uint8_t display_page_for_y(uint8_t y) {
    return (uint8_t) floorf(y/8);
}


void calculate_dbm_ratios(page_control *dbm) {
    dbm->min_ratio = dB_to_ratio(dbm->min);
    dbm->max_ratio = dB_to_ratio(dbm->max);
    dbm->value_ratio = dB_to_ratio(dbm->value);
    dbm->graph_updated = true;  // trigger a rerender
    dbm->value_updated = true;
}

// renders a db meter display with the center point as the db_value
page_control* horizontal_db_meter(uint8_t y_pos,float *machine_value,float min,float max, char *name, float adjust_increment) {
    page_control *ctl = (page_control *) calloc(1, sizeof(page_control));
    ctl->y = y_pos;
    ctl->x = 0;
    ctl->width=SSD1306_WIDTH-6;
    ctl->vertical = false;
    ctl->low_limit = min;
    ctl->high_limit = max;
    ctl->adjust_increment = adjust_increment;
    ctl->min = min;
    ctl->max = max;
    ctl->min_range = 18.0;
    ctl->machine_value = machine_value;
    ctl->value = *(machine_value);
    calculate_dbm_ratios(ctl);
    ctl->edit_mode = false;
    ctl->draw_graph_in_edit_mode = true;
    ctl->draw_graph_in_view_mode = false;
    ctl->start_page = display_page_for_y(ctl->y);
    ctl->end_page = ctl->start_page+2;
    ctl->render = render_db_meter;
    ctl->value_as_line = false;
    ctl->name = name;
    ctl->graph_updated = true;
    ctl->display = true;
    ctl->value_updated = true;
    printf("%s: %s: value=%.2f machine_value=%.2f %.4f  min:%.4f max:%.4f\n",__FUNCTION__,ctl->name,ctl->value, *(machine_value),ctl->value_ratio, ctl->min_ratio, ctl->max_ratio);
    return ctl;
}

void set_db_meter_display_range(page_control *dbm, float low, float high) {
    high = min(high,dbm->high_limit);
    low = max(low,dbm->low_limit);    
    dbm->min=low;
    dbm->max=high;       
    calculate_dbm_ratios(dbm);
    //printf("%s: val=%.3f min=%.3f max=%.3f min_ratio=%.3f max_ratio=%.3f lim_low=%.3f, lim_high=%.3f\n",
    //       __FUNCTION__, dbm->value, dbm->min, dbm->max, dbm->min_ratio, dbm->max_ratio, dbm->low_limit, dbm->high_limit);
}



void shift_db_meter_display_range(page_control *dbm, float amount, bool center) {
    float high = dbm->max;
    float low = dbm->min;
    float value = dbm->value;

    
    if (center) {
	float dist = dbm->min_range/2;
	// get the distance to each end from the current value
	high = value + floorf((dist / 2));	
	low = value - floorf(dist * 1.5);
	//low = high - dbm->min_range;
	if (high > dbm->high_limit) {
	    high = dbm->high_limit;
	    low = high - dbm->min_range;
	} else if (low < dbm->low_limit) {
	    low = dbm->low_limit;
	    high = low + dbm->min_range;	    
	}
    } else {
	high = min(high+amount,dbm->high_limit);
	low = min(low+amount,high-6.0);
	if ((high - low) < dbm->min_range) {
	    if (amount > 0) {
		low = high - dbm->min_range;	
	    } else if (amount < 0) {
		high = low + dbm->min_range;
	    }
	}
    }
    dbm->max = high;
    dbm->min = low;
    calculate_dbm_ratios(dbm);
    // printf("%s: val=%.3f min=%.3f max=%.3f min_ratio=%.3f max_ratio=%.3f lim_low=%.3f, lim_high=%.3f\n",
    //	   __FUNCTION__, dbm->value, dbm->min, dbm->max, dbm->min_ratio, dbm->max_ratio, dbm->low_limit, dbm->high_limit);
}

void destroy_db_meter(page_control *dbm) {
    free(dbm);
}


void destroy_control(page_control *control) {
    free(control);
}


void set_db_meter_display_mode(page_control *dbm, bool state) {
    dbm->display = state;
    if (dbm->display) {
	render_db_meter(dbm);	
    } else {
	erase_area(dbm->x, dbm->y,SSD1306_WIDTH, 32);
	struct render_area update_area = {
	   start_col : 0,
	   end_col : SSD1306_WIDTH,
	   start_page : dbm->start_page,
	   end_page : display_page_for_y(dbm->y+32),
	   id: ++q_id
	};
    //pq(&update_area,__FUNCTION__);
    enqueue_display_operation(&update_area);

    }
}



void render_db_meter_graph(page_control *dbm) {
    uint8_t xpos = 0;
    float ratio = 0;
    uint8_t width = dbm->width;
    float max_ratio = dbm->max_ratio;
    float min_ratio = dbm->min_ratio;
    uint8_t last_number_x = 255;
    uint8_t last_tick = 255;
    int round_db;
    char txt[8];


    if (dbm->display==false) {
	return;
    }
    
    dbm->graph_updated = false;
    
    erase_area(dbm->x, dbm->y,SSD1306_WIDTH, 16);
    
    for(float db=floorf(dbm->max); db>dbm->min; db-=1) {
	ratio = dB_to_ratio(db);
	xpos = (uint32_t) map_range(ratio,min_ratio,max_ratio,0,width);
	//printf("%s: db: %.2f  ratio: %.4f xpos=%d\n",__FUNCTION__,db,ratio,xpos);
	draw_line(disp, xpos, dbm->y+12,xpos, dbm->y+14, true);
	round_db=(int) floorf(db);
	switch (round_db) {
	case 6:
	case 3:
	case 0:
	    if ((last_number_x - xpos) > 15) {
		sprintf(txt,"%.0f",db);	    
		write_text(disp,xpos-3,dbm->y,txt);
		last_number_x = xpos;
	    }
	    if ((last_tick - xpos) > 1) {
		draw_line(disp,xpos,dbm->y+11,xpos,dbm->y+12,true);
		last_tick = xpos;
	    }
	    break;
	case -3:
	case -6:
	case -9:
	case -12:	    
	case -18:	   
	case -24:
	case -28:	   
	case -36:
	case -40:
	case -48:
	    if ((last_number_x - xpos) > 17) {		
		sprintf(txt,"%d",round_db);
		write_text(disp,xpos-9,dbm->y,txt);
		last_number_x = xpos;
	    }
	    if ((last_tick - xpos) > 0) {
		draw_line(disp,xpos,dbm->y+11,xpos,dbm->y+12,true);
		last_tick = xpos;
	    }
	    break;	
	}
    }


    
    struct render_area update_area = {
      start_col : 0,
      end_col : SSD1306_WIDTH,
      start_page : dbm->start_page,
      end_page : dbm->end_page,
      id: ++q_id
    };
    //pq(&update_area,__FUNCTION__);
    enqueue_display_operation(&update_area);

    
    // show the field name

    /*
    erase_area(0, dbm->y+19, 40, 80);
    write_text(disp, 0, dbm->y+19, dbm->name);
    struct render_area update_text_area = {
      start_col : 0,
      end_col : 80,
      start_page : display_page_for_y(dbm->y+19),
      end_page :  display_page_for_y(dbm->y+32),
      id: ++q_id
    };
    //pq(&update_area,__FUNCTION__);
    enqueue_display_operation(&update_text_area);
    */
}



void render_db_meter_value(page_control *dbm) {
    // draw current value
    uint8_t xpos = 0;
    uint8_t xmax = SSD1306_WIDTH-1;
    uint8_t width = dbm->width;
    if (dbm->vertical) {
	return;  // not implemented yet
    } else {
	dbm->value_updated = false;
	if (dbm->display==false) return;
	xpos = (uint32_t) map_range(dbm->value_ratio,dbm->min_ratio,dbm->max_ratio,0,width);
	xpos = min(xpos,SSD1306_WIDTH-1);
	//printf("%s: xpos=%d\n",__FUNCTION__,xpos);
        
	if (dbm->value_as_line) {
	   
	    draw_line(disp,dbm->x,dbm->y+15,min(dbm->x+xpos,xmax),dbm->y+15,true);
	    draw_line(disp,min(xmax,dbm->x+xpos+1),dbm->y+15,width,dbm->y+15,false);
	    
	    uint8_t start_page = display_page_for_y(dbm->y+15);
	    struct render_area update_area = {
	      start_col : 0,
	      end_col : SSD1306_WIDTH,
	      start_page : start_page,
	      end_page : start_page+1,
	      id: ++q_id
	    };
	    //pq(&update_area,__FUNCTION__);
	    enqueue_display_operation(&update_area);
	} else {	    
	    erase_area(0, dbm->y+15, xmax, 4);
	    draw_line(disp, max(0,dbm->x+xpos), dbm->y+15, min(xmax, dbm->x+xpos), dbm->y+16, true);
	    draw_line(disp, max(0,dbm->x+xpos-1), dbm->y+16, min(dbm->x+xpos+1,xmax), dbm->y+16, true);
	    draw_line(disp, max(0,dbm->x+xpos-2), dbm->y+17, min(dbm->x+xpos+2,xmax), dbm->y+17, true);
	    uint8_t start_page = display_page_for_y(dbm->y+15);
	    struct render_area update_area = {
  	      start_col : max(0,dbm->x+xpos-3),
	      end_col : min(xmax,dbm->x+xpos+3),
	      start_page : start_page,
	      end_page : start_page+1,
	      id: ++q_id
	    };
	    //pq(&update_area,__FUNCTION__);
	    enqueue_display_operation(&update_area);
	    char tval[10];
	    sprintf(tval,"%.2fdB",dbm->value);	    
	    erase_area(18, dbm->y+19, 52, 13);
	    OLEDDISPLAY_TEXT_ALIGNMENT orig_align = disp->text_alignment;
	    ssd_set_alignment(disp, TEXT_ALIGN_RIGHT);
	    write_text(disp, (SSD1306_WIDTH/2)+4, dbm->y+19, tval);
	    ssd_set_alignment(disp, orig_align);
	    struct render_area update_text_area = {
	      start_col : xmax-40,
	      end_col : xmax,
	      start_page : display_page_for_y(dbm->y+19),
	      end_page :  display_page_for_y(dbm->y+32),
	      id: ++q_id
	    };
	    //pq(&update_text_area,__FUNCTION__);
	    enqueue_display_operation(&update_text_area);
	}

    }
}

void render_db_meter(page_control *dbm) {
    if (dbm) {
	if (dbm->value != *(dbm->machine_value)) {
	    dbm->value = *(dbm->machine_value);
	    dbm->value_updated = true;
	}
	printf("%s: val updated: %d  graph: %d\n",__FUNCTION__,dbm->value_updated,dbm->graph_updated);
	if (dbm->value_updated) {
	    dbm->value_ratio = dB_to_ratio(dbm->value);	    
	    dbm->sync(dbm->value);
	    //set_compressor_threshold(dbm->value);
	    shift_db_meter_display_range(dbm,0,true); // center to the value between min_range
	    render_db_meter_graph(dbm);
	    render_db_meter_value(dbm);
	}
	//if (dbm->graph_updated) render_db_meter_graph(dbm);
	//if (dbm->value_updated) render_db_meter_value(dbm);
    }
}


page_control* millisecond_control(uint8_t y_pos, float *machine_value, float min, float max, char *name, float adjust_increment) {
    page_control *ctl = (page_control *) calloc(1, sizeof(page_control));
    ctl->y = y_pos;
    ctl->x = 0;
    ctl->width=SSD1306_WIDTH-6;
    ctl->vertical = false;
    ctl->low_limit = min;
    ctl->high_limit = max;
    ctl->adjust_increment = adjust_increment;
    ctl->min = min;
    ctl->max = max;
    ctl->min_range = 100.0;
    ctl->machine_value = machine_value;
    ctl->value = *(machine_value);
    ctl->edit_mode = false;
    ctl->draw_graph_in_edit_mode = true;
    ctl->draw_graph_in_view_mode = false;
    ctl->start_page = display_page_for_y(ctl->y);
    ctl->end_page = ctl->start_page+2;
    ctl->render = render_millis_control;
    ctl->value_as_line = false;
    ctl->name = name;
    ctl->graph_updated = true;
    ctl->display = true;
    ctl->value_updated = true;
    printf("%s: %s: value=%.2f machine_value=%.2f %.4f  min:%.4f max:%.4f\n",__FUNCTION__,ctl->name,ctl->value, *(machine_value),ctl->value_ratio, ctl->min_ratio, ctl->max_ratio);
    return ctl;
}

void set_millis_display_range(page_control *ctl, float low, float high) {
    high = min(high,ctl->high_limit);
    low = max(low,ctl->low_limit);
    ctl->min=low;
    ctl->max=high;
    ctl->graph_updated = true; // to re-render graphical portion
}


void shift_millis_display_range(page_control *ctl, float amount, bool center) {
    float high = ctl->max;
    float low = ctl->min;
    float value = ctl->value;

    
    if (center) {
	float dist = ctl->min_range/2;
	// get the distance to each end from the current value
	high = value + dist;
	low = value - dist;
	//low = high - ctl->min_range;
	if (high > ctl->high_limit) {
	    high = ctl->high_limit;
	    low = high - ctl->min_range;
	} else if (low < ctl->low_limit) {
	    low = ctl->low_limit;
	    high = low + ctl->min_range;	    
	}
    } else {
	high = min(high+amount,ctl->high_limit);
	low = min(low+amount,high-10.0);
	if ((high - low) < ctl->min_range) {
	    if (amount > 0) {
		low = high - ctl->min_range;	
	    } else if (amount < 0) {
		high = low + ctl->min_range;
	    }
	}
    }
    ctl->max = high;
    ctl->min = low;

}



void render_millis_graph(page_control *ctl) {
    uint8_t xpos = 0;
    int xmax = SSD1306_WIDTH - 2;
    int last_number_x = 0;
    int val = 0;
    char txt[12];
    //int num_ticks=20;
    //int tick_inc = (ctl->max - ctl->min)/num_ticks;
    OLEDDISPLAY_TEXT_ALIGNMENT orig_align = disp->text_alignment;

    
    
    if (ctl->display==false) {
	return;
    }
    ctl->graph_updated = false;
    
    ssd_set_alignment(disp, TEXT_ALIGN_CENTER);
    printf("%s: ctl: x=%d y=%d val=%f min=%0.f max=%0.f  xmax=%d\n",__FUNCTION__,ctl->x,ctl->y,ctl->value, ctl->min,ctl->max,xmax);
    erase_area(ctl->x, ctl->y, SSD1306_WIDTH, 16);
    for (val = ctl->min; val < ctl->max; val+=1) {
	xpos = (int) clamp(map_range(val,ctl->min,ctl->max,2,xmax),0,xmax);
	// printf("%s: xpos=%d val=%d\n",__FUNCTION__,xpos,val);	    
	if ((val % 20) == 0) {
	    draw_line(disp,xpos, ctl->y+11,xpos, ctl->y+14, true);	
	    sprintf(txt,"%d",val);
	    write_text(disp,xpos,ctl->y,txt);
	    last_number_x = xpos;
	} else if ((xpos > last_number_x+5) && (val%10 == 0)) {
	    draw_line(disp,xpos, ctl->y+12,xpos, ctl->y+14, true);
	}		
    }
    ssd_set_alignment(disp, orig_align);
     
    struct render_area update_area = {
      start_col : 0,
      end_col : SSD1306_WIDTH,
      start_page : ctl->start_page,
      end_page : ctl->end_page,
      id: ++q_id
    };
    //pq(&update_area,__FUNCTION__);
    enqueue_display_operation(&update_area);               
}


void render_millis_value(page_control *ctl) {


    int xmax = SSD1306_WIDTH-2;
    int xpos = clamp(map_range(ctl->value,ctl->min,ctl->max,2,xmax),0,xmax);
    char txt[8];
    int y = ctl->y;

    OLEDDISPLAY_TEXT_ALIGNMENT orig_align = disp->text_alignment;

    if (ctl->display==false) {
	return;
    }
    ctl->value_updated = false;

    erase_area(0,y+15,SSD1306_WIDTH,8);
    printf("%s: xpos: %d y+15:%d  xmax: %d \n",__FUNCTION__,xpos,y+15,xmax);
    
    draw_line(disp, xpos, y+15, xpos, y+16, true);
    draw_line(disp, max(0,xpos-1), y+16, min(xpos+1,xmax), y+16, true);
    draw_line(disp, max(0,xpos-2), y+17, min(xpos+2,xmax), y+17, true);

    uint8_t start_page = display_page_for_y(y+15);
    struct render_area update_area = {
      start_col: max(0,xpos-3),   
      end_col: min(xpos+3, xmax),
      start_page: start_page,
      end_page: start_page+1,
      id: ++q_id
    };
    enqueue_display_operation(&update_area);
    erase_area(0,y+18,xmax,15);
    ssd_set_alignment(disp, TEXT_ALIGN_RIGHT);
    sprintf(txt,"%dms",(int) ctl->value);
    write_text(disp,(SSD1306_WIDTH/2)+4,y+19,txt);    
    ssd_set_alignment(disp, orig_align);
    struct render_area update_text_area = {
       start_col : xmax-40,
       end_col : xmax,
       start_page : display_page_for_y(y+19),
       end_page :  display_page_for_y(y+32),
       id: ++q_id
    };
    //pq(&update_text_area,__FUNCTION__);
    enqueue_display_operation(&update_text_area);
}


void render_millis_control(page_control *ctl) {
    if (ctl && ctl->value != *(ctl->machine_value)) {
	ctl->value = *(ctl->machine_value);
	ctl->value_updated = true;
    }    
    printf("%s: val updated: %d  graph: %d\n",__FUNCTION__,ctl->value_updated,ctl->graph_updated);
    
    if (ctl->graph_updated) {
	render_millis_graph(ctl);
    }
    if (ctl->value_updated) {
	ctl->sync(ctl->value);
	shift_millis_display_range(ctl, 0, true);
	render_millis_graph(ctl);
	render_millis_value(ctl);

		    
    }
}

uint32_t *button_state;


uint32_t *get_button_state() {
    return button_state;
}


// duration is how long the button was pressed for on a release event

void button_event(BUTTON button, BUTTON_EVENT action, uint32_t duration) {
    char buf[20];
    if (action==PRESS) {
	printf("ui: button press: %d\n",button);
	sprintf(buf,"B%2d",button);
	text_at(4,0,buf);
	if (active_page && active_page->initialized) {
	    if (active_page->on_button_press) active_page->on_button_press(button);
	}
	//button_state[button]=1;
    } else {
	printf("ui: button release: %d  duration: %ld\n",button, duration);
	clear_text(4,0,4);
	if (active_page && active_page->initialized) {
	    if (active_page->on_button_release) active_page->on_button_release(button);
	}
	button_state[button]=0;
    }
}


void rotary_event(uint8_t direction) {
    if (direction>0) {
	if (active_page && active_page->initialized) {
	    printf("ui: rotary up: %s\n",active_page->name);
	    if (active_page->on_rotary_up) active_page->on_rotary_up();
	}
    } else {
	if (active_page && active_page->initialized) {
	    printf("ui: rotary down: %s\n",active_page->name);
	    if (active_page->on_rotary_down) active_page->on_rotary_down();
	}	
    }
}

void update_machine_state() {
    if (current_state) {
	// get direct pointers to certain things in the machine state structure 
	disp = current_state->display;
	button_state = current_state->button_state;
	
    } else {
	printf("%s: %s: null state passed.\n",__FILE_NAME__,__FUNCTION__);
    }
}


ui_page* create_page(char *name) {
    if (name != NULL) {
	ui_page *page = (ui_page *) calloc(1,sizeof(ui_page));
	int len = strlen(name);
	page->name = str_alloc(len+1);
	strcpy(page->name, name);
	page->initialized = false;
	page->edit_mode = false;
	page->next = 0;
	page->prior = 0;
	page->active_control = NULL;
	return page;
    }
    return NULL;
}

ui_page *page_by_name(char *page_name) {
    if (pages == NULL) {
	return NULL;
    }
    ui_page *page = pages;
    do {
	if (strcmp(page_name,page->name)==0) {
	    return page;	    
	} else {
	    page = page->next;
	}
    } while(page);
    return NULL;
}

ui_page *page_by_id(uint16_t id) {
    if (pages == NULL) {
	return NULL;
    }
    ui_page *page = pages;
    do {
	if (id==page->id) {
	    return page;	    
	} else {
	    page = page->next;
	}
    } while(page);
    return NULL;
}


void ui_initialize() {
    active_page = 0;
    update_machine_state();  // grab local references
    ssd_set_font(disp,Monospaced_plain_10);
    pages = initialize_channel_pages();
    active_page = pages;
    //request_channel_status();
}

void set_ui_needs_update() {
    if (active_page && active_page->initialized && active_page->on_external_update) {
	active_page->on_external_update();
    }
}

void ui_update() {
    if (active_page&&active_page->initialized) {
	active_page->render();	
    }
}



