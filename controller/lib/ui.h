#pragma once

/* User interface routines for managing the front panel elements.
   This includes the display (128x64) and the tactile elements.
   
   These routnes operate a level above the raw display control and
   provide a grid based layout, text operations, update
   queue, and various control operations for the system.

   This also manages user events, such as a what a particular button
   press or release (or combination of buttons) maps to in terms
   of machine state.
   
*/


#include <stdint.h>
#include <common.h>
#include "ssd1306_i2c_driver.h"


// clamp defines - make sure that values are between min and max

#define min(X, Y) ((X) < (Y) ? (X) : (Y))
#define max(X, Y) ((X) > (Y) ? (X) : (Y))
#define clamp(V, X, Y) min(Y,max(X,V))  

#define ROW_HEIGHT 12
#define COL_WIDTH 6

// list of modified (dirty) frame buffer rectangles

struct display_update_struct {
    struct render_area *area;

    struct display_update_struct *next;
    struct display_update_struct *prior;
};

typedef struct display_update_struct display_update;


// display management and framebuffer synchronization

void queue_to_display(struct render_area* area);
void enqueue_display_operation(struct render_area *update_area);
void flush_to_display();

uint8_t display_page_for_y(uint8_t y);

// clear the frame buffer

void clear_framebuffer();

// text operations 

void text_at(uint8_t row, uint8_t column, const char *str);
void clear_text(uint8_t row, uint8_t column, uint8_t num_chars);
void inverse_text(uint8_t row, uint8_t column, char *str);

// clear frame buffer at arbitrary areas
void erase_area(uint8_t x, uint8_t y, uint8_t width, uint8_t height);

// dump the framebuffer state to stdout

void dump_display();

// graphical elements

typedef struct {
    uint8_t y;  // top of db_meter if horizontal
    uint8_t x;  // left edge of db_meter if vertical
    uint8_t width; // how wide the meter is
    bool vertical; // true if vertical orientation
    float value; // current value
    float min; // minimum value to display
    float max; // maximum value to display
    char *name;   // name to display for the represented value
    bool edit_mode; // if edit mode, allow for value change
    bool draw_graph_in_edit_mode;
    bool draw_graph_in_view_mode;
    bool display;
    float min_range;
    float min_ratio;
    float max_ratio;
    float high_limit;
    float high_limit_ratio;
    float low_limit;
    float low_limit_ratio;    
    float value_ratio;
    float adjust_increment;
    float *machine_value;
    uint8_t start_page;
    uint8_t end_page;
    bool value_as_line;
    bool value_updated;
    bool graph_updated;
    callback render;
    callback1f sync;
    uint16_t control_id;
} page_control;


page_control* horizontal_db_meter(uint8_t y_pos,float *machine_value,float min,float max,char *name, float adjust_increment);



void calculate_dbm_ratios(page_control *dbm);
void set_db_meter_display_range(page_control *dbm, float low, float high);
void shift_db_meter_display_range(page_control *dbm, float amount, bool center);
void destroy_db_meter(page_control *dbm);
void render_db_meter(page_control *dbm);
void destroy_control(page_control *control);
void render_db_meter_graph(page_control *dbm);
void render_db_meter_value(page_control *dbm);

void set_db_meter_display_mode(page_control *dbm, bool state);

page_control* millisecond_control(uint8_t y_pos, float *machine_value, float min, float max, char *name, float adjust_increment);
void set_millis_display_range(page_control *ctl, float low, float high);
void shift_millis_display_range(page_control *ctl, float amount, bool center);
void render_millis_graph(page_control *ctl);
void render_millis_value(page_control *ctl);
void render_millis_control(page_control *ctl);




// button management

typedef enum {
    R0B0 = 0,
    R0B1 = 1,
    R0B2 = 2,
    R0B3 = 3,
    LEFT = 4,
    R1B0 = 5,
    R1B6 = 6,
    R1B7 = 7,
    R1B8 = 8,
    RIGHT = 9,
    ROTR = 10
} BUTTON;

typedef enum {
    PRESS = 0,
    RELEASE = 1
} BUTTON_EVENT;




struct ui_page_struct {
    callback render;
    
    callback on_rotary_up;
    callback on_rotary_down;
    callback1 on_button_press;
    callback1 on_button_release;
    callback on_slider_change;
    callback initialize;
    callback on_external_update;
    
    bool edit_mode;
    void *active_control;
    bool initialized;
    char *on_page_in;
    char *on_page_out;    
    char *name;
    uint16_t id; // for associations with buttons or other numeric indexes
    
    struct ui_page_struct* next;
    struct ui_page_struct* prior;
    
};

typedef struct ui_page_struct ui_page;

extern ui_page *active_page;
extern SSD_i2c_display *disp;

void update_machine_state();

bool move_to_page(char *page_name);

ui_page *get_page_by_id(uint16_t);



uint32_t *get_button_state();

void button_event(BUTTON button, BUTTON_EVENT action, uint32_t duration);
void rotary_event(uint8_t direction);

ui_page *create_page(char *name);
ui_page *page_by_name(char *name);
ui_page *page_by_id(uint16_t id);

void ui_update(); // draw any updates to the system state if needed
void ui_initialize();
void set_ui_needs_update();// signal that the ui shoudl refresh itself due to non-ui originated changes



