#pragma once

#include "hardware/i2c.h"



struct render_area {
    uint16_t start_col;
    uint16_t end_col;
    uint16_t start_page;
    uint16_t end_page;
    int buflen;
    uint32_t id;
};

typedef enum {
  BLACK = 0,
  WHITE = 1,
  INVERSE = 2
} OLEDDISPLAY_COLOR;

typedef enum {
  TEXT_ALIGN_LEFT = 0,
  TEXT_ALIGN_RIGHT = 1,
  TEXT_ALIGN_CENTER = 2,
  TEXT_ALIGN_CENTER_BOTH = 3
} OLEDDISPLAY_TEXT_ALIGNMENT;



struct SSD_i2c_display_struct {
    i2c_inst_t *i2c;
    uint8_t addr;
    uint8_t *buf;
    uint32_t offset;
    bool error_state;
    const char *font_data;
    OLEDDISPLAY_COLOR color;
    OLEDDISPLAY_TEXT_ALIGNMENT text_alignment;
};


typedef struct SSD_i2c_display_struct SSD_i2c_display;

// Define the size of the display we have attached. This can vary, make sure you
// have the right size defined or the output will look rather odd!
// Code has been tested on 128x32 and 128x64 OLED displays
#ifndef SSD1306_HEIGHT
#define SSD1306_HEIGHT              64
#endif
#ifndef SSD1306_WIDTH
#define SSD1306_WIDTH               128
#endif

// Header Values
#define JUMPTABLE_BYTES 4

#define JUMPTABLE_LSB   1
#define JUMPTABLE_SIZE  2
#define JUMPTABLE_WIDTH 3
#define JUMPTABLE_START 4

#define WIDTH_POS 0
#define HEIGHT_POS 1
#define FIRST_CHAR_POS 2
#define CHAR_NUM_POS 3




#define SSD1306_I2C_ADDR_DEFAULT   _u(0x3C)
#define SSD1306_PAGE_HEIGHT         _u(8)
#define SSD1306_NUM_PAGES           (SSD1306_HEIGHT / SSD1306_PAGE_HEIGHT)
#define SSD1306_BUF_LEN             (SSD1306_NUM_PAGES * SSD1306_WIDTH)



void SSD1306_send_cmd(SSD_i2c_display* disp, uint8_t cmd);

void SSD1306_send_cmd_list(SSD_i2c_display* disp, uint8_t *cmds, int num);


void SSD1306_send_buf(SSD_i2c_display* disp, int buflen, struct render_area* area);


void SSD1306_scroll(SSD_i2c_display *disp, bool on);

void draw_pixel(SSD_i2c_display *disp, int x, int y, bool on);

void ssd_render(SSD_i2c_display *disp, struct render_area *area);

void ssd_render_all(SSD_i2c_display *disp);

void draw_line(SSD_i2c_display *disp, int x0, int y0, int x1, int y1, bool on);

// clear in vertical chunks of 8, so line num should be 0,16,32,40,48,56..

void clear_row(SSD_i2c_display *disp, uint8_t line_num);

//static void write_char(SSD_i2c_display *disp, int16_t x, int16_t y, uint8_t ch);

void write_string(SSD_i2c_display *disp, int16_t x, int16_t y, const char *str);
void ssd_set_font(SSD_i2c_display *disp, const char *font_data);
void ssd_set_color(SSD_i2c_display *disp, OLEDDISPLAY_COLOR color);
void ssd_set_alignment(SSD_i2c_display *disp, OLEDDISPLAY_TEXT_ALIGNMENT alignment);
uint16_t write_text(SSD_i2c_display *disp, int16_t xMove, int16_t yMove, const char *str);

SSD_i2c_display *SSD1306_initialize(i2c_inst_t *i2c_bus, uint8_t addr);

void clear_display(SSD_i2c_display *disp);
