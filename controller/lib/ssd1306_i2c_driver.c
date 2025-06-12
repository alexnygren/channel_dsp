/**
 * Copyright (c) 2021 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <ctype.h>
#include "pico/stdlib.h"
#include "pico/binary_info.h"
#include "hardware/i2c.h"
#include "ssd1306_font.h"
#include "ssd1306_i2c_driver.h"
#include "bits8.h"

/* Example code to talk to an SSD1306-based OLED display

   The SSD1306 is an OLED/PLED driver chip, capable of driving displays up to
   128x64 pixels.

   NOTE: Ensure the device is capable of being driven at 3.3v NOT 5v. The Pico
   GPIO (and therefore I2C) cannot be used at 5v.

   You will need to use a level shifter on the I2C lines if you want to run the
   board at 5v.

   Connections on Raspberry Pi Pico board, other boards may vary.

   GPIO SSD1306_I2C_SDA_PIN (on Pico this is GP4 (pin 6)) -> SDA on display
   board
   GPIO SSD1306_I2C_SCL_PIN (on Pico this is GP5 (pin 7)) -> SCL on
   display board
   3.3v (pin 36) -> VCC on display board
   GND (pin 38)  -> GND on display board
*/


// 400 is usual, but often these can be overclocked to improve display response.
// Tested at 1000 on both 32 and 64 pixel height devices and it worked.
//#define SSD1306_I2C_CLK             400
#define SSD1306_I2C_CLK             1000

#define SSD1306_I2C_SDA_PIN 4
#define SSD1306_I2C_SCL_PIN 5

// commands (see datasheet)
#define SSD1306_SET_MEM_MODE        _u(0x20)
#define SSD1306_SET_COL_ADDR        _u(0x21)
#define SSD1306_SET_PAGE_ADDR       _u(0x22)
#define SSD1306_SET_HORIZ_SCROLL    _u(0x26)
#define SSD1306_SET_SCROLL          _u(0x2E)

#define SSD1306_SET_DISP_START_LINE _u(0x40)

#define SSD1306_SET_CONTRAST        _u(0x81)
#define SSD1306_SET_CHARGE_PUMP     _u(0x8D)

#define SSD1306_SET_SEG_REMAP       _u(0xA0)
#define SSD1306_SET_ENTIRE_ON       _u(0xA4)
#define SSD1306_SET_ALL_ON          _u(0xA5)
#define SSD1306_SET_NORM_DISP       _u(0xA6)
#define SSD1306_SET_INV_DISP        _u(0xA7)
#define SSD1306_SET_MUX_RATIO       _u(0xA8)
#define SSD1306_SET_DISP            _u(0xAE)
#define SSD1306_SET_COM_OUT_DIR     _u(0xC0)
#define SSD1306_SET_COM_OUT_DIR_FLIP _u(0xC0)

#define SSD1306_SET_DISP_OFFSET     _u(0xD3)
#define SSD1306_SET_DISP_CLK_DIV    _u(0xD5)
#define SSD1306_SET_PRECHARGE       _u(0xD9)
#define SSD1306_SET_COM_PIN_CFG     _u(0xDA)
#define SSD1306_SET_VCOM_DESEL      _u(0xDB)


#define SSD1306_WRITE_MODE         _u(0xFE)
#define SSD1306_READ_MODE          _u(0xFF)

// page height is 4 if height is 32

#define pgm_read_byte(addr)   (*(const unsigned char *)(addr))

#define max(X, Y) ((X) > (Y) ? (X) : (Y))


static void set_pixel(SSD_i2c_display *disp, int x,int y, bool on);

void calc_render_area_buflen(SSD_i2c_display* disp, struct render_area *area) {
    // calculate how long the flattened buffer will be for a render area
    area->buflen = (area->end_col - area->start_col + 1) * (area->end_page - area->start_page + 1);
    disp->offset = (area->start_page * SSD1306_WIDTH) + area->start_col;
    
    if (area->buflen < 0) {
	printf("display: sc=%d sp=%d  ec=%d ep=%d  offset=%ld len=%d [%d x %d]\n",area->start_col,area->start_page, area->end_col, area->end_page, disp->offset,area->buflen, (area->end_col - area->start_col + 1), (area->end_page - area->start_page + 1 ));
	area->buflen = 0;
    }
}



void SSD1306_send_cmd(SSD_i2c_display* disp, uint8_t cmd) {
    // I2C write process expects a control byte followed by data
    // this "data" can be a command or data to follow up a command
    // Co = 1, D/C = 0 => the driver expects a command
    uint8_t buf[2] = {0x80, cmd};
    int rval = i2c_write_timeout_us(disp->i2c, disp->addr, buf, 2, false,300000);
    if (rval == 2) return;
    if (rval == PICO_ERROR_TIMEOUT) {
	printf("SSD1306: ERROR timeout\n");
    } else if (rval == PICO_ERROR_GENERIC) {
	printf("SSD1306: ERROR: no device present or address not acknowledged.\n");
    }
    return;
}

void SSD1306_send_cmd_list(SSD_i2c_display* disp, uint8_t *cmds, int num) {
    for (int i=0;i<num;i++)
        SSD1306_send_cmd(disp, cmds[i]);
}




void SSD1306_send_buf(SSD_i2c_display* disp, int buflen, struct render_area* area) {
    // in horizontal addressing mode, the column address pointer auto-increments
    // and then wraps around to the next page, so we can send the entire frame
    // buffer in one gooooooo!

    // copy our frame buffer into a new buffer because we need to add the control byte
    // to the beginning
    if (buflen>0) {
	uint8_t *buf = disp->buf;
	uint16_t num_copied = 0;
	uint16_t copy_offset = (area->start_page*SSD1306_WIDTH)+area->start_col;
	uint8_t cols_per_page = area->end_col - area->start_col + 1;
	uint8_t num_pages = area->end_page - area->start_page + 1;
	uint8_t *temp_buf = malloc(buflen + 1);
	int num_written = 0;
	temp_buf[0] = 0x40;

	// Copy each page byte for the transfer window to the temp buffer.
	// Since the start and stop of the transfer area are not necessarily...
	// sequential in our framebuffer, we need to copy in each in-scope page byte,
	// starting at the starting column and ending at the end column.

	for (uint8_t page=0; page < num_pages; page++) {
	    memcpy(temp_buf+1+num_copied, buf+copy_offset,cols_per_page);
	    copy_offset += SSD1306_WIDTH;  // jump to the next page and starting column
	    num_copied += cols_per_page;
	}
	if (num_copied != buflen) {
	    printf("display: send_buf: num_copied: %d  buflen: %d  [%d x %d]\n",num_copied, buflen, cols_per_page,num_pages);
	}
	// copy from the starting page for the amount of data required
	//memcpy(temp_buf+1, buf+disp->offset, buflen);
	num_written += i2c_write_timeout_us(disp->i2c, disp->addr, temp_buf, buflen + 1, false,300000);
	if (num_written < num_copied) {
	    printf("display: num_written: %d  num_copied: %d\n",num_written, num_copied);
	}
	free(temp_buf);
    }
    
}

void SSD1306_init(SSD_i2c_display* disp) {
    // Some of these commands are not strictly necessary as the reset
    // process defaults to some of these but they are shown here
    // to demonstrate what the initialization sequence looks like
    // Some configuration values are recommended by the board manufacturer

    uint8_t cmds[] = {
        SSD1306_SET_DISP,               // set display off
        /* memory mapping */
        SSD1306_SET_MEM_MODE,           // set memory address mode 0 = horizontal, 1 = vertical, 2 = page
        0x00,                           // horizontal addressing mode
        /* resolution and layout */
        SSD1306_SET_DISP_START_LINE,    // set display start line to 0
        SSD1306_SET_SEG_REMAP | 0x01,   // set segment re-map, column address 127 is mapped to SEG0
        SSD1306_SET_MUX_RATIO,          // set multiplex ratio
        SSD1306_HEIGHT - 1,             // Display height - 1
        SSD1306_SET_COM_OUT_DIR | 0x08, // set COM (common) output scan direction. Scan from bottom up, COM[N-1] to COM0
        SSD1306_SET_DISP_OFFSET,        // set display offset
        0x00,                           // no offset
        SSD1306_SET_COM_PIN_CFG,        // set COM (common) pins hardware configuration. Board specific magic number.
                                        // 0x02 Works for 128x32, 0x12 Possibly works for 128x64. Other options 0x22, 0x32
#if ((SSD1306_WIDTH == 128) && (SSD1306_HEIGHT == 32))
        0x02,
#elif ((SSD1306_WIDTH == 128) && (SSD1306_HEIGHT == 64))
        0x12,
#else
        0x02,
#endif
        /* timing and driving scheme */
        SSD1306_SET_DISP_CLK_DIV,       // set display clock divide ratio
        0x80,                           // div ratio of 1, standard freq
        SSD1306_SET_PRECHARGE,          // set pre-charge period
        0xF1,                           // Vcc internally generated on our board
        SSD1306_SET_VCOM_DESEL,         // set VCOMH deselect level
        0x30,                           // 0.83xVcc
        /* display */
        SSD1306_SET_CONTRAST,           // set contrast control
        0xFF,
        SSD1306_SET_ENTIRE_ON,          // set entire display on to follow RAM content
        SSD1306_SET_NORM_DISP,           // set normal (not inverted) display
        SSD1306_SET_CHARGE_PUMP,        // set charge pump
        0x14,                           // Vcc internally generated on our board
        SSD1306_SET_SCROLL | 0x00,      // deactivate horizontal scrolling if set. This is necessary as memory writes will corrupt if scrolling was enabled
        SSD1306_SET_DISP | 0x01, // turn display on
    };

    SSD1306_send_cmd_list(disp, cmds, count_of(cmds));

}

void SSD1306_scroll(SSD_i2c_display *disp, bool on) {
    // configure horizontal scrolling
    uint8_t cmds[] = {
        SSD1306_SET_HORIZ_SCROLL | 0x00,
        0x00, // dummy byte
        0x00, // start page 0
        0x00, // time interval
        0x03, // end page 3 SSD1306_NUM_PAGES ??
        0x00, // dummy byte
        0xFF, // dummy byte
        SSD1306_SET_SCROLL | (on ? 0x01 : 0) // Start/stop scrolling
    };

    SSD1306_send_cmd_list(disp, cmds, count_of(cmds));
}

void render(SSD_i2c_display *disp, struct render_area *area) {
    // update a portion of the display with a render area
    uint8_t cmds[] = {
        SSD1306_SET_COL_ADDR,
        area->start_col,
        area->end_col,
        SSD1306_SET_PAGE_ADDR,
        area->start_page,
        area->end_page
    };

    SSD1306_send_cmd_list(disp, cmds, count_of(cmds));
    //  printf("%s: sc=%d,ec=%d,sp=%d,ep=%d, buflen=%d, id:%ld\n",
    //	   __FUNCTION__,area->start_col,area->end_col,area->start_page,area->end_page,area->buflen,area->id);
    SSD1306_send_buf(disp, area->buflen, area);
}

void ssd_render(SSD_i2c_display *disp, struct render_area *area) {
    calc_render_area_buflen(disp, area);
    render(disp,area);
}

static void set_pixel(SSD_i2c_display *disp, int x,int y, bool on) {
    assert(x >= 0 && x < SSD1306_WIDTH && y >=0 && y < SSD1306_HEIGHT);

    // The calculation to determine the correct bit to set depends on which address
    // mode we are in. This code assumes horizontal

    // The video ram on the SSD1306 is split up in to 8 rows, one bit per pixel.
    // Each row is 128 long by 8 pixels high, each byte vertically arranged, so byte 0 is x=0, y=0->7,
    // byte 1 is x = 1, y=0->7 etc

    // This code could be optimised, but is like this for clarity. The compiler
    // should do a half decent job optimising it anyway.

    const int BytesPerRow = SSD1306_WIDTH ; // x pixels, 1bpp, but each row is 8 pixel high, so (x / 8) * 8

    int byte_idx = (y / 8) * BytesPerRow + x;
    uint8_t byte = disp->buf[byte_idx];

    if (on)
        byte |=  1 << (y % 8);
    else
        byte &= ~(1 << (y % 8));

    disp->buf[byte_idx] = byte;
}

void draw_pixel(SSD_i2c_display *disp, int x, int y, bool on) {
    set_pixel(disp, x, y, on);
}

// Basic Bresenhams.
void draw_line(SSD_i2c_display *disp, int x0, int y0, int x1, int y1, bool on) {

    int dx =  abs(x1-x0);
    int sx = x0<x1 ? 1 : -1;
    int dy = -abs(y1-y0);
    int sy = y0<y1 ? 1 : -1;
    int err = dx+dy;
    int e2;

    while (true) {
        set_pixel(disp, x0, y0, on);
        if (x0 == x1 && y0 == y1)
            break;
        e2 = 2*err;

        if (e2 >= dy) {
            err += dy;
            x0 += sx;
        }
        if (e2 <= dx) {
            err += dx;
            y0 += sy;
        }
    }
}

static inline int get_font_index(uint8_t ch) {
    if (ch >= 'A' && ch <='Z') {
        return  ch - 'A' + 1;
    }
    else if (ch >= '0' && ch <='9') {
        return  ch - '0' + 27;
    } else if (ch == '.') {
	return 37;
    } else if (ch == '-') {
	return 38;
    } else if (ch == ':') {
	return 39;
    }
    else return  0; // Not got that char so space.
}


static void write_char(SSD_i2c_display *disp, int16_t x, int16_t y, uint8_t ch) {
    if (x > SSD1306_WIDTH - 8 || y > SSD1306_HEIGHT - 8)
        return;

    // For the moment, only write on Y row boundaries (every 8 vertical pixels)
    y = y/8;

    ch = toupper(ch);
    int idx = get_font_index(ch);
    int fb_idx = y * 128 + x;

    for (int i=0;i<8;i++) {
        disp->buf[fb_idx++] = font[idx * 8 + i];
    }
}


void write_string(SSD_i2c_display *disp, int16_t x, int16_t y, const char *str) {
    // Cull out any string off the screen
    if (x > SSD1306_WIDTH - 8 || y > SSD1306_HEIGHT - 8)
        return;

    while (*str) {
        write_char(disp, x, y, *str++);
        x+=8;
    }
}


void ssd_render_all(SSD_i2c_display *disp) {
    struct render_area frame_area = {
        start_col: 0,
        end_col : SSD1306_WIDTH - 1,
        start_page : 0,
        end_page : SSD1306_NUM_PAGES - 1
        };

    calc_render_area_buflen(disp, &frame_area);
    render(disp, &frame_area);
}

// clear in vertical chunks of 8, so line num should be 0,16,32,40,48,56..
// row_num is a value from 0 to 7
void clear_row(SSD_i2c_display *disp, uint8_t row_num) {
    uint16_t offset = row_num * SSD1306_WIDTH;
    for(uint8_t x = 0; x < SSD1306_WIDTH; x++) {
	disp->buf[offset+x] = 0;
    }
}

void clear_display(SSD_i2c_display *disp) {
    // zero the entire display
    struct render_area frame_area = {
      start_col: 0,
      end_col : SSD1306_WIDTH - 1,
      start_page : 0,
      end_page : SSD1306_NUM_PAGES - 1
    };
    
    calc_render_area_buflen(disp, &frame_area);
    memset(disp->buf, 0, SSD1306_BUF_LEN);
    render(disp, &frame_area);
       
}


SSD_i2c_display *SSD1306_initialize(i2c_inst_t *i2c_bus, uint8_t addr) {

    SSD_i2c_display *disp = (SSD_i2c_display *) calloc(1,sizeof(SSD_i2c_display));
    disp->addr = addr;
    disp->i2c = i2c_bus;
    disp->buf = (uint8_t *) malloc(SSD1306_BUF_LEN);
    disp->font_data = NULL;    
    disp->color = WHITE;
    disp->text_alignment = TEXT_ALIGN_LEFT;
    disp->error_state = false;

    // useful information for picotool
    //bi_decl(bi_2pins_with_func(SSD1306_I2C_SDA_PIN, SSD1306_I2C_SCL_PIN, GPIO_FUNC_I2C));
    //bi_decl(bi_program_description("SSD1306 OLED driver I2C"));

    // Assume the bus is initialized

    // run through the complete initialization process
    SSD1306_init(disp);

    // Initialize render area for entire frame (SSD1306_WIDTH pixels by SSD1306_NUM_PAGES pages)
    struct render_area frame_area = {
        start_col: 0,
        end_col : SSD1306_WIDTH - 1,
        start_page : 0,
        end_page : SSD1306_NUM_PAGES - 1
        };

    calc_render_area_buflen(disp, &frame_area);

    // zero the entire display
    uint8_t buf[SSD1306_BUF_LEN];
    memset(buf, 0, SSD1306_BUF_LEN);
    render(disp, &frame_area);
    return disp;
}

char DefaultFontTableLookup(const uint8_t ch) {
    // UTF-8 to font table index converter
    // Code form http://playground.arduino.cc/Main/Utf8ascii
	static uint8_t LASTCHAR;

	if (ch < 128) { // Standard ASCII-set 0..0x7F handling
		LASTCHAR = 0;
		return ch;
	}

	uint8_t last = LASTCHAR;   // get last char
	LASTCHAR = ch;

	switch (last) {    // conversion depnding on first UTF8-character
		case 0xC2: return (uint8_t) ch;
		case 0xC3: return (uint8_t) (ch | 0xC0);
		case 0x82: if (ch == 0xAC) return (uint8_t) 0x80;    // special case Euro-symbol
	}

	return (uint8_t) 0; // otherwise: return zero, if character has to be ignored
}

void ssd_set_font(SSD_i2c_display *disp, const char *font_data) {
    disp->font_data = font_data;
}

void ssd_set_color(SSD_i2c_display *disp, OLEDDISPLAY_COLOR color) {
    disp->color = color;
}

void ssd_set_alignment(SSD_i2c_display *disp, OLEDDISPLAY_TEXT_ALIGNMENT alignment) {
    disp->text_alignment = alignment;
}

uint16_t get_string_width(SSD_i2c_display *disp, const char* text, uint16_t length, bool utf8) {
  uint16_t firstChar        = pgm_read_byte(disp->font_data + FIRST_CHAR_POS);

  uint16_t stringWidth = 0;
  uint16_t maxWidth = 0;

  for (uint16_t i = 0; i < length; i++) {
    char c = text[i];
    if (utf8) {
      c = DefaultFontTableLookup(c);
      if (c == 0)
        continue;
    }
    stringWidth += pgm_read_byte(disp->font_data + JUMPTABLE_START + (c - firstChar) * JUMPTABLE_BYTES + JUMPTABLE_WIDTH);
    if (c == 10) {
      maxWidth = max(maxWidth, stringWidth);
      stringWidth = 0;
    }
  }

  return max(maxWidth, stringWidth);
}

void inline draw_internal(SSD_i2c_display *disp, int16_t xMove, int16_t yMove, int16_t width, int16_t height, uint16_t offset, uint16_t bytesInData) {
  if (width < 0 || height < 0) return;
  if (yMove + height < 0 || yMove > SSD1306_HEIGHT)  return;
  if (xMove + width  < 0 || xMove > SSD1306_WIDTH)   return;

  uint8_t  rasterHeight = 1 + ((height - 1) >> 3); // fast ceil(height / 8.0)
  int8_t   yOffset      = yMove & 7;

  bytesInData = bytesInData == 0 ? width * rasterHeight : bytesInData;

  int16_t initYMove   = yMove;
  int8_t  initYOffset = yOffset;

  for (uint16_t i = 0; i < bytesInData; i++) {

    // Reset if next horizontal drawing phase is started.
    if ( i % rasterHeight == 0) {
      yMove   = initYMove;
      yOffset = initYOffset;
    }
    
    uint8_t currentByte = pgm_read_byte(disp->font_data + offset + i);
    
    int16_t xPos = xMove + (i / rasterHeight);
    int16_t yPos = ((yMove >> 3) + (i % rasterHeight)) * SSD1306_WIDTH;

    
//    int16_t yScreenPos = yMove + yOffset;
    int16_t dataPos    = xPos  + yPos;

    if (dataPos >=  0  && dataPos < SSD1306_BUF_LEN &&
        xPos    >=  0  && xPos    < SSD1306_WIDTH ) {	
      if (yOffset >= 0) {	  
        switch (disp->color) {
          case WHITE:   disp->buf[dataPos] |= currentByte << yOffset; break;
          case BLACK:   disp->buf[dataPos] &= ~(currentByte << yOffset); break;
          case INVERSE: disp->buf[dataPos] ^= currentByte << yOffset; break;
        }

        if (dataPos < (SSD1306_BUF_LEN - SSD1306_WIDTH)) {
          switch (disp->color) {
            case WHITE:   disp->buf[dataPos + SSD1306_WIDTH] |= currentByte >> (8 - yOffset); break;
            case BLACK:   disp->buf[dataPos + SSD1306_WIDTH] &= ~(currentByte >> (8 - yOffset)); break;
            case INVERSE: disp->buf[dataPos + SSD1306_WIDTH] ^= currentByte >> (8 - yOffset); break;
          }
        }
      } else {
        // Make new offset position
        yOffset = -yOffset;

        switch (disp->color) {
          case WHITE:   disp->buf[dataPos] |= currentByte >> yOffset; break;
          case BLACK:   disp->buf[dataPos] &= ~(currentByte >> yOffset); break;
          case INVERSE: disp->buf[dataPos] ^= currentByte >> yOffset; break;
        }

        // Prepare for next iteration by moving one block up
        yMove -= 8;

        // and setting the new yOffset
        yOffset = 8 - yOffset;
      }
    }
  }
}


uint16_t write_string_internal(SSD_i2c_display *disp, int16_t xMove, int16_t yMove, const char* text, uint16_t textLength, uint16_t textWidth, bool utf8) {
  uint8_t textHeight       = pgm_read_byte(disp->font_data + HEIGHT_POS);
  uint8_t firstChar        = pgm_read_byte(disp->font_data + FIRST_CHAR_POS);
  uint16_t sizeOfJumpTable = pgm_read_byte(disp->font_data + CHAR_NUM_POS)  * JUMPTABLE_BYTES;

  uint16_t cursorX         = 0;
  uint16_t cursorY         = 0;
  uint16_t charCount       = 0;

  switch (disp->text_alignment) {
    case TEXT_ALIGN_CENTER_BOTH:
      yMove -= textHeight >> 1;
    // Fallthrough
    case TEXT_ALIGN_CENTER:
      xMove -= textWidth >> 1; // divide by 2
      break;
    case TEXT_ALIGN_RIGHT:
      xMove -= textWidth;
      break;
    case TEXT_ALIGN_LEFT:
      break;
  }

  // Don't draw anything if it is not on the screen.
  if (xMove + textWidth  < 0 || xMove >= SSD1306_WIDTH ) {return 0;}
  if (yMove + textHeight < 0 || yMove >= SSD1306_HEIGHT) {return 0;}

  for (uint16_t j = 0; j < textLength; j++) {
    int16_t xPos = xMove + cursorX;
    int16_t yPos = yMove + cursorY;
    if (xPos > SSD1306_WIDTH)
      break; // no need to continue
    charCount++;

    uint8_t code;
    if (utf8) {
      code = DefaultFontTableLookup(text[j]);
      if (code == 0)
        continue;
    } else
      code = text[j];
    if (code >= firstChar) {
      uint8_t charCode = code - firstChar;

      // 4 Bytes per char code
      uint8_t msbJumpToChar    = pgm_read_byte( disp->font_data + JUMPTABLE_START + charCode * JUMPTABLE_BYTES );                  // MSB  \ JumpAddress
      uint8_t lsbJumpToChar    = pgm_read_byte( disp->font_data + JUMPTABLE_START + charCode * JUMPTABLE_BYTES + JUMPTABLE_LSB);   // LSB /
      uint8_t charByteSize     = pgm_read_byte( disp->font_data + JUMPTABLE_START + charCode * JUMPTABLE_BYTES + JUMPTABLE_SIZE);  // Size
      uint8_t currentCharWidth = pgm_read_byte( disp->font_data + JUMPTABLE_START + charCode * JUMPTABLE_BYTES + JUMPTABLE_WIDTH); // Width

      // Test if the char is drawable
      if (!(msbJumpToChar == 255 && lsbJumpToChar == 255)) {
        // Get the position of the char data
        uint16_t charDataPosition = JUMPTABLE_START + sizeOfJumpTable + ((msbJumpToChar << 8) + lsbJumpToChar);
        draw_internal(disp, xPos, yPos, currentCharWidth, textHeight, charDataPosition, charByteSize);
      }

      cursorX += currentCharWidth;
    }
  }
  return charCount;
}



uint16_t write_text(SSD_i2c_display *disp, int16_t xMove, int16_t yMove, const char *str) {

    // fall back to basic text mode if there isn't a font loaded
    
    if (disp->font_data == NULL) {
	write_string(disp, xMove, yMove, str);
	return strlen(str);
    }

  uint16_t lineHeight = pgm_read_byte(disp->font_data + HEIGHT_POS);

  // char* text must be freed!
  char* text = strdup(str);
  if (!text) {    
    return 0;
  }

  uint16_t yOffset = 0;
  // If the string should be centered vertically too
  // we need to now how heigh the string is.
  if (disp->text_alignment == TEXT_ALIGN_CENTER_BOTH) {
    uint16_t lb = 0;
    // Find number of linebreaks in text
    for (uint16_t i=0;text[i] != 0; i++) {
      lb += (text[i] == 10);
    }
    // Calculate center
    yOffset = (lb * lineHeight) / 2;
  }

  uint16_t charDrawn = 0;
  uint16_t line = 0;
  char* textPart = strtok(text,"\n");
  while (textPart != NULL) {
    uint16_t length = strlen(textPart);
    charDrawn += write_string_internal(disp, xMove, yMove - yOffset + (line++) * lineHeight, textPart, length, get_string_width(disp, textPart, length, true), true);
    textPart = strtok(NULL, "\n");
  }
  free(text);
  return charDrawn;
}
