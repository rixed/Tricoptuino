/*************************************************** 
  This is a library for the Adafruit 1.8" SPI display.
  This library works with the Adafruit 1.8" TFT Breakout w/SD card
  ----> http://www.adafruit.com/products/358
  as well as Adafruit raw 1.8" TFT display
  ----> http://www.adafruit.com/products/618
 
  Check out the links above for our tutorials and wiring diagrams
  These displays use SPI to communicate, 4 or 5 pins are required to
  interface (RST is optional)
  Adafruit invests time and resources providing this open source code,
  please support Adafruit and open-source hardware by purchasing
  products from Adafruit!

  Written by Limor Fried/Ladyada for Adafruit Industries.
  MIT license, all text above must be included in any redistribution
 ****************************************************/

#include "st7735.h"
#include <avr/pgmspace.h>
#include <limits.h>
#include "pins_arduino.h"
#include "wiring_private.h"
#include <spi.h>

static void writecommand(struct st7735 *st, uint8_t c)
{
    *st->rsport &= ~st->rspinmask;
    *st->csport &= ~st->cspinmask;

    spi_transfer(c);

    *st->csport |= st->cspinmask;
}

static void writedata(struct st7735 *st, uint8_t c)
{
    *st->rsport |=  st->rspinmask;
    *st->csport &= ~st->cspinmask;

    spi_transfer(c);

    *st->csport |= st->cspinmask;
} 

// Rather than a bazillion writecommand() and writedata() calls, screen
// initialization commands and arguments are organized in these tables
// stored in PROGMEM.  The table may look bulky, but that's mostly the
// formatting -- storage-wise this is hundreds of bytes more compact
// than the equivalent code.  Companion function follows.
// FIXME: So for a ggiven device we need only part of this table!
#define DELAY 0x80
PROGMEM static uint8_t const
  Bcmd[] = {                  // Initialization commands for 7735B screens
    18,                       // 18 commands in list:
    ST7735_SWRESET,   DELAY,  //  1: Software reset, no args, w/delay
      50,                     //     50 ms delay
    ST7735_SLPOUT ,   DELAY,  //  2: Out of sleep mode, no args, w/delay
      255,                    //     255 = 500 ms delay
    ST7735_COLMOD , 1+DELAY,  //  3: Set color mode, 1 arg + delay:
      0x05,                   //     16-bit color
      10,                     //     10 ms delay
    ST7735_FRMCTR1, 3+DELAY,  //  4: Frame rate control, 3 args + delay:
      0x00,                   //     fastest refresh
      0x06,                   //     6 lines front porch
      0x03,                   //     3 lines back porch
      10,                     //     10 ms delay
    ST7735_MADCTL , 1      ,  //  5: Memory access ctrl (directions), 1 arg:
      0x08,                   //     Row addr/col addr, bottom to top refresh
    ST7735_DISSET5, 2      ,  //  6: Display settings #5, 2 args, no delay:
      0x15,                   //     1 clk cycle nonoverlap, 2 cycle gate
                              //     rise, 3 cycle osc equalize
      0x02,                   //     Fix on VTL
    ST7735_INVCTR , 1      ,  //  7: Display inversion control, 1 arg:
      0x0,                    //     Line inversion
    ST7735_PWCTR1 , 2+DELAY,  //  8: Power control, 2 args + delay:
      0x02,                   //     GVDD = 4.7V
      0x70,                   //     1.0uA
      10,                     //     10 ms delay
    ST7735_PWCTR2 , 1      ,  //  9: Power control, 1 arg, no delay:
      0x05,                   //     VGH = 14.7V, VGL = -7.35V
    ST7735_PWCTR3 , 2      ,  // 10: Power control, 2 args, no delay:
      0x01,                   //     Opamp current small
      0x02,                   //     Boost frequency
    ST7735_VMCTR1 , 2+DELAY,  // 11: Power control, 2 args + delay:
      0x3C,                   //     VCOMH = 4V
      0x38,                   //     VCOML = -1.1V
      10,                     //     10 ms delay
    ST7735_PWCTR6 , 2      ,  // 12: Power control, 2 args, no delay:
      0x11, 0x15,
    ST7735_GMCTRP1,16      ,  // 13: Magical unicorn dust, 16 args, no delay:
      0x09, 0x16, 0x09, 0x20, //     (seriously though, not sure what
      0x21, 0x1B, 0x13, 0x19, //      these config values represent)
      0x17, 0x15, 0x1E, 0x2B,
      0x04, 0x05, 0x02, 0x0E,
    ST7735_GMCTRN1,16+DELAY,  // 14: Sparkles and rainbows, 16 args + delay:
      0x0B, 0x14, 0x08, 0x1E, //     (ditto)
      0x22, 0x1D, 0x18, 0x1E,
      0x1B, 0x1A, 0x24, 0x2B,
      0x06, 0x06, 0x02, 0x0F,
      10,                     //     10 ms delay
    ST7735_CASET  , 4      ,  // 15: Column addr set, 4 args, no delay:
      0x00, 0x02,             //     XSTART = 2
      0x00, 0x81,             //     XEND = 129
    ST7735_RASET  , 4      ,  // 16: Row addr set, 4 args, no delay:
      0x00, 0x02,             //     XSTART = 1
      0x00, 0x81,             //     XEND = 160
    ST7735_NORON  ,   DELAY,  // 17: Normal display on, no args, w/delay
      10,                     //     10 ms delay
    ST7735_DISPON ,   DELAY,  // 18: Main screen turn on, no args, w/delay
      255 },                  //     255 = 500 ms delay

  Rcmd1[] = {                 // Init for 7735R, part 1 (red or green tab)
    15,                       // 15 commands in list:
    ST7735_SWRESET,   DELAY,  //  1: Software reset, 0 args, w/delay
      150,                    //     150 ms delay
    ST7735_SLPOUT ,   DELAY,  //  2: Out of sleep mode, 0 args, w/delay
      255,                    //     500 ms delay
    ST7735_FRMCTR1, 3      ,  //  3: Frame rate ctrl - normal mode, 3 args:
      0x01, 0x2C, 0x2D,       //     Rate = fosc/(1x2+40) * (LINE+2C+2D)
    ST7735_FRMCTR2, 3      ,  //  4: Frame rate control - idle mode, 3 args:
      0x01, 0x2C, 0x2D,       //     Rate = fosc/(1x2+40) * (LINE+2C+2D)
    ST7735_FRMCTR3, 6      ,  //  5: Frame rate ctrl - partial mode, 6 args:
      0x01, 0x2C, 0x2D,       //     Dot inversion mode
      0x01, 0x2C, 0x2D,       //     Line inversion mode
    ST7735_INVCTR , 1      ,  //  6: Display inversion ctrl, 1 arg, no delay:
      0x07,                   //     No inversion
    ST7735_PWCTR1 , 3      ,  //  7: Power control, 3 args, no delay:
      0xA2,
      0x02,                   //     -4.6V
      0x84,                   //     AUTO mode
    ST7735_PWCTR2 , 1      ,  //  8: Power control, 1 arg, no delay:
      0xC5,                   //     VGH25 = 2.4C VGSEL = -10 VGH = 3 * AVDD
    ST7735_PWCTR3 , 2      ,  //  9: Power control, 2 args, no delay:
      0x0A,                   //     Opamp current small
      0x00,                   //     Boost frequency
    ST7735_PWCTR4 , 2      ,  // 10: Power control, 2 args, no delay:
      0x8A,                   //     BCLK/2, Opamp current small & Medium low
      0x2A,  
    ST7735_PWCTR5 , 2      ,  // 11: Power control, 2 args, no delay:
      0x8A, 0xEE,
    ST7735_VMCTR1 , 1      ,  // 12: Power control, 1 arg, no delay:
      0x0E,
    ST7735_INVOFF , 0      ,  // 13: Don't invert display, no args, no delay
    ST7735_MADCTL , 1      ,  // 14: Memory access control (directions), 1 arg:
      0xC8,                   //     row addr/col addr, bottom to top refresh
    ST7735_COLMOD , 1      ,  // 15: set color mode, 1 arg, no delay:
      0x05 },                 //     16-bit color

  Rcmd2green[] = {            // Init for 7735R, part 2 (green tab only)
    2,                        //  2 commands in list:
    ST7735_CASET  , 4      ,  //  1: Column addr set, 4 args, no delay:
      0x00, 0x02,             //     XSTART = 0
      0x00, 0x7F+0x02,        //     XEND = 127
    ST7735_RASET  , 4      ,  //  2: Row addr set, 4 args, no delay:
      0x00, 0x01,             //     XSTART = 0
      0x00, 0x9F+0x01 },      //     XEND = 159
  Rcmd2red[] = {              // Init for 7735R, part 2 (red tab only)
    2,                        //  2 commands in list:
    ST7735_CASET  , 4      ,  //  1: Column addr set, 4 args, no delay:
      0x00, 0x00,             //     XSTART = 0
      0x00, 0x7F,             //     XEND = 127
    ST7735_RASET  , 4      ,  //  2: Row addr set, 4 args, no delay:
      0x00, 0x00,             //     XSTART = 0
      0x00, 0x9F },           //     XEND = 159

  Rcmd3[] = {                 // Init for 7735R, part 3 (red or green tab)
    4,                        //  4 commands in list:
    ST7735_GMCTRP1, 16      , //  1: Magical unicorn dust, 16 args, no delay:
      0x02, 0x1c, 0x07, 0x12,
      0x37, 0x32, 0x29, 0x2d,
      0x29, 0x25, 0x2B, 0x39,
      0x00, 0x01, 0x03, 0x10,
    ST7735_GMCTRN1, 16      , //  2: Sparkles and rainbows, 16 args, no delay:
      0x03, 0x1d, 0x07, 0x06,
      0x2E, 0x2C, 0x29, 0x2D,
      0x2E, 0x2E, 0x37, 0x3F,
      0x00, 0x00, 0x02, 0x10,
    ST7735_NORON  ,    DELAY, //  3: Normal display on, no args, w/delay
      10,                     //     10 ms delay
    ST7735_DISPON ,    DELAY, //  4: Main screen turn on, no args w/delay
      100 };                  //     100 ms delay


// Companion code to the above tables.  Reads and issues
// a series of LCD commands stored in PROGMEM byte array.
static void commandList(struct st7735 *st, uint8_t const *addr)
{
    uint8_t  numCommands, numArgs;
    uint16_t ms;

    numCommands = pgm_read_byte(addr++);   // Number of commands to follow
    while (numCommands--) {                 // For each command...
        writecommand(st, pgm_read_byte(addr++)); //   Read, issue command
        numArgs  = pgm_read_byte(addr++);    //   Number of args to follow
        ms       = numArgs & DELAY;          //   If hibit set, delay follows args
        numArgs &= ~DELAY;                   //   Mask out delay bit
        while(numArgs--) {                   //   For each argument...
            writedata(st, pgm_read_byte(addr++));  //     Read, issue argument
        }

        if(ms) {
            ms = pgm_read_byte(addr++); // Read post-command delay time (ms)
            if(ms == 255) ms = 500;     // If 255, delay for 500 ms
            delay(ms);
        }
    }
}


// Initialization code common to both 'B' and 'R' type displays
static void commonInit(struct st7735 *st, uint8_t const *cmdList)
{
    st->colstart = st->rowstart = 0; // May be overridden in init func

    pinMode(st->rs, OUTPUT);
    pinMode(st->cs, OUTPUT);
    st->csport    = portOutputRegister(digitalPinToPort(st->cs));
    st->cspinmask = digitalPinToBitMask(st->cs);
    st->rsport    = portOutputRegister(digitalPinToPort(st->rs));
    st->rspinmask = digitalPinToBitMask(st->rs);

    spi_begin();
    spi_setClockDivider(SPI_CLOCK_DIV4); // 4 MHz (half speed)
    spi_setBitOrder(MSBFIRST);
    spi_setDataMode(SPI_MODE0);

    // toggle RST low to reset; CS low so it'll listen to us
    *st->csport &= ~st->cspinmask;
    if (st->rst) {
        pinMode(st->rst, OUTPUT);
        digitalWrite(st->rst, HIGH);
        delay(500);
        digitalWrite(st->rst, LOW);
        delay(500);
        digitalWrite(st->rst, HIGH);
        delay(500);
    }

    if (cmdList) commandList(st, cmdList);
}

// Constructor when using hardware SPI.  Faster, but must use SPI pins
// specific to each board type (e.g. 11,13 for Uno, 51,52 for Mega, etc.)
void st7735_ctor(struct st7735 *st, uint8_t cs, uint8_t rs, uint8_t rst, uint8_t model)
{
    st->cs  = cs;
    st->rs  = rs;
    st->rst = rst;
    if (model == MODEL_B) {
        commonInit(st, Bcmd);
    } else {
        commonInit(st, Rcmd1);
        if (model == MODEL_R_GREENTAB) {
            commandList(st, Rcmd2green);
            st->colstart = 2;
            st->rowstart = 1;
        } else if (model == MODEL_R_REDTAB) {
            // colstart, rowstart left at default '0' values
            commandList(st, Rcmd2red);
        }
        commandList(st, Rcmd3);
    }
}

void setAddrWindow(struct st7735 *st, uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1)
{
    writecommand(st, ST7735_CASET); // Column addr set
    writedata(st, 0x00);
    writedata(st, x0 + st->colstart);     // XSTART 
    writedata(st, 0x00);
    writedata(st, x1 + st->colstart);     // XEND

    writecommand(st, ST7735_RASET); // Row addr set
    writedata(st, 0x00);
    writedata(st, y0 + st->rowstart);     // YSTART
    writedata(st, 0x00);
    writedata(st, y1 + st->rowstart);     // YEND

    writecommand(st, ST7735_RAMWR); // write to RAM
}

void pushColor(struct st7735 *st, uint16_t color)
{
    *st->rsport |=  st->rspinmask;
    *st->csport &= ~st->cspinmask;

    spi_transfer(color >> 8);
    spi_transfer(color);

    *st->csport |= st->cspinmask;
}

void drawPixel(struct st7735 *st, int16_t x, int16_t y, uint16_t color)
{
    setAddrWindow(st, x,y,x+1,y+1);

    *st->rsport |=  st->rspinmask;
    *st->csport &= ~st->cspinmask;

    spi_transfer(color >> 8);
    spi_transfer(color);

    *st->csport |= st->cspinmask;
}

// fill a rectangle
void fillRect(struct st7735 *st, int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color)
{
    // rudimentary clipping (drawChar w/big text requires this)
    if((x >= ST7735_TFTWIDTH) || (y >= ST7735_TFTHEIGHT)) return;
    if((x + w - 1) >= ST7735_TFTWIDTH)  w = ST7735_TFTWIDTH  - x;
    if((y + h - 1) >= ST7735_TFTHEIGHT) h = ST7735_TFTHEIGHT - y;

    setAddrWindow(st, x, y, x+w-1, y+h-1);

    uint8_t const hi = color >> 8, lo = color;
    *st->rsport |=  st->rspinmask;
    *st->csport &= ~st->cspinmask;
    for (uint8_t yy=h; yy>0; yy--) {
        for (uint8_t xx=w; xx>0; xx--) {
            spi_transfer(hi);
            spi_transfer(lo);
        }
    }

    *st->csport |= st->cspinmask;
}

void drawFastVLine(struct st7735 *st, int16_t x, int16_t y, int16_t h, uint16_t color)
{
    fillRect(st, x, y, 1, h, color);
}

void drawFastHLine(struct st7735 *st, int16_t x, int16_t y, int16_t w, uint16_t color)
{
    fillRect(st, x, y, w, 1, color);
}

void fillScreen(struct st7735 *st, uint16_t color)
{
    fillRect(st, 0, 0, ST7735_TFTWIDTH, ST7735_TFTHEIGHT, color);
}

// Pass 8-bit (each) R,G,B, get back 16-bit packed color
uint16_t Color565(uint8_t r, uint8_t g, uint8_t b)
{
    return ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3);
}

void invertDisplay(struct st7735 *st, bool i)
{
    writecommand(st, i ? ST7735_INVON : ST7735_INVOFF);
}

uint8_t read_button(void)
{
    uint16_t a = analogRead(3);

    if (a < 41) return BUTTON_DOWN;
    if (a < 205) return BUTTON_RIGHT;
    if (a < 307) return BUTTON_SELECT;
    if (a < 410) return BUTTON_UP;
    if (a < 655) return BUTTON_LEFT;
    else return BUTTON_NONE;
}

