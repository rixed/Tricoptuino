#include <stdlib.h>
#include <arduino.h>
#include <st7735.h>

// TFT display and SD card will share the hardware SPI interface.
// Hardware SPI pins are specific to the Arduino board type and
// cannot be remapped to alternate pins.  For Arduino Uno,
// Duemilanove, etc., pin 11 = MOSI, pin 12 = MISO, pin 13 = SCK.
#define SD_CS    4  // Chip select line for SD card
#define TFT_CS  10  // Chip select line for TFT display
#define TFT_DC   8  // Data/command line for TFT
#define TFT_RST  -1  // Reset line for TFT (or connect to +5V)

struct st7735 tft;

#define NB_DOTS 255
uint8_t dots[NB_DOTS][3];

#define BG_COL 0U
#define DOT_COL 0xFFFFU

static void update(void)
{
    for (uint8_t d = 0; d < NB_DOTS; d++) {
        uint8_t x0 = dots[d][0];
        uint8_t y0 = dots[d][1];
        uint8_t x = x0;
        if (dots[d][2] & 1) x ++; else x --;
        if (x >= ST7735_TFTWIDTH) {
            x = x0;
            dots[d][2] ^= 1;
        }
        uint8_t y = y0;
        if (dots[d][2] & 2) y ++; else y --;
        if (y >= ST7735_TFTHEIGHT) {
            y = y0;
            dots[d][2] ^= 2;
        }
        uint16_t const bg = y0 >> 3;
        uint16_t const fg = 0x8410 | Color565(d, d+d+d+d+d, d+d+d);
        drawPixel(&tft, x0, y0, bg);
        drawPixel(&tft, x, y, fg);
        dots[d][0] = x;
        dots[d][1] = y;
    }
}

static void rand_dots(void)
{
    for (uint8_t d = 0; d < NB_DOTS; d++) {
        dots[d][0] = rand() % ST7735_TFTWIDTH;
        dots[d][1] = rand() % ST7735_TFTHEIGHT;
        dots[d][2] = rand() & 3;
    }
}

static void draw_bg(void)
{
    for (uint8_t y = ST7735_TFTHEIGHT; y --> 0; ) {
        uint16_t const bg_col = y >> 3;
        drawFastHLine(&tft, 0, y, ST7735_TFTWIDTH, bg_col);
    }
}

int main(void)
{
	init();
    st7735_ctor(&tft, TFT_CS, TFT_DC, TFT_RST, MODEL_R_REDTAB);

    while (1) {
        rand_dots();
        draw_bg();
   
	    do update();
        while (BUTTON_SELECT != read_button());
	}

	return 0;
}

