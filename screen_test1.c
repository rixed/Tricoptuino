/***************************************************
  This is an example sketch for the Adafruit 1.8" TFT shield with joystick
  ----> http://www.adafruit.com/products/802

  Check out the links above for our tutorials and wiring diagrams
  These displays use SPI to communicate, 4 pins are required to
  interface
  One pin is also needed for the joystick, we use analog 3
  Adafruit invests time and resources providing this open source code,
  please support Adafruit and open-source hardware by purchasing
  products from Adafruit!

  Written by Limor Fried/Ladyada for Adafruit Industries.
  MIT license, all text above must be included in any redistribution
 ****************************************************/

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

int main(void)
{
	init();
    st7735_ctor(&tft, TFT_CS, TFT_DC, TFT_RST, MODEL_R_REDTAB);

    uint8_t col[3] = { 0,0,0 };
    uint8_t i = 0;
#   define INC 10

	for (;;) {
        fillScreen(&tft, Color565(col[0], col[1], col[2]));
        switch (read_button()) {
            case BUTTON_LEFT:  i = (i-1) % 3; break;
            case BUTTON_RIGHT: i = (i+1) % 3; break;
            case BUTTON_UP:    if (col[i] < 255-INC) col[i] += INC; else col[i] = 255; break;
            case BUTTON_DOWN:  if (col[i] > INC)     col[i] -= INC; else col[i] = 0; break;
            case BUTTON_SELECT: col[0] = col[1] = col[2] = 0; break;
        }
	}

	return 0;
}

