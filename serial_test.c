// Example 1 : Blinking a LED
#include "arduino.h"
#include "hard_serial.h"

#define LED 13
#define BUTTON 7

uint8_t hexdigit_of(uint8_t v)
{
    if (v<10) return '0'+v;
    else return 'a'+(v-10);
}

int main(void)
{
	init();
    hserial_begin(&hserial, 9600);

    for (;;) {
        uint8_t line[32];
        size_t len = hserial_gets(&hserial, line, sizeof(line));
        hserial_print(&hserial, "Got this:\n");
        hserial_write_buf(&hserial, line, len);
        hserial_write(&hserial, '\n');
        /*while (hserial_available(&hserial) <= 0) ;
        uint8_t c = hserial_read(&hserial);
        hserial_write(&hserial, hexdigit_of(c>>4));
        hserial_write(&hserial, hexdigit_of(c&0xfU));
        hserial_write(&hserial, '\r');
        hserial_write(&hserial, '\n');*/
    }

	return 0;
}

