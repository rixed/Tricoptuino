// Getting something out of the ADXL345
#include <arduino.h>
#include <hard_serial.h>
#include <assert.h>
#include <adxl345.h>

uint8_t hexdigit_of(uint8_t v)
{
    v &= 0xf;
    if (v<10) return '0'+v;
    else return 'a'+(v-10);
}

int main(void)
{
    init();
	pinMode(13, OUTPUT);
    digitalWrite(13, 1);

    hserial_begin(&hserial, 9600);

    hserial_print(&hserial, "Initializing ADXL345\r\n");
    adxl_init();
    adxl_write(ADXL_POWER_CTL, ADXL_MEASURE_BIT, 1);

    forever {
        uint16_t xyz[3];
        adxl_xyz(xyz);

        hserial_print(&hserial, "A: ");
        for (uint8_t i = 0; i < 3; i++) {
            if (i > 0) hserial_print(&hserial, ", ");
            hserial_write(&hserial, hexdigit_of(xyz[i]>>12));
            hserial_write(&hserial, hexdigit_of(xyz[i]>>8));
            hserial_write(&hserial, hexdigit_of(xyz[i]>>4));
            hserial_write(&hserial, hexdigit_of(xyz[i]>>0));
        }
        hserial_print(&hserial, "\r\n");

        delay(100);
    }
}
