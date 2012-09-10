#include <arduino.h>
#include <twi.h>
#include <assert.h>
#include <adxl345.h>

uint8_t adxl_read(uint8_t reg)
{
    twi_writeTo(ADXL_ADDR, &reg, 1, 1, 0);
    uint8_t v;
    if (1 != twi_readFrom(ADXL_ADDR, &v, 1, 1)) {
        assert_fail(255, "Cannot read from ADXL");
    }
    return v;
}

void adxl_write(uint8_t reg, uint8_t v, uint8_t wait)
{
    uint8_t err = twi_writeTo(ADXL_ADDR, (uint8_t[2]){ reg, v }, 2, wait, 1);
    if (err) assert_fail(err, "Cannot write");
}

void adxl_xyz(uint16_t xyz[3])
{
    twi_writeTo(ADXL_ADDR, (uint8_t[1]){ ADXL_DATA_X0 }, 1, 1, 0);
    if (6 != twi_readFrom(ADXL_ADDR, (uint8_t *)xyz, 6, 1)) {
        assert_fail(127, "Cannot read position");
    }
}

void adxl_init(void)
{
    twi_init();
    assert(0345 == adxl_read(ADXL_DEVID));
}
