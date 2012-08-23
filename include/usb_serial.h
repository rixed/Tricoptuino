#ifndef USB_SERIAL_H_120812
#define USB_SERIAL_H_120812y
#include <stdbool.h>
#include <stdint.h>

int serial_available(void);
void serial_accept(void);
int serial_peek(void);
int serial_read(void);
void serial_flush(void);
size_t serial_write(uint8_t);
bool serial_is_configured(void);


#endif
