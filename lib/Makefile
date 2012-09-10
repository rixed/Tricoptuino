top_srcdir = ../

all: libarduino.a

include $(top_srcdir)/make.common

SOURCES = \
	arduino.c \
	wiring.c wiring_digital.c wiring_analog.c wiring_pulse.c  wiring_shift.c \
	WInterrupts.c \
	spi.c soft_serial.c hard_serial.c \
	st7735.c \
	twi.c \
	event.c \
	servo.c \
	assert.c \
	adxl345.c

OBJECTS = $(SOURCES:.c=.o)

libarduino.a: $(OBJECTS)
	$(AR) rsc $@ $^

-include .depend
