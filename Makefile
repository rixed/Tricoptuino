top_srcdir = .
all: led_test.hex

include $(top_srcdir)/make.common

clean-spec:

.PHONY: cscope
cscope:
	cscope -Rb -I/usr/avr/include -I/usr/lib/avr/include

-include .depend
