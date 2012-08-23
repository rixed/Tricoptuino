/*
  HardwareSerial.h - Hardware serial library for Wiring
  Copyright (c) 2006 Nicholas Zambetti.  All right reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA

  Modified 28 September 2010 by Mark Sproul
*/

#ifndef HardwareSerial_h
#define HardwareSerial_h

#include <inttypes.h>

struct hserial {
    struct ring_buffer *rx_buffer;
    struct ring_buffer *tx_buffer;
    volatile uint8_t *ubrrh;
    volatile uint8_t *ubrrl;
    volatile uint8_t *ucsra;
    volatile uint8_t *ucsrb;
    volatile uint8_t *udr;
    uint8_t rxen;
    uint8_t txen;
    uint8_t rxcie;
    uint8_t udrie;
    uint8_t u2x;
    uint8_t inited:1;
};

#if defined(UBRRH) || defined(UBRR0H)
  extern struct hserial hserial;
#elif defined(USBCON)
  #include "USBAPI.h"
#endif
#if defined(UBRR1H)
  extern struct hserial hserial1;
#endif
#if defined(UBRR2H)
  extern struct hserial hserial2;
#endif
#if defined(UBRR3H)
  extern struct hserial hserial3;
#endif

void hserial_begin(struct hserial *, unsigned long);
void hserial_end(struct hserial *);

int hserial_available(struct hserial *);
int hserial_peek(struct hserial *);
int hserial_read(struct hserial *);
void hserial_flush(struct hserial *);
void hserial_write(struct hserial *, uint8_t);

size_t hserial_gets(struct hserial *, uint8_t *buf, size_t);
void hserial_write_buf(struct hserial *, uint8_t *buf, size_t);
void hserial_print(struct hserial *, char const *);

extern void hserial_event_run(void) __attribute__((weak));

#endif
