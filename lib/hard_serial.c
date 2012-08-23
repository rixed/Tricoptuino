// TODO
#define __AVR_LIBC_DEPRECATED_ENABLE__

/*
  HardwareSerial.cpp - Hardware serial library for Wiring
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
  
  Modified 23 November 2006 by David A. Mellis
  Modified 28 September 2010 by Mark Sproul
*/

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include <stdbool.h>
#include "arduino.h"
#include "wiring_private.h"

// this next line disables the entire HardwareSerial.cpp, 
// this is so I can support Attiny series and any other chip without a uart
#if defined(UBRRH) || defined(UBRR0H) || defined(UBRR1H) || defined(UBRR2H) || defined(UBRR3H)

#include "hard_serial.h"

// Define constants and variables for buffering incoming serial data.  We're
// using a ring buffer (I think), in which head is the index of the location
// to which to write the next incoming character and tail is the index of the
// location from which to read.
#if (RAMEND < 1000)
#  define SERIAL_BUFFER_SIZE 16U
#else
#  define SERIAL_BUFFER_SIZE 64U
#endif

struct ring_buffer {
  unsigned char buffer[SERIAL_BUFFER_SIZE];
  volatile unsigned int head;
  volatile unsigned int tail;
};

#if defined(USBCON)
  struct ring_buffer rx_buffer = { { 0 }, 0, 0};
  struct ring_buffer tx_buffer = { { 0 }, 0, 0};
#endif
#if defined(UBRRH) || defined(UBRR0H)
  struct ring_buffer rx_buffer  =  { { 0 }, 0, 0 };
  struct ring_buffer tx_buffer  =  { { 0 }, 0, 0 };
#endif
#if defined(UBRR1H)
  struct ring_buffer rx_buffer1  =  { { 0 }, 0, 0 };
  struct ring_buffer tx_buffer1  =  { { 0 }, 0, 0 };
#endif
#if defined(UBRR2H)
  struct ring_buffer rx_buffer2  =  { { 0 }, 0, 0 };
  struct ring_buffer tx_buffer2  =  { { 0 }, 0, 0 };
#endif
#if defined(UBRR3H)
  struct ring_buffer rx_buffer3  =  { { 0 }, 0, 0 };
  struct ring_buffer tx_buffer3  =  { { 0 }, 0, 0 };
#endif

inline void store_char(unsigned char c, struct ring_buffer *buffer)
{
  unsigned i = (unsigned)(buffer->head + 1) % SERIAL_BUFFER_SIZE;

  // if we should be storing the received character into the location
  // just before the tail (meaning that the head would advance to the
  // current location of the tail), we're about to overflow the buffer
  // and so we don't write the character or advance the head.
  if (i != buffer->tail) {
    buffer->buffer[buffer->head] = c;
    buffer->head = i;
  }
}

#if !defined(USART0_RX_vect) && defined(USART1_RX_vect)
// do nothing - on the 32u4 the first USART is USART1
#else
#if !defined(USART_RX_vect) && !defined(SIG_USART0_RECV) && \
    !defined(SIG_UART0_RECV) && !defined(USART0_RX_vect) && \
	!defined(SIG_UART_RECV)
#  error "Don't know what the Data Received vector is called for the first UART"
#else
  void __attribute__((weak)) hserial_event(void) {}
# define serialEvent_implemented
#if defined(USART_RX_vect)
  SIGNAL(USART_RX_vect)
#elif defined(SIG_USART0_RECV)
  SIGNAL(SIG_USART0_RECV)
#elif defined(SIG_UART0_RECV)
  SIGNAL(SIG_UART0_RECV)
#elif defined(USART0_RX_vect)
  SIGNAL(USART0_RX_vect)
#elif defined(SIG_UART_RECV)
  SIGNAL(SIG_UART_RECV)
#endif
  {
#  if defined(UDR0)
    unsigned char c  =  UDR0;
#  elif defined(UDR)
    unsigned char c  =  UDR;
#  else
#    error UDR not defined
#  endif
    store_char(c, &rx_buffer);
  }
#endif
#endif

#if defined(USART1_RX_vect)
  void hserial_event1(void) __attribute__((weak)) {}
#  define serialEvent1_implemented
  SIGNAL(USART1_RX_vect)
  {
    unsigned char c = UDR1;
    store_char(c, &rx_buffer1);
  }
#elif defined(SIG_USART1_RECV)
#  error SIG_USART1_RECV
#endif

#if defined(USART2_RX_vect) && defined(UDR2)
  void hserial_event2(void) __attribute__((weak)) {}
#  define serialEvent2_implemented
  SIGNAL(USART2_RX_vect)
  {
    unsigned char c = UDR2;
    store_char(c, &rx_buffer2);
  }
#elif defined(SIG_USART2_RECV)
#  error SIG_USART2_RECV
#endif

#if defined(USART3_RX_vect) && defined(UDR3)
  void hserial_event3(void) __attribute__((weak)) {}
#  define serialEvent3_implemented
  SIGNAL(USART3_RX_vect)
  {
    unsigned char c = UDR3;
    store_char(c, &rx_buffer3);
  }
#elif defined(SIG_USART3_RECV)
#  error SIG_USART3_RECV
#endif

#if defined(UBRRH) || defined(UBRR0H)
  struct hserial hserial;
#endif
#if defined(UBRR1H)
  struct hserial hserial1;
#endif
#if defined(UBRR2H)
  struct hserial hserial2;
#endif
#if defined(UBRR3H)
  struct hserial hserial3;
#endif

#if !defined(USART0_UDRE_vect) && defined(USART1_UDRE_vect)
// do nothing - on the 32u4 the first USART is USART1
#else
#if !defined(UART0_UDRE_vect) && !defined(UART_UDRE_vect) && !defined(USART0_UDRE_vect) && !defined(USART_UDRE_vect)
#  error "Don't know what the Data Register Empty vector is called for the first UART"
#else
#if defined(UART0_UDRE_vect)
ISR(UART0_UDRE_vect)
#elif defined(UART_UDRE_vect)
ISR(UART_UDRE_vect)
#elif defined(USART0_UDRE_vect)
ISR(USART0_UDRE_vect)
#elif defined(USART_UDRE_vect)
ISR(USART_UDRE_vect)
#endif
{
  if (tx_buffer.head == tx_buffer.tail) {
	// Buffer empty, so disable interrupts
#if defined(UCSR0B)
    cbi(UCSR0B, UDRIE0);
#else
    cbi(UCSRB, UDRIE);
#endif
  }
  else {
    // There is more data in the output buffer. Send the next byte
    unsigned char c = tx_buffer.buffer[tx_buffer.tail];
    tx_buffer.tail = (tx_buffer.tail + 1) % SERIAL_BUFFER_SIZE;
	
#  if defined(UDR0)
    UDR0 = c;
#  elif defined(UDR)
    UDR = c;
#  else
#    error UDR not defined
#  endif
  }
}
#endif
#endif

#ifdef USART1_UDRE_vect
ISR(USART1_UDRE_vect)
{
  if (tx_buffer1.head == tx_buffer1.tail) {
	// Buffer empty, so disable interrupts
    cbi(UCSR1B, UDRIE1);
  }
  else {
    // There is more data in the output buffer. Send the next byte
    unsigned char c = tx_buffer1.buffer[tx_buffer1.tail];
    tx_buffer1.tail = (tx_buffer1.tail + 1) % SERIAL_BUFFER_SIZE;
	
    UDR1 = c;
  }
}
#endif

#ifdef USART2_UDRE_vect
ISR(USART2_UDRE_vect)
{
  if (tx_buffer2.head == tx_buffer2.tail) {
	// Buffer empty, so disable interrupts
    cbi(UCSR2B, UDRIE2);
  }
  else {
    // There is more data in the output buffer. Send the next byte
    unsigned char c = tx_buffer2.buffer[tx_buffer2.tail];
    tx_buffer2.tail = (tx_buffer2.tail + 1) % SERIAL_BUFFER_SIZE;
	
    UDR2 = c;
  }
}
#endif

#ifdef USART3_UDRE_vect
ISR(USART3_UDRE_vect)
{
  if (tx_buffer3.head == tx_buffer3.tail) {
	// Buffer empty, so disable interrupts
    cbi(UCSR3B, UDRIE3);
  }
  else {
    // There is more data in the output buffer. Send the next byte
    unsigned char c = tx_buffer3.buffer[tx_buffer3.tail];
    tx_buffer3.tail = (tx_buffer3.tail + 1) % SERIAL_BUFFER_SIZE;
	
    UDR3 = c;
  }
}
#endif


// Constructors ////////////////////////////////////////////////////////////////

static void hserial_ctor(struct hserial *s, struct ring_buffer *rx_buffer, struct ring_buffer *tx_buffer,
  volatile uint8_t *ubrrh, volatile uint8_t *ubrrl,
  volatile uint8_t *ucsra, volatile uint8_t *ucsrb,
  volatile uint8_t *udr,
  uint8_t rxen, uint8_t txen, uint8_t rxcie, uint8_t udrie, uint8_t u2x)
{
  s->rx_buffer = rx_buffer;
  s->tx_buffer = tx_buffer;
  s->ubrrh = ubrrh;
  s->ubrrl = ubrrl;
  s->ucsra = ucsra;
  s->ucsrb = ucsrb;
  s->udr = udr;
  s->rxen = rxen;
  s->txen = txen;
  s->rxcie = rxcie;
  s->udrie = udrie;
  s->u2x = u2x;
}

// Public Methods //////////////////////////////////////////////////////////////

void hserial_begin(struct hserial *s, unsigned long baud)
{
    if (s->inited) return;

    uint16_t baud_setting;
    bool use_u2x = true;

#   if F_CPU == 16000000UL
    // hardcoded exception for compatibility with the bootloader shipped
    // with the Duemilanove and previous boards and the firmware on the 8U2
    // on the Uno and Mega 2560.
    if (baud == 57600) {
        use_u2x = false;
    }
#   endif

try_again:
  
    if (use_u2x) {
        *s->ucsra = 1 << s->u2x;
        baud_setting = (F_CPU / 4 / baud - 1) / 2;
    } else {
        *s->ucsra = 0;
        baud_setting = (F_CPU / 8 / baud - 1) / 2;
    }

    if ((baud_setting > 4095) && use_u2x)
    {
        use_u2x = false;
        goto try_again;
    }

    // assign the baud_setting, a.k.a. ubbr (USART Baud Rate Register)
    *s->ubrrh = baud_setting >> 8;
    *s->ubrrl = baud_setting;

    sbi(*s->ucsrb, s->rxen);
    sbi(*s->ucsrb, s->txen);
    sbi(*s->ucsrb, s->rxcie);
    cbi(*s->ucsrb, s->udrie);
    s->inited = true;
}

void hserial_end(struct hserial *s)
{
  // wait for transmission of outgoing data
  while (s->tx_buffer->head != s->tx_buffer->tail)
    ;

  cbi(*s->ucsrb, s->rxen);
  cbi(*s->ucsrb, s->txen);
  cbi(*s->ucsrb, s->rxcie);  
  cbi(*s->ucsrb, s->udrie);
  
  // clear any received data
  s->rx_buffer->head = s->rx_buffer->tail;
}

int hserial_available(struct hserial *s)
{
  return (unsigned int)(SERIAL_BUFFER_SIZE + s->rx_buffer->head - s->rx_buffer->tail) % SERIAL_BUFFER_SIZE;
}

void hserial_event_run(void)
{
#ifdef serialEvent_implemented
  if (hserial_available(&hserial)) hserial_event();
#endif
#ifdef serialEvent1_implemented
  if (hserial_available(&hserial1)) hserial_event1();
#endif
#ifdef serialEvent2_implemented
  if (hserial_available(&hserial2)) hserial_event2();
#endif
#ifdef serialEvent3_implemented
  if (hserial_available(&hserial3)) hserial_event3();
#endif
}

int hserial_peek(struct hserial *s)
{
  if (s->rx_buffer->head == s->rx_buffer->tail) {
    return -1;
  } else {
    return s->rx_buffer->buffer[s->rx_buffer->tail];
  }
}

int hserial_read(struct hserial *s)
{
  // if the head isn't ahead of the tail, we don't have any characters
  if (s->rx_buffer->head == s->rx_buffer->tail) {
    return -1;
  } else {
    unsigned char c = s->rx_buffer->buffer[s->rx_buffer->tail];
    s->rx_buffer->tail = (unsigned int)(s->rx_buffer->tail + 1) % SERIAL_BUFFER_SIZE;
    return c;
  }
}

void hserial_flush(struct hserial *s)
{
  while (s->tx_buffer->head != s->tx_buffer->tail)
    ;
}

void hserial_write(struct hserial *s, uint8_t c)
{
  unsigned i = (s->tx_buffer->head + 1) % SERIAL_BUFFER_SIZE;
	
  // If the output buffer is full, there's nothing for it other than to 
  // wait for the interrupt handler to empty it a bit
  // ???: return 0 here instead?
  while (i == s->tx_buffer->tail)
    ;
	
  s->tx_buffer->buffer[s->tx_buffer->head] = c;
  s->tx_buffer->head = i;
	
  sbi(*s->ucsrb, s->udrie);
}

size_t hserial_gets(struct hserial *s, uint8_t *buf, size_t buf_len)
{
    size_t len = 0;
    while (--buf_len > 0) { // we take one for the nul
        int c;
        do { c = hserial_read(s); } while (c < 0);
        buf[len++] = c;
        if (c == '\r') break;
    }

    buf[len] = '\0';
    return len;
}

void hserial_write_buf(struct hserial *s, uint8_t *buf, size_t len)
{
    while (len--) hserial_write(s, *buf++);
}

void hserial_print(struct hserial *s, char const *str)
{
    while (*str) {
        if (*str == '\n') hserial_write(s, '\r');
        hserial_write(s, *str++);
    }
}

// Preinstantiate Objects //////////////////////////////////////////////////////

static void __attribute__((__constructor__)) hserial_preinstantiate(void)
{
#if defined(UBRRH) && defined(UBRRL)
  hserial_ctor(&hserial, &rx_buffer, &tx_buffer, &UBRRH, &UBRRL, &UCSRA, &UCSRB, &UDR, RXEN, TXEN, RXCIE, UDRIE, U2X);
#elif defined(UBRR0H) && defined(UBRR0L)
  hserial_ctor(&hserial, &rx_buffer, &tx_buffer, &UBRR0H, &UBRR0L, &UCSR0A, &UCSR0B, &UDR0, RXEN0, TXEN0, RXCIE0, UDRIE0, U2X0);
#elif defined(USBCON)
  // do nothing - Serial object and buffers are initialized in CDC code
#else
#  error no serial port defined  (port 0)
#endif

#if defined(UBRR1H)
  hserial_ctor(&hserial1, &rx_buffer1, &tx_buffer1, &UBRR1H, &UBRR1L, &UCSR1A, &UCSR1B, &UDR1, RXEN1, TXEN1, RXCIE1, UDRIE1, U2X1);
#endif
#if defined(UBRR2H)
  hserial_ctor(&hserial2, &rx_buffer2, &tx_buffer2, &UBRR2H, &UBRR2L, &UCSR2A, &UCSR2B, &UDR2, RXEN2, TXEN2, RXCIE2, UDRIE2, U2X2);
#endif
#if defined(UBRR3H)
  hserial_ctor(&herial3, &rx_buffer3, &tx_buffer3, &UBRR3H, &UBRR3L, &UCSR3A, &UCSR3B, &UDR3, RXEN3, TXEN3, RXCIE3, UDRIE3, U2X3);
#endif
}

#endif // whole file

