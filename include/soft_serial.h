/*
SoftwareSerial.h (formerly NewSoftSerial.h) - 
Multi-instance software serial library for Arduino/Wiring
-- Interrupt-driven receive and other improvements by ladyada
   (http://ladyada.net)
-- Tuning, circular buffer, derivation from class Print/Stream,
   multi-instance support, porting to 8MHz processors,
   various optimizations, PROGMEM delay tables, inverse logic and 
   direct port writing by Mikal Hart (http://www.arduiniana.org)
-- Pin change interrupt macros by Paul Stoffregen (http://www.pjrc.com)
-- 20MHz processor support by Garrett Mace (http://www.macetech.com)
-- ATmega1280/2560 support by Brett Hagman (http://www.roguerobotics.com/)

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

The latest version of this library can always be found at
http://arduiniana.org.
*/

#ifndef SoftwareSerial_h
#define SoftwareSerial_h

#include <inttypes.h>
#include <stdbool.h>

/******************************************************************************
* Definitions
******************************************************************************/

#ifndef GCC_VERSION
#define GCC_VERSION (__GNUC__ * 10000 + __GNUC_MINOR__ * 100 + __GNUC_PATCHLEVEL__)
#endif

struct sserial {
  uint8_t receivePin;
  uint8_t receiveBitMask;
  volatile uint8_t *receivePortRegister;
  uint8_t transmitBitMask;
  volatile uint8_t *transmitPortRegister;

  uint16_t rx_delay_centering;
  uint16_t rx_delay_intrabit;
  uint16_t rx_delay_stopbit;
  uint16_t tx_delay;

  uint16_t buffer_overflow:1;
  uint16_t inverse_logic:1;
  uint16_t write_error:1;
};


// static data
struct sserial *active_sserial;

// public methods
void sserial_ctor(struct sserial *, uint8_t receivePin, uint8_t transmitPin, bool inverse_logic);
void sserial_dtor(struct sserial *);
void sserial_begin(struct sserial *, long speed);
bool sserial_listen(struct sserial *);
void sserial_end(struct sserial *);
static inline bool sserial_isListening(struct sserial *s)
{
    return s == active_sserial;
}
static inline bool sserial_overflow(struct sserial *s)
{
	bool ret = s->buffer_overflow;
   	s->buffer_overflow = false;
   	return ret;
}

int sserial_peek(struct sserial *);

size_t sserial_write(struct sserial *, uint8_t byte);
int sserial_read(struct sserial *);
int sserial_available(struct sserial *);
void sserial_flush(struct sserial *);

#if 0
// Arduino 0012 workaround
#undef int
#undef char
#undef long
#undef byte
#undef float
#undef abs
#undef round
#endif

#endif
