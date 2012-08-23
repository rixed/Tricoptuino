/*
SoftwareSerial.cpp (formerly NewSoftSerial.cpp) - 
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

// When set, _DEBUG co-opts pins 11 and 13 for debugging with an
// oscilloscope or logic analyzer.  Beware: it also slightly modifies
// the bit times, so don't rely on it too much at high baud rates
#define _DEBUG 0
#define _DEBUG_PIN1 11
#define _DEBUG_PIN2 13
// 
// Includes
// 
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include "arduino.h"
#include "soft_serial.h"
//
// Lookup table
//
typedef struct _DELAY_TABLE
{
  long baud;
  unsigned short rx_delay_centering;
  unsigned short rx_delay_intrabit;
  unsigned short rx_delay_stopbit;
  unsigned short tx_delay;
} DELAY_TABLE;

#if F_CPU == 16000000

static const DELAY_TABLE PROGMEM table[] = 
{
  //  baud    rxcenter   rxintra    rxstop    tx
  { 115200,   1,         17,        17,       12,    },
  { 57600,    10,        37,        37,       33,    },
  { 38400,    25,        57,        57,       54,    },
  { 31250,    31,        70,        70,       68,    },
  { 28800,    34,        77,        77,       74,    },
  { 19200,    54,        117,       117,      114,   },
  { 14400,    74,        156,       156,      153,   },
  { 9600,     114,       236,       236,      233,   },
  { 4800,     233,       474,       474,      471,   },
  { 2400,     471,       950,       950,      947,   },
  { 1200,     947,       1902,      1902,     1899,  },
  { 300,      3804,      7617,      7617,     7614,  },
};

const int XMIT_START_ADJUSTMENT = 5;

#elif F_CPU == 8000000

static const DELAY_TABLE table[] PROGMEM = 
{
  //  baud    rxcenter    rxintra    rxstop  tx
  { 115200,   1,          5,         5,      3,      },
  { 57600,    1,          15,        15,     13,     },
  { 38400,    2,          25,        26,     23,     },
  { 31250,    7,          32,        33,     29,     },
  { 28800,    11,         35,        35,     32,     },
  { 19200,    20,         55,        55,     52,     },
  { 14400,    30,         75,        75,     72,     },
  { 9600,     50,         114,       114,    112,    },
  { 4800,     110,        233,       233,    230,    },
  { 2400,     229,        472,       472,    469,    },
  { 1200,     467,        948,       948,    945,    },
  { 300,      1895,       3805,      3805,   3802,   },
};

const int XMIT_START_ADJUSTMENT = 4;

#elif F_CPU == 20000000

// 20MHz support courtesy of the good people at macegr.com.
// Thanks, Garrett!

static const DELAY_TABLE PROGMEM table[] =
{
  //  baud    rxcenter    rxintra    rxstop  tx
  { 115200,   3,          21,        21,     18,     },
  { 57600,    20,         43,        43,     41,     },
  { 38400,    37,         73,        73,     70,     },
  { 31250,    45,         89,        89,     88,     },
  { 28800,    46,         98,        98,     95,     },
  { 19200,    71,         148,       148,    145,    },
  { 14400,    96,         197,       197,    194,    },
  { 9600,     146,        297,       297,    294,    },
  { 4800,     296,        595,       595,    592,    },
  { 2400,     592,        1189,      1189,   1186,   },
  { 1200,     1187,       2379,      2379,   2376,   },
  { 300,      4759,       9523,      9523,   9520,   },
};

const int XMIT_START_ADJUSTMENT = 6;

#else

#error This version of SoftwareSerial supports only 20, 16 and 8MHz processors

#endif

//
// Statics
//
struct sserial *active_sserial = 0;
#define _SS_MAX_RX_BUFF 64 // RX buffer size
static char receive_buffer[_SS_MAX_RX_BUFF]; 
static volatile uint8_t receive_buffer_tail;
static volatile uint8_t receive_buffer_head;

extern inline bool sserial_isListening(struct sserial *);
extern  inline bool sserial_overflow(struct sserial *);

//
// Debugging
//
// This function generates a brief pulse
// for debugging or measuring on an oscilloscope.
static void DebugPulse(uint8_t pin, uint8_t count)
{
#if _DEBUG
  volatile uint8_t *pport = portOutputRegister(digitalPinToPort(pin));

  uint8_t val = *pport;
  while (count--)
  {
    *pport = val | digitalPinToBitMask(pin);
    *pport = val;
  }
#else
  (void)pin;
  (void)count;
#endif
}

//
// Private methods
//

static void tunedDelay(uint16_t delay)
{ 
  uint8_t tmp=0;

  asm volatile(
    "sbiw    %0, 0x01 \n\t"
    "ldi %1, 0xFF \n\t"
    "cpi %A0, 0xFF \n\t"
    "cpc %B0, %1 \n\t"
    "brne .-10 \n\t"
    : "=r" (delay), "+a" (tmp)
    : "0" (delay)
    );
}

// This function sets the current object as the "listening"
// one and returns true if it replaces another 
bool sserial_listen(struct sserial *s)
{
  if (active_sserial != s)
  {
    s->buffer_overflow = false;
    uint8_t oldSREG = SREG;
    cli();
    receive_buffer_head = receive_buffer_tail = 0;
    active_sserial = s;
    SREG = oldSREG;
    return true;
  }

  return false;
}

static uint8_t sserial_rx_pin_read(struct sserial const *s)
{
  return *s->receivePortRegister & s->receiveBitMask;
}

//
// The receive routine called by the interrupt handler
//
static void sserial_recv(struct sserial *s)
{

#if GCC_VERSION < 40302
// Work-around for avr-gcc 4.3.0 OSX version bug
// Preserve the registers that the compiler misses
// (courtesy of Arduino forum user *etracer*)
  asm volatile(
    "push r18 \n\t"
    "push r19 \n\t"
    "push r20 \n\t"
    "push r21 \n\t"
    "push r22 \n\t"
    "push r23 \n\t"
    "push r26 \n\t"
    "push r27 \n\t"
    ::);
#endif  

  uint8_t d = 0;

  // If RX line is high, then we don't see any start bit
  // so interrupt is probably not for us
  if (s->inverse_logic ? sserial_rx_pin_read(s) : !sserial_rx_pin_read(s))
  {
    // Wait approximately 1/2 of a bit width to "center" the sample
    tunedDelay(s->rx_delay_centering);
    DebugPulse(_DEBUG_PIN2, 1);

    // Read each of the 8 bits
    for (uint8_t i=0x1; i; i <<= 1)
    {
      tunedDelay(s->rx_delay_intrabit);
      DebugPulse(_DEBUG_PIN2, 1);
      uint8_t noti = ~i;
      if (sserial_rx_pin_read(s))
        d |= i;
      else // else clause added to ensure function timing is ~balanced
        d &= noti;
    }

    // skip the stop bit
    tunedDelay(s->rx_delay_stopbit);
    DebugPulse(_DEBUG_PIN2, 1);

    if (s->inverse_logic)
      d = ~d;

    // if buffer full, set the overflow flag and return
    if ((receive_buffer_tail + 1) % _SS_MAX_RX_BUFF != receive_buffer_head) 
    {
      // save new data in buffer: tail points to where byte goes
      receive_buffer[receive_buffer_tail] = d; // save new byte
      receive_buffer_tail = (receive_buffer_tail + 1) % _SS_MAX_RX_BUFF;
    } 
    else 
    {
#if _DEBUG // for scope: pulse pin as overflow indictator
      DebugPulse(_DEBUG_PIN1, 1);
#endif
      s->buffer_overflow = true;
    }
  }

#if GCC_VERSION < 40302
// Work-around for avr-gcc 4.3.0 OSX version bug
// Restore the registers that the compiler misses
  asm volatile(
    "pop r27 \n\t"
    "pop r26 \n\t"
    "pop r23 \n\t"
    "pop r22 \n\t"
    "pop r21 \n\t"
    "pop r20 \n\t"
    "pop r19 \n\t"
    "pop r18 \n\t"
    ::);
#endif
}

static void sserial_tx_pin_write(struct sserial *s, uint8_t pin_state)
{
  if (pin_state == LOW)
    *s->transmitPortRegister &= ~s->transmitBitMask;
  else
    *s->transmitPortRegister |= s->transmitBitMask;
}

//
// Interrupt handling
//

static void sserial_handle_interrupt(void)
{
  if (active_sserial)
  {
    sserial_recv(active_sserial);
  }
}

#if defined(PCINT0_vect)
ISR(PCINT0_vect)
{
  sserial_handle_interrupt();
}
#endif

#if defined(PCINT1_vect)
ISR(PCINT1_vect)
{
  sserial_handle_interrupt();
}
#endif

#if defined(PCINT2_vect)
ISR(PCINT2_vect)
{
  sserial_handle_interrupt();
}
#endif

#if defined(PCINT3_vect)
ISR(PCINT3_vect)
{
  sserial_handle_interrupt();
}
#endif

//
// Public methods
//

void sserial_begin(struct sserial *s, long speed)
{
  s->rx_delay_centering = s->rx_delay_intrabit = s->rx_delay_stopbit = s->tx_delay = 0;

  for (unsigned i=0; i<sizeof(table)/sizeof(table[0]); ++i)
  {
    long baud = pgm_read_dword(&table[i].baud);
    if (baud == speed)
    {
      s->rx_delay_centering = pgm_read_word(&table[i].rx_delay_centering);
      s->rx_delay_intrabit = pgm_read_word(&table[i].rx_delay_intrabit);
      s->rx_delay_stopbit = pgm_read_word(&table[i].rx_delay_stopbit);
      s->tx_delay = pgm_read_word(&table[i].tx_delay);
      break;
    }
  }

  // Set up RX interrupts, but only if we have a valid RX baud rate
  if (s->rx_delay_stopbit)
  {
    if (digitalPinToPCICR(s->receivePin))
    {
      *digitalPinToPCICR(s->receivePin) |= _BV(digitalPinToPCICRbit(s->receivePin));
      *digitalPinToPCMSK(s->receivePin) |= _BV(digitalPinToPCMSKbit(s->receivePin));
    }
    tunedDelay(s->tx_delay); // if we were low this establishes the end
  }

#if _DEBUG
  pinMode(_DEBUG_PIN1, OUTPUT);
  pinMode(_DEBUG_PIN2, OUTPUT);
#endif

  sserial_listen(s);
}

void sserial_end(struct sserial *s)
{
  if (digitalPinToPCMSK(s->receivePin))
    *digitalPinToPCMSK(s->receivePin) &= ~_BV(digitalPinToPCMSKbit(s->receivePin));
}


// Read data from buffer
int sserial_read(struct sserial *s)
{
  if (!sserial_isListening(s))
    return -1;

  // Empty buffer?
  if (receive_buffer_head == receive_buffer_tail)
    return -1;

  // Read from "head"
  uint8_t d = receive_buffer[receive_buffer_head]; // grab next byte
  receive_buffer_head = (receive_buffer_head + 1) % _SS_MAX_RX_BUFF;
  return d;
}

int sserial_available(struct sserial *s)
{
  if (!sserial_isListening(s))
    return 0;

  return (receive_buffer_tail + _SS_MAX_RX_BUFF - receive_buffer_head) % _SS_MAX_RX_BUFF;
}

size_t sserial_write(struct sserial *s, uint8_t b)
{
  if (s->tx_delay == 0) {
    s->write_error = 1;
    return 0;
  }

  uint8_t oldSREG = SREG;
  cli();  // turn off interrupts for a clean txmit

  // Write the start bit
  sserial_tx_pin_write(s, s->inverse_logic ? HIGH : LOW);
  tunedDelay(s->tx_delay + XMIT_START_ADJUSTMENT);

  // Write each of the 8 bits
  if (s->inverse_logic)
  {
    for (byte mask = 0x01; mask; mask <<= 1)
    {
      if (b & mask) // choose bit
        sserial_tx_pin_write(s, LOW); // send 1
      else
        sserial_tx_pin_write(s, HIGH); // send 0
    
      tunedDelay(s->tx_delay);
    }

    sserial_tx_pin_write(s, LOW); // restore pin to natural state
  }
  else
  {
    for (byte mask = 0x01; mask; mask <<= 1)
    {
      if (b & mask) // choose bit
        sserial_tx_pin_write(s, HIGH); // send 1
      else
        sserial_tx_pin_write(s, LOW); // send 0
    
      tunedDelay(s->tx_delay);
    }

    sserial_tx_pin_write(s, HIGH); // restore pin to natural state
  }

  SREG = oldSREG; // turn interrupts back on
  tunedDelay(s->tx_delay);
  
  return 1;
}

void sserial_flush(struct sserial *s)
{
  if (!sserial_isListening(s))
    return;

  uint8_t oldSREG = SREG;
  cli();
  receive_buffer_head = receive_buffer_tail = 0;
  SREG = oldSREG;
}

int sserial_peek(struct sserial *s)
{
  if (!sserial_isListening(s))
    return -1;

  // Empty buffer?
  if (receive_buffer_head == receive_buffer_tail)
    return -1;

  // Read from "head"
  return receive_buffer[receive_buffer_head];
}

//
// Constructor
//
void sserial_ctor(struct sserial *s, uint8_t receivePin, uint8_t transmitPin, bool inverse_logic /* = false */)
{
	s->rx_delay_centering = 0;
	s->rx_delay_intrabit = 0;
	s->rx_delay_stopbit = 0;
	s->tx_delay = 0;
	s->buffer_overflow = 0;
	s->write_error = 0;
	s->inverse_logic = inverse_logic;

	// Set TX
	pinMode(transmitPin, OUTPUT);
	digitalWrite(transmitPin, HIGH);
	s->transmitBitMask = digitalPinToBitMask(transmitPin);
	uint8_t port = digitalPinToPort(transmitPin);
	s->transmitPortRegister = portOutputRegister(port);
	
	// Set RX
	pinMode(receivePin, INPUT);
	if (!inverse_logic)
		digitalWrite(receivePin, HIGH);  // pullup for normal logic!
	s->receivePin = receivePin;
	s->receiveBitMask = digitalPinToBitMask(receivePin);
	port = digitalPinToPort(receivePin);
	s->receivePortRegister = portInputRegister(port);
}

//
// Destructor
//
void sserial_dtor(struct sserial *s)
{
	sserial_end(s);
}

