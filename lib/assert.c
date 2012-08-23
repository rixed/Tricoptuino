#include <stdio.h>
#include "arduino.h"
#include "assert.h"
#include "hard_serial.h"
#include "event.h"
#include "miscmacs.h"

#define ERR_LED 13

static uint8_t err_code = 0;

static void blink_err(struct event *e)
{
    static uint8_t bit = 11;   // for 10 to 0 (so that we have 2 bits pause between display)
    static uint8_t phase = 0;  // 0 -> pause, 1 -> flash, 2 -> bit
#   define OFF_DELAY US_TO_TIMER1_TICKS(30000UL)
#   define FLASH_DELAY US_TO_TIMER1_TICKS(20000UL)
#   define ON_DELAY US_TO_TIMER1_TICKS(500000UL)

    phase ++;
    if (phase == 4) {
        phase = 0;
        if (bit == 0) {
            bit = 11;
        } else {
            bit--;
        }
    }

    digitalWrite(ERR_LED, phase == 1 || (phase == 2 && bit < 8 && IS_BIT_SET(err_code, bit)));
    event_register(e, phase == 0 ? OFF_DELAY : phase == 1 ? FLASH_DELAY : ON_DELAY);
}

void assert_fail(uint8_t code, char const *msg)
{
    event_init();
    hserial_begin(&hserial, 9600UL);
    hserial_print(&hserial, "Assertion failed: ");
    hserial_print(&hserial, msg);
    hserial_print(&hserial, "\n");
    err_code = code;
    pinMode(ERR_LED, OUTPUT);
    static struct event event;
    event_ctor(&event, blink_err);
    blink_err(&event);
}

