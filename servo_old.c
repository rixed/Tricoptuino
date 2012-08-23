#include <stdlib.h>
#include <arduino.h>
#include <stdint.h>
#include <limits.h>
#include "event.h"
#include "servo_old.h"
#include "assert.h"

#define SERVO 10

static void servo_event(void *servo_)
{
    struct servo *servo = servo_;

    static uint8_t phase = 0;
    phase = !phase;
    digitalWrite(SERVO, phase);
    // For our ESC 750ms is OK for arming, result in no rotation whatsoever.
#   define PULSE_MIN US_TO_TIMER1_TICKS(750U)
#   define PULSE_MAX US_TO_TIMER1_TICKS(2400U)
#   define PULSE_PERIOD US_TO_TIMER1_TICKS(20000U)
    uint16_t const pulse_len = PULSE_MIN + (((PULSE_MAX-PULSE_MIN)*speed)>>8U);
    event_register(phase ? pulse_len : PULSE_PERIOD - pulse_len, servo_slow);
}

int main(void)
{
    init();
    event_init();
    pinMode(13, OUTPUT);
    pinMode(12, OUTPUT);
    pinMode(SERVO, OUTPUT) ;                                   // set servo pin to output

    servo_slow();
    delay(7500);
    //blink_led13();
    blink_led12();

    for (;;) {
        for (; speed < 250; speed ++) {
            delay(100);
        }
        for (; speed > 0; speed --) {
            delay(100);
        }
        for (;;) {}
    }
}
