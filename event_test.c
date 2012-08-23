/* Test at an event library using TIMER1 (16 bits) as a scheduler.
 */
#include <stdlib.h>
#include <arduino.h>
#include <stdint.h>
#include <limits.h>
#include "servo.h"
#include "assert.h"

#define SERVO 10

static struct servo servo;

void blink_led13(struct event *e)
{
    static uint8_t level = 1;
    digitalWrite(13, level);
    level = !level;
    event_register(e, US_TO_TIMER1_TICKS(20000U));
}

void blink_led12(struct event *e)
{
    static uint8_t on = 1;
    static uint8_t incr = 1;
    static uint16_t on_duration = 151U;
#   define LED_PERIOD 20000U /* 1ms */
    uint16_t const fade_speed = 4 * servo.angle;
    on = !on;
    digitalWrite(12, on);
    if (on) {
        if (incr) {
            if (on_duration > LED_PERIOD/4U) incr = 0;
            else on_duration += fade_speed;
        } else {
            if (on_duration <= fade_speed) incr = 1;
            else on_duration -= fade_speed;
        }
    }

    event_register(e, on ? on_duration : LED_PERIOD - on_duration);
}

// For our ESC 750ms is OK for arming, result in no rotation whatsoever.
#define PULSE_MIN US_TO_TIMER1_TICKS(750U)
#define PULSE_MAX US_TO_TIMER1_TICKS(2400U)

int main(void)
{
    init();
    event_init();
    pinMode(13, OUTPUT);
    pinMode(12, OUTPUT);
    struct event speed_led_event;
    event_ctor(&speed_led_event, blink_led12);
    struct event blink_led_event;
    event_ctor(&blink_led_event, blink_led12);

    servo_ctor(&servo, SERVO, PULSE_MIN, PULSE_MAX);
    delay(7500);
    //blink_led13();
    blink_led12(&speed_led_event);
            
    for (;;) {
        uint8_t s;
        for (s = 0; s < 50; s ++) {
            servo_set(&servo, s);
            delay(40);
        }
        for (; s > 0; s --) {
            servo_set(&servo, s);
            delay(30);
        }
        for (;;) {}
    }
}
