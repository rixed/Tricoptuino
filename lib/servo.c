#include "arduino.h"
#include "event.h"
#include "miscmacs.h"
#include "servo.h"

#define PULSE_PERIOD US_TO_TIMER1_TICKS(20000U)

static void servo_event(struct event *e)
{
    struct servo *s = DOWNCAST(e, event, servo);
    s->phase = !s->phase;
    digitalWrite(s->pin, s->phase);
    uint16_t const pulse_len = s->pulse_min + (((uint32_t)(s->pulse_max-s->pulse_min)*s->angle)>>8U);
    event_register(e, s->phase ? pulse_len : PULSE_PERIOD - pulse_len);
}

void servo_ctor(struct servo *s, uint8_t pin, uint16_t pulse_min, uint16_t pulse_max)
{
    event_ctor(&s->event, servo_event);
    s->angle = 0;
    s->phase = 0;
    s->pin = pin;
    s->pulse_min = pulse_min;
    s->pulse_max = pulse_max;
    pinMode(pin, OUTPUT);
    servo_event(&s->event);
}

extern inline void servo_set(struct servo *s, uint8_t angle /* 0-255 -> 0-180 */);

void servo_init(void)
{
    event_init();
}
