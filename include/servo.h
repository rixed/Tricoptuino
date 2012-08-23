#ifndef SERVO_H_120816
#define SERVO_H_120816
#include <inttypes.h>
#include <stdbool.h>
#include "event.h"

struct servo {
    struct event event;
    uint16_t pulse_min, pulse_max;
    uint8_t angle;
    uint8_t phase:1;
    uint8_t pin:4;
};

void servo_ctor(struct servo *s, uint8_t pin, uint16_t pulse_min, uint16_t pulse_max);

static inline void servo_set(struct servo *s, uint8_t angle /* 0-255 -> 0-180 */)
{
    s->angle = angle;
}

void servo_init(void);

#endif
