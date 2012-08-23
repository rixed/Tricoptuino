/* Same usage than assert from libc but instead of aborting (there is no abort)
   will send a message to serial console and make led 13 blink the error code. */
#ifndef ASSERT_H_120813
#define ASSERT_H_120813

#include <stdint.h>
#include "miscmacs.h"

#ifndef NDEBUG
void assert_fail(uint8_t code, char const *msg);
#   define assert(x) do { \
        if (! (x)) assert_fail(255, STRIZE(x)); \
    } while (0)
#   define assert2(x, code) do { \
        if (! (x)) assert_fail(code, STRIZE(x)); \
    } while (0)
#else
#   define assert(x)
#endif

#endif
