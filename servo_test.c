#include <arduino.h>
#include <servo_old.h>

#define SERVO_PIN 10
#define LED1 13
#define LED2 12

#define ESC_MIN 20
#define ESC_MAX 40 /* 180 */
#define ESC_ARM_DELAY 7500

/*
 Servo signal goes like this:
 - 20ms period (servo.h says REFRESH_INTERVAL is 20ms)
 - high for about 1 to 2ms (when angle=180)
   (servo.h says the MIN_PULSE_WIDTH is 0.544 ms and MAX_PULSE_WIDTH is 2.4ms)
 So timer0, giving one int every 1.024ms by default, wont be frequent enough.
 Either change it's divider or use another one?

 On the BEC, ground is brown, +5v is red and signal is yellow.
*/

int main(void)
{
	init();
	pinMode(LED1, OUTPUT);
	pinMode(LED2, OUTPUT);
    uint8_t servo = servo_alloc();
    servo_attach(servo, SERVO_PIN, SERVO_DEFAULT_MIN, SERVO_DEFAULT_MAX);
	digitalWrite(LED1, LOW);
	digitalWrite(LED2, LOW);

    // Arm the motor controller
    servo_write(servo, ESC_MIN);
    delay(ESC_ARM_DELAY);
	digitalWrite(LED1, HIGH);

	for (;;) {
        // ramp up
        digitalWrite(LED2, HIGH);
        for (int speed = ESC_MIN; speed < ESC_MAX; speed ++) {
            servo_write(servo, speed);
            delay(100);
        }
        // ramp down
        digitalWrite(LED2, LOW);
        for (int speed = ESC_MAX; speed > ESC_MIN; speed --) {
            servo_write(servo, speed);
            delay(100);
        }
        // stop
	    digitalWrite(LED1, LOW);
        for (;;) {}
	}

	return 0;
}

