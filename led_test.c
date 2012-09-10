// Example 1 : Blinking a LED
#include "arduino.h"

#define LED 13
#define BUTTON 7

int main(void)
{
	init();
	pinMode(LED, OUTPUT);
	for (;;) {
		digitalWrite(LED, HIGH);
		delay(2000);
		digitalWrite(LED, LOW);
		delay(2000);
	}

	return 0;
}

