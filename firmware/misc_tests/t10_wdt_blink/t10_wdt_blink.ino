#include <avr/io.h>
#include <util/delay.h>
#include <avr/power.h>
#include <avr/sleep.h>

#ifndef F_CPU
#define F_CPU 1000000UL
#endif

#define LED PB0

ISR(WDT_vect) {
  PORTB ^= (1 << PB0);
}

// Setup the Watch Dog Timer (WDT)
void setupWDT() {
  // Disable interrupts
  cli();

  // Reset the watchdog timer control register
  //CCP = 0xD8;                                                                                 // Enable confic changes
  WDTCSR = (1 << WDIE) | (0 << WDE) | (0 << WDP3) | (1 << WDP2) | (0 << WDP1) | (1 << WDP0);  // Enable WDT

  // Re-enable interrupts
  sei();
}



// Enter Sleep Mode
void enterSleep(void) {
  cli();
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
  //set_sleep_mode(SLEEP_MODE_IDLE);
  //sleep_enable();
  sleep_cpu();

  // Nothing happens until WDT timeout
  sei();
  sleep_disable();
}

void setup() {

  // LED pin as output
  DDRB |= (1 << LED);

  setupWDT();
}

void loop() {
  enterSleep();
}