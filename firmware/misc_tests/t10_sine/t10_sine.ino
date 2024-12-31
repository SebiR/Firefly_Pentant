#include <avr/io.h>
#include <util/delay.h>
#include <avr/power.h>
#include <avr/wdt.h>
#include <avr/sleep.h>

#ifndef F_CPU
#define F_CPU 1000000UL
#endif

volatile byte sinepos = 0;
volatile byte dlypos = 0;

byte sine[] = { 1, 1, 2, 3, 5, 8, 11, 15, 20, 25, 30, 36, 43, 49, 56, 64, 72, 80, 88, 97, 105, 114, 123, 132, 141, 150, 158, 167, 175, 183, 191, 199, 206, 212, 219, 225, 230, 235, 240, 244, 247, 250, 252, 253, 254, 255, 254, 253, 252, 250, 247, 244, 240, 235, 230, 225, 219, 212, 206, 199, 191, 183, 175, 167, 158, 150, 141, 132, 123, 114, 105, 97, 88, 80, 72, 64, 56, 49, 43, 36, 30, 25, 20, 15, 11, 8, 5, 3, 2, 1, 0 };
byte rando[] = { 226, 25, 134, 174, 33, 177, 92, 2, 20, 43, 105, 91, 116, 164, 15, 185, 107, 144, 96, 52, 130, 176, 186, 69, 173, 110, 129, 150, 244, 79, 86, 73, 56, 131, 28, 235, 45, 136, 199, 247, 50, 124, 75, 145 };

byte rnd_dly[] = { 59, 34, 83, 114, 74, 62, 127, 50, 76, 29, 121, 63, 113 };
// Blank WDT interrupt
ISR(WDT_vect) {
}

// Setup the Watch Dog Timer (WDT)
void setupWDT() {
  wdt_reset();
  wdt_enable(WDTO_30MS);

  //wdt_reset();
  //wdt_enable(WDTO_30MS);
  //RSTFLR &= ~(1 << WDRF);  // Clear the WDRF (Reset Flag).

  // Setting WDCE allows updates for 4 clock cycles end is needed to
  // change WDE or the watchdog pre-scalers.
  //WDTCSR |= (1 << WDCE) | (1 << WDE);
  //WDTCSR |= (1 << WDE);

  // WD each 500ms
  // WDTCR  = (0<<WDP3) | (1<<WDP2) | (0<<WDP0) | (1<<WDP0);

  // WD each ~34ms
  //WDTCSR = (0 << WDP2) | (0 << WDP1) | (1 << WDP0);

  //WDTCSR |= _BV(WDIE);  // Enable the WDT interrupt.
}



// Enter Sleep Mode
void enterSleep(void) {
  //set_sleep_mode(SLEEP_MODE_PWR_DOWN);
  set_sleep_mode(SLEEP_MODE_IDLE);
  sleep_enable();
  sleep_mode();  // Start sleep mode

  // Nothing happens until WDT timeout

  sleep_disable();
  power_all_enable();
}

void setup() {
  DDRB = 1 << DDB0;

  // Timer0 as PWM
  TCCR0A = 1 << COM0A1 | 1 << WGM01 | 1 << WGM00;
  TCCR0B = 1 << CS00;

  //setupWDT();
  //sei();
}

void loop() {

  OCR0A = rando[sinepos];

  sinepos++;
  dlypos++;

  if (sinepos >= sizeof(rando)) {
    sinepos = 0;
  }

  if (dlypos >= sizeof(rnd_dly)) {
    dlypos = 0;
  }
  //_delay_ms(50);
  //_delay_ms(rnd_dly[dlypos]);

  for (byte i = 0; i <= rnd_dly[dlypos]; i++) {
    _delay_ms(1);
  }



  //enterSleep();
}