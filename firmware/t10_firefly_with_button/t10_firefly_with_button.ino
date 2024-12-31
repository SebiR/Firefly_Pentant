#ifdef F_CPU
  #undef F_CPU
  #define F_CPU 31250UL
#endif

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <util/delay.h>



#define LED_PIN PB0     // LED connected to PB0
#define BUTTON_PIN PB2  // Button connected to PB2


// uncomment if you want to manually calibrate the internal clock (WDT clock is independent)
//#define CLOCKCAL

volatile uint8_t button_pressed = 0;  // Flag to track button press

// Blink pattern. Each bit is checked with every watch-dog interrupt (currently every ~125ms) and the LED is set according to the bit status
// So 32bit are roughly 4 seconds of pattern
//uint32_t blink_pattern = 0b11001100110011001100110011001100;
//uint32_t blink_pattern = 0b01010101010101010101010101010101;
uint32_t blink_pattern = 0b10100100000000100100010000000000;

void setupWDT() {
  // Disable interrupts
  cli();

  // Reset the watchdog timer control register
  //CCP = 0xD8;                                                                                 // Enable confic changes
  WDTCSR = (1 << WDIE) | (0 << WDE) | (0 << WDP3) | (0 << WDP2) | (1 << WDP1) | (1 << WDP0);  // Enable WDT every ~125ms

  // Re-enable interrupts
  sei();
}

void setup() {
  // Configure PB0 as output for LED
  DDRB |= (1 << LED_PIN);
  PORTB &= ~(1 << LED_PIN);  // Start with LED off

  // Configure PB2 as input with internal pull-up for button
  DDRB &= ~(1 << BUTTON_PIN);  // Set PB2 as input
  PUEB |= (1 << BUTTON_PIN);   // Enable pull-up resistor on PB2

  // Enable pin change interrupt for PB2

  PCICR |= (1 << PCIE0);       // Enable pin change interrupt
  PCMSK |= (1 << BUTTON_PIN);  // Enable interrupt on PB2

  ADCSRA &= ~(1 << ADEN);  // Disable ADC

  CCP = 0xD8;                                                              // Unprotect CLKPSR reg
  CLKPSR = (1 << CLKPS3) | (0 << CLKPS2) | (0 << CLKPS1) | (0 << CLKPS0);  // Set clock to 31.25kHz
  //OSCCAL = 0x96;    // Adjust clock if you want to

// LED 100ms on and off, can be used to calibrate OSCCAL
#ifdef CLOCKCAL
  while (1) {
    PORTB |= (1 << LED_PIN);
    _delay_ms(100);
    PORTB &= ~(1 << LED_PIN);
    _delay_ms(100);
  }
#endif

  setupWDT();

  // Set up sleep mode to power down
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);

  // Enable global interrupts
  sei();
}

ISR(PCINT0_vect) {
  // Debounce the button
  _delay_ms(500);

  if (!(PINB & (1 << BUTTON_PIN))) {  // Check if button is still pressed
    button_pressed = 1;               // Set flag to toggle LED
  }
}

ISR(WDT_vect) {

  static uint8_t current_bit = 0;  // Track the current bit position

  // Extract the current bit from the pattern
  uint8_t bit_state = (blink_pattern >> (31 - current_bit)) & 0x01;

  // Set or clear the LED based on the bit value
  if (bit_state) {
    PORTB |= (1 << LED_PIN);  // Turn on the LED
  } else {
    PORTB &= ~(1 << LED_PIN);  // Turn off the LED
  }

  // Move to the next bit
  current_bit++;
  if (current_bit > 31) {
    current_bit = 0;  // Wrap around to the start of the pattern
  }
}



void loop() {
  // Check if button was pressed
  if (button_pressed) {
    button_pressed = 0;  // Clear the flag

    // Toggle WDT enable
    WDTCSR ^= (1 << WDIE);

    // LED off
    PORTB &= ~(1 << LED_PIN);

    // Add a small delay to debounce further presses
    _delay_ms(50);
  }

  // Enter sleep mode
  sleep_mode();

  // Execution resumes here after waking up
}
